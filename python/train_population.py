import argparse
import fcntl
import math
import os
import tempfile
import time
from collections import deque
from itertools import count
from pathlib import Path

import apex_core as apex
import torch

from agent import observe_inputs
from explore_circuit import (
    MINIMUM_PROGRESS,
    PHYSICS_DT,
    STALL_SECONDS,
    TICKS_PER_ACTION,
    TICKS_PER_FRAME,
)
from evolution_driver import (
    EvolutionDriver,
    STEERING_ADJUSTMENT_LIMIT,
    THROTTLE_ADJUSTMENT_LIMIT,
)
from sector_telemetry import SectorTelemetry
from racing_observation import observe_opponents

TEAM_DRIVERS = (
    ("Sacha", "Alex"),
    ("Aban", "Nathan"),
    ("Kim", "Jacob"),
    ("Saad", "Henry"),
    ("Quinn", "Cuh"),
    ("Aleck", "Chase"),
    ("Dev", "Fuad"),
    ("Aryan", "Rykc"),
    ("Matthew", "Ethan"),
    ("Jack", "Ali"),
    ("Connor", "Jerry"),
)
DRIVER_NAMES = tuple(
    f"{name} {slot:02d}"
    for slot, name in enumerate((name for team in TEAM_DRIVERS for name in team), start=1)
)
POPULATION_SIZE = len(DRIVER_NAMES)
TRAINING_FINISH_TARGET = POPULATION_SIZE
FINISH_WINDOW_SECONDS = 30
DISPLAY_NAMES = tuple(name.upper() for name in DRIVER_NAMES)
MUTATION_SCALES = (0.005, 0.01, 0.02, 0.05, 0.10)
STAGNATION_LIMIT = 20
EXPLORATION_SCALES = (0.02, 0.05, 0.10, 0.20, 0.30)
ELITE_MUTATION_SCALES = (0.001, 0.0025, 0.005, 0.01, 0.02, 0.05, 0.10)

MINIMUM_PACE_GAIN = PHYSICS_DT / 2
CHAMPION_PREVIEW_INTERVAL = 0.125
PARENT_COUNT = 4
EXPLORATION_DRIVERS = 2
RACING_MUTATION_SCALES = (0.001, 0.0025, 0.005, 0.01, 0.02)
DRIVING_REVISION = 2
VALIDATION_SLOTS = (0, 10, 20)
VALIDATION_INTERVAL = 5
DEFAULT_CHECKPOINT = Path(__file__).resolve().parents[1] / ".apex" / "population.pt"
SHARED_CHECKPOINT = DEFAULT_CHECKPOINT.with_name("shared_population.pt")
CHECKPOINT_VERSION = 3
RECAP_SECONDS = 8.0
CONTACT_TIME_PENALTY = 0.25
IMPACT_TIME_PENALTY = 0.50
MINIMUM_WINDOW_PROGRESS = 0.50


class Candidate:
    def __init__(self, driver, settings, slot, *, race=None):
        self.driver = driver
        self.race = apex.Race(settings, starting_slot=slot) if race is None else race
        self.previous_distance, self.lap_length = apex.track_progress(
            self.race.player.vehicle.body.position, settings.track
        )
        self.telemetry = SectorTelemetry(self.lap_length)
        self.start_distance = self.previous_distance
        self.progress = 0.0
        self.progress_marker = 0.0
        self.last_progress_time = 0.0
        self.progress_history = deque([(0.0, 0.0)])
        self.action = apex.VehicleControls()
        self.lap_time: float | None = None
        self.finished = False
        self.retirement_reason = None
        self.collision_count = 0
        self.collision_severity = 0.0

    @property
    def collision_penalty(self):
        return (CONTACT_TIME_PENALTY * self.collision_count
                + IMPACT_TIME_PENALTY * self.collision_severity)

    def step(self, tick, settings, sensors, track):
        if self.finished:
            return

        if tick % TICKS_PER_ACTION == 0:
            observation = observe_inputs(self.race.player, settings, sensors)
            self.action, *_ = self.driver.sample_action(observation, greedy=True)

        self.race.step(self.action, automatic=False, dt=PHYSICS_DT)
        self.record_step(track)

    def record_step(self, track):
        world = self.race.player
        position = world.vehicle.body.position

        status = self.race.status
        if not status.on_track or status.stuck_on_wall:
            self.retirement_reason = "off track" if not status.on_track else "stuck on wall"
            self.finished = True
            return

        self.telemetry.update(status, self.race.player_controls)

        distance, _ = apex.track_progress(position, track)
        self.progress += math.remainder(
            distance - self.previous_distance, self.lap_length
        )
        self.previous_distance = distance

        if self.progress >= self.progress_marker + MINIMUM_PROGRESS:
            self.progress_marker = self.progress
            self.last_progress_time = world.elapsed_time

        self.lap_time = status.last_lap
        self.progress_history.append((world.elapsed_time, self.progress))
        window_start = world.elapsed_time - STALL_SECONDS
        while len(self.progress_history) > 1 and self.progress_history[1][0] <= window_start:
            self.progress_history.popleft()
        start_time, start_progress = self.progress_history[0]
        window_progress = self.progress - start_progress
        stalled = world.elapsed_time - self.last_progress_time >= STALL_SECONDS
        creeping = (world.elapsed_time - start_time >= STALL_SECONDS
                    and window_progress < MINIMUM_WINDOW_PROGRESS)
        if self.lap_time is None and (stalled or creeping):
            self.retirement_reason = f"blocked or too slow ({window_progress:.3f} units in {STALL_SECONDS:g} s)"
        self.finished = stalled or creeping or self.lap_time is not None


def mutate(parent, scale, *, racing=False, adapt_road=False):
    child = EvolutionDriver()
    child.load_state_dict(parent.state_dict())

    with torch.no_grad():
        for parameter in child.control_head.parameters():
            mutation_mask = torch.rand_like(parameter) < 0.10
            noise = torch.randn_like(parameter) * scale
            parameter.add_(noise * mutation_mask)
        if racing:
            for parameter in child.opponent_head.parameters():
                mutation_mask = torch.rand_like(parameter) < 0.10
                parameter.add_(torch.randn_like(parameter) * scale * mutation_mask)
        if adapt_road:
            road_scale = min(0.02, scale * 0.25)
            for parameter in (*child.features.parameters(), *child.actor.parameters()):
                mutation_mask = torch.rand_like(parameter) < 0.05
                parameter.add_(torch.randn_like(parameter) * road_scale * mutation_mask)

    return child


def advance_population(cars, tick, settings, sensors, grid=None):
    active = [not car.finished for car in cars]
    if grid is None:
        for slot, car in enumerate(cars):
            if active[slot]:
                car.step(tick, settings, sensors, settings.track)
    else:
        if tick % TICKS_PER_ACTION == 0:
            worlds = [car.race.player for car in cars]
            for slot, car in enumerate(cars):
                if not active[slot]:
                    continue
                opponents = [
                    world.vehicle for other, world in enumerate(worlds)
                    if other != slot and active[other]
                ]
                car.action, *_ = car.driver.sample_action(
                    observe_inputs(worlds[slot], settings, sensors),
                    greedy=True, opponents=observe_opponents(worlds[slot], opponents),
                )
        grid.step([car.action for car in cars], active, PHYSICS_DT)
        for slot, car in enumerate(cars):
            contacts = grid.contacts(slot)
            car.collision_count = contacts.count
            car.collision_severity = contacts.severity
            if active[slot]:
                car.record_step(settings.track)
    for slot, car in enumerate(cars):
        if active[slot] and car.finished and car.lap_time is None:
            print(f"Retired {DRIVER_NAMES[slot]} | {car.retirement_reason} | "
                  f"Time {car.race.player.elapsed_time:.2f} s | Distance {car.progress:.2f}", flush=True)
    return [slot for slot, car in enumerate(cars) if active[slot] and car.lap_time is not None]


@torch.no_grad()
def improve_winner_pace(winner, winner_slot, settings, sensors, track, window, generation):
    if winner.lap_time is None:
        return winner, winner.driver

    best = winner
    breeding_parent = winner.driver

    for trial, scale in enumerate(ELITE_MUTATION_SCALES, start=1):
        challenger = Candidate(
            mutate(breeding_parent, scale), settings, winner_slot
        )
        best_finish = best.race.player.elapsed_time
        tick_limit = math.ceil(best_finish / PHYSICS_DT)
        title = (
            f"Generation {generation} | "
            f"Champion trial {trial}/{len(ELITE_MUTATION_SCALES)} | "
            f"Mutation {scale} | Target {best_finish:.2f} s"
        )
        next_preview = 0.0

        # Simulate every physics tick, but only update the preview a few times per second.
        for tick in range(tick_limit):
            if time.perf_counter() >= next_preview:
                if window.poll_events(spectate=True).close_requested:
                    return None
                window.draw_race(challenger.race, sensors, title=title)
                next_preview = time.perf_counter() + CHAMPION_PREVIEW_INTERVAL

            challenger.step(tick, settings, sensors, track)
            if challenger.finished:
                break

        finish = challenger.race.player.elapsed_time
        improved = (challenger.lap_time is not None and finish <= best_finish - MINIMUM_PACE_GAIN)

        if improved:
            best = challenger
            breeding_parent = challenger.driver
            result = f"New champion | Finish {finish:.3f} s"
        elif challenger.lap_time is not None:
            equal_finish = abs(finish - best_finish) < MINIMUM_PACE_GAIN
            equal_lap = abs(challenger.lap_time - best.lap_time) < MINIMUM_PACE_GAIN
            if equal_finish and equal_lap:
                breeding_parent = challenger.driver
                result = "Equal pace | Kept as mutation parent"
            else:
                result = f"Finished in {finish:.3f} s"
        elif challenger.finished:
            result = f"Failed at {challenger.progress:.2f} units"
        else:
            result = "Passed the champion's finish time"

        print(
            f"Champion trial {trial} | "
            f"Mutation {scale} | {result}",
            flush=True,
        )

    return best, breeding_parent


def fitness(car, *, racing=False):
    if car.lap_time is not None:
        if racing:
            return 1.0 + 1.0 / (car.race.player.elapsed_time + car.collision_penalty)
        return 1.0 + 1.0 / car.lap_time

    progress = min(1.0, max(0.0, car.progress / car.lap_length))
    return progress - car.collision_penalty / 100.0 if racing else progress


def load_population(path, *, mode=None):
    drivers = [EvolutionDriver() for _ in range(POPULATION_SIZE)]
    search_state = {
        "best_lap": None,
        "stagnant_generations": 0,
    }
    if not path.exists():
        return 1, drivers,search_state

    checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    if mode is not None and checkpoint.get("mode", "time_trial") != mode:
        raise ValueError(f"Checkpoint mode does not match {mode}: {path}")
    generation = checkpoint["next_generation"]
    states = checkpoint["drivers"]
    if (checkpoint["version"] not in (1, 2, CHECKPOINT_VERSION) or len(states) != len(drivers)
            or type(generation) is not int or generation < 1):
        raise ValueError(f"Incompatible population checkpoint: {path}")

    for driver, state in zip(drivers, states, strict=True):
        driver.load_evolution_state(state)
    torch.set_rng_state(checkpoint["rng_state"])
    print(f"Resuming generation {generation} from {path}", flush=True)
    print(
        f"Evolving control adjustments | "
        f"Throttle +/-{THROTTLE_ADJUSTMENT_LIMIT:.2f} | "
        f"Steering +/-{STEERING_ADJUSTMENT_LIMIT:.2f}",
        flush=True,
    )
    search_state.update(checkpoint.get("search_state", {}))
    return generation, drivers, search_state


def save_population(path, next_generation, drivers, search_state, *, mode="time_trial"):
    checkpoint = {
        "version": CHECKPOINT_VERSION,
        "mode": mode,
        "next_generation": next_generation,
        "drivers": [driver.state_dict() for driver in drivers],
        "rng_state": torch.get_rng_state(),
        "search_state": search_state
    }
    save_checkpoint(path, checkpoint)


def save_checkpoint(path, checkpoint):
    with tempfile.NamedTemporaryFile(dir=path.parent, suffix=".tmp", delete=False) as file:
        temporary = Path(file.name)
        try:
            torch.save(checkpoint, file)
            file.flush()
            os.fsync(file.fileno())
            file.close()
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)


def draw_broadcast(window, sensors, cars, generation, focused_slot, active_count, recap=None,
                   shared_race=False, paused=False, label=None):
    def ranking(slot):
        if recap is not None and slot == recap[0]:
            return -1, 0.0
        car = cars[slot]
        if car.lap_time is not None:
            return 0, car.race.player.elapsed_time
        return 1, -(car.progress + car.start_distance)

    order = sorted(range(len(cars)), key=ranking)
    best_distance = max(car.progress for car in cars)
    focused_driver = DISPLAY_NAMES[focused_slot]
    title = (f"Generation {generation} | {DRIVER_NAMES[focused_slot]} | "
             f"Running {active_count}/{len(cars)} | Best {best_distance:.2f}")
    if shared_race:
        focused = cars[focused_slot]
        title += f" | Shared race | Contacts {focused.collision_count} | Penalty {focused.collision_penalty:.2f} s"
    if paused:
        title += " | Paused"
    if label:
        title = f"{label} | {title}"
    window.draw_population(
        cars[focused_slot].race, sensors,
        [car.race.player.vehicle for slot, car in enumerate(cars) if slot != focused_slot],
        title, focused_slot=focused_slot, focused_driver=focused_driver, driver_names=DISPLAY_NAMES,
        telemetry=(generation, active_count, best_distance),
        eliminated_slots=[slot for slot, car in enumerate(cars) if car.finished and car.lap_time is None],
        standings=[(slot, cars[slot].progress, cars[slot].lap_time) for slot in order],
        recap=recap,
    )


def validation_score(results):
    """Prefer completed heats, then worst progress, then mean penalized finish time."""
    return (sum(result["completed"] for result in results),
            min(result["progress"] for result in results),
            -sum(result["time"] + result["penalty"] for result in results) / len(results))


def snapshot_driver(driver):
    return {name: value.detach().clone() for name, value in driver.state_dict().items()}


def validate_driver(driver, field, settings, sensors, window, generation, label):
    opponents = []
    for state in field:
        opponent = EvolutionDriver()
        opponent.load_evolution_state(state)
        opponents.append(opponent)
    results = []
    for starting_slot in VALIDATION_SLOTS:
        grid = apex.TrainingGrid(settings, list(range(len(field))))
        models = list(opponents)
        models[starting_slot] = driver
        cars = [Candidate(model, settings, slot, race=grid.race(slot))
                for slot, model in enumerate(models)]
        tick = 0
        paused = False
        next_preview = 0.0
        while not cars[starting_slot].finished:
            now = time.monotonic()
            if now >= next_preview:
                events = window.poll_events(spectate=True)
                if events.close_requested:
                    return None
                if events.pause_requested:
                    paused = not paused
                draw_broadcast(window, sensors, cars, generation, starting_slot,
                               sum(not car.finished for car in cars), shared_race=True,
                               paused=paused, label=f"Validation {label} | Grid {starting_slot + 1}")
                next_preview = now + 0.1
            if paused:
                time.sleep(0.01)
                continue
            advance_population(cars, tick, settings, sensors, grid)
            tick += 1
        car = cars[starting_slot]
        result = {"completed": car.lap_time is not None,
                  "progress": min(1.0, max(0.0, car.progress / car.lap_length)),
                  "time": car.race.player.elapsed_time, "penalty": car.collision_penalty}
        if result["completed"]:
            result["progress"] = 1.0
        results.append(result)
        print(f"Validation {label} | Grid {starting_slot + 1} | "
              f"Completed {result['completed']} | Progress {result['progress']:.4f} | "
              f"Time {result['time']:.3f} s | Penalty {result['penalty']:.3f} s", flush=True)
    return results


def validate_champion(challenger, drivers, search_state, settings, sensors, window, generation):
    rng_state = torch.get_rng_state()
    validation = search_state.get("validation")
    if validation is None:
        validation = {"field": [snapshot_driver(driver) for driver in drivers],
                      "champion": snapshot_driver(drivers[0]), "results": None}
        reference_path = SHARED_CHECKPOINT.with_name("shared_population_before_road_exploration.pt")
        if reference_path.is_file():
            reference = torch.load(reference_path, map_location="cpu", weights_only=True)
            validation["champion"] = reference["drivers"][0]
            print(f"Validating archived reference from {reference_path}", flush=True)
    champion = EvolutionDriver()
    champion.load_evolution_state(validation["champion"])
    # Model construction must not perturb the evolutionary mutation sequence.
    try:
        if validation["results"] is None:
            results = validate_driver(champion, validation["field"], settings, sensors,
                                      window, generation, "incumbent")
            if results is None:
                return None
            validation["results"] = results
        results = validate_driver(challenger, validation["field"], settings, sensors,
                                  window, generation, "challenger")
        if results is None:
            return None
        promoted = validation_score(results) > validation_score(validation["results"])
        if promoted:
            validation["champion"] = snapshot_driver(challenger)
            validation["results"] = results
            champion.load_evolution_state(validation["champion"])
        search_state["validation"] = validation
        print(f"Validated champion | Promoted {promoted} | "
              f"Score {validation_score(validation['results'])}", flush=True)
        return champion
    finally:
        torch.set_rng_state(rng_state)


def show_generation_recap(window, sensors, cars, generation, winner_slot, *, shared_race=False):
    focused_slot = winner_slot
    deadline = time.monotonic() + RECAP_SECONDS
    held_remaining = None
    while True:
        frame_started = time.monotonic()
        events = window.poll_events(spectate=True)
        if events.close_requested:
            return False
        if events.continue_requested:
            return True
        if events.pause_requested:
            if held_remaining is None:
                held_remaining = deadline - frame_started
            else:
                deadline = frame_started + held_remaining
                held_remaining = None
        if events.car_selection_delta:
            focused_slot = (focused_slot + events.car_selection_delta) % len(cars)

        remaining = deadline - frame_started if held_remaining is None else held_remaining
        if remaining <= 0:
            return True
        draw_broadcast(window, sensors, cars, generation, focused_slot, 0,
                       recap=(winner_slot, remaining, held_remaining is not None),
                       shared_race=shared_race)
        time.sleep(max(0.0, TICKS_PER_FRAME * PHYSICS_DT - (time.monotonic() - frame_started)))


@torch.no_grad()
def run_population(checkpoint, evaluate=False, shared_race=False, source=DEFAULT_CHECKPOINT):
    torch.manual_seed(42)
    settings = apex.make_race_settings()
    track = settings.track
    sensors = apex.SensorSettings()
    sensors.max_distance = 6.0
    checkpoint_mode = "shared_race" if shared_race else "time_trial"
    seeding = shared_race and not checkpoint.exists()
    if seeding and (source == checkpoint or not source.is_file()):
        raise ValueError("A new shared race requires an existing source population checkpoint")
    first_generation, drivers, search_state = load_population(
        source if seeding else checkpoint, mode=None if seeding else checkpoint_mode
    )
    if seeding:
        search_state = {"best_lap": None, "stagnant_generations": 0}
        print(f"Seeding all {len(drivers)} drivers into a shared race from {source}", flush=True)
    if shared_race and search_state.get("driving_revision") != DRIVING_REVISION:
        search_state = {"best_lap": None, "best_fitness": None,
                        "stagnant_generations": 0, "driving_revision": DRIVING_REVISION}
        print("Starting a new fitness baseline for bounded traffic controls", flush=True)
    icon_path = Path(__file__).resolve().parents[1] / "assets" / "icons" / "apex.png"

    if hasattr(apex, "set_application_name"):
        apex.set_application_name("Apex")

    window = apex.SimulationWindow(track)

    if hasattr(apex, "set_application_icon"):
        apex.set_application_icon(str(icon_path))

    selected_slot = None
    focused_slot = 0
    finish_window_ticks = math.ceil(FINISH_WINDOW_SECONDS / PHYSICS_DT)

    try:
        for generation in count(first_generation):
            validation = search_state.get("validation")
            validation_label = None
            if shared_race and validation:
                score = validation_score(validation["results"])
                validation_label = f"Champion benchmark {score[0]}/3 | Adjusted {-score[2]:.2f} s"
            grid = apex.TrainingGrid(settings, list(range(len(drivers)))) if shared_race else None
            cars = [
                Candidate(driver, settings, slot, race=grid.race(slot) if grid is not None else None)
                for slot, driver in enumerate(drivers)
            ]
            tick = 0
            finisher_slots = []
            finish_target = len(cars) if evaluate else TRAINING_FINISH_TARGET
            finish_deadline = None
            paused = False

            while True:
                frame_started = time.perf_counter()
                events = window.poll_events(spectate=True)
                if events.close_requested:
                    return
                if events.pause_requested:
                    paused = not paused
                if events.follow_leader_requested:
                    selected_slot = None
                elif events.car_selection_delta:
                    selected_slot = (focused_slot + events.car_selection_delta) % len(cars)

                if paused:
                    draw_broadcast(window, sensors, cars, generation, focused_slot,
                                   sum(not car.finished for car in cars), shared_race=shared_race,
                                   paused=True, label=validation_label)
                    time.sleep(max(0.0, TICKS_PER_FRAME * PHYSICS_DT - (time.perf_counter() - frame_started)))
                    continue

                for _ in range(TICKS_PER_FRAME):
                    finisher_slots.extend(advance_population(cars, tick, settings, sensors, grid))
                    tick += 1
                    if not evaluate and finisher_slots and finish_deadline is None:
                        finish_deadline = tick + finish_window_ticks
                    window_expired = finish_deadline is not None and tick >= finish_deadline
                    generation_complete = len(finisher_slots) >= finish_target or window_expired
                    if generation_complete:
                        break

                if generation_complete:
                    break
                active_slots = [slot for slot, car in enumerate(cars) if not car.finished]
                focused_slot = selected_slot
                if focused_slot is None:
                    focused_slot = max(active_slots or range(len(cars)),
                                       key=lambda slot: cars[slot].progress + cars[slot].start_distance)
                if not active_slots:
                    break
                draw_broadcast(window, sensors, cars, generation, focused_slot, len(active_slots),
                               shared_race=shared_race, label=validation_label)

                frame_budget = TICKS_PER_FRAME * PHYSICS_DT
                time.sleep(max(
                    0.0, frame_budget - (time.perf_counter() - frame_started)
                ))

            race_winner_slot = (finisher_slots[0] if finisher_slots
                                else max(range(len(cars)), key=lambda slot: cars[slot].progress))
            winner_slot = (max(range(len(cars)), key=lambda slot: fitness(cars[slot], racing=True))
                           if shared_race else race_winner_slot)
            winner = cars[winner_slot]
            if winner.lap_time is not None:
                sectors = winner.telemetry.times
                splits = " | ".join(f"S{index}: {seconds:.2f} s" for index, seconds in enumerate(sectors, start=1))
                print(f"{DRIVER_NAMES[winner_slot]} | {splits}", flush=True)
            finishers = len(finisher_slots)
            if evaluate:
                print(f"Finishers {finishers}/{len(cars)}", flush=True)

                for slot, car in enumerate(cars):
                    result = (
                        f"Lap {car.lap_time:.2f} s"
                        if car.lap_time is not None
                        else f"Unfinished | Distance {car.progress:.2f}"
                    )
                    print(f"{DRIVER_NAMES[slot]} | {result}", flush=True)

                print(f"Race winner {DRIVER_NAMES[race_winner_slot]}", flush=True)
                show_generation_recap(window, sensors, cars, generation, race_winner_slot,
                                      shared_race=shared_race)
                return
            eligible_parents = finisher_slots if finisher_slots else range(len(cars))
            other_slots = sorted(
                (slot for slot in eligible_parents if slot != winner_slot),
                key=lambda slot: (fitness(cars[slot], racing=shared_race), -cars[slot].race.player.elapsed_time),
                reverse=True,
            )
            parent_slots = [winner_slot, *other_slots[:PARENT_COUNT - 1]]
            if shared_race and race_winner_slot not in parent_slots:
                parent_slots = [winner_slot, race_winner_slot, *other_slots[:PARENT_COUNT - 2]]
            if shared_race:
                next_elite, breeding_parent = winner, winner.driver
                validation = search_state.get("validation")
                validation_due = validation is None or generation % VALIDATION_INTERVAL == 0
                if validation_due:
                    champion = validate_champion(winner.driver, drivers, search_state, settings,
                                                 sensors, window, generation)
                    if champion is None:
                        return
                else:
                    champion = EvolutionDriver()
                    champion.load_evolution_state(validation["champion"])
                breeding_parent = champion
            else:
                refinement = improve_winner_pace(
                    winner, winner_slot, settings, sensors, track, window, generation
                )
                if refinement is None:
                    return
                next_elite, breeding_parent = refinement
            if next_elite is not winner:
                print(
                    f"Champion improved | "
                    f"Finish {winner.race.player.elapsed_time:.2f} s "
                    f"to {next_elite.race.player.elapsed_time:.2f} s",
                    flush=True,
                )

            if shared_race:
                score = fitness(winner, racing=True)
                previous_best = search_state.get("best_fitness")
                if previous_best is None or score > previous_best + 1e-9:
                    search_state["best_fitness"] = score
                    search_state["stagnant_generations"] = 0
                else:
                    search_state["stagnant_generations"] += 1
                if winner.lap_time is not None:
                    best_lap = search_state["best_lap"]
                    search_state["best_lap"] = winner.lap_time if best_lap is None else min(best_lap, winner.lap_time)
            elif next_elite.lap_time is not None:
                best_lap = search_state["best_lap"]
                improved = (
                        best_lap is None
                        or next_elite.lap_time <= best_lap - MINIMUM_PACE_GAIN
                )

                if improved:
                    search_state["best_lap"] = next_elite.lap_time
                    search_state["stagnant_generations"] = 0
                else:
                    search_state["stagnant_generations"] += 1

            exploring = (
                    search_state["stagnant_generations"] >= STAGNATION_LIMIT
            )

            mode = "Exploration" if exploring else "Local search"
            best_lap = search_state["best_lap"]
            best_text = f"{best_lap:.3f} s" if best_lap is not None else "Pending"

            print(
                f"Breeding generation {generation + 1} | "
                f"Mode {mode} | "
                f"Best lap {best_text} | "
                f"Stagnant {search_state['stagnant_generations']}",
                flush=True,
            )
            drivers = []
            child_index = 0
            road_explorers = 0
            champion_slot = (next(slot for slot in reversed(range(len(cars))) if slot not in parent_slots)
                             if shared_race else None)
            mutated_count = len(cars) - len(parent_slots) - EXPLORATION_DRIVERS - int(shared_race)

            for slot in range(len(cars)):
                if slot == champion_slot:
                    drivers.append(champion)
                    continue
                if slot in parent_slots:
                    drivers.append(next_elite.driver if slot == winner_slot else cars[slot].driver)
                    continue

                if child_index < mutated_count:
                    parent_index = (child_index % len(parent_slots) if shared_race
                                    else torch.randint(len(parent_slots), ()).item())
                    parent_slot = parent_slots[parent_index]
                    if parent_slot == winner_slot:
                        parent = breeding_parent
                    else:
                        parent = cars[parent_slot].driver
                    scales = RACING_MUTATION_SCALES if shared_race else MUTATION_SCALES

                    if exploring and child_index % 2 == 1 and not shared_race:
                        scales = EXPLORATION_SCALES

                    scale = scales[child_index % len(scales)]
                    adapt_road = shared_race and exploring and child_index % 3 == 0
                    if adapt_road:
                        scale = EXPLORATION_SCALES[child_index % len(EXPLORATION_SCALES)]
                    drivers.append(mutate(parent, scale, racing=shared_race, adapt_road=adapt_road))
                else:
                    parent_slot = parent_slots[(child_index - mutated_count) % len(parent_slots)]
                    parent = breeding_parent if parent_slot == winner_slot else cars[parent_slot].driver
                    adapt_road = shared_race and exploring
                    drivers.append(mutate(parent, EXPLORATION_SCALES[-1], racing=shared_race,
                                          adapt_road=adapt_road))

                road_explorers += int(adapt_road)
                child_index += 1
            if shared_race:
                print(f"Road policy explorers {road_explorers}/{len(drivers)} | "
                      f"Preserved parents {len(parent_slots)}", flush=True)
            save_population(checkpoint, generation + 1, drivers, search_state, mode=checkpoint_mode)
            if next_elite.lap_time is not None:
                telemetry_directory = "shared_telemetry" if shared_race else "telemetry"
                telemetry_path = checkpoint.parent / telemetry_directory / f"generation_{generation:06d}.csv"
                next_elite.telemetry.save_csv(telemetry_path)

                print(
                    f"Saved champion telemetry to {telemetry_path} | "
                    f"Lap {next_elite.lap_time:.3f} s",
                    flush=True,
                )
            cut_off = sum(not car.finished for car in cars)
            cutoff_result = f" | Cut off {cut_off}" if cut_off else ""
            if cut_off and window_expired and finishers < finish_target:
                cutoff_result += f" ({FINISH_WINDOW_SECONDS}s finish window)"
            result = f"Lap {winner.lap_time:.2f} s" if winner.lap_time is not None else f"Distance {winner.progress:.2f}"
            winner_label = "Selected parent" if shared_race else "Winner"
            print(
                f"Generation {generation} | "
                f"Finishers {finishers}/{len(cars)} | "
                f"{winner_label} {DRIVER_NAMES[winner_slot]} | "
                f"{result} | "
                f"Finish time {winner.race.player.elapsed_time:.2f} s"
                f"{cutoff_result}",
                flush=True,
            )
            if shared_race:
                print(
                    f"Shared race {generation} | First finisher "
                    f"{DRIVER_NAMES[race_winner_slot] if finisher_slots else 'None'} | "
                    f"Selected parent {DRIVER_NAMES[winner_slot]} | "
                    f"Penalty {winner.collision_penalty:.3f} s | "
                    f"Contacts {sum(car.collision_count for car in cars) // 2}",
                    flush=True,
                )
                for slot, car in enumerate(cars):
                    print(
                        f"  {DRIVER_NAMES[slot]} | "
                        f"{'Finished' if car.lap_time is not None else 'Unfinished'} | "
                        f"Distance {car.progress:.2f} | Contacts {car.collision_count} | "
                        f"Penalty {car.collision_penalty:.3f} s | Fitness {fitness(car, racing=True):.6f}",
                        flush=True,
                    )
            if not show_generation_recap(window, sensors, cars, generation, race_winner_slot,
                                         shared_race=shared_race):
                return

    finally:
        window.close()


def main(checkpoint=DEFAULT_CHECKPOINT, evaluate=False, shared_race=False, source=DEFAULT_CHECKPOINT):
    checkpoint = checkpoint.expanduser().resolve()
    checkpoint.parent.mkdir(parents=True, exist_ok=True)
    with checkpoint.with_suffix(checkpoint.suffix + ".lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit(f"Training is already using {checkpoint}") from None
        run_population(checkpoint, evaluate=evaluate, shared_race=shared_race,
                       source=source.expanduser().resolve())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evolve and resume the Apex driver population.")
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--shared-race", action="store_true")
    parser.add_argument("--source", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--evaluate", action="store_true")
    args = parser.parse_args()
    try:
        checkpoint = args.checkpoint or (SHARED_CHECKPOINT if args.shared_race else DEFAULT_CHECKPOINT)
        main(checkpoint, evaluate=args.evaluate, shared_race=args.shared_race, source=args.source)
    except KeyboardInterrupt:
        pass
