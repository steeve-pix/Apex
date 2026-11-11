import argparse
import fcntl
import math
import os
import tempfile
import time
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
from rl_driver import RLDriver

TEAM_DRIVERS = (
    ("Sacha", "Kim"),
    ("Aban", "Nathan"),
    ("Alex", "D"),
    ("Aban", "James"),
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
DISPLAY_NAMES = tuple(name.upper() for name in DRIVER_NAMES)
MUTATION_SCALES = (0.05, 0.05, 0.05, 0.10, 0.20)
DEFAULT_CHECKPOINT = Path(__file__).resolve().parents[1] / ".apex" / "population.pt"
CHECKPOINT_VERSION = 1
RECAP_SECONDS = 8.0


class Candidate:
    def __init__(self, driver, settings, slot):
        self.driver = driver
        self.race = apex.Race(settings, starting_slot=slot)
        self.previous_distance, self.lap_length = apex.track_progress(
            self.race.player.vehicle.body.position, settings.track
        )
        self.progress = 0.0
        self.progress_marker = 0.0
        self.last_progress_time = 0.0
        self.action = apex.VehicleControls()
        self.lap_time = None
        self.finished = False

    def step(self, tick, settings, sensors, track):
        if self.finished:
            return

        if tick % TICKS_PER_ACTION == 0:
            observation = observe_inputs(self.race.player, settings, sensors)
            self.action, *_ = self.driver.sample_action(observation, greedy=True)

        self.race.step(self.action, automatic=False, dt=PHYSICS_DT)
        world = self.race.player
        position = world.vehicle.body.position

        if not apex.is_on_track(position, track):
            self.finished = True
            return

        distance, _ = apex.track_progress(position, track)
        self.progress += math.remainder(
            distance - self.previous_distance, self.lap_length
        )
        self.previous_distance = distance

        if self.progress >= self.progress_marker + MINIMUM_PROGRESS:
            self.progress_marker = self.progress
            self.last_progress_time = world.elapsed_time

        stalled = world.elapsed_time - self.last_progress_time >= STALL_SECONDS
        self.lap_time = self.race.status.last_lap
        self.finished = stalled or self.lap_time is not None


def mutate(parent, scale):
    child = RLDriver()
    child.load_state_dict(parent.state_dict())

    with torch.no_grad():
        for layer in (child.features, child.actor):
            for parameter in layer.parameters():
                parameter.add_(torch.randn_like(parameter) * scale)

    return child


def fitness(car):
    if car.lap_time is not None:
        return 1.0 + 1.0 / car.lap_time

    return min(1.0, max(0.0, car.progress / car.lap_length))


def load_population(path):
    drivers = [RLDriver() for _ in range(POPULATION_SIZE)]
    if not path.exists():
        return 1, drivers

    checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    generation = checkpoint["next_generation"]
    states = checkpoint["drivers"]
    if (checkpoint["version"] != CHECKPOINT_VERSION or len(states) != len(drivers)
            or type(generation) is not int or generation < 1):
        raise ValueError(f"Incompatible population checkpoint: {path}")

    for driver, state in zip(drivers, states, strict=True):
        driver.load_state_dict(state)
    torch.set_rng_state(checkpoint["rng_state"])
    print(f"Resuming generation {generation} from {path}", flush=True)
    return generation, drivers


def save_population(path, next_generation, drivers):
    checkpoint = {
        "version": CHECKPOINT_VERSION,
        "next_generation": next_generation,
        "drivers": [driver.state_dict() for driver in drivers],
        "rng_state": torch.get_rng_state(),
    }
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


def draw_broadcast(window, sensors, cars, generation, focused_slot, active_count, recap=None):
    def ranking(slot):
        if recap is not None and slot == recap[0]:
            return (-1, 0.0)
        car = cars[slot]
        if car.lap_time is not None:
            return (0, car.race.player.elapsed_time)
        return (1, -car.progress)

    order = sorted(range(len(cars)), key=ranking)
    best_distance = max(car.progress for car in cars)
    focused_driver = DISPLAY_NAMES[focused_slot]
    title = (f"Generation {generation} | {DRIVER_NAMES[focused_slot]} | "
             f"Running {active_count}/{len(cars)} | Best {best_distance:.2f}")
    window.draw_population(
        cars[focused_slot].race, sensors,
        [car.race.player.vehicle for slot, car in enumerate(cars) if slot != focused_slot],
        title, focused_slot=focused_slot, focused_driver=focused_driver, driver_names=DISPLAY_NAMES,
        telemetry=(generation, active_count, best_distance),
        eliminated_slots=[slot for slot, car in enumerate(cars) if car.finished and car.lap_time is None],
        standings=[(slot, cars[slot].progress, cars[slot].lap_time) for slot in order],
        recap=recap,
    )


def show_generation_recap(window, sensors, cars, generation, winner_slot):
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
                       recap=(winner_slot, remaining, held_remaining is not None))
        time.sleep(max(0.0, TICKS_PER_FRAME * PHYSICS_DT - (time.monotonic() - frame_started)))


@torch.no_grad()
def run_population(checkpoint,evaluate=False):
    torch.manual_seed(42)
    settings = apex.make_race_settings()
    track = settings.track
    sensors = apex.SensorSettings()
    sensors.max_distance = 6.0
    first_generation, drivers = load_population(checkpoint)
    window = apex.SimulationWindow(track)
    selected_slot = None
    focused_slot = 0

    try:
        for generation in count(first_generation):
            cars = [
                Candidate(driver, settings, slot)
                for slot, driver in enumerate(drivers)
            ]
            tick = 0
            first_finisher_slot = None

            while True:
                frame_started = time.perf_counter()
                events = window.poll_events(spectate=True)
                if events.close_requested:
                    return
                if events.follow_leader_requested:
                    selected_slot = None
                elif events.car_selection_delta:
                    selected_slot = (focused_slot + events.car_selection_delta) % len(cars)

                for _ in range(TICKS_PER_FRAME):
                    for slot, car in enumerate(cars):
                        car.step(tick, settings, sensors, track)
                        if first_finisher_slot is None and car.lap_time is not None:
                            first_finisher_slot = slot
                    tick += 1

                active_slots = [slot for slot, car in enumerate(cars) if not car.finished]
                focused_slot = selected_slot
                if focused_slot is None:
                    focused_slot = max(active_slots or range(len(cars)), key=lambda slot: cars[slot].progress)
                if not active_slots:
                    break
                draw_broadcast(window, sensors, cars, generation, focused_slot, len(active_slots))

                frame_budget = TICKS_PER_FRAME * PHYSICS_DT
                time.sleep(max(
                    0.0, frame_budget - (time.perf_counter() - frame_started)
                ))

            winner_slot = first_finisher_slot
            if winner_slot is None:
                winner_slot = max(range(len(cars)), key=lambda slot: fitness(cars[slot]))
            winner = cars[winner_slot]
            if evaluate:
                finishers = sum(car.lap_time is not None for car in cars)
                print(f"Finishers {finishers}/{len(cars)}", flush=True)

                for slot, car in enumerate(cars):
                    result = (
                        f"Lap {car.lap_time:.2f} s"
                        if car.lap_time is not None
                        else f"Unfinished | Distance {car.progress:.2f}"
                    )
                    print(f"{DRIVER_NAMES[slot]} | {result}", flush=True)

                print(f"Race winner {DRIVER_NAMES[winner_slot]}", flush=True)
                show_generation_recap(window, sensors, cars, generation, winner_slot)
                return
            drivers = []
            child_index = 0
            for slot in range(len(cars)):
                if slot == winner_slot:
                    drivers.append(winner.driver)
                    continue

                scale = MUTATION_SCALES[child_index % len(MUTATION_SCALES)]
                drivers.append(mutate(winner.driver, scale))
                child_index += 1
            save_population(checkpoint, generation + 1, drivers)
            print(
                f"Generation {generation} | "
                f"Winner {DRIVER_NAMES[winner_slot]} | "
                f"Distance {winner.progress:.2f}",
                flush=True,
            )
            if not show_generation_recap(window, sensors, cars, generation, winner_slot):
                return

    finally:
        window.close()


def main(checkpoint=DEFAULT_CHECKPOINT,evaluate=False):
    checkpoint = checkpoint.expanduser().resolve()
    checkpoint.parent.mkdir(parents=True, exist_ok=True)
    with checkpoint.with_suffix(checkpoint.suffix + ".lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit(f"Training is already using {checkpoint}") from None
        run_population(checkpoint,evaluate=evaluate)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evolve and resume the Apex driver population.")
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--evaluate", action="store_true")
    args = parser.parse_args()
    try:
        main(args.checkpoint, evaluate=args.evaluate)
    except KeyboardInterrupt:
        pass
