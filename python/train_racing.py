import argparse
import copy
import fcntl
import time
from dataclasses import dataclass
from itertools import count
from pathlib import Path

import apex_core as apex
import torch

from agent import observe_inputs
from evolution_driver import EvolutionDriver
from explore_circuit import PHYSICS_DT, TICKS_PER_ACTION, TICKS_PER_FRAME
from racing_observation import observe_opponents
from train_population import (
    Candidate,
    CONTACT_TIME_PENALTY,
    DEFAULT_CHECKPOINT,
    IMPACT_TIME_PENALTY,
    load_population,
    save_checkpoint,
)

RACING_CHECKPOINT = DEFAULT_CHECKPOINT.with_name("racing.pt")
RACING_MUTATION_SCALES = (0.001, 0.0025, 0.005, 0.01, 0.02, 0.05)


@dataclass
class RacingResult:
    lap: float | None
    elapsed: float
    progress: float
    lap_length: float
    contacts: int
    severity: float
    passes: int

    @property
    def penalty(self):
        return CONTACT_TIME_PENALTY * self.contacts + IMPACT_TIME_PENALTY * self.severity

    @property
    def score(self):
        if self.lap is not None:
            return 1, -round(self.elapsed + self.penalty, 6)
        return 0, self.progress / self.lap_length - self.penalty / 100.0


def run_heat(driver, opponent, settings, sensors, window, generation, label):
    title = f"Generation {generation} | {label}"
    # A shared lane puts the learner behind the rival for following and passing practice.
    grid = apex.TrainingGrid(settings, [2, 0])
    cars = [
        Candidate(model, settings, slot, race=grid.race(index))
        for index, (model, slot) in enumerate(((driver, 2), (opponent, 0)))
    ]
    starting_distances = [car.previous_distance for car in cars]
    controls = [apex.VehicleControls(), apex.VehicleControls()]
    tick = 0
    paused = False
    passes = 0
    previous_lead = starting_distances[0] - starting_distances[1]
    focused_slot = 0
    follow_leader = False

    def ranking(index):
        car = cars[index]
        if car.lap_time is not None:
            return 0, car.race.player.elapsed_time
        return 1, -(car.progress + starting_distances[index])

    while not cars[0].finished:
        started = time.perf_counter()
        events = window.poll_events(spectate=True)
        if events.close_requested:
            return None
        if events.pause_requested:
            paused = not paused
        if events.follow_leader_requested:
            follow_leader = True
        elif events.car_selection_delta:
            focused_slot = (focused_slot + events.car_selection_delta) % len(cars)
            follow_leader = False
        if not paused:
            for _ in range(TICKS_PER_FRAME):
                active = [not car.finished for car in cars]
                if tick % TICKS_PER_ACTION == 0:
                    for index, car in enumerate(cars):
                        if not active[index]:
                            continue
                        others = [
                            other.race.player.vehicle
                            for other_index, other in enumerate(cars)
                            if other_index != index and active[other_index]
                        ]
                        controls[index], *_ = car.driver.sample_action(
                            observe_inputs(car.race.player, settings, sensors),
                            greedy=True,
                            opponents=observe_opponents(car.race.player, others),
                        )
                grid.step(controls, active, PHYSICS_DT)
                for index, car in enumerate(cars):
                    if active[index]:
                        car.record_step(settings.track)
                lead = cars[0].progress + starting_distances[0] - cars[1].progress - starting_distances[1]
                if previous_lead <= 0.0 < lead and active[1]:
                    passes += 1
                previous_lead = lead
                tick += 1
                if cars[0].finished:
                    break

        contacts = grid.contacts(0)
        penalty = CONTACT_TIME_PENALTY * contacts.count + IMPACT_TIME_PENALTY * contacts.severity
        order = sorted(range(len(cars)), key=ranking)
        if follow_leader:
            focused_slot = order[0]
        window.draw_population(
            cars[focused_slot].race, sensors,
            [car.race.player.vehicle for index, car in enumerate(cars) if index != focused_slot],
            f"{title} | {'Paused | ' if paused else ''}"
            f"Contacts {contacts.count} | Penalty {penalty:.2f} s | Passes {passes}",
            focused_slot=focused_slot, focused_driver=("LEARNER", "RIVAL")[focused_slot],
            driver_names=["LEARNER", "RIVAL"],
            telemetry=(generation, sum(not car.finished for car in cars), max(car.progress for car in cars)),
            eliminated_slots=[index for index, car in enumerate(cars) if car.finished and car.lap_time is None],
            standings=[(index, cars[index].progress, cars[index].lap_time) for index in order],
        )
        time.sleep(max(0.0, TICKS_PER_FRAME * PHYSICS_DT - (time.perf_counter() - started)))

    learner = cars[0]
    contacts = grid.contacts(0)
    result = RacingResult(
        learner.lap_time, learner.race.player.elapsed_time,
        learner.progress, learner.lap_length,
        contacts.count, contacts.severity, passes,
    )
    print(
        f"{title} | "
        f"{'Finished' if result.lap is not None else 'Failed'} | "
        f"Time {result.elapsed:.3f} s | Distance {result.progress:.2f} | "
        f"Contacts {result.contacts} | Severity {result.severity:.3f} | "
        f"Penalty {result.penalty:.3f} s | Passes {result.passes}",
        flush=True,
    )
    return result


def load_racing(checkpoint, source, seed_slot):
    if not checkpoint.exists():
        source_generation, drivers, _ = load_population(source)
        if not 0 <= seed_slot < len(drivers):
            raise ValueError("Seed slot is outside the source population")
        print(f"Seeding two-car racing from slot {seed_slot + 1}, population generation {source_generation}", flush=True)
        return 1, copy.deepcopy(drivers[seed_slot]), copy.deepcopy(drivers[seed_slot])
    saved = torch.load(checkpoint, map_location="cpu", weights_only=True)
    if (saved["version"] != 1 or saved["mode"] != "two_car"
            or type(saved["next_generation"]) is not int or saved["next_generation"] < 1):
        raise ValueError("Incompatible racing checkpoint")
    champion = EvolutionDriver()
    champion.load_state_dict(saved["champion"])
    opponent = EvolutionDriver()
    opponent.load_state_dict(saved["opponent"])
    torch.set_rng_state(saved["rng_state"])
    print(f"Resuming two-car racing generation {saved['next_generation']}", flush=True)
    return saved["next_generation"], champion, opponent


@torch.no_grad()
def train(checkpoint, source, seed_slot):
    torch.manual_seed(42)
    generation, champion, opponent = load_racing(checkpoint, source, seed_slot)
    settings = apex.make_race_settings()
    sensors = apex.SensorSettings()
    sensors.max_distance = 6.0
    apex.set_application_name("Apex")
    window = apex.SimulationWindow(settings.track)
    icon = Path(__file__).resolve().parents[1] / "assets/icons/apex.png"
    apex.set_application_icon(str(icon))
    try:
        for generation in count(generation):
            baseline = run_heat(
                champion, opponent, settings, sensors, window, generation, "Champion"
            )
            if baseline is None:
                return
            challenger = copy.deepcopy(champion)
            scale = RACING_MUTATION_SCALES[(generation - 1) % len(RACING_MUTATION_SCALES)]
            for parameter in challenger.opponent_head.parameters():
                parameter.add_(torch.randn_like(parameter) * scale)
            trial = run_heat(
                challenger, opponent, settings, sensors, window, generation, f"Challenger {scale}"
            )
            if trial is None:
                return
            result = "Rejected"
            if trial.score >= baseline.score:
                champion = challenger
                result = "Improved" if trial.score > baseline.score else "Equal fitness"
            save_checkpoint(checkpoint, {
                "version": 1,
                "mode": "two_car",
                "next_generation": generation + 1,
                "champion": champion.state_dict(),
                "opponent": opponent.state_dict(),
                "rng_state": torch.get_rng_state(),
            })
            print(f"Racing generation {generation} | {result} | Saved {checkpoint}", flush=True)
    finally:
        window.close()


def main():
    parser = argparse.ArgumentParser(description="Visual two-car evolution with opponent observations and collision penalties")
    parser.add_argument("--checkpoint", type=Path, default=RACING_CHECKPOINT)
    parser.add_argument("--source", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--seed-slot", type=int, default=0, help="Zero-based slot of a successful source driver")
    args = parser.parse_args()
    checkpoint = args.checkpoint.expanduser().resolve()
    source = args.source.expanduser().resolve()
    if checkpoint == source:
        raise ValueError("Racing checkpoint must differ from its source population")
    checkpoint.parent.mkdir(parents=True, exist_ok=True)
    with checkpoint.with_suffix(checkpoint.suffix + ".lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit(f"Racing training is already using {checkpoint}") from None
        train(checkpoint, source, args.seed_slot)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
