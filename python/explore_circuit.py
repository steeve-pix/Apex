import argparse
import math
import time
from collections import deque
from itertools import count
from pathlib import Path
from statistics import fmean

import apex_core as apex
import torch

from agent import observe_inputs
from rl_driver import RLDriver

PHYSICS_DT = 1 / 120
TICKS_PER_FRAME = 2
TICKS_PER_ACTION = 12
STALL_SECONDS = 5.0
MINIMUM_PROGRESS = 0.02
FAILURE_PENALTY = 1.0


def main(*, evaluate: bool = False) -> None:
    settings = apex.make_race_settings()
    track = settings.track
    sensors = apex.SensorSettings()
    torch.manual_seed(42)
    driver = RLDriver()
    checkpoint_path = Path("build/rl_drive.pt")
    if evaluate and not checkpoint_path.is_file():
        raise SystemExit(f"Train and save first. Missing {checkpoint_path.resolve()}")

    recent_distances = deque(maxlen=20)
    best_distance = 0.0

    if checkpoint_path.exists():
        checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
        driver.load_state_dict(checkpoint["model"])
        driver.optimizer.load_state_dict(checkpoint["optimizer"])
        print(f"Resumed training from {checkpoint_path}")
    window = apex.SimulationWindow(track)
    try:
        attempts = (1,) if evaluate else count(1)
        for attempt in attempts:
            race = apex.Race(settings)
            action = apex.VehicleControls()
            previous_distance, lap_length = apex.track_progress(race.player.vehicle.body.position, track)

            progress = 0.0
            progress_marker = 0.0
            last_progress_time = 0.0
            ticks = 0
            failure = None

            while failure is None:
                frame_started = time.perf_counter()

                if window.poll_events().close_requested:
                    return

                for _ in range(TICKS_PER_FRAME):
                    if ticks % TICKS_PER_ACTION == 0:
                        observation = observe_inputs(race.player, settings, sensors)
                        with torch.set_grad_enabled(not evaluate):
                            action, log_prob, value, entropy = driver.sample_action(observation, greedy=evaluate)
                        action_start_progress = progress

                    race.step(action, automatic=False, dt=PHYSICS_DT)
                    ticks += 1

                    world = race.player
                    position = world.vehicle.body.position

                    if not apex.is_on_track(position, track):
                        failure = "left the track"
                        break

                    distance, _ = apex.track_progress(position, track)
                    progress += math.remainder(distance - previous_distance, lap_length)
                    previous_distance = distance

                    if progress >= progress_marker + MINIMUM_PROGRESS:
                        progress_marker = progress
                        last_progress_time = world.elapsed_time

                    if world.elapsed_time - last_progress_time >= STALL_SECONDS:
                        failure = "stalled"
                        break

                if not evaluate and (ticks % TICKS_PER_ACTION == 0 or failure is not None):
                    step_reward = progress - action_start_progress
                    next_observation = None

                    if failure is None:
                        next_observation = observe_inputs(race.player, settings, sensors)
                    else:
                        step_reward -= FAILURE_PENALTY

                    driver.learn(log_prob, value, entropy, step_reward, next_observation)

                best_distance = max(best_distance, progress)
                recent_average = (
                    f"{fmean(recent_distances):.1f}" if recent_distances else "pending"
                )
                title = (
                    f"{'RL Evaluation' if evaluate else 'RL Training'} | Attempt {attempt} | "
                    f"Distance {progress:.1f} | Best {best_distance:.1f} | "
                    f"Recent avg {recent_average}"
                )

                window.draw_race(race, sensors, title=title)
                frame_budget = TICKS_PER_FRAME * PHYSICS_DT
                time.sleep(max(0.0, frame_budget - (time.perf_counter() - frame_started)))

            recent_distances.append(progress)
            reward = progress - FAILURE_PENALTY
            print(
                f"Attempt {attempt}: {failure} | "
                f"progress {progress:+.2f} | reward {reward:+.2f}",
                flush=True,
            )
    finally:
        window.close()

        if not evaluate:
            checkpoint_path.parent.mkdir(parents=True, exist_ok=True)
            temporary_path = checkpoint_path.with_suffix(".tmp")
            torch.save({
                "model": driver.state_dict(),
                "optimizer": driver.optimizer.state_dict(),
            }, temporary_path)
            temporary_path.replace(checkpoint_path)
            print(f"Saved training to {checkpoint_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--evaluate", action="store_true")
    args = parser.parse_args()
    main(evaluate=args.evaluate)
