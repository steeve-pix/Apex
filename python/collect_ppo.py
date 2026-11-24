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
from ppo_rollout import RolloutBuffer, Transition
from ppo_update import update_ppo
from rl_driver import RLDriver

ROLLOUT_SIZE = 256
TIME_PENALTY = 0.001
FAILURE_PENALTY = 1.0
LAP_BONUS = 1.0


def save_ppo(path, driver, next_batch, slot):
    checkpoint = {
        "model": driver.state_dict(),
        "optimizer": driver.optimizer.state_dict(),
        "rng_state": torch.get_rng_state(),
        "next_batch": next_batch,
        "slot": slot,
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


def main():
    project_root = Path(__file__).resolve().parents[1]

    parser = argparse.ArgumentParser()
    parser.add_argument("--population", type=Path)
    parser.add_argument("--slot", type=int, choices=range(22))
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=project_root / ".apex" / "ppo.pt",
    )
    args = parser.parse_args()

    checkpoint_path = args.checkpoint.expanduser().resolve()
    checkpoint_path.parent.mkdir(parents=True, exist_ok=True)

    lock_path = checkpoint_path.with_suffix(".pt.lock")
    with lock_path.open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit(
                f"Another trainer is using {checkpoint_path}"
            ) from None

        run_training(args, checkpoint_path)


def run_training(args, checkpoint_path):
    torch.manual_seed(42)
    driver = RLDriver()
    batch = 1

    if checkpoint_path.is_file():
        checkpoint = torch.load(
            checkpoint_path, map_location="cpu", weights_only=True
        )

        saved_slot = checkpoint["slot"]
        if args.slot is not None and args.slot != saved_slot:
            raise ValueError(
                f"Checkpoint uses slot {saved_slot}. "
                "Use a different checkpoint path to change slots."
            )

        args.slot = saved_slot
        driver.load_state_dict(checkpoint["model"])
        driver.optimizer.load_state_dict(checkpoint["optimizer"])
        torch.set_rng_state(checkpoint["rng_state"])
        batch = checkpoint["next_batch"]
        print(f"Resuming PPO at batch {batch}", flush=True)
    else:
        args.slot = 0 if args.slot is None else args.slot

        if args.population is not None:
            checkpoint = torch.load(args.population.expanduser().resolve(), map_location="cpu", weights_only=True)
            driver.load_state_dict(checkpoint["drivers"][args.slot])

    settings = apex.make_race_settings()
    sensors = apex.SensorSettings()
    sensors.max_distance = 6.0
    buffer = RolloutBuffer()
    batch = 1

    icon_path = (
            Path(__file__).resolve().parents[1] / "assets/icons/apex.png"
    )
    if hasattr(apex, "set_application_name"):
        apex.set_application_name("Apex")

    window = apex.SimulationWindow(settings.track)

    try:
        if hasattr(apex, "set_application_icon"):
            apex.set_application_icon(str(icon_path))

        for episode in count(1):
            race = apex.Race(settings, starting_slot=args.slot)
            previous_distance, lap_length = apex.track_progress(race.player.vehicle.body.position, settings.track)

            progress = 0.0
            progress_marker = 0.0
            last_progress_time = 0.0
            episode_reward = 0.0

            while True:
                observation = observe_inputs(race.player, settings, sensors)
                with torch.no_grad():
                    action, log_prob, value, _, action_index = (driver.sample_action(observation, return_index=True))

                start_progress = progress
                start_time = race.player.elapsed_time
                outcome = None

                for tick in range(TICKS_PER_ACTION):
                    if tick % TICKS_PER_FRAME == 0:
                        frame_started = time.perf_counter()
                        if window.poll_events(spectate=True).close_requested:
                            return

                    race.step(action, automatic=False, dt=PHYSICS_DT)
                    world = race.player
                    status = race.status

                    if not status.on_track:
                        outcome = "left track"
                    elif status.stuck_on_wall:
                        outcome = "stuck on wall"
                    else:
                        distance, _ = apex.track_progress(world.vehicle.body.position, settings.track)
                        progress += math.remainder(distance - previous_distance, lap_length)
                        previous_distance = distance

                        if progress >= progress_marker + MINIMUM_PROGRESS:
                            progress_marker = progress
                            last_progress_time = world.elapsed_time

                        if status.last_lap is not None:
                            outcome = "lap completed"
                        elif world.elapsed_time - last_progress_time >= STALL_SECONDS:
                            outcome = "stalled"

                    frame_ticks = tick % TICKS_PER_FRAME + 1
                    if frame_ticks == TICKS_PER_FRAME or outcome is not None:
                        window.draw_race(
                            race,
                            sensors,
                            title=(
                                f"PPO training | Episode {episode} | "
                                f"Batch {batch} | "
                                f"Samples {len(buffer)}/{ROLLOUT_SIZE}"
                            ),
                        )
                        time.sleep(max(0.0, frame_ticks * PHYSICS_DT - (time.perf_counter() - frame_started), ))

                    if outcome is not None:
                        break

                terminated = outcome is not None
                elapsed = race.player.elapsed_time - start_time
                reward = (
                        (progress - start_progress) / lap_length
                        - TIME_PENALTY * elapsed
                )

                if outcome == "lap completed":
                    reward += LAP_BONUS
                elif terminated:
                    reward -= FAILURE_PENALTY

                next_value = 0.0
                if not terminated:
                    next_observation = observe_inputs(
                        race.player, settings, sensors
                    )
                    with torch.no_grad():
                        _, estimate = driver(next_observation)
                        next_value = estimate.item()

                buffer.add(Transition(
                    observation=observation,
                    action_index=action_index,
                    old_log_prob=log_prob.item(),
                    value=value.item(),
                    reward=reward,
                    terminated=terminated,
                    next_value=next_value,
                ))
                episode_reward += reward

                if len(buffer) == ROLLOUT_SIZE:
                    metrics = update_ppo(driver, buffer)
                    print(
                        f"PPO update {batch} | "
                        f"Policy {metrics['policy_loss']:.4f} | "
                        f"Value {metrics['value_loss']:.4f} | "
                        f"Entropy {metrics['entropy']:.4f}",
                        flush=True,
                    )
                    buffer.clear()
                    batch += 1

                    save_ppo(checkpoint_path, driver, batch, args.slot)
                    print(f"Saved PPO to {checkpoint_path}", flush=True)

                if terminated:
                    print(
                        f"Episode {episode} | {outcome} | "
                        f"Distance {progress:.2f} | "
                        f"Reward {episode_reward:.3f}",
                        flush=True,
                    )
                    break
    finally:
        window.close()


if __name__ == "__main__":
    main()
