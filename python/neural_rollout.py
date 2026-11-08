import argparse
import math
import time
from pathlib import Path

import apex_core

from analyze_trajectory import SENSOR_COUNT
from saved_driver import load_driver, predict_controls
from training_data import SENSOR_FIELDS, build_input_features


def observe_inputs(world: apex_core.WorldState, settings: apex_core.WorldSettings, sensors: apex_core.SensorSettings) -> \
        list[float]:
    readings = apex_core.sense_track(world.vehicle, settings.track, sensors)

    if len(readings) != SENSOR_COUNT:
        raise ValueError("Simulation sensor count is incompatible")

    body = world.vehicle.body

    sample = {
        "vx": body.velocity.x,
        "vy": body.velocity.y,
        "heading": world.vehicle.heading,
    }

    for index, reading in enumerate(readings):
        sample[f"sensor_{index}_distance"] = reading.distance
        sample[f"sensor_{index}_hit"] = float(reading.hit)

    return build_input_features(sample)


def failure_reason(world: apex_core.WorldState, settings: apex_core.WorldSettings) -> str | None:
    body = world.vehicle.body

    values = (body.position.x, body.position.y, body.velocity.x, body.velocity.y, world.vehicle.heading,
              world.elapsed_time)

    if not all(math.isfinite(value) for value in values):
        return "Invalid physics state"

    if not apex_core.is_on_track(body.position, settings.track):
        return "Car left the track"

    return None


def run_rollout(checkpoint_path: Path, steps: int = 1200, dt: float = 1.0 / 120.0) -> list[dict[str, float]]:
    if steps <= 0:
        raise ValueError("Steps must be positive")

    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError("Time step must be finite")

    model, stats = load_driver(checkpoint_path)
    settings = apex_core.WorldSettings()
    sensors = apex_core.SensorSettings()
    world = apex_core.make_initial_world(settings)
    controls = apex_core.VehicleControls()

    rows = []

    for step in range(steps + 1):
        features = observe_inputs(world, settings, sensors)

        controls.throttle, controls.steering = predict_controls(model, stats, [features])[0]

        body = world.vehicle.body

        row = {
            "time": world.elapsed_time,
            "x": body.position.x,
            "y": body.position.y,
            "vx": body.velocity.x,
            "vy": body.velocity.y,
            "heading": world.vehicle.heading,
            "on_track": float(apex_core.is_on_track(body.position, settings.track)),
            "throttle": controls.throttle,
            "steering": controls.steering,
        }

        row.update(zip(SENSOR_FIELDS, features[:len(SENSOR_FIELDS)], strict=True))
        rows.append(row)

        if step < steps:
            apex_core.step_world(world, controls, settings, dt)

    return rows


class ProgressTracker:
    def __init__(self, world: apex_core.WorldState, timeout: float = 5.0, minimum_progress: float = 0.02) -> None:
        for value in (timeout, minimum_progress):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("Progress thresholds must be finite and positive")

        position = world.vehicle.body.position
        self.previous_angle = math.atan2(position.y, position.x)
        self.distance = 0.0
        self.checkpoint_distance = 0.0
        self.checkpoint_time = world.elapsed_time
        self.timeout = timeout
        self.minimum_progress = minimum_progress

    def update(self, world: apex_core.WorldState, track: apex_core.TrackSettings) -> str | None:
        position = world.vehicle.body.position
        angle = math.atan2(position.y, position.x)

        angle_change = math.remainder(angle - self.previous_angle, math.tau)

        self.previous_angle = angle
        self.distance += angle_change * track.centerline_radius

        if self.distance >= self.checkpoint_distance + self.minimum_progress:
            self.checkpoint_distance = self.distance
            self.checkpoint_time = world.elapsed_time

        if world.elapsed_time - self.checkpoint_time >= self.timeout:
            return "No forward progress"

        return None


def run_visual(checkpoint_path: Path, dt: float = 1.0 / 120.0) -> None:
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError("Time step must be finite and positive")

    model, stats = load_driver(checkpoint_path)
    settings = apex_core.WorldSettings()
    sensors = apex_core.SensorSettings()
    world = apex_core.make_initial_world(settings)
    progress = ProgressTracker(world)
    controls = apex_core.VehicleControls()

    paused = False
    failure = None
    accumulator = 0.0
    previous_time = time.perf_counter()

    window = apex_core.SimulationWindow(settings.track)

    try:
        if hasattr(apex_core, "set_application_icon"):
            icon_path = (Path(apex_core.__file__).resolve().parent / "apex-icon" / "Apex.icns")
            apex_core.set_application_icon(str(icon_path))

        while True:
            current_time = time.perf_counter()
            frame_time = min(current_time - previous_time, 0.25)
            previous_time = current_time

            inputs = window.poll_events()

            if inputs.close_requested:
                break

            if inputs.reset_requested:
                world = apex_core.make_initial_world(settings)
                progress = ProgressTracker(world)
                failure = None
                paused = False
                accumulator = 0.0
                frame_time = 0.0

            if inputs.pause_requested:
                paused = not paused
                accumulator = 0.0
                frame_time = 0.0

            if not paused and failure is None:
                accumulator += frame_time

                while accumulator >= dt:
                    features = observe_inputs(world, settings, sensors)
                    controls.throttle, controls.steering = predict_controls(model, stats, [features])[0]

                    candidate = apex_core.WorldState()
                    candidate.vehicle = world.vehicle
                    candidate.elapsed_time = world.elapsed_time

                    apex_core.step_world(candidate, controls, settings, dt)
                    failure = failure_reason(candidate, settings)

                    if failure is None:
                        failure = progress.update(candidate, settings.track)

                    accumulator -= dt

                    if failure is not None:
                        accumulator = 0.0
                        break

                    world = candidate

            if failure is not None:
                status = f"Failed | {failure} | R to restart"
            elif paused:
                status = f"Paused | Space to resume"
            else:
                status = f"Neural driver | {world.elapsed_time:.1f} s "

            window.draw(world, sensors, status)
            time.sleep(0.001)
    finally:
        window.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Watch the learned driver in the Apex window"
    )
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("--dt", type=float, default=1.0 / 120.0)
    args = parser.parse_args()

    try:
        run_visual(args.checkpoint, args.dt)
    except (OSError, ValueError, RuntimeError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
