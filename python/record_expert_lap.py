import csv
import time
from pathlib import Path

import apex_core as apex

from agent import observe_inputs
from training_data import SENSOR_FIELDS, TARGET_FIELDS


def record_lap(output: Path) -> None:
    settings = apex.make_race_settings()
    race = apex.Race(settings)
    sensors = apex.SensorSettings()
    controls = apex.VehicleControls()
    rows = []

    dt = 1 / 120
    accumulator = 0.0

    window = apex.SimulationWindow(settings.track)
    previous_time = time.perf_counter()

    try:
        while race.status.completed_laps < 1:
            if window.poll_events().close_requested:
                print("Recording cancelled the lap finished.")
                return

            now = time.perf_counter()
            accumulator += min(now - previous_time, 0.25)
            previous_time = now

            while accumulator >= dt and race.status.completed_laps < 1:
                world = race.player
                body = world.vehicle.body
                observation = observe_inputs(world, settings, sensors)

                row = {
                    "time": world.elapsed_time,
                    "x": body.position.x,
                    "y": body.position.y,
                    "vx": body.velocity.x,
                    "vy": body.velocity.y,
                    "heading": world.vehicle.heading,
                    "on_track": int(race.status.on_track),
                }
                row.update(zip(SENSOR_FIELDS, observation[:len(SENSOR_FIELDS)], strict=True))
                race.step(controls, automatic=True, dt=dt)
                action = race.player_controls
                row.update(throttle=action.throttle, steering=action.steering)
                rows.append(row)
                accumulator -= dt

                if not race.status.on_track:
                    raise RuntimeError("Expert left the track during recording.")

            window.draw_race(race, sensors, automatic=True)
            time.sleep(0.001)
    finally:
        window.close()

    fields = (
        "time", "x", "y", "vx", "vy", "heading", "on_track",
        *SENSOR_FIELDS, *TARGET_FIELDS
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Saved {len(rows)} training pairs to {output}")


if __name__ == "__main__":
    record_lap(Path("build/circuit_runs/expert_001.csv"))
