import argparse
import math
import matplotlib
from pathlib import Path

matplotlib.use('Agg')

import matplotlib.pyplot as plt

from analyze_trajectory import load_trajectory


def plot_trajectory(samples: list[dict[str, float]], output: Path) -> None:
    times = [sample["time"] for sample in samples]
    xs = [sample["x"] for sample in samples]
    ys = [sample["y"] for sample in samples]
    speeds = [math.hypot(sample["vx"], sample["vy"]) for sample in samples]

    has_sensors = "sensor_0_distance" in samples[0]
    panel_count = 3 if has_sensors else 2

    figure, axes = plt.subplots(1, panel_count, figsize=(5 * panel_count, 5), layout="constrained")
    path_axes, speed_axes = axes[:2]

    try:
        path_axes.plot(xs, ys, label="Vehicle path")
        path_axes.scatter(xs[0], ys[0], color="green", label="Start")
        path_axes.scatter(xs[-1], ys[-1], color="red", label="Finish", zorder=3)
        path_axes.set(
            title="Vehicle trajectory",
            xlabel="X (world units)",
            ylabel="Y (world units)"
        )
        path_axes.set_aspect("equal", adjustable="datalim")
        path_axes.legend()

        speed_axes.plot(times, speeds)
        speed_axes.set(
            title="Speed over time",
            xlabel="Simulation time (s)",
            ylabel="Speed (units/s)"
        )
        if has_sensors:
            sensor_axes = axes[2]
            sensors_names = (
                "Right",
                "Front-right",
                "Forward",
                "Front-left",
                "Left"
            )

            for index, name in enumerate(sensors_names):
                distances = [
                    sample[f"sensor_{index}_distance"]
                    if sample[f"sensor_{index}_hit"] == 1.0 else math.nan for sample in samples
                ]

                sensor_axes.plot(times, distances, label=name)

            sensor_axes.set(
                title="Detected edge distances",
                xlabel="Simulation time (s)",
                ylabel="Distance (world units)"
            )
            sensor_axes.legend()

        for panel in axes:
            panel.grid(True, alpha=0.3)

        output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(output, dpi=160)
    finally:
        plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot an Apex trajectory")
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--output", type=Path, default=Path("build/trajectory.png"))
    args = parser.parse_args()

    try:
        samples = load_trajectory(args.trajectory)
        plot_trajectory(samples, args.output)
    except (OSError, ValueError) as e:
        parser.error(str(e))

    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
