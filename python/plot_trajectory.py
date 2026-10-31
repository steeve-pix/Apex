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
    figure, (path_axes, speed_axes) = plt.subplots(1, 2, figsize=(10, 5), layout="constrained")

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

        for axes in (path_axes, speed_axes):
            axes.grid(True, alpha=0.3)

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
