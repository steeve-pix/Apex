import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("telemetry", type=Path)
    args = parser.parse_args()

    with args.telemetry.open(newline="", encoding="utf-8") as file:
        samples = [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(file)
        ]

    if not samples:
        parser.error("The telemetry file contains no samples")

    distances = [sample["lap_distance"] for sample in samples]
    lap_length = distances[-1]

    figure, axes = plt.subplots(
        3, 1, sharex=True, figsize=(12, 8), layout="constrained"
    )

    fields = (
        ("speed", "Speed (world units/s)"),
        ("throttle", "Throttle / brake"),
        ("steering", "Steering"),
    )

    for axis, (field, label) in zip(axes, fields, strict=True):
        axis.plot(distances, [sample[field] for sample in samples])
        axis.set_ylabel(label)
        axis.grid(alpha=0.25)

        for boundary in (lap_length / 3.0, 2.0 * lap_length / 3.0):
            axis.axvline(boundary, color="gray", linestyle="--")

    axes[-1].set_xlabel("Lap distance (world units)")
    figure.suptitle(args.telemetry.stem)
    figure.savefig(args.telemetry.with_suffix(".png"), dpi=150)
    plt.show()


if __name__ == "__main__":
    main()