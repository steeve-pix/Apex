import argparse
import csv
import math
from pathlib import Path


def load_trajectory(path: Path) -> list[dict[str, float]]:
    required = {"time", "x", "y", "vx", "vy", "heading", "on_track"}

    with path.open(newline="", encoding="utf-8") as source:
        reader = csv.DictReader(source)

        if not required.issubset(reader.fieldnames or []):
            raise ValueError("CSV is missing required trajectory columns")

        samples = []

        for line_number, row in enumerate(reader, start=2):
            try:
                sample = {name: float(row[name]) for name in required}
            except (TypeError, ValueError) as e:
                raise ValueError(f"Invalid numeric data on line {line_number}") from e

            if not all(math.isfinite(value) for value in sample.values()):
                raise ValueError(f"Non-finite data on line {line_number}")
            if sample["on_track"] not in (0.0, 1.0):
                raise ValueError(f"Invalid track status on line {line_number}")
            if samples and sample["time"] <= samples[-1]["time"]:
                raise ValueError("Sample timestamps must increase")

            samples.append(sample)

    if not samples:
        raise ValueError("Trajectory contains no samples")

    return samples


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect an Apex trajectory")
    parser.add_argument("trajectory", type=Path)
    args = parser.parse_args()

    try:
        samples = load_trajectory(args.trajectory)
    except (OSError, ValueError) as e:
        parser.error(str(e))

    peak_speed = max(math.hypot(sample["vx"], sample["vy"]) for sample in samples)
    first_off_road = next((sample["time"] for sample in samples if sample["on_track"] == 0), None)

    duration = samples[-1]["time"] - samples[0]["time"]

    print(f"Samples: {len(samples)}")
    print(f"Duration: {duration:.2f} s")
    print(f"Peak speed: {peak_speed:.3f} units/s")

    if first_off_road is None:
        print("Vehicle stayed on track")
    else:
        print(f"first off-road sample: {first_off_road:.3f} s")


if __name__ == "__main__":
    main()
