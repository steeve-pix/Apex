import argparse
import math
from pathlib import Path

from analyze_trajectory import SENSOR_COUNT, load_trajectory

SENSOR_FIELDS = tuple(
    f"sensor_{index}_{field}"
    for index in range(SENSOR_COUNT)
    for field in ("distance", "hit")
)

TARGET_FIELDS = ("throttle", "steering")


def build_input_features(sample: dict[str, float]) -> list[float]:
    cosine = math.cos(sample["heading"])
    sine = math.sin(sample["heading"])

    forward_speed = sample["vx"] * cosine + sample["vy"] * sine
    sideways_speed = -sample["vx"] * sine + sample["vy"] * cosine

    features = [sample[name] for name in SENSOR_FIELDS]
    features.extend([forward_speed, sideways_speed])

    return features


def load_training_data(path: Path) -> tuple[list[list[float]], list[list[float]]]:
    samples = load_trajectory(path)

    required = set(SENSOR_FIELDS) | set(TARGET_FIELDS) | {"vx", "vy", "heading"}

    if not required.issubset(samples[0]):
        raise ValueError("Training data requires sensor readings and control targets")

    inputs = []
    targets = []

    for sample in samples:
        inputs.append(build_input_features(sample))
        targets.append([sample[name] for name in TARGET_FIELDS])

    return inputs, targets


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Inspect Apex training examples"

    )
    parser.add_argument("trajectory", type=Path)
    args = parser.parse_args()

    try:
        inputs, targets = load_training_data(args.trajectory)
    except(OSError, ValueError) as e:
        parser.error(str(e))

    print(f"Samples: {len(inputs)}")
    print(f"Inputs per sample: {len(inputs[0])}")
    print(f"Targets per sample: {len(targets[0])}")


if __name__ == "__main__":
    main()
