import argparse
import json
from pathlib import Path

from training_data import load_training_data

TrainingArrays = tuple[list[list[float]], list[list[float]]]


def load_run_group(run_ids: list[str], run_files: dict[str, Path]) -> tuple[list[list[float]], list[list[float]]]:
    if not run_ids:
        raise ValueError("A dataset group must contain at least one run")

    if len(set(run_ids)) != len(run_ids):
        raise ValueError("A dataset group contains duplicate run IDs")

    inputs = []
    targets = []

    for run_id in run_ids:
        if run_id not in run_files:
            raise ValueError(f"Unknown run ID: {run_id}")

        run_inputs, run_targets = load_training_data(run_files[run_id])

        inputs.extend(run_inputs)
        targets.extend(run_targets)

    return inputs, targets


def load_run_files(manifest_path: Path) -> dict[str, Path]:
    document = json.loads(manifest_path.read_text(encoding="utf-8"))

    if not isinstance(document, dict):
        raise ValueError("Manifest must be a JSON object")

    records = document.get("runs")
    if not isinstance(records, list) or not records:
        raise ValueError("Manifest must contain a nonempty runs list")

    run_files = {}
    seen_files = set()

    for record in records:
        if not isinstance(record, dict):
            raise ValueError("Each run must be a JSON object")

        run_id = record.get("run_id")
        filename = record.get("filename")

        if not isinstance(run_id, str) or not run_id:
            raise ValueError("Each run requires a nonempty run_id")
        if not isinstance(filename, str) or not filename:
            raise ValueError("Each run requires a filename")

        run_path = (manifest_path.parent / filename).resolve()

        if run_id in run_files:
            raise ValueError(f"Duplicate run ID: {run_id}")

        if run_path in seen_files:
            raise ValueError("Multiple run IDs reference the same CSV")

        run_files[run_id] = run_path
        seen_files.add(run_path)

    return run_files


def load_dataset(split_path: Path) -> tuple[TrainingArrays, TrainingArrays]:
    split = json.loads(split_path.read_text(encoding="utf-8"))

    if not isinstance(split, dict):
        raise ValueError("Split must be a JSON object")

    manifest_name = split.get("manifest")
    if not isinstance(manifest_name, str) or not manifest_name:
        raise ValueError("Split requires a manifest path")

    manifest_path = (split_path.parent / manifest_name).resolve()
    run_files = load_run_files(manifest_path)

    training_ids = split.get("training_run_ids")
    validation_ids = split.get("validation_run_ids")

    for group in (training_ids, validation_ids):
        if not isinstance(group, list) or not group:
            raise ValueError("Each split group must be a nonempty list")
        if not all(isinstance(run_id, str) and run_id for run_id in group):
            raise ValueError("Split run IDs must be nonempty strings")

    training_set = set(training_ids)
    validation_set = set(validation_ids)

    if training_set & validation_set:
        raise ValueError("Training and validation runs overlap")
    if training_set | validation_set != set(run_files):
        raise ValueError("Split must cover exactly the manifest's runs")

    training = load_run_group(training_ids, run_files)
    validation = load_run_group(validation_ids, run_files)

    return training, validation


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Inspect an Apex training and validation dataset"
    )
    parser.add_argument("split", type=Path)
    args = parser.parse_args()

    try:
        training, validation = load_dataset(args.split)
    except (OSError, ValueError) as error:
        parser.error(str(error))

    for name, group in (("Training", training), ("Validation", validation)):
        inputs, targets = group

        print(f"{name} samples: {len(inputs)}")
        print(f"  Input features: {len(inputs[0])}")
        print(f"  Control targets: {len(targets[0])}")


if __name__ == "__main__":
    main()
