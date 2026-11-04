import argparse
import json
import math
import random
from pathlib import Path
from tempfile import TemporaryDirectory


def split_run_ids(run_ids: list[str], validation_fraction: float = 0.2, seed: int = 42) -> tuple[list[str], list[str]]:
    if len(run_ids) < 2:
        raise ValueError("At least two runs are required")
    if len(set(run_ids)) != len(run_ids):
        raise ValueError("Run IDs must be unique")
    if not 0.0 < validation_fraction < 1.0:
        raise ValueError("Validation fraction must be between 0 and 1")

    shuffled = sorted(run_ids)
    random.Random(seed).shuffle(shuffled)

    validation_count = math.ceil(len(shuffled) * validation_fraction)
    validation_count = min(len(shuffled) - 1, max(1, validation_count))

    validation_ids = shuffled[:validation_count]
    training_ids = shuffled[validation_count:]

    return training_ids, validation_ids


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Split Apex runs into training and validation groups"
    )
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--validation-fraction", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--output", type=Path, default=Path("build/split.json")
    )
    args = parser.parse_args()

    try:
        document = json.loads(
            args.manifest.read_text(encoding="utf-8")
        )

        if not isinstance(document, dict):
            raise ValueError("Manifest must be a JSON object")

        records = document.get("runs")
        if not isinstance(records, list):
            raise ValueError("Manifest must contain a runs list")

        run_ids = []

        for record in records:
            if not isinstance(record, dict):
                raise ValueError("Each run must be a JSON object")

            run_id = record.get("run_id")
            if not isinstance(run_id, str) or not run_id:
                raise ValueError("Each run requires a nonempty run_id")

            run_ids.append(run_id)

        training_ids, validation_ids = split_run_ids(
            run_ids,
            args.validation_fraction,
            args.seed,
        )

        split = {
            "manifest": str(args.manifest.resolve()),
            "seed": args.seed,
            "validation_fraction": args.validation_fraction,
            "training_run_ids": training_ids,
            "validation_run_ids": validation_ids,
        }

        if args.output.resolve() == args.manifest.resolve():
            raise ValueError("Split output must differ from the manifest")

        args.output.parent.mkdir(parents=True, exist_ok=True)

        with TemporaryDirectory(dir=args.output.parent) as directory:
            temporary_json = Path(directory) / "split.json"
            temporary_json.write_text(
                json.dumps(split, indent=2) + "\n",
                encoding="utf-8",
                )
            temporary_json.replace(args.output)

    except (OSError, ValueError) as error:
        parser.error(str(error))

    print(f"Training runs: {len(training_ids)}")
    print(f"Validation runs: {len(validation_ids)}")
    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
