import argparse
import json
import subprocess
from itertools import product
from pathlib import Path
from tempfile import TemporaryDirectory

from training_data import load_training_data


def collect_run(runner: Path, output: Path, centerline_offset: float, heading_offset: float) -> int:
    command = [
        str(runner.resolve()),
        "--auto",
        str(centerline_offset),
        str(heading_offset),
    ]

    result = subprocess.run(command, capture_output=True, text=True, check=True)

    output.parent.mkdir(parents=True, exist_ok=True)

    with TemporaryDirectory(dir=output.parent) as directory:
        temporary_csv = Path(directory) / "trajectory.csv"
        temporary_csv.write_text(result.stdout, encoding="utf-8")

        inputs, _ = load_training_data(temporary_csv)
        temporary_csv.replace(output)

    return len(inputs)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Collect Apex driving runs"
    )
    parser.add_argument("runner", type=Path)
    parser.add_argument("--output", type=Path, default=Path("build/runs"))
    parser.add_argument("--offsets", type=float, nargs="+", default=[-0.05, 0.0, 0.05])
    parser.add_argument("--headings", type=float, nargs="+", default=[-0.2, 0.0, 0.2])
    args = parser.parse_args()

    try:
        if args.output.exists():
            raise ValueError("Choose a new output directory")

        args.output.parent.mkdir(parents=True, exist_ok=True)

        with TemporaryDirectory(dir=args.output.parent) as directory:
            staged_runs = Path(directory) / "runs"
            records = []

            starts = product(args.offsets, args.headings)

            for index, (offset, heading) in enumerate(starts):
                run_id = f"run_{index:03d}"
                filename = f"{run_id}.csv"

                sample_count = collect_run(args.runner, staged_runs / filename, offset, heading)

                records.append({
                    "run_id": run_id,
                    "filename": filename,
                    "centerline_offset": offset,
                    "heading_offset": heading,
                    "samples": sample_count,
                })

            manifest = staged_runs / "manifest.json"
            manifest.write_text(json.dumps({"runs": records}, indent=2) + "\n", encoding="utf-8")
            staged_runs.rename(args.output)
    except subprocess.CalledProcessError as error:
        parser.error(error.stderr.strip() or str(error))
    except (OSError, ValueError) as error:
        parser.error(str(error))

    print(f"Saved {len(records)} runs to {args.output}")


if __name__ == "__main__":
    main()
