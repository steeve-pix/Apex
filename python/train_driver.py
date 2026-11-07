import argparse
import math
from pathlib import Path
from tempfile import TemporaryDirectory

import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset

from dataset_loader import load_dataset
from neural_driver import NeuralDriver
from normalization import NormalizationStats, fit_normalization, normalize_inputs
from training_data import SENSOR_FIELDS, TARGET_FIELDS


def prepare_datasets(split_path: Path) -> tuple[TensorDataset, TensorDataset, NormalizationStats]:
    training, validation = load_dataset(split_path)
    stats = fit_normalization(training[0])

    datasets = []

    for inputs, targets in (training, validation):
        normalized = normalize_inputs(inputs, stats)

        dataset = TensorDataset(
            torch.tensor(normalized, dtype=torch.float32),
            torch.tensor(targets, dtype=torch.float32)
        )

        datasets.append(dataset)

    return datasets[0], datasets[1], stats


def train_epoch(model: nn.Module, loader: DataLoader, optimizer: torch.optim.Optimizer) -> float:
    model.train()
    loss_function = nn.MSELoss()

    total_loss = 0.0
    sample_count = 0

    for inputs, targets in loader:
        optimizer.zero_grad(set_to_none=True)

        predictions = model(inputs)
        loss = loss_function(predictions, targets)

        if not torch.isfinite(loss).item():
            raise ValueError("Training produced a non-finite loss")

        loss.backward()
        optimizer.step()

        batch_size = inputs.shape[0]
        total_loss += loss.item() * batch_size
        sample_count += batch_size

    if sample_count == 0:
        raise ValueError("Training loader contains no samples")

    return total_loss / sample_count


@torch.inference_mode()
def evaluate(model: nn.Module, loader: DataLoader) -> float:
    model.eval()
    loss_function = nn.MSELoss()

    total_loss = 0.0
    sample_count = 0

    for inputs, targets in loader:
        predictions = model(inputs)
        loss = loss_function(predictions, targets)

        if not torch.isfinite(loss).item():
            raise ValueError("Evaluation produced a non-finite loss")

        batch_size = inputs.shape[0]
        total_loss += loss.item() * batch_size
        sample_count += batch_size

    if sample_count == 0:
        raise ValueError("Validation loader contains no samples")

    return total_loss / sample_count


def save_checkpoint(output: Path, model: NeuralDriver, stats: NormalizationStats, epoch: int,
                    validation_mse: float) -> None:
    checkpoint = {
        "model_state": model.state_dict(),
        "hidden_size": model.network[0].out_features,
        "input_fields": [*SENSOR_FIELDS, "forward_speed", "sideways_speed"],
        "target_fields": list(TARGET_FIELDS),
        "normalization": {
            "means": list(stats.means),
            "scales": list(stats.scales),
        },
        "epoch": epoch,
        "validation_mse": validation_mse,
    }

    output.parent.mkdir(parents=True, exist_ok=True)

    with TemporaryDirectory(dir=output.parent) as directory:
        temporary_file = Path(directory) / "driver.pt"
        torch.save(checkpoint, temporary_file)
        temporary_file.replace(output)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Train an apex neural driver"
    )
    parser.add_argument("split", type=Path)
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--learning-rate", type=float, default=0.001)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", type=Path, default=Path("build/neural_driver.pt"))
    args = parser.parse_args()

    if args.epochs <= 0 or args.batch_size <= 0:
        parser.error("Epochs and batch size must be positive")

    if not math.isfinite(args.learning_rate) or args.learning_rate <= 0:
        parser.error("Learning rate must be finite and positive")

    if args.output.resolve() == args.split.resolve():
        parser.error("Checkpoint output must differ from the split file")

    try:
        training, validation, stats = prepare_datasets(args.split)

        torch.manual_seed(args.seed)
        model = NeuralDriver()

        training_loader = DataLoader(training, batch_size=args.batch_size, shuffle=True)
        validation_loader = DataLoader(validation, batch_size=args.batch_size)

        optimizer = torch.optim.Adam(model.parameters(), lr=args.learning_rate)

        initial_loss = evaluate(model, validation_loader)
        print(f"Initial validation MSE: {initial_loss:.6f}")
        best_validation_loss = initial_loss
        best_epoch = 0
        save_checkpoint(args.output, model, stats, best_epoch, best_validation_loss)

        for epoch in range(1, args.epochs + 1):
            training_loss = train_epoch(model, training_loader, optimizer)
            validation_loss = evaluate(model, validation_loader)

            if validation_loss < best_validation_loss:
                save_checkpoint(args.output, model, stats, epoch, validation_loss)
                best_validation_loss = validation_loss
                best_epoch = epoch

            print(
                f"Epoch {epoch:03d} | "
                f"training MSE {training_loss:.6f} | "
                f"validation MSE {validation_loss:.6f}"
            )

        print(
            f"Saved best checkpoint to {args.output} | "
            f"epoch {best_epoch} | validation MSE {best_validation_loss:.6f}"
        )
    except (OSError, ValueError) as e:
        parser.error(str(e))

    for name, dataset in (("Training", training), ("Validation", validation)):
        inputs, targets = dataset.tensors
        print(f"{name} inputs: {inputs.shape}")
        print(f"{name} targets: {targets.shape}")

    print(f"Normalized features: {len(stats.means)}")


if __name__ == "__main__":
    main()
