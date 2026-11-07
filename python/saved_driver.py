from pathlib import Path

import torch

from neural_driver import NeuralDriver
from normalization import NormalizationStats, normalize_inputs
from training_data import SENSOR_FIELDS, TARGET_FIELDS


def load_driver(checkpoint_path: Path) -> tuple[NeuralDriver, NormalizationStats]:
    checkpoint = torch.load(checkpoint_path, map_location=torch.device('cpu'), weights_only=True)

    if not isinstance(checkpoint, dict):
        raise ValueError("Checkpoint must contain a dictionary")

    expected_input = [
        *SENSOR_FIELDS,
        "forward_speed",
        "sideways_speed",
    ]

    if checkpoint.get("input_fields") != expected_input:
        raise ValueError("Checkpoint input order is incompatible")

    if checkpoint.get("target_fields") != list(TARGET_FIELDS):
        raise ValueError("Checkpoint target order is incompatible")

    normalization = checkpoint["normalization"]
    stats = NormalizationStats(
        means=tuple(normalization["means"]),
        scales=tuple(normalization["scales"])
    )

    if len(stats.means) != len(expected_input):
        raise ValueError("Checkpoint mean and scale are incompatible")

    normalize_inputs([], stats)

    model = NeuralDriver(hidden_size=checkpoint["hidden_size"])
    model.load_state_dict(checkpoint["model_state"])
    model.eval()

    return model, stats


@torch.inference_mode()
def predict_controls(model: NeuralDriver, stats: NormalizationStats, inputs: list[list[float]]) -> list[list[float]]:
    normalized = normalize_inputs(inputs, stats)

    if not normalized:
        return []

    model.eval()
    input_tensor = torch.tensor(normalized, dtype=torch.float32)
    predictions = model(input_tensor)

    if not torch.isfinite(predictions).all().item():
        raise ValueError("Driver produced non-finite controls")

    return predictions.tolist()
