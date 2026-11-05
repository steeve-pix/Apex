import torch
from torch import nn

from training_data import SENSOR_FIELDS, TARGET_FIELDS


class NeuralDriver(nn.Module):
    def __init__(self, hidden_size: int = 32) -> None:
        super().__init__()

        if hidden_size <= 0:
            raise ValueError("Hidden size must be positive")

        input_count = len(SENSOR_FIELDS) + 2
        output_count = len(TARGET_FIELDS)

        self.network = nn.Sequential(
            nn.Linear(input_count, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, output_count),
            nn.Tanh()
        )

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        return self.network(inputs)
