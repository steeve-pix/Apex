import math
from dataclasses import dataclass
from statistics import fmean, pstdev


@dataclass(frozen=True)
class NormalizationStats:
    means: tuple[float, ...]
    scales: tuple[float, ...]


def fit_normalization(inputs: list[list[float]]) -> NormalizationStats:
    if not inputs or not inputs[0]:
        raise ValueError("Normalization requires nonempty input rows")

    feature_count = len(inputs[0])

    for row in inputs:
        if len(row) != feature_count:
            raise ValueError("Input rows must have equal widths")

        if not all(math.isfinite(value) for value in row):
            raise ValueError("Input values must be finite")

    means = []
    scales = []

    for column in zip(*inputs):
        means.append(fmean(column))

        scale = pstdev(column)
        scales.append(scale if scale > 0.0 else 1.0)

    return NormalizationStats(
        means=tuple(means),
        scales=tuple(scales)
    )


def normalize_inputs(inputs: list[list[float]], stats: NormalizationStats) -> list[list[float]]:
    if not stats.means or len(stats.means) != len(stats.scales):
        raise ValueError("Normalization stats have invalid dimensions")
    if not all(math.isfinite(value) for value in stats.means):
        raise ValueError("Normalization stats must have finite values")
    if not all(math.isfinite(scale) and scale > 0.0 for scale in stats.scales):
        raise ValueError("Normalization stats must have finite and positive values")

    normalized = []
    for row in inputs:
        if len(row) != len(stats.means):
            raise ValueError("Input width must match normalization stats")
        if not all(math.isfinite(value) for value in row):
            raise ValueError("Input values must be finite")

        normalized_row = [
            (value - mean) / scale for value, mean, scale in zip(row, stats.means, stats.scales)
        ]

        if not all(math.isfinite(value) for value in normalized_row):
            raise ValueError("Normalization stats must have finite values")

        normalized.append(normalized_row)

    return normalized
