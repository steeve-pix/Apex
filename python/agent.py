import apex_core

from analyze_trajectory import SENSOR_COUNT
from neural_driver import NeuralDriver
from normalization import NormalizationStats
from saved_driver import predict_controls
from training_data import build_input_features


def observe_inputs(world: apex_core.WorldState, settings: apex_core.WorldSettings, sensors: apex_core.SensorSettings) -> \
        list[float]:
    readings = apex_core.sense_track(world.vehicle, settings.track, sensors)

    if len(readings) != SENSOR_COUNT:
        raise ValueError("Simulation sensor count is incompatible")

    body = world.vehicle.body

    sample = {
        "vx": body.velocity.x,
        "vy": body.velocity.y,
        "heading": world.vehicle.heading,
    }

    for index, reading in enumerate(readings):
        sample[f"sensor_{index}_distance"] = reading.distance
        sample[f"sensor_{index}_hit"] = float(reading.hit)

    return build_input_features(sample)


class NeuralAgent:
    def __init__(self, model: NeuralDriver, stats: NormalizationStats) -> None:
        self.model = model
        self.stats = stats

    def act(self, observation: list[float]) -> apex_core.VehicleControls:
        (throttle, steering), = predict_controls(self.model, self.stats, [observation])

        action = apex_core.VehicleControls()
        action.throttle = throttle
        action.steering = steering
        return action
