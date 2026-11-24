import torch
from torch import nn

from rl_driver import RLDriver
from racing_observation import OPPONENT_INPUTS

THROTTLE_ADJUSTMENT_LIMIT = 0.15
STEERING_ADJUSTMENT_LIMIT = 0.10
OPPONENT_THROTTLE_LIMIT = 0.60
OPPONENT_STEERING_LIMIT = 0.15


class EvolutionDriver(RLDriver):
    """Evolve continuous corrections around an existing discrete driving policy."""

    def __init__(self):
        super().__init__()
        self.control_head = nn.Linear(self.actor.in_features, 2)
        nn.init.zeros_(self.control_head.weight)
        nn.init.zeros_(self.control_head.bias)
        self.opponent_head = nn.Linear(OPPONENT_INPUTS, 2, bias=False)
        nn.init.zeros_(self.opponent_head.weight)

    def load_evolution_state(self, state):
        upgraded = dict(state)
        for name in ("control_head", "opponent_head"):
            defaults = getattr(self, name).state_dict()
            if all(f"{name}.{key}" not in upgraded for key in defaults):
                upgraded.update({f"{name}.{key}": value for key, value in defaults.items()})
        self.load_state_dict(upgraded)

    def sample_action(self, observation, *, greedy=False, return_index=False, opponents=None):
        result = super().sample_action(
            observation, greedy=greedy, return_index=return_index
        )
        action = result[0]
        inputs = torch.as_tensor(observation, dtype=torch.float32)
        corrections = torch.tanh(self.control_head(self.features(inputs)))
        throttle_delta, steering_delta = corrections.tolist()
        throttle_correction = THROTTLE_ADJUSTMENT_LIMIT * throttle_delta
        steering_correction = STEERING_ADJUSTMENT_LIMIT * steering_delta
        if opponents is not None:
            nearby = torch.as_tensor(opponents, dtype=torch.float32)
            response = torch.tanh(self.opponent_head(nearby))
            throttle_correction += OPPONENT_THROTTLE_LIMIT * response[0].item()
            steering_correction += OPPONENT_STEERING_LIMIT * response[1].item()

        action.throttle = max(-1.0, min(
            1.0, action.throttle + throttle_correction
        ))
        action.steering = max(-1.0, min(
            1.0, action.steering + steering_correction
        ))
        if observation[-2] <= 0.0:
            action.throttle = max(0.0, action.throttle)

        return result
