import apex_core as apex
import torch
from torch import nn
from torch.distributions import Categorical

from training_data import SENSOR_FIELDS

ACTIONS = tuple(
    (throttle, steering)
    for throttle in (0.0, 0.5, 1.0)
    for steering in (-0.65, 0.0, 0.65)
)


class RLDriver(nn.Module):
    def __init__(self):
        super().__init__()
        self.features = nn.Sequential(
            nn.Linear(len(SENSOR_FIELDS) + 2, 64),
            nn.Tanh()
        )
        self.actor = nn.Linear(64, len(ACTIONS))
        self.critic = nn.Linear(64, 1)
        self.optimizer = torch.optim.Adam(self.parameters(), lr=0.0003)

    def forward(self, observation: list[float]):
        inputs = torch.tensor(observation, dtype=torch.float32)
        features = self.features(inputs)
        return self.actor(features), self.critic(features).squeeze(-1)

    def sample_action(self, observation: list[float], *, greedy: bool = False):
        logits, value = self(observation)
        policy = Categorical(logits=logits)
        choice = logits.argmax() if greedy else policy.sample()

        action = apex.VehicleControls()
        action.throttle, action.steering = ACTIONS[choice.item()]
        if action.throttle == 0.0:
            # Keep the saved action indices. Lifting applies braking in proportion to forward speed.
            forward_speed = observation[-2]
            action.throttle = -min(1.0, 0.46 * max(0.0, forward_speed))

        return action, policy.log_prob(choice), value, policy.entropy()

    def learn(self, log_prob, value, entropy, reward, next_observation):
        with torch.no_grad():
            target = value.new_tensor(reward)
            if next_observation is not None:
                _, next_value = self(next_observation)
                target = target + 0.99 * next_value

        advantage = target - value
        policy_loss = -log_prob * advantage.detach()
        value_loss = 0.5 * advantage.square()
        loss = policy_loss + value_loss - 0.01 * entropy

        self.optimizer.zero_grad(set_to_none=True)
        loss.backward()
        nn.utils.clip_grad_norm_(self.parameters(), 1.0)
        self.optimizer.step()
