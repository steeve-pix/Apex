from dataclasses import dataclass

import torch


@dataclass
class Transition:
    observation: list[float]
    action_index: int
    old_log_prob: float
    value: float
    reward: float
    terminated: bool
    next_value: float


class RolloutBuffer:
    def __init__(self):
        self.transitions: list[Transition] = []

    def add(self, transition: Transition):
        self.transitions.append(transition)

    def clear(self):
        self.transitions.clear()

    def __len__(self):
        return len(self.transitions)

    def training_batch(self, gamma=0.995, gae_lambda=0.95):
        if not self.transitions:
            raise ValueError("Cannot train from an empty rollout")

        advantages = torch.zeros(len(self), dtype=torch.float32)
        advantage = 0.0

        for index in reversed(range(len(self))):
            transition = self.transitions[index]
            continuation = float(not transition.terminated)

            delta = (transition.reward + gamma * continuation * transition.next_value - transition.value)
            advantage = (delta + gamma * gae_lambda * continuation * advantage)
            advantages[index] = advantage

        observations = torch.tensor([step.observation for step in self.transitions], dtype=torch.float32)
        actions = torch.tensor([step.action_index for step in self.transitions], dtype=torch.long)
        old_log_probs = torch.tensor([step.old_log_prob for step in self.transitions], dtype=torch.float32)
        values = torch.tensor([step.value for step in self.transitions], dtype=torch.float32)
        returns = advantages + values

        return observations, actions, old_log_probs, returns, advantages
