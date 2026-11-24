import unittest

import torch

from evolution_driver import (
    EvolutionDriver,
    OPPONENT_STEERING_LIMIT,
    OPPONENT_THROTTLE_LIMIT,
)
from racing_observation import OPPONENT_INPUTS
from train_population import mutate


class EvolutionControlsTests(unittest.TestCase):
    def test_saturated_traffic_response_is_bounded(self):
        driver = EvolutionDriver()
        observation = [0.0] * driver.features[0].in_features
        observation[-2] = 1.0
        with torch.no_grad():
            baseline = driver.sample_action(observation, greedy=True)[0]
            for sign in (-1.0, 1.0):
                driver.opponent_head.weight.fill_(sign * 100.0)
                action = driver.sample_action(
                    observation, greedy=True, opponents=[1.0] * OPPONENT_INPUTS,
                )[0]
                self.assertLessEqual(abs(action.throttle - baseline.throttle), OPPONENT_THROTTLE_LIMIT + 1e-6)
                self.assertLessEqual(abs(action.steering - baseline.steering), OPPONENT_STEERING_LIMIT + 1e-6)

    def test_racing_mutation_preserves_parent_and_road_policy(self):
        torch.manual_seed(42)
        parent = EvolutionDriver()
        before = {name: value.clone() for name, value in parent.state_dict().items()}
        child = mutate(parent, 0.01, racing=True)
        for name, value in parent.state_dict().items():
            self.assertTrue(torch.equal(value, before[name]))
            if name.startswith(("features.", "actor.")):
                self.assertTrue(torch.equal(value, child.state_dict()[name]))
        changed = torch.count_nonzero(child.opponent_head.weight - parent.opponent_head.weight).item()
        self.assertGreater(changed, 0)
        self.assertLess(changed, parent.opponent_head.weight.numel() // 2)

    def test_road_exploration_changes_driving_weights_without_changing_parent(self):
        torch.manual_seed(42)
        parent = EvolutionDriver()
        before = {name: value.clone() for name, value in parent.state_dict().items()}
        child = mutate(parent, 0.3, racing=True, adapt_road=True)
        for name, value in parent.state_dict().items():
            self.assertTrue(torch.equal(value, before[name]))
        for module_name in ("features", "actor"):
            changes = torch.cat([
                (value - before[name]).flatten()
                for name, value in child.state_dict().items()
                if name.startswith(module_name + ".")
            ])
            self.assertGreater(torch.count_nonzero(changes).item(), 0)
            self.assertLess(torch.count_nonzero(changes).item(), changes.numel() // 4)
            self.assertLess(changes.abs().max().item(), 0.1)
        self.assertTrue(torch.equal(child.critic.weight, parent.critic.weight))


if __name__ == "__main__":
    unittest.main()
