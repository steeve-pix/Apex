import unittest
from unittest.mock import patch

import torch

from evolution_driver import EvolutionDriver
from train_population import validate_champion, validation_score


def result(completed=True, progress=1.0, seconds=60.0, penalty=0.0):
    return {"completed": completed, "progress": progress, "time": seconds, "penalty": penalty}


class PopulationValidationTests(unittest.TestCase):
    def test_reliable_completion_precedes_speed(self):
        reliable = [result(seconds=70.0)] * 3
        fast_crash = [result(seconds=40.0), result(seconds=40.0), result(False, 0.9, 35.0)]
        self.assertGreater(validation_score(reliable), validation_score(fast_crash))

    def test_collision_penalty_counts_when_all_heats_finish(self):
        clean = [result(seconds=60.0)] * 3
        rough = [result(seconds=58.0, penalty=5.0)] * 3
        self.assertGreater(validation_score(clean), validation_score(rough))

    def test_champion_survives_rejected_challenger_and_can_improve(self):
        incumbent = EvolutionDriver()
        challenger = EvolutionDriver()
        state = {}
        rng = torch.get_rng_state().clone()
        with patch("train_population.Path.is_file", return_value=False), patch(
            "train_population.validate_driver", side_effect=[
                [result(seconds=60.0)] * 3,
                [result(False, 0.8)] * 3,
            ],
        ) as evaluate:
            champion = validate_champion(challenger, [incumbent], state, None, None, None, 1)
        self.assertEqual(evaluate.call_count, 2)
        self.assertTrue(torch.equal(torch.get_rng_state(), rng))
        self.assertTrue(torch.equal(champion.actor.weight, incumbent.actor.weight))
        cached_results = state["validation"]["results"]
        with patch("train_population.validate_driver", return_value=[result(seconds=55.0)] * 3) as evaluate:
            champion = validate_champion(challenger, [incumbent], state, None, None, None, 5)
        self.assertEqual(evaluate.call_count, 1)
        self.assertTrue(torch.equal(champion.actor.weight, challenger.actor.weight))
        self.assertGreater(validation_score(state["validation"]["results"]), validation_score(cached_results))


if __name__ == "__main__":
    unittest.main()
