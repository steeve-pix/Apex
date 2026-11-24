import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from train_population import Candidate


class PopulationStallTests(unittest.TestCase):
    def candidate(self):
        with patch("train_population.apex.track_progress", return_value=(0.0, 100.0)):
            race = SimpleNamespace(
                player=SimpleNamespace(
                    vehicle=SimpleNamespace(body=SimpleNamespace(position=None)),
                    elapsed_time=0.0,
                ),
                status=SimpleNamespace(on_track=True, stuck_on_wall=False, last_lap=None),
                player_controls=None,
            )
            car = Candidate(None, SimpleNamespace(track=None), 0, race=race)
        car.telemetry = Mock()
        return car

    def feed(self, car, seconds, distance):
        car.race.player.elapsed_time = seconds
        with patch("train_population.apex.track_progress", return_value=(distance, 100.0)):
            car.record_step(None)

    def test_blocked_car_retires_after_five_seconds(self):
        car = self.candidate()
        for tick in range(1, 601):
            self.feed(car, tick / 120, 0.0)
        self.assertTrue(car.finished)
        self.assertIn("blocked or too slow", car.retirement_reason)

    def test_creeping_contact_cannot_keep_resetting_stall_timer(self):
        car = self.candidate()
        for tick in range(1, 601):
            self.feed(car, tick / 120, tick / 120 * 0.05)
        self.assertTrue(car.finished)
        self.assertGreater(car.last_progress_time, 4.0)

    def test_launch_and_slow_corner_continue(self):
        car = self.candidate()
        for tick in range(1, 2401):
            seconds = tick / 120
            self.feed(car, seconds, max(0.0, seconds - 1.0) * 0.2)
            self.assertFalse(car.finished)
        self.assertLessEqual(len(car.progress_history), 603)

    def test_back_and_forth_contact_is_not_forward_progress(self):
        car = self.candidate()
        for tick in range(1, 601):
            self.feed(car, tick / 120, 0.04 if tick % 2 else 0.0)
        self.assertTrue(car.finished)

    def test_lap_completion_takes_priority_over_stall_reason(self):
        car = self.candidate()
        car.race.status.last_lap = 50.0
        self.feed(car, 50.0, 0.0)
        self.assertTrue(car.finished)
        self.assertEqual(car.lap_time, 50.0)
        self.assertIsNone(car.retirement_reason)


if __name__ == "__main__":
    unittest.main()
