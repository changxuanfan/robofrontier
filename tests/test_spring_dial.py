"""Count-only apparatus contract; no policy solver or hidden angle endpoint."""
import math
import unittest

from robofrontier.spring_dial import DialFeedback, PARAMETERS


class SpringDialFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.game = DialFeedback(PARAMETERS['feedback'], secret=[90, 180, 270, 90])
        self.time = 0.

    def hold(self, degrees, duration=.4, speed=0.):
        for _ in range(round(duration/.01)):
            self.time += .01
            result = self.game.update(self.time, [math.radians(a) for a in degrees], [math.radians(speed)]*4)
        return result

    def test_zero_and_transit_do_not_reveal_a_score(self):
        for angles in ([0]*4, [90, 0, 270, 90], [45]*4):
            result = self.hold(angles)
            self.assertIsNone(result['correct_count'])
            self.assertFalse(result['ready']); self.assertEqual(result['valid_trials'], 0)

    def test_stable_wrong_tuple_can_legitimately_score_zero(self):
        self.assertEqual(self.hold([180, 270, 90, 180])['correct_count'], 0)

    def test_left_90_and_both_180_endpoints_are_recognized(self):
        result = self.hold([90, -180, -90, 90])
        self.assertEqual(result['correct_count'], 4); self.assertTrue(result['won'])
        self.assertEqual(set(result), {'ready', 'correct_count', 'won', 'valid_trials'})

    def test_uninterrupted_hold_is_one_trial_not_a_stream_of_queries(self):
        self.hold([90]*4)
        self.assertEqual(self.hold([90]*4, 1.)['valid_trials'], 1)
        self.hold([0]*4)
        self.assertEqual(self.hold([90]*4)['valid_trials'], 2)

    def test_motion_and_short_stop_do_not_submit(self):
        self.assertFalse(self.hold([90]*4, speed=6)['ready'])
        self.assertFalse(self.hold([90]*4, duration=.1)['ready'])

    def test_changing_a_single_dial_starts_another_stability_window(self):
        self.assertEqual(self.hold([90]*4)['correct_count'], 2)
        self.assertFalse(self.hold([90, 180, 90, 90], .1)['ready'])
        result = self.hold([90, 180, 90, 90])
        self.assertEqual(result['correct_count'], 3); self.assertEqual(result['valid_trials'], 2)

    def test_gap_does_not_certify_unobserved_stability(self):
        self.hold([90]*4, .1)
        self.time += 2
        result = self.game.update(self.time, [math.pi/2]*4, [0]*4)
        self.assertFalse(result['ready'])
        with self.assertRaises(ValueError):
            self.game.update(0, [0]*4, [0]*4)

    def test_success_latches_without_more_trials_after_release_or_new_input(self):
        result=self.hold([90,180,-90,90]);self.assertTrue(result['won'])
        self.hold([0]*4)
        result=self.hold([180]*4)
        self.assertTrue(result['won']);self.assertEqual(result['valid_trials'],1)

    def test_gap_does_not_count_same_verified_input_twice(self):
        self.hold([180]*4);self.assertEqual(self.game.queries,1)
        self.time+=2
        self.game.update(self.time,[math.pi]*4,[0]*4)
        self.assertEqual(self.hold([180]*4)['valid_trials'],1)
        self.assertEqual(self.game.sampling_gaps,1)


if __name__ == '__main__':
    unittest.main()
