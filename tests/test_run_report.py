import unittest
from export_run_report import occupancy


class OccupancyTests(unittest.TestCase):
    def test_overlap_uses_terminal_action_intervals_and_clips_to_trial(self):
        result=occupancy([(-2,2),(1,4),(4,9)],0,6)
        self.assertEqual(result['seconds_by_active_arm_count'],{0:0,1:5,2:1})
        self.assertEqual(result['peak'],2)
        self.assertEqual(result['overlap_seconds'],1)
