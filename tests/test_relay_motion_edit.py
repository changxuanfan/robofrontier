import unittest
import numpy as np
from robofrontier.relay_motion_edit import (
    keep_intervals, select_source_rows, retained_timeline, video_rows,
)


class MotionEditTests(unittest.TestCase):
    def test_cut_excludes_waiting_without_bridging_it(self):
        analysis = np.arange(101) / 20
        keep = (analysis < 1) | (analysis > 4)
        intervals = keep_intervals(analysis, keep, 20)
        raw = np.arange(301) / 60
        selected = raw[select_source_rows(raw, intervals)]
        self.assertFalse(np.any((selected >= 1) & (selected <= 4)))
        edited, duration, segments = retained_timeline(selected, intervals, 60)
        self.assertLess(sum(duration), 2.1)
        self.assertEqual(set(segments), {0, 1})
        self.assertTrue(np.all(np.diff(edited) > 0))

    def test_actual_twentyfold_clock_and_context(self):
        times = np.arange(1200) / 60
        edited, duration, _ = retained_timeline(times, [[0, 20]], 60)
        rows, clock = video_rows(edited, duration, fps=30, speed=20)
        self.assertEqual(len(rows), 32)  # 20 source seconds -> 1 video second
        self.assertEqual((rows[0], rows[-1]), (-1, -2))
        np.testing.assert_allclose(np.diff(clock), 20 / 30)
        np.testing.assert_allclose(edited[rows[1:-1]], clock, atol=1/60)

    def test_irregular_source_clock_and_short_segment_remain(self):
        t = np.array([0., .016, .034, 10., 10.016])
        edit, duration, seg = retained_timeline(t, [[0, .05], [10, 10.03]], 60)
        self.assertAlmostEqual(sum(duration), .08)
        self.assertEqual(list(seg), [0, 0, 0, 1, 1])
        self.assertLess(edit[-1], .08)

    def test_invalid_rows_rejected(self):
        with self.assertRaises(ValueError):
            retained_timeline([0, 5], [[0, 1]], 60)
        with self.assertRaises(ValueError):
            keep_intervals([0, 0], [True, True], 20)


if __name__ == '__main__':
    unittest.main()
