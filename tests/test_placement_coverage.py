"""Coverage must not turn supply provenance into reachability evidence."""
from types import SimpleNamespace
import unittest

import numpy as np

from robofrontier.suite.placement_oracle import pickup_coverage


class PickupCoverageTests(unittest.TestCase):
    def trial(self, reachable):
        body = SimpleNamespace(xpos=np.array([0., 0., .83]), xmat=np.eye(3))
        return SimpleNamespace(
            env={'objects': [{'id': 'cube', 'source': 'A'}]},
            data=SimpleNamespace(body=lambda _: body),
            task={'family': 'mosaic', 'arms': {
                robot: {'base_position_m': [0., 0., .8]} for robot in ('A', 'B')}},
            arms={'A': object(), 'B': object()},
            home_q={'A': np.zeros(7), 'B': np.zeros(7)},
            screen_ik=lambda robot, *args, **kwargs:
                np.zeros(7) if robot in reachable else None,
        )

    def test_unreachable_source_is_not_a_pickup_candidate(self):
        self.assertEqual(pickup_coverage(self.trial({'B'})), {'cube': ['B']})

    def test_no_reachable_robot_remains_an_explicit_coverage_gap(self):
        self.assertEqual(pickup_coverage(self.trial(set())), {'cube': []})

    def test_source_is_retained_when_it_passes_the_same_screen(self):
        self.assertEqual(pickup_coverage(self.trial({'A'})), {'cube': ['A']})


if __name__ == '__main__':
    unittest.main()
