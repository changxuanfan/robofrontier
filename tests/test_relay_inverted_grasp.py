"""A real inverted tabletop failure must expose an accessible recovery face."""
import unittest

import numpy as np

from test_relay_descent_recovery import recorded_world
from robofrontier.suite import relay_oracle_v2 as ref


class RelayInvertedGraspTests(unittest.TestCase):
    def test_original_custody_can_plan_exposed_surface_without_moving_world(self):
        t=recorded_world('relay_inverted_table_grasp.json')
        before=t.data.qpos.copy();ctrl=t.data.ctrl.copy()
        body=t.data.body('payload_02');rotation=body.xmat.reshape(3,3).copy()
        self.assertLess(rotation[2,2],-.9)
        team=ref.choose_team(t,'payload_02',body.xpos.copy(),rotation,[.24,.48],
            exclude=set(t.arms)-{'G','H'},departing=(),search_budget_s=8.)
        self.assertEqual(set(team),{'G','H'})
        self.assertTrue(all(value['rotation'][2,2]<-.8 for value in t.relay_grasp_plan.values()))
        np.testing.assert_array_equal(t.data.qpos,before)
        np.testing.assert_array_equal(t.data.ctrl,ctrl)


if __name__=='__main__':unittest.main()
