"""A fixed-chain pose bound does not depend on the IK starting seed."""
import unittest
from unittest.mock import patch
import numpy as np

from tests.test_relay_descent_recovery import recorded_world
from robofrontier.suite import relay_oracle_v2 as ref


class ReceiverPosePruningTests(unittest.TestCase):
    def test_unreachable_pose_is_not_retested_for_every_seed(self):
        t=recorded_world();seen=set()
        def reject(robot,point,rotation):
            key=(robot,np.asarray(point).tobytes(),np.asarray(rotation).tobytes())
            self.assertNotIn(key,seen,'Identical seed-independent bound was repeated')
            seen.add(key);return False
        body=t.data.body('payload_02')
        with patch.object(ref,'potentially_reachable',side_effect=lambda world,*a:reject(*a)):
            with self.assertRaisesRegex(RuntimeError,'No sampled'):
                ref.choose_team(t,'payload_02',body.xpos.copy(),body.xmat.reshape(3,3).copy(),.44,search_budget_s=10.)
        self.assertTrue(seen)


if __name__=='__main__':unittest.main()
