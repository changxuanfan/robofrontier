import unittest
from types import SimpleNamespace

import mujoco
import numpy as np

from robofrontier.suite.relay_observation_state import native_kinematic_state


class NativeObservationTests(unittest.TestCase):
    def setUp(self):
        model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <body pos="0 0 1"><freejoint/><geom type="sphere" size=".04"/></body>
          <body pos="1 0 1"><joint type="hinge"/><geom type="box" size=".1 .02 .02"/></body>
        </worldbody></mujoco>''')
        data=mujoco.MjData(model);mujoco.mj_forward(model,data)
        self.trial=SimpleNamespace(model=model,data=data)

    def test_capture_is_read_only(self):
        data=self.trial.data;before=data.qpos.copy()
        state=native_kinematic_state(self.trial)
        np.testing.assert_array_equal(data.qpos,before)
        state['qpos'][0]=99
        np.testing.assert_array_equal(data.qpos,before)

    def test_free_body_uses_same_cached_pose_as_published_view(self):
        data=self.trial.data;published=data.xpos[1].copy()
        # mj_step integrates joints after computing cached Cartesian poses.
        data.qpos[:3]+=[.01,.02,.03]
        state=native_kinematic_state(self.trial)
        np.testing.assert_array_equal(state['qpos'][:3],published)
        np.testing.assert_allclose(data.qpos[:3],published+[.01,.02,.03])

    def test_other_joint_evidence_and_timestamp_are_retained(self):
        data=self.trial.data;data.qpos[7]=.37;data.time=12.5
        state=native_kinematic_state(self.trial)
        self.assertEqual(state['qpos'][7],.37)
        self.assertEqual(state['sim_time_s'],12.5)
        self.assertEqual(state['version'],1)


if __name__=='__main__':unittest.main()
