"""Joint target references follow the segment that geometry checks sample."""
import unittest
import numpy as np
from test_motion_check import robot_fixture
from robofrontier.task_config import load_harness
from robofrontier.motion_check import JOINT_ACCEL_LIMIT

class JointCoordinationTests(unittest.TestCase):
    def test_unequal_joint_travels_share_progress_and_respect_acceleration(self):
        w=robot_fixture();w.task={};arm=w.arms['A'];start=arm.qref.copy();delta=np.array([.5,.08,-.15,-.12,.10,-.13,.35]);target=start+delta
        command=arm.validate('move',dict(joint_rad=target.tolist(),joint_speed_rad_s=.5),load_harness(),np.full(3,-3.),np.full(3,3.))
        arm.start('segment',command,w.time);previous=arm.velocity.copy();observed=0
        for _ in range(160):
            w.step();progress=(arm.qref-start)/delta
            self.assertLessEqual(max(abs(arm.velocity)),.5+1e-8)
            self.assertLessEqual(max(abs(arm.velocity-previous)),JOINT_ACCEL_LIMIT*.02+1e-8);previous=arm.velocity.copy()
            if .05<max(progress)<.90:
                self.assertLess(np.ptp(progress),.003);observed+=1
                self.assertLess(max(abs(w.data.qpos[arm.qids]-arm.qref)),.012)
            if max(progress)>.98:break
        self.assertGreater(observed,5)

if __name__=='__main__':unittest.main()
