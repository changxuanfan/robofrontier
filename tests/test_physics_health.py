import unittest
import mujoco
from robofrontier.physics_health import require_valid_step
class PhysicsHealthTests(unittest.TestCase):
 def setUp(self):
  self.m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><freejoint/><geom type="sphere" size=".01" mass="1"/></body></worldbody></mujoco>');self.d=mujoco.MjData(self.m)
 def test_valid_step(self):
  before=float(self.d.time);mujoco.mj_step(self.m,self.d);require_valid_step(self.d,before)
 def test_time_reset_rejected(self):
  with self.assertRaisesRegex(RuntimeError,'Numerical'):require_valid_step(self.d,1.)
 def test_numeric_warning_rejected_even_with_finite_state(self):
  self.d.time=1.;self.d.warning[mujoco.mjtWarning.mjWARN_BADQACC].number=1
  with self.assertRaisesRegex(RuntimeError,'BADQACC'):require_valid_step(self.d,0.)
if __name__=='__main__':unittest.main()
