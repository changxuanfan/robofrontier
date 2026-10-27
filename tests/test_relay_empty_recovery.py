"""A physically open, separated donor can move without a false carried object."""
import unittest
from unittest.mock import Mock,patch
import numpy as np
from tests.test_relay_descent_recovery import recorded_world
from robofrontier.suite import relay_oracle_v2 as ref

class EmptyRecoveryTests(unittest.TestCase):
 def trial(self):
  t=recorded_world();a=t.arms['G'];a.job=None;a.grasp_force=None;a.observe=Mock(return_value={'gripper_width_m':.08});t.held={'G':'payload_02'};t.execute=Mock();t.monitor.bilateral=Mock(return_value={'E','F'});t.monitor.relay_v2.states['payload_02']['custody']={'E','F'}
  return t,t.data.qpos[a.qids].copy()
 def test_empty_move_uses_scratch_cache_and_clears_live_cache_only_after_feedback(self):
  t,q=self.trial();before=t.data.qpos.copy()
  def clear(probe,*args,**kwargs):self.assertNotIn('G',probe.held);self.assertIn('G',t.held);return True
  with patch.object(ref,'arm_payload_contact',return_value=False),patch('robofrontier.suite.oracle.Trial.pose_clear',autospec=True,side_effect=clear):ref.move_empty_robot(t,'payload_02','G',q+.01)
  self.assertNotIn('G',t.held);t.execute.assert_called_once();np.testing.assert_array_equal(t.data.qpos,before)
 def test_actual_contact_or_force_control_never_counts_as_empty(self):
  for contact,force in [(True,None),(False,70.)]:
   t,q=self.trial();t.arms['G'].grasp_force=force
   with patch.object(ref,'arm_payload_contact',return_value=contact):
    with self.assertRaisesRegex((ValueError,RuntimeError),'force-free|not separated|still contacts cargo'):
     ref.move_empty_robot(t,'payload_02','G',q)
   t.execute.assert_not_called();self.assertIn('G',t.held)
 def test_current_contact_uses_bounded_withdrawal_instead_of_the_empty_path(self):
  t,q=self.trial()
  with patch.object(ref,'arm_payload_contact',return_value=True),patch('robofrontier.suite.relay_contact_escape.execute_open_contact_withdrawal') as escape:
   ref.move_empty_robot(t,'payload_02','G',q)
  escape.assert_called_once_with(t,'payload_02','G',q,goal_names=ref.terminal_contact_names)
  t.execute.assert_not_called()
 def test_motion_failure_retains_original_cache_and_is_not_reported_as_release(self):
  t,q=self.trial();t.execute.side_effect=RuntimeError('motor original failed')
  with patch.object(ref,'arm_payload_contact',return_value=False),patch('robofrontier.suite.oracle.Trial.pose_clear',return_value=True):
   with self.assertRaisesRegex(RuntimeError,'motor original'):ref.move_empty_robot(t,'payload_02','G',q+.01)
  self.assertIn('G',t.held)
 def test_missing_receiver_support_or_outside_joint_goal_is_rejected(self):
  for support,delta in [({'E'},0.),({'E','F'},100.)]:
   t,q=self.trial();t.monitor.bilateral.return_value=support;t.monitor.relay_v2.environment_contact=Mock(return_value=False)
   with patch.object(ref,'arm_payload_contact',return_value=False):
    with self.assertRaises(ValueError):ref.move_empty_robot(t,'payload_02','G',q+delta)
   t.execute.assert_not_called()
