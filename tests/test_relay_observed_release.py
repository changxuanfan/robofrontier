"""Opening overshoot may be physically clear without a successful motor call."""
import unittest
from unittest.mock import Mock, patch

from tests.test_relay_descent_recovery import recorded_world
from robofrontier.suite import relay_oracle_v2 as ref


class ObservedReleaseTests(unittest.TestCase):
    def trial(self):
        t=recorded_world();arm=t.arms['G'];arm.grasp_force=None;arm.job=None
        q=t.data.qpos[arm.qids].copy();t.relay_release_plan={'G':[q,q+.01]}
        t.held={'G':'payload_02'};t.wait=Mock();t.event=Mock();t.pose_clear=Mock(return_value=True)
        t.monitor.relay_v2.states['payload_02']['dropped']=False
        feedback=dict(status='failed',reason='Requested aperture exceeded',direction='opening',
                      target_width_m=.075,actual_width_m=.07997512744578888,target_width_reached=False)
        t.execute=Mock(side_effect=[{'G':feedback},{'G':dict(status='completed')}])
        return t,feedback

    def test_observed_clear_wider_opening_keeps_failed_feedback_and_rechecks_exit(self):
        t,feedback=self.trial()
        with patch.object(ref,'arm_payload_contact',return_value=False):
            ref.release(t,'payload_02',['G'])
        self.assertNotIn('G',t.held)
        self.assertEqual(feedback['status'],'failed')
        self.assertFalse(feedback['target_width_reached'])
        self.assertEqual(t.execute.call_args_list[1].args[0],'send-clear-selected')
        t.pose_clear.assert_called()
        self.assertTrue(any(c.args[0]=='relay_release_observed_wider' for c in t.event.call_args_list))

    def test_wider_aperture_without_actual_separation_stays_failed(self):
        t,_=self.trial()
        with patch.object(ref,'arm_payload_contact',return_value=True):
            with self.assertRaisesRegex(RuntimeError,'aperture not verified'):
                ref.release(t,'payload_02',['G'])
        self.assertEqual(t.held,{'G':'payload_02'})
        self.assertEqual(t.execute.call_count,1)

    def test_unknown_failure_or_insufficient_aperture_is_not_reclassified(self):
        for change in [dict(reason='Requested opening not reached'),dict(actual_width_m=.06),
                       dict(actual_width_m=float('nan')),dict(direction='closing')]:
            with self.subTest(change=change):
                t,feedback=self.trial();feedback.update(change)
                with patch.object(ref,'arm_payload_contact',return_value=False):
                    with self.assertRaisesRegex(RuntimeError,'aperture not verified'):
                        ref.release(t,'payload_02',['G'])
                self.assertIn('G',t.held)

    def test_force_hold_or_unresolved_job_prevents_known_empty_continuation(self):
        for field,value in [('grasp_force',70.),('job','unresolved-original')]:
            with self.subTest(field=field):
                t,_=self.trial();setattr(t.arms['G'],field,value)
                with patch.object(ref,'arm_payload_contact',return_value=False):
                    with self.assertRaisesRegex(RuntimeError,'aperture not verified'):
                        ref.release(t,'payload_02',['G'])
                self.assertIn('G',t.held)


if __name__=='__main__':unittest.main()
