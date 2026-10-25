"""Allowed final-region cargo contacts must not become extra task gates."""
import unittest
from unittest.mock import Mock,patch
import numpy as np
import json
from tests.test_relay_descent_recovery import recorded_world
from robofrontier.suite import relay_oracle_v2 as ref


class RelayTerminalContactTests(unittest.TestCase):
    def world(self):
        t=recorded_world('relay_terminal_opening_corridor.json')
        t.task['completion']['allow_goal_payload_contact']=True
        t.execute=Mock(return_value={a:{'status':'completed'} for a in ['G','H']})
        t.move_free=Mock();t.pose_command=Mock(return_value=('move',{}))
        return t

    def test_allowed_landing_opens_before_requiring_a_contact_free_exit_plan(self):
        t=self.world()
        with patch.object(ref,'final_support',return_value=True),patch.object(ref,'donor_exit_routes',return_value=None) as forecast,patch.object(ref,'withdrawal_target',return_value=(np.array([1.5,0,1.1]),np.eye(3))):
            ref.release(t,'payload_01',['G','H'],terminal=True)
        forecast.assert_not_called()
        self.assertEqual(t.execute.call_args_list[0].args[0],'relay-release')
        self.assertAlmostEqual(t.execute.call_args_list[0].args[1]['G'][1]['width_m'],.065)
        self.assertFalse(t.held)

    def test_failed_actual_opening_remains_failed(self):
        t=self.world();t.execute.return_value={'G':{'status':'completed'},'H':{'status':'failed','reason':'blocked'}}
        with patch.object(ref,'final_support',return_value=True),patch.object(ref,'donor_exit_routes',return_value=None),patch.object(ref,'withdrawal_target') as withdrawal:
            with self.assertRaisesRegex(RuntimeError,'aperture not verified H'):ref.release(t,'payload_01',['G','H'],terminal=True)
        withdrawal.assert_not_called();self.assertEqual(t.held,{'H':'payload_01'})

    def test_goal_contact_permission_does_not_release_an_airborne_load(self):
        t=self.world()
        with patch.object(ref,'final_support',return_value=False),patch.object(ref,'donor_exit_routes',return_value={}):
            with self.assertRaisesRegex(RuntimeError,'supported in-goal landing'):ref.release(t,'payload_01',['G','H'],terminal=True)
        t.execute.assert_not_called()

    def test_all_nine_public_tasks_declare_the_same_user_contact_contract(self):
        from robofrontier.suite.v2 import cases,directory,public_task,validate_task
        relay=[c for c in cases() if c['family']=='relay'];self.assertEqual(len(relay),9)
        for case in relay:
            task=json.loads((directory(case)/'task.json').read_text());validate_task(task)
            policy=public_task(task)['completion']
            self.assertTrue(policy['allow_goal_payload_contact'])
            self.assertTrue(policy['arbitrary_goal_placement'])
            self.assertTrue(policy['required_airborne_handover'])


if __name__=='__main__':unittest.main()
