"""User-authorized low release inside Goal; final stable support remains required."""
import unittest
from unittest.mock import patch
import numpy as np
from tests.test_relay_descent_recovery import recorded_world


class NearGoalReleaseTests(unittest.TestCase):
    def setUp(self):
        self.t=recorded_world();self.relay=self.t.monitor.relay_v2
        self.ident='payload_02';self.relay.states={self.ident:self.relay.states[self.ident]}
        self.state=self.relay.states[self.ident]
        self.state.update(airborne=True,custody={'G','H'},transitions=[dict(senders=['E','F'],receivers=['G','H'])])
        self.t.task['completion'].update(support_via_goal_payload=True,required_airborne_handover=True,
            allow_goal_partial_support=True,allow_near_goal_release=True,max_timber_release_gap_m=.02)
        self.support={'G','H'};self.inside=True;self.contact=False;self.grounded=False;self.gap=.015
        def bottom(geoms):return .8+self.gap
        def grounds(goal_only=True,*args):return {self.ident} if self.grounded or (not goal_only and self.contact) else set()
        patches=[patch.object(self.t.monitor,'bilateral',side_effect=lambda *a,**k:set(self.support)),
            patch.object(self.t.monitor,'bottom',side_effect=bottom),
            patch.object(self.t.monitor,'velocity',return_value=np.zeros(6)),
            patch.object(self.t.monitor,'released',side_effect=lambda *a:not self.support),
            patch.object(self.relay,'inside',side_effect=lambda *a:self.inside),
            patch.object(self.relay,'environment_contact',side_effect=lambda *a:self.contact),
            patch.object(self.relay,'supported_payloads',side_effect=grounds)]
        for p in patches:p.start();self.addCleanup(p.stop)

    def sample(self,stamp,support=None):
        if support is not None:self.support=set(support)
        self.t.data.time=stamp;self.relay.sample()

    def test_pendant_may_touch_inside_goal_while_original_hands_hold_timber(self):
        self.contact=True;self.gap=.3
        self.sample(1.)
        self.assertFalse(self.state['dropped']);self.assertEqual(self.state['drops'],0)
        self.assertFalse(self.t.monitor.result_cache['task_success'])

    def test_low_release_needs_no_prior_table_contact_and_only_completes_after_settling(self):
        self.sample(1.);self.sample(1.02,set());self.sample(1.4)
        self.assertFalse(self.state['dropped'])
        self.assertFalse(self.t.monitor.result_cache['task_success'])
        self.contact=self.grounded=True
        self.sample(1.5);self.sample(3.)
        self.assertTrue(self.t.monitor.result_cache['task_success'])

    def test_high_release_remains_an_unsupported_drop(self):
        self.gap=.3;self.sample(1.);self.sample(1.02,set());self.sample(1.4)
        self.assertTrue(self.state['dropped'])

    def test_leaving_goal_revokes_low_release_permission(self):
        self.sample(1.);self.sample(1.02,set());self.inside=False;self.contact=True
        self.sample(1.4)
        self.assertTrue(self.state['dropped'])

    def test_near_goal_release_does_not_bypass_real_airborne_handover(self):
        self.state['transitions']=[]
        self.sample(1.);self.sample(1.02,set());self.contact=self.grounded=True
        self.sample(1.5);self.sample(3.)
        self.assertFalse(self.t.monitor.result_cache['task_success'])

    def test_legacy_contract_keeps_its_original_partial_contact_semantics(self):
        self.t.task['completion']['allow_goal_partial_support']=False
        self.t.task['completion']['allow_near_goal_release']=False
        self.contact=True;self.gap=.3;self.sample(1.)
        self.assertTrue(self.state['dropped'])


if __name__=='__main__':unittest.main()
