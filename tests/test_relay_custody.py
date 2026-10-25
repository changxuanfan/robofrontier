"""Flexible custody requires verified new support before any outgoing release."""
import unittest
from unittest.mock import patch

from tests.test_relay_descent_recovery import recorded_world
from robofrontier.suite import relay_oracle_v2 as ref
from unittest.mock import Mock
import numpy as np


class RelayCustodyTests(unittest.TestCase):
    def setUp(self):
        self.t=recorded_world();self.relay=self.t.monitor.relay_v2
        self.relay.states={'payload_02':self.relay.states['payload_02']}
        self.support={'A','B'}
        self.patches=[patch.object(self.t.monitor,'bilateral',side_effect=lambda *a,**k:set(self.support)),
            patch.object(self.t.monitor,'bottom',return_value=1.),
            patch.object(self.relay,'inside',return_value=False),
            patch.object(self.relay,'environment_contact',return_value=False)]
        for p in self.patches:p.start();self.addCleanup(p.stop)
        self.sample(.1,{'A','B'})

    def sample(self,stamp,support):
        self.support=support;self.t.data.time=stamp;self.relay.sample()

    def test_one_holder_can_remain_during_qualified_single_side_exchange(self):
        self.sample(.2,{'A','B','C'});self.sample(.41,{'A','B','C'});self.sample(.42,{'A','C'})
        state=self.relay.states['payload_02']
        self.assertEqual(state['custody'],{'A','C'})
        self.assertEqual(state['transitions'][-1]['retained'],['A'])
        self.assertEqual(state['transitions'][-1]['released'],['B'])
        self.assertEqual(state['transitions'][-1]['joined'],['C'])
        self.assertEqual(self.t.monitor.violations,[])

    def test_whole_pair_exchange_still_requires_two_qualified_receivers(self):
        self.sample(.2,{'A','B','C','D'});self.sample(.41,{'A','B','C','D'});self.sample(.42,{'C','D'})
        self.assertEqual(self.relay.states['payload_02']['custody'],{'C','D'})
        self.assertEqual(self.t.monitor.violations,[])

    def test_premature_single_side_release_is_not_a_valid_handover(self):
        self.sample(.2,{'A','B','C'});self.sample(.25,{'A','C'})
        self.assertEqual(self.relay.states['payload_02']['custody'],{'A','B'})
        self.assertTrue(any(v['kind']=='unverified_handover' for v in self.t.monitor.violations))

    def test_receiver_losing_contact_must_requalify(self):
        self.sample(.2,{'A','B','C'});self.sample(.41,{'A','B','C'})
        self.sample(.45,{'A','B'});self.sample(.46,{'A','B','C'});self.sample(.47,{'A','C'})
        self.assertEqual(self.relay.states['payload_02']['custody'],{'A','B'})
        self.assertTrue(any(v['kind']=='unverified_handover' for v in self.t.monitor.violations))

    def test_retained_holder_is_neither_opened_nor_reapproached(self):
        t=self.t;t.execute=Mock();t.move=Mock();t.wait=Mock()
        t.relay_grasp_plan={}
        for robot in ['G','H']:
            arm=t.arms[robot];q=t.data.qpos[arm.qids].copy()
            t.relay_grasp_plan[robot]=dict(retained=robot=='G',rotation=t.data.site_xmat[arm.site].reshape(3,3).copy(),point_delta=np.zeros(3),arrival_route=[q],seed=q)
        with patch.object(t.monitor,'bilateral',return_value={'G','H'}):ref.grasp(t,'payload_02',['G','H'],(.44,.32))
        for call in t.execute.call_args_list:self.assertNotIn('G',call.args[1])
        self.assertEqual([call.args[0] for call in t.move.call_args_list],['H'])

    def test_final_placement_requires_a_real_prior_airborne_handover(self):
        state=self.relay.states['payload_02'];state.update(airborne=True,landed=True,custody={'A','B'})
        self.t.task['completion']['required_airborne_handover']=True
        with patch.object(self.relay,'inside',return_value=True),patch.object(self.relay,'environment_contact',return_value=True),patch.object(self.t.monitor,'released',return_value=True):
            self.sample(2.,set());self.sample(3.2,set())
            self.assertFalse(self.t.monitor.result_cache['task_success'])
            state['transitions'].append(dict(sim_time_s=1.,senders=['C','D'],receivers=['A','B']))
            self.sample(3.3,set());self.sample(4.4,set())
            self.assertTrue(self.t.monitor.result_cache['task_success'])


if __name__=='__main__':unittest.main()
