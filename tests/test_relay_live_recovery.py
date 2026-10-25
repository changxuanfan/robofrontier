import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
import numpy as np
from robofrontier.suite.relay_live_recovery import RelayLiveRecovery

class RelayLiveRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.t=SimpleNamespace(directory=Path(self.tmp.name),time=1.,
            arms={a:SimpleNamespace(job=None) for a in ['G','H']},event=Mock(),close=Mock(),
            data=SimpleNamespace(body=lambda _:SimpleNamespace(xmat=np.eye(3))),
            monitor=SimpleNamespace(violations=[],result=lambda:{'task_success':False},
                bilateral=lambda *a,**k:{'G','H'},relay_v2=SimpleNamespace(states={'p':{'rod_geoms':{1}}})))
        self.recovery=RelayLiveRecovery(self.t,{})
        self.request=dict(id='repair-1',world_id=self.recovery.world_id,op='carry',payload='p',robots=['G','H'],target_m=[1.3,0,1.2],review='finite reviewed move',assisted=True)
    def tearDown(self):self.tmp.cleanup()
    def test_known_rejection_retains_world_and_failed_request_history(self):
        with patch('robofrontier.suite.relay_live_recovery.ref.carry',side_effect=RuntimeError('guard rejection')):
            result=self.recovery.process(self.request)
        self.assertEqual(result['status'],'failed');self.assertIn('guard rejection',result['error'])
        self.t.close.assert_not_called();self.assertTrue(self.recovery.assisted)
        self.assertTrue((self.recovery.directory/'accepted/repair-1.json').exists())
    def test_duplicate_id_returns_feedback_without_repeating_motion(self):
        with patch('robofrontier.suite.relay_live_recovery.ref.carry') as carry:
            first=self.recovery.process(self.request);second=self.recovery.process(self.request)
        self.assertEqual(first,second);self.assertEqual(carry.call_count,1)
        with self.assertRaises(ValueError):self.recovery.process(dict(self.request,target_m=[1.5,0,1.2]))
    def test_unknown_original_execution_cannot_be_bypassed_by_new_id(self):
        self.t.arms['G'].job='original-unresolved'
        with self.assertRaisesRegex(ValueError,'Original execution unresolved'):self.recovery.process(self.request)
        self.t.close.assert_not_called()
    def test_state_edits_and_wrong_world_are_rejected(self):
        for request in [dict(self.request,op='reset'),dict(self.request,world_id='old-world')]:
            with self.assertRaises(ValueError):self.recovery.process(request)
        self.t.close.assert_not_called()

if __name__=='__main__':unittest.main()
