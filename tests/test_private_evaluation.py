"""Private evaluation evidence is saved at close, never sent to policy roles."""
import json
import unittest
import test_centralized as fixture


class PrivateEvaluationTests(unittest.TestCase):
    def setUp(self):
        self.f=fixture.CentralizedTests();self.f.setUp();self.f.start()
        self.service=self.f.service;self.run=self.f.run
        self.evidence={'samples':[{'time_s':1.,'objects':{'PRIVATE_CANARY':{'position_m':[1,2,3]}}}]}
        self.service.world.evaluation_evidence=lambda:self.evidence
    def tearDown(self):self.f.tearDown()

    def test_normal_finish_saves_private_window_without_exposing_it(self):
        self.assertEqual(self.service.store.get('start_sim_time'),0.)
        self.service.world.time=12.5
        result=self.f.call('Controller','finish')['result']
        self.assertTrue(result['ended']);self.assertNotIn('PRIVATE_CANARY',json.dumps(result))
        saved=json.loads((self.run/'private/outputs/evaluation-window.json').read_text())
        self.assertEqual(saved,self.evidence)
        self.service.close()
        self.assertFalse((self.run/'private/outputs/interruption-evaluation.json').exists())

    def test_interruption_cannot_be_misreported_as_success(self):
        self.service.world.evaluate=lambda:{'all_goals_met':True,'scope':'synthetic geometry only'}
        self.service.close()
        result=json.loads((self.run/'private/outputs/interruption-evaluation.json').read_text())
        self.assertFalse(result['task_success']);self.assertTrue(result['evaluation']['all_goals_met'])
        self.assertEqual(json.loads((self.run/'private/outputs/interruption-evaluation-window.json').read_text()),self.evidence)
        self.assertFalse((self.run/'private/outputs/evaluation.json').exists())

    def test_failed_optional_snapshot_does_not_prevent_owner_cleanup(self):
        def unavailable():raise ValueError('synthetic evaluator unavailable')
        self.service.world.evaluate=unavailable
        self.service.close();self.assertIsNone(self.service.store)
        # Opening the same physical run remains prohibited, independent of evidence availability.
        self.assertTrue((self.run/'private/outputs/runtime.json').exists())
