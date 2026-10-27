import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock


class MosaicRunTests(unittest.TestCase):
    def trial(self, directory):
        return SimpleNamespace(directory=Path(directory),time=0.,held={},actions=[],
            task={'goals':[{'goal_id':f'g{i}'} for i in (1,2,3)]},env={'objects':[]},
            arms={'A':SimpleNamespace(job=None)},mosaic_protected_assignments={},
            monitor=SimpleNamespace(violations=[],result=lambda:{'assignments':{}}),
            event=Mock(),wait=Mock(),close=Mock(),check_stop=Mock())

    def request(self, review, ident, epoch, operation='resume', **fields):
        payload=dict(id=ident,world_id=review.world_id,review_id=epoch,operation=operation,**fields)
        (review.directory/'requests'/f'{ident}.json').write_text(json.dumps(payload))
        return payload

    def test_two_failures_resume_same_trial_without_closing_or_reparking(self):
        from robofrontier.suite.mosaic_run import MosaicReview, run_reference
        with tempfile.TemporaryDirectory() as directory:
            trial=self.trial(directory);accepted={};trial.monitor.result=lambda:{'assignments':dict(accepted)}
            review=MosaicReview(trial,review_wall_s=1,publish=lambda:None)
            for i in (1,2):self.request(review,f'resume-{i}',i)
            seen=[]
            def solver(world,**kwargs):
                self.assertIs(world,trial);seen.append(kwargs['resume'])
                accepted[f'g{len(seen)}']=f'cube{len(seen)}'
                trial.mosaic_protected_assignments=dict(accepted)
                if len(seen)<3:raise RuntimeError('Known stopped placement')
                return {'reference_placed':3}
            result=run_reference(trial,review,solver=solver)
            self.assertEqual(seen,[False,True,True]);self.assertEqual(result['reference_placed'],3)
            self.assertEqual(len(review.history),2);trial.close.assert_not_called()

    def test_productive_reviews_have_no_implicit_elapsed_or_round_limit(self):
        from robofrontier.suite.mosaic_run import MosaicReview,run_reference
        with tempfile.TemporaryDirectory() as directory:
            trial=self.trial(directory);review=MosaicReview(trial,publish=lambda:None)
            self.assertIsNone(review.review_wall_s)
            for i in range(1,23):self.request(review,f'resume-{i}',i)
            calls=[]
            def solver(world,**kwargs):
                calls.append(kwargs['resume'])
                if len(calls)<=22:raise RuntimeError('Known reviewed planning boundary')
                return {'reference_placed':1}
            result=run_reference(trial,review,solver=solver)
            self.assertEqual(len(review.history),22)
            self.assertEqual(result['reference_placed'],1)
            trial.close.assert_not_called()

    def test_unknown_action_does_not_enter_recovery_or_replay(self):
        from robofrontier.suite.mosaic_run import MosaicReview, run_reference
        with tempfile.TemporaryDirectory() as directory:
            trial=self.trial(directory);trial.arms['A'].job='original-id'
            review=MosaicReview(trial,publish=lambda:None)
            with self.assertRaisesRegex(RuntimeError,'Unknown'):
                run_reference(trial,review,solver=Mock(side_effect=RuntimeError('Unknown execution')))
            self.assertEqual(review.history,[]);trial.wait.assert_not_called()

    def test_known_idle_logic_error_keeps_the_same_owner_available_for_review(self):
        from robofrontier.suite.mosaic_run import MosaicReview,run_reference
        with tempfile.TemporaryDirectory() as directory:
            trial=self.trial(directory);review=MosaicReview(trial,publish=lambda:None)
            self.request(review,'repair-logic',1)
            solver=Mock(side_effect=[KeyError(frozenset(('A','C'))),{'reference_placed':1}])
            result=run_reference(trial,review,solver=solver)
            self.assertEqual(result['reference_placed'],1);self.assertEqual(len(review.history),1)
            self.assertTrue(review.history[0]['error'].startswith('KeyError:'))
            self.assertTrue(solver.call_args.kwargs['resume']);trial.close.assert_not_called()

    def test_violations_are_retained_and_never_resume(self):
        from robofrontier.suite.mosaic_run import MosaicReview, run_reference
        with tempfile.TemporaryDirectory() as directory:
            trial=self.trial(directory);trial.monitor.violations.append({'kind':'collision'})
            review=MosaicReview(trial,publish=lambda:None)
            with self.assertRaises(RuntimeError):run_reference(trial,review,solver=Mock(side_effect=RuntimeError('collision')))
            self.assertEqual(review.history,[]);self.assertTrue(trial.monitor.violations)

    def test_duplicate_request_returns_result_without_reexecuting(self):
        from robofrontier.suite.mosaic_run import MosaicReview
        with tempfile.TemporaryDirectory() as directory:
            trial=self.trial(directory);trial.execute=Mock(return_value={'A':{'status':'completed'}})
            review=MosaicReview(trial,publish=lambda:None);review.epoch=1
            payload=self.request(review,'move-1',1,'commands',batches=[{'A':['move',{'joint_rad':[0.]*7}]}])
            review.submit(payload);review.submit(payload)
            self.assertEqual(trial.execute.call_count,1)
            changed=dict(payload,batches=[{'A':['gripper',{'mode':'move','width_m':.08}]}])
            with self.assertRaisesRegex(ValueError,'reused'):review.submit(changed)

    def test_wrong_world_or_stale_review_cannot_execute(self):
        from robofrontier.suite.mosaic_run import MosaicReview
        with tempfile.TemporaryDirectory() as directory:
            trial=self.trial(directory);trial.execute=Mock();review=MosaicReview(trial,publish=lambda:None);review.epoch=2
            payload=self.request(review,'old',1,'commands',batches=[{'A':['move',{}]}])
            with self.assertRaises(ValueError):review.submit(payload)
            payload.update(review_id=2,world_id='another-world')
            with self.assertRaises(ValueError):review.submit(payload)
            trial.execute.assert_not_called()

    def test_failed_repair_stops_sequence_and_preserves_feedback(self):
        from robofrontier.suite.mosaic_run import MosaicReview
        with tempfile.TemporaryDirectory() as directory:
            trial=self.trial(directory);feedback={'A':{'status':'failed','call_id':'real-failure'}}
            trial.execute=Mock(return_value=feedback);review=MosaicReview(trial,publish=lambda:None);review.epoch=1
            payload=self.request(review,'repair',1,'commands',batches=[{'A':['move',{}]},{'A':['move',{}]}])
            result=review.submit(payload)
            self.assertEqual(result['status'],'failed');self.assertEqual(result['feedback'],[feedback])
            self.assertEqual(trial.execute.call_count,1)

    def test_held_payload_cannot_be_silently_rematched(self):
        from robofrontier.suite.mosaic_run import MosaicReview
        with tempfile.TemporaryDirectory() as directory:
            trial=self.trial(directory);trial.held={'A':'cube'}
            review=MosaicReview(trial,publish=lambda:None);review.epoch=1
            with self.assertRaisesRegex(ValueError,'held'):
                review.submit(self.request(review,'resume',1))

    def test_missing_hold_bookkeeping_does_not_release_a_force_held_gripper_for_rematching(self):
        from robofrontier.suite.mosaic_run import MosaicReview
        with tempfile.TemporaryDirectory() as directory:
            trial=self.trial(directory);trial.arms['A'].grasp_force=50.
            review=MosaicReview(trial,publish=lambda:None);review.epoch=1
            with self.assertRaisesRegex(ValueError,'force-held'):
                review.submit(self.request(review,'resume',1))


if __name__=='__main__':unittest.main()
