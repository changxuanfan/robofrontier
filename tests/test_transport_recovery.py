import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from robofrontier.suite.transport_recovery import EngineeringRecovery


class EngineeringRecoveryTests(unittest.TestCase):
    def test_explicit_scope_keeps_current_carrier_without_reacquiring_busy_old_peers(self):
        with TemporaryDirectory() as tmp:
            handler=EngineeringRecovery(tmp)
            old=dict(status='failed',goal=dict(goal_id='slot'),object='item',robots={'J','K','L'})
            runtime=SimpleNamespace(jobs={'old':old},leases={'L':'old','K':'another'},recover=Mock(return_value=True))
            t=SimpleNamespace(arms={r:None for r in 'JKL'})
            handler._retry(t,runtime,'old',dict(robots=['J','L']))
            self.assertEqual(runtime.recover.call_args.args[2],{'J','L'})

    def test_preparation_returns_to_a_recoverable_frontier_without_resuming_goal(self):
        from robofrontier.suite.transport_runtime import PreparationComplete
        with TemporaryDirectory() as tmp:
            handler=EngineeringRecovery(tmp)
            old=dict(status='failed',goal=dict(goal_id='slot'),object='item',robots={'A'})
            runtime=SimpleNamespace(jobs={'old':old},leases={'A':'old'},recover=Mock(return_value=True))
            t=SimpleNamespace(arms={'A':None},execute=Mock())
            handler._retry(t,runtime,'old',dict(robots=['A'],resume_goal=False,
                steps=[dict(robot='A',operation='gripper',parameters=dict(mode='move',width_m=.072))]))
            with patch('robofrontier.suite.transport_recovery.execute_job') as execute_goal:
                with self.assertRaises(PreparationComplete):runtime.recover.call_args.args[3]()
                execute_goal.assert_not_called()
            t.execute.assert_called_once()

    def test_further_real_progress_enables_a_second_current_state_retry(self):
        with TemporaryDirectory() as tmp:
            handler=EngineeringRecovery(tmp)
            state=dict(failed_jobs={'first':dict(goal=dict(goal_id='remaining'))},accepted_goals={})
            handler.observation=Mock(side_effect=lambda *_:state);handler._retry=Mock(return_value='recovery')
            t=SimpleNamespace(time=0.,event=Mock());runtime=SimpleNamespace()
            handler.poll(t,runtime,[],[],force=True)
            state['accepted_goals']={'one':'item1'}
            handler.poll(t,runtime,[],[],force=True)
            self.assertEqual(handler._retry.call_count,1)
            state['failed_jobs']={'second':dict(goal=dict(goal_id='remaining'))}
            handler.poll(t,runtime,[],[],force=True)
            # Static waiting must not reissue a failed action or spin retries.
            handler.poll(t,runtime,[],[],force=True)
            self.assertEqual(handler._retry.call_count,1)
            state['accepted_goals']['two']='item2'
            handler.poll(t,runtime,[],[],force=True)
            self.assertEqual(handler._retry.call_count,2)

    def test_foreign_or_state_edit_input_never_reaches_execution(self):
        for request in (dict(id='x',job='bad',scene_sha256='other'),
                        dict(id='x',job='bad',scene_sha256='correct',qpos=[0])):
            with self.subTest(request=request),TemporaryDirectory() as tmp:
                handler=EngineeringRecovery(tmp)
                handler.observation=Mock(return_value=dict(failed_jobs={},accepted_goals={}))
                (handler.path/'inbox/x.json').write_text(json.dumps(request))
                t=SimpleNamespace(time=0.,task={'scene_sha256':'correct'},event=Mock())
                runtime=SimpleNamespace(recover=Mock())
                self.assertFalse(handler.poll(t,runtime,[],[],force=True))
                runtime.recover.assert_not_called();self.assertTrue((handler.path/'rejected/x.json').exists())

    def test_frontier_polling_is_bounded_not_every_physics_step(self):
        with TemporaryDirectory() as tmp:
            handler=EngineeringRecovery(tmp)
            handler.observation=Mock(return_value=dict(failed_jobs={},accepted_goals={}))
            t=SimpleNamespace(time=0.,task={'scene_sha256':'correct'})
            handler.poll(t,SimpleNamespace(),[],[],force=True)
            for _ in range(100):handler.poll(t,SimpleNamespace(),[],[])
            self.assertEqual(handler.observation.call_count,1)


if __name__=='__main__':unittest.main()
