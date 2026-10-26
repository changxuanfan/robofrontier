import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from robofrontier.suite.transport_recovery import EngineeringRecovery


class EngineeringRecoveryTests(unittest.TestCase):
    def test_automatic_retry_defers_without_discarding_busy_route_actors(self):
        with TemporaryDirectory() as tmp:
            handler=EngineeringRecovery(tmp)
            old=dict(status='failed',goal=dict(goal_id='slot',board='board'),object='item',robots={'A','B'})
            runtime=SimpleNamespace(jobs={'old':old},leases={'A':'old','B':'other'},recover=Mock(return_value=True))
            t=SimpleNamespace(arms={'A':SimpleNamespace(grasp_force=50.),'B':SimpleNamespace(grasp_force=None)},
                data=SimpleNamespace(body=lambda _:SimpleNamespace(xpos=[0.,0.,.8])),
                stations={r:dict(base_position_m=[0.,0.,.8]) for r in 'AB'},
                monitor=SimpleNamespace(result=lambda:dict(assignments={})),finger_contacts=lambda r,n:r=='A')
            with patch('robofrontier.suite.transport_recovery.goal_coverage',return_value=['B']),patch('robofrontier.suite.transport_recovery.candidate_graph',return_value={'A':['B'],'B':['A']}):
                with self.assertRaisesRegex(ValueError,'available.*route'):handler._retry(t,runtime,'old',{})
                runtime.recover.assert_not_called();self.assertEqual(old['robots'],{'A','B'})
                runtime.leases.pop('B');handler._retry(t,runtime,'old',{})
            self.assertEqual(runtime.recover.call_args.args[2],{'A','B'})

    def test_idle_callback_can_freeze_without_advancing_waiting_physics(self):
        class Frozen(Exception):pass
        with TemporaryDirectory() as tmp:
            freeze=Mock(side_effect=Frozen())
            handler=EngineeringRecovery(tmp,on_idle=freeze);handler.poll=Mock(return_value=False)
            t=SimpleNamespace(event=Mock(),snapshot=Mock());runtime=SimpleNamespace(active=False,step=Mock())
            with self.assertRaises(Frozen):handler(t,runtime,[],[])
            freeze.assert_called_once_with(t,runtime);runtime.step.assert_not_called()

    def test_lost_inventory_stops_before_any_recovery_admission(self):
        from robofrontier.suite.transport_inventory import UnreachableTransportInventory
        with TemporaryDirectory() as tmp:
            handler=EngineeringRecovery(tmp,inventory_check=lambda _: [dict(object='lost',position_m=[44,0,.03])])
            handler.observation=Mock();t=SimpleNamespace(motion=None,event=Mock());runtime=SimpleNamespace(recover=Mock())
            with self.assertRaises(UnreachableTransportInventory):handler.poll(t,runtime,[],[],force=True)
            handler.observation.assert_not_called();runtime.recover.assert_not_called()

    def test_frontier_contains_one_coherent_full_joint_snapshot(self):
        import time
        import numpy as np
        with TemporaryDirectory() as tmp:
            handler=EngineeringRecovery(tmp);q=np.array([1.,.030,.028]);v=np.array([0.,.001,-.001])
            t=SimpleNamespace(task=dict(scene_sha256='scene'),time=3.2,started=time.monotonic(),arms={},held={},env=dict(objects=[]),
                data=SimpleNamespace(qpos=q,qvel=v),monitor=SimpleNamespace(result=lambda:dict(assignments={})))
            f=handler.observation(t,SimpleNamespace(jobs={},leases={},running={}))
            self.assertEqual(f['state_qpos'],q.tolist());self.assertEqual(f['state_qvel'],v.tolist())
            self.assertEqual(f['sim_time_s'],3.2);np.testing.assert_array_equal(q,[1.,.030,.028])

    def test_no_free_robot_does_not_admit_an_empty_recovery(self):
        with TemporaryDirectory() as tmp:
            handler=EngineeringRecovery(tmp)
            old=dict(status='failed',goal=dict(goal_id='slot',board='board'),object='item',robots={'A'})
            runtime=SimpleNamespace(jobs={'old':old},leases={'A':'other'},recover=Mock(return_value=True))
            t=SimpleNamespace(arms={'A':None},data=SimpleNamespace(body=lambda _:SimpleNamespace(xpos=[0,0,0])))
            with self.assertRaisesRegex(ValueError,'No available recovery robot'):handler._retry(t,runtime,'old',{})
            runtime.recover.assert_not_called()

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
            t=SimpleNamespace(arms={'A':SimpleNamespace(grasp_force=None)},held={},execute=Mock())
            handler._retry(t,runtime,'old',dict(robots=['A'],resume_goal=False,
                steps=[dict(robot='A',operation='gripper',parameters=dict(mode='move',width_m=.072))]))
            with patch('robofrontier.suite.transport_recovery.execute_job') as execute_goal:
                with self.assertRaises(PreparationComplete):runtime.recover.call_args.args[3]()
                execute_goal.assert_not_called()
            t.execute.assert_called_once()

    def test_preparation_clears_only_a_released_contact_free_carrier_hint(self):
        from robofrontier.suite.transport_runtime import PreparationComplete
        with TemporaryDirectory() as tmp:
            handler=EngineeringRecovery(tmp)
            old=dict(status='failed',goal=dict(goal_id='slot'),object='item',robots=set('ABC'))
            runtime=SimpleNamespace(jobs={'old':old},leases={r:'old' for r in 'ABC'},recover=Mock(return_value=True))
            t=SimpleNamespace(arms={r:SimpleNamespace(grasp_force=50. if r=='B' else None) for r in 'ABC'},
                held={r:'item' for r in 'ABC'},event=Mock())
            handler._retry(t,runtime,'old',dict(robots=list('ABC'),resume_goal=False))
            with patch('robofrontier.suite.transport_recovery._robot_payload_contacts',side_effect=lambda _,r,n:r=='C'):
                with self.assertRaises(PreparationComplete):runtime.recover.call_args.args[3]()
            self.assertEqual(t.held,{'B':'item','C':'item'})

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
