import unittest
from types import SimpleNamespace
from unittest.mock import patch
from pathlib import Path
import tempfile
import numpy as np
from robofrontier.suite.mosaic_oracle_v2 import validate_task, approach_width, solve, candidate_route, loaded_ik, assign_supply, grasp_rotations, deliver_cube, validate_motion_profile


def task():
    return dict(id='mosaic-test', family='mosaic', suite_version=2, level=1, cube_size_m=.06,
                table_height_m=.8, objects=[dict(color='red')],
                goals=[dict(goal_id='r1c1', requires=dict(color='red'),
                            position_m=[0,0,.83])])


class MosaicReferenceTests(unittest.TestCase):
    def test_goal_access_can_use_the_references_actual_release_height_and_keep_tilt_requirement(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import goal_endpoint_candidates
        source=task();source['arms']={'A':{'base_position_m':[-.5,0.,.8]}}
        trial=SimpleNamespace(task=source,arms={'A':object()},event=Mock())
        def tilted(_,robot,point,rotation):
            return np.zeros(7) if -.97<rotation[2,2]<-.96 else None
        with patch('robofrontier.suite.mosaic_oracle_v2.helpers.goal_coverage',side_effect=RuntimeError('No candidate goal approach/seat/withdraw endpoints r1c1')), \
             patch('robofrontier.suite.mosaic_oracle_v2.loaded_ik',side_effect=tilted), \
             patch('robofrontier.suite.mosaic_oracle_v2.screen_release',return_value={'clearance_m':.04,'minimum_joint_margin_rad':.1}):
            result=goal_endpoint_candidates(trial)
        self.assertEqual(result,{'r1c1':['A']})
        self.assertTrue(trial.mosaic_goal_tilted_coverage['r1c1']['A'])

    def test_destination_tilt_requirement_reaches_the_actual_pick_operation(self):
        from unittest.mock import Mock
        trial=SimpleNamespace(task={'cube_size_m':.06},env={'stations':[],'edges':[]},
            data=SimpleNamespace(body=lambda _:SimpleNamespace(xpos=np.array([.5,0.,.83]))),
            mosaic_current_job={'goal_id':'r1c1'},mosaic_goal_tilted_coverage={'r1c1':{'A':True}})
        with patch('robofrontier.suite.mosaic_oracle_v2.pick_transfer') as angled, \
             patch('robofrontier.suite.mosaic_oracle_v2.pick_for_transport') as normal, \
             patch('robofrontier.suite.mosaic_oracle_v2.place_cube'), \
             patch('robofrontier.suite.mosaic_oracle_v2.helpers.park'):
            deliver_cube(trial,'cube',['A'],np.array([0.,0.,.83]))
        normal.assert_not_called();self.assertTrue(angled.call_args.kwargs['required_tilt'])

    def test_endpoint_map_keeps_a_supply_reachable_only_with_an_existing_transfer_tilt(self):
        body=SimpleNamespace(xpos=np.array([.5,0.,.83]),xmat=np.eye(3))
        trial=SimpleNamespace(task={'cube_size_m':.06,'arms':{'A':{'base_position_m':[0.,0.,.8]}}},
            env={'objects':[{'id':'cube'}]},arms={'A':object()},home_q={'A':np.zeros(7)},
            data=SimpleNamespace(body=lambda _:body),
            screen_ik=lambda robot,point,rotation,seed=None:np.zeros(7) if -.95<rotation[2,2]<-.85 else None)
        from robofrontier.suite.mosaic_oracle_v2 import pickup_endpoint_candidates
        with patch('robofrontier.suite.mosaic_oracle_v2.helpers.pickup_coverage',return_value={'cube':[]}):
            self.assertEqual(pickup_endpoint_candidates(trial),{'cube':['A']})

    def test_narrow_transfer_search_is_empty_hand_only(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import pick_transfer
        for job in (None,'unresolved-id'):
            arm=SimpleNamespace(grip_max=.08,grasp_force=None,job=job,observe=lambda:{'gripper_width_m':.072})
            trial=SimpleNamespace(arms={'A':arm},task={'cube_size_m':.06,'arms':{'A':{'base_position_m':[0,0,.8]}}},
                data=SimpleNamespace(body=lambda _:SimpleNamespace(xpos=np.array([.5,0.,.83]),xmat=np.eye(3))),
                actions=[],held={},execute=Mock(),event=Mock())
            with patch('robofrontier.suite.mosaic_oracle_v2.loaded_ik',return_value=None):
                with self.assertRaisesRegex(RuntimeError,'No transfer pickup'):pick_transfer(trial,'A','cube',[.1,0.,.83])
            self.assertEqual(trial.execute.call_count,1 if job is None else 0)
            if job is None:self.assertAlmostEqual(trial.execute.call_args.args[1]['A'][1]['width_m'],.064)
    def stopped_motion_trial(self):
        from unittest.mock import Mock
        arm=SimpleNamespace(job=None,grasp_force=50.,qids=np.arange(7),dofs=np.arange(7))
        result={'C':{'status':'failed','call_id':'original-call','reason':'Action timeout; stopped at current reference'}}
        trial=SimpleNamespace(arms={'C':arm},held={'C':'cube'},task={'table_height_m':.8},
            monitor=SimpleNamespace(violations=[]),data=SimpleNamespace(qpos=np.zeros(7),qvel=np.zeros(7),body=lambda _:SimpleNamespace(xpos=np.array([0.,0.,1.]))),
            actions=[{'label':'mosaic-feedback-transit-0','commands':{'C':('move',{'joint_rad':[0.]*7})},'results':result}],
            event=Mock(),wait=Mock(),finger_contacts=Mock(return_value=True),pose_clear=Mock(return_value=True),
            move_free=Mock(side_effect=RuntimeError('Original motion stopped')))
        return trial,result

    def test_loaded_transit_can_continue_after_verified_terminal_failure_without_changing_feedback(self):
        from copy import deepcopy
        from robofrontier.suite.mosaic_oracle_v2 import execute_loaded_transit
        trial,result=self.stopped_motion_trial();original=deepcopy(result)
        execute_loaded_transit(trial,'C',np.zeros(3),np.eye(3),np.zeros(7))
        self.assertEqual(result,original)
        self.assertEqual(len(trial.mosaic_recovered_motion_failures),1)
        self.assertGreater(trial.wait.call_count,0)

    def test_loaded_transit_never_retries_or_accepts_unknown_execution(self):
        from robofrontier.suite.mosaic_oracle_v2 import execute_loaded_transit
        trial,result=self.stopped_motion_trial();trial.arms['C'].job='original-call';result['C']['status']='uncertain'
        with self.assertRaises(RuntimeError):execute_loaded_transit(trial,'C',np.zeros(3),np.eye(3),np.zeros(7))
        trial.wait.assert_not_called()

    def test_loaded_transit_does_not_accept_a_stopped_pose_away_from_target(self):
        from robofrontier.suite.mosaic_oracle_v2 import execute_loaded_transit
        trial,_=self.stopped_motion_trial();trial.data.qpos[:]=.1
        with self.assertRaises(RuntimeError):execute_loaded_transit(trial,'C',np.zeros(3),np.eye(3),np.zeros(7))

    def recovery_inputs_trial(self, assignments):
        objects=[dict(id='settled',color='red'),dict(id='unused',color='red')]
        goals=[dict(goal_id='done',requires={'color':'red'},position_m=[0.,0.,.83]),
               dict(goal_id='todo',requires={'color':'red'},position_m=[.1,0.,.83])]
        return SimpleNamespace(task={'goals':goals,'arms':{'A':{'base_position_m':[0.,0.,.8]}}},
            env={'objects':objects},monitor=SimpleNamespace(result=lambda:{'assignments':assignments}),
            data=SimpleNamespace(body=lambda name:SimpleNamespace(xpos=np.array([0. if name=='settled' else .2,0.,.83]))))

    def test_replanning_cannot_take_supply_from_a_completed_goal(self):
        from robofrontier.suite.mosaic_oracle_v2 import protected_remaining_inputs
        trial=self.recovery_inputs_trial({'done':'settled'})
        goals,objects,protected=protected_remaining_inputs(trial,{'done':'settled'})
        jobs=assign_supply(trial,goals,{'todo':['A']},
            {'settled':['A'],'unused':['A']},{'A':[]},available_objects=objects)
        self.assertEqual([goal['goal_id'] for goal in goals],['todo'])
        self.assertEqual(jobs[0][1],'unused')
        self.assertEqual(protected,{'done':'settled'})

    def test_replanning_rejects_a_changed_completed_pairing(self):
        from robofrontier.suite.mosaic_oracle_v2 import protected_remaining_inputs
        trial=self.recovery_inputs_trial({'done':'unused'})
        with self.assertRaisesRegex(RuntimeError,'Previously completed pairing changed'):
            protected_remaining_inputs(trial,{'done':'settled'})

    def test_replanning_protects_newly_completed_pairs_too(self):
        from robofrontier.suite.mosaic_oracle_v2 import protected_remaining_inputs
        trial=self.recovery_inputs_trial({'done':'settled','todo':'unused'})
        goals,objects,protected=protected_remaining_inputs(trial,{'done':'settled'})
        self.assertEqual(goals,[]);self.assertEqual(objects,[])
        self.assertEqual(protected,{'done':'settled','todo':'unused'})

    def test_transfer_approach_is_seeded_from_qualified_seat_and_checks_descent(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import transfer_approach_branch
        seat=np.ones(7)*.2;approach=np.ones(7)*.3
        arm=SimpleNamespace(ik=SimpleNamespace(limits=np.array([[-2.,2.]]*7)))
        trial=SimpleNamespace(arms={'A':arm},screen_ik=Mock(return_value=approach),pose_clear=Mock(return_value=True))
        with patch('robofrontier.suite.mosaic_oracle_v2.screen_transfer_segments',return_value=True) as check:
            actual=transfer_approach_branch(trial,'A','cube',np.array([0.,0.,.8]),np.eye(3),seat,.04)
        np.testing.assert_array_equal(actual,approach)
        np.testing.assert_array_equal(trial.screen_ik.call_args.kwargs['seed'],seat)
        np.testing.assert_array_equal(check.call_args.args[4],approach)
        with patch('robofrontier.suite.mosaic_oracle_v2.screen_transfer_segments',return_value=False):
            self.assertIsNone(transfer_approach_branch(trial,'A','cube',np.array([0.,0.,.8]),np.eye(3),seat,.04))

    def test_lower_release_candidates_are_final_pocket_only_and_dimension_scaled(self):
        from robofrontier.suite.mosaic_oracle_v2 import pocket_release_clearances
        source=task();source['grid']={'depth_m':.02}
        trial=SimpleNamespace(task=source)
        values=pocket_release_clearances(trial,np.array([0.,0.,.83]),.04374)
        self.assertAlmostEqual(values[-1],.039)
        self.assertTrue(all(value<.04374-.0005 for value in values))
        self.assertEqual(pocket_release_clearances(trial,np.array([1.,0.,.83]),.04374),())
        for level in (2,3):
            source['level']=level
            self.assertEqual(pocket_release_clearances(trial,np.array([0.,0.,.83]),.04374),())
        source['level']=1;source['cube_size_m']=.12
        wider=pocket_release_clearances(trial,np.array([0.,0.,.83]),.04374)
        self.assertAlmostEqual(wider[0],.014)
        self.assertNotEqual(wider,values)

    def test_failed_lower_release_screen_preserves_original_release_pose(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import lower_pocket_release
        trial=SimpleNamespace(move=Mock(),wait=Mock(),event=Mock())
        with patch('robofrontier.suite.mosaic_oracle_v2.screen_lower_pocket_release',return_value=None):
            lower_pocket_release(trial,'A',np.array([0.,0.,.83]))
        trial.move.assert_not_called();trial.wait.assert_not_called();trial.event.assert_not_called()

    def test_transfer_segments_define_both_actual_endpoints_from_table_independent_datum(self):
        from robofrontier.suite.mosaic_oracle_v2 import transfer_lift_segments
        start=np.array([1.,0.,.855]);segments=transfer_lift_segments(start,[0,0,.8])
        self.assertEqual([label for label,_ in segments],['transfer-pick-lift-clear','transfer-pick-lift-retract'])
        np.testing.assert_allclose(segments[0][1],[1,0,.885])
        np.testing.assert_allclose(segments[1][1],[.9,0,.985])
        np.testing.assert_array_equal(start,[1,0,.855])

    def test_transfer_screen_rejects_unreachable_retract_endpoint(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import screen_transfer_segments
        origin=np.array([0.,0.,1.]);endpoint=origin+[.01,0,.01]
        arm=SimpleNamespace(ik=SimpleNamespace(limits=np.array([[-2,2]]*7),pose=lambda q:(origin,np.eye(3))))
        trial=SimpleNamespace(arms={'A':arm},screen_ik=Mock(return_value=None),pose_clear=Mock(return_value=True))
        self.assertFalse(screen_transfer_segments(trial,'A','cube',np.eye(3),np.zeros(7),[('retract',endpoint)]))
        np.testing.assert_allclose(trial.screen_ik.call_args.args[1],endpoint)
        trial.pose_clear.assert_not_called()

    def test_transfer_screen_checks_intermediate_whole_arm_collision(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import screen_transfer_segments
        origin=np.array([0.,0.,1.]);arm=SimpleNamespace(ik=SimpleNamespace(limits=np.array([[-2,2]]*7),pose=lambda q:(origin,np.eye(3))))
        trial=SimpleNamespace(arms={'A':arm},screen_ik=Mock(return_value=np.ones(7)*.02),
                              pose_clear=Mock(side_effect=lambda robot,q,allowed:q[0]<.014 or q[0]>.016))
        self.assertFalse(screen_transfer_segments(trial,'A','cube',np.eye(3),np.zeros(7),[('retract',origin+[.01,0,.01])]))

    def test_transfer_rechecks_measured_payload_and_does_not_send_rejected_retract(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import pick_transfer
        body=SimpleNamespace(xpos=np.array([1.,0.,.83]),xmat=np.eye(3))
        arm=SimpleNamespace(grip_max=.08,grasp_force=None,observe=lambda:{'gripper_width_m':.072},site=0,qids=np.arange(7))
        trial=SimpleNamespace(arms={'A':arm},task=dict(cube_size_m=.06,arms={'A':dict(base_position_m=[0,0,.8])}),
            data=SimpleNamespace(body=lambda name:body,site_xpos=np.array([[1.,0.,.855]]),qpos=np.zeros(7)),
            execute=Mock(),wait=Mock(),finger_contacts=Mock(return_value=True),held={},move_free=Mock(),move=Mock())
        with patch('robofrontier.suite.mosaic_oracle_v2.grasp_rotations',return_value=[(np.diag([1,-1,-1]),0)]), \
             patch('robofrontier.suite.mosaic_oracle_v2.loaded_ik',return_value=np.zeros(7)), \
             patch('robofrontier.suite.mosaic_oracle_v2.transfer_approach_branch',return_value=np.zeros(7)), \
             patch('robofrontier.suite.mosaic_oracle_v2.screen_transfer_segments',side_effect=[True,True,False,False]) as screen:
            with self.assertRaisesRegex(RuntimeError,'measured payload'):pick_transfer(trial,'A','cube',[.5,0,.83])
        self.assertEqual(screen.call_count,4)
        self.assertEqual(trial.held,{'A':'cube'})
        self.assertEqual([call.args[3] for call in trial.move.call_args_list],['transfer-pick-contact','transfer-pick-lift-clear'])

    def test_transfer_uses_screened_extension_when_inward_retract_is_rejected(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import pick_transfer
        body=SimpleNamespace(xpos=np.array([1.,0.,.83]),xmat=np.eye(3))
        arm=SimpleNamespace(grip_max=.08,grasp_force=None,observe=lambda:{'gripper_width_m':.072},site=0,qids=np.arange(7))
        trial=SimpleNamespace(arms={'A':arm},task=dict(cube_size_m=.06,arms={'A':dict(base_position_m=[0,0,.8])}),
            data=SimpleNamespace(body=lambda name:body,site_xpos=np.array([[1.,0.,.855]]),qpos=np.zeros(7)),
            execute=Mock(),wait=Mock(),finger_contacts=Mock(return_value=True),held={},move_free=Mock(),move=Mock())
        with patch('robofrontier.suite.mosaic_oracle_v2.grasp_rotations',return_value=[(np.diag([1,-1,-1]),0)]), \
             patch('robofrontier.suite.mosaic_oracle_v2.loaded_ik',return_value=np.zeros(7)), \
             patch('robofrontier.suite.mosaic_oracle_v2.transfer_approach_branch',return_value=np.zeros(7)), \
             patch('robofrontier.suite.mosaic_oracle_v2.screen_transfer_segments',side_effect=[False,True,True,False,False]) as screen:
            with self.assertRaisesRegex(RuntimeError,'measured payload'):pick_transfer(trial,'A','cube',[.5,0,.83])
        self.assertEqual(screen.call_count,5)
        self.assertLess(screen.call_args_list[0].args[-1][-1][1][0],1.)
        for call in screen.call_args_list[1:4]:
            self.assertGreater(call.args[-1][-1][1][0],1.)
        self.assertLess(screen.call_args_list[-1].args[-1][-1][1][0],1.)
        self.assertEqual([call.args[3] for call in trial.move.call_args_list],['transfer-pick-contact','transfer-pick-lift-clear'])

    def test_transfer_replans_retract_only_with_measured_path_and_continuation(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import pick_transfer
        for future in (False,True):
            with self.subTest(future=future):
                body=SimpleNamespace(xpos=np.array([1.,0.,.83]),xmat=np.eye(3))
                arm=SimpleNamespace(grip_max=.08,grasp_force=None,observe=lambda:{'gripper_width_m':.072},site=0,qids=np.arange(7))
                data=SimpleNamespace(body=lambda name:body,site_xpos=np.array([[1.,0.,.855]]),qpos=np.zeros(7))
                def move(robot,point,rotation=None,label='move',**kwargs):
                    delta=np.asarray(point)-data.site_xpos[0];body.xpos+=delta;data.site_xpos[0]=point
                trial=SimpleNamespace(arms={'A':arm},task=dict(cube_size_m=.06,arms={'A':dict(base_position_m=[0,0,.8])}),
                    data=data,execute=Mock(),wait=Mock(),finger_contacts=Mock(return_value=True),held={},move_free=Mock(),move=Mock(side_effect=move),event=Mock())
                with patch('robofrontier.suite.mosaic_oracle_v2.grasp_rotations',return_value=[(np.diag([1,-1,-1]),0)]), \
                     patch('robofrontier.suite.mosaic_oracle_v2.loaded_ik',return_value=np.zeros(7)), \
                     patch('robofrontier.suite.mosaic_oracle_v2.transfer_approach_branch',return_value=np.zeros(7)), \
                     patch('robofrontier.suite.mosaic_oracle_v2.screen_transfer_segments',side_effect=[True,True,False,True]), \
                     patch('robofrontier.suite.mosaic_oracle_v2.transfer_continuation_available',return_value=future):
                    if future:pick_transfer(trial,'A','cube',[.5,0,.83])
                    else:
                        with self.assertRaisesRegex(RuntimeError,'measured payload'):pick_transfer(trial,'A','cube',[.5,0,.83])
                labels=[call.args[3] for call in trial.move.call_args_list]
                self.assertEqual(labels,['transfer-pick-contact','transfer-pick-lift-clear']+(['transfer-pick-lift-retract'] if future else []))
                if future:
                    self.assertGreater(trial.move.call_args_list[-1].args[1][0],1.)
                    self.assertTrue(any(call.args[0]=='transfer_measured_retract_replan' for call in trial.event.call_args_list))

    def test_pickup_endpoint_map_covers_actual_low_grasp_and_negative_closing_direction(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import pickup_endpoint_candidates
        body=SimpleNamespace(xpos=np.array([.5,0.,.83]),xmat=np.eye(3))
        def ik(robot,point,rotation,seed=None):
            if np.allclose(rotation[:,1],[0,-1,0]) and (abs(point[2]-.836)<1e-8 or abs(point[2]-.976)<1e-8):return np.zeros(7)
            return None
        trial=SimpleNamespace(task=dict(cube_size_m=.06,arms={'A':dict(base_position_m=[0,0,.8])}),
            env=dict(objects=[dict(id='cube',source='A')]),arms={'A':object()},home_q={'A':np.zeros(7)},
            data=SimpleNamespace(body=lambda name:body),screen_ik=Mock(side_effect=ik))
        with patch('robofrontier.suite.mosaic_oracle_v2.helpers.pickup_coverage',return_value={'cube':[]}):
            self.assertEqual(pickup_endpoint_candidates(trial),{'cube':['A']})
        trial.screen_ik=Mock(return_value=None)
        with patch('robofrontier.suite.mosaic_oracle_v2.helpers.pickup_coverage',return_value={'cube':[]}):
            self.assertEqual(pickup_endpoint_candidates(trial),{'cube':[]})

    def test_normal_picker_does_not_move_without_reserved_approach_candidate(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import pick_for_transport
        arm=SimpleNamespace(grip_max=.08)
        original=Mock()
        trial=SimpleNamespace(task=dict(cube_size_m=.06),arms={'A':arm},pose_clear=Mock(return_value=True),
            event=Mock(),move_free=original)
        def pick(*args,**kwargs):
            try:trial.move_free('A',[0,0,1],np.eye(3),'pick-approach',continuation=([0,0,.85],np.eye(3),('cube',)))
            except RuntimeError:pass
            raise RuntimeError('No pickup pose A/cube')
        trial.pick=pick
        with patch('robofrontier.suite.mosaic_oracle_v2.normal_approach_candidates',return_value=[]):
            with self.assertRaisesRegex(RuntimeError,'No geometrically screened'):
                pick_for_transport(trial,'A','cube',[0,0,.83])
        original.assert_not_called()
        self.assertIs(trial.move_free,original)

    def test_normal_approach_has_bounded_folded_seed_without_relaxing_limits(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import normal_approach_candidates
        trial=SimpleNamespace(task={'cube_size_m':.06},home_q={'A':np.zeros(7)},
            arms={'A':SimpleNamespace(ik=SimpleNamespace(limits=np.array([[-3.,3.]]*7)))},
            screen_ik=Mock(return_value=np.zeros(7)),pose_clear=Mock(return_value=True))
        with patch('robofrontier.suite.mosaic_oracle_v2.loaded_ik_candidates',return_value=[]):
            point,q=next(normal_approach_candidates(trial,'A',np.array([0.,0.,1.]),np.eye(3),expanded=True))
        np.testing.assert_allclose(point,[0.,0.,1.06])
        seed=trial.screen_ik.call_args.kwargs['seed']
        self.assertEqual((seed[0],seed[2]),(-2.,-2.))

    def test_expanded_approach_does_not_preempt_an_existing_normal_grasp(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import pick_for_transport
        selected=[];context={};seen=[]
        def candidates(trial,robot,point,rotation,expanded=True):
            seen.append((context['height'],expanded))
            return [(np.asarray(point),np.zeros(7))] if expanded or abs(context['height']-.02)<1e-8 else []
        trial=SimpleNamespace(task={'cube_size_m':.06},arms={'A':SimpleNamespace(grip_max=.08)},
            pose_clear=Mock(return_value=True),move_free=Mock(),event=Mock())
        def pick(robot,name,height,**kwargs):
            context['height']=height
            try:trial.move_free(robot,[0,0,1.],np.eye(3),'pick-approach',continuation=([0,0,.85],np.eye(3),(name,)))
            except RuntimeError:raise RuntimeError('No pickup pose A/cube')
            selected.append(height)
        trial.pick=pick
        with patch('robofrontier.suite.mosaic_oracle_v2.normal_approach_candidates',side_effect=candidates), \
             patch('robofrontier.suite.mosaic_oracle_v2.screen_transfer_segments',return_value=True):
            pick_for_transport(trial,'A','cube',[0,0,.83])
        np.testing.assert_allclose(selected,[.02],atol=1e-12)
        self.assertTrue(all(expanded is False for _,expanded in seen))

    def test_path_interior_margin_does_not_remove_endpoint_correction_reserve(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import screen_transfer_segments
        arm=SimpleNamespace(ik=SimpleNamespace(limits=np.array([[-2,2]]*7),pose=lambda q:(np.zeros(3),np.eye(3))))
        near=np.array([1.969,0,0,0,0,0,0]);end=np.array([1.9,0,0,0,0,0,0])
        trial=SimpleNamespace(arms={'A':arm},screen_ik=Mock(side_effect=[near,end]),pose_clear=Mock(return_value=True))
        segment=[('contact',np.array([.03,0,0]))]
        self.assertFalse(screen_transfer_segments(trial,'A','cube',np.eye(3),np.zeros(7),segment))
        trial.screen_ik=Mock(side_effect=[near,end])
        self.assertTrue(screen_transfer_segments(trial,'A','cube',np.eye(3),np.zeros(7),segment,path_joint_margin_rad=0.))
        trial.screen_ik=Mock(side_effect=[near,np.array([1.99,0,0,0,0,0,0])])
        self.assertFalse(screen_transfer_segments(trial,'A','cube',np.eye(3),np.zeros(7),segment,path_joint_margin_rad=0.))

    def test_normal_lift_checks_actual_grasped_payload_before_any_lift_command(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import pick_for_transport
        arm=SimpleNamespace(grip_max=.08,site=0,qids=np.arange(7))
        original=Mock();trial=SimpleNamespace(task=dict(cube_size_m=.06),arms={'A':arm},pose_clear=Mock(return_value=True),
            event=Mock(),move=original,held={},finger_contacts=Mock(return_value=True),
            data=SimpleNamespace(site_xmat=np.array([np.eye(3)]),qpos=np.zeros(7)))
        def pick(*args,**kwargs):trial.move('A',[0,0,1.],np.eye(3),'pick-lift')
        trial.pick=pick
        with patch('robofrontier.suite.mosaic_oracle_v2.screen_transfer_segments',return_value=False):
            with self.assertRaisesRegex(RuntimeError,'measured segment rejected'):
                pick_for_transport(trial,'A','cube',[0,0,.83])
        original.assert_not_called()
        self.assertEqual(trial.held,{'A':'cube'})
        self.assertIs(trial.move,original)

    def test_normal_lift_can_end_higher_without_relaxing_the_screen(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import pick_for_transport
        arm=SimpleNamespace(grip_max=.08,site=0,qids=np.arange(7))
        original=Mock();trial=SimpleNamespace(task=dict(cube_size_m=.06),arms={'A':arm},
            pose_clear=Mock(return_value=True),event=Mock(),move=original,held={},
            finger_contacts=Mock(return_value=True),
            data=SimpleNamespace(site_xmat=np.array([np.eye(3)]),qpos=np.zeros(7)))
        trial.pick=lambda *args,**kwargs:trial.move('A',np.array([0.,0.,1.]),np.eye(3),'pick-lift')
        with patch('robofrontier.suite.mosaic_oracle_v2.screen_transfer_segments',side_effect=[False,True]) as screen:
            pick_for_transport(trial,'A','cube',[0,0,.83])
        original.assert_called_once()
        np.testing.assert_allclose(original.call_args.args[1],[0.,0.,1.02])
        self.assertEqual(screen.call_count,2)
        self.assertEqual(screen.call_args.kwargs['path_joint_margin_rad'],0.)

    def test_dimension_derived_approach_and_table_datum(self):
        report=validate_task(task())
        self.assertAlmostEqual(report['approach_width_m'], .072)
        self.assertEqual(report['table_height_m'], .8)

    def test_repair_aperture_clears_sixty_mm_cube(self):
        trial=SimpleNamespace(task=task(), arms={'A':SimpleNamespace(grip_max=.08)})
        self.assertAlmostEqual(approach_width(trial,'A'), .072)

    def test_widened_fingertip_profile_is_checked_on_actual_robot(self):
        source=task();source['cube_size_m']=.1
        self.assertEqual(validate_task(source)['cube_size_m'],.1)
        trial=SimpleNamespace(task=source,arms={'A':SimpleNamespace(grip_max=.12)})
        self.assertAlmostEqual(approach_width(trial,'A'),.112)
        trial.arms['A'].grip_max=.08
        with self.assertRaisesRegex(RuntimeError,'aperture'):approach_width(trial,'A')

    def test_reference_speed_profile_cannot_change_grasp_force(self):
        with self.assertRaisesRegex(ValueError,'Unknown'):
            validate_motion_profile({'grasp_force_n':70})
        self.assertEqual(validate_motion_profile(None),{})
        self.assertEqual(validate_motion_profile({'loaded_joint_speed_rad_s':.65}),{'loaded_joint_speed_rad_s':.65})
        with self.assertRaisesRegex(ValueError,'positive'):
            validate_motion_profile({'coarse_tcp_speed_m_s':float('nan')})

    def test_insufficient_supply_rejected(self):
        source=task();source['objects']=[]
        with self.assertRaisesRegex(ValueError,'supply'):validate_task(source)

    def test_insufficient_hand_clearance_rejected(self):
        trial=SimpleNamespace(task=task(), arms={'A':SimpleNamespace(grip_max=.061)})
        with self.assertRaisesRegex(RuntimeError,'aperture'):approach_width(trial,'A')

    def test_duplicate_goal_rejected(self):
        source=task();source['goals']*=2
        with self.assertRaisesRegex(ValueError,'Duplicate'):validate_task(source)


    def fake_trial(self, directory):
        source=task()
        source['arms']={'A':dict(base_position_m=[0,0,.8])}
        source['objects']=[dict(id='cube',color='red',source='A')]
        source['goals']=[dict(goal_id='r1c1',requires=dict(color='red'),position_m=[0,0,.83])]
        accepted={}
        trial=SimpleNamespace(task=source,env=dict(objects=source['objects']),directory=Path(directory),
            arms={'A':SimpleNamespace(grip_max=.08)},
            data=SimpleNamespace(body=lambda name:SimpleNamespace(xpos=np.array([0.,0.,.83]),xmat=np.eye(3))),
            monitor=SimpleNamespace(violations=[],result=lambda:dict(assignments=accepted)),
            wait=lambda seconds:None,event=lambda *args,**kwargs:None)
        return trial,accepted

    def helper_patches(self):
        from contextlib import ExitStack
        stack=ExitStack()
        for name,value in [('clear_idle_arms',None),('graph',({'A':[]},{})),
                           ('goal_coverage',{'r1c1':['A']}),
                           ('pickup_coverage',{'cube':['A']}),
                           ('park',None)]:
            stack.enter_context(patch('robofrontier.suite.mosaic_oracle_v2.helpers.'+name,return_value=value))
        return stack

    def test_disconnected_alternative_does_not_discard_direct_pickup(self):
        trial=SimpleNamespace(task={'arms':{'A':{'base_position_m':[0,0,0]},'B':{'base_position_m':[1,0,0]}}},
                              data=SimpleNamespace(body=lambda name:SimpleNamespace(xpos=np.array([0,0,0]))))
        self.assertEqual(candidate_route(trial,{'A':[],'B':[]},['B','A'],'A','cube'),['A'])
        self.assertIsNone(candidate_route(trial,{'A':[],'B':[]},['B'],'A','cube'))

    def test_loaded_ik_uses_home_seed_after_local_branch_failure(self):
        from unittest.mock import Mock
        limits=np.array([[-2.,2.]]*7)
        trial=SimpleNamespace(arms={'A':SimpleNamespace(qref=np.ones(7),ik=SimpleNamespace(limits=limits))},home_q={'A':np.zeros(7)},
                              screen_ik=Mock(side_effect=[None,np.zeros(7),None,None]),pose_clear=Mock(return_value=True))
        np.testing.assert_array_equal(loaded_ik(trial,'A',[0,0,1],np.eye(3)),np.zeros(7))

    def test_loaded_ik_rejects_endpoint_at_joint_limit(self):
        from unittest.mock import Mock
        limits=np.array([[-2.,2.]]*7)
        trial=SimpleNamespace(arms={'A':SimpleNamespace(qref=np.ones(7),ik=SimpleNamespace(limits=limits))},home_q={'A':np.zeros(7)},
                              screen_ik=Mock(side_effect=[np.array([2.,0,0,0,0,0,0]),np.zeros(7),None,None]),pose_clear=Mock(return_value=True))
        np.testing.assert_array_equal(loaded_ik(trial,'A',[0,0,1],np.eye(3)),np.zeros(7))

    def test_global_matching_preserves_supply_needed_by_restricted_goal(self):
        source=task();source['arms']={'A':{'base_position_m':[0,0,.8]},'B':{'base_position_m':[1,0,.8]}}
        goals=[dict(goal_id='shared',requires=dict(color='red'),position_m=[0,0,.83]),
               dict(goal_id='restricted',requires=dict(color='red'),position_m=[0,0,.83])]
        trial=SimpleNamespace(task=source,env={'objects':[dict(id='a',color='red'),dict(id='b',color='red')]},
              data=SimpleNamespace(body=lambda name:SimpleNamespace(xpos=np.array([0 if name=='a' else 1,0,.83]))))
        jobs=assign_supply(trial,goals,{'shared':['A','B'],'restricted':['A']},
                           {'a':['A'],'b':['B']},{'A':[],'B':[]})
        self.assertEqual({g['goal_id']:name for g,name,_ in jobs},{'shared':'b','restricted':'a'})

    def test_incomplete_supply_plan_rejected_before_placement(self):
        source=task();source['arms']={'A':{'base_position_m':[0,0,.8]},'B':{'base_position_m':[1,0,.8]}}
        trial=SimpleNamespace(task=source,env={'objects':[dict(id='a',color='red')]},
              data=SimpleNamespace(body=lambda name:SimpleNamespace(xpos=np.array([0,0,.83]))))
        with self.assertRaisesRegex(ValueError,'No supply path'):
            assign_supply(trial,source['goals'],{'r1c1':['B']},{'a':['A']},{'A':[],'B':[]})

    def test_feedback_rejects_dropped_payload_before_any_move(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import align_payload
        body=SimpleNamespace(xpos=np.array([0.,0.,.83]),xmat=np.eye(3))
        trial=SimpleNamespace(held={'A':'cube'},arms={'A':SimpleNamespace(site=0)},
            data=SimpleNamespace(body=lambda name:body,site_xpos=np.array([[0.,0.,1.4]]),site_xmat=np.array([np.eye(3)])),
            task={'cube_size_m':.06},finger_contacts=Mock(return_value=False),
            move=Mock(side_effect=AssertionError('Dropped object must not create a feedback move')))
        with self.assertRaisesRegex(RuntimeError,'Payload contact missing before'):
            align_payload(trial,'A',np.array([0.,0.,1.]),np.eye(3),'hover')
        trial.move.assert_not_called()

    def test_contact_guard_detects_large_relative_slip_even_with_contact(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import check_payload
        body=SimpleNamespace(xpos=np.array([0.,0.,.83]))
        trial=SimpleNamespace(held={'A':'cube'},arms={'A':SimpleNamespace(site=0)},
            data=SimpleNamespace(body=lambda name:body,site_xpos=np.array([[0.,0.,1.4]]),site_xmat=np.array([np.eye(3)])),
            task={'cube_size_m':.06},finger_contacts=Mock(return_value=True))
        with self.assertRaisesRegex(RuntimeError,'Payload slipped during'):
            check_payload(trial,'A',np.array([0.,0.,-.02]),label='transit')

    def test_pose_window_rejects_nominal_pose_without_correction_room(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import pose_window
        arm=SimpleNamespace(ik=SimpleNamespace(limits=np.array([[-2.,2.]]*7)))
        trial=SimpleNamespace(arms={'A':arm},screen_ik=Mock(side_effect=[np.zeros(7),None]),
                              pose_clear=Mock(return_value=True))
        self.assertIsNone(pose_window(trial,'A',np.array([0.,0.,1.]),np.eye(3),np.zeros(7)))
        self.assertEqual(trial.screen_ik.call_count,2)

    def test_pose_window_checks_both_signs_of_each_translation_axis(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import pose_window
        arm=SimpleNamespace(ik=SimpleNamespace(limits=np.array([[-2.,2.]]*7)))
        trial=SimpleNamespace(arms={'A':arm},screen_ik=Mock(return_value=np.zeros(7)),
                              pose_clear=Mock(return_value=True))
        result=pose_window(trial,'A',np.array([0.,0.,1.]),np.eye(3),np.zeros(7))
        self.assertIsNotNone(result)
        self.assertEqual(trial.screen_ik.call_count,7)
        points=[c.args[1] for c in trial.screen_ik.call_args_list]
        for axis in range(3):
            for sign in (-1,1):
                point=np.array([0.,0.,1.]);point[axis]+=sign*.004
                self.assertTrue(any(np.allclose(point,p) for p in points))

    def test_release_screen_requires_reachable_withdrawal(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import screen_release
        arm=SimpleNamespace(ik=SimpleNamespace(limits=np.array([[-2.,2.]]*7),pose=lambda q:(np.array([0.,0.,1.]),np.eye(3))))
        trial=SimpleNamespace(arms={'A':arm},task={'cube_size_m':.06},
            release_clearance_height=Mock(return_value=.02),pose_clear=Mock(return_value=True),
            screen_ik=Mock(side_effect=[np.zeros(7)]*11+[None]))
        self.assertIsNone(screen_release(trial,'A',np.array([0.,0.,.83]),np.eye(3),np.array([0.,0.,.02]),np.eye(3),np.zeros(7)))
        self.assertEqual(trial.screen_ik.call_count,12)

    def test_process_guard_stops_before_next_control_check_after_violation(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import install_process_guard
        original=Mock();trial=SimpleNamespace(check_stop=original,monitor=SimpleNamespace(violations=[]))
        install_process_guard(trial);trial.check_stop()
        original.assert_called_once()
        trial.monitor.violations.append({'kind':'inter_robot_contact'})
        with self.assertRaisesRegex(RuntimeError,'Process violation'):
            trial.check_stop()
        original.assert_called_once()

    def test_private_free_routes_explicitly_request_fine_sampling(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import install_fine_route_screen
        original=Mock();trial=SimpleNamespace(move_free=original)
        install_fine_route_screen(trial);trial.move_free('A',[0.,0.,1.],np.eye(3),label='approach')
        self.assertEqual(original.call_args.kwargs['route_joint_step_rad'],.005)
        self.assertEqual(original.call_args.kwargs['label'],'approach')

    def stable_samples(self):
        return [dict(sim_time_s=i*.02,offset_m=[.032,0.,.017],relative_rotation=np.eye(3).tolist(),
                     linear_speed_m_s=.0001,angular_speed_rad_s=.001,
                     bilateral=True,airborne=True,free_support=True) for i in range(26)]

    def test_stable_reposition_is_not_misclassified_as_a_drop(self):
        from robofrontier.suite.mosaic_oracle_v2 import stable_relocation
        self.assertTrue(stable_relocation(self.stable_samples()))

    def test_stability_requires_a_window_not_two_endpoint_frames(self):
        from robofrontier.suite.mosaic_oracle_v2 import stable_relocation
        samples=self.stable_samples()
        self.assertFalse(stable_relocation([samples[0],samples[-1]]))

    def test_continuing_slip_or_rotation_is_not_stable_relocation(self):
        from robofrontier.suite.mosaic_oracle_v2 import stable_relocation
        samples=self.stable_samples();samples[-1]['offset_m'][0]+=.002
        self.assertFalse(stable_relocation(samples))
        samples=self.stable_samples();angle=.04
        samples[-1]['relative_rotation']=[[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]]
        self.assertFalse(stable_relocation(samples))

    def test_contact_loss_or_external_support_cannot_verify_a_grip(self):
        from robofrontier.suite.mosaic_oracle_v2 import stable_relocation
        for condition in ('bilateral','airborne','free_support'):
            samples=self.stable_samples();samples[12][condition]=False
            self.assertFalse(stable_relocation(samples))

    def test_normal_pick_uses_deepest_geometrically_available_grasp(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import pick_for_transport
        original=Mock(return_value=True)
        trial=SimpleNamespace(task={'cube_size_m':.06},arms={'A':SimpleNamespace(grip_max=.08)},
            pose_clear=original,event=Mock(),pick=Mock(side_effect=[RuntimeError('No pickup pose A/cube'),None]))
        pick_for_transport(trial,'A','cube',np.array([0.,0.,.83]))
        self.assertEqual([round(c.kwargs['height'],6) for c in trial.pick.call_args_list],[.006,.012])
        self.assertIs(trial.pose_clear,original)
        self.assertTrue(all(c.kwargs['force']==50. for c in trial.pick.call_args_list))

    def test_normal_pick_does_not_retry_an_executed_grasp_failure(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import pick_for_transport
        trial=SimpleNamespace(task={'cube_size_m':.06},arms={'A':SimpleNamespace(grip_max=.08)},
            pose_clear=Mock(return_value=True),event=Mock(),pick=Mock(side_effect=RuntimeError('Bilateral cube contact missing cube')))
        with self.assertRaisesRegex(RuntimeError,'Bilateral'):
            pick_for_transport(trial,'A','cube',np.array([0.,0.,.83]))
        self.assertEqual(trial.pick.call_count,1)

    def test_oblique_grasp_preserves_parallel_closing_faces(self):
        up=np.array([0.,0,1.]);axis=np.array([1.,0,0])
        rotations=list(grasp_rotations(up,[axis],np.array([0.,1,0]),tilts=(0,15,25)))
        for rotation,angle in rotations:
            np.testing.assert_allclose(rotation.T@rotation,np.eye(3),atol=1e-12)
            np.testing.assert_allclose(rotation[:,1],axis)
            self.assertAlmostEqual(np.linalg.det(rotation),1.)
            self.assertAlmostEqual(rotation[2,2],-np.cos(np.deg2rad(angle)))

    def test_final_goal_equal_to_buffer_does_not_force_extra_handover(self):
        from unittest.mock import Mock
        point=np.array([0.,0,.83]);trial=SimpleNamespace(
            task={'cube_size_m':.06,'level':1,'goals':[]},env={'stations':[],'edges':[]},
            data=SimpleNamespace(body=lambda name:SimpleNamespace(xpos=np.array([-1.,0,.83]))),
            mosaic_edges={frozenset(('A','B')):point},pick=Mock(),event=Mock(),pose_clear=Mock(return_value=True),arms={'A':SimpleNamespace(grip_max=.08)})
        with patch('robofrontier.suite.mosaic_oracle_v2.place_cube'),patch('robofrontier.suite.mosaic_oracle_v2.helpers.park'):
            deliver_cube(trial,'cube',['A','B'],point)
        self.assertEqual(trial.pick.call_count,1)
        self.assertEqual(trial.pick.call_args.args[0],'A')

    def test_transfer_fallback_only_after_uncommitted_normal_admission(self):
        from unittest.mock import Mock
        from robofrontier.suite.mosaic_oracle_v2 import deliver_cube
        for kind in ('admission','failed_open','grasp','active','force','contact','execution_failure'):
            with self.subTest(kind=kind):
                arm=SimpleNamespace(grip_max=.08,grasp_force=None,job=None)
                trial=SimpleNamespace(task={'cube_size_m':.06,'level':1,'goals':[]},
                    env={'stations':[],'edges':[]},data=SimpleNamespace(body=lambda _:SimpleNamespace(xpos=np.array([-1.,0.,.83]))),
                    arms={'A':arm},held={},actions=[],finger_contacts=Mock(return_value=False),event=Mock())
                def rejected(*args,**kwargs):
                    if kind in ('failed_open','grasp'):
                        trial.actions.append(dict(label='open-for-pick' if kind=='failed_open' else 'pick-grasp',
                            commands={'A':('gripper',dict(mode='move' if kind=='failed_open' else 'grasp'))},
                            results={'A':dict(status='failed' if kind=='failed_open' else 'completed')}))
                    if kind=='active':arm.job='inflight'
                    if kind=='force':arm.grasp_force=50.
                    if kind=='contact':trial.finger_contacts.return_value=True
                    raise RuntimeError('Payload contact lost' if kind=='execution_failure' else 'No geometrically screened normal grasp A/cube')
                with patch('robofrontier.suite.mosaic_oracle_v2.pick_for_transport',side_effect=rejected), \
                     patch('robofrontier.suite.mosaic_oracle_v2.pick_transfer') as alternate, \
                     patch('robofrontier.suite.mosaic_oracle_v2.place_cube'), \
                     patch('robofrontier.suite.mosaic_oracle_v2.helpers.park'):
                    if kind=='admission':
                        deliver_cube(trial,'cube',['A'],np.array([0.,0.,.83]))
                        alternate.assert_called_once()
                    else:
                        with self.assertRaises(RuntimeError):deliver_cube(trial,'cube',['A'],np.array([0.,0.,.83]))
                        alternate.assert_not_called()

    def test_first_failed_placement_stops_after_two_repairs(self):
        with tempfile.TemporaryDirectory() as directory,self.helper_patches():
            trial,_=self.fake_trial(directory)
            from unittest.mock import Mock
            trial.pick=Mock();trial.place=Mock()
            self.addCleanup(patch.stopall)
            patch('robofrontier.suite.mosaic_oracle_v2.place_cube').start()
            with patch('robofrontier.suite.mosaic_oracle_v2.deliver_cube') as delivery:
                with self.assertRaisesRegex(RuntimeError,'bounded repair'):solve(trial)
            self.assertEqual(delivery.call_count,1)
            self.assertEqual(trial.pick.call_count,2)
            self.assertAlmostEqual(trial.pick.call_args.kwargs['aperture'],.072)

    def test_success_needs_unchanged_evaluator_acceptance(self):
        with tempfile.TemporaryDirectory() as directory,self.helper_patches():
            trial,accepted=self.fake_trial(directory)
            def delivery(*args):accepted['r1c1']='cube'
            with patch('robofrontier.suite.mosaic_oracle_v2.deliver_cube',side_effect=delivery):
                result=solve(trial)
            self.assertEqual(result['reference_placed'],1)
            self.assertFalse(result['commissioning_only'])

    def test_resume_preserves_completed_pairs_and_never_reparks(self):
        with tempfile.TemporaryDirectory() as directory,self.helper_patches():
            trial,accepted=self.fake_trial(directory)
            trial.held={}
            trial.task['objects']=[dict(id='cube',color='red'),dict(id='next',color='red')]
            trial.env['objects']=trial.task['objects']
            trial.task['goals'].append(dict(goal_id='next-goal',requires={'color':'red'},position_m=[.1,0,.83]))
            accepted['r1c1']='cube';trial.mosaic_protected_assignments=dict(accepted)
            def deliver(_,name,path,position):
                self.assertEqual(name,'next');accepted['next-goal']=name
            with patch('robofrontier.suite.mosaic_oracle_v2.helpers.clear_idle_arms') as park, \
                 patch('robofrontier.suite.mosaic_oracle_v2.helpers.goal_coverage',return_value={'next-goal':['A']}), \
                 patch('robofrontier.suite.mosaic_oracle_v2.pickup_endpoint_candidates',return_value={'next':['A']}), \
                 patch('robofrontier.suite.mosaic_oracle_v2.deliver_cube',side_effect=deliver):
                result=solve(trial,resume=True)
            park.assert_not_called()
            self.assertEqual(result['reference_placed'],2)
            self.assertTrue(result['resumed'])
            self.assertFalse(result['commissioning_only'])

    def test_resume_refuses_rematching_while_an_object_is_held(self):
        with tempfile.TemporaryDirectory() as directory,self.helper_patches():
            trial,_=self.fake_trial(directory);trial.held={'A':'cube'}
            with patch('robofrontier.suite.mosaic_oracle_v2.assign_supply') as assign:
                with self.assertRaisesRegex(RuntimeError,'held payload'):solve(trial,resume=True)
            assign.assert_not_called()

    def test_resume_completed_world_never_builds_another_pickup_plan(self):
        with tempfile.TemporaryDirectory() as directory,self.helper_patches():
            trial,accepted=self.fake_trial(directory);trial.held={};accepted['r1c1']='cube'
            with patch('robofrontier.suite.mosaic_oracle_v2.pickup_endpoint_candidates') as pickups:
                result=solve(trial,resume=True)
            pickups.assert_not_called();self.assertEqual(result['reference_placed'],1)

    def test_exhausted_supply_search_enters_owner_review_after_trying_buffers(self):
        from unittest.mock import Mock
        with tempfile.TemporaryDirectory() as directory,self.helper_patches():
            trial,_=self.fake_trial(directory);trial.event=Mock()
            with patch('robofrontier.suite.mosaic_oracle_v2.assign_supply',side_effect=ValueError('No complete unique supply assignment')) as assign, \
                 patch('robofrontier.suite.mosaic_oracle_v2.buffer_graph',return_value=({'A':[]},{})):
                with self.assertRaisesRegex(RuntimeError,'needs owner review'):solve(trial)
            self.assertEqual(assign.call_count,2)
            self.assertTrue(any(call.args[0]=='mosaic_supply_plan_exhausted' for call in trial.event.call_args_list))

    def test_uncommitted_admission_defers_until_independent_goal_progress(self):
        from unittest.mock import Mock
        with tempfile.TemporaryDirectory() as directory,self.helper_patches():
            trial,accepted=self.fake_trial(directory)
            trial.task['objects']=[dict(id='red',color='red',source='A'),dict(id='blue',color='blue',source='A')]
            trial.env['objects']=trial.task['objects']
            trial.task['goals']=[dict(goal_id='one',requires={'color':'red'},position_m=[0,0,.83]),
                                 dict(goal_id='two',requires={'color':'blue'},position_m=[.1,0,.83])]
            trial.arms['A'].job=None;trial.arms['A'].grasp_force=None
            trial.held={};trial.actions=[];trial.finger_contacts=Mock(return_value=False)
            seen=[]
            def delivery(_,name,path,position):
                seen.append(name)
                if name=='red' and seen.count('red')==1:
                    raise RuntimeError('No transfer pickup with usable continuation A/red')
                accepted['one' if name=='red' else 'two']=name
            with patch('robofrontier.suite.mosaic_oracle_v2.helpers.goal_coverage',return_value={'one':['A'],'two':['A']}), \
                 patch('robofrontier.suite.mosaic_oracle_v2.pickup_endpoint_candidates',return_value={'red':['A'],'blue':['A']}), \
                 patch('robofrontier.suite.mosaic_oracle_v2.deliver_cube',side_effect=delivery):
                result=solve(trial)
            self.assertEqual(seen,['red','blue','red'])
            self.assertEqual(result['reference_placed'],2)

    def test_admission_queue_stops_when_no_progress_is_possible(self):
        from unittest.mock import Mock
        with tempfile.TemporaryDirectory() as directory,self.helper_patches():
            trial,_=self.fake_trial(directory)
            trial.arms['A'].job=None;trial.arms['A'].grasp_force=None
            trial.held={};trial.actions=[];trial.finger_contacts=Mock(return_value=False)
            with patch('robofrontier.suite.mosaic_oracle_v2.deliver_cube',side_effect=RuntimeError('No transfer pickup with usable continuation A/cube')) as delivery:
                with self.assertRaisesRegex(RuntimeError,'No transfer pickup'):solve(trial)
            self.assertEqual(delivery.call_count,2)


if __name__=='__main__':unittest.main()
