import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch, Mock
from copy import copy

import mujoco
import numpy as np

from robofrontier.suite import transport_oracle_v2 as reference

ROOT = Path(__file__).resolve().parents[1]


def tasks():
    catalog = json.loads((ROOT / 'configs/suites/frontier-v2-r10.json').read_text())
    return [case for case in catalog['cases'] if case['family'] == 'transport']


def fixture_data():
    folder = ROOT / tasks()[0]['directory']
    return (json.loads((folder / 'task.json').read_text()),
            json.loads((folder / 'environment.json').read_text()))


class TransportReferenceTests(unittest.TestCase):
    def test_private_transfer_centers_do_not_recreate_task_buffers(self):
        trial=SimpleNamespace(finger_contacts=lambda *_:True,arms={'A':object(),'B':object()},env={'edges':[]},task={'cube_size_m':.06})
        before=json.dumps(trial.env)
        with patch.object(reference,'_planning_copy',side_effect=RuntimeError('Entered private screening')):
            with self.assertRaisesRegex(RuntimeError,'Entered private screening'):
                reference.plan_air_regrasp(trial,'A','part',candidate_centers={'B':[0.,0.,1.]})
        self.assertEqual(before,json.dumps(trial.env))
        for candidate in ({'A':[0.,0.,1.]},{'C':[0.,0.,1.]},{'B':[float('nan'),0.,1.]},{'B':[0.,1.]}):
            with self.subTest(candidate=candidate),self.assertRaises(ValueError):
                reference.plan_air_regrasp(trial,'A','part',candidate_centers=candidate)

    def test_r10_contract_without_physics(self):
        self.assertEqual(len(tasks()), 9)
        for case in tasks():
            with self.subTest(case=case['id']):
                folder = ROOT / case['directory']
                task = json.loads((folder / 'task.json').read_text())
                env = json.loads((folder / 'environment.json').read_text())
                result = reference.validate_inputs(task, env)
                self.assertEqual(result['objects'], result['goals'])
                self.assertEqual(result['goals'], case['arms'] * 3)
                self.assertEqual(result['table_height_m'], .8)

    def test_inventory_mismatch_stops_before_actuator_execution(self):
        task, env = fixture_data()
        env['objects'].pop()
        with patch.object(reference, '_baseline') as executor:
            with self.assertRaisesRegex(ValueError, 'inventory'):
                reference.solve(SimpleNamespace(task=task, env=env))
            executor.assert_not_called()

    def test_missing_relay_route_is_rejected(self):
        task, env = fixture_data()
        env['edges'] = []
        with self.assertRaisesRegex(ValueError, 'No declared transport route'):
            reference.validate_inputs(task, env)

    def test_supported_release_preserves_failed_feedback_and_gates_withdrawal(self):
        task, env = fixture_data()
        first = {'A': dict(status='failed', reason='Requested opening not reached')}
        retry = {'A': dict(status='completed')}
        for clears in (True, False):
            with self.subTest(clears=clears):
                execute = Mock(side_effect=[first, retry])
                t = SimpleNamespace(task=task, env=env, event=Mock(), pick=Mock(), place=Mock(),
                    execute=execute, held={'A':'A_item1'}, stop_requested=None,
                    arms={'A':SimpleNamespace(grip_max=.08, grasp_force=None)},
                    monitor=SimpleNamespace(violations=[], geoms={'A_item1':[]},
                        released=Mock(side_effect=[False, clears])))
                def baseline(current):
                    result = current.execute('place-relax', {'A':('gripper',dict(mode='move',width_m=.072))}, allow_failure=True)
                    self.assertIs(result, first)
                    current.event('withdraw_allowed')
                with patch.object(reference, '_baseline', side_effect=baseline):
                    if clears:
                        reference.solve(t)
                        self.assertIn(unittest.mock.call('withdraw_allowed'), t.event.call_args_list)
                    else:
                        with self.assertRaisesRegex(RuntimeError, 'not physically released before withdrawal'):
                            reference.solve(t)
                        self.assertNotIn(unittest.mock.call('withdraw_allowed'), t.event.call_args_list)
                self.assertEqual(execute.call_count, 2)
                self.assertEqual(execute.call_args_list[1].args[0], 'reference-release-clear')
                self.assertEqual(first['A']['status'], 'failed')

    def test_actual_capture_can_use_one_supported_unload_before_withdrawal(self):
        task, env = fixture_data()
        env['objects'][0]['shape'] = 'cylinder'
        first = {'A': dict(status='failed', reason='Requested opening not reached')}
        retry = {'A': dict(status='failed', reason='Requested opening not reached')}
        opened = {'A': dict(status='completed')}
        body = SimpleNamespace(xpos=np.array([0., 0., .83]), xmat=np.eye(3).ravel())
        t = SimpleNamespace(task=task, env=env, event=Mock(), pick=Mock(), place=Mock(),
            execute=Mock(side_effect=[first, retry, opened]), held={'A':'A_item1'},
            stop_requested=None, step=Mock(), wait=Mock(), move=Mock(),
            finger_contacts=lambda *_: True,
            data=SimpleNamespace(body=lambda _:body, site_xpos=np.array([[0.,0.,.85]]),
                site_xmat=np.eye(3).reshape(1,9), contact=[SimpleNamespace(
                    geom=[0,1], dist=-.00001, pos=np.array([0.,0.,.8]), frame=np.array([0.,0.,1.,1.,0.,0.,0.,1.,0.]))]),
            model=SimpleNamespace(geom_bodyid=[0,1],body=lambda _:SimpleNamespace(name='world')),
            arms={'A':SimpleNamespace(grip_max=.08, grasp_force=None, site=0,
                observe=lambda:dict(gripper_width_m=.057))},
            monitor=SimpleNamespace(violations=[],geoms={'A_item1':[0]},
                released=Mock(side_effect=[False,False,True,True])))
        def recorded_buffer_motion(*args, **kwargs):
            # The actual stopped buffer trace moved4.72mm transiently and
            # finished supported/released. This is not a final-slot error.
            body.xpos[0] += .00472
            t.step()
            body.xpos[0] -= .00472
        t.move.side_effect = recorded_buffer_motion
        def baseline(current):
            result=current.execute('place-relax',{'A':('gripper',dict(mode='move',width_m=.072))},allow_failure=True)
            self.assertIs(result,first)
            current.event('withdraw_allowed')
        with patch.object(reference,'_baseline',side_effect=baseline), patch.object(
                reference,'_released_buffer_evidence',return_value={'allowed':True}):
            reference.solve(t)
        self.assertEqual(t.execute.call_count,3)
        self.assertEqual(t.move.call_count,1)
        self.assertAlmostEqual(t.move.call_args.args[1][2],.849)
        self.assertIn(unittest.mock.call('withdraw_allowed'),t.event.call_args_list)
        self.assertEqual(first['A']['status'],'failed')
        self.assertGreater(next(c.kwargs['max_object_displacement_m'] for c in t.event.call_args_list
            if c.args[0]=='reference_supported_release_unload'), .003)
        t.execute = Mock(side_effect=[first,retry,opened])
        t.monitor.released = Mock(side_effect=[False,False,True])
        t.event.reset_mock()
        with patch.object(reference,'_baseline',side_effect=baseline), patch.object(
                reference,'_released_buffer_evidence',return_value={'allowed':False}):
            with self.assertRaisesRegex(RuntimeError,'not physically released before withdrawal'):
                reference.solve(t)
        self.assertNotIn(unittest.mock.call('withdraw_allowed'),t.event.call_args_list)

    def test_unload_recovery_is_not_applied_to_sockets_or_unsupported_objects(self):
        arm=SimpleNamespace(grip_max=.08,grasp_force=None,
            observe=lambda:dict(gripper_width_m=.057))
        for level,shape,supported in ((2,'cylinder',True),(1,'cube',True),(1,'cylinder',False)):
            with self.subTest(level=level,shape=shape,supported=supported):
                t=SimpleNamespace(task={'level':level},arms={'A':arm},
                    finger_contacts=lambda *_:True)
                execute=Mock()
                with patch.object(reference,'_table_supported_cylinder',return_value=supported):
                    self.assertFalse(reference._supported_release_unload(t,'A','item',{'item':{'shape':shape}},execute))
                execute.assert_not_called()

    def test_max_open_single_finger_separates_before_withdrawal(self):
        task,env=fixture_data();env['objects'][0]['shape']='cylinder'
        body=SimpleNamespace(xpos=np.array([0.,0.,.83]),xmat=np.eye(3).ravel())
        state={'distance':0.}
        first={'A':dict(status='completed')};retry={'A':dict(status='completed')}
        t=SimpleNamespace(task=task,env=env,event=Mock(),pick=Mock(),place=Mock(),
            execute=Mock(side_effect=[first,retry]),held={'A':'A_item1'},stop_requested=None,
            step=Mock(),wait=Mock(),move=Mock(),finger_contacts=lambda *_:False,
            arms={'A':SimpleNamespace(grip_max=.08,grasp_force=None,site=0,
                observe=lambda:dict(gripper_width_m=.07935))},
            data=SimpleNamespace(body=lambda _:body,site_xpos=np.array([[0.,0.,.85]]),
                site_xmat=np.eye(3).reshape(1,9),contact=[SimpleNamespace(geom=[0,1],dist=-.00018)]),
            model=SimpleNamespace(geom_bodyid=[0,1],body=lambda i:SimpleNamespace(
                name='A_item1' if i==0 else 'A_left_finger')),
            monitor=SimpleNamespace(violations=[],geoms={'A_item1':[0]},
                released=lambda _:state['distance']>=.005))
        def finite_motion(robot,point,*args,**kwargs):
            state['distance']=float(point[1]);t.data.site_xpos[0]=point
            body.xpos[1]=min(state['distance'],.00173);t.step()
        t.move.side_effect=finite_motion
        def baseline(current):
            current.execute('place-relax',{'A':('gripper',dict(mode='move',width_m=.072))},allow_failure=True)
            current.event('withdraw_allowed')
        with patch.object(reference,'_baseline',side_effect=baseline), patch.object(
                reference,'_table_supported_cylinder',return_value=True), patch.object(
                reference,'_supported_separation_clear',return_value=True,create=True), patch.object(
                reference,'_released_buffer_evidence',return_value={'allowed':True}):
            reference.solve(t)
        self.assertEqual(t.move.call_count,2)
        self.assertAlmostEqual(t.move.call_args_list[0].args[1][1],.002)
        self.assertAlmostEqual(t.move.call_args_list[1].args[1][1],.005)
        self.assertIn(unittest.mock.call('withdraw_allowed'),t.event.call_args_list)

    def test_single_finger_recovery_excludes_socket_force_or_nonmaximum_opening(self):
        for level,force,width in ((2,None,.08),(1,50.,.08),(1,None,.057)):
            with self.subTest(level=level,force=force,width=width):
                t=SimpleNamespace(task={'level':level},arms={'A':SimpleNamespace(
                    grasp_force=force,grip_max=.08,observe=lambda:dict(gripper_width_m=width))})
                self.assertFalse(reference._supported_release_separate(t,'A','item',{'item':{'shape':'cylinder'}}))

    def test_single_contact_direction_comes_from_current_finger_and_tool_frame(self):
        bodies=['item','Q_left_finger','Q_right_finger','R_hand']
        t=SimpleNamespace(arms={'Q':SimpleNamespace(site=0)},
            monitor=SimpleNamespace(geoms={'item':[0]}),
            model=SimpleNamespace(geom_bodyid=list(range(4)),body=lambda i:SimpleNamespace(name=bodies[i])),
            data=SimpleNamespace(site_xmat=np.eye(3).reshape(1,9)))
        for contacts,expected in (([1],[0,1,0]),([2],[0,-1,0]),([1,2],None),([3],None),([],None)):
            t.data.contact=[SimpleNamespace(geom=[0,i],dist=-.0001) for i in contacts]
            direction=reference._single_finger_axis(t,'Q','item')
            if expected is None:self.assertIsNone(direction)
            else:np.testing.assert_allclose(direction,expected)

    def test_single_contact_axis_can_explicitly_include_verified_receiver_fingers(self):
        bodies=['item','G_right_finger','F_left_finger','F_right_finger','F_hand']
        t=SimpleNamespace(arms={'G':SimpleNamespace(site=0)},
            monitor=SimpleNamespace(geoms={'item':[0]}),
            model=SimpleNamespace(geom_bodyid=list(range(5)),body=lambda i:SimpleNamespace(name=bodies[i])),
            data=SimpleNamespace(site_xmat=np.eye(3).reshape(1,9),
                contact=[SimpleNamespace(geom=[0,i],dist=-.0001) for i in (1,2,3)]))
        self.assertIsNone(reference._single_finger_axis(t,'G','item'))
        np.testing.assert_allclose(reference._single_finger_axis(t,'G','item',supporting_robots=('F',)),[0,-1,0])
        t.data.contact.append(SimpleNamespace(geom=[0,4],dist=-.0001))
        self.assertIsNone(reference._single_finger_axis(t,'G','item',supporting_robots=('F',)))

    def test_supported_cylinder_height_includes_real_blade_extension(self):
        for level,z,expected in ((1,.83,True),(2,.854,True),(2,.83,False)):
            t=SimpleNamespace(task={'level':level,'table_height_m':.8,'cube_size_m':.06},
                env={'mechanism':{'blade_size_m':[.0025,.018,.024]}},
                data=SimpleNamespace(body=lambda _:SimpleNamespace(xmat=np.eye(3).ravel(),xpos=np.array([0.,0.,z])),
                    contact=[SimpleNamespace(geom=[0,1],dist=-.00001,pos=np.array([0.,0.,.8]),frame=np.array([0.,0.,1.]))]),
                model=SimpleNamespace(geom_bodyid=[0,1],body=lambda _:SimpleNamespace(name='world')),
                monitor=SimpleNamespace(geoms={'item':[0]}))
            self.assertEqual(reference._table_supported_cylinder(t,'item'),expected)

    def test_assist_candidate_is_not_applied_to_other_levels_shapes_or_active_grasps(self):
        for level,shape,force,width in ((1,'cylinder',None,.08),(2,'cube',None,.08),
                                       (2,'cylinder',30.,.08),(2,'cylinder',None,.057)):
            t=SimpleNamespace(task={'level':level},arms={'A':SimpleNamespace(job=None,
                grasp_force=force,grip_max=.08,observe=lambda:dict(gripper_width_m=width))})
            with patch.object(reference,'_plan_supported_side_assist') as planner:
                self.assertFalse(reference._supported_release_assist(t,'A','item',{'item':{'shape':shape}}))
                planner.assert_not_called()

    def test_supported_assist_requires_explicit_reference_opt_in(self):
        for enabled in (False,True):
            task,env=fixture_data()
            first={'A':dict(status='failed',reason='Requested opening not reached')}
            retry={'A':dict(status='completed')}
            t=SimpleNamespace(task=task,env=env,event=Mock(),pick=Mock(),place=Mock(),
                execute=Mock(side_effect=[first,retry]),held={'A':'A_item1'},stop_requested=None,
                arms={'A':SimpleNamespace(grip_max=.08,grasp_force=None)},
                monitor=SimpleNamespace(violations=[],geoms={'A_item1':[]},released=lambda _:False))
            def baseline(current):
                result=current.execute('place-relax',{'A':('gripper',dict(mode='move',width_m=.072))},allow_failure=True)
                self.assertIs(result,first)
                current.event('withdraw_allowed')
            with patch.object(reference,'_baseline',side_effect=baseline), \
                 patch.object(reference,'_supported_release_unload',return_value=False), \
                 patch.object(reference,'_supported_release_separate',return_value=False), \
                 patch.object(reference,'_supported_release_assist',return_value=True) as assist:
                if enabled:reference.solve(t,assisted_release=True)
                else:
                    with self.assertRaisesRegex(RuntimeError,'not physically released'):
                        reference.solve(t)
                self.assertEqual(assist.call_count,int(enabled))
                self.assertEqual(first['A']['status'],'failed')

    def test_forward_queue_continues_independent_work_and_resumes_without_reclearing(self):
        assignments={};seen=[]
        objects=[dict(id='one',identity='red',source='A'),dict(id='two',identity='blue',source='B')]
        goals=[dict(goal_id='g1',requires={'identity':'red'},robot='A',board='boardA',local_position_m=[0,0,0]),
               dict(goal_id='g2',requires={'identity':'blue'},robot='B',board='boardB',local_position_m=[0,0,0])]
        t=SimpleNamespace(task={'level':2,'goals':goals,
                'arms':{r:{'base_position_m':[0,0,.8]} for r in ('A','B')}},
            env={'objects':objects,'stations':[{'id':'A'},{'id':'B'}],
                 'edges':[{'stations':['A','B'],'position_m':[0,0,.8]}]},
            arms={r:SimpleNamespace(grasp_force=None) for r in ('A','B')},held={},
            event=Mock(),wait=Mock(),check_stop=Mock(),
            data=SimpleNamespace(body=lambda _:SimpleNamespace(xpos=np.zeros(3),xmat=np.eye(3))),
            monitor=SimpleNamespace(result=lambda:dict(assignments=dict(assignments),matched_goals=len(assignments))))
        t.transport_yield_on_deferral=True
        def coverage(_,items=None):
            return {o['id']:[o['source']] for o in (objects if items is None else items)}
        def execute_job(current,name,*args):
            seen.append(name)
            if name=='one' and seen.count('one')==1:
                current.held['A']='one'
                raise RuntimeError('Payload not physically released before withdrawal A/one')
            assignments['g1' if name=='one' else 'g2']=name
        with patch('robofrontier.suite.placement_oracle.clear_idle_arms') as clear, \
             patch('robofrontier.suite.placement_oracle.deliver',side_effect=execute_job), \
             patch.object(reference,'_pickup_coverage',side_effect=coverage), \
             patch.object(reference,'_occupied_transport_robots',side_effect=lambda _:set(t.held)), \
             patch.object(reference,'_safe_transport_deferral',return_value=True), \
             patch.object(reference,'_robot_payload_contacts',side_effect=lambda *_:bool(t.held)):
            first=reference._forward_baseline(t)
            self.assertEqual(seen,['one'])
            self.assertEqual(assignments,{})
            self.assertFalse(t.transport_yield_on_deferral)
            self.assertEqual({g['goal_id'] for g,_,_ in t.transport_pending_jobs},{'g1','g2'})
            # No compensation was asserted: the occupied first robot stays
            # untouched, while the saved independent objective can advance.
            first=reference._forward_baseline(t)
            self.assertEqual(seen,['one','two'])
            self.assertTrue(first['needs_recovery'])
            self.assertEqual(assignments,{'g2':'two'})
            # A separate verified compensation clears actual contact; the
            # same live queue may continue without resetting other robots.
            t.held.clear()
            second=reference._forward_baseline(t)
            self.assertFalse(second['needs_recovery'])
            self.assertEqual(seen,['one','two','one'])
            self.assertEqual(clear.call_count,1)

    def test_forward_deferral_never_swallows_unknown_or_prohibited_history(self):
        for violations,job in (([], 'unknown-action'),([{'kind':'inter_robot_contact'}],None)):
            t=SimpleNamespace(monitor=SimpleNamespace(violations=violations),
                arms={'A':SimpleNamespace(job=job,uncertainty_reported=False,grasp_force=None)},held={})
            self.assertFalse(reference._safe_transport_deferral(t,RuntimeError('No pickup pose A/item')))

    def test_forward_deferral_requires_settled_table_support_for_retained_contact(self):
        arm=SimpleNamespace(job=None,uncertainty_reported=False,grasp_force=None)
        velocity=np.zeros(6)
        t=SimpleNamespace(monitor=SimpleNamespace(violations=[]),arms={'A':arm},
            held={'A':'item'},data=SimpleNamespace(joint=lambda _:SimpleNamespace(qvel=velocity)))
        error=RuntimeError('Payload not physically released before withdrawal A/item')
        with patch.object(reference,'_table_supported_cylinder',return_value=True):
            self.assertTrue(reference._safe_transport_deferral(t,error))
            velocity[3]=.1
            self.assertFalse(reference._safe_transport_deferral(t,error))
            velocity[:]=0.;arm.grasp_force=30.
            self.assertFalse(reference._safe_transport_deferral(t,error))
        arm.grasp_force=None
        with patch.object(reference,'_table_supported_cylinder',return_value=False):
            self.assertFalse(reference._safe_transport_deferral(t,error))

    def test_forward_queue_refuses_blanket_reinitialization_of_an_existing_live_world(self):
        t=SimpleNamespace(task={'goals':[]},transport_goal_ids=None,
            actions=[{'label':'already_executed'}],held={},
            arms={'A':SimpleNamespace(grasp_force=None)})
        with patch('robofrontier.suite.placement_oracle.clear_idle_arms') as clear:
            with self.assertRaisesRegex(RuntimeError,'refusing a blanket robot reset'):
                reference._forward_baseline(t)
        clear.assert_not_called()

    def test_forward_entry_does_not_overwrite_an_unresolved_action(self):
        t=SimpleNamespace(arms={'A':SimpleNamespace(job='unresolved',uncertainty_reported=True)})
        with patch('robofrontier.suite.placement_oracle.clear_idle_arms') as clear:
            with self.assertRaisesRegex(RuntimeError,'settled verified action results'):
                reference._forward_baseline(t)
        clear.assert_not_called()

    def test_forward_resume_reserves_actual_accepted_objects_before_assigning_remaining_work(self):
        assignments={'g1':'two'};seen=[]
        objects=[dict(id='one',identity='red',source='A'),dict(id='two',identity='red',source='B')]
        goals=[dict(goal_id='g1',requires={'identity':'red'},robot='A',board='boardA',local_position_m=[0,0,0]),
               dict(goal_id='g2',requires={'identity':'red'},robot='B',board='boardB',local_position_m=[0,0,0])]
        t=SimpleNamespace(task={'level':2,'goals':goals,'arms':{r:{'base_position_m':[0,0,.8]} for r in ('A','B')}},
            env={'objects':objects,'stations':[{'id':'A'},{'id':'B'}],
                 'edges':[{'stations':['A','B'],'position_m':[0,0,.8]}]},
            arms={r:SimpleNamespace(grasp_force=None) for r in ('A','B')},held={},parking_q={},
            event=Mock(),wait=Mock(),check_stop=Mock(),
            data=SimpleNamespace(body=lambda _:SimpleNamespace(xpos=np.zeros(3),xmat=np.eye(3))),
            monitor=SimpleNamespace(result=lambda:dict(assignments=dict(assignments),matched_goals=len(assignments))))
        def execute(current,name,*args):seen.append(name);assignments['g2']=name
        with patch('robofrontier.suite.placement_oracle.clear_idle_arms') as clear, \
             patch('robofrontier.suite.placement_oracle.deliver',side_effect=execute), \
             patch.object(reference,'_pickup_coverage',return_value={'one':['A'],'two':['B']}), \
             patch.object(reference,'_occupied_transport_robots',return_value=set()):
            result=reference._forward_baseline(t)
        self.assertFalse(result['needs_recovery'])
        self.assertEqual(seen,['one'])
        clear.assert_not_called()

    def test_unhandled_boundary_keeps_in_progress_goal_for_verified_live_recovery(self):
        obj={'id':'one','identity':'red','source':'A'}
        goal={'goal_id':'g1','requires':{'identity':'red'},'robot':'A','board':'boardA','local_position_m':[0,0,0]}
        t=SimpleNamespace(task={'level':2,'goals':[goal],'arms':{'A':{'base_position_m':[0,0,.8]}}},
            env={'objects':[obj],'stations':[{'id':'A'}],'edges':[]},
            arms={'A':SimpleNamespace(grasp_force=None)},held={},
            event=Mock(),wait=Mock(),check_stop=Mock(),
            data=SimpleNamespace(body=lambda _:SimpleNamespace(xpos=np.zeros(3),xmat=np.eye(3))),
            monitor=SimpleNamespace(result=lambda:{'assignments':{}}))
        with patch('robofrontier.suite.placement_oracle.clear_idle_arms'), \
             patch('robofrontier.suite.placement_oracle.deliver',side_effect=RuntimeError('No common reachable transit height A/one')), \
             patch.object(reference,'_pickup_coverage',return_value={'one':['A']}), \
             patch.object(reference,'_occupied_transport_robots',return_value=set()), \
             patch.object(reference,'_safe_transport_deferral',return_value=False):
            with self.assertRaisesRegex(RuntimeError,'No common reachable transit height'):
                reference._forward_baseline(t)
        self.assertEqual(t.transport_pending_jobs[0][0]['goal_id'],'g1')
        self.assertEqual(t.transport_last_unhandled_boundary['object'],'one')

    def test_separation_depth_compares_same_screen_model_baseline(self):
        """Inflated screen contacts cannot use a live-model depth baseline."""
        model=SimpleNamespace(geom_bodyid=[0,1],
            body=lambda i:SimpleNamespace(name=('item','A_right_finger')[i]))
        arm=SimpleNamespace(site=0,qids=np.arange(7))
        data=SimpleNamespace(qpos=np.zeros(7),site_xpos=np.zeros((1,3)),
            contact=[SimpleNamespace(geom=[0,1],dist=-.0004)])
        trial=SimpleNamespace(model=model,data=data,arms={'A':arm},
            monitor=SimpleNamespace(geoms={'item':[0]}))
        screen=SimpleNamespace(contact=[])
        scratch=SimpleNamespace(model=model,data=data,screen_data=screen,
            arms={'A':arm},monitor=trial.monitor,held={})
        scratch.screen_ik=lambda _,point,*args,**kwargs:np.r_[point[0],np.zeros(6)]
        def pose_clear(_,q,*args,**kwargs):
            screen.contact=[SimpleNamespace(geom=[0,1],dist=-.0018+q[0])]
            return True
        scratch.pose_clear=pose_clear
        with patch.object(reference,'_planning_copy',return_value=scratch):
            self.assertTrue(reference._supported_separation_clear(
                trial,'A','item',np.array([.002,0.,0.]),np.eye(3)))
            self.assertFalse(reference._supported_separation_clear(
                trial,'A','item',np.array([-.0005,0.,0.]),np.eye(3)))

    def test_unexpected_opening_fault_never_triggers_release_recovery(self):
        task,env=fixture_data();env['objects'][0]['shape']='cylinder'
        for status,reason in (('uncertain','Transport timeout'),('failed','Unexpected driver fault')):
            t=SimpleNamespace(task=task,env=env,event=Mock(),pick=Mock(),place=Mock(),
                execute=Mock(side_effect=[{'A':dict(status='completed')},{'A':dict(status=status,reason=reason)}]),
                held={'A':'A_item1'},stop_requested=None,
                arms={'A':SimpleNamespace(grip_max=.08,grasp_force=None)},
                monitor=SimpleNamespace(violations=[],geoms={'A_item1':[0]},released=lambda _:False))
            def baseline(current):
                current.execute('place-relax',{'A':('gripper',dict(mode='move',width_m=.072))},allow_failure=True)
            with patch.object(reference,'_baseline',side_effect=baseline), patch.object(
                    reference,'_supported_release_unload') as unload, patch.object(
                    reference,'_supported_release_separate') as separate:
                with self.assertRaisesRegex(RuntimeError,'not physically released'):
                    reference.solve(t)
                unload.assert_not_called();separate.assert_not_called()

    def test_private_pickup_coverage_never_invents_declared_source(self):
        item = dict(id='item', source='A', shape='cube')
        body = SimpleNamespace(xpos=np.array([0., 0., .83]), xmat=np.eye(3).ravel())
        arms = {r: SimpleNamespace(grip_max=.08, qids=np.arange(7)) for r in ('A', 'B')}
        for feasible, expected in (({'B'}, ['B']), (set(), []), ({'A'}, ['A'])):
            with self.subTest(feasible=feasible):
                t = SimpleNamespace(env={'objects': [item]}, arms=arms,
                    data=SimpleNamespace(body=lambda _: body, qpos=np.zeros(7)),
                    task=dict(level=1, cube_size_m=.06, arms={
                        r: dict(base_position_m=[0., 0., .8]) for r in arms}),
                    home_q={r: np.zeros(7) for r in arms},
                    screen_ik=lambda robot, *args, **kwargs: np.zeros(7) if robot in feasible else None,
                    pose_clear=lambda *args, **kwargs: True)
                with patch.object(reference, '_planning_copy', return_value=t), patch.object(reference, '_scratch_gap'):
                    self.assertEqual(reference._pickup_coverage(t)['item'], expected)

    def test_private_pickup_coverage_checks_current_seed_used_by_real_picker(self):
        item=dict(id='item',source='A',shape='cube')
        body=SimpleNamespace(xpos=np.array([0.,0.,.83]),xmat=np.eye(3).ravel())
        arm=SimpleNamespace(grip_max=.08,qids=np.arange(7))
        t=SimpleNamespace(env={'objects':[item]},arms={'A':arm},
            data=SimpleNamespace(body=lambda _:body,qpos=np.zeros(7)),
            task=dict(level=1,cube_size_m=.06,arms={'A':dict(base_position_m=[0.,0.,.8])}),
            home_q={'A':np.ones(7)},pose_clear=lambda *args,**kwargs:True,
            screen_ik=lambda *args,seed=None,**kwargs:np.zeros(7) if np.all(seed==0.) else None)
        with patch.object(reference,'_planning_copy',return_value=t),patch.object(reference,'_scratch_gap'):
            self.assertEqual(reference._pickup_coverage(t)['item'],['A'])

    def test_small_placement_repair_does_not_retrieve_distant_or_dropped_objects(self):
        board = SimpleNamespace(xpos=np.zeros(3), xmat=np.eye(3).ravel())
        obj = SimpleNamespace(xpos=np.array([.2, 0, .03]), xmat=np.eye(3).ravel())
        t = SimpleNamespace(data=SimpleNamespace(body=lambda name: board if name == 'board' else obj),
            monitor=SimpleNamespace(geoms={'item': []}, released=lambda _: True, result=lambda: {'assignments': {}}),
            pick=Mock(), place=Mock())
        goal = dict(board='board', local_position_m=[0,0,.03], robot='A', goal_id='slot')
        result = reference._repair_small_l1_placement(t, goal, 'item')
        self.assertEqual(result, {'assignments': {}})
        t.pick.assert_not_called()
        t.place.assert_not_called()

    def test_radial_cylinder_grasp_does_not_follow_roll_clocking(self):
        from robofrontier.suite.scenes import rz
        axis_x = np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
        up, frames = reference._pickup_frames('cylinder', axis_x)
        rolled_up, rolled_frames = reference._pickup_frames('cylinder', axis_x@rz(.73))
        np.testing.assert_allclose(up, [0.,0.,1.])
        np.testing.assert_allclose(rolled_up, up)
        for a, b in zip(frames, rolled_frames):
            np.testing.assert_allclose(a, b)
            np.testing.assert_allclose(a.T@a, np.eye(3))
            self.assertAlmostEqual(float(np.dot(a[:,1], axis_x[:,2])), 0.)

    def test_hand_prefilter_uses_proxy_geometry_and_preserves_source(self):
        from robofrontier.workstation import Workstation
        from robofrontier.suite.evaluate import Monitor
        from robofrontier.robot_control import PandaArm
        folder = ROOT / tasks()[0]['directory']
        source = Workstation(folder)
        source.frame()
        task, env = fixture_data()
        t = SimpleNamespace(model=source.model, data=source.data, task=task, env=env,
            arms={r: PandaArm(source.model, source.data, r) for r in task['arms']},
            _reference_planning_only=True)
        t.monitor = Monitor(t)
        before_q = t.data.qpos.copy()
        before_geom = t.data.geom_xpos.copy()
        site = t.arms['A'].site
        point = t.data.site_xpos[site].copy()
        tool = t.data.site_xmat[site].reshape(3, 3).copy()
        self.assertFalse(reference._hand_geometry_clear(t, 'A', 'B', 'A_item1', point, tool, .06))
        self.assertTrue(reference._hand_geometry_clear(t, 'A', 'B', 'A_item1', point+np.array([3., 0, 0]), tool, .06))
        np.testing.assert_array_equal(t.data.qpos, before_q)
        np.testing.assert_array_equal(t.data.geom_xpos, before_geom)
        t._reference_planning_only = False
        with self.assertRaisesRegex(RuntimeError, 'planning-only'):
            reference._hand_geometry_clear(t, 'A', 'B', 'A_item1', point, tool, .06)

    def test_planning_copy_never_shares_live_state_or_held_map(self):
        from robofrontier.workstation import Workstation
        from robofrontier.suite.oracle import Trial
        from robofrontier.robot_control import PandaArm
        folder = ROOT / tasks()[0]['directory']
        source = Workstation(folder)
        source.frame()
        t = Trial.__new__(Trial)
        t.model, t.data = source.model, source.data
        t.task, t.env = fixture_data()
        t.arms = {r: PandaArm(t.model, t.data, r) for r in t.task['arms']}
        t.screen_model = copy(t.model)
        t.held = {'A': 'A_item1'}
        before = t.data.qpos.copy()
        scratch = reference._planning_copy(t)
        scratch.data.qpos[0] += .1
        scratch.held.clear()
        np.testing.assert_array_equal(t.data.qpos, before)
        self.assertEqual(t.held, {'A': 'A_item1'})
        self.assertIsNot(scratch.data, t.data)
        self.assertIsNot(scratch.model, t.model)
        self.assertFalse(scratch.live_preview)
        with self.assertRaisesRegex(RuntimeError, 'Planning scratch cannot'):
            scratch.event('candidate')

    def test_socket_scratch_clearance_restores_and_never_edits_live_model(self):
        from robofrontier.workstation import Workstation
        from robofrontier.suite.evaluate import Monitor
        from robofrontier.robot_control import PandaArm
        case = next(c for c in tasks() if c['id'] == 'transport-a04-d2')
        folder = ROOT / case['directory']
        source = Workstation(folder)
        source.frame()
        task = json.loads((folder / 'task.json').read_text())
        t = SimpleNamespace(model=source.model, data=source.data, task=task,
            arms={r: PandaArm(source.model, source.data, r) for r in task['arms']})
        t.monitor = Monitor(t)
        t.screen_model = copy(t.model)
        t.screen_model.geom_margin[:] = .004
        before = t.model.geom_margin.copy()
        pin = t.model.geom('C_item2_blade0').id
        hand = next(g for g in range(t.model.ngeom)
                    if t.model.body(int(t.model.geom_bodyid[g])).name == 'A_hand')
        with self.assertRaisesRegex(RuntimeError, 'test exit'):
            with reference.socket_contact_clearance(t, 'C_item2', 'board_A'):
                self.assertEqual(t.screen_model.geom_margin[pin], before[pin])
                self.assertEqual(t.screen_model.geom_margin[hand], .004)
                np.testing.assert_array_equal(t.model.geom_margin, before)
                raise RuntimeError('test exit')
        self.assertEqual(t.screen_model.geom_margin[pin], .004)
        np.testing.assert_array_equal(t.model.geom_margin, before)

    def test_air_planner_rejects_stale_logical_held_state(self):
        t = SimpleNamespace(finger_contacts=lambda *args: False)
        with self.assertRaisesRegex(RuntimeError, 'requires retained donor payload'):
            reference.plan_air_regrasp(t, 'A', 'object')

    def test_gripper_geometry_sweep_rejects_live_trial(self):
        with self.assertRaisesRegex(RuntimeError, 'requires planning-only data'):
            reference._scratch_gap(SimpleNamespace(), 'A', .08)

    def test_release_sweep_rejects_mislabeled_live_data_before_any_write(self):
        data = SimpleNamespace(qpos=np.array([1.,2.,3.]))
        live = SimpleNamespace(data=data, _live_data=data, _reference_planning_only=True)
        before = data.qpos.copy()
        with self.assertRaisesRegex(RuntimeError, 'independent planning-only'):
            reference._release_sweep_clear(live, 'A', 'B', 'object', np.zeros(7), .06)
        with self.assertRaisesRegex(RuntimeError, 'requires planning-only'):
            reference._scratch_gap(live, 'A', .08)
        np.testing.assert_array_equal(data.qpos, before)

    def test_receiver_missing_contact_never_releases_donor(self):
        data = SimpleNamespace(body=lambda _: SimpleNamespace(
            xmat=np.eye(3).ravel(), xpos=np.array([0., 0., 1.])))
        labels = []
        t = SimpleNamespace(data=data, task={'cube_size_m': .06},
            env={'objects': [{'id': 'object', 'shape': 'cube'}]},
            cfg={'allowed_grasp_force_n': [30., 70.]},
            arms={'B': SimpleNamespace(grip_max=.08)},
            event=Mock(), move_free=Mock(), move=Mock(), wait=Mock(),
            execute=lambda label, commands, **kw: labels.append(label),
            finger_contacts=lambda robot, name: robot == 'A', held={'A': 'object'})
        plan = dict(receiver='B', donor_point=np.zeros(3), donor_tool=np.eye(3),
            donor_q=np.zeros(7), receiver_axis=0, receiver_sign=1,
            receiver_depth=.024, receiver_shift=0.)
        with patch.object(reference, 'plan_air_regrasp', return_value=plan):
            with self.assertRaisesRegex(RuntimeError, 'actual bilateral support'):
                reference.regrasp_held_plug(t, 'A', 'object')
        self.assertNotIn('regrasp-donor-release', labels)
        self.assertEqual(t.held, {'A': 'object'})

    def test_turned_donor_retracts_backwards_instead_of_world_up(self):
        horizontal_tool = np.array([[0.,0,1.],[1.,0,0],[0,1.,0]])
        data = SimpleNamespace(body=lambda _: SimpleNamespace(
            xmat=np.eye(3).ravel(), xpos=np.array([0.,0.,1.])),
            site_xpos=np.array([[.5,.2,1.], [0.,0.,1.]]),
            site_xmat=np.array([horizontal_tool.ravel(), np.eye(3).ravel()]), contact=[])
        pose_command = Mock(return_value=('move', {}))
        t = SimpleNamespace(data=data, task={'cube_size_m':.06,'table_height_m':.8},
            env={'objects':[{'id':'object','shape':'cube'}],
                 'mechanism':{'blade_size_m':[.0025,.012,.024]}},
            cfg={'allowed_grasp_force_n':[30.,70.]},
            arms={'A':SimpleNamespace(site=0,grip_max=.08,grasp_force=None,
                                         observe=lambda:{'gripper_width_m':.08}),
                  'B':SimpleNamespace(site=1,grip_max=.08,grasp_force=None)},
            event=Mock(), move_free=Mock(), move=Mock(), wait=Mock(), place=Mock(),
            execute=lambda label, commands, **kw:{r:{'status':'completed'} for r in commands},
            pose_command=pose_command, finger_contacts=lambda *args:True,
            monitor=SimpleNamespace(geoms={'object':[]},geom_owner=[]),
            held={'A':'object'})
        plan=dict(receiver='B',donor_point=np.zeros(3),donor_tool=np.eye(3),
            donor_q=np.zeros(7),receiver_axis=0,receiver_sign=1,
            receiver_depth=.024,receiver_shift=0.,buffer_xy=np.zeros(2))
        with patch.object(reference,'plan_air_regrasp',return_value=plan), \
             patch('robofrontier.suite.placement_oracle.park'):
            reference.regrasp_held_plug(t,'A','object')
        retreat = next(call for call in t.move_free.call_args_list
                       if len(call.args)>3 and call.args[3]=='regrasp-send-clear')
        np.testing.assert_allclose(retreat.args[1], [.34,.2,1.])
        self.assertNotIn('continuation', retreat.kwargs)
        self.assertEqual(t.held, {'B': 'object'})

    def test_air_regrasp_can_return_actual_receiver_without_buffer_or_donor_repick(self):
        data=SimpleNamespace(body=lambda _:SimpleNamespace(
            xmat=np.eye(3).ravel(),xpos=np.array([0.,0.,1.])),
            site_xpos=np.array([[.5,.2,1.],[0.,0.,1.]]),
            site_xmat=np.array([np.eye(3).ravel(),np.eye(3).ravel()]),contact=[])
        t=SimpleNamespace(data=data,task={'cube_size_m':.06,'table_height_m':.8},
            env={'objects':[{'id':'object','shape':'cube'}],
                 'mechanism':{'blade_size_m':[.0025,.012,.024]}},
            cfg={'allowed_grasp_force_n':[30.,70.]},
            arms={'A':SimpleNamespace(site=0,grip_max=.08,grasp_force=None,
                    observe=lambda:{'gripper_width_m':.08}),
                  'B':SimpleNamespace(site=1,grip_max=.08,grasp_force=None)},
            event=Mock(),move_free=Mock(),move=Mock(),wait=Mock(),place=Mock(),
            execute=lambda label,commands,**kw:{r:{'status':'completed'} for r in commands},
            finger_contacts=lambda *args:True,
            monitor=SimpleNamespace(geoms={'object':[]},geom_owner=[]),held={'A':'object'})
        plan=dict(receiver='B',donor_point=np.zeros(3),donor_tool=np.eye(3),
            donor_q=np.zeros(7),receiver_axis=0,receiver_sign=1,
            receiver_depth=.024,receiver_shift=0.,buffer_xy=np.zeros(2))
        with patch.object(reference,'plan_air_regrasp',return_value=plan), \
             patch('robofrontier.suite.placement_oracle.park') as park:
            receiver=reference.regrasp_held_plug(t,'A','object',keep_receiver_held=True)
        self.assertEqual(receiver,'B')
        self.assertEqual(t.held,{'B':'object'})
        t.place.assert_not_called()
        park.assert_called_once_with(t,'A')

    def test_air_regrasp_receiver_filter_rejects_unsupported_candidate_centers(self):
        task,env=fixture_data()
        t=SimpleNamespace(env=env,finger_contacts=lambda *_:True)
        with patch.object(reference,'_planning_copy') as scratch:
            self.assertIsNone(reference.plan_air_regrasp(t,'A','object',allowed_receivers=[]))
            with self.assertRaisesRegex(ValueError,'declared transfer-center'):
                reference.plan_air_regrasp(t,'A','object',allowed_receivers=['C'])
        scratch.assert_not_called()

    def test_cylinder_speed_policy_does_not_mutate_or_slow_empty_arms(self):
        commands = {'A': ('move', {'joint_speed_rad_s': .45}),
                    'B': ('move', {'joint_speed_rad_s': .45})}
        adjusted = reference.cylinder_commands(commands, {'A': 'cylinder'},
                                                {'cylinder': {'shape': 'cylinder'}})
        self.assertEqual(adjusted['A'][1]['joint_speed_rad_s'], .18)
        self.assertEqual(adjusted['B'][1]['joint_speed_rad_s'], .45)
        self.assertEqual(commands['A'][1]['joint_speed_rad_s'], .45)

    def test_first_cylinder_lift_is_slow_even_before_held_bookkeeping(self):
        task, env = fixture_data()
        env['objects'][0]['shape'] = 'cylinder'
        name = env['objects'][0]['id']
        arm = SimpleNamespace(grasp_force=None)
        execute = Mock(return_value={})
        t = SimpleNamespace(task=task, env=env, event=Mock(), execute=execute,
            place=Mock(), held={}, arms={'A': arm}, stop_requested=None,
            cfg={'allowed_grasp_force_n': [30., 70.]},
            monitor=SimpleNamespace(violations=[]))
        def move(label):
            t.execute(label, {'A': ('move', dict(speed_m_s=.1))})
        def original_pick(robot, item, **kwargs):
            move('empty-approach')
            arm.grasp_force = 70.
            move('first-lift')
            self.assertEqual(t.held, {})
            raise RuntimeError('pickup interrupted')
        t.pick = original_pick
        def baseline(current):
            with self.assertRaisesRegex(RuntimeError, 'pickup interrupted'):
                current.pick('A', name)
            # A stopped pickup must not leave stale pending ownership.
            move('after-pickup-exit')
        with patch.object(reference, '_baseline', side_effect=baseline):
            reference.solve(t)
        self.assertEqual([c.args[1]['A'][1]['speed_m_s']
                          for c in execute.call_args_list], [.1, .025, .1])
        self.assertIs(t.pick, original_pick)

    def test_new_process_violation_triggers_stop_and_restores_budget_callback(self):
        task, env = fixture_data()
        original_stop = lambda: False
        t = SimpleNamespace(task=task,env=env,event=Mock(),
            pick=Mock(),execute=Mock(),place=Mock(),
            monitor=SimpleNamespace(violations=[]),stop_requested=original_stop)
        def baseline(current):
            self.assertFalse(current.stop_requested())
            current.monitor.violations.append({'kind':'inter_robot_contact'})
            self.assertTrue(current.stop_requested())
        with patch.object(reference,'_baseline',side_effect=baseline):
            reference.solve(t)
        self.assertIs(t.stop_requested, original_stop)

    def test_delegates_after_board_relative_contract_check(self):
        task, env = fixture_data()
        events = []
        trial = SimpleNamespace(task=task, env=env,
                                event=lambda kind, **fields: events.append((kind, fields)),
                                pick=lambda *a, **k: None, execute=lambda *a, **k: None,
                                place=lambda *a, **k: None)
        with patch.object(reference, '_baseline', return_value='done') as executor:
            self.assertEqual(reference.solve(trial), 'done')
            executor.assert_called_once_with(trial)
        self.assertEqual(events[0][0], 'transport_reference_contract')
        self.assertEqual(events[0][1]['table_height_m'], .8)

    def test_live_frontier_adapter_gets_same_trial_and_resumes_without_rewrapping(self):
        task,env=fixture_data()
        trial=SimpleNamespace(task=task,env=env,event=Mock(),
            pick=Mock(),execute=Mock(),place=Mock(),held={},
            monitor=SimpleNamespace(violations=[]),
            arms={'A':SimpleNamespace(job=None,uncertainty_reported=False)})
        original_pick=trial.pick;blocked={'needs_recovery':True,'remaining_jobs':[{'goal':'g'}]}
        finished={'needs_recovery':False};seen=[]
        def handler(current,frontier,resume):
            self.assertIs(current,trial);self.assertIs(frontier,blocked)
            self.assertTrue(current.transport_yield_on_deferral)
            seen.append(current.pick)
            return resume()
        with patch.object(reference,'_baseline',side_effect=[blocked,finished]) as run:
            result=reference.solve(trial,on_frontier=handler,yield_on_deferral=True)
        self.assertIs(result,finished)
        self.assertEqual(run.call_count,2)
        self.assertIsNot(seen[0],original_pick)
        self.assertIs(trial.pick,original_pick)
        self.assertFalse(hasattr(trial,'transport_yield_on_deferral'))

    def test_early_deferral_requires_explicit_forward_owner(self):
        for handler,forward in ((None,True),(Mock(),False)):
            with self.subTest(forward=forward):
                with self.assertRaisesRegex(ValueError,'frontier handler'):
                    reference.solve(SimpleNamespace(),on_frontier=handler,
                                    forward_recovery=forward,yield_on_deferral=True)

    def test_live_frontier_explicit_retry_retains_attempt_history(self):
        task,env=fixture_data();job=({'goal_id':'g'},{'id':'obj'},['A'])
        trial=SimpleNamespace(task=task,env=env,event=Mock(),pick=Mock(),execute=Mock(),place=Mock(),held={},
            monitor=SimpleNamespace(violations=[]),arms={'A':SimpleNamespace(job=None,uncertainty_reported=False)},
            transport_pending_jobs=[],transport_failed_jobs=[job],transport_job_attempts={'g':2})
        def handler(current,frontier,resume):
            result=resume(retry_failed=True)
            self.assertEqual(current.transport_pending_jobs,[job])
            self.assertEqual(current.transport_failed_jobs,[])
            self.assertEqual(current.transport_job_attempts,{'g':2})
            return result
        with patch.object(reference,'_baseline',side_effect=[{'needs_recovery':True},{'needs_recovery':False}]):
            self.assertFalse(reference.solve(trial,on_frontier=handler)['needs_recovery'])


if __name__ == '__main__':
    unittest.main()
