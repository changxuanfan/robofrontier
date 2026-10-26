"""Reference grasps follow each articulated part, with opposed wrist frames."""
import json
from types import SimpleNamespace
import unittest

import mujoco
import numpy as np

from robofrontier.suite.v2 import cases,directory
from robofrontier.workstation import Workstation
from robofrontier.suite.relay_oracle_v2 import grasp_targets,cross_x


class RelayReferenceTests(unittest.TestCase):
    def test_terminal_recovery_replans_after_other_arm_makes_progress(self):
        from unittest.mock import patch, Mock
        from robofrontier.suite.relay_oracle_v2 import sequential_terminal_release
        finished=set();calls=[]
        t=SimpleNamespace(arms={r:SimpleNamespace(job=None,grasp_force=None) for r in ('G','H')},
            held={'G':'payload','H':'payload'},wait=Mock(),event=Mock(),
            monitor=SimpleNamespace(violations=[],result=lambda:{'payloads':{'payload':{'completed':finished=={'G','H'}}}}))
        def route(world,ident,selected,donors,**kwargs):
            return None if donors==['H'] and 'G' not in finished else {donors[0]:[]}
        def release_one(world,ident,team,terminal=False):
            calls.append(team[0]);finished.add(team[0])
        with patch('robofrontier.suite.relay_oracle_v2.terminal_recovery_state',return_value=True),patch('robofrontier.suite.relay_oracle_v2.beam_table_contact',return_value=True),patch('robofrontier.suite.relay_oracle_v2.donor_exit_routes',route),patch('robofrontier.suite.relay_oracle_v2.release',release_one),patch('robofrontier.suite.relay_oracle_v2.arm_payload_contact',return_value=False):
            self.assertTrue(sequential_terminal_release(t,'payload',['H','G']))
        self.assertEqual(calls,['G','H'])

    def test_terminal_recovery_defers_unknown_execution_without_reissuing(self):
        from unittest.mock import Mock
        from robofrontier.suite.relay_oracle_v2 import terminal_recovery_state
        data=object();t=SimpleNamespace(data=data,_live_data=data,arms={'G':SimpleNamespace(job='unknown-call')},execute=Mock())
        self.assertFalse(terminal_recovery_state(t,'payload',['G']))
        self.assertEqual(t.relay_recovery_pending['active_jobs'],{'G':'unknown-call'})
        t.execute.assert_not_called()

    def test_idle_clearance_never_moves_a_busy_or_holding_arm(self):
        from unittest.mock import Mock
        from robofrontier.suite.relay_oracle_v2 import clear_free_idle_blocker
        for job,force,held in [('job',None,{}),(None,70.,{}),(None,None,{'O':'payload'})]:
            with self.subTest(job=job,force=force,held=held):
                t=SimpleNamespace(task={'level':1},data=object(),
                    relay_screen_reason={'kind':'arm_contact','a':'O_left_finger','b':'P_link6'},
                    arms={'O':SimpleNamespace(job=job,grasp_force=force),'P':object(),'H':object()},
                    held=held,execute=Mock())
                t._live_data=t.data
                self.assertFalse(clear_free_idle_blocker(t,'payload',['H','P'],[0,0,1.3],0.))
                t.execute.assert_not_called()

    def test_idle_clearance_checks_real_support_even_without_held_metadata(self):
        from unittest.mock import Mock
        from robofrontier.suite.relay_oracle_v2 import clear_free_idle_blocker
        t=SimpleNamespace(task={'level':1},data=object(),
            relay_screen_reason={'kind':'arm_contact','a':'O_left_finger','b':'P_link6'},
            arms={'O':SimpleNamespace(job=None,grasp_force=None),'P':object(),'H':object()},
            held={},execute=Mock(),monitor=SimpleNamespace(
                bilateral=lambda *args,**kwargs:{'O','P'},
                relay_v2=SimpleNamespace(states={'payload':{'rod_geoms':{1}}})))
        t._live_data=t.data
        self.assertFalse(clear_free_idle_blocker(t,'payload',['H','P'],[0,0,1.3],0.))
        t.execute.assert_not_called()

    def test_idle_clearance_cannot_actuate_a_planning_scratch_world(self):
        from unittest.mock import Mock
        from robofrontier.suite.relay_oracle_v2 import clear_free_idle_blocker
        t=SimpleNamespace(task={'level':1},data=object(),_live_data=object(),execute=Mock())
        self.assertFalse(clear_free_idle_blocker(t,'payload',['H','P'],[0,0,1.3],0.))
        t.execute.assert_not_called()

    def test_idle_clearance_requires_a_known_live_world_marker(self):
        from unittest.mock import Mock
        from robofrontier.suite.relay_oracle_v2 import clear_free_idle_blocker
        t=SimpleNamespace(task={'level':1},data=object(),execute=Mock())
        self.assertFalse(clear_free_idle_blocker(t,'payload',['H','P'],[0,0,1.3],0.))
        t.execute.assert_not_called()

    def test_weighted_terminal_top_class_keeps_better_joint_branch_first(self):
        from robofrontier.suite.relay_oracle_v2 import terminal_pair_order
        top=np.eye(3)
        side=np.array([[1.,0.,0.],[0.,0.,-1.],[0.,1.,0.]])
        def pair(name,rotation,margin):
            return tuple(dict(robot=name+str(i),rotation=rotation,margin=margin) for i in range(2))
        weak=pair('weak',top,.95);strong=pair('strong',top,1.2);lateral=pair('side',side,1.5)
        self.assertEqual(terminal_pair_order([weak,lateral,strong],np.eye(3),True),
                         [strong,weak,lateral])
        self.assertEqual(terminal_pair_order([weak,lateral,strong],np.eye(3),False),
                         [weak,strong,lateral])

    def test_explicit_carry_orientation_is_screened_and_kept_in_commands(self):
        from unittest.mock import patch, Mock
        from robofrontier.suite.relay_oracle_v2 import carry
        angle=.2
        rotation=np.array([[np.cos(angle),0.,np.sin(angle)],
                           [0.,1.,0.],[-np.sin(angle),0.,np.cos(angle)]])
        body=SimpleNamespace(xpos=np.array([0.,0.,1.3]),xmat=rotation)
        data=SimpleNamespace(body=lambda _:body,
            site_xpos=np.array([[0.,-.3,1.3],[0.,.3,1.3]]),
            site_xmat=np.array([rotation.reshape(-1),rotation.reshape(-1)]))
        rotations=[]
        def command(robot,point,wanted,*args):
            rotations.append(wanted.copy())
            return ('move',{'target_m':point.tolist()})
        t=SimpleNamespace(data=data,arms={'A':SimpleNamespace(site=0),
            'B':SimpleNamespace(site=1)},pose_command=command,execute=Mock(),
            monitor=SimpleNamespace(violations=[],relay_v2=SimpleNamespace(
                states={'payload':{'dropped':False}})))
        with patch('robofrontier.suite.relay_oracle_v2.carry_clear',return_value=True) as screen:
            carry(t,'payload',['A','B'],np.array([.03,0.,1.3]),0.,target_rotation=rotation)
        np.testing.assert_allclose(screen.call_args.kwargs['target_rotation'],rotation)
        self.assertTrue(rotations)
        for observed in rotations:
            np.testing.assert_allclose(observed,rotation,atol=1e-12)

    def test_terminal_support_does_not_mistake_weight_contact_for_beam_support(self):
        from robofrontier.suite.relay_oracle_v2 import final_support
        names={0:'payload_beam',1:'payload_weight',2:'table_0'}
        state={'all_geoms':{0,1},'rod_geoms':{0}}
        relay=SimpleNamespace(states={'payload':state},inside=lambda value:True)
        contact=SimpleNamespace(dist=-.0001,geom1=1,geom2=2)
        t=SimpleNamespace(model=SimpleNamespace(geom=lambda gid:SimpleNamespace(name=names[gid])),
                          data=SimpleNamespace(contact=[contact]),
                          monitor=SimpleNamespace(relay_v2=relay,bilateral=lambda *args,**kwargs:{'A','B'}))
        self.assertFalse(final_support(t,'payload',('A','B')))
        contact.geom1=0
        self.assertTrue(final_support(t,'payload',('A','B')))

    def test_terminal_search_keeps_at_most_four_actual_feasible_candidates(self):
        from unittest.mock import patch
        from robofrontier.suite.oracle import Trial
        from robofrontier.suite.evaluate import Monitor
        from robofrontier.robot_control import PandaArm
        from robofrontier.suite.relay_oracle_v2 import handover_plan
        case=next(c for c in cases() if c['id']=='relay-a08-d3')
        source=Workstation(directory(case));t=Trial.__new__(Trial)
        t.model,t.data=source.model,source.data;mujoco.mj_forward(t.model,t.data)
        t.task=json.loads((source.path/'task.json').read_text());t.arms={a:PandaArm(t.model,t.data,a) for a in t.task['arms']}
        t.monitor=Monitor(t);t.stop_requested=None;t.live_preview=False
        def clear(probe,*args,**kwargs):
            probe.relay_carry_endpoint=dict(qpos=probe.data.qpos.copy(),rotation=np.eye(3))
            return True
        def choose(probe,*args,**kwargs):
            side=np.array([[0.,0,1],[1,0,0],[0,1,0]])
            probe.relay_grasp_plan={robot:dict(rotation=side) for robot in ('G','H')}
            return ['G','H']
        with patch('robofrontier.suite.relay_oracle_v2.carry_clear',clear),patch('robofrontier.suite.relay_oracle_v2.choose_team',side_effect=choose) as selected:
            handover_plan(t,'payload_02',['E','F'],.44,3)
        self.assertEqual(selected.call_count,4)

    def test_terminal_support_requires_table_not_floor(self):
        from robofrontier.suite.relay_oracle_v2 import final_support
        names={0:'payload_beam',1:'floor'}
        state={'all_geoms':{0},'rod_geoms':{0}}
        relay=SimpleNamespace(states={'payload':state},inside=lambda value:True)
        t=SimpleNamespace(model=SimpleNamespace(geom=lambda gid:SimpleNamespace(name=names[gid])),data=SimpleNamespace(contact=[SimpleNamespace(dist=-.0001,geom1=0,geom2=1)]),monitor=SimpleNamespace(relay_v2=relay,bilateral=lambda *args,**kwargs:{'A','B'}))
        self.assertFalse(final_support(t,'payload',('A','B')))
        names[1]='table_0'
        self.assertTrue(final_support(t,'payload',('A','B')))

    def test_terminal_release_rejects_an_unsafe_opening_before_actuation(self):
        from unittest.mock import patch,Mock
        from robofrontier.suite.relay_oracle_v2 import release
        t=SimpleNamespace(execute=Mock())
        with patch('robofrontier.suite.relay_oracle_v2.donor_exit_routes',return_value=None) as route:
            with self.assertRaisesRegex(RuntimeError,'No safe terminal opening/exit plan'):
                release(t,'payload',('A','B'),terminal=True)
        self.assertEqual(route.call_args.kwargs['opening_width'],.080)
        t.execute.assert_not_called()

    def test_contact_placement_stops_real_jobs_without_relabelling_completion(self):
        from unittest.mock import patch,Mock
        from robofrontier.suite.oracle import Trial
        from robofrontier.robot_control import PandaArm
        from robofrontier.suite.relay_oracle_v2 import execute_landing_increment,LANDING_STOP_REASON
        case=next(c for c in cases() if c['id']=='relay-a08-d1')
        source=Workstation(directory(case));t=Trial.__new__(Trial)
        t.model,t.data=source.model,source.data;mujoco.mj_forward(t.model,t.data)
        t.arms={a:PandaArm(t.model,t.data,a) for a in ('A','B')}
        t.step=Mock();original_step=t.step;t.event=Mock();saved={}
        for robot,arm in t.arms.items():
            arm.job='unit-'+robot;arm.job_op='move';arm.job_start=0.;arm.grasp_force=70.;arm.grasp_width=.045
        def execute(label,commands,allow_failure):
            self.assertTrue(allow_failure)
            for tick in range(3):
                t.step()
                for robot,arm in t.arms.items():
                    result=arm.poll(.02*tick,15)
                    if result is not None:saved[robot]=result
            return saved
        t.execute=execute
        commands={a:('move',{'target_m':[0,0,1]}) for a in t.arms}
        with patch('robofrontier.suite.relay_oracle_v2.final_support',return_value=True):
            self.assertTrue(execute_landing_increment(t,'payload_01',('A','B'),commands))
        self.assertIs(t.step,original_step)
        for robot,arm in t.arms.items():
            self.assertIsNone(arm.job);self.assertEqual(arm.grasp_force,70.)
            self.assertEqual(saved[robot]['status'],'failed');self.assertEqual(saved[robot]['reason'],LANDING_STOP_REASON)
            self.assertFalse(saved[robot]['tcp_target_reached'])

    def test_contact_placement_does_not_hide_unrelated_failure(self):
        from unittest.mock import Mock
        from robofrontier.suite.relay_oracle_v2 import execute_landing_increment
        original=Mock();t=SimpleNamespace(step=original,execute=Mock(return_value={'A':{'status':'failed','reason':'Action timeout'}}))
        with self.assertRaisesRegex(RuntimeError,'Terminal placement action failed'):
            execute_landing_increment(t,'payload',('A',),{'A':('move',{})})
        self.assertIs(t.step,original)

    def test_carry_rechecks_actual_post_release_world_before_commands(self):
        from unittest.mock import patch,Mock
        from robofrontier.suite.oracle import Trial
        from robofrontier.suite.evaluate import Monitor
        from robofrontier.robot_control import PandaArm
        from robofrontier.suite.relay_oracle_v2 import carry
        case=next(c for c in cases() if c['id']=='relay-a08-d1')
        source=Workstation(directory(case));t=Trial.__new__(Trial)
        t.model,t.data=source.model,source.data;mujoco.mj_forward(t.model,t.data)
        t.task=json.loads((source.path/'task.json').read_text());t.arms={a:PandaArm(t.model,t.data,a) for a in t.task['arms']}
        t.monitor=Monitor(t);t.execute=Mock();t.relay_screen_reason={'kind':'arm_contact'}
        goal=t.data.body('payload_01').xpos.copy()+np.array([0,0,.04])
        with patch('robofrontier.suite.relay_oracle_v2.carry_clear',return_value=False):
            with self.assertRaisesRegex(RuntimeError,'Actual carry path'):
                carry(t,'payload_01',('A','B'),goal,0.)
        t.execute.assert_not_called()

    def test_successor_preview_does_not_replace_live_arm_objects_or_seeds(self):
        from unittest.mock import patch
        from robofrontier.suite.oracle import Trial
        from robofrontier.robot_control import PandaArm
        from robofrontier.suite.relay_oracle_v2 import received_transport_preview
        case=next(c for c in cases() if c['id']=='relay-a08-d1')
        source=Workstation(directory(case));t=Trial.__new__(Trial)
        t.model,t.data=source.model,source.data;mujoco.mj_forward(t.model,t.data)
        t.task=json.loads((source.path/'task.json').read_text())
        t.arms={a:PandaArm(t.model,t.data,a) for a in t.task['arms']}
        t.home_q={a:t.data.qpos[arm.qids].copy() for a,arm in t.arms.items()}
        for robot in ('A','B'):
            for suffix in ('finger_joint1','finger_joint2'):t.data.joint(robot+'_'+suffix).qpos[0]=(.045-t.arms[robot].grip_min)/2
        mujoco.mj_forward(t.model,t.data)
        before=t.data.qpos.copy();objects=dict(t.arms);seeds={a:arm.qref.copy() for a,arm in t.arms.items()}
        t.relay_grasp_plan={a:dict(contact_q=seeds[a]+.01) for a in ('C','D')}
        def clear(probe,*args,**kwargs):
            for robot in ('A','B'):
                width=sum(probe.data.joint(robot+'_'+suffix).qpos[0] for suffix in ('finger_joint1','finger_joint2'))+probe.arms[robot].grip_min
                self.assertAlmostEqual(width,.075)
            probe.relay_carry_endpoint=dict(qpos=probe.data.qpos.copy())
            return True
        future=dict(future_region=[0.,0.,1.],future_height=1.2,future_yaw=0.,future_waypoints=[])
        with patch('robofrontier.suite.relay_oracle_v2.carry_clear',clear):
            result=received_transport_preview(t,'payload_01',['A','B'],['C','D'],future)
        np.testing.assert_array_equal(t.data.qpos,before)
        for robot,arm in t.arms.items():
            self.assertIs(arm,objects[robot]);np.testing.assert_array_equal(arm.qref,seeds[robot])
        self.assertIsNot(result.arms['C'],t.arms['C'])
        np.testing.assert_allclose(result.arms['C'].qref,seeds['C']+.01)

    def test_weighted_grasp_cost_accounts_for_a_separable_successor(self):
        from robofrontier.suite.evaluate import Monitor
        from robofrontier.suite.relay_oracle_v2 import grasp_horizon_score,grasp_load_moment
        case=next(c for c in cases() if c['id']=='relay-a08-d3')
        case=dict(case,scene_revision=13)  # Recorded pre-r16 load-moment regression.
        source=Workstation(directory(case));mujoco.mj_forward(source.model,source.data)
        t=SimpleNamespace(model=source.model,data=source.data,task=json.loads((source.path/'task.json').read_text()))
        t.arms={robot:None for robot in t.task['arms']};t.monitor=Monitor(t)
        options=(.20,.24,.28,.32,.36,.38,.40,.44,.48,.50,.54,.56,.58)
        current=grasp_load_moment(t,'payload_01',.50)
        self.assertGreater(grasp_load_moment(t,'payload_01',.54),current)
        self.assertLess(grasp_horizon_score(t,'payload_01',.54,options),grasp_horizon_score(t,'payload_01',.50,options))

    def test_handover_checks_executed_legs_not_an_uncommitted_later_reach(self):
        from unittest.mock import patch
        from robofrontier.suite.oracle import Trial
        from robofrontier.suite.evaluate import Monitor
        from robofrontier.robot_control import PandaArm
        from robofrontier.suite.relay_oracle_v2 import handover_plan
        case=next(c for c in cases() if c['id']=='relay-a12-d1')
        source=Workstation(directory(case));t=Trial.__new__(Trial)
        t.model,t.data=source.model,source.data;mujoco.mj_forward(t.model,t.data)
        t.task=json.loads((source.path/'task.json').read_text())
        t.arms={a:PandaArm(t.model,t.data,a) for a in t.task['arms']}
        t.monitor=Monitor(t);t.stop_requested=None;t.live_preview=False
        def clear(probe,*args,**kwargs):
            probe.relay_carry_endpoint=dict(qpos=probe.data.qpos.copy(),rotation=np.eye(3))
            return True
        with patch('robofrontier.suite.relay_oracle_v2.carry_clear',clear),patch('robofrontier.suite.relay_oracle_v2.choose_team',return_value=['C','D']):
            _,_,_,future=handover_plan(t,'payload_01',['A','B'],.44,1,lookahead_depth=1)
        self.assertEqual(len(future['future_waypoints']),1)
        point,yaw=future['future_waypoints'][0]
        np.testing.assert_allclose(point[:2],t.task['route'][1]['center_m'][:2])
        self.assertEqual(yaw,t.task['route'][1]['nominal_yaw_rad'])

    def test_curved_l1_returns_first_actual_successor_witness(self):
        from unittest.mock import patch
        from robofrontier.suite.oracle import Trial
        from robofrontier.suite.evaluate import Monitor
        from robofrontier.robot_control import PandaArm
        from robofrontier.suite.relay_oracle_v2 import handover_plan
        case=next(c for c in cases() if c['id']=='relay-a12-d1')
        source=Workstation(directory(case));t=Trial.__new__(Trial)
        t.model,t.data=source.model,source.data;mujoco.mj_forward(t.model,t.data)
        t.task=json.loads((source.path/'task.json').read_text());t.arms={a:PandaArm(t.model,t.data,a) for a in t.task['arms']}
        t.monitor=Monitor(t);t.stop_requested=None;t.live_preview=False
        def clear(probe,*args,**kwargs):
            probe.relay_carry_endpoint=dict(qpos=probe.data.qpos.copy(),rotation=np.eye(3))
            return True
        successor=(np.array([0.,0.,1.]),0.,.44,{})
        with patch('robofrontier.suite.relay_oracle_v2.carry_clear',clear),patch('robofrontier.suite.relay_oracle_v2.choose_team',return_value=['C','D']) as choose,patch('robofrontier.suite.relay_oracle_v2.received_transport_preview',return_value=t),patch('robofrontier.suite.relay_oracle_v2.handover_plan',return_value=successor) as following:
            handover_plan(t,'payload_01',['A','B'],.44,1)
        self.assertEqual(choose.call_count,1)
        self.assertEqual(following.call_count,1)
        self.assertEqual(following.call_args.kwargs['lookahead_depth'],1)

    def test_opposed_frames_and_real_distal_pose(self):
        case=next(c for c in cases() if c['id']=='relay-a08-d3')
        source=Workstation(directory(case));m,d=source.model,source.data
        task=json.loads((source.path/'task.json').read_text());t=SimpleNamespace(task=task,data=d,model=m)
        ident='payload_02';d.joint(ident+'_flexion').qpos[0]=.2;mujoco.mj_forward(m,d)
        root=d.body(ident);r=root.xmat.reshape(3,3);points=grasp_targets(t,ident,root.xpos,r,.44)
        child=d.body(ident+'_distal');pivot=m.body_pos[m.body(ident+'_distal').id]
        expected=child.xpos+child.xmat.reshape(3,3)@(np.array([cross_x(task,.44),.44,.006])-pivot)
        np.testing.assert_allclose(points[1][0],expected,atol=1e-10)
        self.assertLess(np.dot(points[0][1][:,1],points[1][1][:,1]),-.5)
        for _,rotation in points:np.testing.assert_allclose(rotation.T@rotation,np.eye(3),atol=1e-10)


    def test_carry_screen_does_not_write_live_state(self):
        from robofrontier.suite.oracle import Trial
        from robofrontier.suite.evaluate import Monitor
        from robofrontier.robot_control import PandaArm
        from robofrontier.suite.relay_oracle_v2 import carry_clear
        case=next(c for c in cases() if c['id']=='relay-a08-d1')
        source=Workstation(directory(case));t=Trial.__new__(Trial)
        t.model,t.data=source.model,source.data
        mujoco.mj_forward(t.model,t.data)
        t.task=json.loads((source.path/'task.json').read_text())
        t.arms={a:PandaArm(t.model,t.data,a) for a in t.task['arms']}
        t.monitor=Monitor(t);t.stop_requested=None
        before=[getattr(t.data,k).copy() for k in ('qpos','qvel','ctrl')]
        point=t.data.body('payload_01').xpos.copy()
        point[2]-=.03
        # Deliberately ask to move timber through its supporting table.
        # Screening traverses FK/IK without modifying the live world.
        self.assertFalse(carry_clear(t,'payload_01',('A','B'),point,0.))
        self.assertFalse(carry_clear(t,'payload_01',('A','B'),point,0.,landing=True))
        self.assertEqual(t.relay_screen_reason['kind'],'payload_contact')
        for key,expected in zip(('qpos','qvel','ctrl'),before):
            np.testing.assert_array_equal(getattr(t.data,key),expected)

    def test_arc_jaws_follow_local_beam_normal(self):
        case=next(c for c in cases() if c['id']=='relay-a08-d2')
        source=Workstation(directory(case));m,d=source.model,source.data
        mujoco.mj_forward(m,d)
        task=json.loads((source.path/'task.json').read_text());t=SimpleNamespace(task=task,data=d,model=m)
        body=d.body('payload_01');r=body.xmat.reshape(3,3)
        points=grasp_targets(t,'payload_01',body.xpos,r,.44)
        for sign,(_,tool) in zip((-1,1),points):
            theta=np.arcsin(sign*.44/task['mechanism']['radius_m'])
            expected=-sign*r@np.array([np.cos(theta),np.sin(theta),0.])
            np.testing.assert_allclose(tool[:,1],expected,atol=1e-10)
        pair=grasp_targets(t,'payload_01',body.xpos,r,(.56,.40))
        np.testing.assert_allclose([float((r.T@(p-body.xpos))[1]) for p,_ in pair],[-.56,.40],atol=1e-10)

    def test_receiving_route_screen_never_changes_live_fingers_or_arms(self):
        from copy import copy
        from robofrontier.suite.oracle import Trial
        from robofrontier.suite.evaluate import Monitor
        from robofrontier.robot_control import PandaArm
        from robofrontier.suite.relay_oracle_v2 import receiving_routes
        case=next(c for c in cases() if c['id']=='relay-a08-d1')
        source=Workstation(directory(case));t=Trial.__new__(Trial)
        t.model,t.data=source.model,source.data
        mujoco.mj_forward(t.model,t.data)
        t.task=json.loads((source.path/'task.json').read_text())
        t.arms={a:PandaArm(t.model,t.data,a) for a in t.task['arms']}
        t.monitor=Monitor(t);t.stop_requested=None;t.held={};t.carry_geoms={}
        t.screen_model=copy(t.model);t.screen_model.geom_margin[:]=np.maximum(t.screen_model.geom_margin,.004)
        t.screen_data=mujoco.MjData(t.screen_model)
        before={k:getattr(t.data,k).copy() for k in ('qpos','qvel','ctrl')}
        # This is a safe empty approach at home, not evidence of a grasp.
        selected=[]
        for robot in ('A','B'):
            arm=t.arms[robot];q=t.data.qpos[arm.qids].copy()
            selected.append(dict(robot=robot,seed=q,contact_q=q,rotation=t.data.site_xmat[arm.site].reshape(3,3).copy()))
        paths=receiving_routes(t,'payload_01',selected)
        self.assertEqual(set(paths),{'A','B'})
        for key,expected in before.items():np.testing.assert_array_equal(getattr(t.data,key),expected)

    def test_weighted_arc_inner_grasp_requires_more_roll_moment(self):
        from robofrontier.suite.evaluate import Monitor
        from robofrontier.suite.relay_oracle_v2 import grasp_load_moment
        case=next(c for c in cases() if c['id']=='relay-a08-d2')
        case=dict(case,scene_revision=13)  # Its old end-mounted weight defines this ranking.
        source=Workstation(directory(case));mujoco.mj_forward(source.model,source.data)
        t=SimpleNamespace(model=source.model,data=source.data,task=json.loads((source.path/'task.json').read_text()))
        t.arms={robot:None for robot in t.task['arms']};t.monitor=Monitor(t)
        # On this actual manufactured geometry the apex-side station is a
        # poor roll support. The most extreme end station is not the best,
        # either: an intermediate station aligns nearer the assembly COM.
        self.assertLess(grasp_load_moment(t,'payload_01',.48),grasp_load_moment(t,'payload_01',.54))
        self.assertLess(grasp_load_moment(t,'payload_01',.54),grasp_load_moment(t,'payload_01',.20))
        self.assertLess(grasp_load_moment(t,'payload_01',(.56,.40)),grasp_load_moment(t,'payload_01',.32)/3)

    def test_fixed_chain_ik_cache_reuses_only_identical_queries(self):
        from robofrontier.suite.oracle import Trial
        from robofrontier.robot_control import PandaArm
        from robofrontier.suite.relay_oracle_v2 import screened_ik
        case=next(c for c in cases() if c['id']=='relay-a08-d1')
        source=Workstation(directory(case));mujoco.mj_forward(source.model,source.data)
        t=Trial.__new__(Trial);t.model,t.data=source.model,source.data
        t.arms={'A':PandaArm(t.model,t.data,'A')};t.stop_requested=None;t.live_preview=False
        arm=t.arms['A'];point=t.data.site_xpos[arm.site].copy();rotation=t.data.site_xmat[arm.site].reshape(3,3).copy()
        original=t.screen_ik;calls=[]
        def counted(*args,**kwargs):calls.append(1);return original(*args,**kwargs)
        t.screen_ik=counted
        first=screened_ik(t,'A',point,rotation);expected=first.copy();first[0]+=1
        np.testing.assert_array_equal(screened_ik(t,'A',point,rotation),expected)
        self.assertEqual(len(calls),1)
        screened_ik(t,'A',point+np.array([.002,0,0]),rotation)
        self.assertEqual(len(calls),2)
        self.assertIsNone(screened_ik(t,'A',point+np.array([10.,0,0]),rotation))
        self.assertEqual(len(calls),2)

    def test_geometric_reach_filter_keeps_real_fk_goals(self):
        from robofrontier.robot_control import PandaArm
        from robofrontier.suite.relay_oracle_v2 import potentially_reachable
        case=next(c for c in cases() if c['id']=='relay-a08-d1')
        source=Workstation(directory(case));mujoco.mj_forward(source.model,source.data)
        arm=PandaArm(source.model,source.data,'A')
        t=SimpleNamespace(model=source.model,data=source.data,arms={'A':arm})
        rng=np.random.default_rng(418)
        for _ in range(24):
            q=rng.uniform(*arm.ik.limits.T);point,rotation=arm.ik.pose(q)
            self.assertTrue(potentially_reachable(t,'A',point,rotation))

    def test_stage_deadline_is_checked_inside_nested_solver(self):
        from unittest.mock import patch
        from robofrontier.suite.oracle import Trial,ReferenceStopped
        from robofrontier.suite.relay_oracle_v2 import handover_plan,choose_team
        t=Trial.__new__(Trial);t.data=object();t.stop_requested=None
        def nested(search,*args):search.check_stop()
        with patch('robofrontier.suite.relay_oracle_v2.STAGE_SEARCH_WALL_S',0.),patch('robofrontier.suite.relay_oracle_v2._handover_plan',nested):
            with self.assertRaisesRegex(RuntimeError,'Bounded handover search exhausted'):handover_plan(t,'payload',[],.44,1)
            t.stop_requested=lambda:True
            with self.assertRaises(ReferenceStopped):handover_plan(t,'payload',[],.44,1)
        t.stop_requested=None
        with patch('robofrontier.suite.relay_oracle_v2._choose_team',nested):
            with self.assertRaisesRegex(RuntimeError,'Receiving candidate search exhausted'):choose_team(t,search_budget_s=0.)
        self.assertIsNone(t.stop_requested)

    def test_budget_returns_a_verified_best_candidate_but_user_stop_does_not(self):
        from unittest.mock import patch
        from robofrontier.suite.oracle import Trial,ReferenceStopped
        from robofrontier.suite.relay_oracle_v2 import handover_plan
        t=Trial.__new__(Trial);t.data=object();t.stop_requested=None
        witness=(np.array([0.,0.,1.]),0.,.5,{'future_waypoints':[]})
        def bounded(search,*args):
            search.relay_handover_best=witness
            search.check_stop()
        with patch('robofrontier.suite.relay_oracle_v2._handover_plan',bounded):
            self.assertIs(handover_plan(t,'payload',[],.44,1,search_budget_s=0),witness)
            t.stop_requested=lambda:True
            with self.assertRaises(ReferenceStopped):handover_plan(t,'payload',[],.44,1,search_budget_s=0)

    def test_carry_screen_checks_hand_against_table_not_only_other_arms(self):
        from robofrontier.suite.oracle import Trial
        from robofrontier.suite.evaluate import Monitor
        from robofrontier.robot_control import PandaArm
        from robofrontier.suite.relay_oracle_v2 import carry_clear
        case=next(c for c in cases() if c['id']=='relay-a08-d1')
        source=Workstation(directory(case));t=Trial.__new__(Trial)
        t.model,t.data=source.model,source.data;mujoco.mj_forward(t.model,t.data)
        t.task=json.loads((source.path/'task.json').read_text())
        t.arms={a:PandaArm(t.model,t.data,a) for a in t.task['arms']};t.monitor=Monitor(t);t.stop_requested=None
        arm=t.arms['A'];target=np.asarray(t.task['arms']['A']['base_position_m'])+np.array([0,.3,-.1])
        _,rotation=arm.ik.pose(arm.qref);q,error=arm.ik.solve(target,rotation,arm.qref,350)
        self.assertLess(error,.001)
        # An independent FK-only collision fixture, not a live Oracle reset.
        t.data.qpos[arm.qids]=q;arm.qref=q;mujoco.mj_forward(t.model,t.data)
        before=t.data.qpos.copy();point=t.data.body('payload_01').xpos.copy()+np.array([0,0,.04])
        self.assertFalse(carry_clear(t,'payload_01',('A','B'),point,0.))
        self.assertEqual(t.relay_screen_reason['kind'],'robot_world_contact')
        np.testing.assert_array_equal(t.data.qpos,before)

    def test_carry_screen_unwinds_existing_tilt_continuously(self):
        from unittest.mock import patch
        from robofrontier.suite.oracle import Trial
        from robofrontier.suite.evaluate import Monitor
        from robofrontier.robot_control import PandaArm,matrix_quat,rotation_vector
        from robofrontier.suite.relay_oracle_v2 import carry_clear
        from scripts.verify_floating_dial_oracle import axis_rotation
        case=next(c for c in cases() if c['id']=='relay-a08-d1')
        source=Workstation(directory(case));t=Trial.__new__(Trial)
        t.model,t.data=source.model,source.data
        t.task=json.loads((source.path/'task.json').read_text());t.stop_requested=None
        t.data.joint('payload_01_free').qpos[3:]=matrix_quat(axis_rotation(np.array([1.,0,0]),.2))
        mujoco.mj_forward(t.model,t.data)
        t.arms={a:PandaArm(t.model,t.data,a) for a in t.task['arms']};t.monitor=Monitor(t)
        before=t.data.qpos.copy();initial=t.data.site_xmat[t.arms['A'].site].reshape(3,3).copy();seen=[]
        def inspect(world,robot,point,rotation,seed=None):seen.append(rotation.copy());return None
        with patch('robofrontier.suite.relay_oracle_v2.screened_ik',inspect):
            self.assertFalse(carry_clear(t,'payload_01',('A','B'),t.data.body('payload_01').xpos.copy(),0.))
        # 0.2 rad uses four samples. The first step may unwind only 0.05 rad,
        # rather than teleporting the entire existing pitch/roll to zero.
        self.assertAlmostEqual(float(np.linalg.norm(rotation_vector(seen[0],initial))),.05,places=9)
        np.testing.assert_array_equal(t.data.qpos,before)


if __name__=='__main__':unittest.main()
