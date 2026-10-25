"""Recorded kinematics reproduce candidate coverage, not physical success."""
from copy import copy
import json
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

import mujoco
import numpy as np

from robofrontier.robot_control import PandaArm
from robofrontier.suite.oracle import Trial
from robofrontier.suite.evaluate import Monitor
from robofrontier.suite import transport_full_oracle as full

ROOT=Path(__file__).resolve().parents[1]


def recorded_frontier(fixture_name='transport_air_frontier.json'):
    fixture=json.loads((ROOT/'tests/fixtures'/fixture_name).read_text())
    scene=ROOT/fixture['scene'];t=Trial.__new__(Trial)
    t.task=json.loads((scene/'task.json').read_text());t.env=json.loads((scene/'environment.json').read_text())
    t.model=mujoco.MjModel.from_xml_path(str(scene/'scene.xml'));t.data=mujoco.MjData(t.model)
    mujoco.mj_resetDataKeyframe(t.model,t.data,t.model.key('initial').id);mujoco.mj_forward(t.model,t.data)
    t.arms={r:PandaArm(t.model,t.data,r) for r in t.task['arms']}
    t.home_q={r:t.data.qpos[a.qids].copy() for r,a in t.arms.items()};t.parking_q={}
    for r,a in t.arms.items():
        p=t.data.site_xpos[a.site].copy();p[2]=t.task['table_height_m']+.68
        q,_=a.ik.solve(p,t.data.site_xmat[a.site].reshape(3,3),t.home_q[r],220)
        t.parking_q[r]=q
    t.data.qpos[:]=fixture['qpos'];mujoco.mj_forward(t.model,t.data)
    t.stations={s['id']:s for s in t.env['stations']};t.held=fixture['held']
    t.monitor=Monitor(t);t.screen_model=copy(t.model)
    t.screen_model.geom_margin[:]=np.maximum(t.screen_model.geom_margin,.004)
    t.screen_data=mujoco.MjData(t.screen_model);t.hand_geoms={};t.carry_geoms={}
    t.stop_requested=None;t.live_preview=False;t._reference_planning_only=True;t._live_data=object()
    t.actions=[];t.event=Mock()
    # Retention comes from archived engineering evidence; no dynamics run here.
    t.finger_contacts=lambda r,n:t.held.get(r)==n
    return t


class TransferCoverageTests(unittest.TestCase):
    def test_empty_receiver_can_withdraw_from_existing_positive_close_clearance(self):
        from types import SimpleNamespace
        from robofrontier.suite.transport_runtime import sampled_motion_admission
        fixture=json.loads((ROOT/'tests/fixtures/transport_clearance_escape.json').read_text())
        t=recorded_frontier('transport_clearance_escape.json')
        for r in t.held:t.arms[r].grasp_force=70.
        runtime=SimpleNamespace(t=t,running={},jobs={
            'transfer':dict(object='D_item3',robots={'E','F'}),
            'other':dict(object='E_item1',robots={'G'})})
        before=t.data.qpos.copy();end=np.array(fixture['escape']['joint_rad']);start=t.data.qpos[t.arms['F'].qids]
        request=lambda q:dict(commands={'F':('move',dict(joint_rad=q.tolist(),joint_speed_rad_s=.18))})
        # Recorded gap starts at3.46mm, remains positive and grows beyond6mm.
        self.assertTrue(sampled_motion_admission(request(end),runtime))
        self.assertFalse(sampled_motion_admission(request(2*start-end),runtime))
        runtime.leases={'E':'transfer','F':'transfer','G':'other'}
        runtime.jobs['historical']=dict(object='E_item1',robots={'F'})
        self.assertTrue(sampled_motion_admission(request(end),runtime))
        t.held['F']='D_item3'
        self.assertFalse(sampled_motion_admission(request(end),runtime))
        t.arms['F'].grasp_force=70.
        self.assertFalse(sampled_motion_admission(request(end),runtime))
        np.testing.assert_array_equal(t.data.qpos,before)

    def test_matching_keeps_easy_side_stock_for_direct_socket_work(self):
        from types import SimpleNamespace
        names=['up','side1','side2'];positions={n:np.array([i*.1,0.,.83]) for i,n in enumerate(names)}
        side=np.array([[1.,0,0],[0,0,-1.],[0,1.,0]])
        rotations={'up':np.diag([1.,-1.,-1.]),'side1':side,'side2':side}
        goals=[dict(goal_id=r,robot=r,requires=dict(identity='same'),point=[i,0.,.83]) for i,r in enumerate('ABC')]
        t=SimpleNamespace(arms={r:None for r in 'ABC'},stations={r:dict(base_position_m=[i,0.,.8]) for i,r in enumerate('ABC')},
            env=dict(objects=[dict(id=n,identity='same') for n in names]),task=dict(level=2,goals=goals),event=Mock(),
            data=SimpleNamespace(body=lambda n:SimpleNamespace(xpos=positions[n],xmat=rotations[n].reshape(-1))))
        with patch.object(full,'candidate_graph',return_value={r:[s for s in 'ABC' if s!=r] for r in 'ABC'}),patch.object(full,'goal_coverage',side_effect=lambda _,g:[g['robot']]),patch.object(full,'goal_pose',side_effect=lambda _,g:(np.array(g['point']),np.eye(3))),patch.object(full.ref,'_pickup_coverage',return_value={n:['A'] for n in names}):
            jobs=full.plan_jobs(t)
        direct=next(j for j in jobs if len(j['path'])==1)
        self.assertNotEqual(direct['object'],'up')

    def test_small_clear_retained_load_move_remains_admissible(self):
        from types import SimpleNamespace
        from robofrontier.suite.transport_runtime import sampled_motion_admission
        t=recorded_frontier()
        for r in t.held:t.arms[r].grasp_force=70.
        runtime=SimpleNamespace(t=t,running={},jobs={r:dict(object=name,robots={r}) for r,name in t.held.items()})
        target=t.data.qpos[t.arms['D'].qids].copy();target[0]+=.001
        before=t.data.qpos.copy()
        self.assertTrue(sampled_motion_admission(dict(commands={'D':('move',dict(joint_rad=target.tolist(),joint_speed_rad_s=.18))}),runtime))
        np.testing.assert_array_equal(t.data.qpos,before)

    def test_receiver_cartesian_approach_does_not_cut_through_sender_wrist(self):
        from types import SimpleNamespace
        from robofrontier.suite.transport_runtime import sampled_motion_admission
        f=json.loads((ROOT/'tests/fixtures/transport_tcp_wrist_sweep.json').read_text())
        t=recorded_frontier('transport_tcp_wrist_sweep.json')
        for r in t.held:t.arms[r].grasp_force=70.
        runtime=SimpleNamespace(t=t,running={r:dict(commands={r:c}) for r,c in f['running'].items()},
            jobs={r:dict(object=name,robots=set(f['job_robots'][r])) for r,name in t.held.items()})
        self.assertFalse(sampled_motion_admission(dict(commands=f['request']),runtime))

    def test_intermediate_receiver_needs_next_transfer_not_final_board_reach(self):
        from robofrontier.suite import transport_oracle_v2 as ref
        from robofrontier.robot_control import matrix_quat
        t=recorded_frontier()
        plan=ref.plan_air_regrasp(t,'A','A_item2',planning_budget_s=3.,allowed_receivers=['C'],
            candidate_centers=full.handover_centers(t,'A',['C']),receiver_tilt_deg=-90.,
            receiver_depths=(.026,.020,-.026,-.020,-.012,.006))
        self.assertIsNotNone(plan)
        s=ref._planning_copy(t);body=s.data.body('A_item2');old=body.xmat.reshape(3,3).copy()
        offset=old.T@(s.data.site_xpos[s.arms['A'].site]-body.xpos)
        relative=old.T@s.data.site_xmat[s.arms['A'].site].reshape(3,3)
        rotation=plan['donor_tool']@relative.T;center=plan['donor_point']-rotation@offset
        s.data.qpos[s.arms['A'].qids]=plan['donor_q']
        s.data.joint('A_item2_free').qpos[:]=np.r_[center,matrix_quat(rotation)]
        mujoco.mj_forward(s.model,s.data)
        point=center+rotation[:,2]*plan['receiver_depth']+rotation[:,1-plan['receiver_axis']]*plan['receiver_shift']
        tool=rotation@plan['receiver_relative_rotation']
        final=next(g for g in t.task['goals'] if g['goal_id']=='A_slot1')
        future=dict(final,next_handover_center_m=full.handover_centers(t,'C',['B'])['B'].tolist())
        contact=s.screen_ik('C',point,tool,plan['receiver_approach_q'])
        self.assertIsNotNone(contact)
        before=s.data.qpos.copy()
        self.assertTrue(ref._receiver_successor_clear(s,'A','C','A_item2',future,point,tool,contact_q=contact))
        np.testing.assert_array_equal(s.data.qpos,before)

    def test_unqualified_pickup_waits_for_a_receiver_that_can_finish(self):
        from types import SimpleNamespace
        t=SimpleNamespace(task=dict(level=2),event=Mock())
        runtime=SimpleNamespace(leases={'B':'other'},running={},settled=lambda _:True)
        job=dict(id='slot',path=['A'],object='item',goal=dict(board='board'),receiver_candidates=['A','B'])
        with patch.object(full,'goal_pose',return_value=(np.zeros(3),np.eye(3))),patch.object(full,'goal_coverage',return_value=['A','B']),patch('robofrontier.suite.transport_unit_motion.direct_grasp_candidate',return_value=(None,{})) as planner:
            # C being free is not sufficient if only B is a suitable receiver.
            self.assertIsNone(full.pickup_resources(t,runtime,job))
            self.assertIsNone(full.pickup_resources(t,runtime,job))
            self.assertEqual(planner.call_count,1)
            runtime.running={'C':object()}
            self.assertIsNone(full.pickup_resources(t,runtime,job))
            self.assertEqual(planner.call_count,1)
            runtime.leases.clear()
            self.assertEqual(full.pickup_resources(t,runtime,job),{'A','B'})
        with patch.object(full,'goal_pose',return_value=(np.zeros(3),np.eye(3))),patch('robofrontier.suite.transport_unit_motion.direct_grasp_candidate',return_value=({},{})):
            job.pop('grasp_check_revision');runtime.leases={'B':'other'}
            self.assertEqual(full.pickup_resources(t,runtime,job),{'A'})

    def test_independent_progress_can_reach_other_arms_finished_pose(self):
        from types import SimpleNamespace
        from robofrontier.suite.transport_runtime import sampled_motion_admission
        fixture=json.loads((ROOT/'tests/fixtures/transport_async_sweep.json').read_text())
        t=recorded_frontier('transport_async_sweep.json')
        for r in t.held:t.arms[r].grasp_force=70.
        runtime=SimpleNamespace(t=t,running={r:dict(commands={r:c}) for r,c in fixture['running'].items()},
            jobs={r:dict(object=name,robots={r}) for r,name in t.held.items()})
        self.assertFalse(sampled_motion_admission(dict(commands=fixture['request']),runtime))

    def test_free_release_keeps_original_goal_but_requests_precise_motion(self):
        from types import SimpleNamespace
        t=SimpleNamespace(task=dict(family='transport',level=1),transport_release_clearance={'C':.05},event=Mock(),place=Mock())
        target=np.array([0.,.302,.833]);rotation=np.eye(3)
        full.place_flat(t,'C',target,rotation,lambda:(target,rotation))
        args,kw=t.place.call_args
        np.testing.assert_allclose(args[1],target+[0,0,.035])
        self.assertFalse(kw['precise']);self.assertTrue(kw['motion_precise'])
        np.testing.assert_array_equal(target,[0.,.302,.833])
        t.task['level']=2
        with self.assertRaises(ValueError):full.place_flat(t,'C',target,rotation,lambda:(target,rotation))

    def test_original_blocked_frontier_has_a_receivable_successor_candidate(self):
        t=recorded_frontier();goal=next(g for g in t.task['goals'] if g['goal_id']=='B_slot2')
        class ReachedExecution(Exception):pass
        t.move_free=Mock(side_effect=ReachedExecution)
        before=t.data.qpos.copy()
        with self.assertRaises(ReachedExecution):
            full.air_transfer(t,'D','D_item2',['B'],goal)
        np.testing.assert_array_equal(t.data.qpos,before)


if __name__=='__main__':unittest.main()
