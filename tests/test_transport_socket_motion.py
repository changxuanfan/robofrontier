"""Fit/path regressions. The archived fixture is only kinematic evidence."""
from copy import copy
from contextlib import nullcontext
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import mujoco
import numpy as np

from robofrontier.robot_control import PandaArm
from robofrontier.suite.oracle import Trial
from robofrontier.suite.evaluate import Monitor
from robofrontier.suite import transport_socket_motion as socket
from robofrontier.suite import transport_oracle_v2 as reference
from robofrontier.suite.transport_unit_motion import insertion_grasps
from scripts.verify_transport_parallel_unit import execute_independent_sequences

ROOT=Path(__file__).resolve().parents[1]


class SocketMotionTests(unittest.TestCase):
    def test_fast_arm_continues_while_other_arm_is_busy(self):
        starts=[]
        class Arm:
            job=None
            def validate(self,op,parameters,*_):return parameters['joint_rad'][0]
            def start(self,ident,duration,now):
                self.job=(ident,now+duration);starts.append((ident,now))
            def poll(self,now,*_):
                if self.job is None or now+1e-9<self.job[1]:return None
                ident=self.job[0];self.job=None
                return dict(status='completed',call_id=ident)
        t=SimpleNamespace(time=0.,tick=0,stride=1,cfg={},actions=[],check_stop=lambda:None,
            event=Mock(),arms={'A':Arm(),'B':Arm()},task={'workspace_world_m':{'lower':[0,0,0],'upper':[1,1,1]}})
        def step():t.tick+=1;t.time=round(t.time+.02,8)
        t.step=step
        def op(label,duration,delay=0.):
            return dict(label=label,delay=delay,command=('move',dict(joint_rad=[duration]*7)))
        execute_independent_sequences(t,{'A':[op('first',.1),op('second',.1,.06)],'B':[op('long',.5)]})
        self.assertEqual(starts,[('independent-A-0',0.),('independent-B-0',0.),('independent-A-1',.16)])
        self.assertEqual(t.time,.5);self.assertEqual(len(t.actions),3)
        self.assertTrue(all(a.job is None for a in t.arms.values()))

    def test_replan_does_not_pull_inserted_or_unknown_payload(self):
        data=SimpleNamespace(body=lambda _:SimpleNamespace(xpos=np.zeros(3),xmat=np.eye(3).ravel()),
            geom_xpos=np.array([[0.,0.,.10]]),geom_xmat=np.eye(3).reshape(1,9))
        t=SimpleNamespace(data=data,held={'A':'part'},arms={'A':SimpleNamespace(job=None)},
            monitor=SimpleNamespace(violations=[],geoms={'part':[0]}),finger_contacts=lambda *_:True,
            model=SimpleNamespace(geom_aabb=np.array([[0,0,0,.03,.03,.03]])))
        with patch.object(socket,'payload_clear_of_board',return_value=True) as clearance:
            self.assertTrue(socket.may_replan_in_air(t,'A','part','board','socket-descend'))
            clearance.return_value=False
            self.assertFalse(socket.may_replan_in_air(t,'A','part','board','socket-descend'))
            clearance.return_value=True;t.arms['A'].job='unresolved'
            self.assertFalse(socket.may_replan_in_air(t,'A','part','board','seat_release_descent'))
            t.arms['A'].job=None;t.monitor.violations.append({'kind':'self_robot_contact'})
            self.assertFalse(socket.may_replan_in_air(t,'A','part','board','seat_release_descent'))

    def test_board_clearance_allows_beside_but_not_engaged_payload(self):
        model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <body name="board"><geom name="plate" type="box" size=".1 .1 .016" pos="0 0 .016"/></body>
          <body name="part" pos=".5 0 .03"><freejoint name="part_free"/>
            <geom name="part_shape" type="box" size=".03 .03 .03"/></body>
        </worldbody></mujoco>''')
        data=mujoco.MjData(model);mujoco.mj_forward(model,data)
        t=SimpleNamespace(model=model,data=data,monitor=SimpleNamespace(
            geoms={'part':[model.geom('part_shape').id]},descendant_geoms=lambda _:[model.geom('plate').id]))
        self.assertTrue(socket.payload_clear_of_board(t,'part','board'))
        data.joint('part_free').qpos[0]=.11;mujoco.mj_forward(model,data)
        self.assertFalse(socket.payload_clear_of_board(t,'part','board'))

    def test_aperture_fault_needs_known_result_and_physical_release(self):
        plan=dict(pre=np.array([0,0,.14]),rotation=np.eye(3),pre_q=np.zeros(7),object_rotation=np.eye(3))
        for status,reason,released,can_withdraw in (
            ('failed','Requested aperture exceeded',True,True),
            ('failed','Requested opening not reached',False,False),
            ('uncertain','Timeout',True,False),
            ('failed','Driver fault',True,False)):
            with self.subTest(status=status,reason=reason):
                result=dict(status=status,reason=reason)
                trial=SimpleNamespace(held={'A':'part'},task={'cube_size_m':.06},
                    arms={'A':SimpleNamespace(site=0,grip_max=.08,grasp_force=None)},
                    data=SimpleNamespace(site_xpos=np.zeros((1,3)),site_xmat=np.eye(3).reshape(1,9),
                        body=lambda _:SimpleNamespace(xpos=np.zeros(3),xmat=np.eye(3).ravel())),
                    execute=Mock(return_value={'A':result}),move=Mock(),move_free=Mock(),event=Mock(),wait=Mock(),
                    finger_contacts=lambda *_:True,monitor=SimpleNamespace(geoms={'part':[]},released=lambda _:released))
                with patch.object(reference,'_planning_copy',return_value=object()), \
                     patch.object(reference,'socket_contact_clearance',return_value=nullcontext()), \
                     patch.object(socket,'seating_candidate',return_value=(plan,{})):
                    if can_withdraw:socket.place_socket(trial,'A',np.zeros(3),np.eye(3),'board')
                    else:
                        with self.assertRaises(socket.SocketPlanningError):
                            socket.place_socket(trial,'A',np.zeros(3),np.eye(3),'board')
                self.assertEqual(trial.move_free.call_count,1)
                # Only known release feedback permits withdrawal. A known
                # aperture miss with contact still present is checked again
                # after withdrawal and is not converted into success.
                withdrew=status=='completed' or (status=='failed' and reason in ('Requested aperture exceeded','Requested opening not reached'))
                self.assertEqual(trial.move.call_count,8 if withdrew else 5)
                self.assertEqual(result['status'],status)

    def test_sideways_object_grip_reserves_hand_space_at_both_ends(self):
        # A quarter-turned part admits an oblique approach with the palm
        # above both support planes. The old 0/60/90-degree family missed
        # the balanced 45-degree grip and searched sideways wrists first.
        rotation=np.array([[1.,0,0],[0,0,-1],[0,1,0]])
        _,frames=reference._pickup_frames('cube',rotation)
        _,tool=insertion_grasps(frames,rotation,np.eye(3))[0]
        final_tool=rotation.T@tool
        self.assertGreater(-tool[2,2],.70)
        self.assertGreater(-final_tool[2,2],.70)

    def test_trace_rejects_mid_segment_collision_with_clear_endpoints(self):
        trial=SimpleNamespace(_reference_planning_only=True,data=object(),_live_data=object(),
            check_stop=lambda:None,arms={'A':SimpleNamespace(ik=SimpleNamespace(pose=lambda q:(q[:3],np.eye(3))))},
            screen_ik=lambda robot,p,r,q:np.r_[p,np.zeros(4)],screen_data=SimpleNamespace(contact=[]))
        def clear(t,robot,name,q,p,r,board,*args):
            return (not .004<q[0]<.006),None
        with patch.object(socket,'sample_clear',side_effect=clear):
            path,why=socket.trace_segment(trial,'A','part',np.zeros(3),np.array([.01,0,0]),
                np.eye(3),np.eye(3),np.zeros(7),np.zeros(3),np.eye(3),'board')
        self.assertIsNone(path)
        self.assertEqual(why['reason'],'joint_edge_clearance')

    def test_trace_rejects_ik_branch_jump(self):
        trial=SimpleNamespace(_reference_planning_only=True,data=object(),_live_data=object(),
            check_stop=lambda:None,screen_ik=lambda *args:np.ones(7))
        path,why=socket.trace_segment(trial,'A','part',np.zeros(3),np.array([.01,0,0]),
            np.eye(3),np.eye(3),np.zeros(7),np.zeros(3),np.eye(3),'board')
        self.assertIsNone(path);self.assertEqual(why['reason'],'joint_branch_jump')

    def test_live_data_is_not_accepted_as_planning_scratch(self):
        data=object()
        with self.assertRaisesRegex(RuntimeError,'independent planning'):
            socket.require_scratch(SimpleNamespace(_reference_planning_only=True,data=data,_live_data=data))

    def test_archived_self_collision_is_rejected_before_controller_submission(self):
        fixture=json.loads((ROOT/'tests/fixtures/transport_socket_self_collision.json').read_text())
        scene=ROOT/fixture['scene'];t=Trial.__new__(Trial)
        t.model=mujoco.MjModel.from_xml_path(str(scene/'scene.xml'));t.data=mujoco.MjData(t.model)
        t.data.qpos[:]=fixture['qpos'];mujoco.mj_forward(t.model,t.data)
        t.task=json.loads((scene/'task.json').read_text());t.env=json.loads((scene/'environment.json').read_text())
        t.arms={r:PandaArm(t.model,t.data,r) for r in t.task['arms']}
        t.monitor=Monitor(t);t.screen_model=copy(t.model)
        t.screen_model.geom_margin[:]=np.maximum(t.screen_model.geom_margin,.004)
        t.screen_data=mujoco.MjData(t.screen_model)
        t.stop_requested=None;t.live_preview=False;t.held={'H':'H_item2'}
        t.carry_geoms={};t.hand_geoms={};t.event=Mock();original=Mock();t.execute=original
        before=t.data.qpos.copy();socket.guard_contact_moves(t,fixture['board'])
        with self.assertRaises(socket.SocketPlanningError):
            t.execute('place-prealign',{'H':fixture['command']})
        original.assert_not_called()
        np.testing.assert_array_equal(t.data.qpos,before)
        self.assertEqual(t.held,{'H':'H_item2'})
        self.assertEqual(t.event.call_args.args[0],'socket_motion_rejected')


if __name__=='__main__':unittest.main()
