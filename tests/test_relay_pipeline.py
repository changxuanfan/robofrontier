"""Simultaneous sweeps can conflict despite separate path/endpoint checks."""
from types import SimpleNamespace
from unittest.mock import patch,Mock
import unittest
import tempfile,time
from copy import deepcopy
from pathlib import Path

import mujoco
import numpy as np

from robofrontier.suite.relay_pipeline import simultaneous_carry_clear,carry_chunk,Frontier,packing_shift,solve,extra_prefix_plan,plan_transfer,route_progress,direct_terminal_plan
from tests.test_relay_descent_recovery import recorded_world
from robofrontier.suite import relay_oracle_v2 as ref


class RelayPipelineTests(unittest.TestCase):
    def world(self):
        model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <body name="p1" pos="-.4 -.2 .5"><freejoint name="p1_free"/><geom type="box" size=".03 .03 .03"/></body>
          <body name="p2" pos="0 .4 .5"><freejoint name="p2_free"/><geom type="box" size=".03 .03 .03"/></body>
        </worldbody></mujoco>''')
        data=mujoco.MjData(model);mujoco.mj_forward(model,data)
        states={name:dict(all_geoms=[int(model.body_geomadr[model.body(name).id])],assembly={}) for name in ['p1','p2']}
        return SimpleNamespace(model=model,data=data,task={'completion':{}},arms={a:SimpleNamespace(qids=np.array([],dtype=int)) for a in ['A','B']},
            monitor=SimpleNamespace(geom_owner=np.full(model.ngeom,-1),relay_v2=SimpleNamespace(states=states)))

    def requests(self,second):
        return [dict(ident='p1',team=['A'],point=np.array([.4,-.2,.5]),yaw=0.,rotation=np.eye(3)),
                dict(ident='p2',team=['B'],point=np.array(second),yaw=0.,rotation=np.eye(3))]

    def screen(self,t,ident,team,point,*args,**kwargs):
        qpos=t.data.qpos.copy();adr=t.model.jnt_qposadr[t.model.joint(ident+'_free').id];qpos[adr:adr+3]=point
        t.relay_carry_endpoint=dict(qpos=qpos,q={a:np.array([]) for a in team});return True

    def test_crossing_payloads_are_rejected_between_clear_endpoints(self):
        t=self.world();before=t.data.qpos.copy()
        with patch('robofrontier.suite.relay_pipeline.ref.carry_clear',side_effect=self.screen):
            self.assertFalse(simultaneous_carry_clear(t,self.requests([0,-.8,.5])))
        np.testing.assert_array_equal(t.data.qpos,before)

    def test_spatially_separate_payloads_can_move_together(self):
        t=self.world()
        with patch('robofrontier.suite.relay_pipeline.ref.carry_clear',side_effect=self.screen):
            self.assertTrue(simultaneous_carry_clear(t,self.requests([.4,.4,.5])))

    def test_same_robot_cannot_be_reserved_for_two_loads(self):
        requests=self.requests([.4,.4,.5]);requests[1]['team']=['A']
        self.assertFalse(simultaneous_carry_clear(self.world(),requests))

    def test_receiving_lookahead_keeps_other_load_holders_occupied(self):
        t=recorded_world();t.held={'A':'payload_01','B':'payload_01','C':'payload_02','D':'payload_02'}
        original=dict(t.held)
        t.relay_grasp_plan={a:dict(contact_q=t.home_q[a].copy()) for a in ['E','F']}
        def clear(probe,*args,**kwargs):
            probe.relay_carry_endpoint=dict(qpos=probe.data.qpos.copy());return True
        future=dict(future_region=[0.,0.,1.35],future_height=1.35,future_yaw=0.,future_waypoints=[])
        with patch.object(ref,'carry_clear',side_effect=clear):
            probe=ref.received_transport_preview(t,'payload_01',['A','B'],['E','F'],future)
        self.assertEqual(probe.held,{'C':'payload_02','D':'payload_02','E':'payload_01','F':'payload_01'})
        self.assertEqual(t.held,original)

    def test_in_goal_supported_load_can_descend_without_nominal_center_correction(self):
        t=recorded_world();point=t.data.body('payload_02').xpos.copy()
        f=Frontier('payload_02',1,3,phase='carry',team=['G','H'],target=point+[.013,0,0],following='place')
        with patch.object(t.monitor,'bilateral',return_value={'G','H'}):
            self.assertIsNone(carry_chunk(t,f))

    def test_goal_readiness_does_not_skip_an_intermediate_transfer(self):
        t=recorded_world();point=t.data.body('payload_02').xpos.copy()
        f=Frontier('payload_02',1,3,phase='carry',team=['G','H'],target=point+[.013,0,0],following='receive')
        self.assertIsNotNone(carry_chunk(t,f))

    def test_descent_clearance_alone_does_not_certify_opening_beside_finished_rod(self):
        t=recorded_world('relay_terminal_opening_corridor.json');point=t.data.body('payload_01').xpos.copy()
        t.task['completion']['allow_goal_payload_contact']=False # Legacy private forecast.
        f=Frontier('payload_01',0,3,phase='carry',team=['G','H'],target=np.array([1.395,0,point[2]]),following='place')
        with patch.object(t.monitor,'bilateral',return_value={'G','H'}):
            self.assertIsNotNone(carry_chunk(t,f))

    def test_landing_candidate_preserves_space_for_an_already_delivered_rod(self):
        t=recorded_world();t.data.joint('payload_02_free').qpos[0]=1.595;mujoco.mj_forward(t.model,t.data)
        first=Frontier('payload_01',0,2);second=Frontier('payload_02',1,3)
        before=t.data.qpos.copy()
        with patch.object(t.monitor,'result',return_value={'payloads':{'payload_02':{'completed':True}}}):
            self.assertEqual(packing_shift(t,first,[first,second]),-.12)
        np.testing.assert_array_equal(t.data.qpos,before)

    def test_enabled_terminal_release_precedes_search_for_its_occupied_hands(self):
        t=recorded_world();t.actions=[];t.started=time.monotonic();t.step=Mock();order=[]
        t.relay_frontiers=[Frontier('payload_01',0,2,phase='plan',team=['E','F']),
            Frontier('payload_02',1,3,phase='place',team=['G','H'])]
        result={'payloads':{}}
        def release(*args,**kwargs):order.append('release');result['payloads']['payload_02']={'completed':True}
        def plan(*args,**kwargs):order.append('plan');raise LookupError('Stop before further planning')
        with tempfile.TemporaryDirectory() as folder:
            t.directory=Path(folder)
            with patch.object(t.monitor,'result',side_effect=lambda:result),patch('robofrontier.suite.relay_pipeline.ref.place',side_effect=lambda *a:order.append('place')),patch('robofrontier.suite.relay_pipeline.ref.release',side_effect=release),patch('robofrontier.suite.relay_pipeline.ref.handover_plan',side_effect=plan):
                with self.assertRaises(LookupError):solve(t)
        self.assertEqual(order,['place','release','plan'])

    def test_arrived_terminal_carry_is_promoted_before_upstream_search(self):
        t=recorded_world();t.actions=[];t.started=time.monotonic();t.step=Mock();order=[]
        point=t.data.body('payload_02').xpos.copy()
        t.relay_frontiers=[Frontier('payload_01',0,2,phase='plan',team=['E','F']),
            Frontier('payload_02',1,3,phase='carry',team=['G','H'],target=point,following='place')]
        def record(name):
            order.append(name);raise LookupError('Stop at first enabled work')
        with tempfile.TemporaryDirectory() as folder:
            t.directory=Path(folder)
            with patch.object(t.monitor,'result',return_value={'payloads':{}}),patch('robofrontier.suite.relay_pipeline.carry_chunk',return_value=None),patch('robofrontier.suite.relay_pipeline.ref.place',side_effect=lambda *a:record('place')),patch('robofrontier.suite.relay_pipeline.ref.handover_plan',side_effect=lambda *a,**k:record('plan')):
                with self.assertRaises(LookupError):solve(t)
        self.assertEqual(order,['place'])

    def upstream_holder(self):
        t=recorded_world();t.data.joint('payload_02_free').qpos[:3]=[.04,0,1.35];mujoco.mj_forward(t.model,t.data)
        t.held={'A':'payload_01','B':'payload_01','C':'payload_02','D':'payload_02'}
        return t,Frontier('payload_02',1,2,phase='plan',team=['D','C'])

    def test_extra_anchor_is_private_and_preserves_actual_goal_and_state(self):
        t,f=self.upstream_holder();task=deepcopy(t.task);qpos=t.data.qpos.copy()
        with patch('robofrontier.suite.relay_pipeline.ref.handover_plan',return_value=('finite','candidate')) as planner:
            self.assertEqual(extra_prefix_plan(t,f,5.),('finite','candidate'))
        probe=planner.call_args.args[0]
        self.assertEqual(len(probe.task['route']),len(task['route'])+1)
        self.assertEqual(probe.task['route'][-1],task['route'][-1])
        self.assertIsNot(probe.task,t.task);self.assertEqual(t.task,task)
        np.testing.assert_array_equal(t.data.qpos,qpos)

    def test_actual_upstream_custody_gets_a_useful_prefix_before_final_delivery(self):
        t,f=self.upstream_holder()
        with patch('robofrontier.suite.relay_pipeline.ref.handover_plan',return_value=('finite','candidate')) as planner:
            result,is_extra=plan_transfer(t,f,[f],False)
        self.assertTrue(is_extra);self.assertEqual(result,('finite','candidate'))
        self.assertEqual(planner.call_args.kwargs['lookahead_depth'],1)

    def test_arc_progress_does_not_require_decreasing_straight_line_goal_distance(self):
        route=[{'center_m':[x,y,.8]} for x,y in [(-1,-.48),(-1,0),(-.866,.5),(0,1),(1,0),(1,-.48)]]
        start=np.array([-1.,-.38]);next_point=np.array([-.866,.5]);goal=np.array([1.,-.48])
        self.assertGreater(np.linalg.norm(next_point-goal),np.linalg.norm(start-goal))
        self.assertGreater(route_progress(route,next_point),route_progress(route,start))

    def test_qualified_last_holders_need_no_further_handover_inside_goal(self):
        t=recorded_world();f=Frontier('payload_02',1,2,phase='plan',team=['G','H'])
        with patch.object(t.monitor,'bilateral',return_value={'G','H'}):
            self.assertIsNone(direct_terminal_plan(t,f,[f]))
            t.monitor.relay_v2.states[f.ident]['transitions']=[dict(senders=['E','F'],receivers=['G','H'])]
            plan=direct_terminal_plan(t,f,[f])
        self.assertIsNotNone(plan)
        np.testing.assert_array_equal(plan[0],t.data.body(f.ident).xpos)


if __name__=='__main__':unittest.main()
