"""Simultaneous sweeps can conflict despite separate path/endpoint checks."""
from types import SimpleNamespace
from unittest.mock import patch
import unittest

import mujoco
import numpy as np

from robofrontier.suite.relay_pipeline import simultaneous_carry_clear,carry_chunk,Frontier
from tests.test_relay_descent_recovery import recorded_world


class RelayPipelineTests(unittest.TestCase):
    def world(self):
        model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <body name="p1" pos="-.4 -.2 .5"><freejoint name="p1_free"/><geom type="box" size=".03 .03 .03"/></body>
          <body name="p2" pos="0 .4 .5"><freejoint name="p2_free"/><geom type="box" size=".03 .03 .03"/></body>
        </worldbody></mujoco>''')
        data=mujoco.MjData(model);mujoco.mj_forward(model,data)
        states={name:dict(all_geoms=[int(model.body_geomadr[model.body(name).id])],assembly={}) for name in ['p1','p2']}
        return SimpleNamespace(model=model,data=data,arms={a:SimpleNamespace(qids=np.array([],dtype=int)) for a in ['A','B']},
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

    def test_in_goal_supported_load_can_descend_without_nominal_center_correction(self):
        t=recorded_world();point=t.data.body('payload_02').xpos.copy()
        f=Frontier('payload_02',1,3,phase='carry',team=['G','H'],target=point+[.013,0,0],following='place')
        with patch.object(t.monitor,'bilateral',return_value={'G','H'}):
            self.assertIsNone(carry_chunk(t,f))

    def test_goal_readiness_does_not_skip_an_intermediate_transfer(self):
        t=recorded_world();point=t.data.body('payload_02').xpos.copy()
        f=Frontier('payload_02',1,3,phase='carry',team=['G','H'],target=point+[.013,0,0],following='receive')
        self.assertIsNotNone(carry_chunk(t,f))


if __name__=='__main__':unittest.main()
