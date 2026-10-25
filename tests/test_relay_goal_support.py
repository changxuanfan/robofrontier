"""Allowed stacks need actual contact paths to the tabletop, not robot holds."""
from types import SimpleNamespace
import unittest
import mujoco
import numpy as np
from robofrontier.suite.evaluate_v2 import Relay


class GoalSupportTests(unittest.TestCase):
    def world(self,weight=False):
        extra='<body name="weight" pos=".3 0 .8249"><freejoint name="weight_free"/><geom type="box" size=".03 .03 .025"/></body>' if weight else ''
        model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <geom name="table_0" type="box" pos="0 0 .775" size="1 1 .025"/>
          <geom name="floor" type="plane" size="2 2 .1"/>
          <body name="p2" pos="0 0 .8249"><freejoint name="p2_free"/><geom name="p2_beam" type="box" size=".1 .1 .025"/></body>
          <body name="p1" pos="0 0 .8747"><freejoint name="p1_free"/><geom name="p1_beam" type="box" size=".1 .1 .025"/></body>
        '''+extra+'''</worldbody></mujoco>''')
        data=mujoco.MjData(model);mujoco.mj_forward(model,data)
        task=dict(table_height_m=.8,route=[dict(center_m=[0,0,.8],nominal_yaw_rad=0.,half_size_m=[.4,.4])],
            mechanism=dict(payload_assemblies=[dict(id='p1',weight_body='weight' if weight else None),dict(id='p2')]))
        mon=SimpleNamespace(m=model,d=data,task=task,descendant_geoms=lambda bid:np.flatnonzero(model.geom_bodyid==bid).tolist())
        return model,data,Relay(mon)

    def test_upper_rod_is_supported_through_lower_rod_in_goal(self):
        model,data,relay=self.world()
        self.assertEqual(relay.supported_payloads(),{'p1','p2'})

    def test_air_gap_is_not_a_support_edge(self):
        model,data,relay=self.world();data.joint('p1_free').qpos[2]=.89;mujoco.mj_forward(model,data)
        self.assertEqual(relay.supported_payloads(),{'p2'})

    def test_outside_cargo_and_floor_do_not_certify_goal_support(self):
        model,data,relay=self.world()
        for name in ['p1','p2']:data.joint(name+'_free').qpos[0]=.6
        mujoco.mj_forward(model,data)
        self.assertEqual(relay.supported_payloads(),set())
        self.assertEqual(relay.supported_payloads(False),{'p1','p2'})
        data.joint('p2_free').qpos[:3]=[0,0,.0249];data.joint('p1_free').qpos[:3]=[0,0,.0747];mujoco.mj_forward(model,data)
        self.assertEqual(relay.supported_payloads(),set())

    def test_free_weight_on_table_does_not_weld_a_floating_beam_to_ground(self):
        model,data,relay=self.world(True);data.joint('p1_free').qpos[2]=1.2;mujoco.mj_forward(model,data)
        self.assertNotIn('p1',relay.supported_payloads())
        self.assertIn('p1',relay.supported_payloads(False,False))


if __name__=='__main__':unittest.main()
