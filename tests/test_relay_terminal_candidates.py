"""A terminal search must reach low, verified workspace candidates promptly."""
import json
import unittest
from unittest.mock import patch
import numpy as np
import mujoco
from robofrontier.workstation import Workstation
from robofrontier.suite.v2 import cases,directory
from robofrontier.suite.oracle import Trial
from robofrontier.suite.evaluate import Monitor
from robofrontier.robot_control import PandaArm
from robofrontier.suite import relay_oracle_v2 as ref

class Captured(Exception):pass

class RelayTerminalCandidateTests(unittest.TestCase):
    def world(self):
        source=Workstation(directory(next(c for c in cases() if c['id']=='relay-a08-d1')),verify_assets=False)
        source.frame();t=Trial.__new__(Trial);t.model,t.data=source.model,source.data
        t.task=json.loads((source.path/'task.json').read_text());t.stop_requested=None;t.live_preview=False
        t.data.joint('payload_02_free').qpos[2]=1.35;mujoco.mj_forward(t.model,t.data)
        t.arms={a:PandaArm(t.model,t.data,a) for a in t.task['arms']};t.monitor=Monitor(t)
        return t

    def test_terminal_search_tries_lower_clearance_height_before_tall_hold(self):
        t=self.world();seen=[]
        def capture(world,ident,team,point,yaw,*args,**kwargs):seen.append(float(point[2]));raise Captured()
        with patch.object(ref,'carry_clear',side_effect=capture),self.assertRaises(Captured):
            ref.handover_plan(t,'payload_02',['E','F'],.44,3,search_budget_s=5.)
        self.assertAlmostEqual(seen[0],1.15)

    def test_terminal_future_preview_does_not_force_receiver_back_up(self):
        t=self.world();seen=[]
        def clear(world,ident,team,point,yaw,*args,**kwargs):
            world.relay_carry_endpoint=dict(qpos=world.data.qpos.copy(),rotation=ref.rz(yaw));return True
        def receive(world,*args,**kwargs):seen.append(kwargs);raise Captured()
        with patch.object(ref,'carry_clear',side_effect=clear),patch.object(ref,'choose_team',side_effect=receive),self.assertRaises(Captured):
            ref.handover_plan(t,'payload_02',['E','F'],.44,3,search_budget_s=5.)
        self.assertAlmostEqual(seen[0]['future_waypoints'][0][0][2],1.15)

if __name__=='__main__':unittest.main()
