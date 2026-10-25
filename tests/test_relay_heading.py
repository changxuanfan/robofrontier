"""Exercise heading proposals at the actual handover call boundary, without motion."""
from copy import deepcopy
import json
import math
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from robofrontier.robot_control import PandaArm
from robofrontier.suite.oracle import Trial
from robofrontier.suite.evaluate import Monitor
from robofrontier.suite import relay_oracle_v2 as ref
from robofrontier.suite.v2 import cases,directory
from robofrontier.workstation import Workstation


class Captured(Exception):pass


class RelayHeadingTests(unittest.TestCase):
    def world(self,case_id):
        source=Workstation(directory(next(c for c in cases() if c['id']==case_id)),verify_assets=False)
        source.frame();t=Trial.__new__(Trial);t.model,t.data=source.model,source.data
        t.task=json.loads((source.path/'task.json').read_text());t.stop_requested=None;t.live_preview=False
        t.arms={a:PandaArm(t.model,t.data,a) for a in t.task['arms']};t.monitor=Monitor(t)
        return t

    def test_first_handover_proposal_keeps_measured_heading_instead_of_turning_180(self):
        for case_id,team in [('relay-a08-d2',['A','B']),('relay-a12-d3',['A','H'])]:
            with self.subTest(case=case_id):
                t=self.world(case_id);before=deepcopy(t.task['route']);rotation=t.data.body('payload_01').xmat.reshape(3,3)
                measured=math.atan2(rotation[1,0],rotation[0,0]);seen=[]
                def observe(world,ident,holders,point,yaw,*args,**kwargs):
                    seen.append(yaw);raise Captured()
                with patch.object(ref,'carry_clear',side_effect=observe),self.assertRaises(Captured):
                    ref.handover_plan(t,'payload_01',team,.44,1,search_budget_s=5.)
                error=math.atan2(math.sin(seen[0]-measured),math.cos(seen[0]-measured))
                self.assertAlmostEqual(error,0.,places=10)
                self.assertEqual(t.task['route'],before)

    def test_receiving_future_waypoint_uses_payload_heading_without_rewriting_region(self):
        t=self.world('relay-a08-d2');before=deepcopy(t.task['route']);seen=[]
        def clear(world,ident,holders,point,yaw,*args,**kwargs):
            world.relay_carry_endpoint=dict(qpos=world.data.qpos.copy(),rotation=ref.rz(yaw));return True
        def receive(world,*args,**kwargs):seen.append(kwargs);raise Captured()
        with patch.object(ref,'carry_clear',side_effect=clear),patch.object(ref,'choose_team',side_effect=receive),self.assertRaises(Captured):
            ref.handover_plan(t,'payload_01',['A','B'],.44,1,search_budget_s=5.)
        self.assertAlmostEqual(math.cos(seen[0]['future_waypoints'][0][1]),-1.,places=10)
        self.assertEqual(t.task['route'],before)

    def test_failed_lift_preparation_does_not_start_placement(self):
        from unittest.mock import Mock
        t=self.world('relay-a08-d1');observer=Mock()
        with patch.object(ref,'choose_team',return_value=['E','F']),patch.object(ref,'grasp'),patch.object(ref,'carry_clear',return_value=False),patch.object(ref,'place') as place:
            with self.assertRaisesRegex(RuntimeError,'No whole-arm/beam-clear sampled lift'):
                ref.solve(t,payload_ids=['payload_02'],terminal_observer=observer)
        self.assertEqual([call.args[0] for call in observer.call_args_list],['preparation_started'])
        place.assert_not_called()

    def test_placement_notification_occurs_only_after_supported_preparation(self):
        from unittest.mock import Mock
        t=self.world('relay-a08-d1');t.wait=Mock();events=[]
        # One-region call-chain fixture, not a physical task success.
        t.task['route']=t.task['route'][-1:]
        for item in t.task['mechanism']['payload_assemblies']:item['initial_route_stage_id']=t.task['route'][0]['region_id']
        observer=lambda phase,*_:events.append(phase)
        def enter(*_):events.append('actual_place_entered');raise Captured()
        with patch.object(ref,'choose_team',return_value=['E','F']),patch.object(ref,'grasp'),patch.object(ref,'carry_clear',return_value=True),patch.object(ref,'carry'),patch.object(t.monitor.relay_v2,'inside',return_value=True),patch.object(t.monitor,'bilateral',return_value={'E','F'}),patch.object(ref,'place',side_effect=enter),self.assertRaises(Captured):
            ref.solve(t,payload_ids=['payload_02'],terminal_observer=observer)
        self.assertEqual(events,['preparation_started','prepared','placement_started','actual_place_entered'])


if __name__=='__main__':unittest.main()
