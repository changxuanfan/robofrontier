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


def recorded_frontier():
    fixture=json.loads((ROOT/'tests/fixtures/transport_air_frontier.json').read_text())
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
    def test_original_blocked_frontier_has_a_receivable_successor_candidate(self):
        t=recorded_frontier();goal=next(g for g in t.task['goals'] if g['goal_id']=='B_slot2')
        class ReachedExecution(Exception):pass
        t.move_free=Mock(side_effect=ReachedExecution)
        before=t.data.qpos.copy()
        with self.assertRaises(ReachedExecution):
            full.air_transfer(t,'D','D_item2',['B'],goal)
        np.testing.assert_array_equal(t.data.qpos,before)


if __name__=='__main__':unittest.main()
