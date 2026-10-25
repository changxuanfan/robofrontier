"""Actual recorded local IK failure must admit a screened finite successor."""
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
from robofrontier.suite import relay_oracle_v2 as ref
from robofrontier.suite.v2 import cases,directory


def recorded_world(fixture_name='relay_terminal_local_branch.json'):
    fixture=json.loads((Path(__file__).parent/'fixtures'/fixture_name).read_text())
    scene=directory(next(c for c in cases() if c['id']==fixture['case_id']))
    t=Trial.__new__(Trial);t.model=mujoco.MjModel.from_xml_path(str(scene/'scene.xml'));t.data=mujoco.MjData(t.model)
    mujoco.mj_resetDataKeyframe(t.model,t.data,t.model.key('initial').id);mujoco.mj_forward(t.model,t.data)
    t.task=json.loads((scene/'task.json').read_text());t.stop_requested=None;t.live_preview=False
    if t.task['scene_sha256']!=fixture['source_scene_sha256']:raise ValueError('Regression fixture model identity differs')
    t.arms={a:PandaArm(t.model,t.data,a) for a in t.task['arms']}
    t.home_q={a:t.data.qpos[arm.qids].copy() for a,arm in t.arms.items()}
    t.home={a:(t.data.site_xpos[arm.site].copy(),t.data.site_xmat[arm.site].reshape(3,3).copy()) for a,arm in t.arms.items()}
    t.data.qpos[:]=fixture['qpos'];mujoco.mj_forward(t.model,t.data)
    for arm in t.arms.values():arm.qref=t.data.qpos[arm.qids].copy()
    t.screen_model=copy(t.model);t.screen_model.geom_margin[:]=np.maximum(t.screen_model.geom_margin,.004)
    t.screen_data=mujoco.MjData(t.screen_model);t.held=fixture['held'];t.carry_geoms={};t.hand_geoms={};t.monitor=Monitor(t)
    t._live_data=t.data;t.event=Mock();t.wait=Mock()
    return t


class CandidateChosen(Exception):pass


class RelayDescentRecoveryTests(unittest.TestCase):
    def test_candidate_preserves_source_state_and_checks_lowering_successor(self):
        t=recorded_world();q=t.data.qpos.copy();ctrl=t.data.ctrl.copy()
        refs={a:v.qref.copy() for a,v in t.arms.items()}
        plan=ref.terminal_reposition_plan(t,'payload_02',['G','H'])
        self.assertIsNotNone(plan)
        self.assertEqual(plan['delta_m'],[0,-.02,0])
        self.assertLess(plan['predicted_descent_target_m'][2],.85)
        np.testing.assert_array_equal(t.data.qpos,q);np.testing.assert_array_equal(t.data.ctrl,ctrl)
        for a,v in t.arms.items():np.testing.assert_array_equal(v.qref,refs[a])

    def test_rejected_descent_is_followed_by_screened_coupled_adjustment(self):
        t=recorded_world()
        self.assertIsNone(ref.landing_increment_clear(t,'payload_02',['G','H'],.005))
        with patch.object(ref,'final_support',return_value=False),patch.object(t.monitor,'bilateral',return_value={'G','H'}),patch.object(ref,'carry',side_effect=CandidateChosen) as carry:
            with self.assertRaises(CandidateChosen):ref.place(t,'payload_02',['G','H'])
        self.assertEqual(carry.call_args.args[2],['G','H'])

    def test_pending_original_action_prevents_new_recovery_motion(self):
        t=recorded_world();t.arms['H'].job='original-unresolved'
        with patch.object(ref,'final_support',return_value=False),patch.object(t.monitor,'bilateral',return_value={'G','H'}),patch.object(ref,'carry') as carry:
            with self.assertRaisesRegex(RuntimeError,'No whole-arm-clear terminal descent'):
                ref.place(t,'payload_02',['G','H'])
        carry.assert_not_called()


if __name__=='__main__':unittest.main()
