"""Beam policy/scoring contracts without loading or stepping a physics world."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from robofrontier.beam_task import (make_beam_task,public_beam_task,validate_beam_task,BeamEvaluator,evaluation_rules)
from robofrontier.task_config import ROOT,load_harness,panda_control_profile
from robofrontier.run_setup import prepare_run,verify_inputs


class BeamPolicyTests(unittest.TestCase):
    def setUp(self):self.task=make_beam_task(ROOT/'configs/environments/beam_handoff/facing-08')
    def feed(self,e,start,end,grip=('A','B'),table=False,inside=False,**extras):
        for tick in range(start,end+1):
            e.feed(dict(time_s=tick*.02,gripping_robots=list(grip),table_contact=table,floor_contact=False,
                        inside_goal=inside,bottom_z_m=.4 if table else .6,released=not grip,
                        linear_speed_m_s=0.,angular_speed_rad_s=0.,**extras))
    def transfer(self,e):
        self.feed(e,0,5);self.feed(e,6,19,grip=('A','B','C','D'));self.feed(e,20,25,grip=('C','D'))
    def finish(self,e):
        self.feed(e,26,30,grip=('C','D'),table=True,inside=True);self.feed(e,31,85,grip=(),table=True,inside=True)
    def test_completed_relay_and_public_scoring_share_one_definition(self):
        e=BeamEvaluator(self.task);self.transfer(e);self.finish(e)
        self.assertTrue(e.result()['task_success'])
        self.assertEqual(public_beam_task(self.task)['evaluation_rules'],evaluation_rules())
    def test_hard_failure_remains_failure_after_correct_final_placement(self):
        e=BeamEvaluator(self.task);self.transfer(e);e.collision(.5,['A_link6','C_link5'],.002);self.finish(e)
        self.assertFalse(e.result()['task_success'])
        self.assertIn('inter_robot_collision',public_beam_task(self.task)['evaluation_rules']['invalidating_events'])
    def test_drop_is_recoverable_by_sender_but_receiver_pickup_is_invalid(self):
        e=BeamEvaluator(self.task);self.feed(e,0,5);self.feed(e,6,8,grip=(),table=True)
        self.feed(e,9,10,grip=('A','B'),table=True);self.feed(e,11,15)
        self.assertFalse(e.recovering);self.assertEqual(e.drops,1);self.assertEqual(e.violations,{})
        self.feed(e,16,30,grip=('A','B','C','D'));self.feed(e,31,35,grip=('C','D'))
        self.feed(e,36,40,grip=('C','D'),table=True,inside=True);self.feed(e,41,95,grip=(),table=True,inside=True)
        self.assertTrue(e.result()['task_success'])
        e=BeamEvaluator(self.task);self.feed(e,0,5);self.feed(e,6,8,grip=(),table=True)
        self.feed(e,9,10,grip=('C','D'),table=True)
        self.assertIn('recovery_by_non_sender',e.violations)
    def test_partial_team_replacement_and_receiver_release_after_drop(self):
        e=BeamEvaluator(self.task);self.feed(e,0,5);self.feed(e,6,20,grip=('A','B','C'))
        self.feed(e,21,22,grip=('B','C'));self.assertEqual(e.custody,{'B','C'})
        e=BeamEvaluator(self.task);self.feed(e,0,5);self.feed(e,6,20,grip=('A','B','C','D'))
        self.feed(e,21,22,grip=('A','B','C','D'),table=True)
        self.feed(e,23,24,grip=('A','B'),table=True);self.feed(e,25,28)
        self.assertEqual(e.violations,{})
    def test_unverified_handover_and_unsupported_landing_cannot_pass(self):
        e=BeamEvaluator(self.task);self.feed(e,0,5);self.feed(e,6,15,grip=('C','D'));self.finish(e)
        self.assertIn('unverified_handover',e.violations);self.assertFalse(e.result()['task_success'])
        e=BeamEvaluator(self.task);self.transfer(e);self.feed(e,26,27,grip=(),inside=True)
        self.feed(e,28,90,grip=(),table=True,inside=True)
        self.assertIn('unsupported_final_landing',e.violations);self.assertFalse(e.result()['task_success'])
    def test_score_tampering_and_undeclared_failure_codes_are_rejected(self):
        bad=copy.deepcopy(self.task);bad['evaluation_rules']['invalidating_events'].pop('inter_robot_collision')
        with self.assertRaises(ValueError):validate_beam_task(bad)
        with self.assertRaises(ValueError):BeamEvaluator(self.task).violation(0,'hidden_new_rule')
    def test_public_inputs_have_static_geometry_no_private_reference_or_live_truth(self):
        p=public_beam_task(self.task)
        self.assertIn('planning_hints',p);self.assertNotIn('scene_asset',p)
        for forbidden in ('nominal-plan','attempt-07','last_sample','summed_normal_contact_force','grip_offsets'):
            self.assertNotIn(forbidden,json.dumps(p))
        self.task['observation_contract']['initial_positions_public']=False;p=public_beam_task(self.task)
        self.assertNotIn('initial_position_m',p['objects'][0]);self.assertNotIn('initial_center_world_m',p['beam'])
    def test_current_control_profile_does_not_claim_full_hardware_or_measured_force(self):
        p=panda_control_profile(load_harness())
        self.assertFalse(p['hardware_adapter_validated']);self.assertFalse(p['grasp_force_is_measured'])
        self.assertIn('agent_configurable_impedance',p['not_supported'])
        self.assertFalse(p['simulation_parameters_mutable_by_policy'])
    def test_task_freezes_in_every_topology_without_physics(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);t=root/'task.json';t.write_text(json.dumps(self.task))
            for mode in ('centralized','adaptive','hierarchical','peer'):
                cfg=load_harness();cfg['topology']=mode;c=root/(mode+'.json');c.write_text(json.dumps(cfg))
                run=prepare_run(root/mode,t,c);self.assertEqual(verify_inputs(run),[])
                p=json.loads((run/'public/inputs/task.json').read_text())
                self.assertEqual(p['evaluation_rules'],evaluation_rules())

    def test_invalid_beam_collision_dimension_count_is_rejected_without_physics(self):
        env=json.loads((ROOT/'configs/environments/beam_handoff/facing-08/environment.json').read_text())
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp)
            (folder/'scene.xml').write_text('<mujoco><worldbody><body name="timber"><geom size=".0225 .6" mass="1.458"/></body></worldbody></mujoco>')
            with patch('robofrontier.beam_task.validate_scene_asset',return_value=(folder,env)):
                with self.assertRaises(ValueError):validate_beam_task(self.task)

if __name__=='__main__':unittest.main()
