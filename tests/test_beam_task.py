"""Task portability, private process scoring and live-world information limits."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from robofrontier.beam_task import make_beam_task, public_beam_task, validate_beam_task, BeamEvaluator
from robofrontier.beam_handoff import DIRECTORY
from robofrontier.run_setup import prepare_run,verify_inputs
from robofrontier.task_config import load_harness,load_task
from robofrontier.pipeline import World


class BeamTests(unittest.TestCase):
    def setUp(self):self.task=make_beam_task(DIRECTORY)
    def sample(self,t,grip=('A','B'),table=False,inside=False,**kw):
        return dict(time_s=t,gripping_robots=list(grip),table_contact=table,floor_contact=False,
                    inside_goal=inside,bottom_z_m=.4 if table else .6,released=not grip,
                    linear_speed_m_s=0.,angular_speed_rad_s=0.,**kw)
    def feed(self,e,start,end,**kw):
        for t in np.arange(start,end+.001,.02):e.feed(self.sample(float(t),**kw))
    def transfer(self,e):
        self.feed(e,0,.10);self.feed(e,.12,.38,grip=('A','B','C','D'));self.feed(e,.40,.50,grip=('C','D'))
    def finish(self,e):
        self.feed(e,.52,.60,grip=('C','D'),table=True,inside=True)
        self.feed(e,.62,1.66,grip=(),table=True,inside=True)
    def test_supported_airborne_handoff_and_finish(self):
        e=BeamEvaluator(self.task);self.transfer(e);self.finish(e);r=e.result()
        self.assertTrue(r['all_goals_met'],r);self.assertEqual(len(r['airborne_handovers']),1)
    def test_table_pass_and_initial_receiver_shortcut_do_not_pass(self):
        e=BeamEvaluator(self.task);self.feed(e,0,.1,table=True)
        self.feed(e,.12,.4,grip=('C','D'),table=True)
        self.feed(e,.42,1.7,grip=(),table=True,inside=True)
        self.assertFalse(e.result()['all_goals_met']);self.assertIn('pickup_by_non_sender',e.violations)
    def test_release_without_overlap_is_not_handoff(self):
        e=BeamEvaluator(self.task);self.feed(e,0,.1);self.feed(e,.12,.3,grip=('C','D'))
        self.assertEqual(e.handovers,[]);self.assertIn('unverified_handover',e.violations)
    def test_overlapping_teams_can_replace_one_supporter_and_contact_noise_is_not_release(self):
        e=BeamEvaluator(self.task);self.feed(e,0,.10)
        self.feed(e,.12,.38,grip=('A','B','C'))
        e.feed(self.sample(.40,grip=('B','C'),contacting_robots=['A','B','C']))
        self.assertEqual(e.handovers,[])
        e.feed(self.sample(.42,grip=('B','C')))
        self.assertEqual(e.custody,{'B','C'});self.assertEqual(len(e.handovers),1)
    def test_sender_drop_recovery_is_allowed_but_receiver_pickup_is_not(self):
        e=BeamEvaluator(self.task);self.feed(e,0,.1);self.feed(e,.12,.20,grip=(),table=True)
        self.assertTrue(e.recovering);self.assertEqual(e.drops,1)
        self.feed(e,.22,.3,grip=('A','B'),table=True);self.feed(e,.32,.4)
        self.assertFalse(e.recovering);self.assertEqual(e.violations,{})
        e2=BeamEvaluator(self.task);self.feed(e2,0,.1);self.feed(e2,.12,.2,grip=(),table=True)
        self.feed(e2,.22,.3,grip=('C','D'),table=True)
        self.assertIn('recovery_by_non_sender',e2.violations)
    def test_goal_declaration_cannot_excuse_unsupported_final_drop_or_sampling_gap(self):
        e=BeamEvaluator(self.task);self.transfer(e)
        self.feed(e,.52,.54,grip=(),inside=True);self.feed(e,.56,1.70,grip=(),table=True,inside=True)
        self.assertFalse(e.result()['all_goals_met']);self.assertIn('unsupported_final_landing',e.violations)
        e=BeamEvaluator(self.task);e.feed(self.sample(0));e.feed(self.sample(1))
        self.assertIn('incomplete_process_sampling',e.violations)
    def test_receivers_can_release_after_drop_but_cannot_assist_recovery_lift(self):
        e=BeamEvaluator(self.task);self.feed(e,0,.1);self.feed(e,.12,.38,grip=('A','B','C','D'))
        e.feed(self.sample(.40,grip=('A','B','C','D'),table=True))
        self.assertEqual(e.violations,{})
        self.feed(e,.42,.48,grip=('A','B'),table=True);self.feed(e,.50,.60)
        self.assertFalse(e.recovering);self.assertEqual(e.violations,{})
        e=BeamEvaluator(self.task);self.feed(e,0,.1);self.feed(e,.12,.38,grip=('A','B','C','D'))
        e.feed(self.sample(.40,grip=('A','B','C','D'),table=True))
        e.feed(self.sample(.42,grip=('A','B','C','D')))
        self.assertIn('recovery_by_non_sender',e.violations);self.assertTrue(e.recovering)
    def test_collision_and_disturbance_prevent_false_success(self):
        e=BeamEvaluator(self.task);self.transfer(e);self.finish(e)
        e.collision(1.,['A_link5','C_link7'],.002);self.assertFalse(e.result()['all_goals_met'])
        e=BeamEvaluator(self.task);self.transfer(e);self.finish(e)
        e.feed(self.sample(1.68,grip=(),table=True,inside=False));self.assertFalse(e.result()['all_goals_met'])
    def test_public_task_has_only_declared_static_inputs_and_scene_checks(self):
        public=public_beam_task(self.task);self.assertNotIn('scene_asset',public)
        for key in ('last_sample','contacts','grasp_points','nominal_plan'):self.assertNotIn(key,public)
        t=copy.deepcopy(self.task);t['observation_contract']['initial_positions_public']=False
        p=public_beam_task(t);self.assertNotIn('initial_position_m',p['objects'][0]);self.assertNotIn('initial_center_world_m',p['beam'])
        bad=copy.deepcopy(t);bad['objects'][0]['mass_kg']=1
        with self.assertRaises(ValueError):validate_beam_task(bad)
        bad=copy.deepcopy(t);bad['beam']['width_m']=.05
        with self.assertRaises(ValueError):validate_beam_task(bad)
    def test_every_topology_prepares_same_task_without_mosaic_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);task=root/'task.json';task.write_text(json.dumps(self.task))
            for mode in ('centralized','hierarchical','peer','adaptive'):
                cfg=load_harness();cfg['topology']=mode;p=root/(mode+'.json');p.write_text(json.dumps(cfg))
                run=prepare_run(root/mode,task,p)
                self.assertEqual(verify_inputs(run),[]);pub=json.loads((run/'public/inputs/task.json').read_text())
                self.assertEqual(pub['kind'],'beam_handoff');self.assertEqual(len(pub['cameras']),23)
                self.assertFalse((run/'public/inputs/target-pattern-draft.png').exists())
    def test_real_world_loads_and_sampler_does_not_leak_or_change_object_state(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);task=root/'task.json';task.write_text(json.dumps(self.task))
            run=prepare_run(root/'run',task);w=World(run,json.loads((run/'private/inputs/task.json').read_text()))
            w.start_trial();before=w.data.qpos.copy();w.beam_sampler.check_collisions()
            np.testing.assert_array_equal(w.data.qpos,before)
            w.step();r=w.evaluate();self.assertFalse(r['all_goals_met']);self.assertGreater(r['samples'],1)
            facts=w.robot_facts(load_harness());self.assertNotIn('last_sample',facts)
            self.assertNotIn('timber',json.dumps({a:r.observe() for a,r in w.arms.items()}))
            w.close()
            samples=[json.loads(line) for line in (run/'private/outputs/beam-process.jsonl').read_text().splitlines()]
            self.assertGreaterEqual(len(samples),1)
            self.assertNotIn('position_m',facts)

if __name__=='__main__':unittest.main()
