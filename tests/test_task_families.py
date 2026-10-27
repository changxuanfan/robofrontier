import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from robofrontier.run_setup import verify_inputs
from robofrontier.task_families import (ROOT, RELEASE, read, resolve_case,
    policy_task, validate_policy_task, validate_frozen_policy, prepare)


class FamilyReleaseTests(unittest.TestCase):
    def test_all_purposes_have_36_exact_sources_and_preserve_version_split(self):
        release = read(ROOT/RELEASE)
        for purpose in ('gpt', 'oracle'):
            rows = [c for f in release['families'].values() for c in f[purpose]]
            self.assertEqual(len(rows), 36)
            self.assertEqual(len({c['id'] for c in rows}), 36)
            for c in rows:
                resolve_case(c['id'], purpose)
        mosaic = release['families']['mosaic']
        self.assertIn('r14', mosaic['gpt'][0]['directory'])
        self.assertNotEqual(mosaic['gpt'][0]['source_files']['scene.xml'],
                            mosaic['oracle'][0]['source_files']['scene.xml'])

    def test_prepared_task_is_not_an_inherited_oracle_pass_and_detects_edits(self):
        task = policy_task('transport-a04-d1')
        validate_policy_task(task)
        self.assertFalse(task['oracle_verified'])
        self.assertIsNone(task['run_protocol']['total_wall_limit_s'])
        task['completion']['stable_duration_sim_s'] = 0
        with self.assertRaisesRegex(ValueError, 'changed outside'):
            validate_policy_task(task)

    def test_each_family_freezes_guidance_public_facts_and_private_release(self):
        cases = ['transport-a04-d1', 'mosaic-a04-6x12-d1',
                 'relay-a08-d1', 'dial-best5-n04-d1']
        with tempfile.TemporaryDirectory() as tmp, patch('mujoco.mj_step', side_effect=AssertionError('No physics')):
            for case in cases:
                run = prepare(case, Path(tmp)/case)
                self.assertEqual(verify_inputs(run), [])
                public = read(run/'public/inputs/task.json')
                task = read(run/'private/inputs/task.json')
                validate_frozen_policy(task, run)
                task['run_protocol']['experience']='reuse_oracle'
                with self.assertRaisesRegex(ValueError, 'protocol differs'):
                    validate_frozen_policy(task, run)
                task = read(run/'private/inputs/task.json')
                self.assertIn('completion', public)
                self.assertIn('workspace_world_m', public)
                _, _, spec = resolve_case(case)
                if 'public_projection' in spec:
                    review = read(ROOT/spec['public_projection']['path'])
                    for key in ('rules', 'target', 'mechanism', 'completion', 'initial_supplies'):
                        if key in review:self.assertEqual(public[key], review[key])
                if 'initial_reference_images' in public:
                    self.assertTrue(all((run/'public/inputs'/name).is_file() for name in public['initial_reference_images']))
                self.assertTrue((run/'public/inputs'/public['family_guidance']).is_file())
                self.assertFalse(any(p.name in ('private-code.json', 'actions.json', 'process.jsonl')
                                     for p in (run/'public').rglob('*')))
                material = run/'public/inputs'/public['family_guidance']
                material.write_text('changed')
                self.assertIn('input:'+str(material.relative_to(run)), verify_inputs(run))
                scene = run/'private/inputs/scene/task.json'
                scene.write_text('{}')
                with self.assertRaisesRegex(ValueError, 'scene metadata'):
                    validate_frozen_policy(task, run)

    def test_each_family_starts_frozen_evaluator_and_recording_without_steps(self):
        from robofrontier.pipeline import World
        cases = ['transport-a04-d1', 'mosaic-a04-6x12-d1',
                 'relay-a08-d1', 'dial-best5-n04-d1']
        with tempfile.TemporaryDirectory() as tmp, patch('mujoco.mj_step', side_effect=AssertionError('No physics')):
            for case in cases:
                with self.subTest(case=case):
                    run = prepare(case, Path(tmp)/case)
                    world = World(run, read(run/'private/inputs/task.json'))
                    try:
                        world.start_trial()
                        self.assertIsNotNone(world.suite_sampler)
                        self.assertEqual(world.motion_recorder.meta['sample_hz'], 60)
                        self.assertEqual(world.time, 0.)
                    finally:
                        world.close()


if __name__ == '__main__':
    unittest.main()
