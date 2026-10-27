from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from robofrontier.suite.dial_policy_package import (
    REGISTRY, assert_public, build_review, public_review, read, registry, verify_review, static_geometry,
)
from robofrontier.workstation import Workstation


ROOT = Path(__file__).resolve().parents[1]


class DialPolicyPackageTests(unittest.TestCase):
    def test_all_nine_sources_and_evidence_are_bound_without_promoting_runtime(self):
        with patch('mujoco.mj_step',side_effect=AssertionError('No review physics')):
            value = registry(ROOT)
        self.assertEqual(value,read(ROOT/REGISTRY))
        self.assertEqual(len(value['cases']),9)
        self.assertEqual(sum(c['oracle_evidence']['assisted'] for c in value['cases']),1)
        self.assertFalse(value['formal_policy_ready'])
        self.assertTrue(all(c['runtime_readiness']=='pending_shared_integration' for c in value['cases']))

    def test_public_contract_preserves_best5_and_real_table_holes_without_object_truth(self):
        row = read(ROOT/REGISTRY)['cases'][-1]
        source = Workstation(ROOT/row['oracle_evidence']['recording']/'source')
        with patch('mujoco.mj_step',side_effect=AssertionError('No projection physics')):
            task = public_review(source)
        self.assertEqual(task['mechanism']['feedback']['symbol_count'],5)
        self.assertEqual(task['completion']['stable_duration_sim_s'],.25)
        self.assertEqual(task['task_revision']['scene_revision'],11)
        self.assertTrue(task['process_requirements']['blue_support_required'])
        self.assertEqual(task['static_workstation']['tabletop_z_m'],.8)
        self.assertEqual(task['static_workstation']['geometry_reference'],'table-geometry.json')
        geometries = static_geometry(source)['table_collision_geometries']
        self.assertGreater(len(geometries),10)
        self.assertTrue(any(g['shape']=='mesh' for g in geometries))
        for geom in geometries:
            if 'mesh' in geom:
                mesh = geom['mesh']
                self.assertLess(max(max(f) for f in mesh['triangular_faces']),len(mesh['vertices_local_m']))
        self.assertNotIn('objects',task)
        self.assertNotIn('initial_center_m',task['dials'][0])
        self.assertFalse(task['process_requirements']['nominal_robot_groups_are_exclusive_assignments'])
        self.assertEqual(task['execution_readiness'],'environment_preview_only')

    def test_private_field_and_engineering_path_checks_are_recursive(self):
        for bad in ({'nested':[{'code_symbols':[1,2,3]}]}, {'reference':'runs/oracle/secret'},
                    {'dials':[{'initial_center_m':[0,0,1]}]}):
            with self.assertRaises(ValueError):assert_public(bad)
        assert_public({'arms':{'A':{'base_position_m':[1,0,.8]}}})

    def test_review_build_has_no_runtime_and_rejects_stale_registry(self):
        registered = read(ROOT/REGISTRY)
        with tempfile.TemporaryDirectory() as td:
            folder = Path(td)/'review'
            with (patch('mujoco.mj_step',side_effect=AssertionError('No physics')),
                  patch('robofrontier.run_setup.prepare_run',side_effect=AssertionError('No actual run'))):
                build_review(ROOT,folder,registered)
            report = read(folder/'review.json')
            self.assertFalse(report['inference_started'])
            self.assertFalse(report['formal_policy_ready'])
            self.assertEqual(len(list(folder.glob('dial-*/public/task.json'))),9)
            self.assertFalse(list(folder.glob('**/runtime.sqlite')))
            self.assertFalse(list(folder.glob('**/connections')))
            self.assertFalse(list(folder.glob('**/suite-code.json')))
            self.assertEqual(verify_review(ROOT,folder)['cases_verified'],9)
            task_path = folder/registered['cases'][0]['case_id']/'public/task.json'
            task = read(task_path)
            task['completion']['stable_duration_sim_s'] = .5
            task_path.write_text(json.dumps(task))
            with self.assertRaisesRegex(ValueError,'Public task'):
                verify_review(ROOT,folder)
            bad = deepcopy(registered)
            bad['cases'][0]['source_sha256']['scene.xml'] = 'stale'
            with self.assertRaisesRegex(ValueError,'no longer matches'):
                build_review(ROOT,Path(td)/'stale',bad)


if __name__ == '__main__':unittest.main()
