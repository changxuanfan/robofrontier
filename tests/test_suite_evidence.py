"""Audit regressions: evidence identity, delivery ordering and motion meaning."""
import json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from robofrontier.suite.provenance import bundle_manifest
from scripts.verify_frontier_suite import record_result
from robofrontier.task_config import load_harness,physical_control_fingerprint

class SuiteEvidenceTests(unittest.TestCase):
    def test_physics_identity_ignores_policy_architecture_but_not_action_limits(self):
        code={'robofrontier/'+name:'hash-'+name for name in ('robot_control.py','sim.py','motion_check.py','task_config.py','pipeline.py')};cfg=load_harness();first=physical_control_fingerprint(code,cfg)
        changed={**cfg,'topology':'adaptive','model':'another-policy','effort':'high'};self.assertEqual(first,physical_control_fingerprint(code,changed))
        changed['allowed_tcp_speed_m_s']=[.005,.10];self.assertNotEqual(first,physical_control_fingerprint(code,changed))
    def test_asset_and_contract_changes_invalidate_the_bundle(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);(p/'scene.xml').write_text('<mujoco><compiler/><asset><texture file="texture.png"/></asset></mujoco>')
            for name in ('task.json','environment.json'):(p/name).write_text('{}')
            (p/'texture.png').write_bytes(b'first');before=bundle_manifest(p)['sha256']
            (p/'texture.png').write_bytes(b'second');after=bundle_manifest(p)['sha256'];self.assertNotEqual(before,after)
            (p/'task.json').write_text('{"tolerance": 1}');self.assertNotEqual(after,bundle_manifest(p)['sha256'])

    def test_a_late_old_attempt_does_not_replace_the_latest_result_or_erase_a_witness(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'catalog.json';p.write_text(json.dumps({'cases':[{'id':'case','family':'dial'}]}))
            with patch('robofrontier.suite.scenes.CATALOG',p),patch('scripts.verify_frontier_suite.bundle_manifest',return_value={'sha256':'bundle'}),patch('scripts.verify_frontier_suite.case_directory',return_value=Path(tmp)):
                for stamp,passed in ((10,True),(20,False),(5,False)):
                    record_result('case',dict(passed=passed,verification_seed=0,scene_sha256='scene',scene_bundle_sha256='bundle',executor_sha256='executor',finished_at_unix_s=stamp),f'attempt-{stamp}')
            case=json.loads(p.read_text())['cases'][0]
            self.assertEqual(case['last_oracle_report'],'attempt-20');self.assertFalse(case['last_oracle_passed'])
            self.assertEqual(case['best_oracle_witness'],'attempt-10');self.assertEqual(len(case['oracle_attempts']),3)
            self.assertEqual(case['status'],'oracle_witness_passed');self.assertNotEqual(case['status'],'repeatability_checked')

if __name__=='__main__':unittest.main()
