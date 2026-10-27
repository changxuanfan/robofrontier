"""R17 changes the clarified release contract, preserving every physics input."""
import hashlib,json,unittest
from robofrontier.suite.v2 import cases,directory

class ReleaseSourceTests(unittest.TestCase):
    def test_six_new_contracts_have_identical_manufactured_physics(self):
        for case in cases():
            if case['family']!='relay' or case['level']==1:continue
            self.assertEqual(case['scene_revision'],17)
            new=directory(case);old=directory(dict(case,scene_revision=16))
            task=json.loads((new/'task.json').read_text());prior=json.loads((old/'task.json').read_text())
            self.assertEqual(task['mechanism'],prior['mechanism']);self.assertEqual(task['arms'],prior['arms']);self.assertEqual(task['route'],prior['route'])
            self.assertTrue(task['completion']['allow_goal_partial_support']);self.assertTrue(task['completion']['allow_near_goal_release']);self.assertEqual(task['completion']['max_release_gap_m'],.03)
            self.assertTrue(task['completion']['required_airborne_handover'])
            revision=json.loads((new/'rule-revision.json').read_text())
            for name in revision['physics_files_unchanged']:
                self.assertEqual(hashlib.sha256((new/name).read_bytes()).digest(),hashlib.sha256((old/name).read_bytes()).digest(),name)

    def test_l1_geometry_revisions_remain_unchanged_with_current_goal_rules(self):
        for case in cases():
            if case['family']=='relay' and case['level']==1:
                self.assertEqual(case['scene_revision'],15 if case['arms']==12 else 13)
                from robofrontier.suite.relay_contract import is_current
                self.assertTrue(is_current(json.loads((directory(case)/'task.json').read_text())))

if __name__=='__main__':unittest.main()
