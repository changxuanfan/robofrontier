import unittest
from robofrontier.suite.provenance import environment_logic_manifest


class EnvironmentLogicFingerprintTests(unittest.TestCase):
    def test_v2_geometry_changes_are_not_hidden_by_unchanged_shared_monitor(self):
        shared='robofrontier/suite/evaluate.py';geometry='robofrontier/suite/evaluate_v2.py'
        before=environment_logic_manifest({shared:'same',geometry:'before'})
        after=environment_logic_manifest({shared:'same',geometry:'after'})
        self.assertNotEqual(before['sha256'],after['sha256'])
        self.assertEqual(before['sources'][shared],after['sources'][shared])
        self.assertEqual(before['schema'],'robofrontier.environment-logic.v2')
        self.assertEqual(before,environment_logic_manifest({geometry:'before',shared:'same','unrelated.py':'ignored'}))


if __name__=='__main__':unittest.main()
