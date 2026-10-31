"""Check the released candidate's object-specific contact configuration."""
import unittest
import xml.etree.ElementTree as ET

from scripts.benchmark import select_cases
from robofrontier.task_families import ROOT


class RollingPhysicsCandidateTests(unittest.TestCase):
    def test_transport_cylinders_and_dial_modules(self):
        counts = {'transport': 0, 'dial': 0}
        for case in select_cases(families=['transport', 'dial']):
            geoms = ET.parse(ROOT / case['directory'] / 'scene.xml').getroot().iter('geom')
            selected = []
            for geom in geoms:
                name = geom.get('name', '')
                if case['family'] == 'transport':
                    match = '_item' in name and name.endswith('_geom') and geom.get('type') == 'cylinder'
                else:
                    match = name.startswith('D') and name.endswith(('_module_disk', '_module_stem'))
                if match:
                    selected.append(geom)
                    self.assertEqual(geom.get('condim'), '6', (case['case_id'], name))
                    self.assertAlmostEqual(float(geom.get('friction').split()[2]), .03, msg=(case['case_id'], name))
            if selected:
                counts[case['family']] += 1
            if case['family'] == 'dial':
                self.assertTrue(selected, case['case_id'])
        self.assertEqual(counts, {'transport': 6, 'dial': 9})


if __name__ == '__main__':
    unittest.main()
