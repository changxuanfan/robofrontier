"""Regression: unspecified source PBR must not become glossy plastic."""
import unittest
import json
import numpy as np
from scripts.factory_blender_materials import resolve, robot_role, task_role


class MaterialChecks(unittest.TestCase):
    def test_replayed_numpy_colors_are_json_safe(self):
        source=dict(rgba=np.array([.12,.34,.56,0.],dtype=np.float32),roughness=np.float32(.58),metallic=np.float32(0.))
        for role in (None,'joint','shell','painted_wood'):
            result=resolve(source,role)
            self.assertEqual(json.loads(json.dumps(result))['rgba'][3],0.)
        np.testing.assert_array_equal(source['rgba'],np.array([.12,.34,.56,0.],dtype=np.float32))

    def test_unspecified_pbr_and_source_immutability(self):
        source = dict(rgba=[.25, .25, .25, 1], roughness=-1., metallic=-1.)
        result = resolve(source, robot_role(source))
        self.assertGreaterEqual(result['roughness'], .4)
        self.assertEqual(result['metallic'], 0.)
        self.assertLess(max(result['rgba'][:3]), .05)
        self.assertEqual(source['rgba'], [.25, .25, .25, 1])
        self.assertEqual(source['roughness'], -1.)

    def test_explicit_and_colored_materials_are_preserved(self):
        source = dict(rgba=[0., 1., 0., 1.], roughness=.7, metallic=.2)
        result = resolve(source, robot_role(source))
        self.assertEqual(result, source)

    def test_shell_is_separate_from_joint_and_palette_can_be_disabled(self):
        source = dict(rgba=[.90, .92, .93, 1.], roughness=-1., metallic=-1.)
        self.assertEqual(robot_role(source), 'shell')
        self.assertLess(max(resolve(source, 'shell')['rgba'][:3]), .8)
        self.assertEqual(resolve(source, 'shell', False)['rgba'], source['rgba'])

    def test_task_wood_and_print_do_not_recolor_or_reveal_hidden_geoms(self):
        source=dict(rgba=[.84,.1,.08,0.],roughness=-1.,metallic=-1.)
        for name in ('relay-a08-d1__payload_01_beam','mosaic-a04-6x12-d1__A_stock01_geom'):
            self.assertEqual(task_role(dict(name=name)), 'painted_wood')
            result=resolve(source,task_role(dict(name=name)))
            self.assertEqual(result['rgba'],source['rgba'])
            self.assertGreater(result['roughness'],.5)
        self.assertIsNone(task_role(dict(name='transport-a04-d1__A_item1_blade0')))
