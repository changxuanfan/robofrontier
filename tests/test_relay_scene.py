"""Static scene acceptance: visible boundaries and intact independent assemblies."""
import hashlib
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from robofrontier.suite.v2 import ROOT, cases, directory, rz
from robofrontier.suite.relay_scene import validate, subtract_convex, merge_convex_pieces, polygon_area
from robofrontier.workstation import Workstation


class RelaySceneTests(unittest.TestCase):
    def test_twelve_arm_mounts_span_90_degrees_and_double_inner_ring(self):
        for case in (c for c in cases() if c['family']=='relay' and c['arms']==12):
            station=Workstation(directory(case),verify_assets=False);station.frame()
            task=json.loads((station.path/'task.json').read_text());angles=[]
            for name,arm in task['arms'].items():
                position=station.data.body(name+'_link0').xpos
                np.testing.assert_allclose(position,arm['base_position_m'],atol=1e-12)
                angles.append(np.rad2deg(np.arctan2(position[1],position[0])))
            self.assertAlmostEqual(max(angles)-min(angles),90.)
            self.assertAlmostEqual(min(angles),45.)
            self.assertAlmostEqual(max(angles),135.)
            for index,(name,arm) in enumerate(task['arms'].items()):
                self.assertAlmostEqual(np.linalg.norm(arm['base_position_m'][:2]),3.46 if index<7 else 2.16)
            self.assertEqual(task['mechanism']['outer_arms'],7)
            self.assertEqual(task['mechanism']['inner_arms'],5)
            self.assertEqual(task['mechanism']['outer_robot_ids'],list('ABCDEFG'))
            self.assertEqual(task['mechanism']['inner_robot_ids'],list('HIJKL'))
            self.assertIsNone(task['pairs'])
            self.assertAlmostEqual(task['mechanism']['outer_robot_radius_m']-task['mechanism']['inner_robot_radius_m'],1.30)

    def test_end_table_clipping_preserves_holes_and_merges_flat_seams(self):
        rectangle=lambda x0,y0,x1,y1:[np.array(p,dtype=float) for p in [(x0,y0),(x1,y0),(x1,y1),(x0,y1)]]
        outer=rectangle(-2,-2,2,2);hole=rectangle(-1,-1,1,1)
        pieces=merge_convex_pieces(subtract_convex(outer,hole))
        self.assertAlmostEqual(sum(polygon_area(p) for p in pieces),12.)
        self.assertEqual(subtract_convex(hole,outer),[])
        self.assertEqual(len(merge_convex_pieces([rectangle(-2,-2,2,0),rectangle(-2,0,2,2)])),1)

    def test_only_two_equal_painted_regions_and_sparse_direction_arrows(self):
        for case in (c for c in cases() if c['family']=='relay'):
            with self.subTest(case=case['id']):
                folder=directory(case); task=json.loads((folder/'task.json').read_text())
                world=ET.parse(folder/'scene.xml').find('worldbody')
                self.assertEqual(case['scene_revision'],15 if case['arms']==12 else 13)
                self.assertFalse(any(b.get('name','').startswith(('route_zone_','route_arrow_','pickup_')) for b in world.findall('body')))
                self.assertEqual(len([b for b in world.findall('body') if b.get('name','').startswith('relay_direction_')]),{8:7,12:9,16:11}[case['arms']])
                for name,region in [('start',task['route'][0]),('goal',task['route'][-1])]:
                    self.assertEqual(region['half_size_m'],[.4,.68])
                    frame=world.find(f"body[@name='relay_{name}']")
                    np.testing.assert_allclose(np.fromstring(frame.get('pos'),sep=' ')[:2],region['center_m'][:2])
                    for geom in frame.findall('geom'):
                        self.assertEqual(geom.get('contype'),'0')
                        self.assertEqual(geom.get('conaffinity'),'0')
                self.assertTrue(all(not r['painted'] for r in task['route'][1:-1]))
                mounts=[a['base_position_m'][0] for a in task['arms'].values()]
                if case['arms']==8:
                    self.assertLessEqual(task['route'][0]['center_m'][0]+.4,min(mounts)-.10)
                    self.assertGreaterEqual(task['route'][-1]['center_m'][0]-.4,max(mounts)+.10)
                else:
                    env=json.loads((folder/'environment.json').read_text())
                    self.assertFalse(env['table_geometry']['external_side_tables'])
                    self.assertEqual(env['table_geometry']['angle_degrees'],90 if case['arms']==12 else 180)
                    self.assertEqual(env['table_geometry']['inner_radius_m'],1.6 if case['arms']==12 else .8)
                    self.assertFalse(task['mechanism']['presentation']['end_extensions'])
                self.assertFalse(task['oracle_verified'])

    def test_weight_and_hinge_relative_poses_are_not_broken_by_opening_flip(self):
        for case in (c for c in cases() if c['family']=='relay' and c['level']>1):
            with self.subTest(case=case['id']):
                new=Workstation(directory(case),verify_assets=False)
                old=Workstation(directory(dict(case,scene_revision=10)),verify_assets=False)
                new.frame();old.frame()
                task=json.loads((new.path/'task.json').read_text())
                for assembly in task['mechanism']['payload_assemblies']:
                    ident=assembly['id'];a=old.data.body(ident);b=new.data.body(ident)
                    turn=rz(assembly.get('layout_yaw_change_rad',0.))
                    forward=turn@a.xmat.reshape(3,3)@np.array([1.,0.,0.])
                    opening=b.xmat.reshape(3,3)@np.array([-1.,0.,0.])
                    np.testing.assert_allclose(opening,forward,atol=1e-12)
                    np.testing.assert_allclose(b.xpos,turn@a.xpos+np.asarray(assembly.get('layout_translation_world_m',[0.,0.,0.])),atol=1e-12)
                    for name in [assembly['weight_body']]+assembly['beam_bodies']:
                        before=a.xmat.reshape(3,3).T@(old.data.body(name).xpos-a.xpos)
                        after=b.xmat.reshape(3,3).T@(new.data.body(name).xpos-b.xpos)
                        np.testing.assert_allclose(after,before,atol=1e-12)
                self.assertAlmostEqual(new.model.opt.timestep,.002)

    def test_all_nine_static_checks_never_step_physics(self):
        with patch.object(mujoco,'mj_step',side_effect=AssertionError('No physics steps in preview acceptance')):
            for case in (c for c in cases() if c['family']=='relay'):
                with self.subTest(case=case['id']):
                    result=validate(directory(case),directory(dict(case,scene_revision=10)),persist=False)
                    self.assertEqual(result['physics_steps'],0)
                    self.assertTrue(result['endpoint_regions_fully_supported'])
                    self.assertTrue(result['endpoint_regions_outside_all_robot_mounts'] if case['arms']==8 else result['endpoint_regions_within_integrated_arc_landings'])
                    self.assertEqual(result['initial_robot_payload_or_different_payload_penetrations_over_1mm'],[])

    def test_old_nine_workstations_are_byte_unchanged(self):
        saved=json.loads((ROOT/'docs/generated/frontier-v2/relay-r15/source-preserved-hashes.json').read_text())
        self.assertGreater(len(saved),100)
        for name,checksum in saved.items():
            self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),checksum,name)


if __name__=='__main__':unittest.main()
