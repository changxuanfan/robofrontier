from copy import deepcopy
import json
from pathlib import Path
import shutil
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image

from robofrontier.suite.mosaic_policy_package import (
    ROOT,REGISTRY,build,check,checksum,project_public,read,verify_registry,write)
from robofrontier.suite.evaluate import Monitor
from robofrontier.suite.evaluate_v2 import mosaic_candidates


class MosaicCompletionContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.spec=read(ROOT/REGISTRY)['completion']

    def candidate(self,level=2,position=None,angle=0,floor_gap=.0001):
        c,s=np.cos(angle),np.sin(angle);rotation=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
        task=dict(level=level,cube_size_m=.06,grid=dict(pocket_opening_m=.08,floor_z_m=.78),
                  goals=[dict(goal_id='r1c1',requires={'color':'cream'},position_m=[0.,0.,.81])])
        model=SimpleNamespace(geom=lambda name:SimpleNamespace(id=0 if name=='pocket_bottom' else 1))
        mon=SimpleNamespace(task=task,m=model,d=SimpleNamespace(xmat=np.array([rotation.ravel()])),
            names=['cube'],objects=[dict(id='cube',color='cream')],bodies=[0],
            contacts=lambda:(np.array([[0,1]]),np.array([floor_gap])))
        with patch('mujoco.mj_geomDistance',return_value=floor_gap):
            result=mosaic_candidates(mon,np.array([position if position is not None else [0.,0.,.81]]),[True])
        return bool(result[0,0])

    def test_flat_position_tolerances_match_actual_evaluator(self):
        xy=self.spec['flat']['xy_center_tolerance_m_inclusive'];z=self.spec['flat']['z_center_tolerance_m_inclusive']
        self.assertTrue(self.candidate(position=[xy,0,.81]))
        self.assertFalse(self.candidate(position=[xy+1e-6,0,.81]))
        self.assertTrue(self.candidate(position=[0,0,.81+z-1e-9]))
        self.assertFalse(self.candidate(position=[0,0,.81+z+1e-6]))

    def test_pocket_accepts_full_footprint_instead_of_flat_center_tolerance(self):
        margin=self.spec['recessed']['footprint_containment_margin_m_inclusive']
        limit=.08/2+margin-.06/2
        self.assertTrue(self.candidate(1,[limit-1e-9,0,.81]))
        self.assertFalse(self.candidate(1,[limit+1e-6,0,.81]))
        self.assertFalse(self.candidate(1,angle=np.pi/4))
        floor=self.spec['recessed']['bottom_height_tolerance_m_inclusive']
        self.assertTrue(self.candidate(1,[0,0,.81+floor-1e-9]))
        self.assertFalse(self.candidate(1,[0,0,.81+floor+1e-6]))

    def test_pocket_needs_actual_floor_contact_with_declared_margin(self):
        gap=self.spec['recessed']['floor_contact_distance_m_inclusive']
        self.assertTrue(self.candidate(1,floor_gap=gap))
        self.assertFalse(self.candidate(1,floor_gap=gap+1e-6))

    def test_orientation_tolerance_and_cube_symmetry_match_evaluator(self):
        angle=np.deg2rad(self.spec['axis_alignment_tolerance_deg_inclusive'])
        self.assertTrue(self.candidate(angle=angle-1e-8))
        self.assertFalse(self.candidate(angle=angle+1e-6))
        self.assertTrue(self.candidate(angle=np.pi/2))

    def monitor(self,linear=0.,angular=0.,contact_distance=None):
        task=dict(family='mosaic',suite_version=2,level=2,cube_size_m=.06,
                  goals=[dict(goal_id='r1c1',requires={'color':'cream'},position_m=[0.,0.,.81])],
                  completion={'stable_duration_sim_s':self.spec['stable_duration_sim_s']})
        d=SimpleNamespace(time=0.,xmat=np.array([np.eye(3).ravel()]),
                          body=lambda name:SimpleNamespace(xpos=np.array([0.,0.,.81])))
        pairs=np.empty((0,2),int) if contact_distance is None else np.array([[1,2]])
        dist=np.array([]) if contact_distance is None else np.array([contact_distance])
        return SimpleNamespace(task=task,m=None,d=d,family='mosaic',names=['cube'],
            objects=[dict(id='cube',color='cream')],bodies=[0],
            velocity=lambda body:np.array([angular,0,0,linear,0,0]),contacts=lambda:(pairs,dist),
            geom_object=np.array([-1,0,-1]),geom_owner=np.array([-1,-1,0]),
            since=np.full((1,1),np.nan),violations=[])

    def complete(self,mon):
        Monitor.sample_goals(mon);mon.d.time=self.spec['stable_duration_sim_s'];Monitor.sample_goals(mon)
        return mon.result_cache['task_success']

    def test_distinct_assignment_and_process_compliance_remain_required(self):
        mon=self.monitor();mon.task['goals'].append(dict(goal_id='r1c2',requires={'color':'cream'},position_m=[0.,0.,.81]));mon.since=np.full((2,1),np.nan)
        self.assertFalse(self.complete(mon));self.assertEqual(mon.result_cache['matched_goals'],1)
        mon=self.monitor();mon.violations=[{'kind':'inter_robot_collision'}]
        self.assertFalse(self.complete(mon));self.assertTrue(mon.result_cache['all_goals_met'])

    def test_terminal_policy_scoring_reports_collision_separately(self):
        mon=self.monitor();mon.task['run_protocol']={'evaluation_semantics':'terminal_outcome_v2'}
        mon.violations=[{'kind':'inter_robot_contact'}]
        self.assertTrue(self.complete(mon))
        self.assertTrue(mon.result_cache['terminal_goal_met'])
        self.assertFalse(mon.result_cache['strict_task_success'])

    def test_stability_dwell_speed_and_release_match_monitor(self):
        mon=self.monitor();Monitor.sample_goals(mon)
        mon.d.time=self.spec['stable_duration_sim_s']-1e-5;Monitor.sample_goals(mon)
        self.assertFalse(mon.result_cache['task_success'])
        mon.d.time=self.spec['stable_duration_sim_s'];Monitor.sample_goals(mon)
        self.assertTrue(mon.result_cache['task_success'])
        linear=self.spec['stable_linear_speed_m_s_exclusive'];angular=self.spec['stable_angular_speed_rad_s_exclusive']
        self.assertFalse(self.complete(self.monitor(linear=linear)))
        self.assertFalse(self.complete(self.monitor(angular=angular)))
        self.assertTrue(self.complete(self.monitor(linear=linear-1e-6,angular=angular-1e-6)))
        gap=self.spec['robot_contact_release_distance_m_inclusive']
        self.assertFalse(self.complete(self.monitor(contact_distance=gap)))
        self.assertTrue(self.complete(self.monitor(contact_distance=gap+1e-6)))


@unittest.skipUnless((ROOT/'runs/oracle/mosaic-continuation-20261026/evidence-summary.json').exists(),
                     'Local finalized Oracle fixtures not present')
class MosaicPolicyPackageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory();cls.base=Path(cls.temp.name)
        cls.package=cls.base/'baseline'
        with patch('mujoco.mj_step',side_effect=AssertionError('No package physics')):
            build(cls.package)
        cls.registry=read(ROOT/REGISTRY)
        cls.first=cls.registry['cases'][0]

    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()

    def copy(self):
        directory=Path(tempfile.mkdtemp(dir=self.base));target=directory/'package'
        shutil.copytree(self.package,target);return target

    def test_nine_explicit_r13_cases_and_raw_sources_unchanged(self):
        result=check(self.package);self.assertTrue(result['passed']);self.assertFalse(result['policy_launch_ready'])
        goals=0
        for case in self.registry['cases']:
            task=read(self.package/'public/cases'/case['case_id']/'task.json')
            self.assertEqual(task['scene_revision'],13);self.assertEqual(task['execution_readiness'],'environment_preview_only')
            self.assertNotIn('Visible dial count',task['observation_contract']['policy'])
            self.assertEqual(task['static_geometry']['table_height_m'],.8)
            self.assertEqual(checksum(ROOT/case['environment']/'task.json'),case['source_files']['task.json'])
            goals+=task['target']['rows']*task['target']['columns']
        self.assertEqual(goals,1566)
        self.assertFalse(any(self.package.rglob('runtime.sqlite')))

    def test_projection_ignores_private_truth_and_repairs_even_when_input_contains_them(self):
        folder=ROOT/self.first['environment'];task=read(folder/'task.json');env=read(folder/'environment.json');ws=read(folder/'workstation.json');published=read(folder/'public-task.json')
        task['oracle']={'repair_recipe':'CANARY-PRIVATE-RECIPE'};published['objects']=[{'qpos':'CANARY-LIVE-TRUTH'}]
        task['mechanism']['repair_recipe']='CANARY-SCENE-RECIPE'
        projected=project_public(task,env,ws,published,self.registry['completion']);text=json.dumps(projected)
        self.assertNotIn('CANARY',text);self.assertNotIn('layout_revision',text)
        self.assertEqual(projected['initial_supplies'],published['initial_supplies'])

    def test_even_an_empty_public_symlink_is_rejected(self):
        target=self.copy();private=self.base/'empty-private';private.mkdir(exist_ok=True)
        (target/'public/private-link').symlink_to(private,target_is_directory=True)
        with self.assertRaisesRegex(ValueError,'Public symlink'):check(target)

    def test_public_tampering_and_extra_files_are_rejected(self):
        target=self.copy();case=target/'public/cases'/self.first['case_id']/'task.json'
        payload=read(case);payload['static_geometry']['table_height_m']=.4;write(case,payload)
        with self.assertRaisesRegex(ValueError,'Pinned source differs'):check(target)
        target=self.copy();(target/'public/accidental-private.json').write_text('{"qpos":[0]}')
        with self.assertRaisesRegex(ValueError,'Unexpected or missing'):check(target)

    def test_even_resealed_public_pose_leaks_are_rejected(self):
        target=self.copy();relative=f'cases/{self.first["case_id"]}/task.json';case=target/'public'/relative
        payload=read(case);payload['objects']=[dict(initial_position_m=[0,0,0])];write(case,payload)
        manifest=read(target/'engineering/package-manifest.json');manifest['public_files'][relative]=checksum(case);write(target/'engineering/package-manifest.json',manifest)
        with self.assertRaisesRegex(ValueError,'Private field'):check(target)

    def test_resealed_mirrored_reference_image_is_rejected(self):
        target=self.copy();relative=f'cases/{self.first["case_id"]}/target-pattern.png';path=target/'public'/relative
        with Image.open(path) as image:image.transpose(Image.Transpose.FLIP_LEFT_RIGHT).save(path)
        manifest=read(target/'engineering/package-manifest.json');manifest['public_files'][relative]=checksum(path);write(target/'engineering/package-manifest.json',manifest)
        with self.assertRaisesRegex(ValueError,'orientation/colors'):check(target)

    def test_wrong_source_or_control_pins_are_rejected(self):
        value=deepcopy(self.registry);value['cases'][0]['source_files']['scene.xml']='0'*64
        path=self.base/'wrong-registry.json';write(path,value)
        with self.assertRaisesRegex(ValueError,'Pinned source differs'):verify_registry(registry_path=path)
        value=deepcopy(self.registry);value['cases'][0]['oracle']['physical_control_sha256']='0'*64;write(path,value)
        with self.assertRaisesRegex(ValueError,'controller differs'):verify_registry(registry_path=path)

    def test_review_package_cannot_claim_formal_policy_activation(self):
        target=self.copy();path=target/'engineering/package-manifest.json';value=read(path);value['policy_launch_ready']=True;write(path,value)
        with self.assertRaisesRegex(ValueError,'cannot activate'):check(target)


if __name__=='__main__':unittest.main()
