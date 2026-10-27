import hashlib,json,tempfile,unittest
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch
import mujoco,numpy as np
from robofrontier.suite.transport_policy_package import (assert_variant_parity,table_geometry,public_input,verify_oracle,build_package,load_json,materialize_scene_assets)

ROOT=Path(__file__).resolve().parents[1]

class AssetMaterializationTests(unittest.TestCase):
    def scene_pair(self,root):
        source=root/'frozen';target=root/'r14';(source/'assets').mkdir(parents=True);target.mkdir()
        data=b'original asset';(source/'assets/mesh.obj').write_bytes(data)
        scene='<mujoco/>';(target/'scene.xml').write_text(scene)
        manifest=dict(scene_sha256=hashlib.sha256(scene.encode()).hexdigest(),assets=[dict(path='assets/mesh.obj',sha256=hashlib.sha256(data).hexdigest(),bytes=len(data))])
        for folder in (source,target):(folder/'workstation.json').write_text(json.dumps(manifest))
        return source,target
    def test_missing_asset_is_independent_and_repeated_restore_is_read_only(self):
        with tempfile.TemporaryDirectory()as td:
            source,target=self.scene_pair(Path(td));self.assertEqual(materialize_scene_assets(source,target),1)
            self.assertFalse((source/'assets/mesh.obj').samefile(target/'assets/mesh.obj'))
            self.assertEqual(materialize_scene_assets(source,target),0)
            (target/'assets/mesh.obj').write_bytes(b'editable copy')
            self.assertEqual((source/'assets/mesh.obj').read_bytes(),b'original asset')
            with self.assertRaisesRegex(ValueError,'Existing R14 asset checksum'):materialize_scene_assets(source,target)
    def test_modified_source_is_rejected_before_creating_assets(self):
        with tempfile.TemporaryDirectory()as td:
            source,target=self.scene_pair(Path(td));(source/'assets/mesh.obj').write_bytes(b'wrong source')
            with self.assertRaisesRegex(ValueError,'Source asset checksum'):materialize_scene_assets(source,target)
            self.assertFalse((target/'assets').exists())

class TableGeometryTests(unittest.TestCase):
    def test_hole_is_not_reported_as_support_inside_bounding_box(self):
        model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
        <geom name="table_0" type="box" pos="0 -1.5 .775" size="2 .5 .025"/>
        <geom name="table_1" type="box" pos="0 1.5 .775" size="2 .5 .025"/>
        <geom name="table_2" type="box" pos="-1.5 0 .775" size=".5 1 .025"/>
        <geom name="table_3" type="box" pos="1.5 0 .775" size=".5 1 .025"/>
        </worldbody></mujoco>''')
        result=table_geometry(model);self.assertEqual(result['central_opening_size_m'],[2.,2.]);self.assertEqual(len(result['boxes_world']),4);self.assertEqual(result['outer_size_xy_m'],[4.,4.])
    def test_static_solid_table_remains_solid(self):
        model=mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom name="table_0" type="box" size="1 1 .025"/></worldbody></mujoco>')
        self.assertIsNone(table_geometry(model)['central_opening_size_m'])
    def test_robot_mount_change_is_rejected(self):
        xml='<mujoco><worldbody><body name="base" pos="1 0 0"><geom name="robot" type="box" size=".1 .1 .1"/></body></worldbody></mujoco>'
        a=mujoco.MjModel.from_xml_string(xml);b=mujoco.MjModel.from_xml_string(xml);b.body_pos[1,0]+=.01
        with self.assertRaisesRegex(ValueError,'body_pos'):assert_variant_parity(a,b,[])

class ActualFamilyPreflightTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config=ROOT/'configs/task_families/transport/half-opening-r14.json';cls.cfg=load_json(cls.config)
        first=ROOT/cls.cfg['cases'][0]['physical_scene']
        if not(first/'assets').exists():raise unittest.SkipTest('Materialize the generated Transport family scenes first')
    def test_all_nine_physical_revisions_have_half_holes_and_preview_gate(self):
        from robofrontier.task_config import load_task
        for row in self.cfg['cases']:
            with self.subTest(case=row['case_id']):
                folder=ROOT/row['physical_scene'];task=load_task(folder/'task.json');model=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'));geometry=table_geometry(model)
                self.assertEqual(task['scale']['scene_revision'],14);self.assertEqual(task['execution_readiness'],'environment_preview_only');self.assertFalse(task['oracle_verified'])
                arms=len(task['arms']);expected=None if arms==4 else [.476,.476]if arms==8 else[.833,.833]
                if expected is None:self.assertIsNone(geometry['central_opening_size_m'])
                else:np.testing.assert_allclose(geometry['central_opening_size_m'],expected,rtol=0,atol=1e-12)
    def test_public_projection_does_not_export_injected_private_facts(self):
        row=self.cfg['cases'][0];folder=ROOT/row['physical_scene'];task=load_json(folder/'task.json')
        task['private_answer']='DO_NOT_EXPORT';task['mechanism']['private_qpos_recipe']='DO_NOT_EXPORT'
        model=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'));public=public_input(task,model,self.cfg['acceptance'])
        encoded=json.dumps(public);self.assertNotIn('DO_NOT_EXPORT',encoded);self.assertNotIn('initial_position_m',encoded);self.assertNotIn('oracle_trial',encoded)
        self.assertIn('table_geometry',public);self.assertEqual(public['completion']['stable_duration_sim_s'],1.)
        self.assertEqual(len(public['static_camera_mounts']),model.ncam)
        for camera in public['static_camera_mounts']:
            cid=model.camera(camera['id']).id;body=int(model.cam_bodyid[cid])
            self.assertEqual(camera['frame'],'world'if body==0 else 'body:'+model.body(body).name)
        self.assertEqual(len(public['orders']),12)
    def test_modified_witness_is_rejected_without_modifying_original(self):
        row=deepcopy(self.cfg['cases'][0]);row['report_sha256']='0'*64
        with self.assertRaisesRegex(ValueError,'report checksum'):verify_oracle(row,ROOT)
    def test_failed_preflight_does_not_create_public_package(self):
        with tempfile.TemporaryDirectory()as td:
            cfg=deepcopy(self.cfg);cfg['cases'][0]['scene_sha256']='0'*64;path=Path(td)/'config.json';path.write_text(json.dumps(cfg));output=Path(td)/'package'
            with patch('mujoco.mj_step',side_effect=AssertionError('No physics')):
                with self.assertRaisesRegex(ValueError,'witness reference'):build_package(path,output,ROOT)
            self.assertFalse(output.exists())
