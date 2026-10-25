"""Mosaic variants preserve the old task and use the selected grid geometry."""
from collections import Counter
import json
from collections import deque
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from robofrontier.mosaic import ROOT, add_recessed_fixture, runtime_task, scene_directory, specification
from robofrontier.scene_task import sample_shared, evaluate_shared


class MosaicTests(unittest.TestCase):
    def test_default_specification_preserves_existing_ten_by_ten_task(self):
        env,draft=specification()
        existing=json.loads((ROOT/'configs/tasks/mosaic-four-arm-v1.json').read_text())
        task=runtime_task(env,draft,existing['scene_sha256'])
        for field in ('id','objects','shared_goals','grid','scene_asset','completion'):
            self.assertEqual(task[field],existing[field],field)

    def test_eight_by_eight_inventory_and_goal_geometry(self):
        env,draft=specification(8)
        counts={'blue':24,'yellow':26,'white':8,'red':6}
        self.assertEqual(Counter(o['color'] for o in env['objects']),counts)
        self.assertEqual(Counter(g['color'] for g in draft['goal_cells']),counts)
        self.assertEqual(len(draft['goal_cells']),64)
        self.assertEqual(draft['goal_cells'][0]['cube_center_world_m'],[-.14,.14,.42])
        self.assertEqual(draft['goal_cells'][-1]['cube_center_world_m'],[.14,-.14,.42])
        task=runtime_task(env,draft,'scene-hash')
        self.assertEqual(task['scene_asset'],str(scene_directory(8).relative_to(ROOT)))
        self.assertNotEqual(task['scene_asset'],str(scene_directory(10).relative_to(ROOT)))
        self.assertEqual(len({g['goal_id'] for g in task['shared_goals']}),64)
        self.assertEqual(task['completion'],runtime_task(*specification(10),'scene-hash')['completion'])

    def test_unsupported_size_is_rejected(self):
        with self.assertRaises(ValueError):specification(9)

    def test_seven_by_seven_rocket_has_independent_geometry_and_explicit_objective(self):
        env,draft=specification(7)
        counts={'white':18,'red':15,'blue':11,'yellow':5}
        self.assertEqual(Counter(o['color'] for o in env['objects']),counts)
        self.assertEqual(Counter(g['color'] for g in draft['goal_cells']),counts)
        self.assertEqual(len(draft['goal_cells']),49)
        self.assertEqual(draft['goal_cells'][0]['cube_center_world_m'],[-.12,.12,.42])
        self.assertEqual(draft['goal_cells'][-1]['cube_center_world_m'],[.12,-.12,.42])
        task=runtime_task(env,draft,'scene-hash')
        self.assertEqual(task['execution_objective'],draft['execution_objective'])
        self.assertEqual(task['scene_asset'],str(scene_directory(7).relative_to(ROOT)))
        self.assertEqual(task['completion'],runtime_task(*specification(10),'scene-hash')['completion'])

    def test_recessed_variant_keeps_stock_and_pattern_but_changes_real_goal_geometry(self):
        flat,old=specification(7);env,draft=specification(7,recessed=True)
        self.assertEqual(env['objects'],flat['objects'])
        self.assertEqual(draft['matrix_codes'],old['matrix_codes'])
        grid=env['grid'];fixture=grid['fixture']
        self.assertAlmostEqual(fixture['depth_m'],grid['cube_side_m']/3)
        self.assertAlmostEqual(grid['cell_side_m'],fixture['opening_side_m']+fixture['wall_thickness_m'])
        self.assertGreater(fixture['opening_side_m'],grid['cube_side_m'])
        for g in draft['goal_cells']:
            self.assertAlmostEqual(g['cube_center_world_m'][2],fixture['floor_z_m']+grid['cube_side_m']/2)
        task=runtime_task(env,draft,'scene-hash')
        self.assertEqual(task['scene_asset'],str(scene_directory(7,True).relative_to(ROOT)))
        self.assertNotEqual(task['id'],runtime_task(flat,old,'scene-hash')['id'])

    def test_six_by_six_reference_has_36_cubes_and_50mm_openings(self):
        env,draft=specification(6,True)
        self.assertEqual(draft['matrix_codes'],['WWRRWW','WRRRRW','WBRRBW','BBYYBB','BRYYRB','RYYYYR'])
        self.assertEqual(Counter(o['color'] for o in env['objects']),{'red':12,'blue':8,'yellow':8,'white':8})
        self.assertEqual(len(draft['goal_cells']),36)
        self.assertEqual(env['grid']['fixture']['opening_side_m'],.05)
        self.assertAlmostEqual(env['grid']['cell_side_m'],.051)
        self.assertEqual(draft['goal_cells'][0]['cube_center_world_m'][:2],[-.1275,.1275])
        self.assertEqual(draft['goal_cells'][-1]['cube_center_world_m'][:2],[.1275,-.1275])
        task=runtime_task(env,draft,'scene-hash')
        self.assertEqual(task['scene_asset'],str(scene_directory(6,True).relative_to(ROOT)))
        self.assertNotEqual(task['scene_asset'],str(scene_directory(7,True).relative_to(ROOT)))


class RecessedContactTests(unittest.TestCase):
    """Isolated engineering fixture; no grasp recipe or policy observations."""
    def world(self, on_wall=False, offset=0):
        env,draft=specification(7,True);task=runtime_task(env,draft,'test')
        goals=[g for g in task['shared_goals'] if g['goal_id'] in ('r04c04','r04c05')]
        if on_wall:goals=goals[:1]
        root=ET.Element('mujoco');ET.SubElement(root,'option',timestep='.002',integrator='implicitfast',noslip_iterations='10')
        world=ET.SubElement(root,'worldbody')
        ET.SubElement(world,'geom',name='table_0',type='box',pos='0 0 .375',size='.8 .8 .025',friction='1 .005 .0001',condim='4')
        add_recessed_fixture(world,env)
        objects=[]
        for i,g in enumerate(goals):
            name=f'cube_{i}';position=list(g['position_m']);position[2]=.46
            if i==0:position[0]+=offset
            if on_wall:position[:2]=[env['grid']['cell_side_m']/2]*2
            body=ET.SubElement(world,'body',name=name,pos=' '.join(map(str,position)))
            ET.SubElement(body,'freejoint');ET.SubElement(body,'geom',name=name+'_geom',type='box',size='.02 .02 .02',mass='.05',friction='1 .005 .0001',condim='4')
            objects.append({'id':name,**g['requires']})
        task.update(objects=objects,shared_goals=goals)
        model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));data=mujoco.MjData(model)
        return SimpleNamespace(model=model,data=data,task=task,object_names=set(o['id'] for o in objects),robot_bodies=set(),evaluation_samples=deque(maxlen=100),time=0.)

    def settle(self,w):
        for i in range(1100):
            mujoco.mj_step(w.model,w.data)
            if i%10==0:
                mujoco.mj_forward(w.model,w.data);w.time=w.data.time;sample_shared(w)

    def test_dropped_cubes_seat_below_rim_and_lateral_impulse_does_not_displace_neighbor(self):
        w=self.world();self.settle(w)
        self.assertTrue(evaluate_shared(w)['all_goals_met'])
        p=w.data.body('cube_1').xpos.copy()
        joint=w.model.body('cube_0').jntadr[0];dof=w.model.jnt_dofadr[joint]
        w.data.qvel[dof]=.2
        self.settle(w)
        clearance=(w.task['grid']['fixture']['opening_side_m']-w.task['grid']['cube_side_m'])/2
        offset=w.data.body('cube_0').xpos-np.asarray(w.task['shared_goals'][0]['position_m'])
        self.assertLess(max(abs(offset[:2])),clearance+.0002)
        self.assertLess(abs(offset[2]),.0002)
        self.assertLess(np.linalg.norm(w.data.body('cube_1').xpos-p),.0002)
        self.assertEqual(w.model.neq,0)

    def test_cube_balanced_on_divider_is_not_a_seated_goal(self):
        w=self.world(on_wall=True);self.settle(w)
        self.assertGreater(w.data.body('cube_0').xpos[2],w.task['shared_goals'][0]['position_m'][2]+.007)
        self.assertFalse(evaluate_shared(w)['all_goals_met'])

    def test_4mm_offset_drop_still_fits_the_wider_pocket_without_snap(self):
        w=self.world(offset=.004);self.settle(w)
        offset=w.data.body('cube_0').xpos-np.asarray(w.task['shared_goals'][0]['position_m'])
        self.assertAlmostEqual(offset[0],.004,delta=.0002)
        self.assertLess(abs(offset[2]),.0002)

    def test_pocket_metric_accepts_seating_and_retains_strict_diagnostic(self):
        w=self.world(offset=.004);self.settle(w)
        w.task['completion']['mode']='pocket_seating'
        w.task['completion']['containment_tolerance_m']=.0002
        w.task['completion']['upright_tolerance_rad']=.035
        result=evaluate_shared(w)
        self.assertTrue(result['all_goals_met'])
        self.assertEqual(result['seated_goals'],2)
        self.assertFalse(result['strict_pose_diagnostic']['complete'])

    def test_pocket_metric_checks_full_footprint_release_and_floor(self):
        import copy
        w=self.world();self.settle(w)
        w.task['completion'].update(mode='pocket_seating',containment_tolerance_m=.0002,upright_tolerance_rad=.035)
        saved=copy.deepcopy(w.evaluation_samples)
        for key,value in [('released',False),('pocket_floor_supported',False),
                          ('position_m',[.006,0,.4066666667]),
                          ('angular_velocity_rad_s',[0,0,1]),
                          ('quaternion_wxyz',[np.cos(.3),0,0,np.sin(.3)])]:
            w.evaluation_samples=copy.deepcopy(saved)
            for sample in w.evaluation_samples:sample['objects']['cube_0'][key]=value
            self.assertFalse(evaluate_shared(w)['all_goals_met'],key)

    def test_pocket_metric_accepts_fitting_yaw_but_not_wrong_color_or_short_window(self):
        import copy
        w=self.world();joint=w.model.body('cube_0').jntadr[0];q=w.model.jnt_qposadr[joint]
        w.data.qpos[q+3:q+7]=[np.cos(.05),0,0,np.sin(.05)]
        self.settle(w)
        result=evaluate_shared(w)
        self.assertTrue(result['all_goals_met'])
        self.assertFalse(result['strict_pose_diagnostic']['complete'])
        saved=copy.deepcopy(w.evaluation_samples)
        w.evaluation_samples=deque([saved[-1]])
        self.assertFalse(evaluate_shared(w)['all_goals_met'])
        w.evaluation_samples=saved;w.task['objects'][0]['color']='wrong'
        self.assertFalse(evaluate_shared(w)['all_goals_met'])


if __name__=='__main__':unittest.main()
