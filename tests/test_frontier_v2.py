"""Environment geometry/export contracts, not manipulation success claims."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from robofrontier.robot_control import PandaArm, matrix_quat, quat_matrix
from robofrontier.suite import v2
from robofrontier.suite.scenes import catalog, case_directory
from robofrontier.workstation import Workstation


def load(case_id,revision=None):
    data=v2.cases() if revision is None else json.loads((v2.ROOT/f'configs/suites/frontier-v2-r{revision}.json').read_text())['cases']
    case=next(c for c in data if c['id']==case_id)
    folder=v2.directory(case)
    model=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'))
    data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id)
    mujoco.mj_forward(model,data)
    return folder,model,data


class FrontierV2Tests(unittest.TestCase):
    def test_transport_r11_preserves_manipulation_and_removes_staging_contract(self):
        for case in [c for c in v2.cases() if c['family']=='transport']:
            with self.subTest(case=case['id']):
                folder,m,d=load(case['id']);old_folder,old,od=load(case['id'],revision=10)
                old_task=json.loads((old_folder/'task.json').read_text())
                task=json.loads((folder/'task.json').read_text());env=json.loads((folder/'environment.json').read_text())
                self.assertEqual(case['scene_revision'],13)
                self.assertEqual(env['edges'],[])
                self.assertFalse(any((m.body(i).name or '').startswith('transfer_') for i in range(m.nbody)))
                np.testing.assert_array_equal(d.qpos,od.qpos)
                np.testing.assert_array_equal(m.actuator_gainprm,old.actuator_gainprm)
                np.testing.assert_array_equal(m.actuator_forcerange,old.actuator_forcerange)
                np.testing.assert_array_equal(m.jnt_stiffness,old.jnt_stiffness)
                np.testing.assert_array_equal(m.dof_damping,old.dof_damping)
                self.assertEqual(task['goals'],env['goals'])
                for obj in task['objects']:
                    np.testing.assert_array_equal(d.body(obj['id']).xpos,od.body(obj['id']).xpos)
                    self.assertEqual(float(m.body(obj['id']).mass[0]),float(old.body(obj['id']).mass[0]))
                for goal in task['goals']:
                    cue=goal['target_cue'];obj=next(o for o in task['objects'] if o['identity']==goal['requires']['identity'])
                    self.assertEqual((cue['color'],cue['shape']),(obj['color'],obj['shape']))
                    self.assertAlmostEqual(cue['span_m'],.084)
                    before=next(g for g in old_task['goals'] if g['goal_id']==goal['goal_id'])
                    expected=list(before['local_position_m']);expected[1]*=1.2
                    np.testing.assert_allclose(goal['local_position_m'],expected)
                self.assertAlmostEqual(task['mechanism']['target_center_pitch_m'],.120)
                self.assertAlmostEqual(task['mechanism']['target_cue_gap_m'],.036)
                np.testing.assert_allclose(task['mechanism']['board_size_m'],[.143,.396])
                public=v2.public_task(task)
                self.assertIn('no marked or mandatory intermediate regions',' '.join(public['rules']))
                self.assertNotIn('initial_position_m',json.dumps(public))

    def test_transport_r11_paint_leaves_real_holes_and_boards_separate(self):
        for case in [c for c in v2.cases() if c['family']=='transport']:
            folder,m,d=load(case['id']);task=json.loads((folder/'task.json').read_text());env=json.loads((folder/'environment.json').read_text())
            rectangles=[]
            for robot in task['arms']:
                board=d.body('board_'+robot);r=board.xmat.reshape(3,3)
                half=abs(r[:2,:2])@np.array([.0715,.198]);rectangles.append((board.xpos[:2]-half,board.xpos[:2]+half))
                for x in (-.0715,0,.0715):
                    for y in (-.198,0,.198):
                        p=board.xpos+r@np.array([x,y,0])
                        self.assertTrue(any(np.all(abs(p[:2]-np.array(t['center_m'][:2]))<=np.array(t['half_size_m'][:2])+1e-9) for t in env['tables']))
            for i,(lo,hi) in enumerate(rectangles):
                for a,b in rectangles[i+1:]:self.assertFalse(np.all(np.minimum(hi,b)-np.maximum(lo,a)>1e-9),case['id'])
            if case['level']==1:continue
            # A visual ray must pass through each real socket, rather than
            # hit a noncolliding colored cap hiding the opening.
            m.geom_group[:]=5
            for g in range(m.ngeom):
                if '_cue_' in m.geom(g).name:
                    m.geom_group[g]=0;self.assertEqual(m.geom_contype[g],0);self.assertEqual(m.geom_conaffinity[g],0)
            mask=np.array([1,0,0,0,0,0],dtype=np.uint8)
            for goal in task['goals']:
                board=d.body(goal['board']);r=board.xmat.reshape(3,3);y=goal['local_position_m'][1]
                for x in (-.012,.012):
                    p=board.xpos+r@np.array([x,y,.01])
                    self.assertLess(mujoco.mj_ray(m,d,p,-r[:,2],mask,True,-1,None),0)
                p=board.xpos+r@np.array([.04,y,.01])
                self.assertGreater(mujoco.mj_ray(m,d,p,-r[:,2],mask,True,-1,None),0)

    def test_catalog_inventory_and_public_contract(self):
        cases=v2.cases()
        self.assertEqual(len(cases),36)
        self.assertEqual(sum(c['arms'] for c in cases),324)
        for case in cases:
            with self.subTest(case=case['id']):
                task=json.loads((v2.directory(case)/'task.json').read_text())
                v2.validate_task(task)
                public=v2.public_task(task)
                self.assertEqual(task['execution_readiness'],'environment_preview_only')
                self.assertFalse(task['oracle_verified'])
                self.assertNotIn('objects',public)
                self.assertNotIn('scene_asset',public)
                self.assertNotIn('physics',public)
                self.assertNotIn('initial_position_m',json.dumps(public))
                if case['family']=='mosaic':
                    pattern=task['grid']['pattern']
                    source=json.loads(v2.TULIP_FILE.read_text())
                    names=['cream','navy','coral','sage']
                    raw=source['variants'][f'{case["rows"]}x{case["columns"]}']['grid']
                    expected=np.rot90(np.asarray([[names[c] for c in row] for row in raw]),-1).tolist()
                    self.assertEqual(pattern,expected)
                    counts=Counter(c for row in pattern for c in row)
                    self.assertEqual(set(counts),{'cream','navy','coral','sage'})
                    self.assertEqual(case['colors'],4)
                    self.assertEqual(task['grid']['pitch_m'],.081)
                    self.assertEqual(task['grid']['cube_side_m'],.06)
                    if case['level']==1:self.assertEqual(task['grid']['pocket_opening_m'],.080)

    def test_wide_profile_matches_real_pad_gap_without_extra_joint_travel(self):
        _,m,d=load('transport-a04-d1',revision=1)
        arm=PandaArm(m,d,'A')
        hand=m.body('A_hand').id
        left=m.geom('A_left_finger_pad').id
        right=m.geom('A_right_finger_pad').id
        for q in (0.,.03,.04):
            for suffix in ('finger_joint1','finger_joint2'):
                j=m.joint('A_'+suffix)
                np.testing.assert_allclose(m.jnt_range[j.id],[0,.04])
                d.joint(j.name).qpos[0]=q
            mujoco.mj_forward(m,d)
            local=[d.xmat[hand].reshape(3,3).T@(d.geom_xpos[g]-d.xpos[hand]) for g in (left,right)]
            face_gap=abs(local[0][1]-local[1][1])-2*.0025
            self.assertAlmostEqual(face_gap,.04+2*q,places=9)
            self.assertAlmostEqual(arm.observe()['gripper_width_m'],face_gap,places=9)
            self.assertAlmostEqual(arm.grip_to_width(arm.width_to_grip(face_gap)),face_gap,places=9)
        self.assertAlmostEqual(arm.grip_max,.12)

    def test_cube_fits_upper_grasp_and_empty_closed_hand_is_not_a_grasp(self):
        _,m,d=load('transport-a04-d1')
        arm=PandaArm(m,d,'A')
        for suffix in ('finger_joint1','finger_joint2'):d.joint('A_'+suffix).qpos[0]=.035
        mujoco.mj_forward(m,d)
        hand=d.body('A_hand');rotation=hand.xmat.reshape(3,3).copy()
        cube=d.joint('A_item1_free')
        cube.qpos[:3]=hand.xpos+rotation@np.array([0,0,.1149])
        cube.qpos[3:]=matrix_quat(rotation)
        mujoco.mj_forward(m,d)
        cube_geom=m.geom('A_item1_geom').id
        for contact in d.contact:
            if cube_geom not in contact.geom:continue
            other=next(int(g) for g in contact.geom if g!=cube_geom)
            b=int(m.geom_bodyid[other]);parents=[]
            while b:parents.append(m.body(b).name);b=int(m.body_parentid[b])
            if 'A_hand' in parents:self.assertGreaterEqual(contact.dist,-.0001)
        for suffix in ('finger_joint1','finger_joint2'):d.joint('A_'+suffix).qpos[0]=0
        arm.grasp_force=20.;arm.grasp_confirmed=True;arm.grasp_width=.001;arm.grasp_epsilon=(.01,.01)
        self.assertFalse(arm.observe()['is_grasped'])
        _,standard,sd=load('relay-a08-d1')
        self.assertAlmostEqual(PandaArm(standard,sd,'A').grip_max,.08)

    def test_reported_width_capability_follows_installed_hardware(self):
        from robofrontier.pipeline import World
        from robofrontier.run_setup import load_harness
        from types import SimpleNamespace
        for case_id,limits in [('transport-a04-d1',[0,.08]),('relay-a08-d1',[0,.08])]:
            _,m,d=load(case_id)
            world=World.__new__(World)
            world.model=m;world.data=object()
            world.arms={name:SimpleNamespace(qids=range(7),grip_min=limits[0],grip_max=limits[1]) for name in ['A']}
            world.command_bounds={'lower':[-10,-10,0],'upper':[10,10,2]}
            facts=world.robot_facts(load_harness())
            self.assertEqual(facts['shared_capabilities']['gripper']['width_m'],limits)
            self.assertEqual(facts['robots']['A']['gripper_width_range_m'],limits)

    def test_fixed_and_spring_blind_slots_fit_actual_blades(self):
        for level in (2,3):
            folder,m,d=load(f'transport-a04-d{level}')
            task=json.loads((folder/'task.json').read_text())
            board=d.body('board_A');rotation=board.xmat.reshape(3,3).copy()
            goal=next(g for g in task['goals'] if g['goal_id']=='A_slot1')
            joint=d.joint('A_item1_free')
            joint.qpos[:3]=board.xpos+rotation@np.array(goal['local_position_m'])
            joint.qpos[3:]=matrix_quat(rotation)
            mujoco.mj_forward(m,d)
            for contact in d.contact:
                names=[m.geom(int(g)).name for g in contact.geom]
                if any(n.startswith('A_item1_') for n in names) and any(n.startswith(('board_A','A_slot')) for n in names):
                    self.assertGreaterEqual(contact.dist,-.0001,msg=str(names))
            if level==3:
                self.assertEqual(task['mechanism']['kind'],'spring_board_two_flat_blades')
                self.assertNotIn('latch',task['mechanism'])
                self.assertGreater(m.joint('board_A_z').id,-1)
            # The blade midpoint is genuinely inside the board, not above
            # an invisible solid slab or a merely painted opening.
            for i in range(2):
                relative=rotation.T@(d.geom('A_item1_blade'+str(i)).xpos-board.xpos)
                self.assertAlmostEqual(relative[2],-.012,places=8)

    def test_passive_cable_attachment_and_replay_registry(self):
        for level in (2,3):
            folder,m,d=load(f'relay-a16-d{level}',revision=7)
            manifest=json.loads((folder/'workstation.json').read_text())
            recorded=set(manifest['motion_recording_contract']['dynamic_bodies'])
            for j in range(m.njnt):
                b=int(m.jnt_bodyid[j])
                self.assertIn(m.body(b).name,recorded)
            for eq in range(m.neq):
                if 'attachment' not in m.equality(eq).name:continue
                a,b=int(m.eq_obj1id[eq]),int(m.eq_obj2id[eq])
                np.testing.assert_allclose(d.site_xpos[a],d.site_xpos[b],atol=1e-9)
            for g in range(m.ngeom):
                if '_cable' in m.geom(g).name:
                    count=json.loads((folder/'task.json').read_text())['mechanism']['cable_segments_each']
                    self.assertAlmostEqual(m.geom_size[g,1],.20/count/2)

    def test_grid_and_unmarked_have_identical_physics(self):
        for scale in ('a04-6x12','a06-9x18','a08-12x24'):
            physics=[]
            for level in (2,3):
                c=next(c for c in v2.cases() if c['id']==f'mosaic-{scale}-d{level}')
                physics.append(json.loads((v2.directory(c)/'compiled-physics.json').read_text()))
            self.assertEqual(physics[0],physics[1])

    def test_sixty_mm_objects_have_recomputed_volume_mass_and_blades(self):
        folder,m,d=load('transport-a08-d3')
        task=json.loads((folder/'task.json').read_text())
        for obj in task['objects']:
            g=m.geom(obj['id']+'_geom');body=m.body(obj['id'])
            if obj['source'] in ('G','H'):
                self.assertEqual(obj['shape'],'cylinder')
                np.testing.assert_allclose(g.size[:2],[.03,.03])
                expected=781.25*np.pi*.03**2*.06
            else:
                self.assertEqual(obj['shape'],'cube')
                np.testing.assert_allclose(g.size,[.03]*3)
                expected=.16875
            blade_mass=2*.0025*.012*.024*7800
            self.assertAlmostEqual(body.mass[0],expected+blade_mass)
        self.assertEqual(task['mechanism']['slot_depth_m'],.026)
        self.assertEqual(task['mechanism']['blade_size_m'],[.0025,.012,.024])

    def test_spring_and_fixed_boards_use_identical_inline_socket_geometry(self):
        models=[load('transport-a04-d'+str(level))[1] for level in (2,3)]
        for name in [models[0].geom(g).name for g in range(models[0].ngeom) if models[0].geom(g).name.startswith('board_A_')]:
            ga,gb=[m.geom(name) for m in models]
            np.testing.assert_allclose(ga.pos,gb.pos)
            np.testing.assert_allclose(ga.size,gb.size)
        self.assertEqual(mujoco.mj_name2id(models[0],mujoco.mjtObj.mjOBJ_JOINT,'board_A_z'),-1)
        self.assertGreaterEqual(mujoco.mj_name2id(models[1],mujoco.mjtObj.mjOBJ_JOINT,'board_A_z'),0)

    def test_relay_scale_arc_start_and_sloped_tray(self):
        folder,m,d=load('relay-a16-d3',revision=7)
        task=json.loads((folder/'task.json').read_text());mech=task['mechanism']
        self.assertEqual(mech['outer_arms'],10);self.assertEqual(mech['inner_arms'],6)
        radii=[np.linalg.norm(a['base_position_m'][:2]) for a in task['arms'].values()]
        self.assertEqual(sum(abs(r-2.43)<1e-9 for r in radii),10)
        self.assertEqual(sum(abs(r-1.03)<1e-9 for r in radii),6)
        self.assertEqual(mech['interior_angle_deg'],135)
        self.assertAlmostEqual(mech['chord_m'],1.2)
        self.assertLess(d.body('payload_01').xpos[1],-.37)
        self.assertEqual(mech['tray']['outer_diameter_m'],.10)
        self.assertEqual(mech['tray']['depth_m'],mech['tray']['weight_height_m']/5)
        mid=int(m.geom('payload_01_tray_rim_0').dataid[0]);start=int(m.mesh_vertadr[mid]);count=int(m.mesh_vertnum[mid])
        vertices=m.mesh_vert[start:start+count]
        self.assertGreater(float(np.ptp(vertices[:,2])),.005)

    def test_fixed_dials_have_solid_tables_and_opposed_rows(self):
        for n in (4,6,8):
            folder,m,d=load(f'dial-n{n:02d}-d1')
            task=json.loads((folder/'task.json').read_text())
            self.assertTrue(task['mechanism']['layout'].startswith('rectangular_'))
            self.assertEqual(task['mechanism']['table_inner_radius_m'],0)
            self.assertGreater(mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'table_0'),-1)
            if n in (4,6):
                self.assertEqual(len({round(a['initial_center_m'][1],4) for a in task['dials']}),2)
                self.assertEqual(sum(a['initial_center_m'][1]>0 for a in task['dials']),n//2)
        for level in (2,3):
            folder,m,d=load(f'dial-n06-d{level}')
            task=json.loads((folder/'task.json').read_text())
            positions=np.asarray([a['initial_center_m'][:2] for a in task['dials']])
            np.testing.assert_allclose(np.linalg.norm(positions,axis=1),1.73)
            self.assertTrue(np.all(positions[:,1]>=-1e-9))
            self.assertTrue(np.all(np.diff(positions[:,0])>0))
            self.assertEqual(task['mechanism']['layout'],'upper_semicircle')

    def test_compact_transport_and_aligned_mosaic_layouts(self):
        for n in (4,8,12):
            folder,m,d=load(f'transport-a{n:02d}-d1')
            env=json.loads((folder/'environment.json').read_text())
            self.assertEqual(len(env['tables']),1 if n==4 else 4)
        for c in v2.cases():
            if c['family']!='mosaic' or c['level']!=1:continue
            folder,m,d=load(c['id']);env=json.loads((folder/'environment.json').read_text())
            half_width=c['columns']*.081/2
            xs=[a['base_position_m'][0] for a in env['stations']]
            expected_half_span=.95*(half_width-.085)
            self.assertAlmostEqual(max(xs),expected_half_span)
            self.assertAlmostEqual(min(xs),-expected_half_span)
            self.assertEqual(env['mechanism']['row_spacing_scale_from_r3'],.95)
            self.assertEqual(env['mechanism']['table_side_scale_from_r3'],1.25)
            for obj in env['objects']:
                x,y,_=obj['initial_position_m']
                self.assertTrue(abs(x)>=half_width+.085-1e-9 or abs(y)>=c['rows']*.081/2+.085-1e-9)

    def test_exact_requested_size_changes_and_upper_arc_order(self):
        old_cases=json.loads((v2.ROOT/'configs/suites/frontier-v2-r3.json').read_text())['cases']
        def table_bounds(env):
            lo=np.min([np.asarray(t['center_m'][:2])-t['half_size_m'][:2] for t in env['tables']],axis=0)
            hi=np.max([np.asarray(t['center_m'][:2])+t['half_size_m'][:2] for t in env['tables']],axis=0)
            return hi-lo
        for case in v2.cases():
            if case['family'] not in ('transport','mosaic') or case['level']!=1:continue
            old=next(c for c in old_cases if c['id']==case['id'])
            before=json.loads((v2.ROOT/old['directory']/'environment.json').read_text())
            after=json.loads((v2.directory(case)/'environment.json').read_text())
            factor=.85 if case['family']=='transport' else 1.25
            expected=table_bounds(before)*factor
            if case['family']=='mosaic' and case['arms'] in (4,8):expected[1]*=.90
            np.testing.assert_allclose(table_bounds(after),expected)
            if case['family']=='mosaic':
                np.testing.assert_allclose([s['base_position_m'][0] for s in after['stations']],np.array([s['base_position_m'][0] for s in before['stations']])*.95)
        folder,m,d=load('relay-a12-d3');task=json.loads((folder/'task.json').read_text())
        centers=np.asarray([r['center_m'][:2] for r in task['route']])
        self.assertTrue(np.all(np.diff(centers[:,0])>0))
        self.assertTrue(np.all(centers[1:-1,1]>0))
        self.assertEqual(task['mechanism']['layout'],'upper_arc_90_degrees')

    def test_mosaic_collision_probe_is_read_only_and_rejects_self_contact(self):
        from robofrontier.suite.geometry_checks import owner_map,clear_robot_pose
        folder,m,d=load('mosaic-a04-6x12-d1');arm=PandaArm(m,d,'C')
        initial=d.qpos.copy();gains=m.actuator_gainprm.copy();scratch=mujoco.MjData(m);owners=owner_map(m,'ABCD')
        self.assertTrue(clear_robot_pose(m,initial,scratch,'C',arm.qids,arm.qref,owners))
        target=np.array([-.5425,.83125,.502])
        # This archived R4 stock approach had a valid IK solution but folded
        # link5 into link1. It must not count as an accessible supply pose.
        y=np.array([-1.,0,0]);z=np.array([0.,0,-1.]);rotation=np.column_stack((np.cross(y,z),y,z))
        q,err=arm.ik.solve(target+[0,0,.06],rotation,arm.qref.copy(),140)
        q,err=arm.ik.solve(target,rotation,q,140)
        self.assertLess(err,.0025)
        self.assertFalse(clear_robot_pose(m,initial,scratch,'C',arm.qids,q,owners))
        np.testing.assert_array_equal(d.qpos,initial)
        np.testing.assert_array_equal(m.actuator_gainprm,gains)

    def test_multi_payload_closures_and_declared_inventory(self):
        for n,count in [(8,2),(12,3),(16,4)]:
            for level in (1,2,3):
                folder,m,d=load(f'relay-a{n:02d}-d{level}')
                task=json.loads((folder/'task.json').read_text())
                self.assertEqual(len(task['objects']),count)
                self.assertEqual(task['mechanism']['payload_count'],count)
                ids={o['id'] for o in task['objects']}
                self.assertEqual(len(ids),count)
                self.assertEqual(len({o['start_region_id'] for o in task['objects']}),count)
                for assembly in task['mechanism']['payload_assemblies']:
                    self.assertIn(assembly['beam_body'],ids)
                    for key in ('beam_body','weight_body','tray_body'):
                        if assembly[key]:self.assertGreaterEqual(mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,assembly[key]),0)
                for eq in range(m.neq):
                    if 'attachment' in m.equality(eq).name:
                        np.testing.assert_allclose(d.site_xpos[m.eq_obj1id[eq]],d.site_xpos[m.eq_obj2id[eq]],atol=1e-9)
        for c in v2.cases():
            if c['family']!='transport':continue
            task=__import__('json').loads((v2.directory(c)/'task.json').read_text())
            for obj in task['objects']:
                index=ord(obj['source'])-65
                self.assertEqual(obj['color'],list(v2.PALETTE)[index%6])
                self.assertEqual(obj['shape'],'cube' if index<6 else 'cylinder')
            self.assertEqual(Counter(o['source'] for o in task['objects']),{chr(65+i):3 for i in range(c['arms'])})

    def test_petal_symmetry_and_single_sided_round_crank(self):
        source=json.loads(v2.TULIP_FILE.read_text())
        for row in source['variants']['9x18']['grid']:
            self.assertEqual([x==2 for x in row],[x==2 for x in row[::-1]])
        original=json.loads((v2.ROOT/'configs/patterns/tulip_parameters.json').read_text())
        for key in ('6x12','12x24'):self.assertEqual(source['variants'][key]['grid'],original['variants'][key]['grid'])
        for n in (4,6,8):
            folder,m,d=load(f'dial-n{n:02d}-d3')
            task=json.loads((folder/'task.json').read_text())
            self.assertFalse(task['mechanism']['rotor_counterbalanced'])
            self.assertEqual(task['mechanism']['detent_labels'],['0','1','2','3'])
            self.assertFalse(any('crank_balance' in m.geom(g).name for g in range(m.ngeom)))
            self.assertEqual(m.geom('D01_crank_arm').type[0],mujoco.mjtGeom.mjGEOM_CAPSULE)

    def test_enlarged_disks_and_tolerant_plugs_have_matching_physics(self):
        for level in (2,3):
            folder,m,d=load(f'transport-a04-d{level}')
            task=json.loads((folder/'task.json').read_text());mech=task['mechanism']
            self.assertGreater(mech['slot_opening_m'][0],mech['blade_size_m'][0])
            self.assertGreater(mech['slot_opening_m'][1],mech['blade_size_m'][1])
            self.assertGreater(mech['slot_depth_m'],mech['blade_size_m'][2])
            self.assertEqual(mech['board_thickness_m'],.032)
        for level in (1,2,3):
            folder,m,d=load(f'dial-n04-d{level}')
            self.assertAlmostEqual(float(m.geom('D01_module_disk').size[0]),.150)
            task=json.loads((folder/'task.json').read_text())
            self.assertAlmostEqual(task['mechanism']['digit_height_m'],.040)
            self.assertGreaterEqual(task['mechanism']['digit_outer_rim_clearance_m'],.0079)
            if level==3:
                self.assertFalse(any('arrowhead' in m.geom(g).name or 'orange_pointer' in m.geom(g).name for g in range(m.ngeom)))
                self.assertAlmostEqual(float(m.geom('D01_crank_arm').size[0]),.010)
                self.assertAlmostEqual(float(m.geom('D01_hand_grip').size[0]),.01375)
                task=json.loads((folder/'task.json').read_text())
                self.assertAlmostEqual(task['mechanism']['crank_arm_length_m'],.083)
            else:
                self.assertEqual(m.geom('D01_pointer_tip').type[0],mujoco.mjtGeom.mjGEOM_MESH)

    def test_portable_copy_and_pose_export_never_steps_physics(self):
        for case_id in ('transport-a04-d3','mosaic-a04-6x12-d1','relay-a16-d3','dial-n04-d3'):
            case=next(c for c in v2.cases() if c['id']==case_id)
            with tempfile.TemporaryDirectory() as td:
                dest=Path(td)/'workstation';shutil.copytree(v2.directory(case),dest)
                source=Workstation(dest)
                q=source.initial_qpos.copy()
                q[source.model.joint('A_joint1').qposadr[0]]+=.02
                for j in range(source.model.njnt):
                    if int(source.model.jnt_type[j])==mujoco.mjtJoint.mjJNT_FREE:
                        q[source.model.jnt_qposadr[j]]+=.015
                        break
                with patch('mujoco.mj_step',side_effect=AssertionError('No replay dynamics')):
                    original=source.frame(q,sim_time_s=1.25)
                    placed=source.frame(q,sim_time_s=1.25,translation_m=[3,4,2],yaw_rad=np.pi/2)
                np.testing.assert_array_equal(source.data.qpos,q)
                self.assertEqual(placed['simulation_time_s'],1.25)
                rotation=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
                for kind in ('bodies','geometries'):
                    self.assertEqual(set(original[kind]),set(placed[kind]))
                    for name,pose in original[kind].items():
                        target=placed[kind][name]
                        np.testing.assert_allclose(target['position_m'],rotation@pose['position_m']+[3,4,2],atol=1e-10)
                        np.testing.assert_allclose(quat_matrix(target['quaternion_wxyz']),rotation@quat_matrix(pose['quaternion_wxyz']),atol=1e-10)

    def test_legacy_scene_checksums_still_match(self):
        from robofrontier.suite.tasks import make_task
        for case in catalog()['cases']:
            folder=case_directory(case);task=make_task(folder)
            self.assertEqual(hashlib.sha256((folder/'scene.xml').read_bytes()).hexdigest(),task['scene_sha256'])


if __name__=='__main__':unittest.main()
