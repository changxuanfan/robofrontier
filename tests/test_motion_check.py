"""Headless kinematics only: no World construction, physics step, renderer or GUI."""
import copy
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from robofrontier.motion_check import RobotGeometry, prepare_check, robot_state
from robofrontier.pipeline import Arm, World
from robofrontier.task_config import ROOT, load_harness


def robot_fixture(separation=.65,arm_count=2,base_offset=0.):
    root = ET.parse(ROOT/'vendor/panda/panda.xml').getroot()
    root.find('compiler').set('meshdir',str(ROOT/'vendor/panda/assets'))
    root.remove(root.find('keyframe'))
    world = root.find('worldbody')
    original = copy.deepcopy(world.find('body')); world.clear()
    sections = {tag:list(copy.deepcopy(root.find(tag))) for tag in ('tendon','equality','actuator','contact')}
    for tag in sections:root.find(tag).clear()
    named = {e.get('name') for e in original.iter() if e.get('name')}
    named.update(e.get('name') for children in sections.values() for c in children for e in c.iter() if e.get('name'))
    def rename(element,prefix):
        element=copy.deepcopy(element)
        for e in element.iter():
            for k,v in list(e.attrib.items()):
                if (k=='name' or k in ('joint','joint1','joint2','tendon','body1','body2')) and v in named:
                    e.set(k,prefix+v)
        return element
    stations={chr(65+i):base_offset+i*separation for i in range(arm_count)}
    for name,y in stations.items():
        body=rename(original,name+'_');body.set('pos',f'0 {y} .4')
        ET.SubElement(body.find(f".//body[@name='{name}_hand']"),'site',name=name+'_tcp',pos='0 0 .1029')
        world.append(body)
        for tag,children in sections.items():
            for child in children:root.find(tag).append(rename(child,name+'_'))
    # A dynamic non-robot canary makes accidental whole-MjData copying visible.
    obj=ET.SubElement(world,'body',name='private_object',pos='91 92 93')
    ET.SubElement(obj,'freejoint',name='private_object_free')
    ET.SubElement(obj,'geom',type='sphere',size='.03',mass='.1')
    model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    w=World.__new__(World);w.model=model;w.data=mujoco.MjData(model)
    w.arms={n:Arm(model,w.data,n,y) for n,y in stations.items()}
    mujoco.mj_kinematics(model,w.data)
    return w


class MotionCheckTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world=robot_fixture()

    def test_description_is_robot_only_and_has_whole_link_geometry(self):
        result=RobotGeometry(self.world.model,self.world.arms).describe(self.world,load_harness())
        self.assertEqual(set(result['robots']),{'A','B'})
        self.assertGreater(len(result['robots']['A']['collision_geoms']),10)
        self.assertEqual(len(result['robots']['A']['ik_joint_limits_rad']),7)
        self.assertNotIn('private_object',json.dumps(result))
        self.assertNotIn('initial_position',json.dumps(result))

    def test_tcp_is_fingertip_pad_center_for_different_gripper_widths(self):
        w=robot_fixture()
        for width in (.01,.035):
            for name,arm in w.arms.items():
                pads=[]
                for finger in ('left_finger','right_finger'):
                    body=w.model.body(name+'_'+finger).id
                    joint=w.model.body_jntadr[body]
                    w.data.qpos[w.model.jnt_qposadr[joint]]=width
                    matches=[i for i in range(w.model.ngeom) if w.model.geom_bodyid[i]==body
                             and np.allclose(w.model.geom_pos[i],[0,.0055,.0445])]
                    self.assertEqual(len(matches),1)
                    pads.append(matches[0])
                mujoco.mj_kinematics(w.model,w.data)
                np.testing.assert_allclose(w.data.site_xpos[arm.site],
                                          np.mean(w.data.geom_xpos[pads],axis=0),atol=1e-12)

    def test_ready_facts_follow_model_layout_and_actual_command_bounds(self):
        w=robot_fixture(arm_count=5,base_offset=-1.3)
        cfg=load_harness();cfg.update(workspace_x_m=[.28,.60],workspace_y_radius_m=.31,workspace_z_m=[.45,.85])
        facts=w.robot_facts(cfg)
        self.assertEqual(set(facts['robots']),set('ABCDE'))
        for i,(name,arm) in enumerate(w.arms.items()):
            y=-1.3+i*.65;robot=facts['robots'][name]
            np.testing.assert_allclose(robot['base_pose_world']['position_m'],[0,y,.4])
            np.testing.assert_allclose(robot['base_pose_world']['quaternion_wxyz'],[1,0,0,0])
            np.testing.assert_allclose(robot['tcp_command_bounds_world_m']['lower'],[.28,y-.31,.45])
            np.testing.assert_allclose(robot['tcp_command_bounds_world_m']['upper'],[.60,y+.31,.85])
            self.assertEqual(robot['arm_dof'],len(arm.qids))
            self.assertEqual(robot['grippers'],1)
            # The actual validator admits the current pose and rejects points
            # just beyond every published bound, before attempting any IK.
            target=w.data.site_xpos[arm.site].copy()
            np.testing.assert_allclose(w.validate_action(name,'move',{'target_m':target.tolist()},cfg)['goal'],target)
            for bound,direction in (('lower',-1),('upper',1)):
                for axis in range(3):
                    outside=target.copy();outside[axis]=robot['tcp_command_bounds_world_m'][bound][axis]+direction*.001
                    with self.assertRaisesRegex(ValueError,'TCP workspace'):
                        w.validate_action(name,'move',{'target_m':outside.tolist()},cfg)

    def test_ready_facts_need_no_live_state_or_task_objects(self):
        w=self.world;baseline=w.robot_facts(load_harness())
        shadow=copy.copy(w);shadow.data=object();shadow.task=object()
        shadow.arms={n:copy.copy(a) for n,a in w.arms.items()}
        for arm in shadow.arms.values():arm.data=object()
        self.assertEqual(shadow.robot_facts(load_harness()),baseline)
        self.assertNotIn('private_object',json.dumps(baseline))

    def test_nominal_parallel_motion_is_read_only_and_ignores_object_truth(self):
        w=self.world;cfg=load_harness()
        params={'motions':[{'actor':a,'delta_m':[.01,0,0]} for a in w.arms],'horizon_s':.4}
        q=w.data.qpos.copy();ctrl=w.data.ctrl.copy();before=robot_state(w)
        # If any physics integration is accidentally added, fail this offline test.
        with patch('mujoco.mj_step',side_effect=AssertionError('No physics')):
            result=[r for r in prepare_check(w,params,cfg) if r is not None][-1]
        self.assertEqual(result['assessment'],'no_conflict_found')
        self.assertTrue(result['advisory_only'])
        self.assertNotIn('execution_permission',result)
        self.assertNotIn('integral',result['source_robot']['A'])
        self.assertNotIn('qref',result['source_robot']['A'])
        self.assertEqual(result['samples_checked'],5)
        self.assertEqual(before,robot_state(w))
        np.testing.assert_array_equal(q,w.data.qpos);np.testing.assert_array_equal(ctrl,w.data.ctrl)
        w.data.joint('private_object_free').qpos[:3]=[81,82,83]
        try:
            again=[r for r in prepare_check(w,params,cfg) if r is not None][-1]
            self.assertEqual(result,again)
        finally:w.data.qpos[:]=q

    def test_whole_arm_overlap_is_detected_without_tcp_heuristic(self):
        w=robot_fixture(separation=0)
        w.arms['B']=Arm(w.model,w.data,'B',.3)
        mujoco.mj_kinematics(w.model,w.data)
        self.assertGreater(np.linalg.norm(w.data.site_xpos[w.arms['A'].site]-w.data.site_xpos[w.arms['B'].site]),.29)
        result=next(prepare_check(w,{'motions':[{'actor':'A','delta_m':[.01,0,0]}],'horizon_s':.2},load_harness()))
        self.assertEqual(result['assessment'],'conflict')
        self.assertLess(result['closest']['distance_m'],0)
        self.assertEqual(result['closest']['time_after_snapshot_s'],0)

    def test_delay_and_partial_horizon_are_explicit(self):
        result=[r for r in prepare_check(self.world,{'motions':[{'actor':'A','delta_m':[.1,0,0],'start_after_s':.11}],'horizon_s':.3},load_harness()) if r is not None][-1]
        self.assertEqual(result['nominal_start_after_s']['A'],.12)
        self.assertIn('A',result['unfinished_at_horizon'])
        self.assertIn('between-sample motion',result['not_checked'])

    def test_invalid_requests_and_busy_arm_cannot_create_a_future_queue(self):
        p={'motions':[{'actor':'A','delta_m':[.01,0,0]}],'horizon_s':.4}
        for change in ({'horizon_s':float('nan')},{'sample_dt_s':.03},{'horizon_s':16},
                       {'motions':[{'actor':'missing','delta_m':[0,0,0]}]},
                       {'motions':p['motions']*2}):
            with self.subTest(change=change),self.assertRaises((ValueError,KeyError)):
                prepare_check(self.world,{**p,**change},load_harness())
        self.world.arms['A'].job='busy'
        try:
            with self.assertRaisesRegex(ValueError,'execution'):
                prepare_check(self.world,p,load_harness())
        finally:self.world.arms['A'].job=None

    def test_changing_gripper_returns_unknown(self):
        a=self.world.arms['B'];a.job='grip';a.job_op='gripper'
        try:
            result=next(prepare_check(self.world,{'motions':[{'actor':'A','delta_m':[.01,0,0]}],'horizon_s':.4},load_harness()))
            self.assertEqual(result['assessment'],'unknown')
        finally:a.job=None

    def test_non_finite_geometry_cannot_report_no_conflict(self):
        w=self.world;d=mujoco.MjData(w.model)
        mujoco.mj_kinematics(w.model,d)
        geometry=RobotGeometry(w.model,w.arms)
        d.geom_xpos[geometry.geoms['A'][0],0]=float('nan')
        with self.assertRaisesRegex(ValueError,'Non-finite'):
            geometry.distance(d,.1)

    def test_signed_distance_distinguishes_geometry_not_motion_direction(self):
        m=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <body name="A_link0"><freejoint name="a"/><geom type="sphere" size=".1" mass="1"/></body>
          <body name="B_link0"><freejoint name="b"/><geom type="sphere" size=".1" mass="1"/></body>
        </worldbody></mujoco>''')
        d=mujoco.MjData(m);geometry=RobotGeometry(m,{'A':None,'B':None})
        # Same direction can be separated or catch up; opposite directions can
        # occupy different lanes. Distances checked against analytic fixtures.
        paths=[(lambda t:([.05*t,0],[.6+.05*t,0]),6,.4),
               (lambda t:([.1*t,0],[.33+.025*t,0] if t<=2 else [.38+.26*(t-2),0]),4,-.02),
               (lambda t:([.1*t,0],[.6-.1*t,.5]),6,.3),
               (lambda t:([.1*max(0,t-.7),0],[.33+.025*t,0] if t<=2 else [.38+.26*min(t-2,2),0]),4.7,.05)]
        for path,duration,expected in paths:
            minimum=10.
            for t in np.linspace(0,duration,471):
                a,b=path(t)
                d.joint('a').qpos[:3]=[*a,0];d.joint('b').qpos[:3]=[*b,0]
                mujoco.mj_kinematics(m,d)
                minimum=min(minimum,geometry.distance(d,1)['distance_m'])
            self.assertAlmostEqual(minimum,expected,delta=.001)


if __name__=='__main__':unittest.main()
