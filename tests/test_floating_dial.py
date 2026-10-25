"""Free-apparatus contracts: private gating, relative reference and engine timing."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from robofrontier.floating_dial_task import make_task, FloatingState
from robofrontier.spring_dial import DialFeedback
from robofrontier.spring_dial_task import public_spring_task
from robofrontier.task_config import ROOT

ASSET=ROOT/'configs/environments/spring_dial/floating-04'

class FloatingFeedbackTests(unittest.TestCase):
    def test_floating_crank_keeps_private_state_out_of_public_device_contract(self):
        asset=ROOT/'configs/environments/spring_dial/floating-crank-04-060-tight-noslip20'
        task=make_task(asset);public=public_spring_task(task);m=public['mechanism']
        self.assertEqual(m['apparatus'],'floating');self.assertEqual(m['red_interface'],'crank')
        self.assertAlmostEqual(m['crank']['grip_center_radius_m'],.069)
        self.assertNotIn('knob_core_diameter_m',m)
        self.assertNotIn('contact_friction',m['crank'])
        self.assertNotIn('scene_asset',public)
        self.assertTrue(all('center_m' not in d for d in public['dials']))
        model=mujoco.MjModel.from_xml_path(str(asset/'scene.xml'))
        for a in 'ABCD':
            grip=model.geom('dial_'+a+'_hand_grip');self.assertAlmostEqual(grip.size[0],.011)
            self.assertEqual(model.joint('dial_'+a+'_module_free').type,mujoco.mjtJoint.mjJNT_FREE)
            jid=model.joint('dial_'+a+'_hinge').id
            self.assertFalse(any(model.actuator_trnid[:,0]==jid))
        self.assertEqual(model.opt.noslip_iterations,20)

    def test_grounded_configuration_cannot_score_and_requires_new_stability(self):
        game=DialFeedback(dict(angle_tolerance_deg=4,max_speed_deg_s=5,stable_duration_s=.25,max_sample_gap_s=.03),secret=[90]*4)
        angles=np.ones(4)*np.pi/2
        for i in range(51):game.update(i*.01,angles,np.zeros(4),eligible=False)
        self.assertFalse(game.won);self.assertEqual(game.queries,0)
        for i in range(51,75):game.update(i*.01,angles,np.zeros(4),eligible=True)
        self.assertFalse(game.won)
        game.update(.76,angles,np.zeros(4),eligible=True)
        self.assertTrue(game.won);self.assertEqual(game.queries,1)

    def test_public_task_no_initial_module_positions_or_secret(self):
        task=make_task(ASSET);public=public_spring_task(task)
        self.assertEqual(len(public['arms']),8)
        self.assertNotIn('scene_asset',public)
        self.assertTrue(all('center_m' not in d for d in public['dials']))
        self.assertNotIn('code_symbols',public)
        self.assertEqual(public['mechanism']['minimum_airborne_clearance_m'],.02)

    def test_all_four_modes_freeze_same_apparatus_without_code_in_public_inputs(self):
        from robofrontier.run_setup import prepare_run, verify_inputs
        from robofrontier.task_config import load_harness
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);task_path=folder/'task.json';task_path.write_text(json.dumps(make_task(ASSET)))
            for mode in ('centralized','adaptive','hierarchical','peer'):
                cfg=load_harness();cfg['topology']=mode;harness=folder/(mode+'.json');harness.write_text(json.dumps(cfg))
                run=prepare_run(folder/mode,task_path,harness)
                verify_inputs(run)
                public=json.loads((run/'public/inputs/task.json').read_text())
                self.assertEqual(len(public['arms']),8)
                self.assertNotIn('physics',public)
                self.assertNotIn('code_symbols',public)
                self.assertTrue((run/'private/inputs/spring-code.json').is_file())

    def test_strict_apparatus_in_all_modes_keeps_audit_private(self):
        from robofrontier.run_setup import prepare_run,verify_inputs
        from robofrontier.task_config import load_harness
        asset=ROOT/'configs/environments/spring_dial/floating-04-060'
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);task_path=folder/'task.json';task_path.write_text(json.dumps(make_task(asset)))
            for mode in ('centralized','adaptive','hierarchical','peer'):
                cfg=load_harness();cfg['topology']=mode;hp=folder/(mode+'.json');hp.write_text(json.dumps(cfg))
                run=prepare_run(folder/mode,task_path,hp);verify_inputs(run)
                public=json.loads((run/'public/inputs/task.json').read_text())
                self.assertEqual(public['mechanism']['contact_sequence']['kind'],'lift_before_knob_contact')
                self.assertTrue(all('center_m' not in d for d in public['dials']))
                for name in ('module_pose_world','supporting_robots','red_contact_robots','contact_episodes'):
                    self.assertNotIn(name,json.dumps(public))

    def test_free_body_rotation_does_not_change_relative_dial_angle(self):
        model=mujoco.MjModel.from_xml_path(str(ASSET/'scene.xml'));data=mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id)
        ids=[model.jnt_qposadr[model.joint('dial_'+a+'_hinge').id] for a in 'ABCD']
        data.qpos[ids]=np.pi/2
        for a in 'ABCD':
            q=data.joint('dial_'+a+'_module_free').qpos;q[2]+=.35;q[3:]=[1,0,0,0]
        mujoco.mj_forward(model,data)
        world=SimpleNamespace(model=model,data=data,task=make_task(ASSET))
        app=FloatingState(world)
        self.assertTrue(app.sample(data.qpos[ids]))
        self.assertTrue(np.allclose(data.qpos[ids],np.pi/2))
        data.joint('dial_A_module_free').qpos[2]=.42
        mujoco.mj_forward(model,data)
        self.assertFalse(app.sample(data.qpos[ids]))

    def test_trimmed_table_is_symmetric_and_score_is_inside_middle_robot_gap(self):
        asset=ROOT/'configs/environments/spring_dial/floating-04-060-tight';params=json.loads((asset/'parameters.json').read_text());task=make_task(asset)
        model=mujoco.MjModel.from_xml_path(str(asset/'scene.xml'))
        table=model.geom('table_0');score=model.geom('score_base')
        center=table.pos;half=table.size;bases=[a['base_position_m'][0] for a in params['arms']]
        self.assertAlmostEqual(center[0]-(min(bases)),max(bases)-center[0])
        for axis in (0,1):self.assertLessEqual(abs(score.pos[axis]-center[axis])+score.size[axis],half[axis]+1e-9)
        self.assertAlmostEqual((score.pos[1]-score.size[1])-(center[1]-half[1]),.02)
        self.assertAlmostEqual(score.pos[0],(params['arms'][2]['base_position_m'][0]+params['arms'][4]['base_position_m'][0])/2)
        self.assertAlmostEqual(task['workspace_world_m']['lower'][0],center[0]-half[0])
        self.assertAlmostEqual(task['workspace_world_m']['upper'][0],center[0]+half[0])

    def test_noslip_variant_changes_one_physics_option_and_preserves_initial_state(self):
        import xml.etree.ElementTree as ET
        source=ROOT/'configs/environments/spring_dial/floating-04-060-tight';target=ROOT/'configs/environments/spring_dial/floating-04-060-tight-noslip20'
        a=ET.parse(source/'scene.xml').getroot();b=ET.parse(target/'scene.xml').getroot()
        self.assertEqual(a.find('keyframe/key').attrib,b.find('keyframe/key').attrib)
        self.assertEqual(a.find('option').get('noslip_iterations'),'0');self.assertEqual(b.find('option').get('noslip_iterations'),'20')
        a.find('option').set('noslip_iterations','20')
        def value(e):return (e.tag,sorted(e.attrib.items()),[value(c) for c in e])
        self.assertEqual(value(a),value(b));self.assertEqual(make_task(target)['physics']['noslip_iterations'],20)

    def test_physics_step_matches_twenty_ms_at_both_declared_timesteps(self):
        from robofrontier.pipeline import World
        for dt,expected in ((.001,20),(.002,10)):
            calls=[];arm=SimpleNamespace(control=lambda actual:calls.append(actual),grasp_force=None)
            world=SimpleNamespace(arms={'A':arm},model=SimpleNamespace(opt=SimpleNamespace(timestep=dt)),data=object(),task={})
            with patch('robofrontier.pipeline.mujoco.mj_step') as step,patch('robofrontier.pipeline.mujoco.mj_forward'):
                World.step(world)
            self.assertEqual(step.call_count,expected);self.assertAlmostEqual(calls[0],.02)

class FloatingContactSequenceTests(unittest.TestCase):
    """Real apparatus geometry with synthetic contacts tests the private audit, not dynamics."""
    def setUp(self):
        asset=ROOT/'configs/environments/spring_dial/floating-04-060'
        self.task=make_task(asset);self.model=mujoco.MjModel.from_xml_path(str(asset/'scene.xml'));self.actual=mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model,self.actual,self.model.key('initial').id)
        for a in 'ABCD':
            q=self.actual.joint('dial_'+a+'_module_free').qpos;q[2]+=.5;q[3:]=[1,0,0,0]
        mujoco.mj_forward(self.model,self.actual)
        self.data=SimpleNamespace(time=0.,contact=[],qpos=self.actual.qpos,qvel=self.actual.qvel,geom_xmat=self.actual.geom_xmat,geom_xpos=self.actual.geom_xpos,body=self.actual.body)
        self.world=SimpleNamespace(model=self.model,data=self.data,task=self.task,time=0.)
        self.app=FloatingState(self.world)
        for i,a in enumerate('ABCD'):
            owner=chr(66+2*i);part=self.model.geom('dial_'+a+'_module_stem').id
            for suffix in ('left_finger','right_finger'):
                body=self.model.body(owner+'_'+suffix).id
                finger=next(g for g in range(self.model.ngeom) if self.model.geom_bodyid[g]==body)
                self.data.contact.append(SimpleNamespace(geom1=finger,geom2=part,dist=-.00001))
        self.red_robot=next(g for g,o in self.app.robot_geoms.items() if o=='A')
        self.red_part=self.model.geom('dial_A_red_knob').id
    def sample(self,t):
        self.data.time=t;self.world.time=t
        return self.app.sample(np.zeros(4))
    def red_contact(self):self.data.contact.append(SimpleNamespace(geom1=self.red_robot,geom2=self.red_part,dist=-.00001))
    def test_lift_requires_dwell_then_legal_partner_contact(self):
        self.assertFalse(self.sample(0));self.assertFalse(self.sample(.249));self.assertTrue(self.sample(.251))
        self.red_contact();self.assertTrue(self.sample(.252));self.assertEqual(self.app.contact_episodes,[])
    def test_premature_contact_is_recorded_once_while_contact_continues(self):
        self.red_contact();self.sample(0);self.sample(.1)
        self.assertEqual(len(self.app.contact_episodes),1);self.assertEqual(self.app.contact_episodes[0]['robot'],'A')
    def test_loss_of_support_disables_feedback_and_new_contact_needs_new_dwell(self):
        self.sample(0);self.sample(.26)
        contacts=list(self.data.contact);self.data.contact=[];self.assertFalse(self.sample(.27))
        self.data.contact=contacts;self.red_contact();self.assertFalse(self.sample(.28))
        self.assertEqual(len(self.app.contact_episodes),1);self.assertTrue(self.sample(.54))
    def test_solver_win_does_not_hide_process_violation_or_audit_post_win_cleanup(self):
        from robofrontier.spring_dial_task import SpringDialSampler,CODE_PATH
        self.red_contact()
        with tempfile.TemporaryDirectory() as tmp:
            self.world.directory=Path(tmp);(Path(tmp)/'private/outputs').mkdir(parents=True)
            code=Path(tmp)/CODE_PATH;code.parent.mkdir(parents=True,exist_ok=True);code.write_text(json.dumps({'code_symbols':[1]*4}))
            ids=[self.model.jnt_qposadr[self.model.joint('dial_'+a+'_hinge').id] for a in 'ABCD'];self.actual.qpos[ids]=np.pi/2
            sampler=SpringDialSampler(self.world)
            for i in range(1,65):self.data.time=self.world.time=i*.01;sampler.sample()
            r=sampler.result();self.assertTrue(r['solved']);self.assertFalse(r['task_success']);self.assertFalse(r['process_compliant'])
            count=r['metrics']['premature_knob_contact_events'];self.data.contact=[];self.data.time=self.world.time=.65;sampler.sample()
            self.red_contact();self.data.time=self.world.time=.66;sampler.sample()
            self.assertEqual(sampler.result()['metrics']['premature_knob_contact_events'],count)
            sampler.close()

if __name__=='__main__':unittest.main()
