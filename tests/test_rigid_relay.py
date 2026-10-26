"""Physical/export contracts for cable-free scenes, not a manipulation proof."""
import json
import unittest
from types import SimpleNamespace

import mujoco
import numpy as np

from robofrontier.suite.v2 import cases,directory,public_task
from robofrontier.suite.evaluate import Monitor
from robofrontier.robot_control import PandaArm
from robofrontier.workstation import Workstation


class RigidRelayTests(unittest.TestCase):
    def source(self,arms,level,legacy=False):
        case=next(c for c in cases() if c['id']==f'relay-a{arms:02d}-d{level}')
        if legacy:case=dict(case,scene_revision=15 if arms==12 else 13)
        source=Workstation(directory(case));source.frame();mujoco.mj_forward(source.model,source.data)
        task=json.loads((source.path/'task.json').read_text())
        return source,task

    def test_free_weights_closed_contacts_and_no_hidden_actuators(self):
        for n in (8,12,16):
            for level in (2,3):
                with self.subTest(arms=n,level=level):
                    source,task=self.source(n,level);m,d=source.model,source.data
                    self.assertEqual(m.nplugin,0);self.assertAlmostEqual(m.opt.timestep,.002)
                    self.assertEqual(m.nu,8*n)  # Seven arm joints and one gripper.
                    self.assertIsNone(task['mechanism']['tray'])
                    for item in task['mechanism']['payload_assemblies']:
                        ident=item['id'];weight=item['weight_body']
                        self.assertEqual(int(m.joint(weight+'_free').type[0]),int(mujoco.mjtJoint.mjJNT_FREE))
                        self.assertAlmostEqual(float(m.body(weight).mass[0]),task['mechanism']['suspended_mass_kg'],places=9)
                        self.assertGreater(float(m.body(weight).mass[0]),.6)
                        for prefix in (ident+'_hook_',weight+'_eye_'):
                            ids=[g for g in range(m.ngeom) if m.geom(g).name.startswith(prefix) and 'bracket' not in m.geom(g).name]
                            self.assertEqual(len(ids),20)
                            self.assertTrue(all(m.geom_contype[g] or m.geom_conaffinity[g] for g in ids))
                        if level==3:
                            self.assertEqual(len(item['beam_bodies']),2)
                            self.assertAlmostEqual(m.jnt_stiffness[m.joint(ident+'_flexion').id],18.)
                        else:self.assertEqual(item['beam_bodies'],[ident])
                    self.assertTrue(all('payload' not in m.equality(i).name for i in range(m.neq)))
                    for con in d.contact:
                        names=[m.geom(int(g)).name for g in con.geom]
                        if any('payload' in name for name in names):self.assertGreaterEqual(con.dist,-.0002)

    def test_both_beam_halves_are_graspable_but_weight_and_eyes_are_not(self):
        source,task=self.source(8,3);m,d=source.model,source.data
        world=SimpleNamespace(model=m,data=d,task=task,arms={a:PandaArm(m,d,a) for a in task['arms']})
        mon=Monitor(world)
        for ident,state in mon.relay_v2.states.items():
            names={m.geom(g).name for g in state['rod_geoms']}
            self.assertEqual(names,{ident+'_beam_segment0',ident+'_beam_segment1'})
            self.assertTrue(any(m.geom(g).name==ident+'_weight_core' for g in state['all_geoms']))
        mon.physical_step();self.assertFalse(mon.result()['task_success'])

    def test_passive_hinge_and_weight_are_in_offline_replay_registry(self):
        source,task=self.source(8,3);q=source.initial_qpos.copy();m=source.model
        ident=task['objects'][0]['id'];j=m.joint(ident+'_flexion');q[m.jnt_qposadr[j.id]]=.12
        before=source.frame();after=source.frame(q)
        distal=ident+'_distal'
        self.assertNotEqual(before['bodies'][distal]['quaternion_wxyz'],after['bodies'][distal]['quaternion_wxyz'])
        recorded=set(source.manifest['motion_recording_contract']['dynamic_bodies'])
        self.assertIn(distal,recorded);self.assertIn(ident+'_weight',recorded)
        public=public_task(task)
        self.assertNotIn('initial_position_m',json.dumps(public))
        self.assertNotIn('tray_close',task['cameras'])

    def test_mixed_hinges_match_counts_motion_and_visible_bearing_axis(self):
        for n,expected in ((8,['in_plane','out_of_plane']),
                           (12,['in_plane','out_of_plane','out_of_plane']),
                           (16,['in_plane','in_plane','out_of_plane','out_of_plane'])):
            source,task=self.source(n,3,legacy=True);m,d=source.model,source.data
            assemblies=task['mechanism']['payload_assemblies']
            self.assertEqual([a['elastic_hinge']['mode'] for a in assemblies],expected)
            for item in assemblies:
                ident=item['id'];mode=item['elastic_hinge']['mode'];joint=m.joint(ident+'_flexion')
                axis=[0,0,1] if mode=='in_plane' else [1,0,0]
                np.testing.assert_allclose(m.jnt_axis[joint.id],axis)
                # The cylinder pin, actual joint axis and public description agree.
                housing=m.geom(ident+'_hinge_housing').id
                np.testing.assert_allclose(d.geom_xmat[housing].reshape(3,3)[:,2],d.xaxis[joint.id],atol=1e-12)
                child=d.body(ident+'_distal')
                before=child.xpos+child.xmat.reshape(3,3)@np.array([0,.3,0])
                d.joint(joint.name).qpos[0]=.2;mujoco.mj_forward(m,d)
                child=d.body(ident+'_distal');after=child.xpos+child.xmat.reshape(3,3)@np.array([0,.3,0])
                if mode=='in_plane':self.assertAlmostEqual(after[2],before[2],places=10)
                else:self.assertGreater(abs(after[2]-before[2]),.05)
                d.joint(joint.name).qpos[0]=0.;mujoco.mj_forward(m,d)


if __name__=='__main__':unittest.main()
