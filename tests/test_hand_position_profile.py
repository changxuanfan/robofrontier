"""Opt-in hand driver: encoder position tracking and force-mode isolation."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np
from robofrontier.pipeline import World
from robofrontier.task_config import load_harness,load_task,physical_control_fingerprint


class HandPositionProfileTests(unittest.TestCase):
    def setUp(self):
        self.temp=TemporaryDirectory();self.directory=Path(self.temp.name)
        self.cfg=load_harness();self.cfg['gripper_position_servo']='pd1000-v1'

    def tearDown(self):self.temp.cleanup()

    def world(self,profile='pd1000-v1'):
        cfg={**self.cfg,'gripper_position_servo':profile}
        path=self.directory/'private/inputs/harness.json';path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(cfg))
        task=load_task();task.update(arms={'B':0.},goals={'B':[]},noslip_iterations=10)
        task['objects']=[dict(id='cube',region='B',color='green',pattern='plain',initial_position_m=[.42,0.,.42])]
        return World(self.directory,task)

    def command(self,w,op,parameters):
        command=w.validate_action('B',op,parameters,self.cfg);w.start_action('B','profile-'+op,command)
        deadline=w.time+command['timing']['timeout_sim_s']+4.
        while w.time<deadline:
            w.step();done=w.poll_actions(15.)
            if done:
                self.assertEqual(done[0]['status'],'completed',done[0]);return done[0]
        self.fail('Bounded controller did not settle')

    def test_frozen_profile_tracks_intermediate_and_full_empty_opening(self):
        w=self.world();a=w.arms['B']
        self.assertEqual(a.hand_position_servo,'pd1000-v1')
        self.assertAlmostEqual(w.model.actuator_biasprm[a.grip,1],-1000.)
        self.assertEqual(w.model.actuator_forcerange[a.grip].tolist(),[-70.,70.])
        self.command(w,'gripper',dict(width_m=0.,speed_m_s=.04))
        for width in (.052,.08):
            result=self.command(w,'gripper',dict(width_m=width,speed_m_s=.02))
            self.assertTrue(result['target_width_reached'])
            self.assertLess(abs(a.observe()['gripper_width_m']-width),.002)

    def test_force_grasp_then_position_release_restores_distinct_damping(self):
        w=self.world();a=w.arms['B'];position_parameters=tuple(x.copy() for x in a.grip_position_parameters)
        physical=[w.model.body_mass.copy(),w.model.dof_damping.copy(),w.model.geom_friction.copy()]
        for target in ([.42,0.,.52],[.42,0.,.42]):
            self.command(w,'move',dict(target_m=target,speed_m_s=.06))
        self.command(w,'gripper',dict(mode='grasp',width_m=.04,force_n=50.,speed_m_s=.02,epsilon_inner_m=.008,epsilon_outer_m=.008))
        self.assertEqual(a.grasp_phase,'hold')
        self.assertAlmostEqual(w.model.actuator_biasprm[a.grip,2],-10.)
        with self.assertRaises(ValueError):a.configure_hand_position_servo('legacy')
        self.command(w,'move',dict(target_m=[.42,0.,.54],speed_m_s=.06))
        self.assertGreater(w.data.joint('cube_free').qpos[2],.49)
        self.command(w,'move',dict(target_m=[.42,0.,.42],speed_m_s=.04))
        result=self.command(w,'gripper',dict(mode='move',width_m=.08,speed_m_s=.02))
        self.assertTrue(result['target_width_reached']);self.assertIsNone(a.grasp_force)
        for actual,expected in zip((w.model.actuator_gainprm[a.grip],w.model.actuator_biasprm[a.grip],w.model.actuator_ctrlrange[a.grip],w.model.actuator_forcerange[a.grip]),position_parameters):
            np.testing.assert_array_equal(actual,expected)
        for actual,expected in zip((w.model.body_mass,w.model.dof_damping,w.model.geom_friction),physical):
            np.testing.assert_array_equal(actual,expected)

    def test_legacy_profile_keeps_original_actuator(self):
        w=self.world('legacy');a=w.arms['B']
        self.assertEqual(a.hand_position_servo,'legacy')
        self.assertAlmostEqual(w.model.actuator_gainprm[a.grip,0],.01568627451)
        self.assertEqual(w.model.actuator_forcerange[a.grip].tolist(),[-100.,100.])

    def test_unknown_and_active_reconfiguration_rejected(self):
        w=self.world();a=w.arms['B']
        with self.assertRaises(ValueError):a.configure_hand_position_servo('typo')
        command=w.validate_action('B','gripper',dict(width_m=0.),self.cfg)
        w.start_action('B','active',command)
        with self.assertRaises(ValueError):a.configure_hand_position_servo('legacy')

    def test_driver_selection_changes_physical_control_fingerprint(self):
        hashes={'robofrontier/'+name:'same-source' for name in
                ('robot_control.py','sim.py','motion_check.py','task_config.py','pipeline.py')}
        legacy={**self.cfg,'gripper_position_servo':'legacy'}
        self.assertNotEqual(physical_control_fingerprint(hashes,legacy),physical_control_fingerprint(hashes,self.cfg))


if __name__=='__main__':unittest.main()
