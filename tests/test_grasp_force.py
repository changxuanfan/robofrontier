"""Actual-contact grasp control; no object truth is used by PandaArm itself."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from robofrontier.pipeline import World
from robofrontier.task_config import load_task,load_harness
from robofrontier.motion_check import prepare_check,state_id
from test_motion_check import robot_fixture


class GraspForceTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.cfg=load_harness()

    def tearDown(self):self.tmp.cleanup()

    def world(self, cube=True):
        task=load_task();task.update(arms={'B':0.},goals={'B':[]},noslip_iterations=10)
        task['objects']=([{'id':'cube','region':'B','color':'green','pattern':'plain','initial_position_m':[.42,0.,.42]}] if cube else [])
        return World(Path(self.tmp.name),task)

    def execute(self,w,op,p):
        command=w.validate_action('B',op,p,self.cfg);w.start_action('B','test',command)
        for _ in range(3000):
            w.step();done=w.poll_actions(15)
            if done:return done[0]
        self.fail('No terminal action')

    def at_cube(self,w):
        for point in ([.42,0,.53],[.42,0,.42]):
            self.assertEqual(self.execute(w,'move',dict(target_m=point,speed_m_s=.08))['status'],'completed')

    def grasp(self,w,force=50,width=.04):
        return self.execute(w,'gripper',dict(mode='grasp',width_m=width,force_n=force,speed_m_s=.04))

    def contact_force(self,w):
        total=0.;sides=set();geom=w.model.geom('cube_geom').id
        for i,c in enumerate(w.data.contact):
            if geom not in (c.geom1,c.geom2):continue
            other=c.geom2 if c.geom1==geom else c.geom1
            body=w.model.body(w.model.geom_bodyid[other]).name
            if body in ('B_left_finger','B_right_finger'):
                f=np.zeros(6);mujoco.mj_contactForce(w.model,w.data,i,f)
                total+=max(0.,f[0]);sides.add(body)
        return total,sides

    def test_force_choices_produce_distinct_contact_response_and_restore_position_mode(self):
        measured=[]
        for force in (30,50,70):
            with self.subTest(force=force):
                w=self.world();self.at_cube(w);arm=w.arms['B']
                original=[x[arm.grip].copy() for x in (w.model.actuator_gainprm,w.model.actuator_biasprm,w.model.actuator_ctrlrange,w.model.actuator_forcerange)]
                result=self.grasp(w,force)
                self.assertEqual(result['status'],'completed',result)
                self.assertTrue(result['is_grasped']);self.assertFalse(result['grasp_verified'])
                self.assertIsNone(result['measured_grasp_force_n'])
                for _ in range(30):w.step()
                actual,sides=self.contact_force(w);measured.append(actual)
                self.assertEqual(len(sides),2);self.assertLess(abs(actual-force),force*.15)
                self.assertTrue(arm.observe()['is_grasped'])
                self.assertEqual(w.model.actuator_forcerange[arm.grip].tolist(),[-70,70])
                self.assertLessEqual(abs(w.data.actuator_force[arm.grip]),70+1e-6)
                self.assertEqual(self.execute(w,'gripper',dict(width_m=.075))['status'],'completed')
                for value,before in zip((w.model.actuator_gainprm,w.model.actuator_biasprm,w.model.actuator_ctrlrange,w.model.actuator_forcerange),original):
                    np.testing.assert_array_equal(value[arm.grip],before)
                self.assertFalse(arm.observe()['is_grasped'])
        self.assertTrue(measured[0]<measured[1]<measured[2])

    def test_grasp_hold_survives_arm_move_and_release_is_real(self):
        w=self.world();self.at_cube(w);self.assertEqual(self.grasp(w)['status'],'completed')
        self.assertEqual(self.execute(w,'move',dict(target_m=[.42,0,.58],speed_m_s=.04))['status'],'completed')
        self.assertGreater(w.data.body('cube').xpos[2],.54)
        self.assertTrue(w.arms['B'].observe()['is_grasped'])
        self.assertEqual(self.execute(w,'gripper',dict(width_m=.075))['status'],'completed')
        for _ in range(70):w.step()
        self.assertLess(w.data.body('cube').xpos[2],.44)

    def test_reissuing_grasp_does_not_drop_existing_motor_force(self):
        w=self.world();self.at_cube(w);self.assertEqual(self.grasp(w)['status'],'completed')
        a=w.arms['B'];previous=a.grasp_drive_force
        command=w.validate_action('B','gripper',dict(mode='grasp',width_m=.04,
                                  force_n=70,speed_m_s=.04),self.cfg)
        w.start_action('B','increase-force',command)
        self.assertAlmostEqual(a.grasp_drive_force,previous)
        self.assertAlmostEqual(w.data.ctrl[a.grip],-previous)

    def test_empty_grasp_and_wrong_width_fail_without_leaving_force_close_active(self):
        for cube,width in ((False,.04),(True,.06)):
            w=self.world(cube);self.at_cube(w);result=self.grasp(w,width=width)
            self.assertEqual(result['status'],'failed',result)
            self.assertFalse(result['is_grasped']);self.assertIsNone(w.arms['B'].grasp_force)
            before=w.arms['B'].observe()['gripper_width_m']
            for _ in range(30):w.step()
            self.assertLess(abs(w.arms['B'].observe()['gripper_width_m']-before),.002)

    def test_stop_cancels_unfinished_force_close_and_holds_aperture(self):
        w=self.world(False);a=w.arms['B']
        c=w.validate_action('B','gripper',dict(mode='grasp',width_m=.04,force_n=50,speed_m_s=.01),self.cfg)
        w.start_action('B','cancel-grasp',c)
        for _ in range(20):w.step();w.poll_actions(15)
        w.request_stop('B','cancel-grasp');terminal=None
        for _ in range(200):
            w.step();done=w.poll_actions(15)
            if done:terminal=done[0];break
        self.assertIsNotNone(terminal);self.assertEqual(terminal['reason'],'stopped_by_agent')
        self.assertIsNone(a.grasp_force);width=a.observe()['gripper_width_m']
        for _ in range(50):w.step()
        self.assertLess(abs(a.observe()['gripper_width_m']-width),.002)

    def test_invalid_grasp_requests_do_not_change_live_controller(self):
        w=self.world();a=w.arms['B'];gain=w.model.actuator_gainprm.copy()
        for change in (dict(force_n=80),dict(force_n=float('nan')),dict(width_m=0),dict(epsilon_inner_m=0),dict(speed_m_s=.16),dict(state='close')):
            p=dict(mode='grasp',width_m=.04,force_n=50);p.update(change)
            with self.assertRaises(ValueError):w.validate_action('B','gripper',p,self.cfg)
        np.testing.assert_array_equal(w.model.actuator_gainprm,gain);self.assertIsNone(a.grasp_force)

    def test_force_close_respects_speed_without_alternating_motor_force(self):
        # A force-limited explicit actuator velocity damper can alternate at
        # the physics step frequency. Test the real free finger dynamics.
        for timestep in (.001, .002):
            with self.subTest(timestep=timestep):
                w=self.world(False);a=w.arms['B'];w.model.opt.timestep=timestep
                original_damping=w.model.dof_damping.copy()
                c=w.validate_action('B','gripper',dict(mode='grasp',width_m=.016,
                                    force_n=70,speed_m_s=.02),self.cfg)
                w.start_action('B','free-close',c)
                speeds=[];forces=[]
                for _ in range(round(.6/timestep)):
                    a.control(timestep);mujoco.mj_step(w.model,w.data)
                    speeds.append(a.observe()['gripper_velocity_m_s'])
                    forces.append(w.data.actuator_force[a.grip])
                self.assertLessEqual(max(abs(v) for v in speeds),.021)
                self.assertLessEqual(max(speeds),1e-4)
                # Braking can reverse force briefly; free motion must not
                # bang between the hardware limits or add passive damping.
                self.assertLess(max(abs(f) for f in forces),10.)
                self.assertAlmostEqual(speeds[-1],-.02,delta=.0002)
                np.testing.assert_array_equal(w.model.dof_damping,original_damping)
                a.hold_aperture()
                np.testing.assert_array_equal(w.model.dof_damping,original_damping)

    def test_grasp_timeout_does_not_keep_closing_after_failure(self):
        w=self.world(False);a=w.arms['B']
        c=w.validate_action('B','gripper',dict(mode='grasp',width_m=.04,force_n=30,speed_m_s=.005),self.cfg)
        c['timing']['timeout_sim_s']=.25;w.start_action('B','short-deadline',c)
        terminal=None
        for _ in range(250):
            w.step();done=w.poll_actions(15)
            if done:terminal=done[0];break
        self.assertIsNotNone(terminal);self.assertEqual(terminal['status'],'failed')
        self.assertFalse(terminal['is_grasped']);self.assertIsNone(a.grasp_force)
        width=a.observe()['gripper_width_m']
        for _ in range(50):w.step()
        self.assertLess(abs(a.observe()['gripper_width_m']-width),.002)

    def test_stopping_arm_motion_preserves_completed_grasp_and_its_width_feedback(self):
        w=self.world();self.at_cube(w);self.assertEqual(self.grasp(w)['status'],'completed')
        a=w.arms['B'];c=w.validate_action('B','move',dict(target_m=[.42,0,.60],speed_m_s=.04),self.cfg)
        w.start_action('B','carrying',c)
        for _ in range(20):w.step();w.poll_actions(15)
        w.request_stop('B','carrying')
        self.assertEqual(a.grasp_force,50)
        self.assertAlmostEqual(a.observe()['gripper_target_width_m'],.04)
        terminal=None
        for _ in range(200):
            w.step();done=w.poll_actions(15)
            if done:terminal=done[0];break
        self.assertIsNotNone(terminal);self.assertEqual(terminal['reason'],'stopped_by_agent')
        self.assertTrue(a.observe()['is_grasped']);self.assertEqual(a.grasp_force,50)

    def test_motion_screen_after_force_grasp_is_read_only(self):
        w=robot_fixture();a=w.arms['A']
        for s in ('finger_joint1','finger_joint2'):w.data.joint('A_'+s).qpos[0]=.02
        c=w.validate_action('A','gripper',dict(mode='grasp',width_m=.04,force_n=50),self.cfg)
        w.start_action('A','grasp',c)
        for i in range(80):
            w.data.time=i*.02;a.control(.02)
            if a.poll(w.time,15):break
        self.assertIsNone(a.job);before=state_id(w);ctrl=w.data.ctrl.copy();bias=w.model.actuator_biasprm.copy()
        with patch('mujoco.mj_step',side_effect=AssertionError('No dynamics in read-only screen')):
            list(prepare_check(w,dict(motions=[dict(actor='A',delta_m=[0,0,.01])],horizon_s=.2),self.cfg))
        self.assertEqual(state_id(w),before);np.testing.assert_array_equal(ctrl,w.data.ctrl)
        np.testing.assert_array_equal(bias,w.model.actuator_biasprm)

    def test_grasp_settling_uses_individual_position_window_not_instant_solver_velocity(self):
        w=robot_fixture();a=w.arms['A']
        for s in ('finger_joint1','finger_joint2'):w.data.joint('A_'+s).qpos[0]=.02
        c=w.validate_action('A','gripper',dict(mode='grasp',width_m=.04,force_n=50),self.cfg);w.start_action('A','grasp',c)
        terminal=None
        for i in range(80):
            w.data.time=i*.02
            for s in ('finger_joint1','finger_joint2'):
                j=w.data.joint('A_'+s);j.qpos[0]=.02+(-1)**i*.000002;j.qvel[0]=(-1)**i*.005
            a.control(.02);result=a.poll(w.time,15)
            if result:terminal=result;break
        self.assertIsNotNone(terminal);self.assertEqual(terminal['status'],'completed')
        self.assertFalse(terminal['grasp_verified'])

    def test_unfinished_grasp_preview_cannot_change_live_motor_phase_or_model(self):
        w=robot_fixture();a=w.arms['A']
        for s in ('finger_joint1','finger_joint2'):w.data.joint('A_'+s).qpos[0]=.02
        command=w.validate_action('A','gripper',dict(mode='grasp',width_m=.04,force_n=50),self.cfg);w.start_action('A','pending-grasp',command)
        bias=w.model.actuator_biasprm.copy();ctrl=w.data.ctrl.copy();samples=len(a.grasp_settling)
        with patch('mujoco.mj_step',side_effect=AssertionError('No dynamics in a motion preview')):
            list(prepare_check(w,dict(motions=[dict(actor='B',delta_m=[0,0,.01])],horizon_s=1.),self.cfg))
        np.testing.assert_array_equal(w.model.actuator_biasprm,bias);np.testing.assert_array_equal(w.data.ctrl,ctrl)
        self.assertEqual(a.grasp_phase,'approach');self.assertEqual(len(a.grasp_settling),samples)

    def test_settled_force_hold_retains_a_loaded_cube_without_changing_passive_damping(self):
        w=self.world();self.at_cube(w);a=w.arms['B'];passive=w.model.dof_damping.copy()
        self.assertEqual(self.grasp(w,70)['status'],'completed');self.assertEqual(a.grasp_phase,'hold')
        self.assertEqual(self.execute(w,'move',dict(target_m=[.42,0,.58],speed_m_s=.04))['status'],'completed')
        # Separate stress test only: an imposed load is not a task witness.
        w.data.xfrc_applied[w.model.body('cube').id,2]=-8.
        for _ in range(300):
            w.step();self.assertLessEqual(abs(w.data.actuator_force[a.grip]),70+1e-6)
        self.assertGreater(w.data.body('cube').xpos[2],.54);self.assertTrue(a.observe()['is_grasped'])
        np.testing.assert_array_equal(w.model.dof_damping,passive)

    def test_opposing_finger_drift_cannot_settle_just_because_width_is_constant(self):
        w=robot_fixture();a=w.arms['A']
        for s in ('finger_joint1','finger_joint2'):w.data.joint('A_'+s).qpos[0]=.02
        c=w.validate_action('A','gripper',dict(mode='grasp',width_m=.04,force_n=50),self.cfg);w.start_action('A','grasp',c)
        for i in range(40):
            for sign,s in zip((1,-1),('finger_joint1','finger_joint2')):
                j=w.data.joint('A_'+s);j.qpos[0]=.02+sign*i*.02*.0015;j.qvel[0]=sign*.0015
            self.assertIsNone(a.poll(i*.02,15))
        self.assertIsNotNone(a.job)


if __name__=='__main__':unittest.main()
