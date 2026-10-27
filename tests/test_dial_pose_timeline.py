import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import mujoco
import numpy as np

from robofrontier.suite.dial_pose_timeline import EditedMotion, IdleSettings, build_edit, idle_mask, playback_speed
from robofrontier.motion_recording import MotionWriter
from robofrontier.workstation import Workstation


class MotionEditTests(unittest.TestCase):
    def test_ten_x_default_is_limited_to_dial_family(self):
        self.assertEqual(playback_speed('dial-best5-n04-d1'),10.)
        self.assertEqual(playback_speed('dial-n08-d3'),10.)
        with self.assertRaises(ValueError):playback_speed('transport-a04-d1')
        with self.assertRaises(ValueError):playback_speed('mosaic-a04-d1')
        self.assertEqual(playback_speed('dial-best5-n04-d1',requested=2),2.)

    def setUp(self):
        self.model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <body><joint name="A_joint1"/><geom size=".1"/></body>
          <body pos="1 0 0"><joint name="A_finger_joint1" type="slide"/><geom size=".1"/></body>
          <body pos="2 0 0"><freejoint name="module"/><geom size=".1"/></body>
          <body pos="3 0 0"><joint name="dial_hinge"/><geom size=".1"/></body>
          </worldbody></mujoco>''')
        self.times = np.arange(721)/60
        self.q = np.tile(self.model.qpos0, (len(self.times), 1))

    def mask(self):
        return idle_mask(self.model, self.times, self.q)[0]

    def test_sustained_object_and_arm_micro_jitter_removed(self):
        self.q[:, 0] = np.deg2rad(.02)*np.sin(self.times*40)
        self.q[:, 2] += .00005*np.sin(self.times*31)
        self.q[:, -1] = np.deg2rad(.1)*np.sin(self.times*27)
        keep = self.mask()
        self.assertFalse(keep[120:600].any())
        self.assertTrue(keep[0] and keep[-1])

    def test_real_arm_motion_and_gripper_closing_retained(self):
        for channel, amount in [(0, .1), (1, .008)]:
            self.q[:] = self.model.qpos0
            self.q[:, channel] += amount*self.times
            self.assertTrue(self.mask().all())

    def test_stationary_arms_do_not_hide_drop_or_rotor_motion(self):
        for channel, amount in [(4, -.01), (-1, .1)]:
            self.q[:] = self.model.qpos0
            self.q[:, channel] += amount*self.times
            self.assertTrue(self.mask().all())

    def test_slow_real_drift_not_misclassified_by_window(self):
        self.q[:, 2] += .0002*self.times
        self.assertTrue(self.mask().all())

    def test_quaternion_sign_flips_are_not_real_rotation(self):
        self.q[::2, 5:9] *= -1
        self.assertFalse(self.mask()[120:600].any())

    def test_short_pauses_preserved_and_long_pause_cut(self):
        self.q[:, 0] = np.minimum(self.times, 2)*.1 + np.maximum(self.times-9, 0)*.1
        keep = self.mask()
        self.assertTrue(keep[30:90].all())
        self.assertFalse(keep[240:420].any())
        self.assertTrue(keep[600:690].all())

    def test_source_edit_protects_feedback_binds_hashes_and_preserves_states(self):
        root = Path(__file__).resolve().parents[1]
        case = json.loads((root/'configs/suites/dial-best5-r11.json').read_text())['cases'][0]
        source = Workstation(root/case['directory'])
        with tempfile.TemporaryDirectory() as td, patch('mujoco.mj_step', side_effect=AssertionError('No edit physics')):
            path = Path(td)/'motion'
            writer = MotionWriter(path, source.path, source.model, source.data, hz=60)
            for i in range(1, 481):
                source.data.time = i/60
                if i == 240:
                    source.model.geom_rgba[0, 0] = .13
                writer.capture(source.data)
            writer.close()
            edit = build_edit(path)
            self.assertGreater(edit['removed_frames'], 300)
            view = EditedMotion(path, edit)
            frames = list(view.frames())
            self.assertEqual(len(frames), edit['retained_frames'])
            self.assertIn(240, [f[0] for f in frames])
            for _, _, _, q in frames:
                np.testing.assert_array_equal(q, source.data.qpos.astype(np.float32))
            ids = view.video_indices(30, 12)
            self.assertIn(240, ids)
            self.assertEqual(ids[-1], 480)
            # The same source schedule is used regardless of background/placement.
            np.testing.assert_array_equal(ids, EditedMotion(path, edit).video_indices(30, 12))
            edit['motion_index_sha256'] = 'wrong'
            with self.assertRaisesRegex(ValueError, 'source recording differs'):
                EditedMotion(path, edit)

    def test_factory_translation_yaw_preserve_every_visible_geometry(self):
        from robofrontier.factory_workstations import FactoryPlayback, compose
        from robofrontier.robot_control import quat_matrix
        root = Path(__file__).resolve().parents[1]
        case = json.loads((root/'configs/suites/dial-best5-r11.json').read_text())['cases'][0]
        source = Workstation(root/case['directory'])
        place = dict(case_id=case['id'],family='dial',source_directory=str(source.path),
            translation_m=[0.,0.,0.],yaw_rad=0.,scale=1.,planning_bounds_xy_m=[-3.,-3.,3.,3.])
        with tempfile.TemporaryDirectory() as td, patch('mujoco.mj_step',side_effect=AssertionError('No replay dynamics')):
            folder = Path(td)
            (folder/'assets').mkdir()
            shell = folder/'shell.xml'
            shell.write_text('<mujoco><asset/><worldbody/></mujoco>')
            (folder/'manifest.json').write_text(json.dumps(dict(id='test',regions=[])))
            catalog = folder/'catalog.json'
            catalog.write_text(json.dumps(dict(cases=[case])))
            yaw = .63
            shift = np.array([9.,-4.,.25])
            with patch('robofrontier.factory_workstations.placements',return_value=[place]):
                scene = compose(shell,catalog,folder/'factory',{case['id']:dict(translation_m=shift.tolist(),yaw_rad=yaw)})
            play = FactoryPlayback(scene)
            source.data.qpos[source.model.joint('A_joint1').qposadr[0]] += .12
            source.data.qpos[source.model.joint('D01_hinge').qposadr[0]] += .01
            expected = play.apply(case['id'],source,source.data.qpos,3.)
            for name, target in play.manifest['workstation_names'][case['id']]['geom_names'].items():
                geometry = play.model.geom(target).id
                pose = expected['geometries'][name]
                np.testing.assert_allclose(play.data.geom_xpos[geometry],pose['position_m'],atol=1e-6)
                np.testing.assert_allclose(play.data.geom_xmat[geometry].reshape(3,3),quat_matrix(pose['quaternion_wxyz']),atol=1e-6)
            self.assertEqual(play.model.nv,0)



if __name__ == '__main__':
    unittest.main()
