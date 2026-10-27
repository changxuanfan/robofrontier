from types import SimpleNamespace
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from robofrontier.factory_workstations import FactoryPlayback


class FactoryArrayReplayTests(unittest.TestCase):
    def fixture(self, scale=1):
        source_model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <geom name="table" type="box" size="1 1 .1" pos="0 0 .4"/>
          <body name="robot" pos="1 0 .5"><joint name="joint" axis="0 0 1"/>
            <geom type="box" size=".2 .03 .03" pos=".2 0 0"/></body>
          <body name="cargo" pos="0 0 1"><freejoint/>
            <geom name="beam" type="box" size=".5 .02 .02"/></body>
        </worldbody></mujoco>''')
        angle = .7
        c, s = np.cos(angle/2), np.sin(angle/2)
        target_model = mujoco.MjModel.from_xml_string(f'''<mujoco><worldbody>
          <body name="case__world" pos="4 -3 2" quat="{c} 0 0 {s}">
            <geom name="case__table" type="box" size="1 1 .1" pos="0 0 .4"/></body>
          <body name="case__robot" mocap="true"><geom name="case__finger"
            type="box" size=".2 .03 .03" pos=".2 0 0"/></body>
          <body name="case__cargo" mocap="true"><geom name="case__beam"
            type="box" size=".5 .02 .02"/></body>
        </worldbody></mujoco>''')
        source = SimpleNamespace(model=source_model, data=mujoco.MjData(source_model),
                                 manifest={'scene_sha256':'frozen','geometries':[
                                     dict(id=name,source_index=i) for i,name in
                                     enumerate(['table','robot:geom0','beam'])]})
        play = FactoryPlayback.__new__(FactoryPlayback)
        play.model, play.data = target_model, mujoco.MjData(target_model)
        play._recorded_maps = {}
        play.manifest = dict(workstation_names={'case':dict(
            source_scene_sha256='frozen',
            body_names={n:'case__'+n for n in ['world','robot','cargo']},
            geom_names={'table':'case__table','robot:geom0':'case__finger','beam':'case__beam'})},
            placements=[dict(case_id='case',yaw_rad=angle,translation_m=[4,-3,2],scale=scale)])
        return source, play

    def test_rigid_translation_yaw_and_articulated_states_without_physics(self):
        source, play = self.fixture()
        source.data.qpos[0] = -.8
        with patch('mujoco.mj_step',side_effect=AssertionError('No replay physics')):
            first = play.apply_arrays('case',source,source.data.qpos.copy(),1,verify=True)
            self.assertLess(first['position_error_m'],1e-12)
            old = play.data.geom_xpos[play.model.geom('case__finger').id].copy()
            q = source.data.qpos.copy();q[0] += .2;q[1] += .1
            second = play.apply_arrays('case',source,q,2,verify=True)
            self.assertLess(second['rotation_matrix_error'],1e-12)
            self.assertGreater(np.linalg.norm(play.data.geom_xpos[play.model.geom('case__finger').id]-old),.01)

    def test_revision_and_scale_mismatch_rejected(self):
        source, play = self.fixture(scale=2)
        with self.assertRaisesRegex(ValueError,'cannot be scaled'):
            play.apply_arrays('case',source,source.data.qpos.copy(),0)
        source.manifest['scene_sha256']='other'
        with self.assertRaisesRegex(ValueError,'source revision differs'):
            play.apply_arrays('case',source,source.data.qpos.copy(),0)


if __name__ == '__main__':
    unittest.main()
