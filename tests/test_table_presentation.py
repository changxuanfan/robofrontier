import unittest
import mujoco
import numpy as np
from robofrontier.suite.table_presentation import scale_rectangular_opening_for_render


class TablePresentationTests(unittest.TestCase):
    def test_half_opening_preserves_outer_top_height_and_all_moving_coordinates(self):
        model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <geom name="table_0" type="box" pos="0 -1.5 .775" size="2 .5 .025"/>
          <geom name="table_1" type="box" pos="0 1.5 .775" size="2 .5 .025"/>
          <geom name="table_2" type="box" pos="-1.5 0 .775" size=".5 1 .025"/>
          <geom name="table_3" type="box" pos="1.5 0 .775" size=".5 1 .025"/>
          <geom name="leg" type="box" pos="1.8 1.8 .375" size=".03 .03 .375"/>
          <body name="recorded" pos="1.5 0 1"><freejoint/><geom type="sphere" size=".03"/></body>
        </worldbody></mujoco>''')
        data=mujoco.MjData(model);q=data.qpos.copy();bodies=model.body_pos.copy();leg=model.geom_pos[4].copy()
        moving=model.geom_size[5].copy();counts=(model.nq,model.nbody,model.ngeom)
        result=scale_rectangular_opening_for_render(model)
        self.assertEqual(result['old_opening_m'],[2,2]);self.assertEqual(result['new_opening_m'],[1,1])
        self.assertEqual(result['outer_envelope_m'],[4,4])
        self.assertEqual(counts,(model.nq,model.nbody,model.ngeom))
        np.testing.assert_array_equal(model.body_pos,bodies);np.testing.assert_array_equal(data.qpos,q)
        np.testing.assert_array_equal(model.geom_pos[4],leg);np.testing.assert_array_equal(model.geom_size[5],moving)
        self.assertAlmostEqual(model.geom_pos[0,2]+model.geom_size[0,2],.8)

    def test_solid_top_is_not_given_an_artificial_hole(self):
        model=mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom name="table_0" type="box" size="1 1 .025"/></worldbody></mujoco>')
        before=model.geom_size.copy();self.assertIsNone(scale_rectangular_opening_for_render(model))
        np.testing.assert_array_equal(model.geom_size,before)

    def test_invalid_scale_is_rejected_without_modifying_the_model(self):
        model=mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom name="table_0" type="box" size="1 1 .025"/></worldbody></mujoco>')
        before=model.geom_size.copy()
        with self.assertRaises(ValueError):scale_rectangular_opening_for_render(model,0)
        np.testing.assert_array_equal(model.geom_size,before)
