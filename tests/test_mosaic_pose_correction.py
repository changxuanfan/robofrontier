import copy
import math
import unittest
import numpy as np

from robofrontier.suite.oracle import CUBE_ROTATIONS
from scripts.execute_mosaic_flat_pose_correction import seated_correction_geometry,rz


class SeatedCorrectionGeometryTests(unittest.TestCase):
    def setUp(self):
        self.goal=dict(position_m=[.2,.3,.83])
        self.task=dict(level=3)

    def test_all_twenty_four_cube_orientations_have_the_same_small_angle_admission(self):
        axes=set()
        for base in CUBE_ROTATIONS:
            for sign in (-1,1):
                body=dict(position_m=[.2,.3,.83],rotation=(rz(sign*math.radians(9))@base).tolist())
                before=copy.deepcopy(body)
                result=seated_correction_geometry(self.task,body,self.goal)
                self.assertAlmostEqual(result['orientation_error_rad'],math.radians(9),places=9)
                self.assertEqual(body,before);axes.add(result['up_body_axis'])
        self.assertEqual(axes,{0,1,2})

    def test_small_position_error_includes_the_actual_side_resting_f25_pose(self):
        rotation=[[-5.55e-17,-.9974989403436509,.07068142622565804],[-1.67e-16,.07068142622565804,.9974989403436509],[-1.,5.55e-17,-1.67e-16]]
        body=dict(position_m=[.20370560964262505,.2894009352608345,.8299805923328202],rotation=rotation)
        result=seated_correction_geometry(self.task,body,dict(position_m=[.2025,.2835,.83]))
        self.assertEqual(result['up_body_axis'],0)
        self.assertLess(result['orientation_error_rad'],math.radians(5))
        self.assertGreater(result['xy_error_m'],.005)

    def test_rejects_tilt_far_position_large_rotation_and_nonfinite_geometry(self):
        for change in ('tilt','far','large_turn','nan'):
            body=dict(position_m=[.206,.3,.83],rotation=np.eye(3).tolist())
            if change=='tilt':
                angle=math.radians(4);body['rotation']=[[1,0,0],[0,math.cos(angle),-math.sin(angle)],[0,math.sin(angle),math.cos(angle)]]
            elif change=='far':body['position_m'][0]=.22
            elif change=='large_turn':body['rotation']=rz(math.radians(15)).tolist()
            else:body['rotation'][0][0]=float('nan')
            with self.subTest(change=change),self.assertRaises(ValueError):seated_correction_geometry(self.task,body,self.goal)

    def test_pocket_scope_keeps_its_original_five_mm_and_angular_case_requirement(self):
        body=dict(position_m=[.204,.3,.83],rotation=rz(math.radians(9)).tolist())
        seated_correction_geometry(dict(level=1),body,self.goal)
        body['position_m'][0]=.206
        with self.assertRaises(ValueError):seated_correction_geometry(dict(level=1),body,self.goal)
        body['position_m'][0]=.204;body['rotation']=np.eye(3).tolist()
        with self.assertRaises(ValueError):seated_correction_geometry(dict(level=1),body,self.goal)


if __name__=='__main__':unittest.main()
