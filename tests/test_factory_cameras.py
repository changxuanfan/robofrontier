import unittest
import numpy as np
from robofrontier.factory_cameras import place_station_camera


class StationCameraChecks(unittest.TestCase):
    def test_front_camera_follows_rigid_station_placement_without_changing_framing(self):
        source=dict(id='global',position_m=[0,-2,3],target_m=[0,0,1],
            vertical_fov_deg=43,projection='perspective',hidden_layers=[])
        manifest=dict(placements=[dict(case_id='test-desk',translation_m=[10,20,0],yaw_rad=np.pi/2)])
        camera=place_station_camera(manifest,'test-desk',source)
        np.testing.assert_allclose(camera['position_m'],[12,20,3])
        np.testing.assert_allclose(camera['target_m'],[10,20,1])
        self.assertEqual(camera['vertical_fov_deg'],43)
        self.assertEqual(camera['source_camera'],'global')
        self.assertEqual(source['position_m'],[0,-2,3])
