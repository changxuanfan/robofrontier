"""A height revision preserves relative geometry, control and source records."""
from copy import deepcopy
import json
import unittest

import mujoco
import numpy as np

from robofrontier.suite.v2 import ROOT,directory,public_task
from robofrontier.workstation import Workstation


class WorkstationReframeTests(unittest.TestCase):
    def test_all_sources_translate_without_scaling_or_changing_dynamics(self):
        # R10 is the height-only revision. Later family revisions deliberately
        # change table/cargo topology and are checked by their own contracts.
        for case in json.loads((ROOT/'configs/suites/frontier-v2-r10.json').read_text())['cases']:
            with self.subTest(case=case['id']):
                previous=deepcopy(case);previous['scene_revision']=case['source_scene_revision']
                old=Workstation(directory(previous));new=Workstation(directory(case))
                old.frame();new.frame();a,b=old.model,new.model
                self.assertEqual((a.nq,a.nv,a.nu),(b.nq,b.nv,b.nu))
                for name in ('body_mass','body_inertia','jnt_range','jnt_axis','jnt_stiffness',
                             'dof_damping','actuator_gainprm','actuator_biasprm','geom_friction'):
                    np.testing.assert_allclose(getattr(a,name),getattr(b,name),atol=1e-12)
                np.testing.assert_allclose(new.data.xpos[1:]-old.data.xpos[1:],np.tile([0,0,.4],(b.nbody-1,1)),atol=1e-12)
                np.testing.assert_allclose(new.data.cam_xpos-old.data.cam_xpos,np.tile([0,0,.4],(b.ncam,1)),atol=1e-12)
                self.assertAlmostEqual(b.opt.timestep,.002)
                task=json.loads((new.path/'task.json').read_text())
                self.assertAlmostEqual(task['table_height_m'],.8)
                self.assertAlmostEqual(task['workspace_world_m']['lower'][2],.425)
                self.assertAlmostEqual(task['workspace_world_m']['upper'][2],2.1)
                self.assertAlmostEqual(new.manifest['support_surfaces']['tabletop_z_m'],.8)
                for obj in task['objects']:
                    np.testing.assert_allclose(new.data.body(obj['id']).xpos,obj['initial_position_m'],atol=1e-12)
                if case['family']=='mosaic':self.assertEqual(public_task(task)['target']['center_world_m'][2],.8)
                # The ground remains at zero. Legs are the sole resized solids.
                for i in range(a.ngeom):
                    name=a.geom(i).name
                    if name.startswith('table_leg_'):self.assertAlmostEqual(b.geom_size[i,2]-a.geom_size[i,2],.2)
                    else:np.testing.assert_allclose(a.geom_size[i],b.geom_size[i],atol=1e-12)


if __name__=='__main__':unittest.main()
