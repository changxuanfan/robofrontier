"""Reduced-chain FK/Jacobian must agree with the original full scene."""
import unittest
import mujoco,numpy as np
from robofrontier.sim import RobotIK
from robofrontier.suite.scenes import catalog,case_directory

class RobotOnlyKinematicsTests(unittest.TestCase):
    def test_fk_and_jacobian_match_full_world_at_rotated_bases(self):
        rng=np.random.default_rng(818)
        for case_id,robot in [('transport-a12-d3','H'),('relay-a16-d3','P'),('mosaic-a08-7x21-d1','B'),('dial-n08-d3','F')]:
            case=next(c for c in catalog()['cases'] if c['id']==case_id);m=mujoco.MjModel.from_xml_path(str(case_directory(case)/'scene.xml'));d=mujoco.MjData(m);ik=RobotIK(m,robot+'_')
            self.assertEqual(ik.kinematic_model.nq,7);self.assertEqual(ik.kinematic_model.ngeom,0);self.assertLess(ik.kinematic_model.nbody,13)
            for _ in range(6):
                q=rng.uniform(*ik.limits.T);d.qpos[ik.qids]=q;mujoco.mj_kinematics(m,d);mujoco.mj_comPos(m,d);p,r=ik.pose(q)
                np.testing.assert_allclose(p,d.site_xpos[ik.site],atol=1e-12);np.testing.assert_allclose(r,d.site_xmat[ik.site].reshape(3,3),atol=1e-12)
                jp=np.zeros((3,m.nv));jr=jp.copy();mujoco.mj_jacSite(m,d,jp,jr,ik.site)
                mujoco.mj_jacSite(ik.kinematic_model,ik.data,ik.jp,ik.jr,ik.local_site)
                np.testing.assert_allclose(ik.jp[:,ik.local_dofs],jp[:,ik.dofs],atol=1e-12);np.testing.assert_allclose(ik.jr[:,ik.local_dofs],jr[:,ik.dofs],atol=1e-12)

if __name__=='__main__':unittest.main()
