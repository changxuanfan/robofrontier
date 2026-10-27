"""A lost micro-lift stops before a longer carry; nominal grip flags do not suffice."""
from types import SimpleNamespace
import unittest
import numpy as np

from robofrontier.suite.transport_grasp_safety import PickupSupportLost, guarded_pickup_lift


class GuardedPickupTests(unittest.TestCase):
    def world(self,follow=True,supported=True):
        body=SimpleNamespace(xpos=np.array([0.,0.,.83]))
        site=np.array([[0.,0.,.85]])
        t=SimpleNamespace(data=SimpleNamespace(site_xpos=site,body=lambda _:body),
            arms={'A':SimpleNamespace(site=0,grasp_force=50.,grasp_confirmed=True)},
            monitor=SimpleNamespace(geoms={'part':{1}},bilateral=lambda *_args,**_kw:{'A'} if supported else set()),
            wait=lambda _:None,moves=[])
        def move(r,target,rotation,label,**kw):
            t.moves.append((label,np.asarray(target).copy()))
            if follow:body.xpos[2]+=target[2]-site[0,2]
            site[0]=target
        t.move=move
        return t

    def test_geometry_contact_flag_without_following_load_stops_at_probe(self):
        t=self.world(follow=False,supported=True)
        with self.assertRaises(PickupSupportLost):
            guarded_pickup_lift(t,'A','part',.83,np.eye(3))
        self.assertEqual(len(t.moves),1)
        self.assertAlmostEqual(t.moves[0][1][2],.87)

    def test_height_motion_without_positive_force_support_stops_at_probe(self):
        t=self.world(follow=True,supported=False)
        with self.assertRaises(PickupSupportLost):
            guarded_pickup_lift(t,'A','part',.83,np.eye(3))
        self.assertEqual(len(t.moves),1)

    def test_verified_probe_continues_to_original_total_lift_height(self):
        t=self.world()
        guarded_pickup_lift(t,'A','part',.83,np.eye(3))
        self.assertEqual(len(t.moves),2)
        self.assertAlmostEqual(t.data.body('part').xpos[2],.97)
