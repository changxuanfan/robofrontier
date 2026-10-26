"""The open-contact state must allow separation without treating gaps as hits."""
import unittest
from types import SimpleNamespace
import numpy as np

from robofrontier.suite.relay_contact_escape import target_contact_allowed,plan_open_contact_withdrawal


class ContactEscapeTests(unittest.TestCase):
    def test_existing_compliant_contact_can_separate(self):
        for distance in [-.00035,-.0002,0.,.0008,.004]:
            with self.subTest(distance=distance):
                self.assertTrue(target_contact_allowed(distance,initial_depth=-.0003))


    def test_existing_contact_cannot_be_compressed_further(self):
        self.assertFalse(target_contact_allowed(-.0005,initial_depth=-.0003))


    def test_positive_proximity_is_not_new_penetration(self):
        self.assertTrue(target_contact_allowed(.0008))
        self.assertFalse(target_contact_allowed(-.0002))

    def unsupported_trial(self,support,force=None):
        arm=SimpleNamespace(job=None,grasp_force=force,observe=lambda:{'gripper_width_m':.08})
        relay=SimpleNamespace(states={'cargo':{'all_geoms':set(),'rod_geoms':set()}},
            inside=lambda _:False,environment_contact=lambda _:False,
            supported_payloads=lambda **kwargs:set())
        return SimpleNamespace(arms={'P':arm},held={'P':'cargo'},
            task={'mechanism':{'cross_section_m':[.045,.045]}},
            monitor=SimpleNamespace(relay_v2=relay,bilateral=lambda *args,**kwargs:set(support)))

    def test_open_donor_does_not_count_itself_as_an_independent_receiver(self):
        with self.assertRaisesRegex(ValueError,'two actual receivers'):
            plan_open_contact_withdrawal(self.unsupported_trial({'P','I'}),'cargo','P',[0.]*7)

    def test_contact_escape_never_ends_a_force_hold_implicitly(self):
        with self.assertRaisesRegex(ValueError,'force-free'):
            plan_open_contact_withdrawal(self.unsupported_trial({'I','O'},force=70.),'cargo','P',[0.]*7)

    def test_two_receivers_do_not_allow_an_unbounded_contact_rotation(self):
        t=self.unsupported_trial({'I','O'});arm=t.arms['P']
        arm.qids=np.arange(7);arm.site=0
        arm.ik=SimpleNamespace(limits=np.tile([-3.,3.],(7,1)),
            pose=lambda q:(np.zeros(3),np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])))
        t.data=SimpleNamespace(qpos=np.zeros(7),site_xpos=np.zeros((1,3)),site_xmat=np.eye(3).reshape(1,9))
        with self.assertRaisesRegex(ValueError,'short supported withdrawal'):
            plan_open_contact_withdrawal(t,'cargo','P',[0.]*7)


if __name__=='__main__':unittest.main()
