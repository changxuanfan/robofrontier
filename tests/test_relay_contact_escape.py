"""The open-contact state must allow separation without treating gaps as hits."""
import unittest

from robofrontier.suite.relay_contact_escape import target_contact_allowed


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


if __name__=='__main__':unittest.main()
