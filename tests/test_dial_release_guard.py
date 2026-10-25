"""A failed aperture must not be followed by a captured-rotor withdrawal."""
from types import SimpleNamespace
import unittest
from robofrontier.suite.dial_oracle import Manipulator


class DialReleaseGuardTests(unittest.TestCase):
    def test_failed_wider_opening_stops_before_any_withdrawal(self):
        calls=[]
        def execute(label,commands,allow_failure=False):
            calls.append((label,commands))
            return {'E':dict(status='failed',reason='Requested opening not reached')}
        trial=SimpleNamespace(task={'level':3},execute=execute,arms={'E':SimpleNamespace(grasp_force=None,grip_max=.08)})
        m=Manipulator.__new__(Manipulator);m.t=trial;m.grips={'D03':dict(robot='E')}
        with self.assertRaisesRegex(RuntimeError,'Cannot safely withdraw'):
            m.release([dict(id='D03')])
        self.assertEqual([x[0] for x in calls],['red-release-open','red-release-wide-retry'])
        self.assertEqual(calls[1][1]['E'][1]['width_m'],.08)


if __name__=='__main__':unittest.main()
