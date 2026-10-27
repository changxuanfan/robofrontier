from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
import numpy as np

from robofrontier.suite import transport_oracle_v2 as ref


class ReleaseUnloadingTests(unittest.TestCase):
    def fixture(self,uncertain=False,supported=True):
        state=dict(contact=True)
        t=SimpleNamespace(cfg=dict(allowed_grasp_force_n=[30.,70.]),task=dict(cube_size_m=.06),
            arms={'F':SimpleNamespace(grasp_force=70.),'G':SimpleNamespace(grasp_force=None,grip_max=.08)},
            data=SimpleNamespace(body=lambda _:SimpleNamespace(xpos=np.array([0.,0.,1.]))),
            monitor=SimpleNamespace(geoms={'part':[1]},bilateral=lambda *_,**__:({'F'} if supported else set())),
            wait=Mock(),event=Mock())
        def execute(label,commands,allow_failure=False):
            if 'F' in commands:t.arms['F'].grasp_force=commands['F'][1]['force_n'];return {'F':dict(status='completed')}
            if uncertain:return {'G':dict(status='uncertain')}
            state['contact']=False;return {'G':dict(status='completed')}
        t.execute=Mock(side_effect=execute)
        return t,state

    def test_receiver_pressure_relieves_preload_within_frozen_limits(self):
        t,state=self.fixture()
        with patch.object(ref,'_robot_payload_contacts',side_effect=lambda *_:state['contact']):
            self.assertEqual(ref.unload_receiver_for_donor_release(t,'G','F','part'),70.)
        self.assertEqual(t.arms['F'].grasp_force,50.)
        self.assertEqual(t.execute.call_args_list[0].args[1]['F'][1]['force_n'],50.)
        self.assertFalse(state['contact'])

    def test_unknown_opening_is_not_retried_at_another_pressure(self):
        t,state=self.fixture(uncertain=True)
        with patch.object(ref,'_robot_payload_contacts',side_effect=lambda *_:state['contact']):
            with self.assertRaisesRegex(RuntimeError,'unresolved'):
                ref.unload_receiver_for_donor_release(t,'G','F','part')
        self.assertEqual(t.execute.call_count,2)

    def test_no_support_means_no_pressure_change(self):
        t,_=self.fixture(supported=False)
        with self.assertRaisesRegex(RuntimeError,'unverified'):
            ref.unload_receiver_for_donor_release(t,'G','F','part')
        t.execute.assert_not_called()


if __name__=='__main__':unittest.main()
