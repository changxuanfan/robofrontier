import unittest
from types import SimpleNamespace

import mujoco

from robofrontier.suite.relay_user_goal_policy import eligible_goal_cargo,final_support


class UserGoalPolicyTests(unittest.TestCase):
    def trial(self,outside):
        model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody><body>
          <freejoint/><geom type="box" size=".025 .025 .025"/>
        </body></worldbody></mujoco>''')
        data=mujoco.MjData(model);data.qpos[:3]=[.4+outside-.025,0,.85]
        mujoco.mj_forward(model,data)
        relay=SimpleNamespace(states={'cargo':{'all_geoms':[0],'rod_geoms':[0]}},
            supported_payloads=lambda **kwargs:set())
        monitor=SimpleNamespace(relay_v2=relay,bilateral=lambda *args,**kw:{'A','B'},
            bottom=lambda geoms:.825)
        return SimpleNamespace(model=model,data=data,monitor=monitor,
            task={'table_height_m':.8,'completion':{'allow_goal_payload_contact':True},
                'route':[{'center_m':[0,0,.8],'nominal_yaw_rad':0,'half_size_m':[.4,.68]}]})

    def test_accepted_edge_cargo_remains_a_contact_candidate(self):
        self.assertEqual(eligible_goal_cargo(self.trial(.049)),['cargo'])

    def test_beyond_user_tolerance_is_excluded(self):
        self.assertEqual(eligible_goal_cargo(self.trial(.051)),[])

    def test_low_release_does_not_require_exact_table_contact(self):
        self.assertTrue(final_support(self.trial(.049),'cargo',['A','B']))

    def test_release_still_requires_actual_declared_holders(self):
        self.assertFalse(final_support(self.trial(.049),'cargo',['A','C']))


if __name__=='__main__':unittest.main()
