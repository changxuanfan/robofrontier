import unittest
from scripts.execute_mosaic_repair_route import choose


class RepairRouteTests(unittest.TestCase):
    def rows(self):
        return [dict(robots=['C','A'],position_m=[0,0,0],angled_grasp={'C':15}),
            dict(robots=['C','B'],position_m=[1,0,0],angled_grasp=None),
            dict(robots=['B','A'],position_m=[2,0,0],angled_grasp=None)]
    def test_use_a_reachable_alternative_receiver_instead_of_an_incompatible_shortcut(self):
        self.assertEqual(choose(self.rows(),'C',['A','B'],['A','B','C'])[0],['C','B'])
    def test_use_intermediate_robot_when_only_the_original_receiver_reaches_the_goal(self):
        self.assertEqual(choose(self.rows(),'C',['A'],['A','B','C'])[0],['C','B','A'])
    def test_occupied_port_is_not_silently_reused(self):
        with self.assertRaises(ValueError):choose(self.rows(),'C',['A'],['A','B','C'],[frozenset(('B','C'))])


if __name__=='__main__':unittest.main()
