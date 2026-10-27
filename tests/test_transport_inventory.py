from types import SimpleNamespace
import unittest
import numpy as np

from robofrontier.suite.transport_inventory import unreachable_required_inventory


class InventoryTests(unittest.TestCase):
    def world(self,positions,accepted=None):
        return SimpleNamespace(task=dict(workspace_world_m=dict(lower=[-2.,-2.,.425],upper=[2.,2.,2.1])),
            env=dict(objects=[dict(id=n) for n in positions]),
            data=SimpleNamespace(body=lambda n:SimpleNamespace(xpos=np.asarray(positions[n]))),
            monitor=SimpleNamespace(result=lambda:dict(assignments=accepted or {})))

    def test_required_stock_on_floor_is_not_a_recoverable_robot_target(self):
        t=self.world(dict(on_table=[.2,.4,.83],lost=[44.,-426.,.03]))
        self.assertEqual([r['object'] for r in unreachable_required_inventory(t)],['lost'])

    def test_padding_does_not_reject_a_near_workspace_grasp_offset(self):
        self.assertFalse(unreachable_required_inventory(self.world(dict(near=[2.05,0.,.40]))))

    def test_only_remaining_inventory_is_reported(self):
        self.assertFalse(unreachable_required_inventory(self.world(dict(done=[44.,0.,.03]),dict(slot='done'))))


if __name__=='__main__':unittest.main()
