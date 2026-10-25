import copy
import json
from pathlib import Path
import tempfile
import unittest

from check_transport_schedule import solve, replay
from robofrontier.task_config import ROOT, load_task


class TransportScheduleTests(unittest.TestCase):
    def test_exact_schedule_reaches_arm_workload_lower_bound_with_one_buffer(self):
        for capacity in (1,2):
            result=solve(capacity)
            self.assertEqual(result['serial_phase_slots'],38)
            self.assertEqual(result['optimal_phase_slots'],16)
            self.assertEqual(result['optimal_phase_slots'],result['arm_load_lower_bound'])
            self.assertTrue(replay(result['schedule'],capacity))
            broken=copy.deepcopy(result['schedule']);broken[0]=['A_local','A_to_B']
            with self.assertRaises(ValueError):replay(broken,capacity)

    def test_nine_order_inventory_and_source_constraints(self):
        task=load_task(ROOT/'configs/tasks/three-arm-nine-orders-v1.json')
        self.assertEqual(len(task['objects']),9)
        self.assertTrue(all(len(g)==3 for g in task['goals'].values()))
        self.assertEqual([g['requires']['region'] for g in task['goals']['A']],['A','C','C'])
        task['goals']['A'][0]['requires']={'color':'red','region':'C'}
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'bad.json';p.write_text(json.dumps(task))
            with self.assertRaisesRegex(ValueError,'inventory'):load_task(p)
