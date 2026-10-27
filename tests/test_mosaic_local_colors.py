"""Colour design invariants, distinct-cell capacity and real baseline evidence."""
from copy import deepcopy
import unittest

from robofrontier.suite.mosaic_local_colors import nearest,choose_slots,colour_assignment,validate_plan
from robofrontier.suite.mosaic_policy_package import ROOT,read


class LocalColourTests(unittest.TestCase):
    def task(self):
        return dict(arms={'A':dict(base_position_m=[-.3,-.4,.8]),'B':dict(base_position_m=[.3,-.4,.8])},
            objects=[dict(id=f'{r}{i}',color='red',initial_position_m=[x,-.5,.83]) for r,x in [('A',-.3),('B',.3)] for i in range(4)],
            goals=[dict(goal_id=f'{r}{i}',requires=dict(color=c),position_m=[x,-.1,.83]) for r,x,c in [('A',-.3,'green'),('B',.3,'red')] for i in range(4)])

    def test_nearest_is_spatial_not_source_label(self):
        task=self.task()
        self.assertEqual(nearest(task,[-.29,-.5,.83]),'A')
        self.assertEqual(nearest(task,[.29,-.5,.83]),'B')
        self.assertEqual(nearest(task,[0,-.5,.83]),'A')

    def test_distinct_cells_and_global_colours_are_conserved(self):
        task=self.task();pickups={o['id']:True for o in task['objects']}
        states={(r,g['goal_id']):nearest(task,g['position_m'])==r for r in task['arms'] for g in task['goals']}
        slots=choose_slots(task,pickups,states,.70)
        assignments=colour_assignment(task,slots,pickups)
        self.assertEqual(len({a['object_id'] for a in assignments}),8)
        self.assertEqual(len({a['goal_id'] for a in assignments}),8)
        self.assertEqual(sum(a['colour']=='green' for a in assignments),4)
        for robot in task['arms']:self.assertEqual(sum(a['local'] for a in assignments if a['robot']==robot),3)

    def test_bad_geometry_cannot_count_toward_local_quota(self):
        task=self.task();pickups={o['id']:False for o in task['objects']}
        with self.assertRaisesRegex(ValueError,'pickups'):choose_slots(task,pickups,{},.70)

    def test_insufficient_target_capacity_is_rejected(self):
        task=self.task();pickups={o['id']:True for o in task['objects']}
        states={(r,g['goal_id']):False for r in task['arms'] for g in task['goals']}
        with self.assertRaisesRegex(ValueError,'allocation'):choose_slots(task,pickups,states,.70)

    def test_selected_stock_without_pickup_evidence_is_cross_zone_only(self):
        task=self.task();pickups={o['id']:True for o in task['objects']};pickups['A0']=False
        states={(r,g['goal_id']):nearest(task,g['position_m'])==r for r in task['arms'] for g in task['goals']}
        assignments=colour_assignment(task,choose_slots(task,pickups,states),pickups)
        self.assertFalse(next(a for a in assignments if a['object_id']=='A0')['local'])

    def test_actual_nine_plans_and_tamper_rejection(self):
        engineering=ROOT/'runs/engineering/mosaic-local-colours-r14-20261027'
        if not (engineering/'a04-plan.json').exists():self.skipTest('Local engineering fixtures unavailable')
        for n,rc in [(4,'6x12'),(6,'9x18'),(8,'12x24')]:
            plan=read(engineering/f'a{n:02d}-plan.json')
            for level in (1,2,3):
                task=read(ROOT/f'configs/environments/mosaic/frontier-v2-r13-mosaic-a{n:02d}-{rc}-d{level}/task.json')
                validate_plan(task,plan)
                for row in plan['per_robot']:self.assertGreaterEqual(row['certified_local_fraction'],.60)
            bad=deepcopy(plan);bad['assignments'][0]['goal_id']=bad['assignments'][1]['goal_id']
            with self.assertRaisesRegex(ValueError,'every stock'):validate_plan(task,bad)
            bad=deepcopy(plan);entry=next(a for a in bad['assignments'] if a['local']);bad['nearest_pickup_confirmed'][entry['object_id']]=False
            with self.assertRaisesRegex(ValueError,'geometric evidence'):validate_plan(task,bad)


if __name__=='__main__':unittest.main()
