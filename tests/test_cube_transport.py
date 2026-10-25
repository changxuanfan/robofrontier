"""Offline asset contracts; no agents, dispatcher or action server."""
from collections import Counter
import copy
import json
import unittest

import mujoco
import numpy as np

from robofrontier.cube_transport import (ROOT, COLLECTION, SMALL_ORDERS, layout, make_task,
                                        evaluate_placement_window, match_placement_sample, reference_jobs,
                                        replay_reference, route)
from robofrontier.cube_transport_presentation import project_point


class CubeTransportTests(unittest.TestCase):
    def test_inventory_and_topology_all_sizes(self):
        for n in range(3, 13):
            env = layout(n)
            self.assertEqual(len(env['stations']), n)
            self.assertEqual(len(env['objects']), 3*n)
            self.assertEqual(set(Counter(o['source'] for o in env['objects']).values()), {3})
            self.assertEqual(len(env['edges']), n if n > 3 else n-1)
            self.assertLessEqual(len({o['color'] for o in env['objects']}), 6)
            self.assertEqual(sum(o['shape'] == 'cylinder' for o in env['objects']), 3*max(0, n-6))
            self.assertTrue(all('pattern' not in o for o in env['objects']))
            adjacency = {frozenset(e['stations']) for e in env['edges']}
            for a in env['stations']:
                for b in env['stations']:
                    r = route(a['id'], b['id'], n, env['topology'])
                    self.assertEqual((r[0], r[-1]), (a['id'], b['id']))
                    self.assertEqual(len(r), len(set(r)))
                    self.assertTrue(all(frozenset(pair) in adjacency for pair in zip(r, r[1:])))

    def test_saved_tasks_and_schedules_all_sizes_and_seeds(self):
        for n in range(3, 13):
            env = layout(n)
            for seed in (0, 1, 2):
                task = json.loads((ROOT/f"configs/tasks/{COLLECTION}/{env['id']}/seed-{seed}.json").read_text())
                self.assertTrue(all(len(orders) == 3 for orders in task['orders'].values()))
                assigned = [name for name, _ in reference_jobs(env, task)]
                self.assertCountEqual(assigned, [o['id'] for o in env['objects']])
                for goals in task['orders'].values():
                    for goal in goals:
                        self.assertEqual(set(goal['requires']), {'color', 'shape'})
                        self.assertNotIn('object_id', goal)
                counts = task['distance_distribution']['hop_counts']
                if n > 6:
                    self.assertEqual(sum(v*v for v in counts.values()), task['distance_distribution']['squared_count_lower_bound'])
                if n <= 6 and seed == 0:
                    kinds = {(o['color'], o['shape']): o['type_code'] for o in env['objects']}
                    codes = [''.join(kinds[(g['requires']['color'], g['requires']['shape'])] for g in goals)
                             for goals in task['orders'].values()]
                    self.assertEqual(codes, SMALL_ORDERS[n])
                result = json.loads((ROOT/f"benchmarks/{COLLECTION}/references/{task['id']}.json").read_text())
                replay_reference(env, task, result)
                self.assertEqual(sum(result['arm_phase_load'].values()), result['serial_phase_slots'])
                self.assertEqual(sum(map(len, result['schedule'])), result['serial_phase_slots'])
                self.assertLessEqual(result['lower_bound_phase_slots_for_fixed_routes'], result['parallel_feasible_phase_slots'])
                self.assertLess(result['parallel_feasible_phase_slots'], result['serial_phase_slots'])
                self.assertEqual(result['optimality_proven'], result['lower_bound_phase_slots_for_fixed_routes'] == result['parallel_feasible_phase_slots'])

    def test_cylinder_collision_mass_origin_and_unchanged_orders(self):
        env = layout(12)
        model = mujoco.MjModel.from_xml_path(str(ROOT/f"configs/environments/{COLLECTION}/{env['id']}/scene.xml"))
        for obj in env['objects'][18:]:
            body = model.body(obj['id'])
            geom = model.geom(obj['id']+'_geom')
            self.assertEqual(geom.type[0], mujoco.mjtGeom.mjGEOM_CYLINDER)
            self.assertAlmostEqual(body.mass[0], .05)
            # Uniform cylinder COM coincides with the body origin.
            np.testing.assert_allclose(body.ipos, [0, 0, 0], atol=1e-9)
            np.testing.assert_allclose(sorted(body.inertia), sorted([.05*(3*.02**2+.04**2)/12]*2+[.05*.02**2/2]), rtol=1e-6)
            self.assertTrue(np.all(body.inertia > 0))
        for n in range(3, 13):
            current = layout(n)
            old = json.loads((ROOT/f"configs/tasks/cube_transport/{current['id']}/seed-0.json").read_text())
            new = make_task(current, 0)
            self.assertEqual(old['distance_distribution'], json.loads(json.dumps(new['distance_distribution'])))
            for arm, goals in new['orders'].items():
                for before, after in zip(old['orders'][arm], goals):
                    self.assertEqual(before['position_m'], after['position_m'])
                    self.assertEqual(before['requires']['color'], after['requires']['color'])
                    self.assertEqual(after['requires']['shape'], 'cylinder' if before['requires']['shape'] == 'sphere' else 'cube')

    def test_seed_reproducibility(self):
        env = layout(8)
        self.assertEqual(make_task(env, 2), make_task(env, 2))
        self.assertNotEqual(make_task(env, 1)['orders'], make_task(env, 2)['orders'])

    def test_schedule_replay_rejects_double_booking(self):
        env = layout(4)
        task = json.loads((ROOT/f'configs/tasks/{COLLECTION}/ring-04/seed-0.json').read_text())
        result = json.loads((ROOT/f"benchmarks/{COLLECTION}/references/{task['id']}.json").read_text())
        broken = copy.deepcopy(result)
        broken['schedule'][0].append(copy.deepcopy(broken['schedule'][0][0]))
        with self.assertRaises(AssertionError):
            replay_reference(env, task, broken)

    def test_loadable_reset_keyframe_at_both_scale_endpoints(self):
        for n in (3, 12):
            env = layout(n)
            model = mujoco.MjModel.from_xml_path(str(ROOT/f"configs/environments/{COLLECTION}/{env['id']}/scene.xml"))
            self.assertEqual(model.nu, 8*n)
            self.assertEqual(model.nq, 9*n+7*3*n)
            data = mujoco.MjData(model)
            mujoco.mj_resetDataKeyframe(model, data, model.key('initial').id)
            mujoco.mj_forward(model, data)
            for station in env['stations']:
                np.testing.assert_allclose(data.site(station['id']+'_tcp').xpos, station['home_tcp_m'], atol=.002)
            for obj in env['objects']:
                np.testing.assert_allclose(data.body(obj['id']).xpos, obj['initial_position_m'], atol=1e-9)
            if n == 12:
                camera = model.camera('top').id
                center = project_point(model, data, camera, [0, 0, .4], 1280, 900)
                east = project_point(model, data, camera, [.1, 0, .4], 1280, 900)
                north = project_point(model, data, camera, [0, .1, .4], 1280, 900)
                self.assertGreater(east[0], center[0])
                self.assertLess(north[1], center[1])

    def test_semantic_evaluator_accepts_swaps_and_rejects_held_or_wrong_type(self):
        env = layout(3); task = make_task(env, 0)
        states = {}
        pools = {}
        for obj in env['objects']:
            pools.setdefault((obj['color'], obj['shape']), []).append(obj['id'])
        for goals in task['orders'].values():
            for goal in goals:
                name = pools[tuple(goal['requires'][k] for k in ('color', 'shape'))].pop(0)
                states[name] = {'position_m': goal['position_m'], 'linear_velocity_m_s': [0, 0, 0],
                                'table_supported': True, 'released': True}
        self.assertEqual(match_placement_sample(env, task, states)['matched_goals'], 9)
        states['A_item1'], states['A_item3'] = states['A_item3'], states['A_item1']
        self.assertEqual(match_placement_sample(env, task, states)['matched_goals'], 9)
        states['A_item1']['released'] = False
        self.assertEqual(match_placement_sample(env, task, states)['matched_goals'], 8)
        states['A_item1']['released'] = True
        states['A_item1'], states['B_item1'] = states['B_item1'], states['A_item1']
        self.assertEqual(match_placement_sample(env, task, states)['matched_goals'], 7)

    def test_semantic_evaluator_never_counts_one_object_twice(self):
        env = layout(3); task = make_task(env, 0)
        goal = task['orders']['A'][0]
        duplicate = copy.deepcopy(goal); duplicate['goal_id'] = 'second_goal'
        task['orders'] = {'A': [goal, duplicate]}
        states = {'A_item2': {'position_m': goal['position_m'], 'linear_velocity_m_s': [0, 0, 0],
                              'table_supported': True, 'released': True}}
        self.assertEqual(match_placement_sample(env, task, states)['matched_goals'], 1)

    def test_stable_window_requires_duration_density_and_continuous_placement(self):
        env = layout(3); task = make_task(env, 0)
        goal = task['orders']['A'][0]; task['orders'] = {'A': [goal]}
        states = {'A_item2': {'position_m': goal['position_m'], 'linear_velocity_m_s': [0, 0, 0],
                              'table_supported': True, 'released': True}}
        samples = [{'time_s': i*.02, 'objects': copy.deepcopy(states)} for i in range(51)]
        self.assertTrue(evaluate_placement_window(env, task, samples)['complete'])
        self.assertFalse(evaluate_placement_window(env, task, samples[1:])['complete'])
        self.assertFalse(evaluate_placement_window(env, task, samples[::2])['complete'])
        samples[25]['objects']['A_item2']['released'] = False
        self.assertFalse(evaluate_placement_window(env, task, samples)['complete'])
        with self.assertRaises(ValueError):
            evaluate_placement_window(env, task, list(reversed(samples)))

    def test_public_robot_labels_and_camera_metadata(self):
        for n in range(3, 13):
            env = layout(n)
            context = json.loads((ROOT/f"configs/environments/{COLLECTION}/{env['id']}/startup.json").read_text())
            self.assertEqual(len(context['cameras']), 2*n+3)
            expected = {s['id'] for s in env['stations']}
            self.assertEqual({s['robot_id'] for s in context['robots']}, expected)
            for view in ('top', 'global'):
                meta = json.loads((ROOT/f"benchmarks/{COLLECTION}/previews/{env['id']}-{view}.json").read_text())
                self.assertEqual({s['robot_id'] for s in meta['robot_labels']}, expected)
                self.assertFalse(any(s['overlap_area_pixel2'] for s in meta['robot_labels']))
                left, top, right, bottom = meta['scene_viewport_pixel']
                self.assertEqual(meta['render_size_pixel'], [1280, 900])
                for label in meta['robot_labels']:
                    x1, y1, x2, y2 = label['label_box_pixel']
                    self.assertTrue(x2 <= left or x1 >= right or y2 <= top or y1 >= bottom)
                self.assertIn('not live', meta['frame_scope'])


if __name__ == '__main__':
    unittest.main()
