"""Offline cube-transport assets. Does not import or change the live harness.

Frames use local x toward the work surface and local y to the robot's left.
The scheduling reference is deliberately discrete, not a motion certificate.
"""
from __future__ import annotations

import copy
from collections import Counter
import math
import os
from pathlib import Path
import random
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .scene import ROOT, vector
from .sim import RobotIK

COLLECTION = 'cube_transport_cylinders'

COUNTS = {4: (1, 1, 1, 1), 5: (2, 1, 1, 1), 6: (2, 1, 2, 1),
          7: (2, 2, 2, 1), 8: (2, 2, 2, 2), 9: (3, 2, 2, 2),
          10: (3, 2, 3, 2), 11: (3, 3, 3, 2), 12: (3, 3, 3, 3)}
PALETTE = [('red', (.85, .13, .13)), ('blue', (.12, .36, .85)),
           ('green', (.12, .62, .25)), ('orange', (.95, .43, .08)),
           ('purple', (.52, .23, .76)), ('cyan', (.05, .68, .79))]
# Agreed seed-0 orders, frozen for 3..6 arms. Letters denote visible types.
SMALL_ORDERS = {
    3: ['acc', 'bac', 'abb'],
    4: ['bcc', 'bdd', 'aad', 'abc'],
    5: ['bde', 'bdd', 'aae', 'cce', 'abc'],
    6: ['cde', 'aac', 'bff', 'bde', 'abd', 'cef'],
}


def balanced_destinations(n, seed):
    """Balance the global hop histogram; supply/demand each stay exactly three.

    The achieved sum of squared counts meets an integer lower bound. On an
    even ring the number of odd-hop demands is even (equal partition stock).
    Local per-station mixes vary; they are a secondary diversity criterion.
    """
    rng = random.Random(20261021+1000*seed+n)
    k, m = n//2+1, 3*n
    distance = [[min((a-b) % n, (b-a) % n) for b in range(n)] for a in range(n)]
    ideal = [m//k+(i < m % k) for i in range(k)]
    bound = sum(x*x for x in ideal)
    if n % 2 == 0 and len(set(ideal)) == 1 and sum(ideal[1::2]) % 2:
        bound += 2
    best = None
    for _ in range(32):
        destinations = list(range(n))*3
        rng.shuffle(destinations)
        hist = Counter(distance[i//3][d] for i, d in enumerate(destinations))
        energy = sum(hist[h]**2 for h in range(k))
        for step in range(12000):
            if energy == bound:
                break
            i, j = rng.sample(range(m), 2)
            if i//3 == j//3 or destinations[i] == destinations[j]:
                continue
            delta = Counter((distance[i//3][destinations[j]], distance[j//3][destinations[i]]))
            delta.subtract((distance[i//3][destinations[i]], distance[j//3][destinations[j]]))
            change = sum(2*hist[h]*v+v*v for h, v in delta.items())
            temp = max(.12, 3*(1-step/12000))
            if change <= 0 or rng.random() < math.exp(-change/temp):
                destinations[i], destinations[j] = destinations[j], destinations[i]
                hist.update(delta); energy += change
        signatures = [tuple(sorted(distance[i//3][d] for i, d in enumerate(destinations) if d == a)) for a in range(n)]
        rank = (energy, -len(set(signatures)), sum(s == (0, 0, 0) for s in signatures))
        if best is None or rank < best[0]:
            best = rank, destinations[:]
    if best[0][0] != bound:
        raise RuntimeError('Balance search did not reach the histogram lower bound')
    return best[1], bound


def xyz(station, depth, lateral, z):
    yaw = station['yaw_rad']
    c, s = math.cos(yaw), math.sin(yaw)
    b = station['base_position_m']
    return [round(b[0]+c*depth-s*lateral, 8),
            round(b[1]+s*depth+c*lateral, 8), z]


def layout(count):
    if count not in range(3, 13):
        raise ValueError('Supported arm count: 3..12')
    height = .4
    stations = []
    if count == 3:
        poses = [(0, (i-1)*.72, 0, 'line') for i in range(3)]
        tables = [{'center_m': [.2, 0, height-.025], 'half_size_m': [.43, 1.15, .025]}]
        sides = None
    else:
        sides = COUNTS[count]  # south, east, north, west; increasing station IDs.
        hx = .65 + .36*(max(sides[0], sides[2])-1)
        hy = .65 + .36*(max(sides[1], sides[3])-1)
        poses = []
        for side, k in enumerate(sides):
            for j in range(k):
                t = (j-(k-1)/2)*.72
                poses.append(((t, -hy, math.pi/2, 'south'),
                              (hx, t, math.pi, 'east'),
                              (-t, hy, -math.pi/2, 'north'),
                              (-hx, -t, 0, 'west'))[side])
        # Four non-overlapping tabletop strips leave a genuine central opening.
        ix, iy, ox, oy = hx-.56, hy-.56, hx+.20, hy+.20
        tables = [
            {'center_m': [0, -(oy+iy)/2, height-.025], 'half_size_m': [ox, (oy-iy)/2, .025]},
            {'center_m': [0, (oy+iy)/2, height-.025], 'half_size_m': [ox, (oy-iy)/2, .025]},
            {'center_m': [-(ox+ix)/2, 0, height-.025], 'half_size_m': [(ox-ix)/2, iy, .025]},
            {'center_m': [(ox+ix)/2, 0, height-.025], 'half_size_m': [(ox-ix)/2, iy, .025]},
        ]
    for i, (x, y, yaw, side) in enumerate(poses):
        station = {'id': chr(65+i), 'side': side, 'base_position_m': [x, y, height],
                   'yaw_rad': yaw}
        station.update(home_tcp_m=xyz(station, .34, 0, .70),
                       source_positions_m=[xyz(station, .30, v, .42) for v in (-.105, 0, .105)],
                       goal_positions_m=[xyz(station, .42, v, .42) for v in (-.105, 0, .105)],
                       parking_positions_m=[xyz(station, .32, v, .42) for v in (-.23, .23)])
        stations.append(station)
    edges = []
    for i in range(count if count > 3 else count-1):
        a, b = stations[i], stations[(i+1) % count]
        p = (np.array(xyz(a, .34, 0, .42))+xyz(b, .34, 0, .42))/2
        edges.append({'id': a['id']+'_'+b['id'], 'stations': [a['id'], b['id']],
                      'position_m': p.tolist(), 'capacity': 1})
    objects = []
    for i, station in enumerate(stations):
        color, rgb = PALETTE[i % 6]
        for j in range(3):
            objects.append({'id': f"{station['id']}_item{j+1}", 'source': station['id'],
                            'color': color, 'rgb': list(rgb), 'shape': 'cube' if i < 6 else 'cylinder',
                            'type_code': station['id'].lower(),
                            'initial_position_m': station['source_positions_m'][j]})
    return {'schema': 'robofrontier.cube_transport.environment.v4', 'rules_version': 4,
            'id': f"{'line' if count == 3 else 'ring'}-{count:02d}",
            'collection': COLLECTION, 'arm_count': count, 'topology': 'line' if count == 3 else 'ring',
            'side_counts_south_east_north_west': sides,
            'physics': {'table_height_m': height, 'cube_size_m': .04, 'cylinder_radius_m': .02, 'cylinder_height_m': .04,
                        'object_mass_kg': .05,
                        'contact_friction': [1, .005, .0001], 'noslip_iterations': 10,
                        'timestep_s': .002, 'gravity_m_s2': [0, 0, -9.81]},
            'appearance': 'light', 'background_palette': 'pale_blue_ground_and_sky',
            'floor_rgba': [.72, .86, .98, 1], 'table_finish': 'matte_oak',
            'table_rgba': [.76, .51, .29, 1], 'stations': stations, 'tables': tables,
            'edges': edges, 'objects': objects,
            'observation_contract': {'initial_positions_public': True,
                                     'runtime': ['RGB', 'robot_proprioception', 'action_status'],
                                     'live_object_truth_for_policy': False}}


def route(source, destination, count, topology):
    a, b = ord(source)-65, ord(destination)-65
    if topology == 'line':
        step = 1 if b >= a else -1
        return [chr(65+i) for i in range(a, b+step, step)]
    forward = (b-a) % count
    step = 1 if forward <= count-forward else -1
    return [chr(65+(a+step*i) % count) for i in range(min(forward, count-forward)+1)]


def make_task(env, seed):
    n = env['arm_count']
    ids = [s['id'] for s in env['stations']]
    bound = None
    if n <= 6:
        codes = [list(row) for row in SMALL_ORDERS[n]]
        if seed:
            # Preserve the agreed seed-0 task and its distance mix. Other order
            # seeds reflect the graph and change slot order, not object poses.
            remap = (lambda i: n-1-i) if seed == 1 or n == 3 else (lambda i: (i+1) % n)
            changed = [None]*n
            for a, row in enumerate(codes):
                changed[remap(a)] = [chr(97+remap(ord(code)-97)) for code in row]
            codes = changed
            for row in codes:
                random.Random(seed).shuffle(row)
    else:
        destinations, bound = balanced_destinations(n, seed)
        codes = [[chr(97+i//3) for i, d in enumerate(destinations) if d == a] for a in range(n)]
    orders = {arm: [] for arm in ids}
    for a, row in enumerate(codes):
        arm = ids[a]
        for slot, code in enumerate(row):
            kind = env['objects'][3*(ord(code)-97)]
            orders[arm].append({'goal_id': f'{arm}_slot{slot+1}', 'slot': slot+1,
                                'requires': {k: kind[k] for k in ('color', 'shape')},
                                'position_m': env['stations'][a]['goal_positions_m'][slot]})
    histogram = Counter(len(route(chr(ord(c)-32), ids[a], n, env['topology']))-1 for a, row in enumerate(codes) for c in row)
    return {'schema': 'robofrontier.cube_transport.task.v4', 'rules_version': 4,
            'id': f"{env['id']}-v4-seed{seed}", 'environment': env['id'],
            'seed': seed, 'seed_scope': 'orders only; initial object poses are fixed',
            'goal': 'Place an object of the requested color and shape in each numbered slot.',
            'orders': orders, 'identical_types_interchangeable': True,
            'slot_numbers_impose_execution_order': False,
            'distance_distribution': {'hop_counts': dict(sorted(histogram.items())),
                'scope': 'global initial supply to goals; shortest allowed graph distance',
                'rule': 'agreed small task' if n <= 6 else 'minimize global sum of squared hop counts',
                'squared_count_lower_bound': bound},
            'completion': {'xy_tolerance_m': .02, 'z_tolerance_m': .01,
                           'max_linear_speed_m_s': .01, 'stable_duration_sim_s': 1,
                           'max_sample_gap_sim_s': .025,
                           'table_support_required': True, 'released_required': True,
                           'distinct_object_per_goal': True}}


def reference_jobs(env, task):
    """Engineering-only witness assigning interchangeable items to goal slots."""
    pools = {}
    for obj in env['objects']:
        pools.setdefault((obj['color'], obj['shape']), []).append(obj)
    jobs = []
    for arm, orders in task['orders'].items():
        for goal in orders:
            kind = tuple(goal['requires'][k] for k in ('color', 'shape'))
            if not pools.get(kind):
                raise ValueError('Insufficient distinct inventory for '+goal['goal_id'])
            obj = pools[kind].pop()
            jobs.append((obj['id'], route(obj['source'], arm, env['arm_count'], env['topology'])))
    if any(pools.values()):
        raise ValueError('Orders do not consume the full inventory')
    return jobs


def _eligible(obj, goal, state, criteria):
    if state is None or any(obj[k] != v for k, v in goal['requires'].items()):
        return False
    p = np.asarray(state['position_m'], dtype=float)
    v = np.asarray(state['linear_velocity_m_s'], dtype=float)
    if p.shape != (3,) or v.shape != (3,) or not np.isfinite(p).all() or not np.isfinite(v).all():
        return False
    target = np.asarray(goal['position_m'])
    return bool(np.linalg.norm(p[:2]-target[:2]) <= criteria['xy_tolerance_m']
                and abs(p[2]-target[2]) <= criteria['z_tolerance_m']
                and np.linalg.norm(v) <= criteria['max_linear_speed_m_s']
                and state.get('table_supported', False) and state.get('released', False))


def _match_goals(env, task, eligible):
    objects = {obj['id']: obj for obj in env['objects']}
    goals = [g for orders in task['orders'].values() for g in orders]
    matched = {}
    def assign(index, seen):
        goal = goals[index]
        for name, obj in objects.items():
            if name in seen or not eligible(name, obj, goal):
                continue
            seen.add(name)
            if name not in matched or assign(matched[name], seen):
                matched[name] = index
                return True
        return False
    for i in range(len(goals)):
        assign(i, set())
    return {'matched_goals': len(matched), 'total_goals': len(goals),
            'assignments': {goals[i]['goal_id']: name for name, i in matched.items()}}


def match_placement_sample(env, task, states):
    """Private evaluator at one instant; does not certify stable completion."""
    result = _match_goals(env, task, lambda name, obj, goal: _eligible(obj, goal, states.get(name), task['completion']))
    return {**result, 'scope': 'instantaneous only; does not certify stable-duration completion'}


def evaluate_placement_window(env, task, samples, *, eligible=None):
    """Private sampled-physics evaluator; never a policy observation tool.

    Each sample has time_s and objects keyed by private object IDs. A matching
    object must satisfy a goal throughout the final one-second window. Same-type
    IDs remain interchangeable; the witness cannot count one body twice.
    """
    incomplete = {'complete': False, 'matched_goals': 0, 'total_goals': sum(map(len, task['orders'].values()))}
    if not samples:
        return {**incomplete, 'reason': 'no samples'}
    times = np.asarray([s['time_s'] for s in samples], dtype=float)
    if not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError('Evaluation timestamps must be finite and strictly increasing')
    duration = task['completion']['stable_duration_sim_s']
    starts = np.flatnonzero(times <= times[-1]-duration+1e-9)
    if not len(starts):
        return {**incomplete, 'reason': 'insufficient stable-duration window'}
    start = starts[-1]
    if np.max(np.diff(times[start:])) > task['completion']['max_sample_gap_sim_s']+1e-9:
        return {**incomplete, 'reason': 'sampling gap exceeds declared limit'}
    window = samples[start:]
    predicate=eligible or _eligible
    result = _match_goals(env, task, lambda name, obj, goal: all(
        predicate(obj, goal, sample['objects'].get(name), task['completion']) for sample in window))
    return {**result, 'complete': result['matched_goals'] == result['total_goals'],
            'checked_duration_sim_s': float(times[-1]-times[start]), 'sample_count': len(window),
            'scope': 'sampled truth; requires future run adapter, not exposed to policy'}


def reference(env, task):
    """Offline multistart insertion; no-wait transfers ensure buffer feasibility.

    This finds a resource-feasible upper bound, not a physical motion plan or
    a guaranteed optimum. Receiver pickup follows sender placement next slot.
    """
    jobs = reference_jobs(env, task)
    names, routes = [j[0] for j in jobs], [j[1] for j in jobs]
    end = [2*len(r) for r in routes]
    def edge(a, b):
        return '-'.join(sorted((a, b)))
    arm_load = Counter(a for r in routes for a in r for _ in range(2))
    edge_load = Counter(edge(a, b) for r in routes for a, b in zip(r, r[1:]) for _ in range(2))
    lower = max(max(arm_load.values()), max(end), max(edge_load.values(), default=0))
    rng = random.Random(task['seed'])
    batches = None
    for trial in range(256):
        order = list(range(len(routes)))
        if trial == 0:
            order.sort(key=lambda i: -len(routes[i]))
        else:
            rng.shuffle(order)
        timeline, occupied = [], []
        for i in order:
            r = routes[i]
            resources = []
            for j, arm in enumerate(r):
                pick, place = {('arm', arm)}, {('arm', arm)}
                if j:
                    pick.add(('edge', edge(r[j-1], arm)))
                if j+1 < len(r):
                    place.add(('edge', edge(arm, r[j+1])))
                resources.extend((pick, place))
            start = 0
            while any(start+k < len(occupied) and occupied[start+k] & needed
                      for k, needed in enumerate(resources)):
                start += 1
            while len(timeline) < start+len(resources):
                timeline.append([]); occupied.append(set())
            for k, needed in enumerate(resources):
                occupied[start+k].update(needed)
                timeline[start+k].append({'object': names[i], 'arm': r[k//2],
                                          'phase': 'place_clear' if k % 2 else 'pick_clear',
                                          'route_index': k//2})
        if batches is None or len(timeline) < len(batches):
            batches = timeline
        if len(batches) == lower:
            break
    return {'task': task['id'], 'scope': 'unit pick-clear/place-clear resource abstraction',
            'route_rule': 'shortest hop; increasing station IDs for an equal-length ring route',
            'method': 'deterministic multistart no-wait insertion', 'trials_used': trial+1,
            'serial_phase_slots': sum(end), 'parallel_feasible_phase_slots': len(batches),
            'lower_bound_phase_slots_for_fixed_routes': lower,
            'optimality_proven': len(batches) == lower,
            'arm_phase_load': dict(arm_load), 'edge_phase_load': dict(edge_load),
            'hop_histogram': dict(sorted(Counter(len(r)-1 for r in routes).items())),
            'assumptions': ['unit phase durations', 'one held object per robot',
                            'edge capacity one; receiver picks in slot after sender places',
                            'phase finishes with arm clear', 'no geometric trajectory constraints',
                            'all grasps succeed; no model, image or communication latency'],
            'schedule': batches}


def replay_reference(env, task, result):
    routes = dict(reference_jobs(env, task))
    progress = {name: 0 for name in routes}
    held, buffers = {}, {}
    for batch in result['schedule']:
        arms, edges, updates = set(), set(), []
        for op in batch:
            name, arm = op['object'], op['arm']
            r, p = routes[name], progress[name]
            assert p < 2*len(r) and arm == r[p//2] and arm not in arms
            assert op['route_index'] == p//2
            placing = p % 2
            assert op['phase'] == ('place_clear' if placing else 'pick_clear')
            resource = None
            if placing:
                assert held[arm] == name
                if p//2+1 < len(r):
                    resource = tuple(sorted((arm, r[p//2+1])))
                    assert resource not in buffers
            else:
                assert arm not in held
                if p//2:
                    resource = tuple(sorted((r[p//2-1], arm)))
                    assert buffers.get(resource) == name
            if resource:
                assert resource not in edges
                edges.add(resource)
            arms.add(arm)
            updates.append((name, arm, placing, resource))
        for name, arm, placing, resource in updates:
            if placing:
                del held[arm]
                if resource:
                    buffers[resource] = name
            else:
                held[arm] = name
                if resource:
                    del buffers[resource]
            progress[name] += 1
    assert not held and not buffers
    assert all(progress[name] == 2*len(r) for name, r in routes.items())


def _font(size):
    try:
        return ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', size)
    except OSError:
        return ImageFont.load_default(size=size)


def _camera(world, name, position, target, fovy=48):
    z = np.array(position, float)-target
    z /= np.linalg.norm(z)
    x = np.cross([0, 0, 1], z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    ET.SubElement(world, 'camera', name=name, pos=vector(position), xyaxes=vector([*x, *y]), fovy=str(fovy))


def build_xml(env, directory):
    directory = Path(directory)
    (directory/'assets').mkdir(parents=True, exist_ok=True)
    for suffix in ('*_dots.png', '*_stripes.png'):
        for obsolete in (directory/'assets').glob(suffix):
            obsolete.unlink()
    root = ET.parse(ROOT/'vendor/panda/panda.xml').getroot()
    root.set('model', 'RoboFrontier cube transport '+env['id'])
    # Relative paths keep the generated assets relocatable with this repository.
    root.find('compiler').set('meshdir', os.path.relpath(ROOT/'vendor/panda/assets', directory))
    root.remove(root.find('keyframe'))
    p = env['physics']
    root.find('option').attrib.update(timestep=str(p['timestep_s']), gravity=vector(p['gravity_m_s2']),
        integrator='implicitfast', solver='Newton', iterations='100', noslip_iterations=str(p['noslip_iterations']))
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='1280', offheight='900')
    ET.SubElement(visual, 'headlight', ambient='.25 .25 .25', diffuse='.40 .40 .40')
    ET.SubElement(visual, 'rgba', haze='.88 .94 1 1')
    world, asset = root.find('worldbody'), root.find('asset')
    original = copy.deepcopy(world.find('body')); world.clear()
    sections = {}
    for tag in ('tendon', 'equality', 'actuator', 'contact'):
        sections[tag] = list(copy.deepcopy(root.find(tag)))
        root.find(tag).clear()
    named = {e.get('name') for e in original.iter() if e.get('name')}
    named.update(e.get('name') for values in sections.values() for child in values for e in child.iter() if e.get('name'))
    def renamed(element, prefix):
        element = copy.deepcopy(element)
        for e in element.iter():
            for key, value in list(e.attrib.items()):
                if key in ('name', 'joint', 'joint1', 'joint2', 'tendon', 'body1', 'body2') and value in named:
                    e.set(key, prefix+value)
        return element
    ET.SubElement(asset, 'texture', name='sky', type='skybox', builtin='gradient',
                  rgb1='.75 .86 .98', rgb2='.90 .96 1', width='256', height='256')
    ET.SubElement(asset, 'material', name='floor_light', rgba=vector(env['floor_rgba']), emission='.25', specular='0')
    ET.SubElement(asset, 'material', name='table_oak', rgba=vector(env['table_rgba']),
                  emission='.12', specular='.05', shininess='.05', reflectance='0')
    ET.SubElement(world, 'light', pos='0 0 5', dir='0 0 -1', diffuse='.5 .5 .5', castshadow='true')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='20 20 .1', material='floor_light')
    for i, table in enumerate(env['tables']):
        ET.SubElement(world, 'geom', name=f'table_{i}', type='box', pos=vector(table['center_m']),
                      size=vector(table['half_size_m']), material='table_oak',
                      friction=vector(p['contact_friction']), condim='4')
        # Solid supporting plinth below each strip, no unmodelled floating table.
        half = table['half_size_m']
        ET.SubElement(world, 'geom', name=f'table_support_{i}', type='box',
                      pos=vector([*table['center_m'][:2], .17]),
                      size=vector([max(.02, half[0]-.04), max(.02, half[1]-.04), .17]), rgba='.36 .29 .23 1')
    for station in env['stations']:
        arm = station['id']; yaw = station['yaw_rad']
        quat = vector([math.cos(yaw/2), 0, 0, math.sin(yaw/2)])
        body = renamed(original, arm+'_')
        body.set('pos', vector(station['base_position_m'])); body.set('quat', quat)
        for suffix in ('finger_joint1', 'finger_joint2'):
            joint = body.find(f".//joint[@name='{arm}_{suffix}']")
            joint.set('solreflimit', '0.004 1'); joint.set('solimplimit', '0.99 0.999 0.0001 0.5 2')
        ET.SubElement(body.find(f".//body[@name='{arm}_hand']"), 'site', name=arm+'_tcp',
                      pos='0 0 .1029', size='.004', rgba='1 .65 .05 1')
        world.append(body)
        for tag, children in sections.items():
            for child in children:
                root.find(tag).append(renamed(child, arm+'_'))
        image = Image.new('RGB', (256, 256), (230, 241, 255))
        ImageDraw.Draw(image).text((55, 5), arm, font=_font(210), fill=(15, 35, 65))
        image.save(directory/'assets'/f'label_{arm}.png')
        ET.SubElement(asset, 'texture', name='label_'+arm, type='2d', file=f'assets/label_{arm}.png')
        ET.SubElement(asset, 'material', name='label_'+arm, texture='label_'+arm)
        ET.SubElement(world, 'geom', name=arm+'_label', type='box', size='.065 .065 .0003',
                      pos=vector(xyz(station, .12, 0, .4006)), quat=quat, material='label_'+arm,
                      contype='0', conaffinity='0')
        for j, position in enumerate(station['goal_positions_m']):
            slot_name = f'{arm}_slot{j+1}'
            image = Image.new('RGB', (128, 128), (248, 237, 181))
            ImageDraw.Draw(image).text((40, 15), str(j+1), font=_font(90), fill=(65, 56, 30))
            image.save(directory/'assets'/f'{slot_name}.png')
            ET.SubElement(asset, 'texture', name=slot_name, type='2d', file=f'assets/{slot_name}.png')
            ET.SubElement(asset, 'material', name=slot_name, texture=slot_name)
            ET.SubElement(world, 'geom', name=f'{arm}_slot{j+1}', type='box', size='.032 .032 .0003',
                          pos=vector([*position[:2], .4005]), quat=quat, material=slot_name, contype='0', conaffinity='0')
            ET.SubElement(world, 'site', name=f'{arm}_goal{j+1}', pos=vector(position), size='.001', rgba='0 0 0 0')
        for j, position in enumerate(station['parking_positions_m']):
            ET.SubElement(world, 'geom', name=f'{arm}_parking{j+1}', type='box', size='.029 .029 .0003',
                          pos=vector([*position[:2], .4005]), quat=quat, rgba='.70 .85 .82 1', contype='0', conaffinity='0')
        _camera(world, arm+'_front', xyz(station, 1.1, 0, 1.25), np.array(xyz(station, .35, 0, .45)), 50)
        _camera(world, arm+'_side', xyz(station, .65, -.85, 1.15), np.array(xyz(station, .35, 0, .45)), 50)
    for edge in env['edges']:
        ET.SubElement(world, 'geom', name='handoff_'+edge['id'], type='cylinder', size='.033 .0003',
                      pos=vector([*edge['position_m'][:2], .4005]), rgba='.55 .73 .93 1', contype='0', conaffinity='0')
    for obj in env['objects']:
        attrs = {'rgba': vector([*obj['rgb'], 1])}
        body = ET.SubElement(world, 'body', name=obj['id'], pos=vector(obj['initial_position_m']))
        ET.SubElement(body, 'freejoint', name=obj['id']+'_free')
        shape = ({'type': 'cylinder', 'size': vector([p['cylinder_radius_m'], p['cylinder_height_m']/2])} if obj['shape'] == 'cylinder'
                 else {'type': 'box', 'size': vector([p['cube_size_m']/2]*3)})
        ET.SubElement(body, 'geom', name=obj['id']+'_geom', mass=str(p['object_mass_kg']),
                      friction=vector(p['contact_friction']), condim='4', **shape, **attrs)
    radius = max(np.linalg.norm(s['base_position_m'][:2]) for s in env['stations'])+.4
    _camera(world, 'global', [radius*1.1, -radius*1.45, .4+radius*2.1], np.array([0, 0, .45]), 48)
    _camera(world, 'global_side', [-radius*1.7, -radius*1.15, .4+radius*1.6], np.array([0, 0, .50]), 48)
    ET.SubElement(world, 'camera', name='top', pos=vector([0, 0, .4+radius*3]),
                  xyaxes='1 0 0 0 1 0', fovy='48')
    ET.SubElement(root, 'statistic', center='0 0 .5', extent=str(radius*2))
    return root


def initialize(model, env):
    data = mujoco.MjData(model)
    seed = np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    for station in env['stations']:
        name = station['id']; ik = RobotIK(model, name+'_')
        _, rotation = ik.pose(seed)
        q, error = ik.solve(np.array(station['home_tcp_m']), rotation, seed, 250)
        if error > .002:
            raise ValueError(f'{name}: home unreachable: {error}')
        data.qpos[ik.qids] = q
        for i in range(7):
            data.ctrl[model.actuator(f'{name}_actuator{i+1}').id] = q[i]
        data.ctrl[model.actuator(name+'_actuator8').id] = 255
        for suffix in ('finger_joint1', 'finger_joint2'):
            data.joint(name+'_'+suffix).qpos[0] = .04
    mujoco.mj_forward(model, data)
    return data


def add_initial_keyframe(root, model, data):
    keys = ET.SubElement(root, 'keyframe')
    ET.SubElement(keys, 'key', name='initial', qpos=vector(data.qpos), ctrl=vector(data.ctrl))
