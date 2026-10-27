"""Mosaic R14 colour-only design; private assignment evidence, no physics steps.

Nearby means nearest robot base in tabletop XY (ties by robot ID). A local
certificate uses distinct stock and target cells and sampled pickup / release
geometry for that same robot. It is not a dynamic or parallel-path certificate.
"""
from collections import Counter
from copy import deepcopy
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import linear_sum_assignment

from .mosaic_policy_package import ROOT, checksum, read, write

CATALOG = 'configs/suites/frontier-mosaic-r13.json'
COVERAGE = 'runs/engineering/mosaic-r13-coverage-20261025'


def nearest(task, point):
    return min(task['arms'], key=lambda r: (math.dist(task['arms'][r]['base_position_m'][:2], point[:2]), r))


def geometry_signature(folder):
    """A colour-independent exact compiled collision/initial-state identity."""
    import mujoco
    from .v2 import physical_summary
    model = mujoco.MjModel.from_xml_path(str(Path(folder)/'scene.xml'))
    return physical_summary(model)


def choose_slots(task, pickup_ok, goal_states, fraction=.70):
    """Minimum-cost one-to-one placement design with a local quota per robot.

    True/False/None goal states mean confirmed/rejected/not yet checked. Only
    selected unknown edges are subsequently probed, avoiding an exhaustive IK
    search. Cross-zone slots preserve room for collaboration, not forced routes.
    """
    objects, goals = task['objects'], task['goals']
    slots = []
    for robot in task['arms']:
        stock = [o for o in objects if nearest(task, o['initial_position_m']) == robot]
        quota = math.ceil(fraction*len(stock))
        if quota > sum(pickup_ok[o['id']] for o in stock):
            raise ValueError('Insufficient confirmed nearest-robot pickups: '+robot)
        slots += [(robot, True)]*quota + [(robot, False)]*(len(stock)-quota)
    cost = np.full((len(slots), len(goals)), 1e9)
    for i, (robot, local) in enumerate(slots):
        base = task['arms'][robot]['base_position_m']
        for j, goal in enumerate(goals):
            receiver = nearest(task, goal['position_m'])
            distance = math.dist(base[:2], goal['position_m'][:2])
            if local:
                if goal_states.get((robot, goal['goal_id'])) is False:
                    continue
                # Only neighboring parts of the same side, not distant opposite
                # rows or the complete geometric bounding sphere.
                near_distance = math.dist(task['arms'][receiver]['base_position_m'][:2], goal['position_m'][:2])
                if distance > .86 or distance > near_distance+.24 or base[1]*goal['position_m'][1] < 0:
                    continue
                cost[i, j] = distance + (0 if receiver == robot else 2)
            elif receiver != robot:
                # Favor adjacent cross-zone demand over long relays.
                cost[i, j] = math.dist(base[:2], task['arms'][receiver]['base_position_m'][:2]) + .1*distance
    rows, cols = linear_sum_assignment(cost)
    if any(cost[i, j] >= 1e9 for i, j in zip(rows, cols)):
        raise ValueError('No allocation at requested local fraction using admitted candidates')
    return [dict(robot=slots[i][0], local=slots[i][1], goal_index=int(j)) for i, j in zip(rows, cols)]


def colour_assignment(task, slots, pickup_ok):
    """Use a distinct physical stock cube for each allocated target colour."""
    assignments = []
    for robot in task['arms']:
        stock = [o for o in task['objects'] if nearest(task, o['initial_position_m']) == robot]
        selected = [s for s in slots if s['robot'] == robot]
        cost = np.zeros((len(stock), len(selected)))
        for i, obj in enumerate(stock):
            for j, slot in enumerate(selected):
                goal = task['goals'][slot['goal_index']]
                cost[i, j] = math.dist(obj['initial_position_m'][:2], goal['position_m'][:2])
                cost[i, j] += .015*(obj['color'] != goal['requires']['color'])
                if slot['local'] and not pickup_ok[obj['id']]:
                    cost[i, j] = 1e9
        rows, cols = linear_sum_assignment(cost)
        for i, j in zip(rows, cols):
            if cost[i, j] >= 1e9:
                raise ValueError('Local quota requires an unconfirmed pickup')
            obj, slot = stock[i], selected[j]
            goal = task['goals'][slot['goal_index']]
            assignments.append(dict(object_id=obj['id'], robot=robot,
                goal_id=goal['goal_id'], colour=goal['requires']['color'],
                local=slot['local'], previous_colour=obj['color']))
    if len({a['object_id'] for a in assignments}) != len(task['objects']) or len({a['goal_id'] for a in assignments}) != len(task['goals']):
        raise ValueError('Assignment must be one-to-one')
    if Counter(a['colour'] for a in assignments) != Counter(g['requires']['color'] for g in task['goals']):
        raise ValueError('Colour inventory not conserved')
    return sorted(assignments, key=lambda a:a['object_id'])


def validate_plan(task, plan):
    objects={o['id']:o for o in task['objects']};goals={g['goal_id']:g for g in task['goals']}
    entries=plan['assignments']
    if len(entries)!=len(objects) or {a['object_id'] for a in entries}!=set(objects) or len({a['goal_id'] for a in entries})!=len(goals):
        raise ValueError('Design must use every stock cube and target once')
    confirmed={(p['robot'],p['goal_id']) for p in plan['confirmed_goal_pairs']}
    for entry in entries:
        obj,goal=objects[entry['object_id']],goals[entry['goal_id']]
        if entry['robot']!=nearest(task,obj['initial_position_m']) or entry['colour']!=goal['requires']['color'] or entry['previous_colour']!=obj['color']:
            raise ValueError('Design identity, nearest robot or colour differs')
        if entry['local'] and (not plan['nearest_pickup_confirmed'][obj['id']] or (entry['robot'],entry['goal_id']) not in confirmed):
            raise ValueError('Local pairing lacks geometric evidence')
    for robot in task['arms']:
        chosen=[a for a in entries if a['robot']==robot]
        local=sum(a['local'] for a in chosen)
        if local<math.ceil(plan['requested_local_fraction']*len(chosen)):
            raise ValueError('Local quota not met')
        summary=next(r for r in plan['per_robot'] if r['robot']==robot)
        if summary['certified_local_pairs']!=local or summary['nearby_stock']!=len(chosen):
            raise ValueError('Design summary differs from pairing evidence')


def _probe_robot(checker, robot, goal):
    # Coverage.probe normally stops at the first successful robot. Restrict its
    # search while retaining the complete initial-scene collision owner map.
    saved = checker.arms
    checker.arms = {robot:saved[robot]}
    try:
        return checker.probe('goal', goal)
    finally:
        checker.arms = saved


def plan_scale(arms, output, fraction=.70):
    from scripts.check_mosaic_coverage_r12 import Coverage
    cases = [c for c in read(ROOT/CATALOG)['cases'] if c['arms'] == arms]
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    tasks = [read(ROOT/c['directory']/'task.json') for c in cases]
    task = tasks[0]
    for other in tasks[1:]:
        assert task['objects'] == other['objects'] and task['arms'] == other['arms']
        assert task['grid']['pattern'] == other['grid']['pattern']
    reports = [read(ROOT/COVERAGE/c['id']/'report.json') for c in cases]
    for case, report in zip(cases, reports):
        assert report['source_scene_sha256'] == checksum(ROOT/case['directory']/'scene.xml')
        assert report['complete'] and all(r['confirmed'] for k in ('sources','goals') for r in report[k])
    pickup_ok = {o['id']:all(next(r for r in report['sources'] if r['id'] == o['id'])['witness']['robot'] == nearest(task, o['initial_position_m']) for report in reports) for o in task['objects']}
    # All three levels need the same robot to have a geometric goal witness.
    states = {}
    for g in task['goals']:
        robots = [next(r for r in report['goals'] if r['id'] == g['goal_id'])['witness']['robot'] for report in reports]
        if len(set(robots)) == 1:
            states[(robots[0], g['goal_id'])] = True
    cache_file = output/f'a{arms:02d}-extra-geometry.json'
    extra = read(cache_file) if cache_file.exists() else dict(source_scene_sha256=[t['scene_sha256'] for t in tasks], checks={})
    if extra['source_scene_sha256'] != [t['scene_sha256'] for t in tasks]:
        raise ValueError('Stale geometric cache')
    checkers = {}
    while True:
        slots = choose_slots(task, pickup_ok, states, fraction)
        pending = sorted({(s['robot'], task['goals'][s['goal_index']]['goal_id']) for s in slots if s['local'] and states.get((s['robot'], task['goals'][s['goal_index']]['goal_id'])) is None})
        if not pending:
            break
        for robot, ident in pending:
            goal_index = next(i for i,g in enumerate(task['goals']) if g['goal_id'] == ident)
            outcomes = []
            for index, case in enumerate(cases):
                key = f"{case['id']}/{robot}/{ident}"
                if key not in extra['checks']:
                    # L2/L3 may reuse only after exact compiled physical identity.
                    sibling_key = f"{cases[1]['id']}/{robot}/{ident}"
                    if index == 2 and sibling_key in extra['checks'] and geometry_signature(ROOT/case['directory']) == geometry_signature(ROOT/cases[1]['directory']):
                        extra['checks'][key] = dict(extra['checks'][sibling_key], equivalent_to=sibling_key)
                    else:
                        if index not in checkers:
                            checkers[index] = Coverage(ROOT/case['directory'])
                        print('GEOMETRY', case['id'], robot, ident, flush=True)
                        extra['checks'][key] = _probe_robot(checkers[index], robot, tasks[index]['goals'][goal_index])
                    write(cache_file, extra)
                outcomes.append(extra['checks'][key]['confirmed'])
            states[(robot, ident)] = all(outcomes)
    assignments = colour_assignment(task, slots, pickup_ok)
    rows = []
    for robot in task['arms']:
        selected = [a for a in assignments if a['robot'] == robot]
        local = [a for a in selected if a['local']]
        near_goals = [g for g in task['goals'] if nearest(task, g['position_m']) == robot]
        old = Counter(a['previous_colour'] for a in selected)
        new = Counter(a['colour'] for a in selected)
        demand = Counter(g['requires']['color'] for g in near_goals)
        rows.append(dict(robot=robot, nearby_stock=len(selected), certified_local_pairs=len(local),
            certified_local_fraction=len(local)/len(selected), nearest_goal_count=len(near_goals),
            old_nearest_region_colour_matches=sum((old & demand).values()),
            new_nearest_region_colour_matches=sum((new & demand).values()),
            old_colours=dict(old), new_colours=dict(new), nearest_region_demand=dict(demand)))
    result = dict(schema='robofrontier.mosaic-local-colour-plan.v1', arms=arms,
        source_cases=[dict(id=c['id'], directory=c['directory'], scene_sha256=t['scene_sha256'],
            coverage_sha256=checksum(ROOT/COVERAGE/c['id']/'report.json')) for c,t in zip(cases,tasks)],
        requested_local_fraction=fraction, physics_steps=0, policy_input=False,
        scope='One-to-one colour design with same-robot sampled pickup and target approach/open/withdraw witnesses; no loaded connecting path, dynamic or parallel execution certification',
        assignments=assignments, per_robot=rows,
        extra_geometry_file=str(cache_file.resolve()), extra_geometry_sha256=checksum(cache_file) if cache_file.exists() else None,
        confirmed_goal_pairs=[dict(robot=r, goal_id=g) for (r,g), ok in sorted(states.items()) if ok],
        nearest_pickup_confirmed=pickup_ok)
    write(output/f'a{arms:02d}-plan.json', result)
    print('PLANNED', arms, [(r['robot'],r['certified_local_pairs'],r['nearby_stock']) for r in rows], flush=True)
    return result


def build_case(case, plan):
    """Clone only to a new revision. Strip RGBA and XML must be identical."""
    import mujoco
    from . import v2
    source = ROOT/case['directory']
    new_case = dict(case, scene_revision=14, source_scene_revision=13)
    new_case['directory'] = str(v2.directory(new_case).relative_to(ROOT))
    target = ROOT/new_case['directory']
    if target.exists():
        raise FileExistsError(target)
    original_task = read(source/'task.json'); env = read(source/'environment.json')
    pin=next(c for c in plan['source_cases'] if c['id']==case['id'])
    if pin['directory']!=case['directory'] or pin['scene_sha256']!=checksum(source/'scene.xml') or pin['coverage_sha256']!=checksum(ROOT/COVERAGE/case['id']/'report.json'):
        raise ValueError('Plan baseline has changed')
    validate_plan(original_task,plan)
    colours = {a['object_id']:a['colour'] for a in plan['assignments']}
    root = ET.parse(source/'scene.xml').getroot()
    original_xml = ET.tostring(root)
    for obj in env['objects']:
        obj['color'] = colours[obj['id']]
        obj['rgb'] = [int(env['grid']['palette'][obj['color']][i:i+2],16)/255 for i in (1,3,5)]
        geom = root.find(f"worldbody/body[@name='{obj['id']}']/geom[@name='{obj['id']}_geom']")
        geom.set('rgba',v2.vector(obj['rgb']+[1.]))
    # Compare the complete XML excluding the only permitted visual attributes.
    def stripped(element):
        element = deepcopy(element)
        for name in colours:
            element.find(f"worldbody/body[@name='{name}']/geom[@name='{name}_geom']").attrib.pop('rgba')
        return ET.tostring(element)
    assert stripped(ET.fromstring(original_xml)) == stripped(root)
    target.mkdir(parents=True)
    frozen = read(source/'workstation.json')
    for asset in frozen['assets']:
        src, dest = source/asset['path'], target/asset['path']
        if checksum(src) != asset['sha256']:
            raise ValueError('Source asset differs')
        dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(src,dest)
    for name in ('target-pattern.png','target-original.png','source-pattern.png'):
        if (source/name).exists():shutil.copyfile(source/name,target/name)
    ET.ElementTree(root).write(target/'scene.xml',encoding='unicode')
    env['suite_case'] = new_case
    write(target/'environment.json',env)
    task = v2.task_for(new_case,env,target)
    write(target/'task.json',task); write(target/'public-task.json',v2.public_task(task))
    write(target/'parameters.json',dict(physics=env['physics'],mechanism=env['mechanism'],grid=env['grid']))
    model = mujoco.MjModel.from_xml_path(str(target/'scene.xml')); data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id); mujoco.mj_forward(model,data)
    state = v2.manifest(model,data,new_case,target,env)
    assert geometry_signature(source) == geometry_signature(target)
    assert original_task['arms'] == task['arms'] and original_task['goals'] == task['goals'] and original_task['grid'] == task['grid']
    assert [{k:v for k,v in o.items() if k not in ('color','rgb')} for o in original_task['objects']] == [{k:v for k,v in o.items() if k not in ('color','rgb')} for o in task['objects']]
    assert Counter(o['color'] for o in task['objects']) == Counter(g['requires']['color'] for g in task['goals'])
    v2.validate_task(task)
    write(target/'validation.json',dict(status='colour_only_design_geometry_equivalent_to_r13',
        oracle_verified=False,policy_verified=False,frozen_input=False,physics_steps=0,
        robot_count=case['arms'],physics_sha256=state['physics_sha256'],
        source_scene_sha256=checksum(source/'scene.xml'), source_revision=13,
        certified_local_fraction_min=min(r['certified_local_fraction'] for r in plan['per_robot']),
        old_oracle_applies_to='Original R13 colours only; not an Oracle success of this R14 task',
        scope=plan['scope']))
    return new_case
