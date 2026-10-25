"""Exact, unit-duration resource model. Engineering only, never a policy tool.

One station visit consists of pick-and-clear, then place-and-clear. A gripper
holds its object between those phases. Neighbor buffers and shared entry are
capacity limited. This does not model trajectories, physics or model latency.
"""
import argparse
import heapq
import itertools
import json
from pathlib import Path

ROUTES = {
    'A_local': 'A', 'A_to_B': 'AB', 'A_to_C': 'ABC', 'B_local': 'B',
    'B_to_C_1': 'BC', 'B_to_C_2': 'BC',
    'C_to_A_1': 'CBA', 'C_to_A_2': 'CBA', 'C_to_B': 'CB',
}


def boundary(a, b):
    return ''.join(sorted((a, b)))


def resources(state, routes):
    held, buffers = {}, {'AB': [], 'BC': []}
    for index, (phase, route) in enumerate(zip(state, routes)):
        if phase % 2:
            arm = route[phase//2]
            if arm in held:
                raise ValueError('Two objects held by one arm')
            held[arm] = index
        elif 0 < phase < 2*len(route):
            buffers[boundary(route[phase//2-1], route[phase//2])].append(index)
    return held, buffers


def candidates(state, routes, capacity):
    held, buffers = resources(state, routes)
    result = []
    for index, (phase, route) in enumerate(zip(state, routes)):
        if phase == 2*len(route):
            continue
        station = phase//2
        arm = route[station]
        edge = None
        if phase % 2 == 0:
            if arm in held:
                continue
            if station:
                edge = boundary(route[station-1], arm)
        elif station+1 < len(route):
            edge = boundary(arm, route[station+1])
            if len(buffers[edge]) >= capacity:
                continue
        result.append((index, arm, edge))
    return result


def transitions(state, routes, capacity):
    options = candidates(state, routes, capacity)
    for count in range(min(3,len(options)), 0, -1):
        for selected in itertools.combinations(options, count):
            arms = [v[1] for v in selected]
            edges = [v[2] for v in selected if v[2]]
            if len(set(arms)) != len(arms) or len(set(edges)) != len(edges):
                continue
            next_state = list(state)
            for index, _, _ in selected:
                next_state[index] += 1
            yield tuple(next_state), tuple(v[0] for v in selected)


def lower_bound(state, routes):
    loads = {arm:0 for arm in 'ABC'}
    for phase, route in zip(state, routes):
        for remaining in range(phase, 2*len(route)):
            loads[route[remaining//2]] += 1
    return max(loads.values())


def replay(schedule, capacity=1):
    names, routes = tuple(ROUTES), tuple(ROUTES.values())
    state = (0,)*len(routes)
    for batch in schedule:
        selected = tuple(sorted(names.index(name) for name in batch))
        choices = {tuple(sorted(actions)):nxt for nxt,actions in transitions(state,routes,capacity)}
        if selected not in choices:
            raise ValueError('Invalid simultaneous phase batch: '+str(batch))
        state = choices[selected]
        held, buffers = resources(state, routes)
        if any(len(v)>capacity for v in buffers.values()):
            raise ValueError('Buffer overflow')
    if state != tuple(2*len(route) for route in routes):
        raise ValueError('Schedule unfinished')
    return True


def solve(capacity=1):
    if capacity not in (1,2):
        raise ValueError('This check compares buffer capacity one or two')
    names, routes = tuple(ROUTES), tuple(ROUTES.values())
    initial = (0,)*len(routes)
    goal = tuple(2*len(route) for route in routes)
    queue = [(lower_bound(initial,routes),0,initial)]
    distances, parents = {initial:0}, {}
    expanded = 0
    while queue:
        _, cost, state = heapq.heappop(queue)
        if cost != distances[state]:
            continue
        if state == goal:
            schedule = []
            while state != initial:
                previous, actions = parents[state]
                schedule.append([names[i] for i in actions]);state=previous
            schedule.reverse();replay(schedule,capacity)
            return {'capacity_per_neighbor_buffer':capacity, 'serial_phase_slots':sum(goal),
                    'arm_load_lower_bound':lower_bound(initial,routes), 'optimal_phase_slots':cost,
                    'expanded_states':expanded, 'schedule':schedule,
                    'scope':'Exact within unit pick-clear/place-clear model; no trajectories, physics, inference or communication latency',
                    'assumptions':['one gripper per arm','one entrant per shared edge per phase',
                                   'phase end fully clears the region','all grasps succeed',
                                   'independent motions are geometrically separable']}
        expanded += 1
        for nxt, actions in transitions(state,routes,capacity):
            if cost+1 < distances.get(nxt, float('inf')):
                distances[nxt]=cost+1;parents[nxt]=(state,actions)
                heapq.heappush(queue,(cost+1+lower_bound(nxt,routes),cost+1,nxt))
    raise ValueError('No feasible resource schedule')


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path)
    args=p.parse_args()
    result=[solve(1),solve(2)]
    raw=json.dumps(result,ensure_ascii=False,indent=2)+'\n'
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(raw)
    print(raw)
