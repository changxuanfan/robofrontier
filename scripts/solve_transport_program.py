"""Optional offline CP-SAT proof, installed only inside the engineering run.

Optimizes unit pick/place makespan on fixed shortest routes. No physics or
policy process; the saved schedule is independently replay-checked.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy  # Keep the workspace ABI before adding optional solver-only deps.

from robofrontier.cube_transport import ROOT, reference_jobs, replay_reference


def solve(count, output, seconds, collection='cube_transport'):
    from ortools.sat.python import cp_model
    name = f'line-{count:02}' if count == 3 else f'ring-{count:02}'
    env = json.loads((ROOT/f'configs/environments/{collection}/{name}/environment.json').read_text())
    task = json.loads((ROOT/f'configs/tasks/{collection}/{name}/seed-0.json').read_text())
    original = json.loads((ROOT/f"benchmarks/{collection}/references/{task['id']}.json").read_text())
    jobs = reference_jobs(env, task)
    horizon = original['parallel_feasible_phase_slots']
    model = cp_model.CpModel()
    arms = {s['id']: [] for s in env['stations']}
    edges, variables, ends = {}, {}, []
    for obj, route in jobs:
        visits = []
        for j, arm in enumerate(route):
            pick = model.new_int_var(0, horizon-1, f'{obj}_{j}_pick')
            place = model.new_int_var(0, horizon-1, f'{obj}_{j}_place')
            end = model.new_int_var(1, horizon, f'{obj}_{j}_end')
            duration = model.new_int_var(2, horizon, f'{obj}_{j}_duration')
            model.add(place >= pick+1)
            model.add(end == place+1)
            arms[arm].append(model.new_interval_var(pick, duration, end, f'{obj}_{j}_gripper_occupancy'))
            variables[(obj, j, 'pick_clear')] = pick
            variables[(obj, j, 'place_clear')] = place
            visits.append((pick, place, end))
        ends.append(visits[-1][2])
        for j in range(len(visits)-1):
            pick, place, end = visits[j]
            next_pick, _, _ = visits[j+1]
            model.add(next_pick >= end)
            edge = tuple(sorted(route[j:j+2]))
            size = model.new_int_var(2, horizon, f'{obj}_{j}_edge_duration')
            edge_end = model.new_int_var(1, horizon, f'{obj}_{j}_edge_end')
            model.add(edge_end == next_pick+1)
            edges.setdefault(edge, []).append(model.new_interval_var(place, size, edge_end, f'{obj}_{j}_buffer_occupancy'))
    for intervals in [*arms.values(), *edges.values()]:
        model.add_no_overlap(intervals)
    makespan = model.new_int_var(original['lower_bound_phase_slots_for_fixed_routes'], horizon, 'makespan')
    model.add_max_equality(makespan, ends)
    model.minimize(makespan)
    for tick, batch in enumerate(original['schedule']):
        for op in batch:
            model.add_hint(variables[(op['object'], op['route_index'], op['phase'])], tick)
    model.add_hint(makespan, horizon)
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = seconds
    solver.parameters.num_search_workers = 4
    solver.parameters.random_seed = 0
    status = solver.solve(model)
    result = dict(original)
    if status in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        schedule = [[] for _ in range(solver.value(makespan))]
        for obj, route in jobs:
            for j, arm in enumerate(route):
                for phase in ('pick_clear', 'place_clear'):
                    schedule[solver.value(variables[(obj, j, phase)])].append(
                        {'object': obj, 'arm': arm, 'phase': phase, 'route_index': j})
        result.update(schedule=schedule, parallel_feasible_phase_slots=len(schedule),
                      optimality_proven=status == cp_model.OPTIMAL)
    result.update(collection=collection, method='CP-SAT fixed-route resource optimization', solver_status=solver.status_name(status),
                  solver_best_bound=solver.best_objective_bound, solver_wall_s=solver.wall_time,
                  solver_response_stats=solver.response_stats(),
                  proof_scope='unit phase durations; fixed shortest routes; whole-gripper and capacity-1 edge occupancy; excludes trajectories and model latency')
    replay_reference(env, task, result)
    result['schedule_replay_passed'] = True
    directory = output/name; directory.mkdir(parents=True, exist_ok=True)
    (directory/'schedule.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'environment': name, 'rounds': len(result['schedule']),
                      'status': result['solver_status'], 'optimality_proven': result['optimality_proven'],
                      'best_bound': result['solver_best_bound'], 'wall_s': result['solver_wall_s']}), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--counts', type=int, nargs='+', default=list(range(3, 13)))
    parser.add_argument('--seconds', type=float, default=45)
    parser.add_argument('--collection', default='cube_transport')
    parser.add_argument('--solver-deps', type=Path, help='Existing isolated optional solver dependencies')
    args = parser.parse_args()
    sys.path.insert(0, str(args.solver_deps.resolve() if args.solver_deps else args.output.resolve()/'solver-deps'))
    for n in args.counts:
        solve(n, args.output.resolve(), args.seconds, args.collection)
