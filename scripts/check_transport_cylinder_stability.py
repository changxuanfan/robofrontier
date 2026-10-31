"""Check passive cylinder drift in prepared Transport runs, without model calls.

This finite engineering check does not certify grasp, placement or task success.
Run on a fresh batch prepared by `python -m scripts.tasks prepare-batch`.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from robofrontier.pipeline import World


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path, required=True, help='Prepared batch.json')
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--max-drift-mm', type=float, default=5.0)
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.seconds, args.max_drift_mm)):
        parser.error('seconds and max-drift-mm must be finite and positive')
    batch = json.loads(args.batch.read_text())
    jobs = [job for job in batch['jobs'] if job['case_id'].startswith('transport-')]
    if not jobs:
        raise ValueError('Batch has no Transport cases')
    failed = False
    for job in jobs:
        run = Path(job['run_directory'])
        task = json.loads((run/'private/inputs/task.json').read_text())
        names = [o['id'] for o in task['objects'] if o.get('shape') == 'cylinder']
        if not names:
            print(json.dumps({'case': job['case_id'], 'status': 'no_cylinders'}), flush=True)
            continue
        world = World(run, task)
        starts = {name: world.data.body(name).xpos[:2].copy() for name in names}
        drift = {name: 0.0 for name in names}
        end = float(world.data.time) + args.seconds
        while world.data.time < end - 1e-9:
            world.step()
            for name in names:
                displacement = float(np.linalg.norm(world.data.body(name).xpos[:2] - starts[name]))
                drift[name] = max(drift[name], displacement)
        worst = max(drift, key=drift.get)
        worst_mm = drift[worst]*1000
        finite = bool(np.isfinite(world.data.qpos).all())
        passed = finite and worst_mm <= args.max_drift_mm
        failed |= not passed
        print(json.dumps({'case': job['case_id'], 'passed': passed,
                          'sim_s': round(float(world.data.time), 3),
                          'max_horizontal_drift_mm': round(worst_mm, 3),
                          'worst_cylinder': worst, 'finite': finite}), flush=True)
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
