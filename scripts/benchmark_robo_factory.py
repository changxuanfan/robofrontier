"""Compare two factory revisions with the same warmed camera workload."""
import argparse
import json
from pathlib import Path
import statistics
import subprocess
import sys


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline', type=Path, required=True)
    p.add_argument('--candidate', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--rounds', type=int, default=3)
    p.add_argument('--frames', type=int, default=80)
    p.add_argument('--width', type=int, default=1600)
    p.add_argument('--height', type=int, default=1000)
    a = p.parse_args()
    if min(a.rounds, a.frames, a.width, a.height) <= 0:
        p.error('Rounds, frames and image dimensions must be positive')
    views = ['interior_south', 'interior_diagonal', 'detail_storage', 'detail_ceiling', 'top']
    trials = {'baseline': [], 'candidate': []}
    a.output.mkdir(parents=True, exist_ok=True)
    for run in range(a.rounds):
        # Alternate order to reduce systematic warm-up/thermal ordering effects.
        order = ['baseline', 'candidate'] if run % 2 == 0 else ['candidate', 'baseline']
        for label in order:
            output = a.output / f'{label}-{run + 1}'
            subprocess.run([
                sys.executable, '-m', 'scripts.view_robo_factory', '--render',
                '--scene', str(getattr(a, label).resolve()), '--output', str(output),
                '--width', str(a.width), '--height', str(a.height),
                '--benchmark-frames', str(a.frames), '--views', *views,
            ], check=True, stdout=subprocess.DEVNULL)
            result = json.loads((output / 'render-report.json').read_text())['views']
            trials[label].append(result)
            print(json.dumps({'round': run + 1, 'revision': label, 'done': True}), flush=True)
    comparisons = []
    for index, name in enumerate(views):
        if trials['baseline'][0][index]['camera_state'] != trials['candidate'][0][index]['camera_state']:
            raise ValueError('Camera mismatch for ' + name)
        values = {label: [r[index]['measured_view_mean_ms'] for r in runs] for label, runs in trials.items()}
        medians = {label: statistics.median(v) for label, v in values.items()}
        comparisons.append(dict(view=name, trial_mean_ms=values, median_ms=medians,
                                change_percent=100 * (medians['candidate'] / medians['baseline'] - 1)))
    report = dict(scope='Static rendering including pixel readback; no tasks or physics; timings vary with system load',
                  resolution=[a.width, a.height], rounds=a.rounds, frames_per_view_per_round=a.frames,
                  comparisons=comparisons,
                  scene_sha256={label: runs[0][0]['source_scene_sha256'] for label, runs in trials.items()},
                  peak_process_rss_mib={label: max(v['peak_process_rss_mib'] for run in runs for v in run) for label, runs in trials.items()})
    (a.output / 'comparison.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
