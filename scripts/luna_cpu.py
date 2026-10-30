"""Headless GPT-6 Luna benchmark entrypoint; no LLM supervisor required."""
import argparse
import hashlib
from collections import deque
from datetime import datetime
import json
import math
import os
from pathlib import Path
import platform
import re
import shutil
import sqlite3
import subprocess
import sys
import time

from scripts.benchmark import load_env
from robofrontier.task_families import ROOT

CPU_PROFILES = ('luna', 'openlux-terra', 'openlux-luna')


def cpu_environment():
    env = os.environ.copy()
    env.update(MUJOCO_GL='osmesa', PYOPENGL_PLATFORM='osmesa',
               LIBGL_ALWAYS_SOFTWARE='true', GALLIUM_DRIVER='llvmpipe',
               CUDA_VISIBLE_DEVICES='')
    return env


def batch_codex_home(batch):
    path = Path(batch).absolute()
    label = ''.join(c if c.isalnum() or c in '-_' else '_' for c in path.name)[:48] or 'batch'
    identity = hashlib.sha256(str(path).encode()).hexdigest()[:12]
    return ROOT/'.codex-luna'/f'{label}-{identity}'


def sandbox_probe(env):
    result = subprocess.run(['codex', 'sandbox', '--', 'sh', '-c', 'test -r configs/models.json'],
                            cwd=ROOT, env=env, text=True, capture_output=True, timeout=30)
    if result.returncode:
        raise RuntimeError('Codex sandbox cannot read files: '+result.stderr[-1200:])


def render_probe(env):
    # Separate process: MuJoCo chooses its GL backend at import time.
    if platform.system() != 'Linux':
        raise RuntimeError('The MuJoCo OSMesa backend requires Linux; on macOS use the documented Linux container.')
    code = r'''import json
import mujoco
from OpenGL import GL
model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type="sphere" size="0.1"/></worldbody></mujoco>')
data = mujoco.MjData(model)
with mujoco.Renderer(model, width=64, height=64) as r:
    r.update_scene(data)
    pixels = r.render()
    renderer = GL.glGetString(GL.GL_RENDERER).decode()
    vendor = GL.glGetString(GL.GL_VENDOR).decode()
print(json.dumps({'renderer': renderer, 'vendor': vendor, 'size': list(pixels.shape)}))
'''
    result = subprocess.run([sys.executable, '-c', code], env=env, text=True,
                            capture_output=True, timeout=60, cwd=ROOT)
    if result.returncode:
        raise RuntimeError('CPU render probe failed: ' + result.stderr[-2000:])
    info = json.loads(result.stdout.strip().splitlines()[-1])
    if not any(name in info['renderer'].lower() for name in ('llvmpipe', 'softpipe', 'swrast', 'software rasterizer')):
        raise RuntimeError('Renderer is not a verified software renderer: ' + info['renderer'])
    return info


def usage_report(batch_path):
    batch = json.loads((batch_path/'batch.json').read_text())
    sessions = Path(os.environ.get('CODEX_HOME', Path.home()/'.codex'))/'sessions'
    seen = set()
    observations = []
    result = []
    for job in batch['jobs']:
        db = Path(job['run_directory'])/'private/state/runtime.sqlite'
        with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as conn:
            ids = [json.loads(value).get('thread_id') for value, in conn.execute('select value from agents')]
        ids = [i for i in ids if i]
        totals = dict(input_tokens=0, cached_input_tokens=0, cache_write_input_tokens=0,
                      output_tokens=0, reasoning_output_tokens=0)
        missing = []
        missing_response_ids = 0
        for thread_id in ids:
            paths = list(sessions.rglob('*'+thread_id+'.jsonl')) if sessions.is_dir() else []
            if not paths: missing.append(thread_id)
            for path in paths:
                with path.open() as stream:
                    for line in stream:
                        try: record = json.loads(line)
                        except json.JSONDecodeError: continue  # incomplete last line of an active trace
                        payload = record.get('payload', {})
                        if record.get('type') != 'token_usage_record' or payload.get('thread_id') != thread_id: continue
                        response = payload.get('response_id')
                        if not response:
                            missing_response_ids += 1
                            continue
                        if response in seen: continue
                        seen.add(response)
                        usage = payload.get('usage', {})
                        for key in totals: totals[key] += usage.get(key, 0)
                        try:
                            timestamp = datetime.fromisoformat(record['timestamp'].replace('Z', '+00:00')).timestamp()
                            observations.append((timestamp, usage.get('input_tokens', 0), usage.get('output_tokens', 0)))
                        except (ValueError, KeyError, TypeError):
                            pass
        totals['cache_hit_ratio'] = totals['cached_input_tokens']/totals['input_tokens'] if totals['input_tokens'] else None
        result.append({'case_id': job['case_id'], 'threads': len(ids), 'missing_traces': missing,
                       'records_without_response_id': missing_response_ids, **totals})
    total_input = sum(job['input_tokens'] for job in result)
    total_cached = sum(job['cached_input_tokens'] for job in result)
    total_output = sum(job['output_tokens'] for job in result)
    window = deque(); running_input = running_output = peak_input = peak_output = peak_combined = 0
    for timestamp, input_count, output_count in sorted(observations):
        window.append((timestamp, input_count, output_count)); running_input += input_count; running_output += output_count
        while window and window[0][0] <= timestamp-60:
            _, old_input, old_output = window.popleft(); running_input -= old_input; running_output -= old_output
        peak_input = max(peak_input, running_input); peak_output = max(peak_output, running_output)
        peak_combined = max(peak_combined, running_input + running_output)
    elapsed = max(0., batch.get('finished_at', time.time())-batch.get('started_at', time.time()))
    return {'scope': 'Recorded unique API responses; active traces and billing may differ.',
            'total_input_tokens': total_input, 'total_cached_input_tokens': total_cached, 'total_output_tokens': total_output,
            'cache_hit_ratio': total_cached/total_input if total_input else None, 'batch_elapsed_s': elapsed,
            'average_input_tpm': total_input*60/elapsed if elapsed else None,
            'average_output_tpm': total_output*60/elapsed if elapsed else None,
            'peak_60s_completed_input_tokens': peak_input, 'peak_60s_completed_output_tokens': peak_output,
            'peak_60s_completed_combined_tokens': peak_combined,
            'tpm_note': 'Rolling windows use response completion timestamps, not provider-side admission/rate-limit counters.',
            'jobs': result}


def collect_batch(batch_path, output):
    """Collect stopped task evidence through the shared engine-aware collector."""
    from scripts.collect_benchmark_batch import collect_batch as collect
    return collect(batch_path, output)


def interval_union(intervals):
    total = 0.; end = None
    for start, stop in sorted(intervals):
        if stop <= start: continue
        if end is None or start > end:
            total += stop-start; end = stop
        elif stop > end:
            total += stop-end; end = stop
    return total


def performance_report(batch_path):
    batch = json.loads((batch_path/'batch.json').read_text()); result = []
    for job in batch['jobs']:
        run = Path(job['run_directory']); db = run/'private/state/runtime.sqlite'
        with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as conn:
            starts = [v for v, in conn.execute("select created from events where kind='START' order by seq limit 1")]
            ends = [v for v, in conn.execute("select created from events where kind in ('POLICY_WALL_LIMIT','POLICY_BUDGET_CLOSED','RUN_SEALED') order by seq limit 1")]
            values = [json.loads(v) for v, in conn.execute('select value from actions')]
        if not starts:
            result.append({'case_id': job['case_id'], 'started': False}); continue
        start = starts[0]; end = ends[0] if ends else time.time(); elapsed = max(0., end-start)
        actions = [(max(start, v['started_at']), min(end, v.get('finished_at') or end)) for v in values if v.get('started_at')]
        spans = {}; finished = []
        path = run/'private/outputs/codex-host.jsonl'
        if path.exists():
            for line in path.open():
                try: event = json.loads(line)
                except json.JSONDecodeError: continue
                key = (event.get('thread_id'), event.get('turn_id'))
                if event.get('event') == 'turn/started': spans[key] = event['at']
                elif event.get('event') == 'turn/completed' and key in spans:
                    finished.append((max(start, spans.pop(key)), min(end, event['at'])))
        finished.extend((max(start, at), end) for at in spans.values())
        perf = run/'private/outputs/cpu-performance.jsonl'; latest = {}
        if perf.exists():
            for line in perf.open():
                try: latest = json.loads(line)
                except json.JSONDecodeError: continue
        action_time=interval_union(actions); turn_time=interval_union(finished)
        result.append({'case_id':job['case_id'],'elapsed_s':elapsed,'actions':len(values),
                       'any_action_wall_s':action_time, 'any_action_wall_fraction':action_time/elapsed if elapsed else None,
                       'any_model_turn_wall_s':turn_time, 'any_model_turn_wall_fraction':turn_time/elapsed if elapsed else None,
                       'owner_counters':latest,
                       'scope':'Action and model-turn wall intervals overlap; model-turn time includes tool waits and is not API latency. Owner counters start at owner launch and pump includes capture.'})
    return {'jobs':result}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    doctor_cmd = sub.add_parser('doctor')
    doctor_cmd.add_argument('--profile', choices=CPU_PROFILES, default='luna')
    preflight = sub.add_parser('preflight')
    preflight.add_argument('--profile', choices=CPU_PROFILES, default='luna')
    preflight.add_argument('--output', type=Path, help='Use the isolated Codex home for this future batch')
    assets = sub.add_parser('setup-assets')
    assets.add_argument('--archive', type=Path)
    collect = sub.add_parser('collect')
    collect.add_argument('batch', type=Path)
    collect.add_argument('--output', type=Path, required=True)
    smoke = sub.add_parser('l3-max-test', help='Four maximum-arm Level 3 engineering tasks, parallel 4, one hour each')
    smoke.add_argument('--profile', choices=CPU_PROFILES, default='luna')
    smoke.add_argument('--output', type=Path, required=True)
    smoke.add_argument('--prepare-only', action='store_true', help='Prepare the batch without model calls')
    smoke.add_argument('--wall-limit-s', type=float, default=3600, help='Per-task seconds after START (default 3600)')
    for command in ('prepare', 'submit'):
        prepare = sub.add_parser(command)
        prepare.add_argument('--profile', choices=CPU_PROFILES, default='luna')
        prepare.add_argument('--output', type=Path, required=True)
        selection = prepare.add_mutually_exclusive_group()
        selection.add_argument('--level', type=int, choices=(1, 2, 3), default=None)
        selection.add_argument('--all-levels', action='store_true', help='Select all three levels (36 cases without other filters)')
        prepare.add_argument('--min-arms', action='store_true')
        prepare.add_argument('--families', nargs='+', choices=('transport', 'mosaic', 'relay', 'dial'))
        prepare.add_argument('--cases', nargs='+', help='Select exact released case IDs')
        prepare.add_argument('--wall-limit-s', type=float, default=86400)
        if command == 'submit': prepare.add_argument('--parallel', type=int, default=36)
    for command in ('run', 'status', 'stop', 'usage', 'performance'):
        p = sub.add_parser(command); p.add_argument('batch', type=Path)
        if command == 'run': p.add_argument('--parallel', type=int, default=36)
    args = parser.parse_args()
    load_env(ROOT/'.env')
    if args.command == 'l3-max-test':
        from scripts.benchmark import select_cases
        rows = select_cases(level=3)
        cases = []
        for family in ('transport', 'mosaic', 'relay', 'dial'):
            candidates = [row for row in rows if row['family'] == family]
            if not candidates:
                raise SystemExit('No released Level 3 case for ' + family)
            maximum = max(row['arms'] for row in candidates)
            winners = [row for row in candidates if row['arms'] == maximum]
            if len(winners) != 1:
                raise SystemExit('Expected exactly one maximum-arm Level 3 case for ' + family)
            cases.append(winners[0]['case_id'])
        print('Level 3 maximum-arm engineering test: ' + ', '.join(cases), flush=True)
        args.command = 'prepare' if args.prepare_only else 'submit'
        args.level, args.min_arms, args.families = 3, False, None
        args.parallel, args.cases = 4, cases
    if args.command == 'setup-assets':
        from scripts.benchmark_handoff import setup_assets
        print(json.dumps(setup_assets(archive_path=args.archive), ensure_ascii=False, indent=2))
        return 0
    if args.command == 'collect':
        report = collect_batch(args.batch, args.output)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report['complete'] else 1
    batch = args.output if args.command in ('prepare', 'submit', 'preflight') else getattr(args, 'batch', None)
    # Legacy batches have no marker and retain the caller's existing CODEX_HOME.
    marker = Path(batch)/'codex-home.json' if batch else None
    if batch and args.command in ('prepare', 'submit', 'preflight'):
        home = batch_codex_home(batch)
    elif marker and marker.is_file():
        home = Path(json.loads(marker.read_text())['codex_home'])
    else:
        home = None
    if home is not None:
        home.mkdir(parents=True, exist_ok=True, mode=0o700)
        os.environ['CODEX_HOME'] = str(home)
    env = cpu_environment()
    if args.command == 'doctor':
        from scripts.benchmark_handoff import doctor
        report = doctor([args.profile])
        try:
            report['cpu_render'] = {'passed': True, **render_probe(env)}
        except (RuntimeError, subprocess.TimeoutExpired) as error:
            report['cpu_render'] = {'passed': False, 'error': str(error)}
        try:
            sandbox_probe(env); report['codex_sandbox'] = {'passed': True}
        except (RuntimeError, subprocess.TimeoutExpired) as error:
            report['codex_sandbox'] = {'passed': False, 'error': str(error)}
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report['cpu_render']['passed'] and report['codex_sandbox']['passed'] and report['profiles'][args.profile]['credential_set'] and report['assets']['missing'] == 0 and report['assets']['changed'] == 0 else 1
    if args.command == 'usage':
        print(json.dumps(usage_report(args.batch), ensure_ascii=False, indent=2))
        return 0
    if args.command == 'performance':
        print(json.dumps(performance_report(args.batch), ensure_ascii=False, indent=2))
        return 0
    if args.command == 'run':
        batch_info = json.loads((args.batch/'batch.json').read_text())
        names = {job.get('profile') for job in batch_info.get('jobs', [])}
        if len(names) != 1 or not names <= set(CPU_PROFILES):
            raise SystemExit('Expected a prepared batch with one supported profile.')
        selected_profile = names.pop()
    else:
        selected_profile = getattr(args, 'profile', 'luna')
    key_name = {'luna': 'OPENAI_API_KEY', 'openlux-terra': 'OPENLUX_API_KEY', 'openlux-luna': 'OPENLUX_API_KEY'}[selected_profile]
    if args.command in ('preflight', 'run', 'submit') and not env.get(key_name):
        raise SystemExit('Set ' + key_name + ' in .env or the process environment.')
    if args.command in ('preflight', 'run', 'submit'):
        sandbox_probe(env)
    if args.command in ('run', 'submit'):
        print('CPU renderer: ' + json.dumps(render_probe(env)), flush=True)
    cmd = [sys.executable, '-m', 'scripts.benchmark', 'prepare' if args.command == 'submit' else args.command]
    if args.command == 'preflight': cmd += ['--models', selected_profile]
    elif args.command in ('prepare', 'submit'):
        if not math.isfinite(args.wall_limit_s) or args.wall_limit_s <= 0: raise SystemExit('Wall limit must be positive and finite.')
        cmd += ['--models', selected_profile, '--wall-limit-s', str(args.wall_limit_s), '--output', str(args.output)]
        if args.level is not None: cmd += ['--level', str(args.level)]
        if args.min_arms: cmd += ['--min-arms']
        if args.families: cmd += ['--families', *args.families]
        if getattr(args, 'cases', None): cmd += ['--cases', *args.cases]
    else:
        cmd += [str(args.batch)]
        if args.command == 'run': cmd += ['--parallel', str(args.parallel)]
    result = subprocess.call(cmd, cwd=ROOT, env=env)
    if result == 0 and args.command in ('prepare', 'submit'):
        (Path(args.output)/'codex-home.json').write_text(json.dumps({'codex_home': str(home)}, indent=2)+'\n')
    if result == 0 and args.command == 'submit':
        result = subprocess.call([sys.executable, '-m', 'scripts.benchmark', 'run', str(args.output),
                                  '--parallel', str(args.parallel)], cwd=ROOT, env=env)
    return result


if __name__ == '__main__':
    sys.exit(main())
