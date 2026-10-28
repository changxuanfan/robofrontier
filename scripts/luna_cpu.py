"""Headless GPT-6 Luna benchmark entrypoint; no LLM supervisor required."""
import argparse
import json
import math
import os
from pathlib import Path
import platform
import sqlite3
import subprocess
import sys

from scripts.benchmark import load_env
from robofrontier.task_families import ROOT


def cpu_environment():
    env = os.environ.copy()
    env.update(MUJOCO_GL='osmesa', PYOPENGL_PLATFORM='osmesa',
               LIBGL_ALWAYS_SOFTWARE='true', GALLIUM_DRIVER='llvmpipe',
               CUDA_VISIBLE_DEVICES='')
    return env


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
                        for key in totals: totals[key] += payload.get('usage', {}).get(key, 0)
        totals['cache_hit_ratio'] = totals['cached_input_tokens']/totals['input_tokens'] if totals['input_tokens'] else None
        result.append({'case_id': job['case_id'], 'threads': len(ids), 'missing_traces': missing,
                       'records_without_response_id': missing_response_ids, **totals})
    total_input = sum(job['input_tokens'] for job in result)
    total_cached = sum(job['cached_input_tokens'] for job in result)
    return {'scope': 'Recorded unique API responses; active traces and billing may differ.',
            'total_input_tokens': total_input, 'total_cached_input_tokens': total_cached,
            'cache_hit_ratio': total_cached/total_input if total_input else None, 'jobs': result}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('doctor')
    sub.add_parser('preflight')
    for command in ('prepare', 'submit'):
        prepare = sub.add_parser(command)
        prepare.add_argument('--output', type=Path, required=True)
        prepare.add_argument('--level', type=int, choices=(1, 2, 3), default=1)
        prepare.add_argument('--min-arms', action='store_true')
        prepare.add_argument('--families', nargs='+', choices=('transport', 'mosaic', 'relay', 'dial'))
        prepare.add_argument('--wall-limit-s', type=float, default=600)
        if command == 'submit': prepare.add_argument('--parallel', type=int, default=4)
    for command in ('run', 'status', 'stop', 'usage'):
        p = sub.add_parser(command); p.add_argument('batch', type=Path)
        if command == 'run': p.add_argument('--parallel', type=int, default=4)
    args = parser.parse_args()
    load_env(ROOT/'.env')
    env = cpu_environment()
    if args.command == 'doctor':
        from scripts.benchmark_handoff import doctor
        report = doctor(['luna'])
        try:
            report['cpu_render'] = {'passed': True, **render_probe(env)}
        except (RuntimeError, subprocess.TimeoutExpired) as error:
            report['cpu_render'] = {'passed': False, 'error': str(error)}
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report['cpu_render']['passed'] and report['profiles']['luna']['credential_set'] and report['assets']['missing'] == 0 and report['assets']['changed'] == 0 else 1
    if args.command == 'usage':
        print(json.dumps(usage_report(args.batch), ensure_ascii=False, indent=2))
        return 0
    if args.command in ('preflight', 'run', 'submit') and not env.get('OPENAI_API_KEY'):
        raise SystemExit('Set OPENAI_API_KEY in .env or the process environment.')
    if args.command in ('run', 'submit'):
        if args.command == 'run':
            batch = json.loads((args.batch/'batch.json').read_text())
            if not batch.get('jobs') or any(job.get('profile') != 'luna' for job in batch['jobs']):
                raise SystemExit('This entrypoint accepts only a prepared Luna batch.')
        print('CPU renderer: ' + json.dumps(render_probe(env)), flush=True)
    cmd = [sys.executable, '-m', 'scripts.benchmark', 'prepare' if args.command == 'submit' else args.command]
    if args.command == 'preflight': cmd += ['--models', 'luna']
    elif args.command in ('prepare', 'submit'):
        if not math.isfinite(args.wall_limit_s) or args.wall_limit_s <= 0: raise SystemExit('Wall limit must be positive and finite.')
        cmd += ['--models', 'luna', '--level', str(args.level), '--wall-limit-s', str(args.wall_limit_s), '--output', str(args.output)]
        if args.min_arms: cmd += ['--min-arms']
        if args.families: cmd += ['--families', *args.families]
    else:
        cmd += [str(args.batch)]
        if args.command == 'run': cmd += ['--parallel', str(args.parallel)]
    result = subprocess.call(cmd, cwd=ROOT, env=env)
    if result == 0 and args.command == 'submit':
        result = subprocess.call([sys.executable, '-m', 'scripts.benchmark', 'run', str(args.output),
                                  '--parallel', str(args.parallel)], cwd=ROOT, env=env)
    return result


if __name__ == '__main__':
    sys.exit(main())
