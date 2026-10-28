"""Claude benchmark planning and offline readiness. Execution is not yet enabled."""
import argparse
import hashlib
import secrets
import sys
import time
import json
import os
from pathlib import Path
import shutil
import subprocess
from urllib.parse import urlsplit

from robofrontier.task_families import ROOT, RELEASE, read
from scripts.benchmark import load_env, select_cases
from scripts.benchmark_handoff import digest
from robofrontier.run_setup import write_json
from robofrontier.claude.runner import _oauth_endpoint_allowed
from scripts.tasks import check_case_assets

CATALOG = ROOT / 'configs/claude-models.json'


def profile(name, catalog=CATALOG):
    document = read(catalog)
    if document.get('schema') != 'robofrontier.claude-models.v1':
        raise ValueError('Unsupported Claude model catalog')
    try:
        selected = dict(document['profiles'][name])
    except KeyError as exc:
        raise ValueError('Unknown Claude model profile: ' + name) from exc
    if not all(isinstance(selected.get(k), str) and selected[k] for k in ('provider', 'model', 'env_key', 'effort')):
        raise ValueError('Incomplete model profile')
    if selected['effort'] not in ('low', 'medium', 'high', 'xhigh', 'max'):
        raise ValueError('Unsupported Claude effort')
    auth_mode=selected.get('auth_mode','api_key')
    if auth_mode not in ('api_key','oauth_token'):
        raise ValueError('Unsupported Claude authentication mode')
    if auth_mode=='oauth_token' and (selected['provider']!='anthropic' or selected['env_key']!='CLAUDE_CODE_OAUTH_TOKEN'):
        raise ValueError('OAuth token mode is limited to the Anthropic subscription profile')
    base = selected.get('base_url')
    if 'base_url_env' in selected:
        base = os.environ.get(selected['base_url_env'])
    if base:
        url = urlsplit(base)
        if url.scheme != 'https' or not url.hostname or url.username or url.password or url.query or url.fragment:
            raise ValueError('Base URL must be HTTPS without credentials, query or fragment')
    if auth_mode == 'oauth_token' and not _oauth_endpoint_allowed(base):
        raise ValueError('OAuth token may only be sent to the Anthropic endpoint')
    selected['base_url'] = base
    return selected


def plan(names, families=None, cases=None, level=None, min_arms=False, parallel=4, catalog=CATALOG):
    if not names or len(set(names)) != len(names):
        raise ValueError('Choose distinct model profiles')
    if not 1 <= parallel <= 36:
        raise ValueError('Parallel tasks must be 1..36')
    profiles = {name: profile(name, catalog) for name in names}
    rows = select_cases(families, cases, level, min_arms)
    jobs = [dict(profile=name, model=p['model'], provider=p['provider'], **row)
            for name, p in profiles.items() for row in rows]
    return dict(schema='robofrontier.claude-plan.v1', execution_available=False,
                model_calls=0, simulator_started=False, parallel=parallel, task_count=len(jobs), jobs=jobs,
                note='Planning only; providers and model capabilities are not verified.')



def prepare_draft(output, names, families=None, cases=None, level=None, min_arms=False, catalog=CATALOG, topology='adaptive', record_video=False):
    """Freeze engineering drafts only. No Claude runtime is permitted to start them yet."""
    from robofrontier.task_families import prepare, policy_task
    from robofrontier.suite.dial_feedback import alphabet

    if topology not in ('adaptive', 'centralized'):
        raise ValueError('Draft topology must be adaptive or centralized')
    if type(record_video) is not bool:
        raise ValueError('record_video must be a boolean')
    selection = plan(names, families, cases, level, min_arms, 1, catalog)
    profiles = {name: profile(name, catalog) for name in names}
    selected = list(dict.fromkeys(job['case_id'] for job in selection['jobs']))
    release = read(ROOT / RELEASE)
    released = {case['id']: case for family in release['families'].values() for case in family['gpt']}
    for case in selected:
        report = check_case_assets(released[case], ROOT)
        if report['missing'] or report['changed']:
            raise ValueError('Missing or changed scene assets for ' + case)
    if not (ROOT / 'vendor/panda/SOURCE.json').is_file():
        raise ValueError('Missing Panda source assets; restore the published policy asset bundle')
    output = Path(output).absolute()
    output.mkdir(parents=True, exist_ok=False)
    batch = {'schema': 'robofrontier.claude-draft.v1', 'status': 'preparing',
             'created_at': time.time(), 'execution_available': False, 'model_calls': 0, 'jobs': []}
    write_json(output / 'batch.json', batch)
    puzzles = {}
    for case in selected:
        task = policy_task(case)
        if task.get('family') == 'dial':
            target = output / 'private/dial' / (case + '.json')
            write_json(target, {'code_symbols': [secrets.choice(alphabet(task)) for _ in task['dials']]})
            puzzles[case] = target
    for name, p in profiles.items():
        cfg = read(ROOT / 'configs/harnesses/policy.json')
        cfg.update(engine='claude', topology=topology, model=p['model'], effort=p['effort'], observation_renderer='classic', record_realtime_video=record_video)
        cfg_path = output / 'profiles' / name / 'harness.json'
        write_json(cfg_path, cfg)
        for case in selected:
            run = output / name / case
            prepare(case, run, cfg_path, puzzles.get(case))
            inputs = run / 'private/inputs'
            launch = inputs / 'claude.json'
            write_json(launch, {'schema': 'robofrontier.claude-launch-draft.v1', 'profile': p,
                                'qualified': False, 'execution_available': False})
            prompt = run / 'public/inputs/claude-controller.md'
            prompt.write_text((ROOT / 'prompts/claude/controller.md').read_text())
            manifest_path = inputs / 'manifest.json'
            manifest = read(manifest_path)
            for file in (launch, prompt):
                manifest['input_sha256'][str(file.relative_to(run))] = digest(file)
            for file in (ROOT / 'scripts/claude_benchmark.py', ROOT / 'configs/claude-models.json'):
                relative = file.relative_to(ROOT)
                frozen = inputs / 'frozen-source' / relative
                frozen.parent.mkdir(parents=True, exist_ok=True)
                frozen.write_bytes(file.read_bytes())
                manifest['source_sha256'][str(relative)] = digest(file)
            manifest.update(engine='claude', status='engineering draft; not qualified or runnable')
            write_json(manifest_path, manifest)
            batch['jobs'].append({'profile': name, 'case_id': case, 'run_directory': str(run), 'status': 'draft'})
            write_json(output / 'batch.json', batch)
    batch['status'] = 'prepared_not_qualified'
    write_json(output / 'batch.json', batch)
    return output / 'batch.json'


def doctor(names, root=ROOT, catalog=CATALOG):
    load_env(root / '.env')
    binary = shutil.which('claude')
    cli = {'available': False}
    if binary:
        try:
            result = subprocess.run([binary, '--version'], capture_output=True, text=True, timeout=15)
            cli = {'available': result.returncode == 0, 'version': (result.stdout or result.stderr).strip().splitlines()[0]}
        except (OSError, subprocess.TimeoutExpired, IndexError):
            cli = {'available': False}
    profiles = {}
    for name in names:
        p = profile(name, catalog)
        profiles[name] = {'model': p['model'], 'provider': p['provider'], 'credential_env': p['env_key'],
                          'credential_set': bool(os.environ.get(p['env_key'])),
                          'base_url_configured': bool(p['base_url']), 'live_verified': False}
    assets = [check_case_assets(case, root) for family in read(root / RELEASE)['families'].values() for case in family['gpt']]
    return {'claude': cli, 'profiles': profiles, 'release_sha256': digest(root / RELEASE),
            'assets': {'missing': sum(len(a['missing']) for a in assets), 'changed': sum(len(a['changed']) for a in assets)},
            'recording_tools': {name: bool(shutil.which(name)) for name in ('ffmpeg', 'ffprobe')},
            'panda_manifest_available': (root / 'vendor/panda/SOURCE.json').is_file(),
            'execution_available': False,
            'note': 'Offline checks only; no credential validation, model calls or physics.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('models')
    p = sub.add_parser('doctor'); p.add_argument('--models', nargs='+', default=['haiku', 'opus', 'opus_subscription', 'kimi', 'kimi_code', 'qwen', 'glm', 'deepseek'])
    for command in ('plan', 'prepare-draft'):
        p = sub.add_parser(command); p.add_argument('--models', nargs='+', required=True)
        p.add_argument('--families', nargs='+'); p.add_argument('--cases', nargs='+')
        p.add_argument('--level', type=int, choices=(1, 2, 3)); p.add_argument('--min-arms', action='store_true')
        if command == 'plan': p.add_argument('--parallel', type=int, default=4)
        else:
            p.add_argument('--output', type=Path, required=True)
            p.add_argument('--topology', choices=('adaptive', 'centralized'), default='adaptive',
                           help='Engineering draft topology; neither option qualifies a scored run')
            p.add_argument('--record-video', action='store_true', help='Record a real-time MP4 during the engineering run')
    p = sub.add_parser('run-engineering', help='Unqualified centralized or adaptive experiments; may spend API funds')
    p.add_argument('batch', type=Path); p.add_argument('--parallel', type=int, required=True)
    p.add_argument('--turn-budget-usd', type=float, required=True)
    p.add_argument('--task-budget-usd', type=float, required=True)
    p.add_argument('--wall-limit-s', type=float, required=True)
    p.add_argument('--confirm-engineering-spend', action='store_true',
                   help='Acknowledge model calls and estimated costs for this engineering batch')
    for command in ('status-engineering', 'stop-engineering'):
        p = sub.add_parser(command); p.add_argument('batch', type=Path)
    args = parser.parse_args()
    if args.command == 'models': result = read(CATALOG)
    elif args.command == 'doctor': result = doctor(args.models)
    elif args.command == 'status-engineering':
        from robofrontier.claude.engineering_batch import status_batch
        result = status_batch(args.batch)
    elif args.command == 'stop-engineering':
        from robofrontier.claude.engineering_batch import stop_batch
        result = stop_batch(args.batch)
    elif args.command == 'run-engineering':
        if not args.confirm_engineering_spend:
            result = {'status': 'not_started', 'reason': 'Missing --confirm-engineering-spend',
                      'model_calls': 0, 'simulator_started': False}
        else:
            from robofrontier.claude.engineering_batch import run_batch
            load_env(ROOT / '.env')
            import signal
            import threading
            stop = threading.Event()
            previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
            def request_stop(signum, frame):
                stop.set()
            try:
                for sig in previous: signal.signal(sig, request_stop)
                result = run_batch(args.batch, dict(os.environ), parallel=args.parallel,
                                   turn_budget_usd=args.turn_budget_usd, total_budget_usd=args.task_budget_usd,
                                   wall_limit_s=args.wall_limit_s, stop_requested=stop.is_set)
            finally:
                for sig, handler in previous.items(): signal.signal(sig, handler)
    else:
        load_env(ROOT / '.env')
        if args.command == 'plan':
            result = plan(args.models, args.families, args.cases, args.level, args.min_arms, args.parallel)
        else:
            result = {'batch': str(prepare_draft(args.output, args.models, args.families, args.cases, args.level, args.min_arms,
                                                  topology=args.topology, record_video=args.record_video)),
                      'execution_available': False}
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__': main()
