"""Single-use, unqualified engineering batches; not a scored benchmark runner."""
from concurrent.futures import FIRST_COMPLETED, ThreadPoolExecutor, wait
import fcntl
import json
from pathlib import Path
import re
import time

from robofrontier.claude.engineering_run import _positive, run_engineering
from robofrontier.claude.runner import build_env
from robofrontier.run_setup import verify_inputs, write_json

ID = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z')


def _jobs(folder):
    folder = Path(folder).resolve()
    batch = json.loads((folder / 'batch.json').read_text())
    if batch.get('schema') != 'robofrontier.claude-draft.v1' or batch.get('status') != 'prepared_not_qualified' or batch.get('execution_available') is not False:
        raise ValueError('Expected a completed unqualified engineering draft batch')
    jobs = batch.get('jobs')
    if not isinstance(jobs, list) or not jobs: raise ValueError('Batch has no jobs')
    seen = set()
    for job in jobs:
        name, case = job.get('profile'), job.get('case_id')
        if not isinstance(name, str) or not ID.fullmatch(name) or not isinstance(case, str) or not ID.fullmatch(case):
            raise ValueError('Invalid profile or case name')
        expected = (folder / name / case).resolve()
        if not expected.is_relative_to(folder) or Path(job['run_directory']).resolve() != expected or expected in seen:
            raise ValueError('Unexpected or duplicate run directory')
        if not expected.is_dir(): raise ValueError('Run directory missing')
        seen.add(expected)
    return jobs


def _preflight(jobs, env, verifier):
    for job in jobs:
        run = Path(job['run_directory'])
        if (run / 'STOP').exists() or (run / 'private/state/claude-engineering-claim.json').exists():
            raise ValueError('A run is stopped or already claimed')
        changed = verifier(run)
        if changed: raise ValueError('Frozen inputs changed: ' + ', '.join(changed))
        launch = json.loads((run / 'private/inputs/claude.json').read_text())
        config = json.loads((run / 'private/inputs/harness.json').read_text())
        profile = launch['profile']
        if (launch.get('schema') != 'robofrontier.claude-launch-draft.v1' or launch.get('qualified') is not False
                or launch.get('execution_available') is not False or config.get('engine') != 'claude'
                or config.get('topology') not in ('centralized', 'adaptive') or config.get('model') != profile.get('model')
                or config.get('effort') != profile.get('effort')):
            raise ValueError('Run does not match the engineering launch profile')
        if not env.get(profile.get('env_key')): raise ValueError('Missing provider credential variable')
        if profile.get('provider') != 'anthropic' and not profile.get('base_url'):
            raise ValueError('Third-party provider base URL is missing')
        # Validate authentication for every job before claiming or launching any.
        # The returned environment is deliberately discarded and never logged.
        build_env(profile, env, run / 'private/claude-config')


def status_batch(folder):
    folder = Path(folder).resolve()
    jobs = _jobs(folder)
    state_file = folder / 'engineering-status.json'
    state = json.loads(state_file.read_text()) if state_file.exists() else {'status': 'not_started'}
    lock_path = folder / 'engineering-supervisor.lock'
    live = False
    if lock_path.exists():
        with lock_path.open('r') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                live = True
            else:
                fcntl.flock(lock, fcntl.LOCK_UN)
    state['supervisor_live'] = live
    if not state_file.exists() and (folder / 'engineering-claim.json').exists():
        state['status'] = 'starting' if live else 'claim_without_status_needs_review'
    elif not state_file.exists() and (folder / 'STOP').exists():
        state['status'] = 'stopped_before_start'
    if state.get('status') == 'running' and not live:
        state['status'] = 'supervisor_lost_needs_review'
    state['runs'] = []
    for job in jobs:
        path = Path(job['run_directory']) / 'private/outputs/claude-engineering-status.json'
        state['runs'].append({'profile': job['profile'], 'case_id': job['case_id'],
                              'detail': json.loads(path.read_text()) if path.exists() else None})
    return state


def stop_batch(folder):
    folder = Path(folder).resolve()
    jobs = _jobs(folder)  # Validate every path before changing any stop marker.
    (folder / 'STOP').touch()
    for job in jobs: (Path(job['run_directory']) / 'STOP').touch()
    return {'stop_requested': True, 'jobs': len(jobs), 'note': 'Request only; inspect status and sealing outcome.'}


def run_batch(folder, base_env, *, parallel, turn_budget_usd, total_budget_usd, wall_limit_s,
              runner=run_engineering, verifier=verify_inputs):
    """Run drafts once. Caller explicitly authorizes potential paid model calls."""
    folder = Path(folder).resolve()
    if type(parallel) is not int or not 1 <= parallel <= 36: raise ValueError('Parallel must be 1..36')
    _positive(turn_budget_usd, 'per-turn budget', 1000)
    _positive(total_budget_usd, 'per-task budget', 1000)
    _positive(wall_limit_s, 'wall limit', 86400)
    jobs = _jobs(folder)
    with (folder / 'engineering-supervisor.lock').open('a+') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        claim = folder / 'engineering-claim.json'
        if claim.exists(): raise ValueError('Batch already claimed; never restart')
        if (folder / 'STOP').exists(): raise ValueError('Batch was stopped before launch')
        _preflight(jobs, base_env, verifier)
        with claim.open('x') as out:
            import os
            os.fchmod(out.fileno(), 0o600)
            json.dump({'claimed_at': time.time(), 'pid': os.getpid()}, out)
            out.flush(); os.fsync(out.fileno())
        state = {'schema': 'robofrontier.claude-engineering-status.v1', 'status': 'running',
                 'started_at': time.time(), 'parallel': parallel, 'scored': False,
                 'turn_budget_usd': turn_budget_usd, 'task_budget_usd': total_budget_usd, 'wall_limit_s': wall_limit_s,
                 'jobs': [{'profile': j['profile'], 'case_id': j['case_id'], 'status': 'queued'} for j in jobs]}
        state_path = folder / 'engineering-status.json'
        write_json(state_path, state)
        next_job = 0
        pending = {}
        with ThreadPoolExecutor(max_workers=parallel) as pool:
            while next_job < len(jobs) or pending:
                stopping = (folder / 'STOP').exists()
                if stopping:
                    for job in jobs: (Path(job['run_directory']) / 'STOP').touch()
                while not stopping and next_job < len(jobs) and len(pending) < parallel:
                    # A stop request can arrive while other jobs are being submitted.
                    if (folder / 'STOP').exists():
                        stopping = True
                        break
                    index = next_job; next_job += 1
                    state['jobs'][index]['status'] = 'starting'
                    write_json(state_path, state)
                    future = pool.submit(runner, jobs[index]['run_directory'], base_env,
                                         turn_budget_usd=turn_budget_usd, total_budget_usd=total_budget_usd,
                                         wall_limit_s=wall_limit_s)
                    pending[future] = index
                if stopping and next_job < len(jobs):
                    for index in range(next_job, len(jobs)): state['jobs'][index]['status'] = 'stopped_before_start'
                    next_job = len(jobs); write_json(state_path, state)
                if not pending: break
                done, _ = wait(pending, timeout=.25, return_when=FIRST_COMPLETED)
                for future in done:
                    index = pending.pop(future)
                    try:
                        result = future.result()
                        state['jobs'][index]['status'] = 'closed' if result.get('status') == 'finished' else 'needs_review'
                        state['jobs'][index]['reason'] = result.get('reason')
                    except Exception as error:
                        state['jobs'][index].update(status='needs_review', error_type=type(error).__name__)
                    write_json(state_path, state)
        if any(j['status'] == 'needs_review' for j in state['jobs']):
            state['status'] = 'needs_review'
        elif any(j['status'] == 'stopped_before_start' or j.get('reason') == 'stop_requested'
                 for j in state['jobs']):
            state['status'] = 'stopped'
        else:
            state['status'] = 'closed'
        state['finished_at'] = time.time()
        write_json(state_path, state)
        return state
