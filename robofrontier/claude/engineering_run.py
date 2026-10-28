"""Experimental single-run launcher; never admits a scored benchmark batch.

Starting this module may call a paid model. It is deliberately not wired into
benchmark run commands. An engineering draft can be consumed once, never replayed.
"""
import fcntl
from functools import partial
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from urllib.parse import urlsplit

from robofrontier.claude.policy_host import CentralizedPolicyHost
from robofrontier.claude.adaptive_policy_host import AdaptivePolicyHost
from robofrontier.claude.budget import BudgetLedger
from robofrontier.claude.session_host import ClaudeSessionHost
from robofrontier.claude.supervisor import CentralizedSupervisor
from robofrontier.run_seal import seal_run
from robofrontier.run_setup import verify_inputs, write_json
from robofrontier.task_config import ROOT


def _positive(value, name, limit):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= limit:
        raise ValueError('Invalid ' + name)


def run_engineering(run_dir, base_env, *, turn_budget_usd, total_budget_usd, wall_limit_s, startup_timeout_s=120,
                    poll_s=0.5, owner_factory=subprocess.Popen, policy_factory=CentralizedPolicyHost,
                    adaptive_policy_factory=AdaptivePolicyHost,
                    clock=time.monotonic, sleep=time.sleep):
    """Run one explicit engineering draft and conservatively retain uncertain state.

    Total budget is a best-effort limit based on CLI-reported cost, not billing.
    The caller must authorize model calls before invoking this function.
    """
    for val, name, limit in ((turn_budget_usd, 'per-turn budget', 1000),
                             (total_budget_usd, 'task budget', 1000),
                             (wall_limit_s, 'wall limit', 86400),
                             (startup_timeout_s, 'startup timeout', 86400),
                             (poll_s, 'poll interval', 60)):
        _positive(val, name, limit)
    run = Path(run_dir).resolve()
    launch = json.loads((run / 'private/inputs/claude.json').read_text())
    manifest = json.loads((run / 'private/inputs/manifest.json').read_text())
    if launch.get('schema') != 'robofrontier.claude-launch-draft.v1' or launch.get('qualified') is not False or launch.get('execution_available') is not False:
        raise ValueError('Only unqualified engineering drafts are accepted')
    if manifest.get('status') != 'engineering draft; not qualified or runnable' or manifest.get('engine') != 'claude':
        raise ValueError('Expected a frozen Claude engineering draft')
    changed = verify_inputs(run)
    if changed: raise ValueError('Frozen inputs changed: ' + ', '.join(changed))
    profile = launch['profile']
    config = json.loads((run / 'private/inputs/harness.json').read_text())
    topology = config.get('topology')
    if topology not in ('centralized', 'adaptive') or config.get('engine') != 'claude':
        raise ValueError('Expected a centralized or adaptive Claude engineering draft')
    if not base_env.get(profile['env_key']): raise ValueError('Missing credential variable: ' + profile['env_key'])
    if (run / 'STOP').exists(): raise ValueError('Run was stopped before launch')
    host_factory = partial(ClaudeSessionHost, max_budget_usd=turn_budget_usd, total_budget_usd=total_budget_usd)
    policy = None
    ledger = None
    policy_closed = False
    owner = None
    result = None
    claimed = False
    claim = run / 'private/state/claude-engineering-claim.json'
    status = run / 'private/outputs/claude-engineering-status.json'
    lock = (run / 'private/state/claude-engineering.lock').open('a+')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        # Recheck under the lock; a claim survives crashes and bars all retries.
        if claim.exists(): raise ValueError('Run already claimed; never restart the world')
        if (run / 'STOP').exists(): raise ValueError('Run was stopped before launch')
        if verify_inputs(run): raise ValueError('Frozen inputs changed before launch')
        if topology == 'centralized':
            policy = policy_factory(run, profile, base_env, host_factory=host_factory)
        with claim.open('x') as out:
            os.fchmod(out.fileno(), 0o600)
            json.dump({'claimed_at': time.time(), 'pid': os.getpid(), 'mode': 'engineering'}, out)
            out.flush(); os.fsync(out.fileno())
        claimed = True
        if topology == 'adaptive':
            ledger = BudgetLedger.create(run / 'private/state/claude-budget.sqlite', total_budget_usd)
            policy = adaptive_policy_factory(run, profile, base_env, ledger,
                                             turn_budget_usd=turn_budget_usd,
                                             setup_budget_usd=turn_budget_usd)
        write_json(status, {'status': 'starting', 'claimed_at': time.time(), 'scored': False,
                            'turn_budget_usd': turn_budget_usd, 'task_budget_usd': total_budget_usd,
                            'wall_limit_s': wall_limit_s})
        # Do not pass model credentials to the physical owner.
        owner_env = {k: v for k, v in base_env.items() if k in ('PATH', 'HOME', 'LANG', 'LC_ALL', 'TMPDIR', 'DISPLAY', 'MUJOCO_GL', 'PYTHONPATH', 'DYLD_LIBRARY_PATH')}
        with (run / 'private/outputs/owner.log').open('a') as log:
            owner = owner_factory([sys.executable, '-m', 'robofrontier.pipeline', '--run-dir', str(run), '--hidden'],
                                  cwd=ROOT, env=owner_env, stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                                  start_new_session=True)
        write_json(status, {'status': 'waiting_for_owner', 'owner_pid': owner.pid, 'scored': False})
        deadline = clock() + startup_timeout_s
        connection = run / 'private/state/connections/Meta.json'
        while True:
            if owner.poll() is not None: raise RuntimeError('Physical owner exited before publishing its connection')
            if (run / 'STOP').exists(): raise RuntimeError('Stop requested during owner startup')
            if clock() >= deadline: raise TimeoutError('Physical owner startup timed out')
            info = json.loads(connection.read_text())
            if info.get('url') and policy.store.get('execution_owner_attached'):
                url = urlsplit(info['url'])
                if url.scheme != 'http' or url.hostname != '127.0.0.1' or not url.port or url.path or url.query or url.fragment:
                    raise ValueError('Expected the owner local loopback endpoint')
                break
            sleep(poll_s)
        write_json(status, {'status': 'running', 'owner_pid': owner.pid, 'scored': False})
        supervisor = CentralizedSupervisor(policy, lambda: owner.poll() is None,
                                            stop_requested=lambda: (run / 'STOP').exists(),
                                            startup_timeout_s=startup_timeout_s, wall_limit_s=wall_limit_s,
                                            poll_s=poll_s, clock=clock, sleep=sleep)
        result = supervisor.run()
    except Exception as error:
        if not claimed:
            raise
        result = {'reason': 'launch_error', 'error_type': type(error).__name__, 'sealed': False}
        if owner is not None and policy is not None and policy.store.get('execution_owner_attached'):
            try: result['seal'] = seal_run(policy.store, 30)
            except Exception as seal_error: result['seal_error_type'] = type(seal_error).__name__
        # A still-live owner is not force-killed: its state must be reviewed.
    finally:
        try:
            if policy is not None: policy.close()
            policy_closed = True
        except Exception:
            if result is not None: result['policy_close_error'] = True
        if ledger is not None and policy_closed:
            ledger.close()
        if result is not None:
            if owner is not None and (result.get('seal') or {}).get('sealed') and owner.poll() is None:
                try: owner.wait(timeout=5)
                except (subprocess.TimeoutExpired, AttributeError): pass
            owner_alive = bool(owner and owner.poll() is None)
            video_required = config.get('record_realtime_video', False) is True
            video_readable = (result.get('seal') or {}).get('video', {}).get('readable') is True
            closed = ((result.get('seal') or {}).get('sealed') and not owner_alive
                      and not result.get('policy_close_error') and not result.get('error_type')
                      and (not video_required or video_readable)
                      and result.get('reason') in ('world_ended', 'stop_requested', 'wall_limit'))
            result.update(status='finished' if closed else 'needs_review', owner_alive=owner_alive, scored=False,
                          video_required=video_required)
            write_json(status, result)
        fcntl.flock(lock, fcntl.LOCK_UN); lock.close()
    return result
