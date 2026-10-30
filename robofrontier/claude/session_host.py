"""Role-bound Claude session lifecycle. This is not a benchmark launcher.

The supervisor creates one host per role. Native turns are asynchronous so the
physical owner and deterministic scheduler can keep progressing. Unknown turn
outcomes are terminal for this host: never issue another turn automatically.
"""
import json
import math
import os
from decimal import Decimal, InvalidOperation
from pathlib import Path
import re
import sys
import threading
import uuid

from robofrontier.claude.runner import PreLaunchCancelled, run_turn
from robofrontier.run_setup import verify_inputs, write_json

ACTOR_NAME = re.compile(r'[A-Za-z][A-Za-z0-9_-]{0,63}')


def role_mcp_config(run_dir, actor, python=sys.executable, *, _validated_delegate=False):
    """Write a host-owned connection bound to exactly one existing policy role."""
    run = Path(run_dir).resolve()
    if not isinstance(actor, str) or not ACTOR_NAME.fullmatch(actor) or actor == 'Meta':
        raise ValueError('Expected a policy role name')
    roles = json.loads((run / 'public/inputs/roles.json').read_text())['roles']
    if actor not in roles:
        if not _validated_delegate:
            raise ValueError('Role is not in the frozen policy team')
        from robofrontier.runtime_store import RuntimeStore
        path = run / 'private/state/runtime.sqlite'
        if not path.is_file(): raise ValueError('Runtime state missing')
        store = RuntimeStore(path)
        try:
            agent = store.agent(actor)
            if agent['kind'] != 'delegate' or agent.get('delegation_state') != 'active':
                raise ValueError('Delegate is not active')
        finally: store.close()
    elif roles[actor].get('kind') == 'manager':
        raise ValueError('Role is not in the frozen policy team')
    connection = run / 'private/state/connections' / (actor + '.json')
    info = json.loads(connection.read_text())
    if info.get('actor') != actor or info.get('run_id') != run.name or info.get('schema_version') != 3:
        raise ValueError('Role connection does not match this run')
    target = run / 'private/claude-mcp' / (actor + '.json')
    # A model never chooses the actor or a connection path. The server binds it.
    project = Path(__file__).resolve().parents[2]
    write_json(target, {'mcpServers': {'robot': {
        'command': str(Path(python).resolve()),
        'args': ['-m', 'robofrontier.claude.robot_mcp', '--run-dir', str(run), '--connection', str(connection)],
        'env': {'PYTHONPATH': str(project)},
    }}})
    return target


class ClaudeSessionHost:
    """Own one policy role's CLI turns; lifecycle events are never task results."""
    def __init__(self, run_dir, actor, profile, base_env, *, python=sys.executable,
                 binary='claude', turn_timeout=None, max_budget_usd=None, total_budget_usd=None,
                 budget_ledger=None, runner=run_turn, _validated_delegate=False):
        self.run = Path(run_dir).resolve()
        self.actor = actor
        self.profile = dict(profile)
        self.base_env = dict(base_env)
        self.binary = binary
        self.turn_timeout = turn_timeout
        self.max_budget_usd = max_budget_usd
        if budget_ledger is not None and (total_budget_usd is not None or max_budget_usd is None):
            raise ValueError('Shared ledger requires a per-turn allowance and no local total budget')
        self.budget_ledger = budget_ledger
        if total_budget_usd is not None:
            if isinstance(total_budget_usd, bool): raise ValueError('Invalid total budget')
            try: total_budget = Decimal(str(total_budget_usd))
            except InvalidOperation as error: raise ValueError('Invalid total budget') from error
            if not total_budget.is_finite() or total_budget <= 0: raise ValueError('Invalid total budget')
        else:
            total_budget = None
        self.total_budget = total_budget
        self.reported_spend = Decimal('0')
        self.budget_status = 'tracking' if total_budget is not None or budget_ledger is not None else 'not_set'
        self.runner = runner
        self.mcp = role_mcp_config(self.run, actor, python, _validated_delegate=_validated_delegate)
        self.session_id = str(uuid.uuid4())
        self.lock = threading.RLock()
        self.state = 'new'
        self.turn_id = None
        self.last_result = None
        self.worker = None
        self.cancel_event = threading.Event()

    @classmethod
    def adopt_verified_fork(cls, run_dir, actor, profile, base_env, *, budget_ledger, max_budget_usd,
                            _verifier=verify_inputs, **kwargs):
        """Resume one host-created fork only after runtime activation and identity checks.

        This is an engineering primitive, not an adaptive admission switch. Its
        local evidence assumes the host and runtime are trusted by the OS user.
        """
        from robofrontier.claude.fork_setup import ACTOR
        from robofrontier.claude.provenance import fork_provenance
        from robofrontier.delegation import Delegations
        from robofrontier.runtime_store import RuntimeStore
        run = Path(run_dir).resolve()
        if not isinstance(actor, str) or not ACTOR.fullmatch(actor): raise ValueError('Invalid delegate actor')
        if budget_ledger is None or not Path(budget_ledger.path).resolve().is_relative_to(run/'private'):
            raise ValueError('A run-local shared budget ledger is required')
        if _verifier(run): raise ValueError('Frozen inputs changed')
        launch = json.loads((run/'private/inputs/claude.json').read_text())
        if launch.get('profile') != profile:
            raise ValueError('Delegate profile differs from frozen launch profile')
        claim_path = run/'private/claude-forks'/(actor+'.json')
        claim = json.loads(claim_path.read_text())
        if claim.get('status') != 'history_verified_no_authority' or claim.get('actor') != actor:
            raise ValueError('Fork creation was not verified')
        store_path = run/'private/state/runtime.sqlite'
        if not store_path.is_file(): raise ValueError('Runtime state missing')
        store = RuntimeStore(store_path)
        try:
            config = store.get('config')
            if (config.get('engine') != 'claude' or config.get('topology') != 'adaptive'
                    or config.get('model') != profile.get('model') or config.get('effort') != profile.get('effort')
                    or store.get('lifecycle') != 'running' or store.get('dispatch_paused',False)):
                raise ValueError('Run or profile cannot adopt a delegate')
            agent = store.agent(actor); batch = Delegations(store).batch(claim['batch_id'])
            child = next((c for c in batch['children'] if c['actor'] == actor), None)
            if (agent.get('kind') != 'delegate' or agent.get('delegation_state') != 'active'
                    or batch['status'] != 'active' or batch['claimed_by'] != claim.get('owner')
                    or not child or not child.get('verified') or child.get('start_status') != 'pending'
                    or child['thread_id'] != claim.get('child_session') or agent['thread_id'] != child['thread_id']
                    or batch['source_thread_id'] != claim.get('parent_session')
                    or batch.get('source_provenance') != claim.get('proof')):
                raise ValueError('Delegate runtime identity or activation mismatch')
            call = store.call(claim['batch_id'], 'Controller')
            if call['op'] != 'delegate' or call['status'] != 'accepted':
                raise ValueError('Parent delegation boundary is no longer pending')
            if fork_provenance(claim['parent_session'], claim['child_session'], run/'private/claude-config') != claim['proof']:
                raise ValueError('Fork history changed after verification')
            # A consumed claim survives crashes; an uncertain child is never resent.
            adopted = run/'private/claude-forks'/(actor+'.adopted')
            with adopted.open('x') as out:
                os.fchmod(out.fileno(), 0o600)
                json.dump({'actor': actor, 'session_id': child['thread_id'], 'delegation_id': claim['batch_id']}, out)
                out.flush(); os.fsync(out.fileno())
        finally: store.close()
        host = cls(run, actor, profile, base_env, budget_ledger=budget_ledger,
                   max_budget_usd=max_budget_usd, _validated_delegate=True, **kwargs)
        host.session_id = child['thread_id']
        host.state = 'idle'  # The no-tool creation turn completed under this exact ID.
        return host

    def send(self, prompt):
        """Start an initial or resumed turn, returning once thread launch is known.

        A failed thread start restores the previous state. Any uncertainty after
        the worker starts blocks further turns and requires engineering review.
        """
        if not isinstance(prompt, str) or not prompt.strip():
            raise ValueError('A nonempty prompt is required')
        with self.lock:
            if self.state not in ('new', 'idle'):
                raise ValueError('Role session is active or requires review')
            previous = self.state
            turn_id = str(uuid.uuid4())
            turn_budget = self.max_budget_usd
            if self.budget_ledger is not None:
                turn_budget = self.budget_ledger.reserve(turn_id, self.actor, self.max_budget_usd)
            if self.total_budget is not None:
                remaining = self.total_budget - self.reported_spend
                if self.budget_status != 'tracking' or remaining <= 0:
                    raise ValueError('Task budget exhausted or cost unknown')
                turn_budget = min(float(remaining), turn_budget) if turn_budget is not None else float(remaining)
            self.state, self.turn_id = 'active', turn_id
            worker = threading.Thread(target=self._run, args=(prompt, previous, turn_id, turn_budget), daemon=True)
            self.worker = worker
            try:
                worker.start()
            except Exception:
                self.state, self.turn_id, self.worker = previous, None, None
                if self.budget_ledger is not None:
                    self.budget_ledger.abandon_before_launch(turn_id)
                raise
            return {'threadId': self.session_id, 'turnId': turn_id}

    def _run(self, prompt, previous, turn_id, turn_budget):
        before_launch = False
        try:
            record = self.runner(
                self.profile, self.profile['model'], self.profile['effort'], self.mcp,
                prompt, self.run, self.base_env,
                session_id=self.session_id if previous == 'new' else None,
                resume=self.session_id if previous == 'idle' else None,
                timeout=self.turn_timeout, max_budget_usd=turn_budget,
                binary=self.binary, events_name=f'{self.actor}-{turn_id}.jsonl',
                cancel_event=self.cancel_event)
            summary = record.get('summary') or {}
            if record.get('process') != 'exited' or record.get('exit_code') != 0:
                state = 'error' if summary.get('turn_status') == 'failed' else 'unknown'
            elif summary.get('session_id') != self.session_id:
                state = 'unknown'
            elif summary.get('turn_status') == 'ended' and not summary.get('malformed_lines'):
                state = 'idle'
            elif summary.get('turn_status') == 'failed':
                state = 'error'
            else:
                state = 'unknown'
        except PreLaunchCancelled:
            before_launch = True
            record = {'process': 'not_launched', 'error_type': 'PreLaunchCancelled'}
            state = 'unknown'
        except Exception as error:
            # Do not record exception text: provider errors can contain credentials.
            record = {'process': 'unknown', 'error_type': type(error).__name__}
            state = 'unknown'
        with self.lock:
            if self.cancel_event.is_set():
                state = 'unknown'
            if self.budget_ledger is not None:
                summary = record.get('summary') or {}
                known = (not self.cancel_event.is_set() and record.get('process') == 'exited'
                         and record.get('exit_code') == 0
                         and summary.get('session_id') == self.session_id
                         and summary.get('turn_status') in ('ended', 'failed')
                         and not summary.get('malformed_lines'))
                try:
                    if before_launch:
                        self.budget_ledger.abandon_before_launch(turn_id)
                        shared = self.budget_ledger.snapshot()
                    else:
                        shared = self.budget_ledger.settle(turn_id, summary.get('total_cost_usd'), outcome_known=known)
                    self.budget_status = shared['status']
                    self.reported_spend = Decimal(shared['reported_spend_usd'])
                    if shared['status'] == 'uncertain': state = 'unknown'
                    if shared['status'] == 'exhausted' and not any(r['status'] == 'reserved' for r in shared['reservations']):
                        if state == 'idle': state = 'error'
                except Exception:
                    self.budget_status = 'cost_unknown'
                    state = 'unknown'
            if self.total_budget is not None and not before_launch:
                cost = (record.get('summary') or {}).get('total_cost_usd')
                try:
                    amount = Decimal(str(cost)) if not isinstance(cost, bool) else Decimal('NaN')
                    if not amount.is_finite() or amount <= 0: raise InvalidOperation
                except (InvalidOperation, ValueError):
                    self.budget_status = 'cost_unknown'
                    state = 'unknown'
                else:
                    self.reported_spend += amount
                    if self.reported_spend >= self.total_budget:
                        self.budget_status = 'exhausted'
                        if state == 'idle': state = 'error'
            accounting = {'status': self.budget_status, 'reported_spend_usd': str(self.reported_spend),
                          'limit_usd': str(self.total_budget) if self.total_budget is not None else
                                       shared['limit_usd'] if self.budget_ledger is not None and 'shared' in locals() else None,
                          'source': 'cli_reported_estimate_not_provider_bill'}
            self.last_result = record
            try:
                write_json(self.run / 'private/evidence' / f'{self.actor}-{turn_id}-summary.json',
                           {'actor': self.actor, 'session_id': self.session_id, 'turn_id': turn_id,
                            'status': state, 'record': record, 'cost_accounting': accounting})
            except Exception:
                # An unrecorded turn cannot be resumed safely.
                state = 'unknown'
            self.state = state

    def observe(self):
        """Return bounded process evidence, not an assertion of physical success."""
        with self.lock:
            summary = (self.last_result or {}).get('summary') or {}
            subtype = summary.get('result_subtype')
            if subtype not in ('success', 'error_max_budget_usd', 'error_max_turns',
                               'error_during_execution', 'error_max_structured_output_retries'):
                subtype = None
            cost = summary.get('total_cost_usd')
            if isinstance(cost, bool) or not isinstance(cost, (int, float)) or not math.isfinite(cost) or cost < 0:
                cost = None
            return {'actor': self.actor, 'thread_id': self.session_id,
                    'status': self.state, 'turn_id': self.turn_id, 'budget_status': self.budget_status,
                    'reported_spend_usd': str(self.reported_spend),
                    'last_result_subtype': subtype, 'last_turn_reported_cost_usd': cost}

    def wait(self, timeout=None):
        with self.lock:
            worker = self.worker
        if worker:
            worker.join(timeout)
        return self.observe()

    def stop(self):
        """Request termination of an active turn; physical state still needs sealing."""
        with self.lock:
            self.cancel_event.set()
            if self.state in ('new', 'idle'):
                self.state = 'unknown'
            return self.observe()
