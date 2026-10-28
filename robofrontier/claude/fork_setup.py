"""Host-recorded, tool-free Claude fork creation for offline engineering.

Does not bind, activate, or grant a child any robot permission. Runtime admission
for Claude delegation remains disabled until the complete protocol is qualified.
"""
from decimal import Decimal, InvalidOperation
import hashlib
import json
import os
from pathlib import Path
import re
import time
import uuid

from robofrontier.claude.provenance import fork_provenance, history, transcript_rows
from robofrontier.claude.runner import run_turn
from robofrontier.delegation import Delegations
from robofrontier.run_setup import write_json

ACTOR = re.compile(r'Delegate_[a-f0-9]{12}\Z')
SETUP_PROMPT = ('This is an engineering setup turn with no tools. Do not act on the inherited task. '
                'Reply READY and wait for a later role assignment.')


def _pending_history(store, parent, batch_id, actor, owner, config, cancel_event):
    """Wait until the native transcript contains the actual blocked tool call.

    The runtime can accept the MCP call before Claude flushes its assistant
    message. Forking the older prefix would produce inconsistent provenance.
    No model call has been launched while waiting here.
    """
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        if cancel_event is not None and cancel_event.is_set():
            raise ValueError('Fork setup was cancelled')
        _boundary(store, parent, batch_id, actor, owner)
        try:
            source = history(transcript_rows(config, parent.session_id), parent.session_id)
        except ValueError:
            source = []  # File creation or a concurrent append may still be in progress.
        for row in reversed(source):
            msg = row.get('message') or {}
            content = msg.get('content') if isinstance(msg, dict) else None
            if not isinstance(content, list): continue
            for item in content:
                if not isinstance(item, dict) or item.get('type') != 'tool_use': continue
                data = item.get('input')
                if (isinstance(data, dict) and data.get('op') == 'delegate'
                        and data.get('call_id') == batch_id and item.get('name') == 'mcp__robot__robot'):
                    return source
        time.sleep(.05)
    raise ValueError('Pending delegation was not recorded in the native transcript')


def _boundary(store, parent_host, batch_id, actor, owner):
    cfg = store.get('config')
    if cfg.get('engine') != 'claude' or cfg.get('topology') != 'adaptive':
        raise ValueError('Expected an adaptive Claude run')
    if store.get('lifecycle') != 'running' or store.get('dispatch_paused', False):
        raise ValueError('Run is not active')
    batch = Delegations(store).batch(batch_id)
    if batch['status'] != 'creating' or batch['claimed_by'] != owner or time.time() >= batch['setup_deadline']:
        raise ValueError('Delegation ownership or creation boundary changed')
    if batch['parent'] != 'Controller' or batch['source_thread_id'] != parent_host.session_id:
        raise ValueError('Parent session does not match delegation')
    if parent_host.observe()['status'] != 'active': raise ValueError('Parent is not at an active tool turn')
    call = store.call(batch_id, 'Controller')
    if call['op'] != 'delegate' or call['status'] != 'accepted':
        raise ValueError('Parent delegation tool call is not pending')
    child = next((c for c in batch['children'] if c['actor'] == actor), None)
    if child is None or child['thread_id'] is not None or child['status'] != 'creating':
        raise ValueError('Child is not an uncreated scope')
    return batch


def create_fork(store, parent_host, batch_id, actor, owner, *, max_budget_usd,
                timeout=90, runner=run_turn, budget_ledger=None, cancel_event=None):
    """Create exactly one no-tool setup turn; on any ambiguity never retry.

    The returned evidence is not permission to run or bind a child. In particular,
    a caller still needs a task-wide concurrent cost ledger before enabling adaptive.
    """
    if not isinstance(actor, str) or not ACTOR.fullmatch(actor): raise ValueError('Invalid delegate actor')
    if isinstance(max_budget_usd, bool) or not isinstance(max_budget_usd, (int, float)) or not 0 < max_budget_usd < float('inf'):
        raise ValueError('A finite positive setup budget is required')
    _boundary(store, parent_host, batch_id, actor, owner)
    run = Path(parent_host.run).resolve()
    config = run / 'private/claude-config'
    source = _pending_history(store, parent_host, batch_id, actor, owner, config, cancel_event)
    if not source: raise ValueError('Parent transcript has no messages')
    digest = hashlib.sha256(json.dumps(source, sort_keys=True).encode()).hexdigest()
    child = str(uuid.uuid4())
    claim = run / 'private/claude-forks' / (actor + '.json')
    claim.parent.mkdir(parents=True, exist_ok=True)
    entry = {'status': 'claimed', 'batch_id': batch_id, 'actor': actor, 'owner': owner,
             'parent_session': parent_host.session_id, 'child_session': child,
             'parent_history_sha256': digest, 'claimed_at': time.time(), 'tool_permissions': 'none'}
    # Durable identity and intent precede the first potentially paid model call.
    with claim.open('x') as out:
        os.fchmod(out.fileno(), 0o600)
        json.dump(entry, out); out.flush(); os.fsync(out.fileno())
    reservation = None
    settled = False
    try:
        _boundary(store, parent_host, batch_id, actor, owner)
        profile = parent_host.profile
        if budget_ledger is not None:
            reservation = 'fork-' + child
            max_budget_usd = budget_ledger.reserve(reservation, actor, max_budget_usd)
        if cancel_event is not None and cancel_event.is_set():
            raise ValueError('Fork setup was cancelled')
        record = runner(profile, profile['model'], profile['effort'], None, SETUP_PROMPT, run,
                        parent_host.base_env, session_id=child, fork_from=parent_host.session_id,
                        enable_robot=False, max_budget_usd=max_budget_usd, timeout=timeout,
                        binary=parent_host.binary, events_name=actor+'-fork-setup.jsonl', cancel_event=cancel_event)
        summary = record.get('summary') or {}
        if (record.get('process') != 'exited' or record.get('exit_code') != 0
                or summary.get('session_id') != child or summary.get('turn_status') != 'ended'
                or summary.get('malformed_lines') or summary.get('tool_calls') != 0):
            raise ValueError('Fork setup outcome incomplete or unexpected')
        cost = summary.get('total_cost_usd')
        try:
            amount = Decimal(str(cost)) if not isinstance(cost, bool) else Decimal('NaN')
            if not amount.is_finite() or amount <= 0: raise InvalidOperation
        except (InvalidOperation, ValueError) as error:
            raise ValueError('Fork setup cost is unknown') from error
        if amount > Decimal(str(max_budget_usd)):
            raise ValueError('Fork setup reported cost exceeds its allowance')
        if budget_ledger is not None:
            shared = budget_ledger.settle(reservation, amount, outcome_known=True)
            settled = True
            if shared['status'] == 'uncertain': raise ValueError('Shared budget is uncertain')
        _boundary(store, parent_host, batch_id, actor, owner)
        proof = fork_provenance(parent_host.session_id, child, config)
        if proof['source_sha256'] != digest:
            raise ValueError('Parent history changed across fork setup')
        entry.update(status='history_verified_no_authority', completed_at=time.time(), proof=proof,
                     reported_cost_usd=str(amount))
        write_json(claim, entry)
        return entry
    except Exception as error:
        if reservation is not None and not settled:
            try: budget_ledger.settle(reservation, None, outcome_known=False)
            except Exception: pass  # A remaining reservation still conservatively consumes its allowance.
        entry.update(status='uncertain', error_type=type(error).__name__, completed_at=time.time())
        try: write_json(claim, entry)
        except OSError: pass
        raise
