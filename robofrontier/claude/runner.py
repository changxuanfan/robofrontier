"""Native Claude Code CLI runner for one policy session.

Offline logic only: building the CLI command and isolated environment, running
one `claude -p` turn, and summarising its stream-json evidence. A process exit
or a `result` event only means the model turn ended; task success is decided by
the runtime and is never inferred here.
"""
import json
import os
import signal
import subprocess
import time
import uuid
from pathlib import Path
from urllib.parse import urlsplit

ENV_PASSTHROUGH = ('PATH', 'LANG', 'LC_ALL', 'TMPDIR')
ROBOT_TOOL = 'mcp__robot__robot'
EFFORTS = ('low', 'medium', 'high', 'xhigh', 'max')


def _oauth_endpoint_allowed(base):
    if not base: return True  # Claude's default Anthropic endpoint.
    try:
        url=urlsplit(base)
        return (url.scheme=='https' and url.hostname=='api.anthropic.com' and url.port in (None,443)
                and not url.username and not url.password and url.path in ('','/') and not url.query and not url.fragment)
    except ValueError:
        return False


def build_command(model, effort, mcp_config, prompt, session_id=None, max_budget_usd=None, binary='claude', resume=None,
                  fork_from=None, enable_robot=True, auth_mode='api_key'):
    """Return argv for one headless turn with only the bound robot MCP tool exposed.

    `resume` continues an existing session by its explicit ID; it never picks the most recent one.
    """
    if effort not in EFFORTS:
        raise ValueError('Unsupported effort: ' + str(effort))
    if not model or not isinstance(model, str):
        raise ValueError('Model ID is required')
    if resume and (session_id or fork_from):
        raise ValueError('Give either session_id or resume, not both')
    if fork_from and (not session_id or enable_robot):
        raise ValueError('Fork creation requires an explicit child ID and no robot tools')
    session_flags = (['--resume', fork_from, '--fork-session', '--session-id', session_id] if fork_from else
                     ['--resume', resume] if resume else ['--session-id', session_id or str(uuid.uuid4())])
    mcp_arg = str(mcp_config) if enable_robot else '{"mcpServers":{}}'
    if auth_mode not in ('api_key','oauth_token'):
        raise ValueError('Unsupported Claude authentication mode')
    isolation = ['--bare'] if auth_mode == 'api_key' else ['--setting-sources', '']
    cmd = [binary, '-p'] + isolation + ['--model', model, '--effort', effort] + session_flags + [
           '--mcp-config', mcp_arg, '--strict-mcp-config',
           '--tools', '',
           '--permission-mode', 'dontAsk',
           '--output-format', 'stream-json', '--verbose']
    if enable_robot:
        cmd += ['--allowedTools', ROBOT_TOOL]
    if max_budget_usd is not None:
        if not isinstance(max_budget_usd, (int, float)) or max_budget_usd <= 0:
            raise ValueError('max_budget_usd must be a positive number')
        cmd += ['--max-budget-usd', str(max_budget_usd)]
    # Claude -p is a boolean flag. A prompt beginning with '-' must follow the
    # options terminator rather than being parsed as a CLI option.
    cmd += ['--', prompt]
    return cmd


def build_env(profile, base_env, config_dir):
    """Pass only this profile's endpoint and credential; never inherit other providers' keys."""
    key = base_env.get(profile['env_key'])
    if not key:
        raise ValueError('Missing credential variable: ' + profile['env_key'])
    env = {name: base_env[name] for name in ENV_PASSTHROUGH if name in base_env}
    auth_mode=profile.get('auth_mode','api_key')
    if auth_mode == 'oauth_token':
        if profile.get('provider') != 'anthropic' or profile['env_key'] != 'CLAUDE_CODE_OAUTH_TOKEN':
            raise ValueError('OAuth token mode is limited to the Anthropic subscription profile')
        if not _oauth_endpoint_allowed(profile.get('base_url')):
            raise ValueError('OAuth token may only be sent to the Anthropic endpoint')
        credential_name='CLAUDE_CODE_OAUTH_TOKEN'
    elif auth_mode == 'api_key':
        credential_name='ANTHROPIC_API_KEY'
    else:
        raise ValueError('Unsupported Claude authentication mode')
    env.update({
        credential_name: key,
        'CLAUDE_CONFIG_DIR': str(config_dir),
        'DISABLE_TELEMETRY': '1',
        'DISABLE_ERROR_REPORTING': '1',
        'CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC': '1',
        # wait_events can run for 600 seconds; reserve time for transport.
        'MCP_TOOL_TIMEOUT': '660000',
        'ENABLE_TOOL_SEARCH': 'false',
        'ENABLE_CLAUDEAI_MCP_SERVERS': 'false',
    })
    if profile.get('base_url'):
        env['ANTHROPIC_BASE_URL'] = profile['base_url']
    return env


def summarize(lines):
    """Reduce stream-json lines to turn-level facts. `task_success` is always None here."""
    events, malformed = [], 0
    for line in lines:
        if not line.strip():
            continue
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            malformed += 1
            continue
        if isinstance(event, dict):
            events.append(event)
        else:
            malformed += 1
    init = next((e for e in events if e.get('type') == 'system' and e.get('subtype') == 'init'), {})
    results = [e for e in events if e.get('type') == 'result']
    result = results[-1] if results else None
    tool_calls = sum(
        1 for e in events if e.get('type') == 'assistant'
        for block in (e.get('message') or {}).get('content') or []
        if isinstance(block, dict) and block.get('type') == 'tool_use')
    observe_top_ids = {
        block.get('id') for e in events if e.get('type') == 'assistant'
        for block in (e.get('message') or {}).get('content') or []
        if isinstance(block, dict) and block.get('type') == 'tool_use'
        and block.get('name') == ROBOT_TOOL and isinstance(block.get('input'), dict)
        and block['input'].get('op') == 'observe' and isinstance(block['input'].get('views'), list)
        and 'top' in block['input']['views'] and isinstance(block.get('id'), str) and block['id']}
    observe_top_image_results = sum(
        1 for e in events if e.get('type') == 'user'
        for block in (e.get('message') or {}).get('content') or []
        if isinstance(block, dict) and block.get('type') == 'tool_result'
        and block.get('tool_use_id') in observe_top_ids and not block.get('is_error')
        and isinstance(block.get('content'), list)
        and any(isinstance(item, dict) and item.get('type') == 'image' for item in block['content']))
    if result is None:
        turn = 'unknown'
    elif result.get('is_error') or result.get('subtype') != 'success':
        turn = 'failed'
    else:
        turn = 'ended'
    return {
        'session_id': (result or init).get('session_id') or init.get('session_id'),
        'reported_model': init.get('model'),
        'turn_status': turn,
        'result_subtype': (result or {}).get('subtype'),
        'num_turns': (result or {}).get('num_turns'),
        'result_text': ((result or {}).get('result') or '')[:2000],
        'tool_calls': tool_calls,
        'observe_top_calls': len(observe_top_ids),
        'observe_top_image_results': observe_top_image_results,
        'usage': (result or {}).get('usage'),
        'model_usage': (result or {}).get('modelUsage'),
        'total_cost_usd': (result or {}).get('total_cost_usd'),
        'malformed_lines': malformed,
        'task_success': None,
    }


def _redact(text, secret):
    # JSON escaping can change how a credential appears in the event stream.
    variants = {secret, json.dumps(secret, ensure_ascii=True)[1:-1],
                json.dumps(secret, ensure_ascii=False)[1:-1]}
    for variant in sorted(variants, key=len, reverse=True):
        if variant:
            text = text.replace(variant, '[redacted]')
    return text


class PreLaunchCancelled(InterruptedError):
    """Cancellation was observed before a CLI subprocess was created."""


def run_turn(profile, model, effort, mcp_config, prompt, run_dir, base_env,
             session_id=None, timeout=600, max_budget_usd=None, binary='claude', resume=None, events_name='events.jsonl',
             cancel_event=None, fork_from=None, enable_robot=True):
    """Run one turn, persist its raw events under run_dir, and return a status record.

    Status is `timeout` when the process is killed at the deadline, otherwise
    `ended` or `failed` from the turn result. Callers must still verify physical
    outcomes through the runtime before treating a task as finished.
    """
    run_dir = Path(run_dir).resolve()
    evidence = run_dir / 'private/evidence'
    evidence.mkdir(parents=True, exist_ok=True)
    config_dir = run_dir / 'private/claude-config'
    config_dir.mkdir(parents=True, exist_ok=True)
    cmd = build_command(model, effort, mcp_config, prompt, session_id, max_budget_usd, binary, resume,
                        fork_from=fork_from, enable_robot=enable_robot, auth_mode=profile.get('auth_mode','api_key'))
    env = build_env(profile, base_env, config_dir)
    raw = evidence / events_name
    # A queued worker may reach this point after its supervisor has stopped it.
    # Do not launch a potentially paid CLI turn when cancellation is already set.
    # Cancellation racing with Popen is still treated as an unknown outcome.
    if cancel_event is not None and cancel_event.is_set():
        raise PreLaunchCancelled('Claude turn cancelled before launch')
    process = subprocess.Popen(cmd, cwd=run_dir, env=env, text=True, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, stdin=subprocess.DEVNULL, start_new_session=True)
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        cancelled = cancel_event is not None and cancel_event.is_set()
        if cancelled or remaining <= 0:
            # The owner must reconcile outstanding physical operations separately.
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            stdout, stderr = process.communicate()
            status = 'cancelled' if cancelled else 'timeout'
            break
        try:
            stdout, stderr = process.communicate(timeout=min(0.25, remaining) if cancel_event is not None else remaining)
            status = 'exited'
            break
        except subprocess.TimeoutExpired:
            continue
    secret = base_env[profile['env_key']]
    stdout = _redact(stdout, secret)
    # Evidence can contain model output and private task observations.
    fd = os.open(raw, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, 'w') as output:
        os.fchmod(output.fileno(), 0o600)
        output.write(stdout)
    summary = summarize(stdout.splitlines())
    if status in ('timeout', 'cancelled'):
        summary['turn_status'] = 'unknown'
    return {
        'process': status,
        'exit_code': process.returncode,
        'requested_session_id': session_id or resume,
        'summary': summary,
        'stderr_tail': _redact(stderr, secret)[-1000:],
        'events_path': str(raw.relative_to(run_dir)),
    }
