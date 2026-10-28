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
import uuid
from pathlib import Path

ENV_PASSTHROUGH = ('PATH', 'LANG', 'LC_ALL', 'TMPDIR')
ROBOT_TOOL = 'mcp__robot__robot'
EFFORTS = ('low', 'medium', 'high', 'xhigh', 'max')


def build_command(model, effort, mcp_config, prompt, session_id=None, max_budget_usd=None, binary='claude'):
    """Return argv for one headless turn with only the bound robot MCP tool exposed."""
    if effort not in EFFORTS:
        raise ValueError('Unsupported effort: ' + str(effort))
    if not model or not isinstance(model, str):
        raise ValueError('Model ID is required')
    session_id = session_id or str(uuid.uuid4())
    cmd = [binary, '-p', prompt, '--bare', '--model', model, '--effort', effort,
           '--session-id', session_id,
           '--mcp-config', str(mcp_config), '--strict-mcp-config',
           '--allowedTools', ROBOT_TOOL, '--tools', '',
           '--permission-mode', 'dontAsk',
           '--output-format', 'stream-json', '--verbose']
    if max_budget_usd is not None:
        if not isinstance(max_budget_usd, (int, float)) or max_budget_usd <= 0:
            raise ValueError('max_budget_usd must be a positive number')
        cmd += ['--max-budget-usd', str(max_budget_usd)]
    return cmd


def build_env(profile, base_env, config_dir):
    """Pass only this profile's endpoint and credential; never inherit other providers' keys."""
    key = base_env.get(profile['env_key'])
    if not key:
        raise ValueError('Missing credential variable: ' + profile['env_key'])
    env = {name: base_env[name] for name in ENV_PASSTHROUGH if name in base_env}
    env.update({
        'ANTHROPIC_API_KEY': key,
        'CLAUDE_CONFIG_DIR': str(config_dir),
        'DISABLE_TELEMETRY': '1',
        'DISABLE_ERROR_REPORTING': '1',
        'CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC': '1',
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
        'num_turns': (result or {}).get('num_turns'),
        'tool_calls': tool_calls,
        'usage': (result or {}).get('usage'),
        'total_cost_usd': (result or {}).get('total_cost_usd'),
        'malformed_lines': malformed,
        'task_success': None,
    }


def run_turn(profile, model, effort, mcp_config, prompt, run_dir, base_env,
             session_id=None, timeout=600, max_budget_usd=None, binary='claude'):
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
    cmd = build_command(model, effort, mcp_config, prompt, session_id, max_budget_usd, binary)
    env = build_env(profile, base_env, config_dir)
    raw = evidence / 'events.jsonl'
    process = subprocess.Popen(cmd, cwd=run_dir, env=env, text=True, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, start_new_session=True)
    try:
        stdout, stderr = process.communicate(timeout=timeout)
        status = 'exited'
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        stdout, stderr = process.communicate()
        status = 'timeout'
    raw.write_text(stdout)
    summary = summarize(stdout.splitlines())
    if status == 'timeout':
        summary['turn_status'] = 'unknown'
    secret = env['ANTHROPIC_API_KEY']
    return {
        'process': status,
        'exit_code': process.returncode,
        'requested_session_id': session_id,
        'summary': summary,
        'stderr_tail': stderr[-1000:].replace(secret, '[redacted]'),
        'events_path': str(raw.relative_to(run_dir)),
    }
