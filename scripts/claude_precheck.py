"""Real Claude precheck for one model profile under an explicit spend cap.

Without --confirm-spend this only prints the plan and makes no model call.
Checks: text reply, a local robot tool call whose returned image must be read
to answer, and a resumed turn on the same explicit session ID. Each check is
judged from the turn result and robot-stub request count, not from the reply
alone. A passing receipt covers only this profile, CLI version and cap.
"""
import argparse
import http.server
import json
import math
import os
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

from PIL import Image

from robofrontier.claude.runner import run_turn
from robofrontier.task_families import ROOT
from scripts.benchmark import load_env
from scripts.claude_benchmark import profile

COLOR = (37, 201, 113)
COLOR_NAME = 'green'
MIN_TURN_BUDGET = 0.02

PROMPTS = [
    ('text', 'Reply with exactly the word READY and nothing else.'),
    ('tool_image', 'Call the robot tool once with op "observe", data {} and views ["top"]. '
                   'Then reply with only the dominant color name of the returned image, chosen from: '
                   'red, green, blue, yellow, black, white.'),
    ('resume', 'Without calling any tool again, repeat the color name you identified, in one word.'),
]


class World(http.server.BaseHTTPRequestHandler):
    """Local stand-in for the robot service; serves one fixed observation image."""
    image = None
    requests = 0

    def log_message(self, *args):
        pass

    def do_POST(self):
        self.rfile.read(int(self.headers.get('Content-Length', 0)))
        World.requests += 1
        data = json.dumps({'observation': {'images': {'top': str(World.image)}}}).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def claude_version(binary='claude'):
    try:
        out = subprocess.run([binary, '--version'], capture_output=True, text=True, timeout=15).stdout.strip()
        return out.splitlines()[0] if out else None
    except (OSError, subprocess.TimeoutExpired):
        return None


def setup_run(output, base_env):
    run = output / 'run'
    conn = run / 'private/state/connections/Controller.json'
    conn.parent.mkdir(parents=True)
    image = run / 'public/observations/top.png'
    image.parent.mkdir(parents=True)
    Image.new('RGB', (64, 64), COLOR).save(image)
    World.image = image
    server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), World)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    conn.write_text(json.dumps({'run_id': run.name, 'actor': 'Controller', 'schema_version': 3,
                                'url': f'http://127.0.0.1:{server.server_port}', 'token': 'precheck-local'}))
    mcp = run / 'private/mcp.json'
    mcp.write_text(json.dumps({'mcpServers': {'robot': {
        'command': sys.executable,
        'args': ['-m', 'robofrontier.claude.robot_mcp', '--run-dir', str(run), '--connection', str(conn)],
        'env': {'PYTHONPATH': str(ROOT)}}}}))
    return run, mcp, server


def judge(name, record, requests_before, requests_after, expected_session=None):
    summary = record['summary']
    text = (summary.get('result_text') or '').lower()
    checks = {'process_exited': record['process'] == 'exited' and record['exit_code'] == 0,
              'turn_ended': summary['turn_status'] == 'ended'}
    if name == 'text':
        checks['reply_ready'] = 'ready' in text
    elif name == 'tool_image':
        checks['tool_called'] = summary['tool_calls'] >= 1
        checks['robot_request_made'] = requests_after > requests_before
        checks['image_color_named'] = COLOR_NAME in text
    elif name == 'resume':
        checks['same_session'] = summary['session_id'] == expected_session
        checks['color_recalled'] = COLOR_NAME in text
        checks['no_new_robot_call'] = requests_after == requests_before
    return {'name': name, 'passed': all(checks.values()), 'checks': checks,
            'turn_status': summary['turn_status'], 'reported_model': summary['reported_model'],
            'tool_calls': summary['tool_calls'], 'session_id': summary['session_id'],
            'cost_usd': summary.get('total_cost_usd'), 'usage': summary.get('usage')}


def run_precheck(name, output, budget, base_env, binary='claude'):
    prof = profile(name)
    output.mkdir(parents=True, exist_ok=False)
    run, mcp, server = setup_run(output, base_env)
    receipt = {'schema': 'robofrontier.claude-precheck.v1', 'profile': name, 'requested_model': prof['model'],
               'effort': prof['effort'], 'budget_usd': budget, 'claude_version': claude_version(binary),
               'started_at': time.time(), 'checks': [], 'stopped': None}
    spent = 0.0
    session = str(uuid.uuid4())
    try:
        for index, (check, prompt) in enumerate(PROMPTS):
            remaining = round(budget - spent, 6)
            if remaining < MIN_TURN_BUDGET:
                receipt['stopped'] = 'budget exhausted before ' + check
                break
            before = World.requests
            first = index == 0
            record = run_turn(prof, prof['model'], prof['effort'], mcp, prompt, run, base_env,
                              session_id=session if first else None, timeout=180,
                              max_budget_usd=remaining, binary=binary,
                              resume=None if first else session, events_name=f'{check}.jsonl')
            result = judge(check, record, before, World.requests, expected_session=session)
            receipt['checks'].append(result)
            cost = result['cost_usd']
            if isinstance(cost, bool) or not isinstance(cost, (int, float)) or not math.isfinite(cost) or cost <= 0:
                receipt['stopped'] = 'cost unknown at ' + check
                result['passed'] = False
                break
            spent += cost
            if spent > budget:
                receipt['stopped'] = 'reported cost exceeded budget at ' + check
                result['passed'] = False
                break
            if not result['passed']:
                receipt['stopped'] = 'failed at ' + check
                break
    finally:
        server.shutdown()
        server.server_close()
    receipt.update(total_cost_usd=round(spent, 6), passed=bool(receipt['checks']) and all(c['passed'] for c in receipt['checks'])
                   and len(receipt['checks']) == len(PROMPTS), finished_at=time.time())
    (output / 'receipt.json').write_text(json.dumps(receipt, indent=2, ensure_ascii=False))
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', default='haiku')
    parser.add_argument('--max-budget-usd', type=float, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--confirm-spend', action='store_true', help='Required: allow real model calls and spend')
    args = parser.parse_args()
    load_env(ROOT / '.env')
    prof = profile(args.profile)
    plan = {'profile': args.profile, 'model': prof['model'], 'budget_usd': args.max_budget_usd,
            'checks': [name for name, _ in PROMPTS], 'real_model_calls_if_confirmed': len(PROMPTS)}
    if not args.confirm_spend:
        print(json.dumps({'status': 'not_run', 'reason': 'Missing --confirm-spend', 'plan': plan}, indent=2))
        return 2
    if args.max_budget_usd <= 0:
        raise SystemExit('Budget must be positive')
    if not os.environ.get(prof['env_key']):
        raise SystemExit('Missing credential variable: ' + prof['env_key'])
    stamp = time.strftime('%Y%m%d-%H%M%S')
    output = args.output or ROOT / 'runs/engineering/claude-precheck' / (args.profile + '-' + stamp)
    receipt = run_precheck(args.profile, Path(output).absolute(), args.max_budget_usd, dict(os.environ))
    print(json.dumps({k: receipt[k] for k in ('profile', 'requested_model', 'passed', 'total_cost_usd', 'stopped')}
                     | {'receipt': str(Path(output).absolute() / 'receipt.json')}, indent=2, ensure_ascii=False))
    return 0 if receipt['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
