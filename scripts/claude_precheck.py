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
import secrets
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

from PIL import Image

from robofrontier.claude.profile_identity import fingerprint
from robofrontier.claude.runner import run_turn
from robofrontier.claude.provenance import fork_provenance
from robofrontier.task_families import ROOT
from scripts.benchmark import load_env
from scripts.claude_benchmark import profile

COLORS = {'red': (220, 35, 35), 'green': (37, 201, 113), 'blue': (35, 60, 220),
          'yellow': (230, 210, 30), 'black': (10, 10, 10), 'white': (245, 245, 245)}
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

    def log_message(self, *args):
        pass

    def do_POST(self):
        raw = self.rfile.read(int(self.headers.get('Content-Length', 0)))
        try: body = json.loads(raw)
        except (ValueError, UnicodeDecodeError): body = None
        valid = (self.path == '/op' and isinstance(body, dict) and body.get('op') == 'observe'
                 and isinstance(body.get('data'), dict))
        with self.server.request_lock:
            self.server.request_count += 1
            if valid: self.server.observe_count += 1
        if not valid:
            self.send_error(400, 'Expected an observe request')
            return
        entered = getattr(self.server, 'blocked_entered', None)
        if entered is not None:
            entered.set()
            self.server.blocked_release.wait(180)
        data = json.dumps({'observation': {'images': {'top': str(self.server.image)}}}).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        try: self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError): pass


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
    color_name = secrets.choice(tuple(COLORS))
    Image.new('RGB', (64, 64), COLORS[color_name]).save(image)
    server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), World)
    server.image = image
    server.request_count = 0
    server.observe_count = 0
    server.request_lock = threading.Lock()
    server.challenge_color = color_name
    threading.Thread(target=server.serve_forever, daemon=True).start()
    conn.write_text(json.dumps({'run_id': run.name, 'actor': 'Controller', 'schema_version': 3,
                                'url': f'http://127.0.0.1:{server.server_port}', 'token': 'precheck-local'}))
    mcp = run / 'private/mcp.json'
    mcp.write_text(json.dumps({'mcpServers': {'robot': {
        'command': sys.executable,
        'args': ['-m', 'robofrontier.claude.robot_mcp', '--run-dir', str(run), '--connection', str(conn)],
        'env': {'PYTHONPATH': str(ROOT)}}}}))
    return run, mcp, server


def request_count(server, *, observe_only=False):
    with server.request_lock:
        return server.observe_count if observe_only else server.request_count


def judge(name, record, requests_before, requests_after, expected_session=None, expected_color='green',
          observes_before=0, observes_after=0):
    summary = record['summary']
    text = (summary.get('result_text') or '').strip().lower()
    checks = {'process_exited': record['process'] == 'exited' and record['exit_code'] == 0,
              'turn_ended': summary['turn_status'] == 'ended',
              'same_session': summary['session_id'] == expected_session,
              'valid_events': summary.get('malformed_lines') == 0}
    if name == 'text':
        checks['reply_ready'] = text == 'ready'
        checks['no_new_robot_call'] = requests_after == requests_before
    elif name == 'tool_image':
        checks['tool_called'] = summary['tool_calls'] >= 1
        checks['robot_request_made'] = observes_after > observes_before
        checks['image_view_requested'] = summary.get('observe_top_calls', 0) >= 1
        checks['image_result_recorded'] = summary.get('observe_top_image_results', 0) >= 1
        checks['image_color_named'] = text == expected_color
    elif name == 'resume':
        checks['color_recalled'] = text == expected_color
        checks['no_new_robot_call'] = requests_after == requests_before
    return {'name': name, 'passed': all(checks.values()), 'checks': checks,
            'turn_status': summary['turn_status'], 'reported_model': summary['reported_model'],
            'tool_calls': summary['tool_calls'], 'session_id': summary['session_id'],
            'cost_usd': summary.get('total_cost_usd'), 'usage': summary.get('usage')}


def run_precheck(name, output, budget, base_env, binary='claude', check_fork=False):
    if isinstance(budget,bool) or not isinstance(budget,(int,float)) or not math.isfinite(budget) or budget<=0:
        raise ValueError('Budget must be positive and finite')
    prof = profile(name)
    output.mkdir(parents=True, exist_ok=False)
    run, mcp, server = setup_run(output, base_env)
    receipt = {'schema': 'robofrontier.claude-precheck.v1', 'profile': name, 'requested_model': prof['model'],
               'effort': prof['effort'], 'budget_usd': budget, 'claude_version': claude_version(binary),
               'profile_sha256': fingerprint(prof),
               'started_at': time.time(), 'checks': [], 'stopped': None}
    spent = 0.0
    session = str(uuid.uuid4())
    try:
        for index, (check, prompt) in enumerate(PROMPTS):
            remaining = round(budget - spent, 6)
            if remaining < MIN_TURN_BUDGET:
                receipt['stopped'] = 'budget exhausted before ' + check
                break
            before = request_count(server)
            observes_before = request_count(server, observe_only=True)
            first = index == 0
            record = run_turn(prof, prof['model'], prof['effort'], mcp, prompt, run, base_env,
                              session_id=session if first else None, timeout=180,
                              max_budget_usd=remaining, binary=binary,
                              resume=None if first else session, events_name=f'{check}.jsonl')
            result = judge(check, record, before, request_count(server), expected_session=session,
                           expected_color=server.challenge_color, observes_before=observes_before,
                           observes_after=request_count(server, observe_only=True))
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
        if check_fork and receipt['stopped'] is None and len(receipt['checks']) == len(PROMPTS):
            remaining = round(budget-spent, 6)
            if remaining < MIN_TURN_BUDGET:
                receipt['stopped'] = 'budget exhausted before fork'
            else:
                child = str(uuid.uuid4())
                before = request_count(server)
                record = run_turn(prof, prof['model'], prof['effort'], None,
                                  'This is a tool-free fork. Recall the color you identified and reply with only the color name.',
                                  run, base_env, session_id=child, fork_from=session, enable_robot=False,
                                  timeout=180, max_budget_usd=remaining, binary=binary, events_name='fork.jsonl')
                summary = record.get('summary') or {}
                checks = {'process_exited':record.get('process')=='exited' and record.get('exit_code')==0,
                          'turn_ended':summary.get('turn_status')=='ended',
                          'distinct_expected_session':summary.get('session_id')==child and child!=session,
                          'no_tool_calls':summary.get('tool_calls')==0 and request_count(server)==before,
                          'color_recalled':(summary.get('result_text') or '').strip().lower()==server.challenge_color,
                          'valid_events':summary.get('malformed_lines')==0}
                try:
                    proof=fork_provenance(session,child,run/'private/claude-config')
                    checks['native_history_prefix']=proof.get('source_thread_id')==session
                except (ValueError,OSError):
                    checks['native_history_prefix']=False
                result={'name':'fork','passed':all(checks.values()),'checks':checks,
                        'turn_status':summary.get('turn_status'),'reported_model':summary.get('reported_model'),
                        'tool_calls':summary.get('tool_calls'),'session_id':summary.get('session_id'),
                        'cost_usd':summary.get('total_cost_usd'),'usage':summary.get('usage')}
                receipt['checks'].append(result)
                cost=result['cost_usd']
                if isinstance(cost,bool) or not isinstance(cost,(int,float)) or not math.isfinite(cost) or cost<=0:
                    result['passed']=False; receipt['stopped']='cost unknown at fork'
                else:
                    spent+=cost
                    if spent>budget:
                        result['passed']=False; receipt['stopped']='reported cost exceeded budget at fork'
                    elif not result['passed']:
                        receipt['stopped']='failed at fork'
    finally:
        server.shutdown()
        server.server_close()
    receipt.update(total_cost_usd=round(spent, 6), passed=bool(receipt['checks']) and all(c['passed'] for c in receipt['checks'])
                   and len(receipt['checks']) == len(PROMPTS)+(1 if check_fork else 0), finished_at=time.time(),
                   fork_checked=check_fork, challenge_color=server.challenge_color)
    (output / 'receipt.json').write_text(json.dumps(receipt, indent=2, ensure_ascii=False))
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', default='haiku')
    parser.add_argument('--max-budget-usd', type=float, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--check-fork', action='store_true', help='Also run a tool-free native fork and verify transcript history')
    parser.add_argument('--confirm-spend', action='store_true', help='Required: allow real model calls and spend')
    args = parser.parse_args()
    load_env(ROOT / '.env')
    prof = profile(args.profile)
    plan = {'profile': args.profile, 'model': prof['model'], 'budget_usd': args.max_budget_usd,
            'checks': [name for name, _ in PROMPTS]+(['fork'] if args.check_fork else []),
            'real_model_calls_if_confirmed': len(PROMPTS)+(1 if args.check_fork else 0)}
    if not args.confirm_spend:
        print(json.dumps({'status': 'not_run', 'reason': 'Missing --confirm-spend', 'plan': plan}, indent=2))
        return 2
    if not math.isfinite(args.max_budget_usd) or args.max_budget_usd <= 0:
        raise SystemExit('Budget must be positive and finite')
    if not os.environ.get(prof['env_key']):
        raise SystemExit('Missing credential variable: ' + prof['env_key'])
    stamp = time.strftime('%Y%m%d-%H%M%S')
    output = args.output or ROOT / 'runs/engineering/claude-precheck' / (args.profile + '-' + stamp)
    receipt = run_precheck(args.profile, Path(output).absolute(), args.max_budget_usd, dict(os.environ), check_fork=args.check_fork)
    print(json.dumps({k: receipt[k] for k in ('profile', 'requested_model', 'passed', 'total_cost_usd', 'stopped')}
                     | {'receipt': str(Path(output).absolute() / 'receipt.json')}, indent=2, ensure_ascii=False))
    return 0 if receipt['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
