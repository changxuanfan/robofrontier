import json
import stat
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

from robofrontier.claude.runner import build_command, build_env, run_turn, summarize, ROBOT_TOOL

PROFILE = {'provider': 'kimi', 'model': 'kimi-k3.1-pro', 'env_key': 'KIMI_API_KEY',
           'base_url': 'https://api.moonshot.cn/anthropic', 'effort': 'high'}
SECRET = 'sk-test-kimi-secret'


def fake_claude(directory, body):
    """Write an executable stand-in for the claude CLI; it makes no network calls."""
    path = Path(directory) / 'claude'
    path.write_text('#!' + sys.executable + '\nimport json, os, sys, time\n' + body)
    path.chmod(path.stat().st_mode | stat.S_IEXEC)
    return path


SUCCESS_BODY = '''
print(json.dumps({"type": "system", "subtype": "init", "session_id": "s1", "model": "kimi-k3.1-pro"}))
print(json.dumps({"type": "assistant", "session_id": "s1", "message": {"content": [{"type": "tool_use", "id": "t1", "name": "mcp__robot__robot", "input": {}}]}}))
print(json.dumps({"type": "result", "subtype": "success", "is_error": False, "session_id": "s1", "num_turns": 2,
                  "usage": {"input_tokens": 3, "output_tokens": 4}, "total_cost_usd": 0.01,
                  "result": "turn finished", "key_seen": os.environ.get("ANTHROPIC_API_KEY") == "%s",
                  "other_key_seen": "OPENAI_API_KEY" in os.environ, "base": os.environ.get("ANTHROPIC_BASE_URL")}))
''' % SECRET


class ClaudeRunnerTests(unittest.TestCase):
    def test_command_exposes_only_bound_robot_tool_and_isolates_session(self):
        cmd = build_command('kimi-k3.1-pro', 'high', '/tmp/mcp.json', 'go', session_id='11111111-1111-1111-1111-111111111111')
        self.assertIn('--strict-mcp-config', cmd)
        self.assertIn('--bare', cmd)
        self.assertEqual(cmd[cmd.index('--allowedTools') + 1], ROBOT_TOOL)
        self.assertEqual(cmd[cmd.index('--tools') + 1], '')
        self.assertEqual(cmd[cmd.index('--session-id') + 1], '11111111-1111-1111-1111-111111111111')
        self.assertNotIn('--resume', cmd)
        self.assertNotIn('--fork-session', cmd)
        self.assertEqual(cmd[-2:], ['--', 'go'])
        with self.assertRaises(ValueError): build_command('m', 'extreme', 'x', 'p')
        with self.assertRaises(ValueError): build_command('m', 'high', 'x', 'p', max_budget_usd=0)

    def test_resume_uses_explicit_id_and_never_both_flags(self):
        cmd = build_command('m', 'high', 'x', 'p', resume='22222222-2222-2222-2222-222222222222')
        self.assertEqual(cmd[cmd.index('--resume') + 1], '22222222-2222-2222-2222-222222222222')
        self.assertNotIn('--session-id', cmd)
        with self.assertRaises(ValueError): build_command('m', 'high', 'x', 'p', session_id='a', resume='b')

    def test_fork_creation_pins_ids_and_has_no_mcp_or_builtin_tools(self):
        cmd = build_command('m', 'high', '/run/role-mcp.json', 'p', session_id='child',
                            fork_from='parent', enable_robot=False)
        self.assertEqual(cmd[cmd.index('--resume') + 1], 'parent')
        self.assertEqual(cmd[cmd.index('--session-id') + 1], 'child')
        self.assertIn('--fork-session', cmd)
        self.assertEqual(json.loads(cmd[cmd.index('--mcp-config') + 1]), {'mcpServers': {}})
        self.assertEqual(cmd[cmd.index('--tools') + 1], '')
        self.assertNotIn('--allowedTools', cmd)
        with self.assertRaises(ValueError): build_command('m', 'high', 'x', 'p', session_id='child', fork_from='parent')
        with self.assertRaises(ValueError): build_command('m', 'high', 'x', 'p', fork_from='parent', enable_robot=False)

    def test_environment_carries_only_own_profile_credential(self):
        base = {'PATH': '/bin', 'KIMI_API_KEY': SECRET, 'OPENAI_API_KEY': 'other', 'ANTHROPIC_API_KEY': 'wrong'}
        env = build_env(PROFILE, base, '/run/config')
        self.assertEqual(env['ANTHROPIC_API_KEY'], SECRET)
        self.assertEqual(env['ANTHROPIC_BASE_URL'], PROFILE['base_url'])
        self.assertNotIn('OPENAI_API_KEY', env)
        self.assertEqual(env['CLAUDE_CONFIG_DIR'], '/run/config')
        with self.assertRaisesRegex(ValueError, 'KIMI_API_KEY'):
            build_env(PROFILE, {'PATH': '/bin'}, '/run/config')

    def test_summary_never_declares_task_success(self):
        lines = [json.dumps({'type': 'result', 'subtype': 'success', 'is_error': False, 'session_id': 's'}), '{bad json', '']
        summary = summarize(lines)
        self.assertEqual(summary['turn_status'], 'ended')
        self.assertIsNone(summary['task_success'])
        self.assertEqual(summary['malformed_lines'], 1)
        self.assertEqual(summarize([json.dumps({'type': 'result', 'subtype': 'error_max_turns', 'is_error': True})])['turn_status'], 'failed')
        self.assertEqual(summarize([])['turn_status'], 'unknown')

    def test_summary_keeps_aggregate_model_usage_and_failure_subtype(self):
        event={'type':'result','subtype':'error_max_budget_usd','is_error':True,
               'usage':{'output_tokens':302},'modelUsage':{'model':{
                   'outputTokens':13707,'thinkingTokens':13299,'cacheReadInputTokens':28889}},
               'total_cost_usd':.4755318}
        result=summarize([json.dumps(event)])
        self.assertEqual(result['turn_status'],'failed')
        self.assertEqual(result['result_subtype'],'error_max_budget_usd')
        self.assertEqual(result['usage']['output_tokens'],302)
        self.assertEqual(result['model_usage']['model']['outputTokens'],13707)
        self.assertEqual(result['model_usage']['model']['thinkingTokens'],13299)
        self.assertEqual(result['total_cost_usd'],.4755318)

    def test_image_evidence_must_match_the_observe_tool_call(self):
        call={'type':'assistant','message':{'content':[{'type':'tool_use','id':'wanted',
              'name':'mcp__robot__robot','input':{'op':'observe','views':['top']}}]}}
        result={'type':'user','message':{'content':[{'type':'tool_result','tool_use_id':'other',
                'content':[{'type':'image'}]}]}}
        def summary(): return summarize([json.dumps(call),json.dumps(result)])
        self.assertEqual(summary()['observe_top_calls'],1)
        self.assertEqual(summary()['observe_top_image_results'],0)
        result['message']['content'][0]['tool_use_id']='wanted'
        self.assertEqual(summary()['observe_top_image_results'],1)
        result['message']['content'][0]['is_error']=True
        self.assertEqual(summary()['observe_top_image_results'],0)

    def test_run_records_events_env_and_tool_use_with_fake_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp) / 'run'; run.mkdir()
            conn_cfg = run / 'mcp.json'; conn_cfg.write_text('{}')
            binary = fake_claude(tmp, SUCCESS_BODY)
            base = {'PATH': '/usr/bin:/bin', 'KIMI_API_KEY': SECRET, 'OPENAI_API_KEY': 'other'}
            record = run_turn(PROFILE, 'kimi-k3.1-pro', 'high', conn_cfg, 'go', run, base, binary=str(binary), timeout=20)
            self.assertEqual(record['process'], 'exited')
            self.assertEqual(record['exit_code'], 0)
            self.assertEqual(record['summary']['turn_status'], 'ended')
            self.assertEqual(record['summary']['tool_calls'], 1)
            self.assertEqual(record['summary']['session_id'], 's1')
            events = (run / record['events_path']).read_text()
            self.assertIn('"key_seen": true', events)
            self.assertIn('"other_key_seen": false', events)
            self.assertNotIn(SECRET, json.dumps(record))
            self.assertNotIn(SECRET, events.replace('"key_seen": true', ''))

    def test_cancellation_interrupts_cli_and_keeps_outcome_unknown(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp) / 'run'; run.mkdir()
            config = run / 'mcp.json'; config.write_text('{}')
            binary = fake_claude(tmp, 'time.sleep(30)\n')
            cancel = threading.Event()
            timer = threading.Timer(0.3, cancel.set); timer.start()
            try:
                record = run_turn(PROFILE, 'm', 'high', config, 'go', run,
                                  {'PATH': '/usr/bin:/bin', 'KIMI_API_KEY': SECRET},
                                  binary=str(binary), timeout=10, cancel_event=cancel)
            finally:
                timer.cancel()
            self.assertEqual(record['process'], 'cancelled')
            self.assertEqual(record['summary']['turn_status'], 'unknown')
            self.assertIsNone(record['summary']['task_success'])

    def test_cancellation_before_launch_never_starts_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp) / 'run'; run.mkdir()
            config = run / 'mcp.json'; config.write_text('{}')
            cancel = threading.Event(); cancel.set()
            with patch('robofrontier.claude.runner.subprocess.Popen') as spawn:
                with self.assertRaisesRegex(InterruptedError, 'cancelled before launch'):
                    run_turn(PROFILE, 'm', 'high', config, 'go', run,
                             {'PATH': '/usr/bin:/bin', 'KIMI_API_KEY': SECRET},
                             cancel_event=cancel)
                spawn.assert_not_called()

    def test_event_and_stderr_redact_credential_before_truncation(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp) / 'run'; run.mkdir()
            secret = 'fake-"unicode-密钥'
            binary = fake_claude(tmp, '''
secret = os.environ['ANTHROPIC_API_KEY']
print(json.dumps({'type':'result', 'subtype':'success', 'is_error':False, 'result':secret}))
sys.stderr.write(secret + 'x' * 990)
''')
            record = run_turn(PROFILE, 'm', 'high', None, 'go', run,
                              {'PATH': '/usr/bin:/bin', 'KIMI_API_KEY': secret},
                              binary=str(binary), timeout=10, enable_robot=False)
            events = run / record['events_path']
            self.assertEqual(json.loads(events.read_text())['result'], '[redacted]')
            self.assertEqual(record['summary']['result_text'], '[redacted]')
            self.assertNotIn(secret, json.dumps(record, ensure_ascii=False))
            self.assertNotIn(secret[-5:], record['stderr_tail'])
            self.assertEqual(stat.S_IMODE(events.stat().st_mode), 0o600)

    def test_failed_turn_and_deadline_are_not_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp) / 'run'; run.mkdir()
            conn_cfg = run / 'mcp.json'; conn_cfg.write_text('{}')
            base = {'PATH': '/usr/bin:/bin', 'KIMI_API_KEY': SECRET}
            failing = fake_claude(tmp, 'print(json.dumps({"type": "result", "subtype": "error_during_execution", "is_error": True}))\nsys.exit(1)\n')
            record = run_turn(PROFILE, 'm', 'high', conn_cfg, 'go', run, base, binary=str(failing), timeout=20)
            self.assertEqual(record['summary']['turn_status'], 'failed')
            self.assertIsNone(record['summary']['task_success'])
            hanging = fake_claude(tmp, 'time.sleep(30)\n')
            record = run_turn(PROFILE, 'm', 'high', conn_cfg, 'go', run, base, binary=str(hanging), timeout=1)
            self.assertEqual(record['process'], 'timeout')
            self.assertEqual(record['summary']['turn_status'], 'unknown')


if __name__ == '__main__': unittest.main()
