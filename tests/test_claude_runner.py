import json
import stat
import sys
import tempfile
import unittest
from pathlib import Path

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
        with self.assertRaises(ValueError): build_command('m', 'extreme', 'x', 'p')
        with self.assertRaises(ValueError): build_command('m', 'high', 'x', 'p', max_budget_usd=0)

    def test_resume_uses_explicit_id_and_never_both_flags(self):
        cmd = build_command('m', 'high', 'x', 'p', resume='22222222-2222-2222-2222-222222222222')
        self.assertEqual(cmd[cmd.index('--resume') + 1], '22222222-2222-2222-2222-222222222222')
        self.assertNotIn('--session-id', cmd)
        with self.assertRaises(ValueError): build_command('m', 'high', 'x', 'p', session_id='a', resume='b')

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
