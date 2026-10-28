import json
import stat
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts import claude_precheck as pc

FAKE_BODY = '''
import urllib.request
argv = sys.argv[1:]
if "--version" in argv:
    print("fake-claude 0.0"); sys.exit(0)
prompt = argv[argv.index("--") + 1]
session = argv[argv.index("--resume") + 1] if "--resume" in argv else argv[argv.index("--session-id") + 1]
print(json.dumps({"type": "system", "subtype": "init", "session_id": session, "model": "claude-haiku-5-5"}))
answer, tool = "READY", False
if prompt.startswith("Call the robot tool"):
    conn = json.load(open("private/state/connections/Controller.json"))
    body = json.dumps({"op": "observe", "data": {}}).encode()
    urllib.request.urlopen(urllib.request.Request(conn["url"], data=body, method="POST"), timeout=10).read()
    tool = True
    answer = "%s"
elif prompt.startswith("Without calling"):
    answer = "%s"
print(json.dumps({"type": "assistant", "session_id": session, "message": {"content": [
    {"type": "tool_use", "id": "t", "name": "mcp__robot__robot", "input": {}}] if tool else []}}))
print(json.dumps({"type": "result", "subtype": "success", "is_error": False, "session_id": session,
                  "num_turns": 1, "result": answer, "total_cost_usd": 0.01, "usage": {"input_tokens": 1}}))
'''


def fake_claude(directory, tool_answer='green', recall_answer='green'):
    path = Path(directory) / 'claude'
    path.write_text('#!' + sys.executable + '\nimport json, sys\n' + FAKE_BODY % (tool_answer, recall_answer))
    path.chmod(path.stat().st_mode | stat.S_IEXEC)
    return path


class PrecheckTests(unittest.TestCase):
    def test_without_confirm_spend_nothing_runs(self):
        with patch.object(sys, 'argv', ['claude_precheck', '--max-budget-usd', '0.5']), \
             patch.object(pc, 'run_precheck') as run:
            self.assertEqual(pc.main(), 2)
            run.assert_not_called()

    def test_passes_when_robot_image_is_read_and_session_resumes(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = fake_claude(tmp)
            base = {'PATH': '/usr/bin:/bin', 'ANTHROPIC_API_KEY': 'fake-key-not-real'}
            receipt = pc.run_precheck('haiku', Path(tmp) / 'out', 0.5, base, binary=str(binary))
            self.assertTrue(receipt['passed'], receipt)
            self.assertEqual([c['name'] for c in receipt['checks']], ['text', 'tool_image', 'resume'])
            self.assertAlmostEqual(receipt['total_cost_usd'], 0.03)
            self.assertEqual(receipt['checks'][1]['session_id'], receipt['checks'][2]['session_id'])
            self.assertNotIn('fake-key-not-real', (Path(tmp) / 'out/receipt.json').read_text())

    def test_wrong_colour_stops_before_resume(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = fake_claude(tmp, tool_answer='red')
            base = {'PATH': '/usr/bin:/bin', 'ANTHROPIC_API_KEY': 'fake-key-not-real'}
            receipt = pc.run_precheck('haiku', Path(tmp) / 'out', 0.5, base, binary=str(binary))
            self.assertFalse(receipt['passed'])
            self.assertEqual(receipt['stopped'], 'failed at tool_image')
            self.assertEqual(len(receipt['checks']), 2)

    def test_colour_substring_does_not_count_as_correct(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary=fake_claude(tmp,tool_answer='not green')
            base={'PATH':'/usr/bin:/bin','ANTHROPIC_API_KEY':'fake-key-not-real'}
            receipt=pc.run_precheck('haiku',Path(tmp)/'out',.5,base,binary=str(binary))
            self.assertFalse(receipt['passed'])
            self.assertFalse(receipt['checks'][1]['checks']['image_color_named'])

    def test_budget_is_split_and_exhaustion_stops(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = fake_claude(tmp)
            base = {'PATH': '/usr/bin:/bin', 'ANTHROPIC_API_KEY': 'fake-key-not-real'}
            receipt = pc.run_precheck('haiku', Path(tmp) / 'out', 0.01, base, binary=str(binary))
            self.assertFalse(receipt['passed'])
            self.assertIn('budget exhausted', receipt['stopped'])

    def test_missing_cost_stops_before_additional_calls(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = fake_claude(tmp)
            binary.write_text(binary.read_text().replace('"total_cost_usd": 0.01', '"total_cost_usd": None'))
            base = {'PATH': '/usr/bin:/bin', 'ANTHROPIC_API_KEY': 'fake-key-not-real'}
            receipt = pc.run_precheck('haiku', Path(tmp) / 'out', 0.5, base, binary=str(binary))
            self.assertFalse(receipt['passed'])
            self.assertEqual(receipt['stopped'], 'cost unknown at text')
            self.assertEqual(len(receipt['checks']), 1)

    def test_optional_fork_requires_distinct_session_and_verified_native_history(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary=fake_claude(tmp)
            base={'PATH':'/usr/bin:/bin','ANTHROPIC_API_KEY':'fake-key-not-real'}
            original=pc.run_turn; forks=[]
            def runner(*args,**kwargs):
                if not kwargs.get('fork_from'): return original(*args,**kwargs)
                forks.append(kwargs)
                return {'process':'exited','exit_code':0,'summary':{'session_id':kwargs['session_id'],
                        'turn_status':'ended','tool_calls':0,'result_text':'green','malformed_lines':0,
                        'total_cost_usd':.01,'reported_model':'fake','usage':{}}}
            def proof(parent,child,config): return {'source_thread_id':parent}
            with patch.object(pc,'run_turn',runner), patch.object(pc,'fork_provenance',proof):
                receipt=pc.run_precheck('haiku',Path(tmp)/'pass',.5,base,binary=str(binary),check_fork=True)
            self.assertTrue(receipt['passed'],receipt)
            self.assertEqual(len(receipt['checks']),4)
            self.assertTrue(receipt['fork_checked'])
            self.assertFalse(forks[0]['enable_robot'])
            self.assertNotEqual(forks[0]['session_id'],forks[0]['fork_from'])
            with patch.object(pc,'run_turn',runner), patch.object(pc,'fork_provenance',side_effect=ValueError('bad prefix')):
                receipt=pc.run_precheck('haiku',Path(tmp)/'fail',.5,base,binary=str(binary),check_fork=True)
            self.assertFalse(receipt['passed'])
            self.assertEqual(receipt['stopped'],'failed at fork')
            self.assertFalse(receipt['checks'][-1]['checks']['native_history_prefix'])


if __name__ == '__main__': unittest.main()
