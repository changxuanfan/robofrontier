"""Offline role/session lifecycle checks; no model or physics calls."""
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from robofrontier.claude.session_host import ClaudeSessionHost, role_mcp_config

PROFILE = {'model': 'fake-model', 'effort': 'high', 'env_key': 'FAKE_KEY'}


def make_run(tmp):
    run = Path(tmp) / 'run'
    roles = run / 'public/inputs/roles.json'
    roles.parent.mkdir(parents=True)
    roles.write_text(json.dumps({'roles': {'Meta': {'kind': 'manager'}, 'Controller': {'kind': 'controller'}}}))
    connection = run / 'private/state/connections/Controller.json'
    connection.parent.mkdir(parents=True)
    connection.write_text(json.dumps({'schema_version': 3, 'run_id': 'run', 'actor': 'Controller', 'token': 'fake'}))
    return run


def result(sid, status='ended', code=0, malformed=0):
    return {'process': 'exited', 'exit_code': code,
            'summary': {'session_id': sid, 'turn_status': status, 'malformed_lines': malformed, 'task_success': None}}


class SessionHostTests(unittest.TestCase):
    def test_role_config_is_bound_and_manager_cannot_be_selected(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = make_run(tmp)
            config = json.loads(role_mcp_config(run, 'Controller').read_text())
            args = config['mcpServers']['robot']['args']
            self.assertEqual(args[args.index('--connection') + 1], str((run / 'private/state/connections/Controller.json').resolve()))
            self.assertNotIn('fake', json.dumps(config))
            for actor in ('Meta', '../Controller', 'Unknown'):
                with self.assertRaises(ValueError): role_mcp_config(run, actor)

    def test_no_overlap_and_explicit_resume_of_the_same_role(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = make_run(tmp); entered = threading.Event(); release = threading.Event(); calls = []
            def runner(*args, **kwargs):
                calls.append(kwargs); entered.set(); release.wait(3)
                return result(kwargs.get('session_id') or kwargs.get('resume'))
            host = ClaudeSessionHost(run, 'Controller', PROFILE, {}, runner=runner)
            first = host.send('bootstrap'); self.assertTrue(entered.wait(2))
            self.assertEqual(host.observe()['status'], 'active')
            with self.assertRaises(ValueError): host.send('overlapping')
            release.set(); self.assertEqual(host.wait(3)['status'], 'idle')
            host.send('next turn'); self.assertEqual(host.wait(3)['status'], 'idle')
            self.assertEqual(calls[0]['session_id'], first['threadId'])
            self.assertEqual(calls[1]['resume'], first['threadId'])
            self.assertNotEqual(calls[0]['events_name'], calls[1]['events_name'])
            self.assertEqual(len(list((run/'private/evidence').glob('*-summary.json'))), 2)

    def test_mismatch_failure_or_missing_evidence_never_allows_resume(self):
        scenarios = [lambda sid: result('different'), lambda sid: result(sid, 'failed', 1),
                     lambda sid: result(sid, malformed=1),
                     lambda sid: {'process': 'timeout', 'summary': {'turn_status': 'unknown'}}]
        for response in scenarios:
            with self.subTest(response=response), tempfile.TemporaryDirectory() as tmp:
                run = make_run(tmp)
                def runner(*args, **kwargs): return response(kwargs['session_id'])
                host = ClaudeSessionHost(run, 'Controller', PROFILE, {}, runner=runner)
                host.send('start'); self.assertIn(host.wait(3)['status'], ('error', 'unknown'))
                with self.assertRaises(ValueError): host.send('must not retry')

    def test_stop_blocks_subsequent_turn_even_if_exit_races(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = make_run(tmp); entered = threading.Event(); release = threading.Event()
            def runner(*args, **kwargs):
                entered.set(); release.wait(3)
                return result(kwargs['session_id'])
            host = ClaudeSessionHost(run, 'Controller', PROFILE, {}, runner=runner)
            host.send('start'); self.assertTrue(entered.wait(2)); host.stop(); release.set()
            self.assertEqual(host.wait(3)['status'], 'unknown')
            with self.assertRaises(ValueError): host.send('do not restart')

    def test_failed_evidence_write_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = make_run(tmp)
            def runner(*args, **kwargs): return result(kwargs['session_id'])
            host = ClaudeSessionHost(run, 'Controller', PROFILE, {}, runner=runner)
            with patch('robofrontier.claude.session_host.write_json', side_effect=OSError('write failed')):
                host.send('start'); self.assertEqual(host.wait(3)['status'], 'unknown')
            with self.assertRaises(ValueError): host.send('must not retry')

    def test_reported_task_cost_reduces_each_turn_allowance(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = make_run(tmp); calls = []
            def runner(*args, **kwargs):
                calls.append(kwargs)
                response = result(kwargs.get('session_id') or kwargs.get('resume'))
                response['summary']['total_cost_usd'] = [.3, .4, .3][len(calls)-1]
                return response
            host = ClaudeSessionHost(run, 'Controller', PROFILE, {}, runner=runner,
                                     max_budget_usd=.6, total_budget_usd=1)
            for _ in range(2):
                host.send('continue'); self.assertEqual(host.wait(3)['status'], 'idle')
            host.send('continue'); observation = host.wait(3)
            self.assertEqual(observation['budget_status'], 'exhausted')
            self.assertEqual(observation['reported_spend_usd'], '1.0')
            self.assertEqual([round(c['max_budget_usd'], 3) for c in calls], [.6, .6, .3])
            with self.assertRaises(ValueError): host.send('another')

    def test_missing_zero_or_invalid_cost_blocks_further_turns(self):
        for value in (None, 0, True, float('nan'), -1):
            with self.subTest(value=value), tempfile.TemporaryDirectory() as tmp:
                run = make_run(tmp)
                def runner(*args, **kwargs):
                    response = result(kwargs['session_id']); response['summary']['total_cost_usd'] = value
                    return response
                host = ClaudeSessionHost(run, 'Controller', PROFILE, {}, runner=runner, total_budget_usd=1)
                host.send('start'); observation = host.wait(3)
                self.assertEqual(observation['status'], 'unknown')
                self.assertEqual(observation['budget_status'], 'cost_unknown')
                with self.assertRaises(ValueError): host.send('retry')

    def test_two_sessions_share_reserved_task_budget(self):
        from robofrontier.claude.budget import BudgetLedger
        with tempfile.TemporaryDirectory() as tmp:
            run = make_run(tmp); role_path = run/'public/inputs/roles.json'
            roles = json.loads(role_path.read_text()); roles['roles']['A'] = {'kind':'worker'}
            role_path.write_text(json.dumps(roles))
            (run/'private/state/connections/A.json').write_text(json.dumps({'schema_version':3,'run_id':'run','actor':'A'}))
            ledger = BudgetLedger.create(run/'private/state/budget.sqlite', 1)
            entered = threading.Barrier(3); release = threading.Event(); allowances=[]
            def runner(*args, **kwargs):
                allowances.append(kwargs['max_budget_usd']); entered.wait(3); release.wait(3)
                response=result(kwargs['session_id']); response['summary']['total_cost_usd']=.1
                return response
            try:
                a=ClaudeSessionHost(run,'Controller',PROFILE,{},max_budget_usd=.6,budget_ledger=ledger,runner=runner)
                b=ClaudeSessionHost(run,'A',PROFILE,{},max_budget_usd=.6,budget_ledger=ledger,runner=runner)
                a.send('start'); b.send('start'); entered.wait(3)
                self.assertEqual(sorted(allowances), [.4,.6])
                self.assertEqual(ledger.snapshot()['reserved_usd'], '1.0')
                release.set(); self.assertEqual(a.wait(3)['status'],'idle'); self.assertEqual(b.wait(3)['status'],'idle')
                self.assertEqual(ledger.snapshot()['reported_spend_usd'],'0.2')
            finally:
                release.set(); ledger.close()

    def test_nonzero_exit_keeps_shared_budget_uncertain(self):
        from robofrontier.claude.budget import BudgetLedger
        with tempfile.TemporaryDirectory() as tmp:
            run = make_run(tmp)
            ledger = BudgetLedger.create(run/'private/state/budget.sqlite', 1)
            def runner(*args, **kwargs):
                response = result(kwargs['session_id'], code=1)
                response['summary']['total_cost_usd'] = .4755318
                response['summary']['result_subtype'] = 'error_max_budget_usd'
                return response
            try:
                host = ClaudeSessionHost(run, 'Controller', PROFILE, {}, max_budget_usd=.6,
                                         budget_ledger=ledger, runner=runner)
                host.send('start')
                observation=host.wait(3)
                self.assertEqual(observation['status'], 'unknown')
                self.assertEqual(observation['last_result_subtype'],'error_max_budget_usd')
                self.assertEqual(observation['last_turn_reported_cost_usd'],.4755318)
                self.assertEqual(ledger.snapshot()['status'], 'uncertain')
                with self.assertRaisesRegex(ValueError, 'uncertain'):
                    ledger.reserve('another', 'other-role', .1)
            finally:
                ledger.close()

    def test_prelaunch_cancellation_releases_only_its_shared_reservation(self):
        from robofrontier.claude.budget import BudgetLedger
        from robofrontier.claude.runner import PreLaunchCancelled
        with tempfile.TemporaryDirectory() as tmp:
            run=make_run(tmp); ledger=BudgetLedger.create(run/'private/state/budget.sqlite',1)
            entered=threading.Event(); release=threading.Event()
            def runner(*args, **kwargs):
                entered.set(); release.wait(3)
                raise PreLaunchCancelled('no process created')
            try:
                host=ClaudeSessionHost(run,'Controller',PROFILE,{},max_budget_usd=.6,
                                       budget_ledger=ledger,runner=runner)
                host.send('start'); self.assertTrue(entered.wait(2))
                self.assertEqual(ledger.snapshot()['reserved_usd'],'0.6')
                host.stop(); release.set()
                self.assertEqual(host.wait(3)['status'],'unknown')
                snapshot=ledger.snapshot()
                self.assertEqual(snapshot['status'],'tracking')
                self.assertEqual(snapshot['available_usd'],'1')
                self.assertEqual(snapshot['reservations'][0]['status'],'not_launched')
                self.assertEqual(host.last_result['process'],'not_launched')
                with self.assertRaises(ValueError): host.send('do not restart')
            finally:
                release.set(); ledger.close()


if __name__ == '__main__': unittest.main()
