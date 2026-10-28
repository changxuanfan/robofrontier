import json
import multiprocessing
import os
from pathlib import Path
import signal
import sys
import tempfile
import threading
import time
import unittest

from robofrontier.claude.engineering_batch import run_batch, status_batch, stop_batch


class BatchTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.root = Path(self.tmp.name) / 'batch'; self.root.mkdir()
        self.jobs = []
        for i in range(4):
            case = f'case{i}'; run = self.root / 'fake' / case
            (run/'private/inputs').mkdir(parents=True); (run/'private/state').mkdir(parents=True)
            profile = {'provider': 'anthropic', 'model': 'fake', 'effort': 'high', 'env_key': 'FAKE_KEY'}
            (run/'private/inputs/claude.json').write_text(json.dumps({'schema':'robofrontier.claude-launch-draft.v1',
                'qualified':False,'execution_available':False,'profile':profile}))
            (run/'private/inputs/harness.json').write_text(json.dumps({'engine':'claude', 'topology':'centralized',
                                                                       'model':'fake','effort':'high'}))
            self.jobs.append({'profile':'fake', 'case_id':case, 'run_directory':str(run), 'status':'draft'})
        (self.root/'batch.json').write_text(json.dumps({'schema':'robofrontier.claude-draft.v1',
            'status':'prepared_not_qualified', 'execution_available':False, 'jobs':self.jobs}))
        self.env = {'FAKE_KEY':'secret'}

    def tearDown(self): self.tmp.cleanup()

    def run_case(self, runner, parallel=2):
        return run_batch(self.root, self.env, parallel=parallel, turn_budget_usd=.2, total_budget_usd=1, wall_limit_s=5,
                         runner=runner, verifier=lambda _: [])

    def test_parallel_limit_all_jobs_and_single_use(self):
        guard = threading.Lock(); counts = {'active':0, 'maximum':0, 'total':0}
        def runner(*args, **kwargs):
            with guard:
                counts['active'] += 1; counts['total'] += 1
                counts['maximum'] = max(counts['active'], counts['maximum'])
            time.sleep(.08)
            with guard: counts['active'] -= 1
            return {'status':'finished', 'reason':'world_ended'}
        state = self.run_case(runner)
        self.assertEqual(counts, {'active':0, 'maximum':2, 'total':4})
        self.assertEqual(state['status'], 'closed')
        self.assertEqual([x['status'] for x in state['jobs']], ['closed']*4)
        self.assertEqual(len(status_batch(self.root)['runs']), 4)
        with self.assertRaisesRegex(ValueError, 'never restart'): self.run_case(runner)

    def test_stop_marks_active_and_skips_queued(self):
        entered = threading.Event(); finish = threading.Event(); outcome = []
        def runner(*args, **kwargs): entered.set(); finish.wait(3); return {'status':'finished', 'reason':'stop_requested'}
        thread = threading.Thread(target=lambda: outcome.append(self.run_case(runner, parallel=1)))
        thread.start(); self.assertTrue(entered.wait(3))
        self.assertTrue(stop_batch(self.root)['stop_requested'])
        finish.set(); thread.join(3); self.assertFalse(thread.is_alive())
        self.assertEqual(outcome[0]['status'], 'stopped')
        self.assertEqual([j['status'] for j in outcome[0]['jobs']], ['closed'] + ['stopped_before_start']*3)
        self.assertTrue(all((Path(j['run_directory'])/'STOP').exists() for j in self.jobs))

    def test_cli_sigterm_requests_stop_and_waits_for_active_job(self):
        ctx = multiprocessing.get_context('fork')
        entered = ctx.Event()
        def child():
            from scripts.claude_benchmark import main
            import robofrontier.claude.engineering_batch as batch_module
            original = batch_module.run_batch
            def runner(run, *args, **kwargs):
                entered.set()
                deadline=time.monotonic()+10
                while not (Path(run)/'STOP').exists() and time.monotonic()<deadline:
                    time.sleep(.02)
                return {'status':'finished','reason':'stop_requested'}
            def launch(*args, **kwargs):
                return original(*args, **kwargs, runner=runner, verifier=lambda _: [])
            batch_module.run_batch=launch
            os.environ['FAKE_KEY']='synthetic'
            sys.argv=['claude_benchmark','run-engineering',str(self.root),'--parallel','1',
                      '--turn-budget-usd','.2','--task-budget-usd','1','--wall-limit-s','5',
                      '--confirm-engineering-spend']
            main()
        process=ctx.Process(target=child); process.start()
        try:
            self.assertTrue(entered.wait(10))
            os.kill(process.pid, signal.SIGTERM)
            process.join(10)
            self.assertFalse(process.is_alive())
            self.assertEqual(process.exitcode,0)
            state=status_batch(self.root)
            self.assertEqual(state['status'],'stopped')
            self.assertEqual([j['status'] for j in state['jobs']],
                             ['closed']+['stopped_before_start']*3)
            self.assertFalse(state['supervisor_live'])
        finally:
            if process.is_alive(): process.kill(); process.join(10)
            process.close()

    def test_failed_run_and_incompatible_profile_are_not_hidden(self):
        def fail(*args, **kwargs): raise RuntimeError('not for user output')
        result = self.run_case(fail)
        self.assertEqual(result['status'], 'needs_review')
        self.assertEqual(result['jobs'][0]['error_type'], 'RuntimeError')
        self.assertNotIn('not for user output', json.dumps(result))

    def test_preflight_checks_all_before_start_and_rejects_path_escape(self):
        harness = Path(self.jobs[-1]['run_directory'])/'private/inputs/harness.json'
        data = json.loads(harness.read_text()); data['topology'] = 'unsupported'; harness.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, 'engineering launch profile'):
            self.run_case(lambda *a, **k: self.fail('model call'))
        self.assertFalse((self.root/'engineering-claim.json').exists())
        batch = json.loads((self.root/'batch.json').read_text()); batch['jobs'][0]['run_directory'] = '/tmp/outside'
        (self.root/'batch.json').write_text(json.dumps(batch))
        with self.assertRaisesRegex(ValueError, 'run directory'): stop_batch(self.root)
        self.assertFalse((self.root/'STOP').exists())

    def test_adaptive_draft_is_admitted_to_unqualified_engineering_batch(self):
        for job in self.jobs:
            harness = Path(job['run_directory'])/'private/inputs/harness.json'
            data = json.loads(harness.read_text()); data['topology'] = 'adaptive'
            harness.write_text(json.dumps(data))
        calls=[]
        def runner(run, *args, **kwargs):
            calls.append(run)
            return {'status':'finished', 'reason':'world_ended'}
        result=self.run_case(runner)
        self.assertEqual(result['status'], 'closed')
        self.assertFalse(result['scored'])
        self.assertEqual(len(calls), len(self.jobs))

    def test_stale_running_status_is_not_claimed_to_be_live(self):
        (self.root/'engineering-status.json').write_text(json.dumps({'status': 'running'}))
        (self.root/'engineering-supervisor.lock').touch()
        self.assertEqual(status_batch(self.root)['status'], 'supervisor_lost_needs_review')
        self.assertFalse(status_batch(self.root)['supervisor_live'])

    def test_real_supervisor_process_loss_preserves_claim_and_does_not_replay(self):
        ctx = multiprocessing.get_context('fork')
        entered = ctx.Event()
        def runner(run, *args, **kwargs):
            (Path(run)/'private/state/claude-engineering-claim.json').write_text('{}')
            entered.set()
            time.sleep(30)
            return {'status':'finished', 'reason':'world_ended'}
        process = ctx.Process(target=self.run_case, args=(runner, 1))
        process.start()
        try:
            self.assertTrue(entered.wait(10), 'Test supervisor did not start its fake job')
            self.assertTrue(status_batch(self.root)['supervisor_live'])
            process.terminate()
            process.join(10)
            self.assertFalse(process.is_alive())
            status = status_batch(self.root)
            self.assertEqual(status['status'], 'supervisor_lost_needs_review')
            self.assertFalse(status['supervisor_live'])
            self.assertTrue((self.root/'engineering-claim.json').exists())
            self.assertEqual(status['jobs'][0]['status'], 'starting')
            self.assertTrue(status['runs'][0]['claim_record_present'])
            self.assertTrue(all(not run['claim_record_present'] for run in status['runs'][1:]))
            self.assertTrue(stop_batch(self.root)['stop_requested'])
            stopped = status_batch(self.root)
            self.assertEqual(stopped['status'], 'supervisor_lost_needs_review')
            self.assertTrue(stopped['stop_requested'])
            self.assertTrue(all(item['stop_requested'] for item in stopped['runs']))
            with self.assertRaisesRegex(ValueError, 'never restart'):
                self.run_case(lambda *a, **k: self.fail('unexpected launch'))
        finally:
            if process.is_alive():
                process.kill()
                process.join(10)
            process.close()

    def test_claim_without_status_needs_review_and_cannot_restart(self):
        (self.root/'engineering-claim.json').write_text('{}')
        self.assertEqual(status_batch(self.root)['status'], 'claim_without_status_needs_review')
        with self.assertRaisesRegex(ValueError, 'never restart'):
            self.run_case(lambda *a, **k: self.fail('unexpected launch'))

    def test_stop_before_launch_is_visible_and_blocks_launch(self):
        self.assertFalse(status_batch(self.root)['stop_requested'])
        stop_batch(self.root)
        self.assertEqual(status_batch(self.root)['status'], 'stopped_before_start')
        self.assertTrue(status_batch(self.root)['stop_requested'])
        with self.assertRaisesRegex(ValueError, 'stopped before launch'):
            self.run_case(lambda *a, **k: self.fail('unexpected launch'))
        self.assertFalse((self.root/'engineering-claim.json').exists())

    def test_review_takes_precedence_over_stop(self):
        entered = threading.Event(); finish = threading.Event(); outcome = []
        def runner(*args, **kwargs):
            entered.set(); finish.wait(3)
            return {'status':'needs_review', 'reason':'owner_unavailable'}
        thread = threading.Thread(target=lambda: outcome.append(self.run_case(runner, parallel=1)))
        thread.start(); self.assertTrue(entered.wait(3))
        stop_batch(self.root); finish.set(); thread.join(3)
        self.assertFalse(thread.is_alive())
        self.assertEqual(outcome[0]['status'], 'needs_review')
        self.assertEqual(outcome[0]['jobs'][1]['status'], 'stopped_before_start')

    def test_invalid_oauth_endpoint_rejects_entire_batch_before_claim(self):
        launch = Path(self.jobs[-1]['run_directory'])/'private/inputs/claude.json'
        data = json.loads(launch.read_text())
        data['profile'].update(auth_mode='oauth_token', env_key='CLAUDE_CODE_OAUTH_TOKEN',
                               base_url='https://other.example')
        launch.write_text(json.dumps(data))
        self.env['CLAUDE_CODE_OAUTH_TOKEN'] = 'synthetic'
        with self.assertRaisesRegex(ValueError, 'Anthropic endpoint'):
            self.run_case(lambda *a, **k: self.fail('unexpected launch'))
        self.assertFalse((self.root/'engineering-claim.json').exists())


if __name__ == '__main__': unittest.main()
