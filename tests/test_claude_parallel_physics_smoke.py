"""Opt-in four-way unscored physical smoke; real CLI, synthetic loopback model."""
import hashlib
import http.server
import json
import os
from pathlib import Path
import shutil
import tempfile
import threading
import time
import unittest

from test_claude_physics_smoke import MotionModel
from robofrontier.claude.engineering_batch import run_batch, status_batch
from robofrontier.runtime_store import RuntimeStore
from scripts.claude_benchmark import prepare_draft


class CompactRequests(list):
    def append(self, body):
        # Keep summaries only: repeated multimodal requests can contain large PNGs.
        def image(value):
            if isinstance(value, dict): return value.get('type') == 'image' or any(image(v) for v in value.values())
            return isinstance(value, list) and any(image(v) for v in value)
        super().append(image(body.get('messages', [])))


class BatchMotionModel(MotionModel):
    requests = CompactRequests()


@unittest.skipUnless(os.environ.get('ROBOFRONTIER_PHYSICS_BATCH') == '1' and shutil.which('claude'),
                     'Opt-in four-process physics smoke requires Claude CLI and scene assets')
class ParallelPhysicsSmokeTests(unittest.TestCase):
    def test_four_cases_use_distinct_worlds_and_close(self):
        directory = Path(tempfile.mkdtemp(prefix='claude-parallel-physics-'))
        server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), BatchMotionModel)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        BatchMotionModel.requests.clear()
        outcome = []
        failures = []
        worker = None
        try:
            batch = directory / 'batch'
            prepare_draft(batch, ['opus'], level=1, min_arms=True, topology='adaptive')
            jobs = json.loads((batch / 'batch.json').read_text())['jobs']
            self.assertEqual(len(jobs), 4)
            for job in jobs:
                run = Path(job['run_directory'])
                launch = run / 'private/inputs/claude.json'
                data = json.loads(launch.read_text())
                data['profile'].update(base_url=f'http://127.0.0.1:{server.server_port}', env_key='FAKE_KEY')
                launch.write_text(json.dumps(data))
                manifest = run / 'private/inputs/manifest.json'
                frozen = json.loads(manifest.read_text())
                frozen['input_sha256']['private/inputs/claude.json'] = hashlib.sha256(launch.read_bytes()).hexdigest()
                manifest.write_text(json.dumps(frozen))
            env = {'PATH': os.environ['PATH'], 'HOME': os.environ['HOME'], 'FAKE_KEY': 'synthetic'}
            def launch_batch():
                try:
                    outcome.append(run_batch(batch, env, parallel=4, turn_budget_usd=.5,
                                             total_budget_usd=2, wall_limit_s=20))
                except Exception as error:
                    failures.append(repr(error))
            worker = threading.Thread(target=launch_batch)
            worker.start()
            maximum_live = 0
            while worker.is_alive():
                status = status_batch(batch)
                live = set()
                for item in status['runs']:
                    detail = item.get('detail') or {}
                    pid = detail.get('owner_pid')
                    if not pid: continue
                    try: os.kill(pid, 0)
                    except ProcessLookupError: continue
                    live.add(pid)
                maximum_live = max(maximum_live, len(live))
                worker.join(.25)
            self.assertFalse(failures, failures)
            self.assertEqual(outcome[0]['status'], 'closed', outcome[0])
            self.assertFalse(outcome[0]['scored'])
            self.assertEqual(maximum_live, 4, f'Expected four overlapping physical owners, observed {maximum_live}')
            self.assertTrue(any(BatchMotionModel.requests), 'No camera image reached synthetic endpoint')
            for job in jobs:
                run = Path(job['run_directory'])
                detail = json.loads((run/'private/outputs/claude-engineering-status.json').read_text())
                self.assertEqual(detail['status'], 'finished', detail)
                self.assertTrue(detail['seal']['sealed'], detail)
                state = RuntimeStore(run/'private/state/runtime.sqlite')
                try:
                    self.assertEqual(state.get('delegations', {})['synthetic-delegate']['status'], 'finished')
                    self.assertEqual(state.actions()['synthetic-motion']['status'], 'completed')
                    self.assertEqual(state.get('robot_owners')['A'], 'Controller')
                finally:
                    state.close()
        finally:
            # Never remove files backing a failed, uncertain or still-running owner.
            if worker is not None and worker.is_alive():
                print('Running batch retained for review:', directory)
            elif outcome and outcome[0].get('status') == 'closed':
                shutil.rmtree(directory)
            else:
                print('Uncertain batch retained for review:', directory)
            server.shutdown(); server.server_close()


if __name__ == '__main__': unittest.main()
