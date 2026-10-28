"""Opt-in, unscored MuJoCo smoke with real Claude CLI and loopback fake model.

Run with ROBOFRONTIER_PHYSICS_SMOKE=1; requires the R14 assets and graphics.
No provider requests or genuine credentials are used. A failed live owner is
left on disk for inspection rather than deleting its working directory.
"""
import hashlib
import http.server
import json
import os
from pathlib import Path
import shutil
import tempfile
import threading
import unittest

from test_claude_adaptive_integration import StubModel
from robofrontier.claude.engineering_run import run_engineering
from robofrontier.runtime_store import RuntimeStore
from scripts.claude_benchmark import prepare_draft


class MotionModel(StubModel):
    requests = []

    def choose_tool(self, names, text, blocks, has_result):
        if names and 'Your current runtime role is Delegate_' in text and not has_result:
            return {'op':'observe','data':{'cameras':['global']},'views':['global']}, 'toolu_observe'
        if names and has_result:
            ident = blocks[-1].get('tool_use_id')
            if ident == 'toolu_observe':
                return {'op':'move','data':{'robot':'A','delta_m':[0,0,.005]},
                        'call_id':'synthetic-motion'}, 'toolu_move'
            if ident == 'toolu_move':
                return {'op':'return_task','data':{'outcome':'completed','result':'synthetic motion test'}}, 'toolu_return'
        return super().choose_tool(names,text,blocks,has_result)


@unittest.skipUnless(os.environ.get('ROBOFRONTIER_PHYSICS_SMOKE') == '1' and shutil.which('claude'),
                     'Opt-in real physics smoke requires Claude CLI and scene assets')
class PhysicsSmokeTests(unittest.TestCase):
    def test_adaptive_unscored_lifecycle_with_synthetic_model(self):
        directory = Path(tempfile.mkdtemp(prefix='claude-physics-synthetic-'))
        case = os.environ.get('ROBOFRONTIER_PHYSICS_CASE', 'transport-a04-d1')
        record_video = os.environ.get('ROBOFRONTIER_PHYSICS_VIDEO') == '1'
        MotionModel.requests.clear()
        server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), MotionModel)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        result = None
        try:
            base = directory / 'batch'
            prepare_draft(base, ['opus'], cases=[case], topology='adaptive', record_video=record_video)
            run = base / 'opus' / case
            launch = run / 'private/inputs/claude.json'
            data = json.loads(launch.read_text())
            data['profile'].update(base_url=f'http://127.0.0.1:{server.server_port}', env_key='FAKE_KEY')
            launch.write_text(json.dumps(data))
            manifest = run / 'private/inputs/manifest.json'
            frozen = json.loads(manifest.read_text())
            frozen['input_sha256']['private/inputs/claude.json'] = hashlib.sha256(launch.read_bytes()).hexdigest()
            manifest.write_text(json.dumps(frozen))
            env = {'PATH': os.environ['PATH'], 'HOME': os.environ['HOME'], 'FAKE_KEY': 'synthetic'}
            result = run_engineering(run, env, turn_budget_usd=.5, total_budget_usd=2,
                                     wall_limit_s=15, startup_timeout_s=90, poll_s=.1)
            self.assertFalse(result['scored'])
            self.assertEqual(result['reason'], 'wall_limit', result)
            self.assertEqual(result['status'], 'needs_review' if result['budget_review_required'] else 'finished', result)
            self.assertTrue(result['seal']['sealed'], result)
            self.assertFalse(result['owner_alive'], result)
            self.assertEqual(result['seal']['unresolved_action_ids'], [])
            if record_video:
                self.assertTrue(result['seal']['video']['readable'], result['seal']['video'])
            state = RuntimeStore(run / 'private/state/runtime.sqlite')
            try:
                batch = state.get('delegations', {}).get('synthetic-delegate')
                self.assertIsNotNone(batch)
                self.assertEqual(batch['status'], 'finished', batch)
                self.assertEqual(state.get('robot_owners')['A'], 'Controller')
                action=state.actions().get('synthetic-motion')
                self.assertIsNotNone(action)
                self.assertEqual(action['status'], 'completed', action)
                self.assertEqual(action['arm'], 'A')
            finally:
                state.close()
            def contains_image(value):
                if isinstance(value,dict):
                    return value.get('type')=='image' or any(contains_image(v) for v in value.values())
                return isinstance(value,list) and any(contains_image(v) for v in value)
            self.assertTrue(any(contains_image(r.get('messages',[])) for r in MotionModel.requests),
                            'camera image was not returned to the model endpoint')
        finally:
            server.shutdown(); server.server_close()
            if (result is not None and result.get('owner_alive')) or (result is None and (directory/'batch/opus'/case/'private/outputs/owner.log').exists()):
                print('Possible live owner retained for review:', directory)
            else:
                shutil.rmtree(directory)


if __name__ == '__main__': unittest.main()
