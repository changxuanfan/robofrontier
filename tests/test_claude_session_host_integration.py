"""Actual Claude CLI + loopback stub services, without network/provider access."""
import http.server
import json
import os
from pathlib import Path
import shutil
import tempfile
import threading
import unittest

from PIL import Image
from robofrontier.claude.session_host import ClaudeSessionHost
from robofrontier.claude.dispatch import ClaudeDispatcher
from robofrontier.runtime_store import RuntimeStore
from scripts import claude_offline_probe as probe


@unittest.skipUnless(shutil.which('claude'), 'Claude CLI is not installed')
class ClaudeSessionHostIntegrationTests(unittest.TestCase):
    def test_native_cli_turn_and_resume_with_role_bound_image_tool(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp) / 'run'
            roles = run / 'public/inputs/roles.json'; roles.parent.mkdir(parents=True)
            roles.write_text(json.dumps({'roles': {'Controller': {'kind': 'controller'}}}))
            image = run / 'public/observations/top.png'; image.parent.mkdir(parents=True)
            Image.new('RGB', (4, 4), 'red').save(image)
            probe.World.image = image; probe.REQUESTS.clear()
            world = http.server.ThreadingHTTPServer(('127.0.0.1', 0), probe.World)
            server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), probe.H)
            for instance in (world, server): threading.Thread(target=instance.serve_forever, daemon=True).start()
            try:
                conn = run / 'private/state/connections/Controller.json'; conn.parent.mkdir(parents=True)
                conn.write_text(json.dumps({'run_id': 'run', 'actor': 'Controller', 'schema_version': 3,
                                            'url': f'http://127.0.0.1:{world.server_port}', 'token': 'fake'}))
                profile = {'model': 'claude-opus-5-5', 'effort': 'high', 'env_key': 'FAKE_KEY',
                           'base_url': f'http://127.0.0.1:{server.server_port}'}
                host = ClaudeSessionHost(run, 'Controller', profile,
                                         {'PATH': os.environ.get('PATH', ''), 'FAKE_KEY': 'fake'}, turn_timeout=45)
                host.send('Call robot observe tool.')
                self.assertEqual(host.wait(60)['status'], 'idle', host.last_result)
                first = host.session_id
                self.assertGreaterEqual(len(probe.REQUESTS), 2)
                self.assertIn('"type": "image"', json.dumps(probe.REQUESTS[-1]))
                cfg = json.loads((Path(__file__).resolve().parents[1] / 'configs/harnesses/policy.json').read_text())
                cfg.update(engine='claude', topology='centralized', startup_protocol='ready_barrier_v1')
                store = RuntimeStore(run / 'private/state/runtime.sqlite')
                try:
                    store.initialize('run', cfg, ['A'])
                    store.update_agent('Controller', ready=True, thread_id=first)
                    store.start()
                    dispatched = ClaudeDispatcher(store, host).cycle()
                    self.assertEqual(len(dispatched['dispatched']), 1)
                    self.assertEqual(host.wait(60)['status'], 'idle', host.last_result)
                    self.assertEqual(host.session_id, first)
                    self.assertIn('toolu_test', json.dumps(probe.REQUESTS[-1]['messages']))
                    self.assertEqual(store.get('lifecycle'), 'running')
                finally:
                    store.close()
            finally:
                for instance in (server, world): instance.shutdown(); instance.server_close()


if __name__ == '__main__': unittest.main()
