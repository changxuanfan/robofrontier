"""Offline lifecycle using the real runtime, scheduler and a synthetic world."""
import hashlib
import http.server
import json
from pathlib import Path
import tempfile
import threading
import unittest

from fake_pipeline import FakeWorld, fake_capture
from robofrontier.claude.policy_host import CentralizedPolicyHost
from robofrontier.pipeline_client import client_call
from robofrontier.run_setup import prepare_run
from robofrontier.runtime_service import RuntimeService

ROOT = Path(__file__).resolve().parents[1]


class Host:
    def __init__(self, run, actor, profile, base_env):
        self.actor = actor
        self.session_id = 'offline-session'
        self.state = 'new'
        self.turn = None
        self.prompts = []

    def send(self, prompt):
        self.prompts.append(prompt)
        self.state = 'active'
        self.turn = 'turn-' + str(len(self.prompts))
        return {'threadId': self.session_id, 'turnId': self.turn}

    def observe(self):
        return {'actor': self.actor, 'thread_id': self.session_id, 'status': self.state, 'turn_id': self.turn}

    def stop(self):
        self.state = 'unknown'


class LifecycleTests(unittest.TestCase):
    def test_frozen_inputs_real_runtime_start_and_scheduler(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            cfg = json.loads((ROOT / 'configs/harnesses/policy.json').read_text())
            cfg.update(engine='claude', topology='centralized', model='fake', effort='high')
            config_path = root / 'config.json'
            config_path.write_text(json.dumps(cfg))
            run = prepare_run(root / 'run', harness_path=config_path)
            prompt = run / 'public/inputs/claude-controller.md'
            prompt.write_text((ROOT / 'prompts/claude/controller.md').read_text())
            manifest_path = run / 'private/inputs/manifest.json'
            manifest = json.loads(manifest_path.read_text())
            manifest['input_sha256']['public/inputs/claude-controller.md'] = hashlib.sha256(prompt.read_bytes()).hexdigest()
            manifest_path.write_text(json.dumps(manifest))
            service = RuntimeService(run, FakeWorld, fake_capture)

            class Handler(http.server.BaseHTTPRequestHandler):
                def log_message(self, *args): pass
                def do_POST(self):
                    body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                    if self.path != '/op' or service.tokens.get(body['actor']) != self.headers.get('X-Token'):
                        self.send_error(403); return
                    with service.lock:
                        job = service.accept(body['actor'], body['op'], body['data'], body['call_id'], body.get('ack_delivery'))
                        service.pump()
                    result = json.dumps(job['result']).encode()
                    self.send_response(200); self.end_headers(); self.wfile.write(result)

            server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), Handler)
            threading.Thread(target=server.serve_forever, daemon=True).start()
            for actor in ('Meta', 'Controller'):
                conn = run / 'private/state/connections' / (actor + '.json')
                data = json.loads(conn.read_text()); data['url'] = f'http://127.0.0.1:{server.server_port}'
                conn.write_text(json.dumps(data))
            host = CentralizedPolicyHost(run, {'model': 'fake', 'effort': 'high'}, {}, host_factory=Host)
            try:
                host.bind_and_bootstrap()
                with self.assertRaisesRegex(ValueError, 'READY'):
                    host.start_when_ready()
                result = client_call(run / 'private/state/connections/Controller.json', 'ready', {}, 'ready-1', log_call_id=False)
                self.assertTrue(result['ready'])
                host.host.state = 'idle'
                self.assertTrue(host.start_when_ready()['started'])
                self.assertEqual(service.store.get('lifecycle'), 'running')
                self.assertEqual(len(host.cycle()['dispatched']), 1)
                self.assertEqual(len(host.host.prompts), 2)
                host.host.state = 'idle'
            finally:
                host.close()
                server.shutdown(); server.server_close()
                service.close()


if __name__ == '__main__': unittest.main()
