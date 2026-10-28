"""Actual CLI, local fake model and a parent blocked at an MCP boundary."""
import http.server
import json
import os
from pathlib import Path
import shutil
import tempfile
import threading
import unittest
import uuid

from PIL import Image
from robofrontier.claude.provenance import fork_provenance
from robofrontier.claude.runner import run_turn
from robofrontier.claude.session_host import ClaudeSessionHost
from scripts import claude_offline_probe as probe


@unittest.skipUnless(shutil.which('claude'), 'Claude CLI is not installed')
class ForkBoundaryTests(unittest.TestCase):
    def test_fork_creation_is_tool_free_while_parent_waits_at_mcp(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp) / 'run'
            roles = run/'public/inputs/roles.json'; roles.parent.mkdir(parents=True)
            roles.write_text(json.dumps({'roles': {'Controller': {'kind':'controller'}}}))
            image = run/'public/observations/top.png'; image.parent.mkdir(parents=True)
            Image.new('RGB', (4,4), 'red').save(image)
            entered = threading.Event(); release = threading.Event()
            class World(http.server.BaseHTTPRequestHandler):
                def log_message(self, *args): pass
                def do_POST(self):
                    self.rfile.read(int(self.headers.get('Content-Length', 0))); entered.set(); release.wait(40)
                    payload=json.dumps({'observation':{'images':{'top':str(image)}}}).encode()
                    try:
                        self.send_response(200); self.end_headers(); self.wfile.write(payload)
                    except (BrokenPipeError, ConnectionResetError): pass
            world=http.server.ThreadingHTTPServer(('127.0.0.1',0),World)
            model=http.server.ThreadingHTTPServer(('127.0.0.1',0),probe.H)
            for server in (world,model): threading.Thread(target=server.serve_forever,daemon=True).start()
            probe.REQUESTS.clear()
            host = None
            try:
                connection=run/'private/state/connections/Controller.json'; connection.parent.mkdir(parents=True)
                connection.write_text(json.dumps({'run_id':'run','actor':'Controller','schema_version':3,
                    'url':f'http://127.0.0.1:{world.server_port}','token':'fake'}))
                profile={'model':'claude-opus-5-5','effort':'high','env_key':'FAKE_KEY',
                         'base_url':f'http://127.0.0.1:{model.server_port}'}
                env={'PATH':os.environ.get('PATH',''),'FAKE_KEY':'fake'}
                host=ClaudeSessionHost(run,'Controller',profile,env,turn_timeout=45)
                host.send('Call robot observe tool.')
                self.assertTrue(entered.wait(25),'Parent did not reach the blocked MCP call')
                child=str(uuid.uuid4())
                record=run_turn(profile,profile['model'],profile['effort'],'unused',
                    'This is a no-tools setup turn. Reply READY.',run,env,session_id=child,
                    fork_from=host.session_id,enable_robot=False,timeout=30,events_name='fork-setup.jsonl')
                self.assertEqual(record['exit_code'],0,record)
                self.assertEqual(record['summary']['session_id'],child)
                self.assertEqual(record['summary']['turn_status'],'ended')
                self.assertFalse(probe.REQUESTS[-1].get('tools'))
                proof=fork_provenance(host.session_id,child,run/'private/claude-config')
                self.assertEqual(proof['source_thread_id'],host.session_id)
                self.assertEqual(host.observe()['status'],'active')
            finally:
                release.set()
                if host:
                    host.wait(30)
                    if host.observe()['status']=='active': host.stop(); host.wait(10)
                for server in (model,world): server.shutdown(); server.server_close()


if __name__ == '__main__': unittest.main()
