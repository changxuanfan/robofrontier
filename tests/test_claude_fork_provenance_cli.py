"""Pin fork provenance to the real Claude CLI's transcripts, using a local stub model only.

No provider is contacted: the CLI talks to an in-process fake Anthropic endpoint and the
robot MCP talks to a fake world. The test is skipped when no `claude` binary is installed.
"""
import http.server
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading
import unittest

from scripts import claude_offline_probe as probe
from robofrontier.claude.provenance import fork_provenance

ROOT = Path(__file__).resolve().parents[1]
PARENT = '44444444-4444-4444-8444-444444444444'
CHILD = '55555555-5555-4555-8555-555555555555'


@unittest.skipUnless(shutil.which('claude'), 'Claude Code CLI is not installed')
class RealCliForkTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        t = Path(self.tmp.name)
        self.run_dir = t / 'run'
        self.config = t / 'config'
        conn = self.run_dir / 'private/state/connections/Controller.json'
        conn.parent.mkdir(parents=True)
        image = self.run_dir / 'public/observations/top.png'
        image.parent.mkdir(parents=True)
        from PIL import Image
        Image.new('RGB', (4, 4), 'red').save(image)
        probe.World.image = image
        self.world = http.server.ThreadingHTTPServer(('127.0.0.1', 0), probe.World)
        self.server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), probe.H)
        for server in (self.world, self.server):
            threading.Thread(target=server.serve_forever, daemon=True).start()
        conn.write_text(json.dumps({'run_id': 'run', 'actor': 'Controller', 'schema_version': 3,
                                    'url': 'http://127.0.0.1:%d' % self.world.server_port, 'token': 'fake'}))
        self.mcp = t / 'mcp.json'
        self.mcp.write_text(json.dumps({'mcpServers': {'robot': {
            'command': sys.executable, 'args': ['-m', 'robofrontier.claude.robot_mcp', '--run-dir', str(self.run_dir),
                                                '--connection', str(conn)], 'env': {'PYTHONPATH': str(ROOT)}}}}))
        self.env = {'PATH': os.environ.get('PATH', ''), 'HOME': str(t),
                    'ANTHROPIC_BASE_URL': 'http://127.0.0.1:%d' % self.server.server_port,
                    'ANTHROPIC_API_KEY': 'local-fake-key', 'CLAUDE_CONFIG_DIR': str(self.config),
                    'DISABLE_TELEMETRY': '1', 'DISABLE_ERROR_REPORTING': '1',
                    'CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC': '1'}

    def tearDown(self):
        for server in (self.world, self.server):
            server.shutdown()
            server.server_close()
        self.tmp.cleanup()

    def turn(self, *session_flags):
        command = ['claude', '--bare', '-p', 'Call robot observe tool.', '--model', 'claude-opus-5-5',
                   '--mcp-config', str(self.mcp), '--strict-mcp-config', '--allowedTools', 'mcp__robot__robot',
                   '--permission-mode', 'dontAsk', '--tools', '', '--output-format', 'stream-json', '--verbose',
                   *session_flags]
        done = subprocess.run(command, cwd=self.run_dir, env=self.env, text=True, capture_output=True,
                              stdin=subprocess.DEVNULL, timeout=90)
        self.assertEqual(done.returncode, 0, done.stderr[-500:])

    def test_forked_transcript_is_proven_and_tampering_is_caught(self):
        self.turn('--session-id', PARENT)
        self.turn('--resume', PARENT, '--fork-session', '--session-id', CHILD)
        proof = fork_provenance(PARENT, CHILD, self.config)
        self.assertEqual(proof['source_thread_id'], PARENT)
        self.assertGreaterEqual(proof['source_ordinal'], 4)  # prompt, tool call, tool result, reply
        self.assertEqual(proof, fork_provenance(PARENT, CHILD, self.config))
        parent_file = next(self.config.glob('projects/*/%s.jsonl' % PARENT))
        text = parent_file.read_text()
        self.assertIn('"role":"assistant"', text.replace(' ', ''))
        self.assertIn('Call robot observe tool.', text)
        parent_file.write_text(text.replace('Call robot observe tool.', 'Call something else.'))
        with self.assertRaises(ValueError):
            fork_provenance(PARENT, CHILD, self.config)


if __name__ == '__main__':
    unittest.main()
