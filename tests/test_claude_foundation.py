import base64
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts.claude_benchmark import plan, profile
from robofrontier.claude.robot_mcp import RobotBridge, handle


class ClaudeFoundationTests(unittest.TestCase):
    def test_plan_four_minimum_level_one_tasks_without_execution(self):
        result = plan(['kimi'], level=1, min_arms=True, parallel=4)
        self.assertEqual(result['task_count'], 4)
        self.assertEqual([j['family'] for j in result['jobs']], ['transport', 'mosaic', 'relay', 'dial'])
        self.assertEqual([j['arms'] for j in result['jobs']], [4, 4, 8, 4])
        self.assertEqual(result['model_calls'], 0)
        self.assertFalse(result['execution_available'])

    def test_precheck_uses_haiku_and_bench_keeps_opus(self):
        self.assertEqual(profile('haiku')['model'], 'claude-haiku-5-5')
        self.assertEqual(profile('opus')['model'], 'claude-opus-5-5')
        self.assertEqual(profile('haiku')['env_key'], 'ANTHROPIC_API_KEY')

    def test_invalid_provider_address_and_parallel_are_rejected(self):
        with patch.dict('os.environ', {'DASHSCOPE_ANTHROPIC_BASE_URL': 'https://secret:pass@example.com'}):
            with self.assertRaises(ValueError): profile('qwen')
        with self.assertRaises(ValueError): plan(['kimi'], parallel=0)
        with self.assertRaises(ValueError): plan(['kimi', 'kimi'])

    def test_role_binding_image_and_stable_call_id(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp) / 'run'; connection = run / 'private/state/connections/Controller.json'
            connection.parent.mkdir(parents=True)
            connection.write_text(json.dumps({'run_id': 'run', 'actor': 'Controller', 'token': 'secret'}))
            image = run / 'public/observations/scene.png'; image.parent.mkdir(parents=True)
            png = b'\x89PNG\r\n\x1a\nimage'; image.write_bytes(png)
            calls = []
            def invoke(conn, op, data, call_id, **kwargs):
                calls.append((conn, op, data, call_id))
                return {'observation': {'images': {'top': str(image)}}}
            bridge = RobotBridge(run, connection, invoke)
            with self.assertRaisesRegex(ValueError, 'stable call_id'):
                bridge.call({'op': 'move', 'data': {}})
            response = bridge.call({'op': 'move', 'data': {}, 'call_id': 'move-1', 'views': ['top']})
            self.assertEqual(calls[0][1:], ('move', {'cameras': ['top']}, 'move-1'))
            self.assertEqual(base64.b64decode(response['content'][1]['data']), png)
            self.assertNotIn('secret', json.dumps(response))
            with self.assertRaises(ValueError): bridge.call({'op': 'start', 'data': {}})
            self.assertEqual(handle({'method': 'tools/list'}, bridge)['tools'][0]['name'], 'robot')
            outside = Path(tmp) / 'outside.png'; outside.write_bytes(png)
            image.unlink(); image.symlink_to(outside)
            with self.assertRaisesRegex(ValueError, 'outside'):
                bridge.call({'op': 'observe', 'data': {}, 'views': ['top']})

    def test_public_images_are_limited_to_frozen_task_allowlist(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp) / 'run'; connection = run / 'private/state/connections/Controller.json'
            connection.parent.mkdir(parents=True)
            connection.write_text(json.dumps({'run_id': 'run', 'actor': 'Controller'}))
            public = run / 'public/inputs'; public.mkdir(parents=True)
            (public / 'task.json').write_text(json.dumps({'initial_reference_images': ['family/view.png'],
                                                         'robot_layout_image': 'public/inputs/robot-layout.png'}))
            image = public / 'family/view.png'; image.parent.mkdir()
            png = b'\x89PNG\r\n\x1a\nimage'; image.write_bytes(png)
            (public / 'robot-layout.png').write_bytes(png)
            bridge = RobotBridge(run, connection)
            response = bridge.call({'op': 'public_image', 'data': {'names': ['family/view.png']}})
            self.assertEqual(base64.b64decode(response['content'][1]['data']), png)
            layout = bridge.call({'op': 'public_image', 'data': {'names': ['public/inputs/robot-layout.png']}})
            self.assertEqual(base64.b64decode(layout['content'][1]['data']), png)
            with self.assertRaises(ValueError):
                bridge.call({'op': 'public_image', 'data': {'names': ['../private/secret.png']}})
            image.unlink(); image.symlink_to(Path(tmp) / 'outside.png')
            with self.assertRaises(ValueError):
                bridge.call({'op': 'public_image', 'data': {'names': ['family/view.png']}})

    def test_connection_cannot_be_manager(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp) / 'r'; conn = run / 'private/state/connections/Meta.json'; conn.parent.mkdir(parents=True)
            conn.write_text(json.dumps({'run_id': 'r', 'actor': 'Meta'}))
            with self.assertRaises(ValueError): RobotBridge(run, conn)


if __name__ == '__main__': unittest.main()
