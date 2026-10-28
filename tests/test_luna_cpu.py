import json
import os
from pathlib import Path
import sqlite3
import tempfile
import unittest
from unittest.mock import patch

from scripts.benchmark import profile, launch_config
from scripts.luna_cpu import cpu_environment, usage_report


class LunaCpuTests(unittest.TestCase):
    def test_profile_requires_openai_api_and_max(self):
        p = profile('luna')
        with tempfile.TemporaryDirectory() as folder:
            launch = launch_config(p, Path(folder))
            self.assertEqual((p['model'], p['effort']), ('gpt-6-luna', 'max'))
            self.assertIn('model_providers.openai_api.env_key="OPENAI_API_KEY"', launch['args'])
            self.assertIn('model_providers.openai_api.base_url="https://api.openai.com/v1"', launch['args'])
            self.assertIn('model_providers.openai_api.wire_api="responses"', launch['args'])

    def test_cpu_environment_overrides_gpu_backend(self):
        with patch.dict(os.environ, {'MUJOCO_GL': 'egl', 'CUDA_VISIBLE_DEVICES': '0'}):
            env = cpu_environment()
            self.assertEqual(env['MUJOCO_GL'], 'osmesa')
            self.assertEqual(env['CUDA_VISIBLE_DEVICES'], '')
            self.assertEqual(env['LIBGL_ALWAYS_SOFTWARE'], 'true')

    def test_usage_counts_unique_response_not_inherited_history(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); run = root/'run'; db = run/'private/state/runtime.sqlite'
            db.parent.mkdir(parents=True)
            with sqlite3.connect(db) as conn:
                conn.execute('create table agents(id text, value text)')
                conn.executemany('insert into agents values (?, ?)', [
                    ('controller', json.dumps({'thread_id': 'parent'})),
                    ('delegate', json.dumps({'thread_id': 'child'}))])
            batch = root/'batch'; batch.mkdir()
            (batch/'batch.json').write_text(json.dumps({'jobs': [{'case_id': 'demo', 'run_directory': str(run)}]}))
            sessions = root/'home/sessions/2026/01/01'; sessions.mkdir(parents=True)
            def record(thread, response, amount):
                return json.dumps({'type': 'token_usage_record', 'payload': {'thread_id': thread,
                    'response_id': response, 'usage': {'input_tokens': amount, 'cached_input_tokens': amount//2}}})+'\n'
            (sessions/'x-parent.jsonl').write_text(record('parent', 'one', 100)*2)
            (sessions/'x-child.jsonl').write_text(record('child', 'one', 100)+record('child', 'two', 200))
            with patch.dict(os.environ, {'CODEX_HOME': str(root/'home')}):
                item = usage_report(batch)['jobs'][0]
            self.assertEqual(item['input_tokens'], 300)
            self.assertEqual(item['cached_input_tokens'], 150)
            self.assertEqual(item['cache_hit_ratio'], .5)
            self.assertEqual(item['missing_traces'], [])


if __name__ == '__main__': unittest.main()
