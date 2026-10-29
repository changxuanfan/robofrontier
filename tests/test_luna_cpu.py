import json
import os
from pathlib import Path
import sqlite3
import tempfile
import unittest
from unittest.mock import patch

from scripts.benchmark import profile, launch_config, select_cases
from scripts.luna_cpu import cpu_environment, interval_union, usage_report, collect_batch, main


class LunaCpuTests(unittest.TestCase):
    def test_l3_max_test_prepares_exact_four_maximum_arm_cases(self):
        with tempfile.TemporaryDirectory() as tmp, patch.dict(os.environ, {}, clear=False):
            output = Path(tmp)/'batch'; output.mkdir()
            with patch('sys.argv', ['luna_cpu', 'l3-max-test', '--output', str(output), '--prepare-only']), \
                 patch('scripts.luna_cpu.ROOT', Path(tmp)), patch('scripts.luna_cpu.load_env'), \
                 patch('scripts.luna_cpu.subprocess.call', return_value=0) as call:
                self.assertEqual(main(), 0)
            command = call.call_args.args[0]
            self.assertEqual(command[command.index('--wall-limit-s') + 1], '3600')
            self.assertEqual(command[command.index('--cases') + 1:], [
                'transport-a12-d3', 'mosaic-a08-12x24-d3', 'relay-a16-d3', 'dial-best5-n08-d3'])
            self.assertEqual(call.call_count, 1)

    def test_l3_max_test_starts_four_in_parallel(self):
        with tempfile.TemporaryDirectory() as tmp, patch.dict(os.environ, {'OPENAI_API_KEY':'test'}, clear=False):
            output = Path(tmp)/'batch'; output.mkdir()
            with patch('sys.argv', ['luna_cpu', 'l3-max-test', '--output', str(output)]), \
                 patch('scripts.luna_cpu.ROOT', Path(tmp)), patch('scripts.luna_cpu.load_env'), \
                 patch('scripts.luna_cpu.sandbox_probe'), patch('scripts.luna_cpu.render_probe', return_value={}), \
                 patch('scripts.luna_cpu.subprocess.call', return_value=0) as call:
                self.assertEqual(main(), 0)
            self.assertEqual(call.call_count, 2)
            self.assertEqual(call.call_args_list[1].args[0][-2:], ['--parallel', '4'])

    def test_l3_max_test_supports_a_30_minute_limit(self):
        with tempfile.TemporaryDirectory() as tmp, patch.dict(os.environ, {}, clear=False):
            output = Path(tmp)/'batch'; output.mkdir()
            with patch('sys.argv', ['luna_cpu', 'l3-max-test', '--output', str(output), '--prepare-only', '--wall-limit-s', '1800']), \
                 patch('scripts.luna_cpu.ROOT', Path(tmp)), patch('scripts.luna_cpu.load_env'), \
                 patch('scripts.luna_cpu.subprocess.call', return_value=0) as call:
                self.assertEqual(main(), 0)
            command = call.call_args.args[0]
            self.assertEqual(command[command.index('--wall-limit-s') + 1], '1800.0')

    def test_collect_copies_controller_and_delegate_and_detects_missing(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); batch=root/'batch'; run=batch/'luna/demo'; state=run/'private/state'
            state.mkdir(parents=True); outputs=run/'private/outputs'; outputs.mkdir()
            (outputs/'evaluation.json').write_text('{}')
            with sqlite3.connect(state/'runtime.sqlite') as conn:
                conn.execute('create table agents(id text,value text)')
                conn.executemany('insert into agents values (?,?)', [('controller',json.dumps({'thread_id':'parent'})),
                                                                  ('delegate',json.dumps({'thread_id':'child'}))])
            (batch/'batch.json').write_text(json.dumps({'status':'closed','jobs':[{'case_id':'demo','status':'closed','run_directory':str(run)}]}))
            home=root/'home'; sessions=home/'sessions/2026'; sessions.mkdir(parents=True)
            (batch/'codex-home.json').write_text(json.dumps({'codex_home':str(home)}))
            (sessions/'rollout-parent.jsonl').write_text('{}\n')
            report=collect_batch(batch, root/'missing')
            self.assertFalse(report['complete'])
            (sessions/'rollout-child.jsonl').write_text('{}\n')
            report=collect_batch(batch, root/'complete')
            self.assertTrue(report['complete'])
            self.assertTrue((root/'complete/sessions/demo/child.jsonl').is_file())
            self.assertTrue((root/'complete/batch/luna/demo/private/outputs/evaluation.json').is_file())

    def test_all_levels_passes_no_level_filter_and_selects_36_cases(self):
        self.assertEqual(len(select_cases()), 36)
        with tempfile.TemporaryDirectory() as tmp, patch.dict(os.environ, {}, clear=False):
            output = Path(tmp)/'batch'; output.mkdir()
            with patch('sys.argv', ['luna_cpu', 'prepare', '--output', str(output), '--all-levels']), \
                 patch('scripts.luna_cpu.ROOT', Path(tmp)), patch('scripts.luna_cpu.load_env'), \
                 patch('scripts.luna_cpu.subprocess.call', return_value=0) as call:
                self.assertEqual(main(), 0)
            command = call.call_args.args[0]
            self.assertNotIn('--level', command)
            self.assertIn('luna', command)

    def test_default_prepare_selects_all_levels_and_uses_isolated_home(self):
        with tempfile.TemporaryDirectory() as tmp, patch.dict(os.environ, {}, clear=False):
            output = Path(tmp)/'batch'; output.mkdir()
            with patch('sys.argv', ['luna_cpu', 'prepare', '--output', str(output)]), \
                 patch('scripts.luna_cpu.ROOT', Path(tmp)), patch('scripts.luna_cpu.load_env'), \
                 patch('scripts.luna_cpu.subprocess.call', return_value=0) as call:
                self.assertEqual(main(), 0)
            command = call.call_args.args[0]
            self.assertNotIn('--level', command)
            self.assertEqual(command[command.index('--wall-limit-s') + 1], '86400')
            self.assertIn('.codex-luna', json.loads((output/'codex-home.json').read_text())['codex_home'])

    def test_parallel_intervals_are_not_double_counted(self):
        self.assertEqual(interval_union([(0, 4), (2, 6), (7, 9), (8, 10)]), 9)

    def test_profile_requires_openai_api_and_high(self):
        p = profile('luna')
        with tempfile.TemporaryDirectory() as folder:
            launch = launch_config(p, Path(folder))
            self.assertEqual((p['model'], p['effort']), ('gpt-6-luna', 'high'))
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
