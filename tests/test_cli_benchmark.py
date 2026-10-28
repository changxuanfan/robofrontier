import json
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from scripts.benchmark import profile, launch_config, prepare_batch, load_env, select_cases
from robofrontier.runtime_cli import fork_provenance
from robofrontier.task_families import prepare
from robofrontier.runtime_service import RuntimeService
from test_policy_probe import BudgetWorld
from fake_pipeline import fake_capture
from scripts import benchmark


class CliBenchmarkTests(unittest.TestCase):
    def test_level_and_minimum_arms_select_four_tasks_without_launch(self):
        expected=['transport-a04-d2','mosaic-a04-6x12-d2','relay-a08-d2','dial-best5-n04-d2']
        with patch.object(benchmark.subprocess,'run') as launch:
            rows=select_cases(level=2,min_arms=True)
            self.assertEqual([row['case_id'] for row in rows],expected)
            self.assertEqual([row['arms'] for row in rows],[4,4,8,8])
            self.assertEqual(len(select_cases(level=1)),12)
            self.assertEqual(len(select_cases()),36)
            launch.assert_not_called()
        with self.assertRaisesRegex(ValueError,'Unknown task families'):select_cases(families=['typo'])
        with self.assertRaisesRegex(ValueError,'conflicting'):select_cases(cases=[expected[0]],level=1)

    def test_context_and_native_vision_are_required(self):
        for name in ('astra', 'luna', 'deepseek', 'kimi', 'glm'):
            p=profile(name)
            self.assertEqual((p['context_window'],p['auto_compact_token_limit']),(272000,245000))
        with tempfile.TemporaryDirectory() as tmp:
            file=Path(tmp)/'models.json'
            file.write_text(json.dumps(dict(context_window=400000,auto_compact_token_limit=350000,
                profiles={'small':dict(model='small',provider='openai',vision=True,context_capacity=128000)})))
            with self.assertRaisesRegex(ValueError,'capacity'):profile('small',file)
            file.write_text(json.dumps(dict(context_window=272000,auto_compact_token_limit=245000,
                profiles={'text':dict(model='text',provider='openai',vision=False,context_capacity=272000)})))
            with self.assertRaisesRegex(ValueError,'native images'):profile('text',file)

    def test_custom_provider_is_scoped_and_has_image_metadata(self):
        with tempfile.TemporaryDirectory() as tmp:
            launch=launch_config(profile('deepseek'),Path(tmp))
            self.assertIn('model_provider="deepseek"',launch['args'])
            model=json.loads((Path(tmp)/'models.json').read_text())['models'][0]
            self.assertIn('image',model['input_modalities'])
            self.assertIsNone(model['tool_mode'])
            self.assertFalse(model['use_responses_lite'])
            self.assertNotIn('experimental_bearer_token',json.dumps(launch))

    def test_failed_preflight_cannot_relabel_an_older_pass(self):
        selected=profile('astra')
        with tempfile.TemporaryDirectory() as tmp:
            target=Path(tmp)/'runs/engineering/model-compatibility/astra'
            target.mkdir(parents=True)
            (target/'report.json').write_text(json.dumps({'passed':True,'profile_sha256':'old'}))
            with patch.object(benchmark,'ROOT',Path(tmp)),patch.object(benchmark,'profile',return_value=selected),\
                    patch.object(benchmark,'launch_config',return_value={}),\
                    patch('sys.argv',['benchmark','preflight','--models','astra']),\
                    patch.object(benchmark.subprocess,'run',return_value=SimpleNamespace(returncode=1)):
                with self.assertRaises(SystemExit):benchmark.main()
            self.assertFalse(json.loads((target/'report.json').read_text())['passed'])

    def test_models_share_fresh_private_dial_instance_but_not_runtime(self):
        with tempfile.TemporaryDirectory() as tmp:
            file=prepare_batch(Path(tmp)/'batch',['astra','deepseek'],cases=['dial-best5-n04-d1'])
            batch=json.loads(file.read_text());a,b=[Path(j['run_directory']) for j in batch['jobs']]
            self.assertNotEqual(a,b)
            self.assertEqual((a/'private/inputs/suite-code.json').read_bytes(),(b/'private/inputs/suite-code.json').read_bytes())
            self.assertEqual(batch['model_calls'],0)
            for run in (a,b):
                cfg=json.loads((run/'private/inputs/harness.json').read_text())
                self.assertEqual(cfg['observation_renderer'],'classic')
                self.assertFalse(cfg['record_realtime_video'])
                self.assertFalse(any('suite-code' in str(p) for p in (run/'public').rglob('*')))

    def test_env_is_literal_and_existing_values_win(self):
        with tempfile.TemporaryDirectory() as tmp, patch.dict(os.environ, {'RF_EXISTING':'existing'}, clear=True):
            env=Path(tmp)/'.env'
            env.write_text("RF_EXISTING=other\nRF_LITERAL='$(touch nope)'\nRF_EMPTY=\n")
            load_env(env)
            self.assertEqual(os.environ['RF_EXISTING'], 'existing')
            self.assertEqual(os.environ['RF_LITERAL'], '$(touch nope)')
            self.assertEqual(os.environ['RF_EMPTY'], '')
            env.write_text('INVALID KEY=value\n')
            with self.assertRaisesRegex(ValueError, 'line 1'): load_env(env)

    def test_launch_works_without_local_model_cache(self):
        with tempfile.TemporaryDirectory() as tmp, patch.dict(os.environ, {'CODEX_HOME':tmp}):
            output=Path(tmp)/'launch'; output.mkdir()
            launch=launch_config(profile('luna'), output)
            self.assertEqual(launch['model'],'gpt-6-luna')
            self.assertIn('image',json.loads((output/'models.json').read_text())['models'][0]['input_modalities'])

    def test_changed_cli_or_catalog_rejects_old_preflight_before_start(self):
        import hashlib
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); folder=root/'batch'; run=folder/'astra'/'case'
            (run/'private/inputs').mkdir(parents=True)
            launch={'profile':{'provider':'openai'}}
            (run/'private/inputs/codex.json').write_text(json.dumps(launch))
            (run/'private/inputs/models.json').write_text('{"models": []}')
            (folder/'batch.json').write_text(json.dumps({'status':'prepared_not_started','jobs':[
                {'profile':'astra','run_directory':str(run),'status':'prepared'}]}))
            receipt=root/'runs/engineering/model-compatibility/astra/report.json'
            receipt.parent.mkdir(parents=True)
            profile_hash=hashlib.sha256(json.dumps(launch['profile'],sort_keys=True).encode()).hexdigest()
            receipt.write_text(json.dumps({'passed':True,'profile_sha256':profile_hash,
                                           'catalog_sha256':'old','codex_version':'old'}))
            with patch.object(benchmark,'ROOT',root),patch.object(benchmark,'verify_inputs',return_value=[]),\
                    patch.object(benchmark.subprocess,'check_output',return_value='codex new\n'),\
                    patch.object(benchmark.subprocess,'run') as start:
                with self.assertRaisesRegex(ValueError,'rerun preflight'):
                    benchmark.run_batch(folder,1)
                start.assert_not_called()

    def test_released_policy_service_enforces_24h_without_host(self):
        with tempfile.TemporaryDirectory() as tmp:
            run=prepare('transport-a04-d1',Path(tmp)/'run')
            s=RuntimeService(run,BudgetWorld,fake_capture)
            try:
                s.store.update_agent('Controller',ready=True,thread_id='engineering-only')
                s.store.start();s.store.put('start_at',time.time()-86401)
                s.pump()
                self.assertEqual(s.store.get('termination_reason'),'wall_limit')
                self.assertEqual(s.store.get('lifecycle'),'ended')
                self.assertTrue(s.shutdown)
                self.assertEqual(json.loads((run/'private/outputs/budget-cutoff.json').read_text())['protocol']['wall_limit_s'],86400)
            finally:s.close()

    def test_paginated_fork_lineage_ignores_unrelated_matching_filenames(self):
        parent='00000000-0000-0000-0000-000000000001';child='00000000-0000-0000-0000-000000000002'
        def line(ordinal,typ,payload):return dict(ordinal=ordinal,type=typ,payload=payload)
        with tempfile.TemporaryDirectory() as tmp,patch.dict(os.environ,{'CODEX_HOME':tmp}):
            folder=Path(tmp)/'sessions';folder.mkdir()
            a=line(0,'session_meta',{'id':parent});b=line(1,'response_item',{'role':'user'})
            (folder/f'rollout-{parent}_a.jsonl').write_text(json.dumps(a)+'\n'+json.dumps(b)+'\n')
            (folder/f'rollout-{parent}_b.jsonl').write_text(json.dumps(line(2,'response_item',{'role':'assistant'}))+'\n')
            # Each paginated segment carries its own session metadata in practice.
            (folder/f'rollout-{parent}_b.jsonl').write_text(json.dumps(line(2,'session_meta',{'id':parent}))+'\n')
            (folder/f'rollout-{child}.jsonl').write_text(json.dumps(line(3,'session_meta',{
                'id':child,'forked_from_id':parent,'forked_from_ordinal_exclusive':3}))+'\n')
            (folder/f'rollout-{parent}_other.jsonl').write_text(json.dumps(line(0,'session_meta',{'id':child}))+'\n')
            p=fork_provenance(parent,child)
            self.assertEqual(p['source_ordinal'],3)
            self.assertEqual(p['source_thread_id'],parent)


if __name__=='__main__':unittest.main()
