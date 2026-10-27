import json
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from scripts.benchmark import profile, launch_config, prepare_batch
from robofrontier.runtime_cli import fork_provenance
from robofrontier.task_families import prepare
from robofrontier.runtime_service import RuntimeService
from test_policy_probe import BudgetWorld
from fake_pipeline import fake_capture
from scripts import benchmark


class CliBenchmarkTests(unittest.TestCase):
    def test_context_and_native_vision_are_required(self):
        for name in ('gpt', 'deepseek', 'kimi', 'glm'):
            p=profile(name)
            self.assertEqual((p['context_window'],p['auto_compact_token_limit']),(400000,350000))
        for name in ('deepseek-pro-text','glm-pro-text'):
            with self.assertRaisesRegex(ValueError,'native images'):profile(name)
        with tempfile.TemporaryDirectory() as tmp:
            file=Path(tmp)/'models.json'
            file.write_text(json.dumps(dict(context_window=400000,auto_compact_token_limit=350000,
                profiles={'small':dict(model='small',provider='openai',vision=True,context_capacity=128000)})))
            with self.assertRaisesRegex(ValueError,'capacity'):profile('small',file)

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
        selected=profile('gpt')
        with tempfile.TemporaryDirectory() as tmp:
            target=Path(tmp)/'runs/engineering/model-compatibility/gpt'
            target.mkdir(parents=True)
            (target/'report.json').write_text(json.dumps({'passed':True,'profile_sha256':'old'}))
            with patch.object(benchmark,'ROOT',Path(tmp)),patch.object(benchmark,'profile',return_value=selected),\
                    patch.object(benchmark,'launch_config',return_value={}),\
                    patch('sys.argv',['benchmark','preflight','--models','gpt']),\
                    patch.object(benchmark.subprocess,'run',return_value=SimpleNamespace(returncode=1)):
                with self.assertRaises(SystemExit):benchmark.main()
            self.assertFalse(json.loads((target/'report.json').read_text())['passed'])

    def test_models_share_fresh_private_dial_instance_but_not_runtime(self):
        with tempfile.TemporaryDirectory() as tmp:
            file=prepare_batch(Path(tmp)/'batch',['gpt','deepseek'],cases=['dial-best5-n04-d1'])
            batch=json.loads(file.read_text());a,b=[Path(j['run_directory']) for j in batch['jobs']]
            self.assertNotEqual(a,b)
            self.assertEqual((a/'private/inputs/suite-code.json').read_bytes(),(b/'private/inputs/suite-code.json').read_bytes())
            self.assertEqual(batch['model_calls'],0)
            for run in (a,b):
                cfg=json.loads((run/'private/inputs/harness.json').read_text())
                self.assertEqual(cfg['observation_renderer'],'classic')
                self.assertFalse(cfg['record_realtime_video'])
                self.assertFalse(any('suite-code' in str(p) for p in (run/'public').rglob('*')))

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
