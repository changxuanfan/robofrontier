"""Opt-in real physics evaluation smoke using a local synthetic policy."""
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
from scripts.claude_benchmark import prepare_draft
from scripts.export_run_report import export


class FinishModel(StubModel):
    requests=[]
    def choose_tool(self,names,text,blocks,has_result):
        if names and not has_result and 'wake_id=' in text:
            return {'op':'finish','data':{'note':'synthetic early finish'}},'toolu_finish'
        return super().choose_tool(names,text,blocks,has_result)


@unittest.skipUnless(os.environ.get('ROBOFRONTIER_PHYSICS_FINISH')=='1' and shutil.which('claude'),
                     'Opt-in physics finish smoke requires Claude CLI and scene assets')
class FinishPhysicsSmokeTests(unittest.TestCase):
    def test_early_finish_produces_failed_evaluation_and_report(self):
        directory=Path(tempfile.mkdtemp(prefix='claude-finish-physics-'))
        server=http.server.ThreadingHTTPServer(('127.0.0.1',0),FinishModel)
        threading.Thread(target=server.serve_forever,daemon=True).start()
        result=None
        try:
            batch=directory/'batch'; case='transport-a04-d1'
            prepare_draft(batch,['opus'],cases=[case],topology='centralized')
            run=batch/'opus'/case
            launch=run/'private/inputs/claude.json'; data=json.loads(launch.read_text())
            data['profile'].update(base_url=f'http://127.0.0.1:{server.server_port}',env_key='FAKE_KEY')
            launch.write_text(json.dumps(data))
            manifest=run/'private/inputs/manifest.json'; frozen=json.loads(manifest.read_text())
            frozen['input_sha256']['private/inputs/claude.json']=hashlib.sha256(launch.read_bytes()).hexdigest()
            manifest.write_text(json.dumps(frozen))
            env={'PATH':os.environ['PATH'],'HOME':os.environ['HOME'],'FAKE_KEY':'synthetic'}
            result=run_engineering(run,env,turn_budget_usd=.5,total_budget_usd=2,wall_limit_s=30,poll_s=.1)
            self.assertEqual(result['status'],'finished',result)
            self.assertEqual(result['reason'],'world_ended',result)
            self.assertTrue(result['seal']['sealed'],result)
            self.assertFalse(result['owner_alive'],result)
            evaluation=json.loads((run/'private/outputs/evaluation.json').read_text())
            self.assertFalse(evaluation['task_success'],evaluation)
            report=export(run)
            self.assertEqual(report['lifecycle'],'ended')
            self.assertEqual(report['evaluation'],evaluation)
            self.assertEqual(report['engine'],'claude')
            self.assertFalse(report['benchmark_qualified'])
            self.assertIn('未取得正式 benchmark 资格',(run/'private/outputs/report.md').read_text())
            self.assertFalse(report['all_requested_models_verified'])
            self.assertEqual(report['changed_inputs'],[])
        finally:
            server.shutdown();server.server_close()
            if result and not result.get('owner_alive'): shutil.rmtree(directory)
            else: print('Uncertain finish smoke retained for review:',directory)


if __name__=='__main__': unittest.main()
