"""Opt-in Mosaic reference transfer through real Claude CLI, no provider calls."""
import http.server
import base64
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
import threading
import unittest

from robofrontier.claude.runner import run_turn
from robofrontier.task_config import ROOT
from scripts.claude_benchmark import prepare_draft
from test_claude_adaptive_integration import StubModel


class ReferenceModel(StubModel):
    requests=[]
    def choose_tool(self,names,text,blocks,has_result):
        messages=self.request_body.get('messages',[])
        if names and not any('tool_result' in str(m.get('content','')) for m in messages) and any(
                'MOSAIC_IMAGE_CHECK' in str(m.get('content','')) for m in messages):
            return {'op':'public_image','data':{'names':['target-pattern.png']}},'toolu_reference'
        return None,None


def images(value):
    if isinstance(value,dict):
        if value.get('type')=='image': yield value
        for item in value.values(): yield from images(item)
    elif isinstance(value,list):
        for item in value: yield from images(item)


@unittest.skipUnless(os.environ.get('ROBOFRONTIER_MOSAIC_IMAGE')=='1' and shutil.which('claude'),
                     'Opt-in Mosaic image check requires Claude CLI and R14 assets')
class MosaicReferenceIntegrationTests(unittest.TestCase):
    def test_real_cli_sends_r14_target_to_synthetic_endpoint(self):
        server=http.server.ThreadingHTTPServer(('127.0.0.1',0),ReferenceModel)
        threading.Thread(target=server.serve_forever,daemon=True).start()
        ReferenceModel.requests.clear()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                batch=Path(tmp)/'batch'; case='mosaic-a04-6x12-d1'
                prepare_draft(batch,['opus'],cases=[case])
                run=batch/'opus'/case
                mcp=run/'private/mosaic-check-mcp.json'
                mcp.write_text(json.dumps({'mcpServers':{'robot':{'command':sys.executable,
                    'args':['-m','robofrontier.claude.robot_mcp','--run-dir',str(run),
                            '--connection',str(run/'private/state/connections/Controller.json')],
                    'env':{'PYTHONPATH':str(ROOT)}}}}))
                profile={'model':'claude-opus-5-5','effort':'high','env_key':'FAKE_KEY',
                         'base_url':f'http://127.0.0.1:{server.server_port}'}
                result=run_turn(profile,profile['model'],profile['effort'],mcp,'MOSAIC_IMAGE_CHECK',
                                run,{'PATH':os.environ['PATH'],'FAKE_KEY':'synthetic'},timeout=45,max_budget_usd=.5)
                self.assertEqual(result['summary']['turn_status'],'ended',result)
                self.assertGreaterEqual(result['summary']['tool_calls'],1,
                    {'summary':result['summary'],'tools':[[t.get('name') for t in x.get('tools',[])] for x in ReferenceModel.requests],
                     'user_messages':[str(x.get('messages',[])[-1].get('content',''))[:500] for x in ReferenceModel.requests]})
                expected=hashlib.sha256((run/'public/inputs/target-pattern.png').read_bytes()).hexdigest()
                received=[]
                for request in ReferenceModel.requests:
                    for block in images(request.get('messages',[])):
                        source=block.get('source',{})
                        if source.get('type')=='base64':
                            received.append(hashlib.sha256(base64.b64decode(source['data'])).hexdigest())
                self.assertIn(expected,received,'Mosaic target image bytes did not reach synthetic endpoint')
        finally:
            server.shutdown();server.server_close()


if __name__=='__main__': unittest.main()
