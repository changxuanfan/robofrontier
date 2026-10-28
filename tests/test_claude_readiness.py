import http.server
import json
import os
from pathlib import Path
import tempfile
import shutil
import threading
import unittest
import uuid
from unittest.mock import patch

from robofrontier.claude.profile_identity import fingerprint
from scripts.claude_readiness import BASIC, BLOCKED, check_receipts
from scripts import claude_precheck, claude_blocked_precheck
from test_claude_precheck_integration import PrecheckModel


class ReadinessTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.root=Path(self.tmp.name)
        self.profile={'model':'test-model','effort':'high','env_key':'FAKE_KEY','base_url':'https://example.invalid'}
        self.version='test-cli 1'; parent=str(uuid.uuid4()); child=str(uuid.uuid4())
        identity={'profile':'test','requested_model':'test-model','profile_sha256':fingerprint(self.profile),
                  'claude_version':self.version,'budget_usd':1,'passed':True}
        self.basic={**identity,'schema':'robofrontier.claude-precheck.v1','fork_checked':True,'total_cost_usd':.04,
                    'checks':[{'name':name,'passed':True,'checks':{k:True for k in facts},'cost_usd':.01,
                               'session_id':child if name=='fork' else parent} for name,facts in BASIC.items()]}
        self.blocked={**identity,'schema':'robofrontier.claude-blocked-precheck.v1','cost_complete':True,
                      'reported_cost_usd':.03,'checks':{k:True for k in BLOCKED},'error_type':None,
                      'parent_session':parent,'child_session':child}
    def tearDown(self): self.tmp.cleanup()
    def check(self):
        basic=self.root/'basic.json';blocked=self.root/'blocked.json'
        basic.write_text(json.dumps(self.basic));blocked.write_text(json.dumps(self.blocked))
        return check_receipts('test',basic,blocked,selected=self.profile,version=self.version)
    def test_matching_receipts_are_consistent_but_not_formally_qualified(self):
        result=self.check()
        self.assertTrue(result['precheck_evidence_matches'],result)
        self.assertFalse(result['formal_execution_available'])
        self.assertEqual(result['model_calls'],0)
    def test_configuration_cli_missing_check_and_budget_fail_closed(self):
        for target,key,value in ((self.basic,'profile_sha256','changed'),(self.basic,'claude_version','changed'),
                                 (self.basic,'total_cost_usd',.03),(self.blocked,'cost_complete',False),
                                 (self.blocked,'child_session',self.blocked['parent_session'])):
            old=target[key];target[key]=value
            self.assertFalse(self.check()['precheck_evidence_matches'],(key,value))
            target[key]=old
        del self.basic['checks'][1]['checks']['image_color_named']
        self.assertFalse(self.check()['precheck_evidence_matches'])
    def test_invalid_receipt_files_do_not_count(self):
        result=check_receipts('test',self.root/'missing',self.root/'also-missing',selected=self.profile,version=self.version)
        self.assertFalse(result['precheck_evidence_matches'])

    @unittest.skipUnless(shutil.which('claude'),'Claude CLI is not installed')
    def test_real_cli_synthetic_provider_receipts_match_current_profile(self):
        server=http.server.ThreadingHTTPServer(('127.0.0.1',0),PrecheckModel)
        threading.Thread(target=server.serve_forever,daemon=True).start()
        try:
            selected={'model':'claude-opus-5-5','effort':'high','env_key':'FAKE_KEY',
                      'base_url':f'http://127.0.0.1:{server.server_port}'}
            env={'PATH':os.environ['PATH'],'FAKE_KEY':'synthetic'}
            with patch.object(claude_precheck,'profile',return_value=selected), \
                 patch.object(claude_blocked_precheck,'profile',return_value=selected):
                basic=claude_precheck.run_precheck('synthetic',self.root/'basic',2,env,check_fork=True)
                blocked=claude_blocked_precheck.run_blocked_precheck('synthetic',self.root/'blocked',2,env)
            self.assertTrue(basic['passed'],basic)
            self.assertTrue(blocked['passed'],blocked)
            result=check_receipts('synthetic',self.root/'basic/receipt.json',self.root/'blocked/receipt.json',
                                  selected=selected,version=claude_precheck.claude_version())
            self.assertTrue(result['precheck_evidence_matches'],result)
        finally:
            server.shutdown();server.server_close()


if __name__=='__main__':unittest.main()
