"""Claude subscription candidate tested only with a fake token and loopback provider."""
import http.server
import json
import os
from pathlib import Path
import shutil
import tempfile
import threading
import unittest
import uuid
from unittest.mock import patch

from robofrontier.claude.runner import build_command, build_env, run_turn
from scripts import claude_precheck, claude_blocked_precheck
from scripts.claude_benchmark import profile as load_profile
from test_claude_precheck_integration import PrecheckModel


class OAuthModel(PrecheckModel):
    auth_schemes=[]
    def choose_tool(self,names,text,blocks,has_result):
        # Non-bare CLI appends an extra budget message after the user prompt.
        for message in reversed(self.request_body.get('messages',[])):
            if message.get('role')!='user': continue
            content=message.get('content',[])
            content=[{'type':'text','text':content}] if isinstance(content,str) else content
            if not isinstance(content,list): continue
            if any(isinstance(x,dict) and x.get('type')=='tool_result' for x in content): return None,None
            if any(isinstance(x,dict) and 'Call the robot tool once' in x.get('text','') for x in content):
                return ({'op':'observe','data':{},'views':['top']},'toolu_precheck_image') if names else (None,None)
        return None,None
    def do_POST(self):
        self.auth_schemes.append(self.headers.get('Authorization','').split(' ',1)[0])
        super().do_POST()


class SubscriptionTests(unittest.TestCase):
    def test_subscription_environment_never_passes_other_credentials(self):
        profile={'provider':'anthropic','env_key':'CLAUDE_CODE_OAUTH_TOKEN','auth_mode':'oauth_token'}
        env=build_env(profile,{'CLAUDE_CODE_OAUTH_TOKEN':'synthetic','ANTHROPIC_API_KEY':'other'},'/tmp/test')
        self.assertEqual(env['CLAUDE_CODE_OAUTH_TOKEN'],'synthetic')
        self.assertNotIn('ANTHROPIC_API_KEY',env)
        cmd=build_command('model','high',None,'prompt',enable_robot=False,auth_mode='oauth_token')
        self.assertNotIn('--bare',cmd)
        self.assertEqual(cmd[cmd.index('--setting-sources')+1],'')
        with self.assertRaises(ValueError): build_env({**profile,'provider':'third-party'}, {'CLAUDE_CODE_OAUTH_TOKEN':'x'},'/tmp/test')
        with self.assertRaises(ValueError): build_env({**profile,'base_url':'https://other.example'}, {'CLAUDE_CODE_OAUTH_TOKEN':'x'},'/tmp/test')
        with tempfile.TemporaryDirectory() as tmp:
            catalog=Path(tmp)/'models.json'
            candidate={**profile,'model':'example','effort':'high','base_url':'https://other.example'}
            catalog.write_text(json.dumps({'schema':'robofrontier.claude-models.v1','profiles':{'candidate':candidate}}))
            with self.assertRaisesRegex(ValueError, 'Anthropic endpoint'):
                load_profile('candidate',catalog)

    @unittest.skipUnless(shutil.which('claude'),'Claude CLI is not installed')
    def test_fake_token_with_local_cli_and_suppressed_project_hook(self):
        server=http.server.ThreadingHTTPServer(('127.0.0.1',0),OAuthModel)
        threading.Thread(target=server.serve_forever,daemon=True).start()
        OAuthModel.auth_schemes.clear()
        OAuthModel.requests.clear()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                run=Path(tmp)/'run'; (run/'.claude').mkdir(parents=True)
                marker=run/'hook-ran'
                settings={'hooks':{'SessionStart':[{'hooks':[{'type':'command','command':'touch '+str(marker)}]}]}}
                (run/'.claude/settings.json').write_text(json.dumps(settings))
                (run/'CLAUDE.md').write_text('UNTRUSTED_AUTODISCOVERY_SENTINEL')
                profile={'provider':'anthropic','model':'claude-opus-5-5','effort':'high',
                         'env_key':'CLAUDE_CODE_OAUTH_TOKEN','auth_mode':'oauth_token',
                         'base_url':f'http://127.0.0.1:{server.server_port}'}
                env={'PATH':os.environ['PATH'],'CLAUDE_CODE_OAUTH_TOKEN':'synthetic-token'}
                session=str(uuid.uuid4())
                with patch('robofrontier.claude.runner._oauth_endpoint_allowed',return_value=True):
                    record=run_turn(profile,profile['model'],profile['effort'],None,'Reply READY',run,env,
                                    session_id=session,timeout=30,max_budget_usd=.5,enable_robot=False)
                self.assertEqual(record['summary']['turn_status'],'ended',record)
                self.assertEqual(record['summary']['session_id'],session)
                self.assertFalse(marker.exists(),'Untrusted project hook was executed')
                self.assertIn('Bearer',OAuthModel.auth_schemes)
                self.assertNotIn('UNTRUSTED_AUTODISCOVERY_SENTINEL',json.dumps(OAuthModel.requests))
        finally:
            server.shutdown();server.server_close()

    @unittest.skipUnless(shutil.which('claude'),'Claude CLI is not installed')
    def test_fake_subscription_runs_both_prechecks_over_loopback(self):
        OAuthModel.requests.clear()
        server=http.server.ThreadingHTTPServer(('127.0.0.1',0),OAuthModel)
        threading.Thread(target=server.serve_forever,daemon=True).start()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                profile={'provider':'anthropic','model':'claude-opus-5-5','effort':'high',
                         'env_key':'CLAUDE_CODE_OAUTH_TOKEN','auth_mode':'oauth_token',
                         'base_url':f'http://127.0.0.1:{server.server_port}'}
                env={'PATH':os.environ['PATH'],'CLAUDE_CODE_OAUTH_TOKEN':'synthetic-token'}
                with patch.object(claude_precheck,'profile',return_value=profile), \
                     patch.object(claude_blocked_precheck,'profile',return_value=profile), \
                     patch('robofrontier.claude.runner._oauth_endpoint_allowed',return_value=True):
                    basic=claude_precheck.run_precheck('synthetic',Path(tmp)/'basic',2,env,check_fork=True)
                    blocked=claude_blocked_precheck.run_blocked_precheck('synthetic',Path(tmp)/'blocked',2,env)
                self.assertTrue(basic['passed'],{'basic':basic,'requests':[[t.get('name') for t in x.get('tools',[])] for x in OAuthModel.requests]})
                self.assertTrue(blocked['passed'],blocked)
                self.assertTrue(OAuthModel.requests)
                for request in OAuthModel.requests:
                    self.assertLessEqual({tool['name'] for tool in request.get('tools',[])}, {'mcp__robot__robot'})
        finally:
            server.shutdown();server.server_close()


if __name__=='__main__':unittest.main()
