import http.server
import os
from pathlib import Path
import shutil
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

from scripts import claude_blocked_precheck as check
from test_claude_precheck_integration import PrecheckModel


@unittest.skipUnless(shutil.which('claude'),'Claude CLI is not installed')
class BlockedPrecheckTests(unittest.TestCase):
    def test_real_cli_against_loopback_model_preserves_blocked_fork(self):
        server=http.server.ThreadingHTTPServer(('127.0.0.1',0),PrecheckModel)
        threading.Thread(target=server.serve_forever,daemon=True).start()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                prof={'model':'claude-opus-5-5','effort':'high','env_key':'FAKE_KEY',
                      'base_url':f'http://127.0.0.1:{server.server_port}'}
                with patch.object(check,'profile',return_value=prof):
                    result=check.run_blocked_precheck('synthetic',Path(tmp)/'out',2,
                                                       {'PATH':os.environ['PATH'],'FAKE_KEY':'synthetic'})
                self.assertTrue(result['passed'],result)
                self.assertTrue(all(result['checks'].values()))
                self.assertLessEqual(result['reported_cost_usd'],2)
        finally:
            server.shutdown();server.server_close()

    def test_budget_must_be_finite_and_sufficient_before_any_call(self):
        with tempfile.TemporaryDirectory() as tmp:
            for budget in (float('nan'),float('inf'),0,.01):
                with self.assertRaises(ValueError):
                    check.run_blocked_precheck('haiku',Path(tmp)/str(budget),budget,{})

    def test_without_spend_confirmation_no_runner_is_called(self):
        with patch.object(sys,'argv',['blocked-precheck','--profile','haiku','--max-budget-usd','1']), \
             patch.object(check,'run_blocked_precheck') as runner:
            self.assertEqual(check.main(),2)
            runner.assert_not_called()

    def test_unexpected_fork_tool_call_is_not_reported_as_pass(self):
        server=http.server.ThreadingHTTPServer(('127.0.0.1',0),PrecheckModel)
        threading.Thread(target=server.serve_forever,daemon=True).start()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                prof={'model':'claude-opus-5-5','effort':'high','env_key':'FAKE_KEY',
                      'base_url':f'http://127.0.0.1:{server.server_port}'}
                real_runner=check.run_turn
                def runner(*args,**kwargs):
                    result=real_runner(*args,**kwargs)
                    if kwargs.get('fork_from'): result['summary']['tool_calls']=1
                    return result
                with patch.object(check,'profile',return_value=prof),patch.object(check,'run_turn',runner):
                    result=check.run_blocked_precheck('synthetic',Path(tmp)/'out',2,
                                                       {'PATH':os.environ['PATH'],'FAKE_KEY':'synthetic'})
                self.assertFalse(result['passed'])
                self.assertFalse(result['checks']['fork_has_no_tools'])
                self.assertTrue(result['cost_complete'])
        finally:
            server.shutdown();server.server_close()


if __name__=='__main__': unittest.main()
