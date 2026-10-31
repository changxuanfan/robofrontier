import json
from pathlib import Path
import tempfile
import threading
import time
import unittest

from robofrontier.claude.fork_setup import create_fork
from robofrontier.delegation import Delegations
from robofrontier.runtime_store import RuntimeStore

PARENT='11111111-1111-4111-8111-111111111111'


class Parent:
    session_id=PARENT; binary='claude'; profile={'model':'fake','effort':'high'}; base_env={}
    def __init__(self,run): self.run=run; self.state='active'
    def observe(self): return {'status':self.state}


class ForkSetupTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.run=Path(self.tmp.name)/'run'
        self.run.mkdir(); self.store=RuntimeStore(self.run/'private/state/runtime.sqlite')
        cfg=json.loads((Path(__file__).resolve().parents[1]/'configs/harnesses/policy.json').read_text())
        cfg.update(engine='claude',topology='adaptive',model='fake',effort='high')
        self.store.initialize('run',cfg,['A','B']); self.store.update_agent('Controller',thread_id=PARENT,ready=True)
        self.store.start(); self.store.begin_call('batch','Controller','delegate',{'scopes':[]})
        self.d=Delegations(self.store)
        batch=self.d.create('Controller','batch',getattr(self,'test_scopes',[{'robots':['A'],'goal':'g','done_when':'d'}]))
        self.actor=batch['children'][0]['actor']; self.d.claim('batch','owner')
        self.parent=Parent(self.run); self.folder=self.run/'private/claude-config/projects/-run'
        self.folder.mkdir(parents=True)
        self.row={'type':'assistant','uuid':'m1','sessionId':PARENT,'message':{'role':'assistant','content':[
            {'type':'tool_use','name':'mcp__robot__robot','input':{'op':'delegate','call_id':'batch'}}]}}
        (self.folder/(PARENT+'.jsonl')).write_text(json.dumps(self.row)+'\n')
        self.calls=[]
    def tearDown(self): self.store.close(); self.tmp.cleanup()
    def runner(self,*args,**kw):
        self.calls.append(kw); child=kw['session_id']
        source=(self.folder/(kw['fork_from']+'.jsonl')).read_text().splitlines()
        (self.folder/(child+'.jsonl')).write_text(''.join(json.dumps({**json.loads(row),'sessionId':child})+'\n' for row in source))
        return {'process':'exited','exit_code':0,'summary':{'session_id':child,'turn_status':'ended',
                'malformed_lines':0,'tool_calls':0,'total_cost_usd':.01}}
    def create(self,runner=None):
        return create_fork(self.store,self.parent,'batch',self.actor,'owner',max_budget_usd=.1,runner=runner or self.runner)
    def test_records_identity_before_tool_free_creation_and_grants_no_permission(self):
        def checked(*args,**kwargs):
            claim=json.loads((self.run/'private/claude-forks'/(self.actor+'.json')).read_text())
            self.assertEqual(claim['status'],'claimed'); self.assertEqual(claim['child_session'],kwargs['session_id'])
            self.assertFalse(kwargs['enable_robot']); self.assertEqual(kwargs['fork_from'],PARENT)
            return self.runner(*args,**kwargs)
        output=self.create(checked)
        self.assertEqual(output['status'],'history_verified_no_authority')
        self.assertEqual(output['reported_cost_usd'],'0.01')
        self.assertEqual(self.d.batch('batch')['children'][0]['thread_id'],None)
        self.assertEqual(self.store.agent(self.actor)['delegation_state'],'creating')
        with self.assertRaises(FileExistsError): self.create()
        self.assertEqual(len(self.calls),1)
    def test_uncertain_fork_is_saved_and_never_repeated(self):
        def lost(*args,**kwargs): raise OSError('lost acknowledgement')
        with self.assertRaises(OSError): self.create(lost)
        claim=json.loads((self.run/'private/claude-forks'/(self.actor+'.json')).read_text())
        self.assertEqual(claim['status'],'uncertain'); self.assertEqual(claim['error_type'],'OSError')
        with self.assertRaises(FileExistsError): self.create()
        self.assertEqual(self.calls,[])
    def test_changed_parent_history_fails_closed(self):
        def changed(*args,**kwargs):
            response=self.runner(*args,**kwargs)
            with (self.folder/(PARENT+'.jsonl')).open('a') as f:
                f.write(json.dumps({**self.row,'uuid':'m2'})+'\n')
            return response
        with self.assertRaises(ValueError): self.create(changed)
        self.assertEqual(json.loads((self.run/'private/claude-forks'/(self.actor+'.json')).read_text())['status'],'uncertain')
    def test_missing_cost_never_authorizes_or_retries(self):
        def no_cost(*args,**kwargs):
            response=self.runner(*args,**kwargs)
            response['summary']['total_cost_usd']=None
            return response
        with self.assertRaisesRegex(ValueError,'cost is unknown'): self.create(no_cost)
        self.assertEqual(json.loads((self.run/'private/claude-forks'/(self.actor+'.json')).read_text())['status'],'uncertain')
        with self.assertRaises(FileExistsError): self.create()
    def test_reported_cost_above_setup_allowance_is_uncertain(self):
        def expensive(*args,**kwargs):
            response=self.runner(*args,**kwargs)
            response['summary']['total_cost_usd']=.11
            return response
        with self.assertRaisesRegex(ValueError,'exceeds its allowance'): self.create(expensive)
        self.assertEqual(json.loads((self.run/'private/claude-forks'/(self.actor+'.json')).read_text())['status'],'uncertain')
    def test_requires_pending_parent_delegation(self):
        self.parent.state='idle'
        with self.assertRaisesRegex(ValueError,'active tool turn'): self.create()
        self.assertFalse((self.run/'private/claude-forks'/(self.actor+'.json')).exists())
    def test_waits_for_pending_tool_call_to_be_flushed_before_fork(self):
        transcript=self.folder/(PARENT+'.jsonl')
        transcript.write_text('')
        def flush():
            time.sleep(.1)
            transcript.write_text(json.dumps(self.row)+'\n')
        worker=threading.Thread(target=flush); worker.start()
        try: self.assertEqual(self.create()['status'],'history_verified_no_authority')
        finally: worker.join()
        self.assertEqual(len(self.calls),1)
    def test_fork_shares_parent_budget_reservations(self):
        from robofrontier.claude.budget import BudgetLedger
        ledger=BudgetLedger.create(self.run/'private/state/budget.sqlite',1)
        try:
            ledger.reserve('parent-turn','Controller',.95)
            result=create_fork(self.store,self.parent,'batch',self.actor,'owner',max_budget_usd=.1,
                               runner=self.runner,budget_ledger=ledger)
            self.assertEqual(self.calls[0]['max_budget_usd'],.05)
            self.assertEqual(result['status'],'history_verified_no_authority')
            self.assertEqual(ledger.snapshot()['reported_spend_usd'],'0.01')
            self.assertEqual(ledger.snapshot()['reserved_usd'],'0.95')
        finally: ledger.close()

    def test_cancelled_before_fork_launch_releases_only_fork_allowance(self):
        from robofrontier.claude.budget import BudgetLedger
        from robofrontier.claude.runner import PreLaunchCancelled
        ledger=BudgetLedger.create(self.run/'private/state/budget.sqlite',1)
        def cancelled(*args, **kwargs): raise PreLaunchCancelled('no process created')
        try:
            ledger.reserve('parent-turn','Controller',.4)
            with self.assertRaises(PreLaunchCancelled):
                create_fork(self.store,self.parent,'batch',self.actor,'owner',max_budget_usd=.1,
                            runner=cancelled,budget_ledger=ledger)
            snapshot=ledger.snapshot()
            self.assertEqual(snapshot['status'],'tracking')
            self.assertEqual(snapshot['reserved_usd'],'0.4')
            self.assertEqual(sorted(row['status'] for row in snapshot['reservations']),
                             ['not_launched','reserved'])
            claim=json.loads((self.run/'private/claude-forks'/(self.actor+'.json')).read_text())
            self.assertEqual(claim['status'],'uncertain')
        finally: ledger.close()

    def test_adoption_requires_activation_exact_identity_and_single_use(self):
        from robofrontier.claude.budget import BudgetLedger
        from robofrontier.claude.session_host import ClaudeSessionHost
        roles=self.run/'public/inputs/roles.json'; roles.parent.mkdir(parents=True)
        roles.write_text(json.dumps({'roles':{'Controller':{'kind':'controller'}}}))
        conn=self.run/'private/state/connections'/(self.actor+'.json'); conn.parent.mkdir(parents=True)
        conn.write_text(json.dumps({'schema_version':3,'run_id':'run','actor':self.actor,'token':'fake'}))
        ledger=BudgetLedger.create(self.run/'private/state/budget.sqlite',1)
        profile={'model':'fake','effort':'high','env_key':'FAKE'}
        launch=self.run/'private/inputs/claude.json'; launch.parent.mkdir(parents=True,exist_ok=True)
        launch.write_text(json.dumps({'profile':profile}))
        try:
            result=create_fork(self.store,self.parent,'batch',self.actor,'owner',max_budget_usd=.1,
                               runner=self.runner,budget_ledger=ledger)
            with self.assertRaises(ValueError):
                ClaudeSessionHost(self.run,self.actor,profile,{})
            resumed=[]
            def child_runner(*args, **kwargs):
                resumed.append(kwargs)
                return {'process':'exited','exit_code':0,'summary':{'session_id':kwargs['resume'],
                        'turn_status':'ended','malformed_lines':0,'total_cost_usd':.02}}
            adopt=lambda: ClaudeSessionHost.adopt_verified_fork(self.run,self.actor,profile,{},
                                                               budget_ledger=ledger,max_budget_usd=.2,runner=child_runner,
                                                               _verifier=lambda _: [])
            with self.assertRaisesRegex(ValueError,'activation mismatch'): adopt()
            self.d.bind('batch',self.actor,result['child_session'],result['proof'],'owner')
            self.d.activate('batch','owner')
            self.assertEqual(self.store.action_robot(self.actor,'A'),'A')
            with self.assertRaises(ValueError): self.store.action_robot('Controller','A')
            host=adopt()
            self.assertEqual(host.session_id,result['child_session'])
            self.assertEqual(host.observe()['status'],'idle')
            cfg=json.loads(host.mcp.read_text())
            args=cfg['mcpServers']['robot']['args']
            self.assertEqual(args[args.index('--connection')+1],str(conn.resolve()))
            host.send('Assigned scope'); self.assertEqual(host.wait(3)['status'],'idle')
            self.assertEqual(resumed[0]['resume'],result['child_session'])
            self.assertEqual(ledger.snapshot()['reported_spend_usd'],'0.03')
            with self.assertRaises(FileExistsError): adopt()
        finally: ledger.close()

    def test_mismatched_claim_cannot_adopt_active_child(self):
        from robofrontier.claude.budget import BudgetLedger
        from robofrontier.claude.session_host import ClaudeSessionHost
        ledger=BudgetLedger.create(self.run/'private/state/budget.sqlite',1)
        profile={'model':'fake','effort':'high','env_key':'FAKE'}
        launch=self.run/'private/inputs/claude.json'; launch.parent.mkdir(parents=True,exist_ok=True)
        launch.write_text(json.dumps({'profile':profile}))
        try:
            result=create_fork(self.store,self.parent,'batch',self.actor,'owner',max_budget_usd=.1,
                               runner=self.runner,budget_ledger=ledger)
            self.d.bind('batch',self.actor,result['child_session'],result['proof'],'owner')
            self.d.activate('batch','owner')
            claim=self.run/'private/claude-forks'/(self.actor+'.json')
            data=json.loads(claim.read_text()); data['child_session']='different'; claim.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError,'identity or activation mismatch'):
                ClaudeSessionHost.adopt_verified_fork(self.run,self.actor,profile,{},budget_ledger=ledger,max_budget_usd=.2,
                                                      _verifier=lambda _: [])
            self.assertFalse((self.run/'private/claude-forks'/(self.actor+'.adopted')).exists())
        finally: ledger.close()


if __name__=='__main__': unittest.main()
