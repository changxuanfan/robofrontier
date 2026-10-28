"""Experimental adaptive policy host for an already-owned physical world.

Not exposed as a benchmark launcher. The caller creates the world and a shared
budget ledger, explicitly starts supervision, and seals the world afterwards.
"""
import json
from pathlib import Path
import time
import uuid

from robofrontier.claude.adaptive_dispatch import AdaptiveDispatcher
from robofrontier.claude.adaptive_setup import AdaptiveForkSetup
from robofrontier.claude.bootstrap import bootstrap_prompt
from robofrontier.claude.session_host import ClaudeSessionHost
from robofrontier.pipeline_client import client_call
from robofrontier.run_setup import verify_inputs
from robofrontier.runtime_store import RuntimeStore


class AdaptivePolicyHost:
    def __init__(self, run_dir, profile, base_env, ledger, *, turn_budget_usd, setup_budget_usd,
                 host_factory=ClaudeSessionHost, setup_factory=AdaptiveForkSetup,
                 client=client_call, verifier=verify_inputs):
        self.run=Path(run_dir).resolve()
        if not Path(ledger.path).resolve().is_relative_to(self.run/'private'):
            raise ValueError('A run-local shared budget ledger is required')
        self.store=RuntimeStore(self.run/'private/state/runtime.sqlite')
        try:
            cfg=self.store.get('config')
            if not cfg or cfg.get('engine')!='claude' or cfg.get('topology')!='adaptive':
                raise ValueError('Expected adaptive Claude engineering run')
            launch=json.loads((self.run/'private/inputs/claude.json').read_text())
            if cfg.get('model')!=profile.get('model') or cfg.get('effort')!=profile.get('effort') or launch.get('profile')!=profile:
                raise ValueError('Profile differs from frozen run')
            if self.store.get('lifecycle')!='prepared' or self.store.agent('Controller')['thread_id']:
                raise ValueError('A previously bound or started world cannot be restarted')
            changed=verifier(self.run)
            if changed: raise ValueError('Frozen inputs changed: '+', '.join(changed))
            self.prompt=bootstrap_prompt(self.run,topology='adaptive')
            self.host=host_factory(self.run,'Controller',profile,base_env,
                                   max_budget_usd=turn_budget_usd,budget_ledger=ledger)
            self.ledger=ledger; self.setup_factory=setup_factory; self.client=client
            self.setup_budget_usd=setup_budget_usd; self.turn_budget_usd=turn_budget_usd
            self.bound=False; self.bind_attempted=False; self.setup=None; self.dispatcher=None
        except Exception:
            self.store.close(); raise

    def bind_and_bootstrap(self):
        if self.bind_attempted or self.store.agent('Controller')['thread_id']:
            raise ValueError('Never repeat a role binding')
        self.bind_attempted=True
        conn=self.run/'private/state/connections/Meta.json'
        reply=self.client(conn,'bind',{'actor':'Controller','thread_id':self.host.session_id},
                          'bind-'+uuid.uuid4().hex,log_call_id=False)
        if not reply.get('bound') or self.store.agent('Controller')['thread_id']!=self.host.session_id:
            raise ValueError('Role binding outcome unknown; review instead of retrying')
        self.bound=True
        self.setup=self.setup_factory(self.store,self.host,self.ledger,'claude-meta-'+uuid.uuid4().hex,
                                      setup_budget_usd=self.setup_budget_usd,child_budget_usd=self.turn_budget_usd)
        self.dispatcher=AdaptiveDispatcher(self.store,self.setup)
        return self.host.send(self.prompt)

    def start_when_ready(self):
        if not self.bound or not self.dispatcher: raise ValueError('Controller has not been bound')
        if not self.dispatcher.observe('Controller') or self.host.observe()['status']!='idle' or not self.store.agent('Controller')['ready']:
            raise ValueError('Controller must be READY and its startup turn idle')
        reply=self.dispatcher.admin('request-start')
        if not reply.get('started'): raise ValueError('World did not confirm START')
        return reply

    def cycle(self):
        if not self.dispatcher: raise ValueError('Controller has not been bound')
        return self.dispatcher.cycle()

    def stop(self):
        """Request all role turns stop before the caller seals the world."""
        errors=[]
        if self.setup:
            try: self.setup.request_stop()
            except Exception as error: errors.append(type(error).__name__)
        hosts=[self.host]+list(self.setup.hosts.values() if self.setup else [])
        for host in hosts:
            try: host.stop()
            except Exception as error: errors.append(type(error).__name__)
        deadline=time.monotonic()+5
        if self.dispatcher and not self.dispatcher.wait_setup(timeout=max(0,deadline-time.monotonic())):
            errors.append('ForkSetupStillActive')
        for host in hosts:
            if hasattr(host,'wait'):
                try: host.wait(timeout=max(0,deadline-time.monotonic()))
                except Exception as error: errors.append(type(error).__name__)
        if errors or any(host.observe()['status']=='active' for host in hosts):
            raise RuntimeError('At least one role did not stop; review before closure')
        return {'stop_requested':True,'roles':len(hosts)}

    def close(self):
        if self.dispatcher and not self.dispatcher.wait_setup(timeout=0):
            raise ValueError('Wait for fork setup before closing')
        hosts=[self.host]+list(self.setup.hosts.values() if self.setup else [])
        if any(host.observe()['status']=='active' for host in hosts):
            raise ValueError('Stop and wait for every role before closing')
        self.store.close()
