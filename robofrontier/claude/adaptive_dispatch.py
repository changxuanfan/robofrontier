"""Experimental deterministic dispatch for an existing adaptive Claude world."""
import time
import threading

from robofrontier.delegation import Delegations
from robofrontier.runtime_cli import execute


class AdaptiveDispatcher:
    def __init__(self, store, setup):
        cfg=store.get('config')
        if cfg.get('engine') != 'claude' or cfg.get('topology') != 'adaptive':
            raise ValueError('Expected adaptive Claude runtime')
        if setup.store is not store or store.agent('Controller')['thread_id'] != setup.parent.session_id:
            raise ValueError('Setup does not own this runtime and parent')
        self.store=store; self.setup=setup; self.owner=setup.owner
        self.setup_thread=None; self.setup_result=None

    def _establish(self, batch_id):
        try:
            self.setup_result=self.setup.establish(batch_id)
        except Exception as error:
            self.setup_result={'status':'needs_review','error_type':type(error).__name__}
            self.store.put('dispatch_paused',True)

    def wait_setup(self, timeout=None):
        if self.setup_thread: self.setup_thread.join(timeout)
        return not self.setup_thread or not self.setup_thread.is_alive()

    def admin(self, op, data=None):
        return execute(self.store,op,data or {},self.owner,time.time())

    def pause(self, reason):
        self.admin('pause',{'reason':reason})
        return False

    def host(self, actor):
        return self.setup.parent if actor == 'Controller' else self.setup.hosts.get(actor)

    def observe(self, actor):
        agent=self.store.agent(actor); host=self.host(actor)
        if host is None or getattr(host,'actor',None) != actor or getattr(host,'session_id',None) != agent['thread_id']:
            return self.pause('Missing or mismatched Claude role host')
        try: value=host.observe()
        except Exception: value=None
        if (not isinstance(value,dict) or value.get('thread_id') != agent['thread_id']
                or value.get('status') not in ('idle','active')
                or not isinstance(value.get('turn_id'),str) or not value['turn_id']):
            self.admin('session-unavailable',{'actor':actor,'thread_id':agent['thread_id'],
                                               'reason':'Claude role state unavailable or failed'})
            return self.pause('Claude role state requires review')
        self.admin('observe-sessions',{'observations':[{'actor':actor,'thread_id':value['thread_id'],
            'status':value['status'],'turn_id':value['turn_id'],'observed_at':time.time(),
            'source':'claude.host','turn_status':None}]})
        return True

    def observe_team(self):
        for actor,agent in self.store.agents().items():
            if agent['kind']=='manager' or not agent['thread_id']: continue
            if agent['kind']=='delegate' and agent.get('delegation_state') != 'active': continue
            if not self.observe(actor): return False
        return True

    def cycle(self):
        snapshot=self.admin('snapshot')
        if snapshot['paused'] or snapshot['lifecycle'] in ('ended','interrupted','recovery_required'):
            return {'dispatched':[], 'paused':snapshot['paused'], 'lifecycle':snapshot['lifecycle']}
        if self.setup_thread:
            if self.setup_thread.is_alive():
                # During activation hosts are installed one by one. Do not
                # observe a half-installed team or run competing dispatch.
                return {'dispatched':[], 'paused':False, 'setup':'pending'}
            self.setup_thread.join(); self.setup_thread=None
            if not self.setup_result or self.setup_result.get('status')!='active':
                self.pause('Claude fork setup requires review')
                return {'dispatched':[], 'paused':True}
            self.setup_result=None
        if not self.observe_team(): return {'dispatched':[], 'paused':True}
        if snapshot['lifecycle'] != 'running': return {'dispatched':[], 'paused':False}
        for batch in Delegations(self.store).active():
            if batch['status'] != 'creating': continue
            if batch['claimed_by'] is not None:
                Delegations(self.store).uncertain(batch['id'],'Interrupted Claude fork setup; never automatically refork',batch['claimed_by'])
                self.pause('Claude fork setup already claimed')
                return {'dispatched':[], 'paused':True}
            self.setup_result=None
            self.setup_thread=threading.Thread(target=self._establish,args=(batch['id'],),daemon=True)
            try: self.setup_thread.start()
            except Exception:
                self.setup_thread=None
                self.pause('Could not launch Claude fork setup')
                return {'dispatched':[], 'paused':True}
            return {'dispatched':[], 'paused':False, 'setup':'pending'}
        sent=[]
        for command in self.admin('plan')['commands']:
            actor=command['actor']
            if not self.observe(actor): return {'dispatched':sent, 'paused':True}
            claimed=self.admin('claim',{'wake_id':command['wake_id']})['command']
            if not claimed: continue
            try:
                host=self.host(actor)
                if not host or host.session_id != claimed['thread_id']:
                    raise ValueError('Wake does not match the bound role')
                ack=host.send(claimed['prompt'])
                if ack.get('threadId') != claimed['thread_id']:
                    raise ValueError('Child turn acknowledgement mismatch')
            except Exception:
                self.admin('dispatch-result',{'wake_id':claimed['wake_id'],'outcome':'uncertain',
                                              'result':{'error':'Claude wake outcome requires review'}})
                self.pause('Claude wake outcome uncertain')
                return {'dispatched':sent, 'paused':True}
            self.admin('dispatch-result',{'wake_id':claimed['wake_id'],'outcome':'sent','result':{'accepted':True}})
            sent.append(claimed['wake_id'])
        return {'dispatched':sent, 'paused':False}
