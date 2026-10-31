"""Experimental host-controlled adaptive fork setup; not an admission endpoint.

The caller is the trusted process which owns the active parent and shared budget.
This module does not start a world, schedule subsequent wakes, or qualify a batch.
"""
import json
import hashlib
import math
import threading
from pathlib import Path
import time

from robofrontier.claude.fork_setup import _boundary, create_fork
from robofrontier.claude.provenance import fork_provenance
from robofrontier.claude.session_host import ClaudeSessionHost
from robofrontier.delegation import Delegations
from robofrontier.runtime_cli import execute


def child_prompt(actor, child):
    scope = {key: child[key] for key in ('robots', 'goal', 'done_when', 'constraints') if key in child}
    return ('Your current runtime role is ' + actor + '. This assignment replaces the inherited parent role, '
            'connection and finish authority. The parent task evidence restrictions remain in force. '
            'Use only your bound robot MCP tool and assigned robots; inherited pending calls are history, '
            'not work to replay. You cannot create delegates or finish the team run. '
            'Carry out the bounded scope, communicate through the role protocol, and call return_task '
            'with outcome and evidence when handing back. Save a waiting state if you must pause. '
            'The assigned scope is: ' + json.dumps(scope, ensure_ascii=False))


class AdaptiveForkSetup:
    def __init__(self, store, parent, ledger, owner, *, setup_budget_usd, child_budget_usd,
                 fork_factory=create_fork, child_factory=ClaudeSessionHost.adopt_verified_fork):
        cfg = store.get('config')
        if cfg.get('engine') != 'claude' or cfg.get('topology') != 'adaptive':
            raise ValueError('Expected adaptive Claude runtime')
        if parent.actor != 'Controller' or store.agent('Controller')['thread_id'] != parent.session_id:
            raise ValueError('Parent host does not match runtime')
        if parent.budget_ledger is not ledger:
            raise ValueError('Parent and children must share the exact task budget ledger')
        if not Path(ledger.path).resolve().is_relative_to(Path(parent.run).resolve()/'private'):
            raise ValueError('Task budget ledger must belong to the run')
        for value in (setup_budget_usd, child_budget_usd):
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value <= 0:
                raise ValueError('Positive finite fork and child budgets are required')
        self.store=store; self.parent=parent; self.ledger=ledger; self.owner=owner
        self.setup_budget_usd=setup_budget_usd; self.child_budget_usd=child_budget_usd
        self.fork_factory=fork_factory; self.child_factory=child_factory
        self.hosts={}
        self.cancel_event=threading.Event()
        self.launch_lock=threading.RLock()

    def request_stop(self):
        """Prevent new child turns and interrupt any tool-free fork creation."""
        self.cancel_event.set()
        with self.launch_lock:
            for host in list(self.hosts.values()): host.stop()

    def admin(self, op, data):
        return execute(self.store, op, data, self.owner, time.time())

    def establish(self, batch_id):
        """Claim and set up once; errors preserve reservations and pause the run."""
        delegates=Delegations(self.store)
        batch=self.admin('delegate-claim', {'id':batch_id})['delegation']
        try:
            first=batch['children'][0]['actor']
            snap=self.fork_factory(self.store,self.parent,batch_id,first,self.owner,
                                   max_budget_usd=self.setup_budget_usd,budget_ledger=self.ledger,
                                   cancel_event=self.cancel_event,snapshot=True)
            snap_path=Path(self.parent.run)/'private/claude-forks'/('snapshot-'+hashlib.sha256(batch_id.encode()).hexdigest()+'.json')
            if (json.loads(snap_path.read_text()) != snap or snap.get('status') != 'history_verified_no_authority'
                    or not snap.get('snapshot') or snap.get('batch_id') != batch_id
                    or snap.get('actor') != first or snap.get('owner') != self.owner
                    or snap.get('parent_session') != self.parent.session_id
                    or snap.get('source_session') != self.parent.session_id
                    or snap['proof']['source_sha256'] != snap['parent_history_sha256']
                    or not isinstance(snap.get('snapshot_sha256'),str)):
                raise ValueError('Trusted snapshot record is incomplete or changed')
            # create_fork already checked the parent while creating the snapshot;
            # the parent may append afterwards, so retain that verified proof.
            delegates.snapshot_bind(batch_id,snap['child_session'],
                                    {**snap['proof'],'snapshot_sha256':snap['snapshot_sha256']},self.owner)
            for child in batch['children']:
                if self.cancel_event.is_set(): raise ValueError('Fork setup cancelled')
                actor=child['actor']
                result=self.fork_factory(self.store, self.parent, batch_id, actor, self.owner,
                                         max_budget_usd=self.setup_budget_usd, budget_ledger=self.ledger,
                                         cancel_event=self.cancel_event,source_session=snap['child_session'])
                if self.cancel_event.is_set(): raise ValueError('Fork setup cancelled')
                _boundary(self.store,self.parent,batch_id,actor,self.owner)
                # The known ID is committed before any further fallible checks.
                with self.store.transaction():
                    self.store.lease(self.owner,time.time(),self.store.get('config')['manager_lease_s'])
                    delegates.created(batch_id,actor,result['child_session'],self.owner)
                claim_path=Path(self.parent.run)/'private/claude-forks'/(actor+'.json')
                claim=json.loads(claim_path.read_text())
                if claim != result or result.get('status') != 'history_verified_no_authority':
                    raise ValueError('Trusted fork record is incomplete or changed')
                if (result['actor'] != actor or result['batch_id'] != batch_id or result['owner'] != self.owner
                        or result['parent_session'] != self.parent.session_id
                        or result.get('source_session') != snap['child_session']
                        or result['proof']['source_sha256'] != result['parent_history_sha256']
                        or fork_provenance(result['source_session'],result['child_session'],
                                           Path(self.parent.run)/'private/claude-config') != result['proof']):
                    raise ValueError('Trusted fork record does not match requested scope')
                with self.store.transaction():
                    self.store.lease(self.owner,time.time(),self.store.get('config')['manager_lease_s'])
                    delegates.bind(batch_id,actor,result['child_session'],result['proof'],self.owner)
            with self.launch_lock:
                if self.cancel_event.is_set(): raise ValueError('Fork setup cancelled')
                self.admin('delegate-activate', {'id':batch_id})
            for child in delegates.batch(batch_id)['children']:
                actor=child['actor']
                with self.launch_lock:
                    if self.cancel_event.is_set(): raise ValueError('Fork setup cancelled')
                    host=self.child_factory(self.parent.run,actor,self.parent.profile,self.parent.base_env,
                                            budget_ledger=self.ledger,max_budget_usd=self.child_budget_usd)
                    self.hosts[actor]=host
                    ack=host.send(child_prompt(actor,child))
                if ack.get('threadId') != child['thread_id']:
                    raise ValueError('Child start acknowledgement has wrong session')
                self.admin('delegate-started', {'id':batch_id,'actor':actor,'outcome':'sent'})
            return {'status':'active','actors':list(self.hosts)}
        except Exception as error:
            # Even a failed fork may have created its explicitly allocated ID.
            for child in delegates.batch(batch_id)['children']:
                path=Path(self.parent.run)/'private/claude-forks'/(child['actor']+'.json')
                if not path.is_file(): continue
                try:
                    saved=json.loads(path.read_text()); ident=saved.get('child_session')
                    if ident and saved.get('batch_id')==batch_id and saved.get('actor')==child['actor']:
                        delegates.created(batch_id,child['actor'],ident,self.owner)
                except Exception: pass
            try:
                delegates.uncertain(batch_id,'Claude fork setup or child start requires engineering review',self.owner)
            finally:
                # Preserve paused state even if writing the delegation outcome fails.
                self.store.put('dispatch_paused',True)
            return {'status':'needs_review','error_type':type(error).__name__,'actors':list(self.hosts)}
