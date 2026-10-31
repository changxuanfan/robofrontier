"""Durable bounded delegation; scopes are authored by Controller, never solved here."""
import json
import time
import uuid

from .runtime_store import packed


class Delegations:
    def __init__(self, store):
        self.s = store

    def batches(self):
        return self.s.get('delegations', {})

    def batch(self, key):
        return self.batches()[key]

    def save(self, batch):
        values = self.batches(); values[batch['id']] = batch
        self.s.put('delegations', values)

    def active(self):
        return [b for b in self.batches().values() if b['status'] != 'finished']

    def owners(self):
        return self.s.get('robot_owners', {r:'Controller' for r in self.s.get('robots',[])})

    def check_idle(self, robots):
        if any(a['arm'] in robots and a['status'] in ('pending','running','uncertain') for a in self.s.actions().values()):
            raise ValueError('Resolve running/uncertain robot actions before handing over')
        for group in self.s.get('action_groups',{}).values():
            if group['state'] in ('collecting','running') and any(set(p['robots'])&set(robots) for p in group['parts']):
                raise ValueError('Resolve the signed group operation before handing over')
        for row in self.s.db.execute("SELECT * FROM calls WHERE status='accepted' AND op IN ('move','gripper','action_batch')"):
            parameters=json.loads(row['parameters'])
            selected=([a.get('robot') for a in parameters.get('actions',[]) if isinstance(a,dict)]
                      if row['op']=='action_batch' else [parameters.get('robot',row['actor'])])
            if set(selected)&set(robots):
                raise ValueError('Resolve accepted robot calls before handing over')

    def create(self, actor, key, scopes):
        with self.s.transaction():
            if self.s.get('config')['topology'] != 'adaptive' or actor != 'Controller':
                raise ValueError('Only adaptive Controller can delegate')
            if self.s.get('lifecycle') != 'running' or self.s.get('physics_state') != 'running' or self.s.get('dispatch_paused',False):
                raise ValueError('Delegation requires a running unpaused world')
            if key in self.batches():
                return self.batch(key)
            if any(b['status']=='creating' for b in self.active()):
                raise ValueError('One native fork batch may be establishing its source boundary')
            if not isinstance(scopes,list) or not scopes:
                raise ValueError('Provide a nonempty list of bounded scopes')
            if len(packed(scopes).encode())>self.s.get('config')['max_batch_bytes']:
                raise ValueError('Delegation batch exceeds run byte limit')
            owners = self.owners(); used = set()
            for scope in scopes:
                if not isinstance(scope,dict) or set(scope)-{'robots','goal','done_when','constraints'}:
                    raise ValueError('Scope fields: robots, goal, done_when, optional constraints')
                if not isinstance(scope.get('constraints',''),str):raise ValueError('Constraints must be text')
                robots = scope['robots']
                if (not isinstance(robots,list) or not robots or len(set(robots))!=len(robots)
                    or any(r not in owners or owners[r]!=actor for r in robots) or used.intersection(robots)):
                    raise ValueError('Scopes require disjoint robots currently controlled by Controller')
                for field in ('goal','done_when'):
                    if not isinstance(scope.get(field),str) or not scope[field].strip():
                        raise ValueError('Each scope needs goal and done_when')
                if len(packed(scope)) > self.s.get('config')['max_message_chars']:
                    raise ValueError('Scope exceeds message size limit')
                self.check_idle(robots); used.update(robots)
            parent = self.s.agent(actor)
            if not parent['thread_id']:
                raise ValueError('Bind the native parent session first')
            batch = {'id':key,'parent':actor,'source_thread_id':parent['thread_id'],
                     'status':'creating','created_at':time.time(),'claimed_by':None,'children':[],
                     'setup_deadline':time.time()+self.s.get('config').get('native_setup_timeout_s',120)}
            for scope in scopes:
                name = 'Delegate_'+uuid.uuid4().hex[:12]
                child = {'actor':name, **scope, 'thread_id':None, 'verified':False,
                         'status':'creating','start_status':'pending'}
                batch['children'].append(child)
                info = dict(parent)
                info.update(kind='delegate', thread_id=None, ready=False, robots=scope['robots'],
                            delegation_id=key, delegation_state='creating', work_state='waiting',
                            note='', work_revision=0, next_check=None, work_saved_at=None,
                            session_status='unknown',session_turn=None,session_observed_at=None,recovery_required=False)
                self.s.db.execute('INSERT INTO agents VALUES (?,?)',(name,packed(info)))
                for r in scope['robots']:owners[r] = name
            self.s.put('robot_owners',owners);self.save(batch)
            self.s.event('DELEGATION_REQUESTED',actor,batch)
            return batch

    def claim(self, key, owner):
        with self.s.transaction():
            b = self.batch(key)
            if b['status']!='creating' or b['claimed_by'] is not None:
                raise ValueError('Fork batch already claimed; never automatically refork')
            b['claimed_by']=owner;self.save(b)
            self.s.event('DELEGATION_CLAIMED','Meta',{'id':key,'owner':owner})
            return b

    def owned(self, key, owner):
        b = self.batch(key)
        if b['claimed_by'] != owner or b['status'] not in ('creating','active'):
            raise ValueError('Native delegation ownership/state changed')
        return b

    def created(self, key, actor, thread_id, owner):
        """Record a known native ID before fallible lineage reads; no permission grant."""
        with self.s.transaction():
            b=self.batch(key)
            if b['claimed_by']!=owner:raise ValueError('Native delegation ownership changed')
            c=next(c for c in b['children'] if c['actor']==actor)
            if c['thread_id']==thread_id:return b
            if c['thread_id'] is not None or not isinstance(thread_id,str) or not thread_id:
                raise ValueError('Child already recorded or native ID invalid')
            if any(a!=actor and v['thread_id']==thread_id for a,v in self.s.agents().items()):
                raise ValueError('Native session already belongs to another role')
            c['thread_id']=thread_id;self.save(b)
            self.s.update_agent(actor,thread_id=thread_id)
            self.s.event('NATIVE_FORK_CREATED','Meta',{'id':key,'actor':actor,'thread_id':thread_id,'verified':False})
            return b

    def snapshot_created(self, key, thread_id, owner):
        """Persist a native snapshot ID before fallible lineage verification."""
        with self.s.transaction():
            b = self.owned(key, owner)
            if b['status'] != 'creating' or not isinstance(thread_id, str) or not thread_id:
                raise ValueError('Invalid snapshot creation state or ID')
            if b.get('snapshot_thread_id') not in (None, thread_id):
                raise ValueError('Snapshot already recorded')
            if thread_id == b['source_thread_id'] or any(a['thread_id'] == thread_id for a in self.s.agents().values()):
                raise ValueError('Snapshot must be a separate unassigned native session')
            b['snapshot_thread_id'] = thread_id
            self.save(b)
            self.s.event('DELEGATION_SNAPSHOT_CREATED', 'Meta', {'id':key, 'thread_id':thread_id, 'verified':False})
            return b

    def snapshot_bind(self, key, thread_id, provenance, owner):
        with self.s.transaction():
            b = self.owned(key, owner)
            if b['status'] != 'creating' or b.get('snapshot_thread_id') != thread_id:
                raise ValueError('Snapshot identity or state changed')
            if provenance['source_thread_id'] != b['source_thread_id']:
                raise ValueError('Snapshot has wrong source session')
            if b.get('snapshot_provenance') not in (None, provenance):
                raise ValueError('Snapshot source boundary changed')
            b['snapshot_provenance'] = provenance
            self.save(b)
            self.s.event('DELEGATION_SNAPSHOT_BOUND', 'Meta', {'id':key, 'thread_id':thread_id, **provenance})

    def bind(self, key, actor, thread_id, provenance, owner):
        self.created(key,actor,thread_id,owner)
        with self.s.transaction():
            b = self.owned(key,owner)
            expected_source = b.get('snapshot_thread_id') or b['source_thread_id']
            if b.get('snapshot_thread_id') and not b.get('snapshot_provenance'):
                raise ValueError('Snapshot is not verified')
            if provenance['source_thread_id']!=expected_source:
                raise ValueError('Fork has wrong source session')
            if b.get('source_provenance') not in (None,provenance):
                raise ValueError('Forks did not share the exact source history boundary')
            c = next(c for c in b['children'] if c['actor']==actor)
            b['source_provenance']=provenance;c['verified']=True;self.save(b)
            self.s.event('DELEGATION_FORK_BOUND','Meta',{'id':key,'actor':actor,'thread_id':thread_id,**provenance})

    def activate(self, key, owner):
        with self.s.transaction():
            b = self.owned(key,owner)
            if b.get('snapshot_thread_id') and not b.get('snapshot_provenance'):
                raise ValueError('Snapshot is not verified')
            if not all(c['thread_id'] and c.get('verified') for c in b['children']):
                raise ValueError('Fork batch incomplete or unverified')
            b['status']='active';b['activated_at']=time.time()
            for c in b['children']:
                c['status']='active'
                self.s.update_agent(c['actor'],delegation_state='active',ready=True,work_state='continue',
                                    note='Execute assigned bounded scope; inherited parent pending calls are historical')
            self.save(b);self.s.event('DELEGATION_ACTIVATED','Meta',{'id':key})

    def started(self, key, actor, outcome, owner):
        with self.s.transaction():
            b = self.batch(key)
            if b['claimed_by']!=owner or b['status'] not in ('active','finished','needs_review'):
                raise ValueError('Native delegation ownership/state changed')
            c = next(c for c in b['children'] if c['actor']==actor)
            if outcome not in ('sent','uncertain'):raise ValueError('Invalid initial input outcome')
            if outcome=='uncertain' and c['status']!='finished':
                return self.uncertain(key,'Initial input delivery uncertain for '+actor,owner)
            c['start_status']='observed' if c['status']=='finished' else 'sent'
            c['started_at']=c.get('started_at',time.time());self.save(b)
            self.s.update_agent(actor,delegation_started=True)
            self.s.event('DELEGATION_CHILD_STARTED','Meta',{'id':key,'actor':actor})
            return b

    def uncertain(self, key, reason, owner):
        with self.s.transaction():
            b = self.batch(key)
            if b['claimed_by']!=owner:raise ValueError('Native delegation ownership changed')
            if all(c['status']=='finished' for c in b['children']):
                for c in b['children']:
                    if c['start_status']=='pending':c['start_status']='observed'
                b['status']='finished';self.save(b)
                return b
            if b['status']=='needs_review':return b
            b.update(status='needs_review',reason=reason);self.save(b)
            self.s.put('dispatch_paused',True)
            self.s.signal(b['parent'],{'event':'delegation_needs_review','id':key,'reason':reason})
            self.s.event('DELEGATION_NEEDS_REVIEW','Meta',{'id':key,'reason':reason})
            return b

    def return_task(self, actor, outcome, result):
        with self.s.transaction():
            a = self.s.agent(actor)
            if a['kind']!='delegate' or a['delegation_state']!='active':raise ValueError('No active delegated scope')
            if outcome not in ('completed','failed','cancelled') or not isinstance(result,str) or not result.strip():
                raise ValueError('Return outcome and evidence/result text')
            self.check_idle(a['robots'])
            b=self.batch(a['delegation_id']);c=next(c for c in b['children'] if c['actor']==actor)
            owners=self.owners()
            for r in a['robots']:
                if owners[r]!=actor:raise ValueError('Robot ownership changed unexpectedly')
            from .run_setup import write_json
            path=self.s.path.parents[2]/'public/results'/f'{actor}.json'
            reported_at=time.time()
            try:
                write_json(path,{'delegation_id':b['id'],'actor':actor,'robots':a['robots'],
                                 'scope':{k:c[k] for k in ('robots','goal','done_when','constraints') if k in c},
                                 'outcome':outcome,'reported_at':reported_at,'result':result})
            except OSError as error:
                raise ValueError('Report save failed; ownership retained: '+str(error)) from error
            c.update(status='finished',outcome=outcome,result_ref=str(path),returned_at=reported_at)
            if c['start_status']=='pending':c['start_status']='observed'
            for r in a['robots']:
                owners[r]=b['parent']
            self.s.put('robot_owners',owners)
            self.s.update_agent(actor,delegation_state='finished',work_state='available',next_check=None)
            if all(c['status']=='finished' for c in b['children']):b['status']='finished'
            self.save(b)
            event={'event':'delegation_returned','id':b['id'],'actor':actor,'robots':a['robots'],
                   'outcome':outcome,'result_ref':str(path),'reported_at':c['returned_at'],
                   'note':'Agent-reported result; not evaluator-verified goal truth'}
            self.s.signal(b['parent'],event);self.s.event('DELEGATION_RETURNED',actor,event)
            return event

    def observe_stopped(self, actor, status, turn_id, observed_at):
        """A native turn ending is a fact, not a task return or control release."""
        a=self.s.agent(actor)
        if (a['kind']!='delegate' or a.get('delegation_state')!='active'
            or not a.get('delegation_started') or status not in ('idle','error','needsAttention') or not turn_id
            or a.get('last_stop_notice_turn')==turn_id):
            return
        b=self.batch(a['delegation_id'])
        unresolved=[i for i,v in self.s.actions(actor).items() if v['status'] in ('pending','running','uncertain')]
        unresolved.extend(row['id'] for row in self.s.db.execute(
            "SELECT id FROM calls WHERE actor=? AND status='accepted' AND op IN ('move','gripper','action_batch')",(actor,)))
        self.s.update_agent(actor,last_stop_notice_turn=turn_id,
                            stop_notice_message_seq=self.s.pending(actor)['message_seq'])
        event={'event':'delegate_stopped_without_return','id':b['id'],'actor':actor,
               'thread_id':a['thread_id'],'turn_id':turn_id,'native_status':status,
               'native_turn_status':a.get('session_turn_status'),'observed_at':observed_at,
               'robots':a['robots'],'execution_state':'unresolved' if unresolved else 'idle',
               'unresolved_count':len(unresolved),
               'work_state':a['work_state'],'next_check':a['next_check'],
               'note':'Scope and ownership remain active. Distinguish a deliberate saved review from an unexpected stop using work/execution facts; native idle is neither handback nor clearance.'}
        self.s.signal(b['parent'],event);self.s.event('DELEGATE_STOPPED_WITHOUT_RETURN',actor,event,observed_at)

    def status(self, parent, selected=None):
        """Compact own-child monitoring; full work note only on explicit request."""
        if self.s.agent(parent)['kind']!='controller':raise ValueError('Controller delegate view only')
        result={}
        for batch in self.batches().values():
            if batch['parent']!=parent:continue
            for child in batch['children']:
                actor=child['actor']
                if selected is not None and actor!=selected or selected is None and child['status']=='finished':continue
                a=self.s.agent(actor)
                work={'state':a['work_state'],'saved_at':a.get('work_saved_at'),
                      'revision':a['work_revision'],'next_check':a['next_check']}
                if selected is not None:work['note']=a['note']
                result[actor]={'delegation_id':batch['id'],'robots':child['robots'],'thread_id':a['thread_id'],
                    'state':a['delegation_state'],'session':{'status':a['session_status'],'turn_id':a['session_turn'],
                    'turn_status':a.get('session_turn_status'),
                    'observed_at':a['session_observed_at']},'work':work,
                    'active_executions':{i:{'robot':v['arm'],'status':v['status'],'type':v['type']}
                        for i,v in self.s.actions(actor).items() if v['status'] in ('pending','running','uncertain')},
                    'result_ref':child.get('result_ref')}
                if selected is not None:result[actor]['latest_execution']=self.s.latest_execution(actor)
        if selected is not None and selected not in result:raise ValueError('Select a delegate belonging to this Controller')
        return result

    def authorize(self, actor, robot):
        a=self.s.agent(actor)
        if self.owners().get(robot)!=actor or a['kind']=='delegate' and a['delegation_state']!='active':
            raise ValueError('Robot is reserved/delegated or this scope has ended')
        return robot
