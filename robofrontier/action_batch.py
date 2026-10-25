"""Finite same-cycle commands, optionally signed in parts by robot owners.

No future task queue, geometry policy or object lookup. Group records use the
existing store; physical execution and per-robot ownership remain unchanged.
"""
import hashlib
import json
import math
import re
import threading
import time


def child_ids(call_id, count):
    return ['batch-'+hashlib.sha256((call_id+':'+str(i)).encode()).hexdigest()[:40] for i in range(count)]


def groups(store):return store.get('action_groups',{})


def save_group(store,group):
    values=groups(store);values[group['id']]=group;store.put('action_groups',values)


def part_ids(store,call_id):
    call=store.call(call_id);data=json.loads(call['parameters']);spec=data.get('sync_group',{})
    group=groups(store).get(spec.get('id')) if isinstance(spec,dict) else None
    if group and any(p['id']==call_id for p in group['parts']):return [p['id'] for p in group['parts']]
    return [call_id]


def member_ids(store,call_id,all_parts=False):
    parts=part_ids(store,call_id) if all_parts else [call_id];result=[]
    for part in parts:
        actions=json.loads(store.call(part)['parameters']).get('actions',[])
        if isinstance(actions,list):result.extend(child_ids(part,len(actions)))
    return result


def external_wait(store,action):
    """The immutable caller request, not the internal result-collection flag."""
    parent=action.get('batch_id')
    if parent:
        try:return json.loads(store.call(parent)['parameters']).get('wait',True) is not False
        except (ValueError,KeyError,TypeError):return True
    return action.get('parameters',{}).get('wait',True) is not False


def get_batch_status(store,call_id):
    call=store.call(call_id)
    if call['op']!='action_batch':raise ValueError('Not an action batch')
    own_ids=member_ids(store,call_id);ids=member_ids(store,call_id,True);all_actions=store.actions()
    actions={i:all_actions[i] for i in ids if i in all_actions};original=call['result'] or {}
    if call['status']=='uncertain' or any(a['status']=='uncertain' for a in actions.values()):status='uncertain'
    elif any(a['status']=='running' for a in actions.values()):status='running'
    elif any(a['status']=='pending' for a in actions.values()):status='pending'
    elif actions and len(actions)==len(ids):status='completed' if all(a['status']=='completed' for a in actions.values()) else 'failed'
    else:status='failed' if original.get('error') else 'pending'
    # Shared execution facts do not open another role's commands or mailbox.
    executions={i:(a if i in own_ids else {k:v for k,v in a.items() if k in
                    ('action_id','arm','type','status','started_sim_s','finished_sim_s','reason')}) for i,a in actions.items()}
    feedback={}
    for ident in own_ids:
        if ident in actions:
            result=store.call(ident)['result'] or {}
            if 'execution_feedback' in result:feedback[ident]=result['execution_feedback']
    return dict(execution_status=status,batch_id=call_id,action_ids=[i for i in own_ids if i in actions],executions=executions,
                member_feedback=feedback,execution_started=any('started_sim_s' in a or a['status']=='uncertain' for a in actions.values()),
                note='Same-cycle start, not synchronized arrival or a payload/collision guarantee. Inspect each member outcome.')


def summary(service,job):return get_batch_status(service.store,job['id'])


def _reserve(service,job,prepared,pending=False,group_id=None):
    for ident,(robot,action,command) in zip(child_ids(job['id'],len(prepared)),prepared):
        parameters={**action['parameters'],'robot':robot,'wait':True,'cameras':[]}
        if service.store.begin_call(ident,job['actor'],action['op'],parameters)['status']!='new':
            raise ValueError('Existing member; query original batch, never replay')
        service.store.start_action(ident,job['actor'],action['op'],parameters,float(service.world.time),
                                  command.get('timing'),batch_id=job['id'],pending=pending,sync_group_id=group_id)


def _start(service,parts,prepared,activate=False):
    """One executor implementation for same-owner and multi-owner submissions."""
    members=[]
    for job,commands in zip(parts,prepared):
        for ident,(robot,action,command) in zip(child_ids(job['id'],len(commands)),commands):
            members.append((job,ident,robot,action,command))
    if activate:
        with service.store.transaction():
            for job,ident,robot,action,command in members:
                old=service.store.actions()[ident]
                service.store.start_action(ident,job['actor'],action['op'],old['parameters'],float(service.world.time),
                    command.get('timing'),batch_id=job['id'],sync_group_id=old['sync_group_id'])
            group=groups(service.store)[parts[0]['data']['sync_group']['id']]
            group['state']='running';save_group(service.store,group)
    for job,ident,robot,action,command in members:
        parameters={**action['parameters'],'robot':robot,'wait':True,'cameras':[]}
        service.requests[ident]=dict(id=ident,actor=job['actor'],op=action['op'],data=parameters,
                                     result=None,batch_id=job['id'],done=threading.Event())
    try:
        for job,ident,robot,action,command in members:service.world.start_action(robot,ident,command)
        service.store.event('ACTION_BATCH_STARTED','executor',{'parts':[p['id'] for p in parts],
                            'action_ids':[m[1] for m in members],'sim_time_s':float(service.world.time)})
    except Exception as error:
        for job,ident,robot,action,command in members:
            if action['op']=='move':
                try:service.world.request_stop(robot,ident)
                except Exception:pass
            service.store.uncertain_action(ident,'Batch start acknowledgement failed: '+str(error))
        for job in parts:service.respond(job,summary(service,job));job['done'].set()
        return
    for job in parts:
        service.action_batches[job['id']]=job
        if not job['data'].get('wait',True):service.respond(job,summary(service,job))


def _prepare(service,job):
    data=job['data'];actions=data.get('actions')
    if set(data)-{'actions','wait','cameras','sync_group'} or not isinstance(data.get('wait',True),bool):
        raise ValueError('action_batch accepts actions, boolean wait, cameras and optional sync_group')
    if not isinstance(actions,list) or not 1<=len(actions)<=len(service.task['arms']):
        raise ValueError('Provide one current action per selected owned robot')
    service.selected_cameras(data.get('cameras',[]),allow_empty=True)
    prepared=[];used=set()
    for action in actions:
        if not isinstance(action,dict) or set(action)!={'robot','op','parameters'} or action['op'] not in ('move','gripper'):
            raise ValueError('Members require robot, move/gripper op and parameters')
        robot=service.store.action_robot(job['actor'],action['robot'])
        if robot in used:raise ValueError('One batch action per robot')
        used.add(robot)
        if not isinstance(action['parameters'],dict):raise ValueError('Action parameters must be an object')
        prepared.append((robot,action,service.world.validate_action(robot,action['op'],action['parameters'],service.config)))
    return prepared


def submit(service,job):
    prepared=_prepare(service,job);spec=job['data'].get('sync_group')
    if spec is None:
        with service.store.transaction():_reserve(service,job,prepared)
        _start(service,[job],[prepared]);return
    if not isinstance(spec,dict) or set(spec)!={'id','robots','timeout_s'}:
        raise ValueError('sync_group requires id, robots and timeout_s')
    ident=spec['id'];robots=spec['robots'];timeout=spec['timeout_s']
    if not isinstance(ident,str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,80}',ident):raise ValueError('Invalid sync group ID')
    if not isinstance(robots,list) or not robots or any(not isinstance(r,str) or r not in service.task['arms'] for r in robots) or len(set(robots))!=len(robots):
        raise ValueError('Sync group requires distinct known robot IDs')
    if isinstance(timeout,bool) or not isinstance(timeout,(int,float)) or not math.isfinite(timeout) or not 0<timeout<=service.config['max_check_interval_s']:
        raise ValueError('Choose a finite synchronization timeout within run limits')
    owned=set()
    for r in robots:
        try:service.store.action_robot(job['actor'],r);owned.add(r)
        except ValueError:pass
    if {p[0] for p in prepared}!=owned:raise ValueError('Submit exactly your owned robots from this group roster')
    with service.store.transaction():
        group=groups(service.store).get(ident)
        if group:
            if group['state']!='collecting':raise ValueError('Sync group already committed/closed; query it or choose a new group ID')
            if group['robots']!=sorted(robots) or group['timeout_s']!=timeout:raise ValueError('Sync group contract mismatch')
            if time.time()>=group['deadline_at']:raise ValueError('Sync deadline expired; never renew by resubmission')
            if any(p['actor']==job['actor'] for p in group['parts']):raise ValueError('One immutable submission per owner in a sync group')
        else:group=dict(id=ident,robots=sorted(robots),timeout_s=timeout,deadline_at=time.time()+timeout,state='collecting',parts=[])
        _reserve(service,job,prepared,pending=True,group_id=ident)
        group['parts'].append(dict(id=job['id'],actor=job['actor'],robots=sorted(owned)));save_group(service.store,group)
        service.store.event('SYNC_PART_RESERVED',job['actor'],{'group_id':ident,'call_id':job['id'],'robots':sorted(owned),'deadline_at':group['deadline_at']})
    service.action_batches[job['id']]=job
    if not job['data'].get('wait',True):service.respond(job,summary(service,job))


def cancel_collecting(store,reason,sim_time,group_id=None):
    """Known unstarted commands are failed, not ambiguous physical execution."""
    with store.transaction():
        for group in groups(store).values():
            if group['state']!='collecting' or group_id is not None and group['id']!=group_id:continue
            for part in group['parts']:
                for ident in member_ids(store,part['id']):
                    action=store.actions()[ident]
                    if action['status']!='pending':raise ValueError('Collecting group contains a started member; recovery required')
                    action=store.finish_action(ident,'failed',sim_time,reason)
                    store.finish_call(ident,{'call_id':ident,'execution':action})
            group.update(state='terminal',reason=reason);save_group(store,group)
            for part in group['parts']:
                value=get_batch_status(store,part['id']);store.finish_call(part['id'],{'call_id':part['id'],**value})
                store.signal(part['actor'],{'event':'sync_group_cancelled','call_id':part['id'],'reason':reason})
            store.event('SYNC_GROUP_CANCELLED','executor',{'group_id':group['id'],'reason':reason})


def stop_group_motions(service,job,reason):
    actions=service.store.actions()
    for ident in member_ids(service.store,job['id'],True):
        action=actions.get(ident)
        if action and action['status']=='running' and action['type']=='move':service.world.request_stop(action['arm'],ident)
    service.store.event('ACTION_BATCH_MOTION_STOP_REQUESTED',job['actor'],{'batch_id':job['id'],'reason':reason,
                            'scope':'Agreed group braking; does not open grippers or assert payload retention'})


def stop(service,job):
    spec=job['data'].get('sync_group');group=groups(service.store).get(spec['id']) if spec else None
    if group and group['state']=='collecting':cancel_collecting(service.store,'withdrawn_by_participant',float(service.world.time),group['id'])
    else:stop_group_motions(service,job,'Agent requested group motion stop')


def poll(service):
    store=service.store
    for group in groups(store).values():
        if group['state']!='collecting':continue
        paused=store.get('dispatch_paused',False) or store.get('physics_state')!='running' or service.ended
        if paused or time.time()>=group['deadline_at']:
            cancel_collecting(store,'run_paused_or_ended' if paused else 'synchronization_deadline_expired',float(service.world.time),group['id']);continue
        if set(r for p in group['parts'] for r in p['robots'])!=set(group['robots']):continue
        parts=[service.requests[p['id']] for p in group['parts']]
        try:
            prepared=[_prepare(service,p) for p in parts]
            _start(service,parts,prepared,activate=True)
        except (ValueError,TypeError,KeyError) as error:
            cancel_collecting(store,'commit_validation_failed: '+str(error),float(service.world.time),group['id'])
    for ident,job in list(service.action_batches.items()):
        value=summary(service,job)
        if value['execution_status']=='pending':continue
        if not job.get('batch_braking') and any(a['status'] in ('failed','uncertain') for a in value['executions'].values()):
            stop_group_motions(service,job,'A member did not complete')
            for part in part_ids(store,ident):
                if part in service.action_batches:service.action_batches[part]['batch_braking']=True
        if value['execution_status']=='running':continue
        value['observation']=service.capture(job['data'].get('cameras',[]))
        service.respond(job,value);job['done'].set()
        if not job['data'].get('wait',True):store.signal(job['actor'],{'event':'action_batch_finished','call_id':ident,'status':value['execution_status']})
        spec=job['data'].get('sync_group')
        if spec:
            group=groups(store)[spec['id']]
            if group['state']=='running':group['state']='terminal';save_group(store,group)
        del service.action_batches[ident]
