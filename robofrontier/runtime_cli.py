"""Local administrative operations for Meta's deterministic dispatcher.

This module never calls Codex or controls a robot.
"""
import argparse
import json
from pathlib import Path
import time
from .runtime_store import RuntimeStore
from .scheduling import Scheduler
from .delegation import Delegations


def fork_provenance(parent, child):
    """Verify native lineage without exposing conversation contents to policy."""
    import hashlib
    import re
    if any(not re.fullmatch(r'[0-9a-f-]{36}',v) for v in (parent,child)):
        raise ValueError('Expected native session UUIDs')
    def records(tid):
        import os
        home=Path(os.environ.get('CODEX_HOME',Path.home()/'.codex'))
        paths=[p for directory in ('sessions','archived_sessions')
               for p in (home/directory).glob('**/*'+tid+'*.jsonl')]
        indexed={}
        for path in paths:
            rows=[json.loads(line) for line in path.read_text().splitlines()]
            meta=next((r['payload'] for r in rows if r['type']=='session_meta'),{})
            if meta.get('id')!=tid:continue  # parent IDs can appear in fork filenames
            for row in rows:
                ordinal=row.get('ordinal')
                if type(ordinal) is not int:raise ValueError('Native ordinal missing')
                previous=indexed.get(ordinal)
                if previous is not None and previous!=row:
                    raise ValueError('Conflicting native history segments')
                indexed[ordinal]=row
        if not indexed:raise ValueError('Native session provenance missing')
        return [indexed[k] for k in sorted(indexed)]
    rows=records(child)
    meta=next(r['payload'] for r in rows if r['type']=='session_meta')
    cutoff=meta.get('forked_from_ordinal_exclusive')
    if meta.get('forked_from_id')!=parent or type(cutoff) is not int:
        raise ValueError('Native fork lineage does not match requested parent')
    prefix=[r for r in records(parent) if r['ordinal']<cutoff]
    return {'source_thread_id':parent,'source_ordinal':cutoff,
            'source_sha256':hashlib.sha256(json.dumps(prefix,sort_keys=True).encode()).hexdigest()}


def native_fork_provenance(store, parent, child):
    """Dispatch lineage verification to the evidence format of this run's engine."""
    engine=store.get('config').get('engine','codex')
    if engine=='claude':
        from .claude.provenance import fork_provenance as claude_provenance
        # One isolated Claude config directory per run, shared by every role session.
        return claude_provenance(parent,child,store.path.parents[1]/'claude-config')
    return fork_provenance(parent,child)


def execute(store, op, data, owner, now):
    cfg=store.get('config')
    if op=='seal':
        from .run_seal import seal_run
        if set(data)-{'timeout_s'}:raise ValueError('Seal accepts only an engineering timeout_s')
        return seal_run(store,data.get('timeout_s',30))
    if op=='snapshot':
        return {'lifecycle':store.get('lifecycle'),'paused':store.get('dispatch_paused',False),
                'dispatcher_health':store.get('dispatcher_health',{}),
                'config':{k:cfg[k] for k in ('poll_interval_s','session_observation_ttl_s','model','effort')},
                'agents':{a:{k:x for k,x in v.items() if k!='note'} for a,v in store.agents().items()},
                'delegations':Delegations(store).active() if cfg['topology']=='adaptive' else [],
                'pending':{a:store.pending(a) for a,v in store.agents().items() if v['kind']!='manager'}}
    if op=='delegate-bind':
        # Commit identity before reading platform files: a failed check must
        # not erase a native fork that already exists.
        d=Delegations(store);key=data['id']
        with store.transaction():
            store.lease(owner,now,cfg['manager_lease_s'])
            d.created(key,data['actor'],data['thread_id'],owner)
        provenance=native_fork_provenance(store,d.batch(key)['source_thread_id'],data['thread_id'])
        with store.transaction():
            store.lease(owner,time.time(),cfg['manager_lease_s'])
            d.bind(key,data['actor'],data['thread_id'],provenance,owner)
        return {'recorded':True}
    if op.startswith('delegate-'):
        with store.transaction():
            store.lease(owner,now,cfg['manager_lease_s'])
            d=Delegations(store);key=data['id']
            if op=='delegate-claim':return {'delegation':d.claim(key,owner)}
            if op=='delegate-activate':d.activate(key,owner)
            elif op=='delegate-started':
                b=d.started(key,data['actor'],data['outcome'],owner)
                return {'recorded':True,'status':b['status']}
            elif op=='delegate-uncertain':
                # A new lease owner may flag an abandoned creation for review,
                # but must not acquire it and create another set of forks.
                d.uncertain(key,data['reason'],d.batch(key)['claimed_by'])
            else:raise ValueError('Unknown native delegation operation')
            return {'recorded':True}
    if op=='session-unavailable':
        with store.transaction():
            store.lease(owner,now,cfg['manager_lease_s'])
            a=store.agent(data['actor'])
            if a['thread_id']!=data['thread_id']:raise ValueError('Unavailable status has wrong role binding')
            store.update_agent(data['actor'],session_status='unknown',session_observed_at=None)
            health=store.get('dispatcher_health',{})
            health.update(owner=owner,last_cycle_at=now,last_error={'at':now,'actor':data['actor'],'reason':data['reason']})
            store.put('dispatcher_health',health)
            store.event('SESSION_STATUS_UNAVAILABLE',cfg['manager'],data)
        return {'recorded':True}
    if op=='observe-sessions':
        with store.transaction():
            store.lease(owner,now,cfg['manager_lease_s'])
            for item in data['observations']:
                agent=store.agent(item['actor'])
                if item['thread_id']!=agent['thread_id']:
                    raise ValueError('Observed thread does not match bound role')
                store.observe_session(item['actor'],item['status'],item['turn_id'],item['observed_at'],item['source'],item.get('turn_status'))
            if data['observations']:
                health=store.get('dispatcher_health',{})
                health.update(owner=owner,last_cycle_at=now,
                    last_successful_observation_at=max(item['observed_at'] for item in data['observations']))
                if 'cycle' in data:health['cycle']=data['cycle']
                store.put('dispatcher_health',health)
        return {'observed':len(data['observations'])}
    if op=='request-start':
        from contextlib import redirect_stderr
        from .pipeline_client import client_call
        connection=store.path.parent/'connections'/f"{cfg['manager']}.json"
        with (store.path.parent/'admin-client.log').open('a') as log, redirect_stderr(log):
            return client_call(connection,'start',{},'start-'+__import__('hashlib').sha256(store.get('run_id').encode()).hexdigest()[:24])
    if op=='plan':return {'commands':Scheduler(store).plan(owner,now)}
    if op=='claim':return {'command':Scheduler(store).claim(data['wake_id'],owner,now)}
    if op=='dispatch-result':
        store.record_dispatch(data['wake_id'],data['outcome'],data.get('result'))
        if data['outcome']=='sent':
            health=store.get('dispatcher_health',{});health['last_delivery_at']=now;store.put('dispatcher_health',health)
        return {'recorded':True}
    if op=='pause':
        store.put('dispatch_paused',True);store.event('DISPATCH_PAUSED',cfg['manager'],data)
        return {'paused':True}
    if op=='resume-dispatch':
        if store.get('lifecycle')!='running':raise ValueError('Run is not running')
        store.put('dispatch_paused',False);store.event('DISPATCH_RESUMED',cfg['manager'],data)
        return {'paused':False}
    if op=='export':
        store.export_events(Path(data['path']))
        return {'exported':data['path']}
    raise ValueError('Unknown runtime operation')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-dir',type=Path,required=True)
    p.add_argument('--owner',default='Meta')
    p.add_argument('op',choices=['snapshot','observe-sessions','request-start','plan','claim','dispatch-result','pause','resume-dispatch','export',
                               'delegate-claim','delegate-bind','delegate-activate','delegate-started','delegate-uncertain','session-unavailable','seal'])
    p.add_argument('--data',default='{}')
    args=p.parse_args()
    path=args.run_dir.resolve()/'private/state/runtime.sqlite'
    if not path.is_file():raise SystemExit('Run is not prepared')
    store=RuntimeStore(path)
    try:print(json.dumps(execute(store,args.op,json.loads(args.data),args.owner,time.time()),ensure_ascii=False))
    finally:store.close()

if __name__=='__main__':main()
