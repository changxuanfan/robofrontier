"""Run complete physical suite references; failed attempts remain immutable."""
import argparse,json,time
from pathlib import Path
from robofrontier.suite.scenes import ROOT,catalog,case_directory,update_case
from robofrontier.suite.oracle import Trial,ReferenceStopped
from robofrontier.suite import placement_oracle,dial_oracle,relay_oracle
from robofrontier.task_config import load_harness
from robofrontier.suite.provenance import bundle_manifest
from robofrontier.run_setup import write_json

def parallel_run(job):
    if _STOP_EVENT.is_set():return dict(case_id=job[0]['id'],verification_seed=job[1],passed=False,skipped=True)
    return run(job[0],job[1],_SOURCES,_CONTROLLER,_STOP_EVENT.is_set,_RECORD_HZ)

def initialize_worker():
    import signal
    # The parent owns cooperative stop. Pool's normal cleanup sends SIGTERM
    # to idle workers; that must not set the shared stop flag after success.
    signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_DFL)

def record_result(case_id,report,reference):
    """Merge evidence atomically; a late old process cannot erase a new run."""
    import fcntl
    from robofrontier.suite.scenes import CATALOG
    with CATALOG.with_suffix('.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        data=json.loads(CATALOG.read_text());case=next(c for c in data['cases'] if c['id']==case_id)
        entry={k:report.get(k) for k in ('passed','verification_seed','scene_sha256','scene_bundle_sha256','executor_sha256','physical_control_sha256','environment_logic_sha256','finished_at_unix_s','interrupted')};entry['report']=reference
        history=case.setdefault('oracle_attempts',[])
        if not any(v['report']==reference for v in history):history.append(entry)
        current_bundle=bundle_manifest(case_directory(case))['sha256']
        if entry['interrupted']:case['last_interrupted_reference']=reference
        eligible=[v for v in history if v['scene_bundle_sha256']==current_bundle and v['executor_sha256'] and not v.get('interrupted')]
        if eligible:
            latest=max(eligible,key=lambda v:v['finished_at_unix_s'] or 0);version=latest['executor_sha256'];results={}
            for v in sorted(eligible,key=lambda v:v['finished_at_unix_s'] or 0):
                if v['executor_sha256']==version:results[str(v['verification_seed'])]=v
            repeated=all(results.get(str(i),{}).get('passed') for i in (0,1,2))
            witnesses=[v for v in eligible if v['passed'] and v['verification_seed']==0]
            if 'verification_results' in case and 'legacy_verification_results' not in case:case['legacy_verification_results']=case['verification_results']
            case.update(status='repeatability_checked' if repeated else 'oracle_witness_passed' if witnesses else 'oracle_failed',verification_results=results,last_oracle_report=latest['report'],last_oracle_passed=latest['passed'])
            if witnesses:case['best_oracle_witness']=max(witnesses,key=lambda v:v['finished_at_unix_s'] or 0)['report']
        write_json(CATALOG,data)

def run(case,seed=0,sources=None,controller_cfg=None,stop_requested=None,record_hz=4.):
    parent=ROOT/'runs/oracle/frontier-v1'/case['id']/f'seed-{seed}';parent.mkdir(parents=True,exist_ok=True);attempt=1
    while (parent/f'attempt-{attempt:03d}').exists():attempt+=1
    output=parent/f'attempt-{attempt:03d}';t=Trial(case_directory(case),output,seed=seed,sources=sources,controller_cfg=controller_cfg,record_hz=record_hz);report=dict(passed=False,case_id=case['id'],verification_seed=seed,scope='Full task actuator/contact Oracle; not a GPT result',requested_record_hz=record_hz)
    t.stop_requested=stop_requested
    try:
        t.wait(.6);t.snapshot('initial')
        if case['family'] in ('transport','mosaic'):
            from robofrontier.suite.placement_oracle import transport,mosaic
            (transport if case['family']=='transport' else mosaic)(t)
        elif case['family']=='dial':
            from robofrontier.suite.dial_oracle import solve
            solve(t)
        else:
            from robofrontier.suite.relay_oracle import solve
            solve(t)
        result=t.monitor.result();report.update(result,passed=bool(result.get('task_success')));t.snapshot('final')
    except ReferenceStopped as error:
        report.update(interrupted=True,error=str(error),evaluation=t.monitor.result(),unresolved_robot_actions=[r for r,a in t.arms.items() if a.job is not None])
        t.snapshot('interrupted')
    except Exception as error:
        report.update(error=type(error).__name__+': '+str(error),evaluation=t.monitor.result())
        try:t.snapshot('failure')
        except Exception as snapshot_error:report['snapshot_error']=str(snapshot_error)
    finally:t.close(report)
    reference=str((output/'report.json').relative_to(ROOT));current=next(c for c in catalog()['cases'] if c['id']==case['id'])
    if current.get('scene_sha256')!=report['scene_sha256']:
        print(json.dumps(dict(case_id=case['id'],seed=seed,passed=report['passed'],report=reference,scope='Archived scene revision; current catalogue not overwritten')),flush=True);return report
    record_result(case['id'],report,reference)
    print(json.dumps(dict(case_id=case['id'],seed=seed,passed=report['passed'],error=report.get('error'),report=reference)),flush=True);return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',action='append');p.add_argument('--family',action='append',choices=('transport','mosaic','relay','dial'));p.add_argument('--all',action='store_true');p.add_argument('--seeds',type=int,nargs='+',default=[0]);p.add_argument('--jobs',type=int,choices=range(1,7),default=1);p.add_argument('--record-hz',type=int,choices=(4,15),default=4);a=p.parse_args()
    if not(a.case or a.family or a.all):p.error('Choose --case, --family or --all')
    cases=[c for c in catalog()['cases'] if a.all or c['id'] in (a.case or []) or c['family'] in (a.family or [])]
    known={c['id']:c for c in catalog()['cases']}
    for name in a.case or []:
        if name not in known:p.error('Unknown case: '+name)
    if a.case:cases=[known[name] for name in dict.fromkeys(a.case)]+[c for c in cases if c['id'] not in a.case]
    failed=[]
    import multiprocessing,signal
    _STOP_EVENT=multiprocessing.get_context('fork').Event()
    signal.signal(signal.SIGINT,lambda *_:_STOP_EVENT.set());signal.signal(signal.SIGTERM,lambda *_:_STOP_EVENT.set())
    # Import all reference executors before capturing their files. Later edits
    # in the engineering workspace must not be labeled as the code loaded by
    # this already-running batch. Each fresh process captures one version.
    paths=list((ROOT/'robofrontier').rglob('*.py'))+[Path(__file__).resolve(),ROOT/'scripts/verify_floating_dial_oracle.py',ROOT/'configs/harness.json']
    sources={str(path.relative_to(ROOT)):path.read_bytes() for path in paths};controller_cfg=load_harness()
    jobs=[(c,seed) for c in cases for seed in a.seeds]
    if a.jobs==1:
        for c,seed in jobs:
            if _STOP_EVENT.is_set():break
            if not run(c,seed,sources,controller_cfg,_STOP_EVENT.is_set,a.record_hz)['passed']:failed.append((c['id'],seed))
    else:
        # One pre-imported source snapshot for all children. No graphics
        # context is created in the parent before forking. Each child owns a
        # separate reference world; this does not create policy Agents.
        _SOURCES=sources;_CONTROLLER=controller_cfg;_RECORD_HZ=a.record_hz
        with multiprocessing.get_context('fork').Pool(a.jobs,initializer=initialize_worker) as pool:
            for report in pool.imap_unordered(parallel_run,jobs):
                if not report['passed'] and not report.get('skipped'):failed.append((report['case_id'],report['verification_seed']))
    if _STOP_EVENT.is_set():raise SystemExit('Reference batch stopped; partial worlds were saved and remaining jobs were not started')
    if failed:raise SystemExit('Failed task witnesses: '+str(failed))
