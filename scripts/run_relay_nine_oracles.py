"""Frozen Relay case queue with explicit concurrency and no total deadlines.

Recoverable failures keep their world alive in the existing owner interface.
The supervisor neither plans robot work nor kills slow cases. A failed closed
episode is retained for review; only native full-task success counts as passed.
"""
import argparse,hashlib,json,os,subprocess,sys,time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]


def full_task_passed(report,returncode):
    result=report.get('evaluation',{})
    return bool(returncode==0 and report.get('passed') and not report.get('commissioning_only')
        and result.get('task_success') and result.get('completed_payloads')==result.get('total_payloads')
        and result.get('total_payloads',0)>0 and not result.get('violations') and not report.get('unresolved_robot_actions'))


def reusable_l1_records(base,cases,root=ROOT):
    """Import only native verified L1 episodes whose current task is byte-identical."""
    status=json.loads((Path(base)/'batch-status.json').read_text());records={r['case_id']:r for r in status['results']};out=[]
    for case in cases:
        if case['level']!=1:continue
        record=records.get(case['id'])
        if not record or not record.get('passed'):raise ValueError('Missing verified L1 result '+case['id'])
        report_path=Path(record['report']);report=json.loads(report_path.read_text());run=report_path.parent
        if not full_task_passed(report,record.get('returncode')):raise ValueError('L1 native acceptance/exit incomplete '+case['id'])
        task_hash=hashlib.sha256((Path(root)/case['directory']/'task.json').read_bytes()).hexdigest()
        if task_hash!=report.get('task_contract_sha256') or hashlib.sha256((run/'task.json').read_bytes()).hexdigest()!=task_hash:
            raise ValueError('L1 source contract changed; cannot reuse '+case['id'])
        proof=json.loads((run/'RECORDING-VERIFIED.json').read_text());stream=json.loads((run/'motion/stream.json').read_text())
        if proof.get('status')!='complete' or stream.get('status')!='complete' or proof.get('frames_verified',0)<=0 or proof['frames_verified']!=stream['frames']:
            raise ValueError('L1 complete motion verification missing '+case['id'])
        out.append(dict(record,imported=True,imported_from=str(base),task_contract_sha256=task_hash,scope='Unchanged L1 native full success and complete recording retained'))
    return out


def main():
    p=argparse.ArgumentParser();p.add_argument('--output-root',type=Path,required=True)
    p.add_argument('--workers',type=int,choices=range(1,10),default=2);p.add_argument('--cases',nargs='*')
    p.add_argument('--reuse-l1-from',type=Path,help='Retain verified L1 results only if current contracts are byte-identical');a=p.parse_args()
    from robofrontier.run_setup import write_json
    from robofrontier.motion_recording import freeze_source,MotionReader
    from robofrontier.suite.source_snapshot import freeze_executor
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():raise RuntimeError('Commit the executor first')
    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    cases=[c for c in json.loads((ROOT/'configs/suites/frontier-v2.json').read_text())['cases'] if c['family']=='relay' and (not a.cases or c['id'] in a.cases)]
    if not cases or set(a.cases or [])-{c['id'] for c in cases}:raise ValueError('Unknown Relay case selection')
    order=['relay-a08-d1','relay-a08-d2','relay-a12-d1','relay-a16-d1','relay-a08-d3','relay-a12-d2','relay-a12-d3','relay-a16-d2','relay-a16-d3']
    cases.sort(key=lambda c:order.index(c['id']))
    retained=reusable_l1_records(a.reuse_l1_from,cases) if a.reuse_l1_from else []
    base=a.output_root.resolve();base.mkdir(parents=True,exist_ok=False);(base/'inputs').mkdir()
    harness=ROOT/'runs/oracle/relay-key-units-20261025-213449/inputs/harness.json'
    (base/'inputs/harness.json').write_bytes(harness.read_bytes())
    for case in cases:
        destination=base/'scenes'/case['id'];destination.parent.mkdir(exist_ok=True)
        freeze_source(ROOT/case['directory'],destination,cache=base/'asset-cache')
        if case.get('scene_revision')==16:
            (ROOT/case['directory']/'FROZEN').write_text(str(base)+'\n')
        case['frozen_scene']=str(destination)
        task=json.loads((destination/'task.json').read_text())
        if not task['completion'].get('support_via_goal_payload'):raise ValueError('Stable-stack contract missing')
    write_json(base/'inputs/catalog.json',dict(cases=cases))
    frozen=freeze_executor(ROOT,base/'executor',base/'inputs/harness.json',base/'inputs/catalog.json')
    manifest=dict(git_commit=commit,worktree=str(ROOT),cases=cases,workers=a.workers,total_wall_deadline_s=None,
        task_rule_revision='relay-goal-contact-stack-20261026',scope='Nine full Relay Oracle cases only; no GPT trials',retained_l1=retained,created_unix_s=time.time())
    write_json(base/'run.json',manifest);write_json(base/'manifest.json',manifest)
    retained_ids={r['case_id'] for r in retained}
    pending=[c for c in cases if c['id'] not in retained_ids];active={};results=list(retained)
    def status():
        rows=[]
        for cid,record in active.items():
            out=Path(record['output']);latest={}
            if (out/'status.json').exists():latest=json.loads((out/'status.json').read_text())
            rows.append(dict(case_id=cid,pid=record['process'].pid,output=str(out),latest=latest))
        state='running' if active or pending else 'complete' if all(r['passed'] for r in results) else 'needs_review'
        write_json(base/'batch-status.json',dict(state=state,passed=sum(r['passed'] for r in results),total=len(cases),active=rows,pending=[c['id'] for c in pending],results=results,updated_unix_s=time.time(),total_wall_deadline_s=None))
    while pending or active:
        while pending and len(active)<a.workers:
            case=pending.pop(0);cid=case['id'];out=base/cid/'attempt-001';out.parent.mkdir()
            env=os.environ.copy();env.update(PYTHONPATH=str(frozen),ROBOFRONTIER_ORACLE_FROZEN_ROOT=str(frozen),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONUNBUFFERED='1')
            cmd=[sys.executable,str(frozen/'scripts/run_relay_live_oracle.py'),'--case',cid,'--output',str(out),'--scene',case['frozen_scene'],'--harness',str(frozen/'inputs/harness.json'),'--wall-s','0','--pipeline','--frozen']
            if case['arms']>8 or case['level']>1:cmd.append('--flexible')
            log=(out.parent/'stdout.log').open('x')
            process=subprocess.Popen(cmd,cwd=ROOT,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            active[cid]=dict(process=process,log=log,output=str(out),command=cmd)
            write_json(out.parent/'process.json',dict(pid=process.pid,command=cmd,git_commit=commit,total_wall_deadline_s=None,started_unix_s=time.time()))
            print(json.dumps(dict(event='started',case_id=cid,pid=process.pid,output=str(out))),flush=True)
        for cid,record in list(active.items()):
            process=record['process'];code=process.poll()
            if code is None:continue
            record['log'].close();out=Path(record['output']);path=out/'report.json'
            report=json.loads(path.read_text()) if path.exists() else {}
            passed=full_task_passed(report,code);frames=None
            if passed:
                reader=MotionReader(out/'motion');frames=sum(1 for _ in reader.frames())
                if reader.meta['status']!='complete':passed=False
            row=dict(case_id=cid,passed=passed,returncode=code,report=str(path) if path.exists() else None,error=report.get('error'),frames_verified=frames,execution_mode=report.get('execution_mode'),git_commit=commit)
            results.append(row);write_json(out.parent/'result-index.json',row);active.pop(cid)
            print(json.dumps(dict(event='finished',**row)),flush=True)
        status()
        if active:time.sleep(2.)
    status()
    return 0 if all(r['passed'] for r in results) else 1


if __name__=='__main__':raise SystemExit(main())
