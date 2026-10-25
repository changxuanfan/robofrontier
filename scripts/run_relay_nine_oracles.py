"""Frozen Relay case queue: at most two worlds, no default total deadlines.

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


def main():
    p=argparse.ArgumentParser();p.add_argument('--output-root',type=Path,required=True)
    p.add_argument('--workers',type=int,choices=(1,2),default=2);p.add_argument('--cases',nargs='*');a=p.parse_args()
    from robofrontier.run_setup import write_json
    from robofrontier.motion_recording import freeze_source,MotionReader
    from robofrontier.suite.source_snapshot import freeze_executor
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():raise RuntimeError('Commit the executor first')
    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    cases=[c for c in json.loads((ROOT/'configs/suites/frontier-v2.json').read_text())['cases'] if c['family']=='relay' and (not a.cases or c['id'] in a.cases)]
    if not cases or set(a.cases or [])-{c['id'] for c in cases}:raise ValueError('Unknown Relay case selection')
    order=['relay-a08-d1','relay-a08-d2','relay-a12-d1','relay-a16-d1','relay-a08-d3','relay-a12-d2','relay-a12-d3','relay-a16-d2','relay-a16-d3']
    cases.sort(key=lambda c:order.index(c['id']))
    base=a.output_root.resolve();base.mkdir(parents=True,exist_ok=False);(base/'inputs').mkdir()
    harness=ROOT/'runs/oracle/relay-key-units-20261025-213449/inputs/harness.json'
    (base/'inputs/harness.json').write_bytes(harness.read_bytes())
    for case in cases:
        destination=base/'scenes'/case['id'];destination.parent.mkdir(exist_ok=True)
        freeze_source(ROOT/case['directory'],destination,cache=base/'asset-cache')
        case['frozen_scene']=str(destination)
        task=json.loads((destination/'task.json').read_text())
        if not task['completion'].get('support_via_goal_payload'):raise ValueError('Stable-stack contract missing')
    write_json(base/'inputs/catalog.json',dict(cases=cases))
    frozen=freeze_executor(ROOT,base/'executor',base/'inputs/harness.json',base/'inputs/catalog.json')
    manifest=dict(git_commit=commit,worktree=str(ROOT),cases=cases,workers=a.workers,total_wall_deadline_s=None,
        task_rule_revision='relay-goal-contact-stack-20261026',scope='Nine full Relay Oracle cases only; no GPT trials',created_unix_s=time.time())
    write_json(base/'run.json',manifest);write_json(base/'manifest.json',manifest)
    pending=list(cases);active={};results=[]
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
