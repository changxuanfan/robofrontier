"""Frozen BEST5 blind batch, one pilot then at most two independent worlds.

No puzzle content is read by this supervisor. Secrets are passed as evaluator
file paths. The output root is a unique external artifact directory.
"""
import argparse
import concurrent.futures
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from robofrontier.run_setup import write_json

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--output-root',required=True,type=Path)
    p.add_argument('--workers',type=int,choices=(1,2),default=2);p.add_argument('--wall-s',type=float,default=0,help='Optional wall budget per case; 0 means no limit')
    p.add_argument('--cases',nargs='*');a=p.parse_args();base=a.output_root.resolve()
    if not math.isfinite(a.wall_s) or a.wall_s<0:p.error('Wall budget must be nonnegative; zero disables it')
    catalog=ROOT/'configs/suites/dial-best5-r11.json';cases=json.loads(catalog.read_text())['cases']
    ids=[c['id'] for c in cases if not a.cases or c['id'] in a.cases]
    if a.cases and set(ids)!=set(a.cases):raise ValueError('Unknown case')
    if not ids:raise ValueError('No cases selected')
    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():raise RuntimeError('Commit the run source before starting a batch')
    base.mkdir(parents=True,exist_ok=False);(base/'inputs').mkdir()
    harness=ROOT/'configs/harness-frontier-v2-servo2.json'
    (base/'inputs/harness.json').write_bytes(harness.read_bytes());(base/'inputs/catalog.json').write_bytes(catalog.read_bytes())
    write_json(base/'manifest.json',dict(protocol='dial-best5-continuous-grasp-repair-v2',git_commit=commit,worktree=str(ROOT),
        solver_password_input=False,mechanical_geometry_privileged=True,mechanical_warmup_in_episode=False,
        cases=ids,workers=a.workers,wall_budget_per_case_s=a.wall_s or None,sim_budget_per_case_s=None,
        catalog_sha256=hashlib.sha256(catalog.read_bytes()).hexdigest(),created_unix_s=time.time()))
    results=[]
    def status(active):
        write_json(base/'batch-status.json',dict(active=active,total=len(ids),completed=len(results),passed=sum(bool(r.get('passed')) for r in results),cases=results))
    def run(cid):
        out=base/cid/'attempt-001';out.parent.mkdir()
        private=ROOT/f'configs/evaluations/private/dial-best5-r11/{cid}.json'
        if not private.exists():raise FileNotFoundError('Missing evaluator-only puzzle file')
        cmd=[sys.executable,str(ROOT/'scripts/run_blind_dial_oracle.py'),'--case',cid,'--output',str(out),
            '--codebook',str(private),'--harness',str(base/'inputs/harness.json'),'--wall-s',str(a.wall_s)]
        env=os.environ.copy();env.update(PYTHONPATH=str(ROOT),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONUNBUFFERED='1')
        began=time.monotonic();record=dict(case_id=cid,phase='running',output=str(out),started_unix_s=time.time())
        with (base/(cid+'.log')).open('w') as log:
            process=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            record['pid']=process.pid;write_json(base/(cid+'-process.json'),record);signalled=False;signal_time=None
            while process.poll() is None:
                elapsed=time.monotonic()-began
                if ((base/'STOP').exists() or (a.wall_s>0 and elapsed>a.wall_s+40)) and not signalled:
                    os.kill(process.pid,signal.SIGTERM);signalled=True;signal_time=elapsed
                if signalled and elapsed-signal_time>35:
                    os.killpg(process.pid,signal.SIGKILL);process.wait();break
                time.sleep(.5)
        report=out/'report.json';r=json.loads(report.read_text()) if report.exists() else {}
        passed=bool(process.returncode==0 and r.get('passed') and r.get('evaluation',{}).get('task_success') and not r.get('unresolved_robot_actions'))
        record.update(phase='passed' if passed else 'failed',passed=passed,returncode=process.returncode,
            error=r.get('error') or (None if passed else 'Process/report did not establish a clean success'),
            report=str(report) if report.exists() else None,wall_s=time.monotonic()-began)
        write_json(base/(cid+'-process.json'),record);print(json.dumps(record),flush=True);return record
    status(True)
    pilot=run(ids[0]);results.append(pilot);status(True)
    if not pilot['passed']:
        write_json(base/'pilot-needs-review.json',pilot)
    with concurrent.futures.ThreadPoolExecutor(max_workers=a.workers) as pool:
        pending={pool.submit(run,c):c for c in ids[1:]}
        for task in concurrent.futures.as_completed(pending):
            try:row=task.result()
            except Exception as error:row=dict(case_id=pending[task],passed=False,error=type(error).__name__+': '+str(error))
            results.append(row);status(True)
    status(False);return 0 if all(r['passed'] for r in results) else 1


if __name__=='__main__':raise SystemExit(main())
