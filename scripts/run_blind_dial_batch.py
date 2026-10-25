"""Bounded independent physical worlds; no model decisions or answer parsing."""
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
B=ROOT/'runs/oracle/dial-hold-blind-20261025'
BOOK=B/'private/codebook.json'
CASES=[f'dial-n{n:02}-d{d}' for n in (4,6,8) for d in (1,2,3)]


def case_run(cid):
    out=B/cid/'attempt-001';report=out/'report.json'
    if report.exists():return dict(case_id=cid,phase='finished',report=str(report),**{k:v for k,v in json.loads(report.read_text()).items() if k in ('passed','error','wall_time_s')})
    if out.exists():raise RuntimeError('Unclosed episode must not be duplicated '+cid)
    env=os.environ.copy();env.update(PYTHONPATH=str(ROOT),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    cmd=[sys.executable,str(ROOT/'scripts/run_blind_dial_oracle.py'),'--case',cid,'--output',str(out),'--codebook',str(BOOK),'--wall-s','1200','--sim-s','1800']
    started=time.monotonic()
    with (B/(cid+'.log')).open('w') as log:
        process=subprocess.Popen(cmd,env=env,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        record=dict(case_id=cid,phase='running',pid=process.pid,started_unix_s=time.time(),output=str(out),wall_budget_s=1200)
        (B/(cid+'-process.json')).write_text(json.dumps(record,indent=2)+'\n')
        signalled=False
        while process.poll() is None:
            elapsed=time.monotonic()-started
            if ((B/'STOP').exists() or elapsed>1240) and not signalled:
                os.kill(process.pid,signal.SIGTERM);signalled=True
            if elapsed>1280:
                os.killpg(process.pid,signal.SIGKILL);process.wait();break
            time.sleep(.5)
    result=json.loads(report.read_text()) if report.exists() else {}
    record.update(phase='finished',return_code=process.returncode,passed=bool(result.get('passed')),error=result.get('error'),report=str(report) if report.exists() else None,wall_s=time.monotonic()-started)
    (B/(cid+'-process.json')).write_text(json.dumps(record,indent=2)+'\n')
    return record


if __name__=='__main__':
    B.mkdir(parents=True,exist_ok=True)
    commitment=json.loads((B/'private/commitment.json').read_text())
    assert hashlib.sha256(BOOK.read_bytes()).hexdigest()==commitment['sha256']
    results=[]
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
        pending={pool.submit(case_run,c):c for c in CASES}
        for task in concurrent.futures.as_completed(pending):
            try:row=task.result()
            except Exception as error:row=dict(case_id=pending[task],passed=False,error=type(error).__name__+': '+str(error))
            results.append(row);payload=dict(protocol='dial-continuous-grasp-v1',cases=results,completed=len(results),total=9,passed=sum(r.get('passed',False) for r in results),active=True)
            tmp=B/'batch-status.tmp';tmp.write_text(json.dumps(payload,indent=2)+'\n');tmp.replace(B/'batch-status.json');print(json.dumps(row),flush=True)
    payload['active']=False;(B/'batch-status.json').write_text(json.dumps(payload,indent=2)+'\n')
