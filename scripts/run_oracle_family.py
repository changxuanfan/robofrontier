"""One family, serial isolated worlds, outer wall timeout and fail-fast reports."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from robofrontier.run_setup import write_json
from robofrontier.suite.v2 import ROOT


def main():
    p=argparse.ArgumentParser();p.add_argument('--family',required=True,choices=('transport','mosaic','relay','dial'))
    p.add_argument('--batch',type=Path,default=ROOT/'runs/oracle/suite-v2-r10-20261025')
    p.add_argument('--case',action='append');p.add_argument('--wall-budget-s',type=float,default=900)
    p.add_argument('--sim-budget-s',type=float,default=3600);p.add_argument('--action-budget',type=int,default=20000)
    p.add_argument('--summary-name',default='family-report.json');p.add_argument('--max-goals',type=int);p.add_argument('--continue-after-failure',action='store_true');a=p.parse_args()
    if Path(a.summary_name).name!=a.summary_name or Path(a.summary_name).suffix!='.json':raise SystemExit('Summary name must be a JSON filename')
    batch=json.loads((a.batch/'batch.json').read_text())
    if not batch.get('execution_ready'):raise SystemExit('Batch execution gate is closed')
    if not batch.get('family_execution_ready',{}).get(a.family,True):raise SystemExit('Family reference is not ready')
    catalog=ROOT/batch['catalog'];all_cases=json.loads(catalog.read_text())['cases']
    selected=[c for c in all_cases if c['family']==a.family and (not a.case or c['id'] in a.case)]
    if not selected or set(a.case or [])-{c['id'] for c in selected}:raise SystemExit('Unknown/empty family selection')
    out=a.batch/a.family;out.mkdir(exist_ok=True);rows=[]
    for case in selected:
        case_dir=out/case['id'];case_dir.mkdir(exist_ok=True)
        stamp=str(time.time_ns());log=case_dir/('stdout-'+stamp+'.log');err=case_dir/('stderr-'+stamp+'.log')
        cmd=[sys.executable,str(ROOT/'scripts/verify_frontier_v2.py'),'--execute','--case',case['id'],
             '--catalog',str(catalog),'--harness',str(ROOT/batch.get('controller_config','configs/harness-frontier-v2.json')),'--output-root',str(out),'--wall-budget-s',str(a.wall_budget_s),
             '--sim-budget-s',str(a.sim_budget_s),'--action-budget',str(a.action_budget)]
        if a.max_goals is not None:cmd+=['--max-goals',str(a.max_goals)]
        env=os.environ.copy();env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
        before={str(f) for f in case_dir.rglob('report.json')};started=time.monotonic();killed=False;stop_at=None
        with log.open('w') as stdout,err.open('w') as stderr:
            process=subprocess.Popen(cmd,cwd=ROOT,stdout=stdout,stderr=stderr,env=env)
            write_json(case_dir/'process.json',dict(pid=process.pid,phase='running',started_unix_s=time.time(),wall_budget_s=a.wall_budget_s))
            while process.poll() is None:
                if (out/'STOP').exists() and stop_at is None:process.terminate();stop_at=time.monotonic()
                if stop_at is not None and time.monotonic()-stop_at>10:
                    process.kill();process.wait();killed=True;break
                if time.monotonic()-started>a.wall_budget_s+30:
                    process.terminate()
                    try:process.wait(timeout=10)
                    except subprocess.TimeoutExpired:process.kill();process.wait()
                    killed=True;break
                time.sleep(.5)
        reports=[f for f in case_dir.rglob('report.json') if str(f) not in before]
        path=max(reports,key=lambda f:f.stat().st_mtime) if reports else None
        result=json.loads(path.read_text()) if path else {}
        commissioning=bool(result.get('reference_result',{}).get('commissioning_only'))
        passed=bool(result.get('passed'));partial_ok=commissioning and not result.get('error')
        row=dict(case_id=case['id'],passed=passed,commissioning_only=commissioning,commissioning_ok=partial_ok,
                 hard_timeout=killed,return_code=process.returncode,wall_s=time.monotonic()-started,
                 report=str(path) if path else None,error=result.get('error'),stdout=str(log),stderr=str(err))
        rows.append(row);write_json(out/a.summary_name,dict(family=a.family,status='running',scope='selected_cases_only',selected_count=len(selected),family_case_count=sum(c['family']==a.family for c in all_cases),cases=rows,oracle=True))
        write_json(case_dir/'process.json',dict(pid=process.pid,phase='finished' if not killed else 'hard_timeout',return_code=process.returncode,unresolved_outcomes='unknown' if killed or (path is None and process.returncode!=0) else None))
        print(json.dumps(row),flush=True)
        if not passed and not partial_ok and not a.continue_after_failure:break
    write_json(out/a.summary_name,dict(family=a.family,scope='selected_cases_only',selected_count=len(selected),family_case_count=sum(c['family']==a.family for c in all_cases),status='complete' if len(rows)==len(selected) else 'stopped_on_failure',
        passed=all(r['passed'] for r in rows) and len(rows)==len(selected),cases=rows,oracle=True))


if __name__=='__main__':main()
