"""Offline recorded-state exports, two independent rendering processes."""
import concurrent.futures
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1];B=ROOT/'runs/oracle/dial-hold-blind-20261025';OUT=ROOT/'outputs/oracle/dial-hold-blind-20261025'


def render(report):
    r=json.loads(report.read_text());out=OUT/r['case_id']
    if (out/'manifest.json').exists():return dict(case_id=r['case_id'],ready=True,reused=True)
    env=os.environ.copy();env.update(PYTHONPATH=str(ROOT),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    out.parent.mkdir(parents=True,exist_ok=True)
    with (OUT/(r['case_id']+'-render.log')).open('w') as log:
        result=subprocess.run([sys.executable,str(ROOT/'scripts/render_frontier_oracle.py'),str(report.parent),'--speed','12','--output',str(out)],env=env,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
    return dict(case_id=r['case_id'],ready=result.returncode==0,error=None if result.returncode==0 else str(OUT/(r['case_id']+'-render.log')))


if __name__=='__main__':
    reports=[p for p in sorted(B.glob('dial-*/attempt-001/report.json')) if json.loads(p.read_text()).get('passed')]
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        rows=list(pool.map(render,reports))
    (OUT/'render-progress.json').write_text(json.dumps(rows,indent=2)+'\n');print(json.dumps(rows),flush=True)
