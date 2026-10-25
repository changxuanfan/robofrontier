"""Private zero-step differential timing; never a robot experiment."""
import argparse,json,time,statistics,hashlib
from pathlib import Path
from tests.test_mosaic_acceptance_equivalence import world,seat_all
from tests.fixtures.mosaic_scalar_reference import placement_candidates as scalar
from robofrontier.suite.evaluate_v2 import placement_candidates
import numpy as np


def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=Path('runs/engineering/evaluator-v2-vectorization-20261025/report.json'));a=p.parse_args();rows=[]
 for case in ('mosaic-a04-6x12-d1','mosaic-a08-12x24-d1'):
  mon=world(case)
  for state in ('initial','fully_seated'):
   if state=='fully_seated':seat_all(mon)
   pos=mon.d.xpos[mon.bodies].copy();stable=[True]*len(mon.names)
   np.testing.assert_array_equal(scalar(mon,pos,stable),placement_candidates(mon,pos,stable))
   timings={}
   for label,fn in [('scalar',scalar),('vectorized',placement_candidates)]:
    samples=[]
    for _ in range(9):
     start=time.perf_counter();fn(mon,pos,stable);samples.append(time.perf_counter()-start)
    timings[label]=statistics.median(samples)
   rows.append(dict(case_id=case,state=state,objects=len(mon.names),equivalent=True,median_ms={k:1000*v for k,v in timings.items()},speedup=timings['scalar']/timings['vectorized']))
 report=dict(scope='Only acceptance function CPU cost, zero physics integration; not full-task speedup',thresholds_changed=False,rows=rows,source_sha256=hashlib.sha256(Path('robofrontier/suite/evaluate_v2.py').read_bytes()).hexdigest())
 a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))


if __name__=='__main__':main()
