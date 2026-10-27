"""Audit Mosaic-only candidate waits/oscillations without editing or replaying."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
from robofrontier.suite.mosaic_motion_review import audit_case
from robofrontier.suite.mosaic_policy_package import ROOT,REGISTRY,read,write,verify_registry


def run(output,workers=3):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    registry,_=verify_registry()
    def one(case):
        report=audit_case(ROOT/case['oracle']['trial'],output/(case['case_id']+'.json'))
        counts={}
        for row in report['results']:counts[row['label']]=counts.get(row['label'],0)+1
        print('MOSAIC_REVIEW',case['case_id'],counts,flush=True)
        return dict(case_id=case['case_id'],report=case['case_id']+'.json',labels=counts,
                    candidate_source_frames_checked=report['candidate_source_frames_checked'])
    with ThreadPoolExecutor(max_workers=workers) as pool:cases=list(pool.map(one,registry['cases']))
    write(output/'index.json',dict(cases=cases,total=9,edits_applied=False,policy_input=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--workers',type=int,choices=range(1,4),default=3);a=p.parse_args();run(a.output,a.workers)
