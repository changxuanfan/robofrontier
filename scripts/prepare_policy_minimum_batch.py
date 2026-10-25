"""Prepare12 bounded fresh GPT inputs; never starts robots or inference."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

from robofrontier.policy_probe import PROBE_READINESS,protocol
from robofrontier.run_setup import prepare_run,write_json
from robofrontier.suite.v2 import ROOT


def prepare(batch):
    batch=Path(batch).resolve();batch.mkdir(parents=True,exist_ok=False)
    catalog=ROOT/'configs/suites/frontier-v2-r10.json';harness=ROOT/'configs/harness-frontier-v2-servo2.json'
    rows=[];all_cases=json.loads(catalog.read_text())['cases']
    for family in ('transport','mosaic','relay','dial'):
        for level in (1,2,3):
            case=min((c for c in all_cases if c['family']==family and c['level']==level),key=lambda c:c['arms'])
            source=ROOT/case['directory']/'task.json';task=deepcopy(json.loads(source.read_text()))
            task.update(execution_readiness=PROBE_READINESS,oracle_verified=False,run_protocol=protocol(1200.))
            task_path=batch/'inputs'/f"{case['id']}.json";write_json(task_path,task)
            run=batch/case['id'];prepare_run(run,task_path,harness)
            rows.append(dict(case_id=case['id'],family=family,level=level,arms=case['arms'],
                run_dir=str(run),source_task=str(source),source_task_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                status='prepared',thread_id=None,model='gpt-6.1-sol',effort='high',wall_limit_s=1200))
    result=dict(schema='robofrontier.bounded-policy-batch.v1',purpose='Explicit user-authorized limited GPT trials; not Oracle readiness certification',
        topology='adaptive',reuse_oracle_experience=False,clock='Each START begins1200 wall seconds; braking/closure reported separately',
        catalog=str(catalog),harness=str(harness),cases=rows)
    write_json(batch/'batch.json',result)
    print(json.dumps(dict(batch=str(batch),cases=len(rows),all_oracle_verified=False)))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('batch',type=Path);a=p.parse_args();prepare(a.batch)
