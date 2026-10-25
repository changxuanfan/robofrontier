"""Select a new development revision while preserving old scene/evidence paths."""
import argparse
from copy import deepcopy
from robofrontier.suite.scenes import ROOT,catalog,case_directory,update_case

def revise(case,reason):
    history=deepcopy(case.get('history',[]));history.append({k:deepcopy(case[k]) for k in ('directory','scene_revision','scene_sha256','status','verification_results','last_oracle_report') if k in case});history[-1]['superseded_reason']=reason
    revision=case.get('scene_revision',1)+1;new={**case,'scene_revision':revision}
    update_case(case['id'],scene_revision=revision,directory=str(case_directory(new).relative_to(ROOT)),history=history,status='planned',verification_results={},scene_sha256=None,last_oracle_report=None)
    print(case['id'],'revision',revision,flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',action='append');p.add_argument('--family',choices=('dial','transport','mosaic','relay'));p.add_argument('--reason',required=True);a=p.parse_args()
    if not(a.case or a.family):p.error('Choose cases or family')
    for case in catalog()['cases']:
        if case['id'] in (a.case or []) or case['family']==a.family:revise(case,a.reason)
