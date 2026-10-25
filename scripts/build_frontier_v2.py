"""Build simple v2 scenes only. Never starts an Oracle, policy or factory."""
import argparse
from robofrontier.suite.v2 import cases,build,CATALOG
from robofrontier.run_setup import write_json

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',action='append');p.add_argument('--family',choices=('transport','mosaic','relay','dial'));p.add_argument('--all',action='store_true');p.add_argument('--replace-draft',action='store_true');p.add_argument('--no-render',action='store_true');a=p.parse_args()
    if not(a.case or a.family or a.all):p.error('Select a case, family or all')
    data=cases();known={c['id'] for c in data}
    if set(a.case or [])-known:p.error('Unknown case')
    chosen=[c for c in data if a.all or c['id'] in (a.case or []) or c['family']==a.family]
    for c in chosen:build(c,a.replace_draft,not a.no_render)
    write_json(CATALOG,dict(schema_version=2,case_count=36,status='Environment build only; no Oracle/policy commissioning',cases=data))
