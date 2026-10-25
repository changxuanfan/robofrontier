"""Generate selected suite assets. This does not run an Oracle or a model."""
import argparse
from robofrontier.suite.scenes import catalog,build_case

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',action='append');p.add_argument('--family',action='append',choices=('transport','mosaic','relay','dial'));p.add_argument('--all',action='store_true');p.add_argument('--rebuild',action='store_true');p.add_argument('--no-render',action='store_true');a=p.parse_args()
    if not (a.case or a.family or a.all):p.error('Select --case, --family or --all')
    cases=[c for c in catalog()['cases'] if (a.all or c['id'] in (a.case or []) or c['family'] in (a.family or []))]
    for c in cases:build_case(c,a.rebuild,not a.no_render)
