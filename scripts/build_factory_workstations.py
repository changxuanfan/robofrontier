"""Compose frozen workstations for display; never run physics or an Oracle."""
import argparse,json
from pathlib import Path
from robofrontier.factory_workstations import compose

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--shell',type=Path,required=True);p.add_argument('--catalog',type=Path,default=Path('configs/suites/frontier-current-oracle.json'));p.add_argument('--output',type=Path,required=True)
    p.add_argument('--timeline-dir',type=Path,help='Use each edit timeline recording/source, preserving its exact frozen scene revision')
    p.add_argument('--source-root',type=Path,help='Root for catalog sources without recording overrides')
    p.add_argument('--table-opening-scale',type=float,help='Explicit override; default uses each source catalog entry')
    p.add_argument('--placements',type=Path,help='Rigid translation/yaw overrides')
    a=p.parse_args();overrides={}
    if a.timeline_dir:
        for path in sorted(a.timeline_dir.glob('*.json')):
            meta=json.loads(path.read_text())
            if meta.get('schema')!='robofrontier.motion-edit.v1':continue
            overrides[meta['case_id']]=Path(meta['recording'])/'source'
        if not overrides:raise ValueError('No motion-edit timelines found')
    print(compose(a.shell,a.catalog,a.output,a.table_opening_scale,overrides,a.source_root,placement_overrides=json.loads(a.placements.read_text()) if a.placements else None))
