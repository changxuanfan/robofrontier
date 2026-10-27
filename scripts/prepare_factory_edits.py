"""Produce a new audited presentation release without altering historical edits."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from pathlib import Path

from robofrontier.task_families import ROOT,RELEASE,read,sha
from robofrontier.station_edit import build
from robofrontier.run_setup import write_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--workers',type=int,choices=range(1,5),default=2)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    release=deepcopy(read(ROOT/RELEASE))
    work=[(name,f,row) for name,f in release['families'].items() for row in f['oracle']]
    def one(entry):
        name,family,row=entry;destination=args.output/row['id']
        row.setdefault('historical_edit',row['edit']);row.setdefault('historical_edit_sha256',row['edit_sha256'])
        original=dict(row,edit=row['historical_edit'],edit_sha256=row['historical_edit_sha256'])
        result=build(original,destination)
        row['edit']=str((destination/'edit.json').resolve());row['edit_sha256']=sha(row['edit'])
        row.pop('derived_track',None)
        print(row['id'],'additional removed states',result['additional_removed_frames'],flush=True)
        return dict(case_id=row['id'],**{k:result[k]for k in ('retained_frames','removed_frames','additional_removed_frames','restored_event_context_frames')})
    with ThreadPoolExecutor(max_workers=args.workers)as pool:results=list(pool.map(one,work))
    write_json(args.output/'release.json',release)
    write_json(args.output/'summary.json',dict(cases=results,physics_steps=0,original_edits_overwritten=False))


if __name__=='__main__':main()
