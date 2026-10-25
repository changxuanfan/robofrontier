"""Read-only integrity and time coverage audit of full successful Oracle streams."""
import argparse
import json
from pathlib import Path
import numpy as np
from robofrontier.motion_recording import MotionReader
from robofrontier.run_setup import write_json
from scripts.summarize_oracle_batch import summarize


def check(recording):
    reader=MotionReader(recording);meta=reader.meta;count=0;previous=None;first=None;max_gap=0.
    for stamp,state in reader.frames():
        count+=1
        if first is None:first=stamp
        if previous is not None:max_gap=max(max_gap,stamp-previous)
        previous=stamp
    limit=np.ceil(1./meta['sample_hz']/meta['physics_timestep_s'])*meta['physics_timestep_s']+1e-7
    source=json.loads((reader.path/'source/workstation.json').read_text());ids={g['id'] for g in source['geometries']}
    event_count=0;appearance=0;last_event=-float('inf')
    for event in reader.events():
        stamp=event['sim_time_s']
        if not np.isfinite(stamp) or stamp<last_event-1e-9:raise ValueError('Non-monotonic visual event stream')
        last_event=stamp;event_count+=1
        if event['event']=='rgba':
            appearance+=1
            if set(event['geoms'])-ids:raise ValueError('Appearance event references unknown geometry')
            if not all(np.asarray(v).shape==(4,) and np.isfinite(v).all() for v in event['geoms'].values()):raise ValueError('Invalid appearance event')
    errors=[]
    if meta['status']!='complete':errors.append('Recording is not complete')
    if count!=meta['frames']:errors.append('Frame count does not match committed index')
    if count==0 or first>1e-9:errors.append('Initial state is missing')
    if max_gap>limit:errors.append('Recorded sample gap exceeds rate/physics quantization')
    if last_event>(previous or 0)+1e-7:errors.append('Events extend beyond final stored state')
    return dict(recording=str(recording),passed=not errors,errors=errors,frames=count,initial_sim_s=first,last_sim_s=previous,
        max_sample_gap_s=max_gap,permitted_gap_s=limit,events=event_count,appearance_events=appearance,
        compressed_chunk_bytes=sum(c['bytes'] for c in meta['chunks']),source_scene_sha256=meta['source_scene_sha256'],
        scope='All committed state chunks, checksums, timestamps and visual events; no physics restart claim')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('batch',type=Path);a=p.parse_args()
    rows=[]
    for case in summarize(a.batch)['cases']:
        if not case['has_full_success']:continue
        path=Path(case['successful_reports'][-1]);row=check(path.parent/'motion');row['case_id']=case['case_id'];rows.append(row)
    write_json(a.batch/'motion-audit.json',dict(passed=all(x['passed'] for x in rows),audited_full_cases=len(rows),cases=rows))
    print(json.dumps(dict(audited_full_cases=len(rows),passed=all(x['passed'] for x in rows),frames=sum(x['frames'] for x in rows))))
    if any(not x['passed'] for x in rows):raise SystemExit(1)
