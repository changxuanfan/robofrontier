"""Audit the dial family and recorded source cursors, without a runtime or physics."""
import argparse
from pathlib import Path

import numpy as np

from robofrontier.motion_recording import MotionReader
from robofrontier.run_setup import write_json
from robofrontier.suite.dial_policy_package import (
    REGISTRY, build_review, read, registry, verify_review,
)
from robofrontier.suite.dial_motion_edit import build_dial_edit, DialReplayTrack


def check(root, output):
    root,output = Path(root),Path(output)
    registered = read(root/REGISTRY)
    if registry(root)!=registered:raise ValueError('Evidence registry changed')
    output.mkdir(parents=True,exist_ok=False)
    build_review(root,output/'policy-review',registered)
    review = verify_review(root,output/'policy-review')
    rows = []
    for case in registered['cases']:
        recording = root/case['oracle_evidence']['recording']
        edit = build_dial_edit(recording)
        edit_path = output/'motion-edits'/(case['case_id']+'.json')
        write_json(edit_path,edit)
        with DialReplayTrack(recording,edit_path) as track:
            position = 0
            for source_index,(stamp,qpos) in enumerate(MotionReader(recording).frames()):
                if source_index != track.indices[position]:continue
                sample = track.at(position)
                if sample['source_time_s'] != stamp or not np.array_equal(sample['qpos'],qpos):
                    raise ValueError('Track differs from original source states')
                position += 1
                if position==len(track.indices):break
            if position!=len(track.indices):raise ValueError('Incomplete output source schedule')
            for source_index in edit['appearance_frames']:
                if source_index not in track.indices:raise ValueError('Semantic transition omitted')
            for span in edit['removed_spans']:
                if np.any((track.indices>=span['first_frame']) & (track.indices<span['stop_frame'])):
                    raise ValueError('Removed interval leaked into output')
            rows.append(dict(case_id=case['case_id'],source_frames=edit['source_frames'],
                source_samples_checked=position,source_state_equal=True,
                removed_source_s=edit['removed_frames']/edit['sample_hz'],
                removed_oscillation_s=edit['dial_profile']['removed_oscillation_frames']/edit['sample_hz'],
                source_evidence_complete=edit['dial_profile']['source_evidence_complete'],
                semantic_event_count=edit['dial_profile']['semantic_events_protected'],
                assisted=case['oracle_evidence']['assisted']))
        write_json(output/'status.json',dict(completed=len(rows),total=9,cases=rows))
        print(case['case_id'],position,'source samples verified',flush=True)
    if registry(root)!=registered:raise ValueError('Sources changed during the audit')
    report = dict(family='dial',cases=rows,public_review=review,formal_policy_ready=False,
        gpt_started=False,oracle_rerun=False,source_evidence_unchanged=True,
        shared_harness_modified=False,shared_compositor_implemented=False,
        removed_source_s=sum(r['removed_source_s'] for r in rows),
        measured_force_available=False,
        integration_requests='configs/task_families/dial/integration-requests.json')
    write_json(output/'summary.json',report)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    check(Path(__file__).resolve().parents[1],args.output)
