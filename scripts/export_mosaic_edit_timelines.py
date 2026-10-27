"""Export portable exact-frame and full-recording edit maps for all nine cases."""
import argparse
import json
import os
from pathlib import Path

import numpy as np

from robofrontier.motion_edit import build_timeline, MotionEdit
from robofrontier.motion_recording import MotionReader, digest


def export(folder, output):
    folder = Path(folder).resolve()
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    index = json.loads((folder / 'video-index.json').read_text())
    if index['variant'] != 'visible_idle_removed' or index['ready'] != 9:
        raise ValueError('Require the reviewed nine visible-idle edits')
    rows = []
    for row in index['videos']:
        reader = MotionReader(Path(row['source_trial']) / 'motion')
        if row.get('motion_edit_timeline'):
            previous = MotionEdit(row['motion_edit_timeline'], reader)
            edit = dict(previous.data)
            edit['derived_from_timeline_sha256'] = digest(row['motion_edit_timeline'])
        else:
            mask_file = Path(row['work_directory']) / 'motion-mask.npz'
            with np.load(mask_file, allow_pickle=False) as mask:
                edit = build_timeline(reader, mask['source_time_s'],
                                      mask['video_source_indices'], mask['video_keep'],
                                      fps=row.get('fps',30),speedup=row['speedup'])
            edit['derived_from_mask_sha256'] = digest(mask_file)
        if (edit['output_frames'] != row['kept_frames']
                or edit['nominal_speedup'] != row['speedup']):
            raise ValueError('Exported edit differs from reviewed video/cadence')
        edit.update(derived_from_reviewed_video_sha256=row['sha256'],
                    source_recording_hint=os.path.relpath(reader.path.resolve(), output))
        target = output / (row['case_id'] + '.json')
        target.write_text(json.dumps(edit, ensure_ascii=False, indent=2) + '\n')
        MotionEdit(target, reader)
        rows.append(dict(case_id=row['case_id'], label=row['label'],
                         timeline=target.name, sha256=digest(target),
                         output_frames=edit['output_frames'],
                         source_frames=edit['source_frames'],
                         output_duration_s=edit['output_duration_s'],
                         keep_source_ranges=len(edit['keep_source_ranges'])))
        print('TIMELINE_READY', row['label'], edit['output_frames'], flush=True)
    report = dict(schema='robofrontier.motion-edit-batch.v1', ready=len(rows), total=9,
                  fps=edit['fps'], nominal_speedup=edit['nominal_speedup'], original_recordings_unchanged=True,
                  background_independent=True, timelines=rows)
    (output / 'index.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--folder', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    export(a.folder, a.output)
