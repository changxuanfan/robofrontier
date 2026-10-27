"""Render selected closed dial episodes in both backgrounds with shared edits."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys

from robofrontier.motion_edit import EditedMotion, checksum
from robofrontier.run_setup import write_json


def run(selection, edits, factory, output, speed=10., fps=30.):
    cases = json.loads(Path(selection).read_text())['cases']
    if any(not c['case_id'].startswith('dial-') for c in cases):
        raise ValueError('This batch renderer is scoped to dial tasks only')
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    results = []
    for case in cases:
        case_id = case['case_id']
        recording = Path(case['report']).parent/'motion'
        edit = Path(edits).resolve()/(case_id+'.json')
        target = output/case_id
        schedule = EditedMotion(recording, edit).video_indices(fps, speed).tolist()
        report_path = target/'report.json'
        if not report_path.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            with (output/(case_id+'.log')).open('x') as log:
                subprocess.run([sys.executable,'-u','-m','scripts.replay_motion_in_factory',str(recording),
                    '--factory',str(Path(factory).resolve()),'--output',str(target),'--edit',str(edit),
                    '--render','--speed',str(speed),'--fps',str(fps)], stdout=log, stderr=subprocess.STDOUT, check=True)
        report = json.loads(report_path.read_text())
        if (not report['passed'] or report['edit_sha256'] != checksum(edit)
                or report['video_source_frame_indices'] != schedule
                or report['factory_scene_sha256'] != checksum(factory)
                or report['speedup'] != speed or report['fps'] != fps
                or any(not v['full_decode_passed'] or v['frames'] != len(schedule) or v['dropped_frames'] for v in report['videos'].values())):
            raise ValueError('Incomplete or differently edited render: '+case_id)
        name = f"{case['dials']:02d}-dials-L{case['level']}"+('-assisted' if case['assisted'] else '')+'.mp4'
        decoded_frame_counts = {}
        for background in ('simple','factory'):
            folder = output/(background+'-videos')
            folder.mkdir(exist_ok=True)
            destination = folder/name
            source = target/(background+'.mp4')
            info = json.loads(subprocess.check_output(['ffprobe','-v','error','-count_frames',
                '-select_streams','v:0','-show_entries','stream=nb_read_frames','-of','json',str(source)]))
            actual_frames = int(info['streams'][0]['nb_read_frames'])
            if actual_frames != len(schedule):
                raise ValueError('Decoded frame count differs from shared source schedule')
            decoded_frame_counts[background] = actual_frames
            if destination.exists():
                if checksum(destination) != checksum(source):
                    raise ValueError('Conflicting existing published video')
            else:
                shutil.copy2(source, destination)
        results.append(dict(case_id=case_id, edit=str(edit), report=str(report_path),
            output_frames=len(schedule), shared_source_schedule_verified=True,
            decoded_frame_counts=decoded_frame_counts,
            source_duration_s=EditedMotion(recording, edit).edit['source_duration_s'],
            edited_duration_s=EditedMotion(recording, edit).edit['edited_duration_s']))
        write_json(output/'batch.json', dict(completed=len(results), total=len(cases), cases=results))
        print(case_id, 'paired render verified', len(schedule), 'frames', flush=True)
    return results


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--selection', type=Path, required=True)
    parser.add_argument('--edits', type=Path, required=True)
    parser.add_argument('--factory', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--speed', type=float, default=10.)
    parser.add_argument('--fps', type=float, default=30.)
    args = parser.parse_args()
    run(args.selection,args.edits,args.factory,args.output,args.speed,args.fps)
