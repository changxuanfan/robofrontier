"""Audit all source clocks, exact loop reset, appearance and rigid placement."""
import argparse
from pathlib import Path
from unittest.mock import patch

import numpy as np

from robofrontier.task_families import ROOT,RELEASE,read
from robofrontier.factory_playback import FactoryTimeline,current_scene
from robofrontier.run_setup import write_json


def check(scene):
    with patch('mujoco.mj_step',side_effect=AssertionError('Replay must not step physics')):
        timeline=FactoryTimeline(scene,read(ROOT/RELEASE))
        try:
            frames=[0,1,max(t.frame_count for t in timeline.tracks)+17,2,0]
            checked=[]
            for frame in frames:
                row=timeline.at(frame);errors=[]
                for track,sample in zip(timeline.tracks,row['stations']):
                    assert sample['local_frame']==frame%track.frame_count
                    assert sample['cycle']==frame//track.frame_count
                    assert sample['source_frame']==int(track.indices[sample['local_frame']])
                    errors.append(timeline.play.verify_station(track.spec['id'],track.source))
                checked.append(dict(frame=frame,stations=len(errors),
                    position_error_m=max(e['position_error_m']for e in errors),
                    rotation_matrix_error=max(e['rotation_matrix_error']for e in errors)))
            resets=[]
            for track in timeline.tracks:
                first=track.at(0);track.at(track.frame_count-1);restart=track.at(track.frame_count)
                np.testing.assert_array_equal(first['qpos'],restart['qpos'])
                assert first['rgba']==restart['rgba'] and restart['appearance_reset']
                resets.append(dict(case_id=track.spec['id'],output_frames=track.frame_count,
                    duration_s=track.frame_count/track.fps,source_hz=track.reader.meta['sample_hz'],
                    speedup=track.speed,exact_qpos_reset=True,appearance_reset=True))
            return dict(passed=True,physics_steps=0,fps=timeline.fps,frames=checked,loop_boundaries=resets,
                geometry_scope='Rigid placement relative to each configured FK display model; Transport table-only display changes listed separately',
                presentation_geometry={key:value['table']for key,value in timeline.play.presentations.items()if value['table']})
        finally:timeline.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',type=Path,default=current_scene())
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=check(args.scene);write_json(args.output,result)
    print('Verified',len(result['loop_boundaries']),'independent station clocks and resets')


if __name__=='__main__':main()
