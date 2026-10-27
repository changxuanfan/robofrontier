from copy import deepcopy
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from robofrontier.task_families import ROOT,RELEASE,read
from robofrontier.factory_playback import StationTrack,FactoryTimeline,current_scene
from robofrontier.factory_cameras import shots,camera_at


class FactoryTimelineTests(unittest.TestCase):
    def test_each_producer_rewinds_exact_states_and_resets_appearance(self):
        release=read(ROOT/RELEASE)
        with patch('mujoco.mj_step',side_effect=AssertionError('No replay physics')):
            for family in release['families'].values():
                track=StationTrack(family['oracle'][0],family['speedup'])
                try:
                    first=track.at(0)
                    self.assertTrue(first['appearance_reset'])
                    end=track.at(track.frame_count-1)
                    restart=track.at(track.frame_count)
                    self.assertEqual(restart['cycle'],1)
                    self.assertTrue(restart['appearance_reset'])
                    np.testing.assert_array_equal(first['qpos'],restart['qpos'])
                    self.assertEqual(first['source_time_s'],restart['source_time_s'])
                    self.assertEqual(restart['rgba'],first['rgba'])
                    self.assertGreater(end['source_time_s'],first['source_time_s'])
                    track.loop=False
                    a=track.at(track.frame_count+9)
                    np.testing.assert_array_equal(a['qpos'],end['qpos'])
                    self.assertTrue(a['completed'])
                finally:track.close()

    def test_two_different_clocks_commit_one_global_frame(self):
        scene=current_scene()
        if not scene.exists():self.skipTest('Generated current factory not present')
        release=deepcopy(read(ROOT/RELEASE))
        release['families']={n:f for n,f in release['families'].items()if n in ('dial','transport')}
        for f in release['families'].values():f['oracle']=f['oracle'][:1]
        timeline=FactoryTimeline(scene,release)
        try:
            with patch('mujoco.mj_step',side_effect=AssertionError('No replay physics')),\
                    patch.object(timeline.play,'commit_frame',wraps=timeline.play.commit_frame)as commit:
                frame=max(t.frame_count for t in timeline.tracks)+5
                row=timeline.at(frame)
                self.assertEqual(commit.call_count,1)
                self.assertEqual(timeline.play.data.time,frame/30)
                self.assertEqual(len(row['stations']),2)
                self.assertTrue(all(r['cycle']>=1 for r in row['stations']))
                self.assertNotEqual(row['stations'][0]['source_time_s'],row['stations'][1]['source_time_s'])
        finally:timeline.close()

    def test_camera_tour_keeps_task_order_and_has_independent_time(self):
        manifest=dict(placements=[dict(family='mosaic',case_id='mosaic-a04-6x12-d2',planning_bounds_xy_m=[0,0,4,4]),
                                  dict(family='mosaic',case_id='mosaic-a04-6x12-d1',planning_bounds_xy_m=[6,0,10,4])])
        plan=shots(manifest)
        self.assertTrue(plan[2]['id'].endswith('d1'))
        self.assertTrue(plan[3]['id'].endswith('d2'))
        self.assertEqual(camera_at(plan,0),camera_at(plan,sum(p['duration_s']for p in plan)))


if __name__=='__main__':unittest.main()
