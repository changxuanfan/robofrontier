"""Validate source-bound edits and sample rigid factory poses, without physics."""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from robofrontier.factory_workstations import FactoryPlayback
from robofrontier.motion_edit import MotionEdit
from robofrontier.motion_recording import MotionReader, digest
from robofrontier.robot_control import quat_matrix
from robofrontier.workstation import Workstation


def check(timelines, factory, output, render_case=None):
    timelines = Path(timelines).resolve()
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    index = json.loads((timelines / 'index.json').read_text())
    renderer = None
    if render_case:
        from robofrontier.factory_view import FactoryRenderer
        renderer = FactoryRenderer(factory, renderer='classic', width=960, height=600)
    play = FactoryPlayback(factory, model=renderer.model if renderer else None,
                           data=renderer.data if renderer else None)
    rows = []
    try:
        for entry in index['timelines']:
            path = timelines / entry['timeline']
            if digest(path) != entry['sha256']:
                raise ValueError('Edit timeline checksum changed')
            raw = json.loads(path.read_text())
            reader = MotionReader(timelines / raw['source_recording_hint'])
            edit = MotionEdit(path, reader)
            case = edit.data['case_id']
            if play.manifest['workstation_names'][case]['source_scene_sha256'] != edit.data['source_scene_sha256']:
                raise ValueError('Factory workstation does not match actual recorded source: ' + case)
            source = Workstation(reader.path / 'source')
            source.model.geom_rgba[:] = reader.meta['initial_rgba']
            ids = {g['id']:g['source_index'] for g in source.manifest['geometries']}
            play.appearance(case, {name:reader.meta['initial_rgba'][g] for name,g in ids.items()})
            events = iter(reader.events())
            event = next(events, None)
            frames = edit.data['render_frames']
            samples = {0, len(frames)//2, len(frames)-1}
            cut_before = cut_after = None
            if case == 'mosaic-a08-12x24-d1':
                # The old 2:50–2:57 clip maps to these original recorded indices.
                indices = np.array([f['source_frame'] for f in frames])
                if np.any((indices >= 637920) & (indices <= 663000)):
                    raise ValueError('Previously reported static interval was reintroduced')
                cut_after = int(np.searchsorted(indices, 663001))
                cut_before = cut_after - 1
                samples.update([cut_before, cut_after])
            print('TRANSFER_CHECK_START', case, len(frames), flush=True)
            maximum = 0.
            checked = []
            for i, (stamp, qpos) in enumerate(edit.frames(reader)):
                if i not in samples:
                    continue
                while event is not None and event['sim_time_s'] <= stamp + 1e-9:
                    if event['event'] == 'rgba':
                        play.appearance(case, event['geoms'])
                        for name,value in event['geoms'].items():
                            source.model.geom_rgba[ids[name]] = value
                    event = next(events, None)
                expected = play.apply(case, source, qpos, stamp)
                names = play.manifest['workstation_names'][case]['geom_names']
                error = max(float(np.linalg.norm(play.data.geom_xpos[play.model.geom(target).id]
                                                 - expected['geometries'][name]['position_m']))
                            for name,target in names.items())
                maximum = max(maximum, error)
                checked.append(dict(output_frame=i, output_time_s=i/edit.data['fps'],
                                    source_frame=frames[i]['source_frame'], source_time_s=stamp,
                                    maximum_geometry_position_error_m=error))
                if renderer and case == render_case and i in (cut_before, cut_after):
                    label = 'before-cut' if i == cut_before else 'after-cut'
                    place = next(p for p in play.manifest['placements'] if p['case_id'] == case)
                    yaw = place['yaw_rad']
                    rotation = np.array([[np.cos(yaw),-np.sin(yaw),0],
                                         [np.sin(yaw),np.cos(yaw),0],[0,0,1.]])
                    shift = np.asarray(place['translation_m'])
                    camera = source.model.camera('global').id
                    position = source.model.cam_pos[camera]
                    direction = quat_matrix(source.model.cam_quat[camera])[:,2]
                    renderer.set_camera(dict(position_m=(rotation@position+shift).tolist(),
                        target_m=(rotation@(position-direction)+shift).tolist(),
                        projection='perspective',vertical_fov_deg=float(source.model.cam_fovy[camera]),hidden_layers=['roof']))
                    factory_image = renderer.render().copy()
                    with mujoco.Renderer(source.model, height=600, width=960) as simple:
                        simple.update_scene(source.data, camera='global')
                        Image.fromarray(simple.render()).save(output/(case+'-'+label+'-simple.png'))
                    Image.fromarray(factory_image).save(output/(case+'-'+label+'-factory.png'))
            if maximum > 1e-6:
                raise ValueError('Factory pose differs from rigid source placement')
            rows.append(dict(case_id=case, output_frames=len(frames),
                             source_frames=reader.meta['frames'],
                             all_source_chunks_and_selected_timestamps_verified=True,
                             source_revision_matches_factory=True,
                             sampled_factory_frames=checked,
                             maximum_geometry_position_error_m=maximum, passed=True))
            print('TRANSFER_CHECK_PASS', case, maximum, flush=True)
    finally:
        if renderer:
            renderer.close()
    report = dict(passed=len(rows)==9, cases=rows,
                  factory=str(Path(factory).resolve()), factory_scene_sha256=digest(factory),
                  timeline_index_sha256=digest(timelines/'index.json'),
                  physics_stepped=False, collision_or_dynamics_validation=False,
                  scope='All source recording checksums and exact edit timestamps; sampled rigid pose transfer. Four preview images, no full factory videos exported.')
    (output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--timelines', type=Path, required=True)
    p.add_argument('--factory', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--render-case')
    a = p.parse_args()
    check(a.timelines, a.factory, a.output, a.render_case)
