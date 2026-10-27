"""Replay immutable source motion with one shared edit in simple/factory views."""
import argparse
import json
from pathlib import Path
import subprocess

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from robofrontier.cube_transport import _font
from robofrontier.factory_workstations import FactoryPlayback
from robofrontier.suite.dial_pose_timeline import EditedMotion, checksum, playback_speed, DIAL_VIDEO_FPS
from robofrontier.motion_recording import MotionReader
from robofrontier.recording import Video
from robofrontier.robot_control import quat_matrix
from robofrontier.run_setup import write_json
from robofrontier.workstation import Workstation


def video_sample(stamp, deadline, rate=30):
    if stamp+1e-9 < deadline:
        return False, deadline
    while deadline <= stamp+1e-9:
        deadline += 1/rate
    return True, deadline


def replay(recording, factory, output, render=False, edit=None, speed=None, fps=DIAL_VIDEO_FPS, renderer='classic'):
    reader = MotionReader(recording)
    if not reader.meta['case_id'].startswith('dial-best5-'):
        raise ValueError('Dial replay requires an R11 BEST5 recording')
    speed = playback_speed(reader.meta['case_id'],speed)
    if not np.isfinite([speed, fps]).all() or min(speed, fps) <= 0:
        raise ValueError('Positive finite playback rate required')
    source = Workstation(reader.path/'source')
    edited = EditedMotion(recording, edit) if edit is not None else None
    selection = set(edited.video_indices(fps, speed).tolist()) if edited else None
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    factory_renderer = source_renderer = None
    videos = {}
    if render:
        from robofrontier.factory_view import FactoryRenderer
        factory_renderer = FactoryRenderer(factory, renderer=renderer, width=1280, height=800)
        play = FactoryPlayback(factory, model=factory_renderer.model, data=factory_renderer.data)
        case = reader.meta['case_id']
        place = next(p for p in play.manifest['placements'] if p['case_id'] == case)
        yaw = place['yaw_rad']
        rot = np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1.]])
        camera = source.model.camera('global').id
        position = source.model.cam_pos[camera]
        rotation = quat_matrix(source.model.cam_quat[camera])
        shift = np.asarray(place['translation_m'])
        factory_renderer.set_camera(dict(position_m=(rot@position+shift).tolist(),
            target_m=(rot@(position-rotation[:,2])+shift).tolist(), projection='perspective',
            vertical_fov_deg=float(source.model.cam_fovy[camera]), hidden_layers=['roof']))
        source_renderer = mujoco.Renderer(source.model, width=1280, height=800, max_geom=max(5000,source.model.ngeom+100))
        videos = {name:Video(output/(name+'.mp4'),1280,800,fps) for name in ('simple','factory')}
    else:
        play = FactoryPlayback(factory)
    case = reader.meta['case_id']
    names = play.manifest['workstation_names'][case]['geom_names']
    initial = reader.meta['initial_rgba']
    source.model.geom_rgba[:] = initial
    geometry_ids = {g['id']:g['source_index'] for g in source.manifest['geometries']}
    play.appearance(case, {name:initial[index] for name,index in geometry_ids.items()})
    events = iter(reader.events())
    event = next(events, None)
    frames = read_frames = 0
    maximum_error = 0.
    image_times, image_indices = [], []
    next_image = 0.
    trial_report_path = reader.path.parent/'report.json'
    trial_report = json.loads(trial_report_path.read_text()) if trial_report_path.exists() else {}
    assisted = bool(trial_report.get('assisted', trial_report.get('llm_used', False)))
    font = _font(21)
    outcome = ('PASS' if trial_report.get('passed') else 'RECORDED REPLAY') + (' (ASSISTED)' if assisted else '')
    try:
        for index, (stamp, qpos) in enumerate(reader.frames()):
            read_frames += 1
            # Consume events in original time, including across an edited-out gap.
            while event is not None and event['sim_time_s'] <= stamp+1e-9:
                if event['event'] == 'rgba':
                    play.appearance(case, event['geoms'])
                    for name, value in event['geoms'].items():
                        source.model.geom_rgba[geometry_ids[name]] = value
                event = next(events, None)
            if selection is not None:
                selected = index in selection
            else:
                selected, next_image = video_sample(stamp, next_image, fps/speed)
            if render and not selected:
                continue
            if not render and selection is not None and not selected:
                continue
            expected = play.apply(case, source, qpos, stamp)
            frames += 1
            target_ids = [play.model.geom(target).id for target in names.values()]
            positions = np.array([expected['geometries'][name]['position_m'] for name in names])
            maximum_error = max(maximum_error, float(np.max(np.linalg.norm(play.data.geom_xpos[target_ids]-positions, axis=1))))
            if render:
                source_renderer.update_scene(source.data, camera='global')
                for name, pixels in [('simple',source_renderer.render()), ('factory',factory_renderer.render())]:
                    image = Image.fromarray(pixels)
                    draw = ImageDraw.Draw(image)
                    draw.rectangle((12,12,1160,73), fill=(245,242,235))
                    draw.text((23,18), f'{case} | Oracle {outcome} | recorded-state replay', font=font, fill=(23,32,43))
                    draw.text((23,43), f'{name} | source sim {stamp:.2f}s | {speed:g}x | '+('shared idle edit' if edited else 'full timeline'), font=font, fill=(23,32,43))
                    if not videos[name].frame(np.asarray(image), block=True):
                        raise RuntimeError('Offline encoder could not preserve every selected frame')
                    if not image_times:
                        image.save(output/(name+'-first.png'))
                    image.save(output/(name+'-last.png')) if index == reader.meta['frames']-1 else None
                image_times.append(stamp)
                image_indices.append(index)
            if frames % 100 == 0:
                print('checked/rendered states', frames, flush=True)
    finally:
        checks = {name:video.close() for name,video in videos.items()}
        if source_renderer:
            source_renderer.close()
        if factory_renderer:
            factory_renderer.close()
    for name, check in checks.items():
        decoded = subprocess.run(['ffmpeg','-v','error','-i',str(output/(name+'.mp4')),'-f','null','-'], capture_output=True, text=True)
        check['full_decode_passed'] = decoded.returncode == 0
        if decoded.returncode:
            raise RuntimeError(decoded.stderr)
    report = dict(scope='Recorded-state FK + rigid factory placement; no physics, commands, IK or resimulation',
        case_id=case, source_frames_read=read_frames, states_checked=frames,
        maximum_geometry_position_error_m=maximum_error, passed=maximum_error<1e-6,
        record_status=reader.meta['status'], videos=checks, speedup=speed, fps=fps,
        assisted=assisted, edit_file=str(Path(edit).resolve()) if edit else None,
        edit_sha256=checksum(edit) if edit else None, source_scene_sha256=reader.meta['source_scene_sha256'],
        factory_scene_sha256=play.manifest['scene_sha256'],
        video_source_timestamps_s=image_times, video_source_frame_indices=image_indices)
    write_json(output/'report.json', report)
    if not report['passed'] or any(c['error'] for c in checks.values()):
        raise RuntimeError('Recorded-state replay verification failed')
    print(json.dumps(dict(output=str(output),frames=frames,maximum_geometry_position_error_m=maximum_error)), flush=True)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--factory', type=Path, default=Path('outputs/robo-factory/dial-best5-edited-10x-20261027/scene/scene.xml'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--render', action='store_true')
    parser.add_argument('--edit', type=Path, required=True)
    parser.add_argument('--speed', type=float, help='Default: 10x')
    parser.add_argument('--fps', type=float, default=DIAL_VIDEO_FPS)
    parser.add_argument('--renderer', choices=['classic'], default='classic')
    args = parser.parse_args()
    replay(args.recording,args.factory,args.output,args.render,args.edit,args.speed,args.fps,args.renderer)
