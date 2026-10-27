"""Freeze reusable edited 60Hz tracks; render paired backgrounds at 20x/30fps.

Only private presentation models are changed. Original worlds, trajectories,
native reports and earlier movies are never written or stepped.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from robofrontier.relay_motion_edit import (
    keep_intervals, select_source_rows, retained_timeline, video_rows,
)
from robofrontier.motion_recording import MotionReader, digest, freeze_source
from robofrontier.workstation import Workstation
from robofrontier.factory_workstations import compose, FactoryPlayback
from robofrontier.factory_view import FactoryRenderer


def write_json(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')


def prepare_case(case, analysis, output):
    folder = output / case['case_id']
    folder.mkdir(parents=True, exist_ok=True)
    info = json.loads((analysis / case['case_id'] / 'source.json').read_text())
    maskfile = analysis / case['case_id'] / 'visual-keep.npy'
    if (folder / 'edit.json').exists():
        prior = json.loads((folder / 'edit.json').read_text())
        if (prior['source_scene_sha256'] != info['scene_sha256'] or
            prior['mask_sha256'] != digest(maskfile) or
            abs(prior['original_completion_cutoff_s'] - info['end_sim_s']) > 1e-8 or
            Path(prior['original_recording']) != (Path(info['source']) / 'motion').resolve()):
            raise ValueError('Existing edit belongs to different source/cuts; use a new output folder')
        if digest(folder / 'track.npz') != prior['track_sha256']:
            raise ValueError('Existing derived track checksum mismatch')
        return prior
    reader = MotionReader(Path(info['source']) / 'motion')
    with np.load(analysis / case['case_id'] / 'samples.npz') as z:
        times = z['times'].copy()
        context_q = z['qpos'][[0, -1]].copy()
    keep = np.load(maskfile)
    intervals = keep_intervals(times, keep, info['analysis_hz'])
    if not len(intervals):
        raise ValueError('No motion to export')
    rows, states, ids = [], [], []
    prefix, used = [], []
    offset = 0
    for chunk in reader.meta['chunks']:
        if chunk['start_sim_s'] > info['end_sim_s']:
            break
        prefix.append(chunk.copy())
        overlap = ((intervals[:, 0] <= chunk['end_sim_s']) &
                   (intervals[:, 1] > chunk['start_sim_s']))
        if np.any(overlap):
            path = reader.path / chunk['file']
            if digest(path) != chunk['sha256']:
                raise ValueError('Original recording checksum mismatch')
            with np.load(path, allow_pickle=False) as z:
                stamps = z['time_s']
                q = np.bitwise_xor(np.bitwise_xor.accumulate(z['xor'], axis=0),
                                   z['base']).view('<f4')
                selected = select_source_rows(stamps, intervals) & (stamps <= info['end_sim_s'] + 1e-8)
                rows.append(stamps[selected].copy())
                states.append(q[selected].copy())
                ids.append(np.flatnonzero(selected) + offset)
            used.append(chunk['file'])
        offset += chunk['frames']
    stamps, q, ids = np.concatenate(rows), np.concatenate(states), np.concatenate(ids)
    if np.any(np.diff(stamps) <= 0) or not np.isfinite(q).all():
        raise ValueError('Invalid selected recording')
    edited_time, duration, segment = retained_timeline(stamps, intervals, reader.meta['sample_hz'])
    track = folder / 'track.npz'
    np.savez_compressed(track, source_time_s=stamps, qpos=q, source_frame=ids,
                        edited_time_s=edited_time, duration_s=duration,
                        segment=segment, context_qpos=context_q,
                        context_source_time_s=times[[0, -1]])
    freeze_source(reader.path / 'source', folder / 'source', output / 'asset-cache')
    events = [e for e in reader.events() if e['sim_time_s'] <= info['end_sim_s'] + 1e-8]
    (folder / 'events.jsonl').write_text(''.join(json.dumps(e) + '\n' for e in events))
    write_json(folder / 'source-chunks.json', dict(chunks=prefix, selected_verified_chunks=used))
    write_json(folder / 'initial-rgba.json', reader.meta['initial_rgba'])
    old = json.loads((analysis / case['case_id'] / 'manifest.json').read_text())
    result = dict(schema='robofrontier.relay-motion-edit.v1', case_id=case['case_id'],
        original_recording=str(reader.path.resolve()), source_scene_sha256=info['scene_sha256'],
        source_sample_hz=reader.meta['sample_hz'], motion_decision_hz=info['analysis_hz'],
        original_completion_cutoff_s=info['end_sim_s'],
        original_native_record_status=reader.meta['status'],
        rule=old['rule'], thresholds=old['thresholds'], visual_audit=old['visual_audit'],
        mask_sha256=digest(maskfile), analysis_directory=str((analysis / case['case_id']).resolve()),
        kept_intervals_source_s=intervals.tolist(), retained_recorded_states=len(stamps),
        retained_duration_s=float(edited_time[-1] + duration[-1]),
        track_sha256=digest(track), events_sha256=digest(folder / 'events.jsonl'),
        initial_rgba_sha256=digest(folder / 'initial-rgba.json'),
        qpos_layout='source/workstation.json joints; metres, radians, quaternion wxyz',
        timeline='Concatenated recorded sample cells; cuts are discontinuities. No state interpolation or physics at cuts.',
        transferable='Use one rigid workstation transform on all bodies/table/robots/cargo and camera; no scale or relative-layout change. Materials/background may vary. Replay only, not evidence for new physics.',
        raw_trajectories_unchanged=True, physics_stepped=False)
    write_json(folder / 'edit.json', result)
    print(case['case_id'], '60Hz edited states', len(stamps),
          'retained seconds', round(result['retained_duration_s'], 3), flush=True)
    return result


def prepare(analysis, output):
    summary = json.loads((analysis / 'summary.json').read_text())
    output.mkdir(parents=True, exist_ok=True)
    edits = [prepare_case(c, analysis, output) for c in summary['cases']]
    write_json(output / 'summary.json', dict(complete=True, cases=edits))


def build_factory(bundle, output, shell, catalog):
    selected = json.loads((bundle / 'summary.json').read_text())['cases']
    catalog_data = json.loads(catalog.read_text())
    byid = {c['id']:c for c in catalog_data['cases']}
    catalog_data['cases'] = [dict(byid[c['case_id']], directory=str((bundle / c['case_id'] / 'source').resolve())) for c in selected]
    source = bundle / 'factory-catalog.json'
    write_json(source, catalog_data)
    print(compose(shell, source, output), flush=True)


def encoder(path, fps):
    return subprocess.Popen(['ffmpeg', '-y', '-hide_banner', '-loglevel', 'error',
        '-filter_threads', '1', '-filter_complex_threads', '1', '-f', 'rawvideo',
        '-pix_fmt', 'rgb24', '-s', '1280x800', '-r', str(fps), '-i', '-', '-an',
        '-c:v', 'h264_videotoolbox', '-threads', '1', '-b:v', '5M', '-allow_sw', '0',
        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(path)], stdin=subprocess.PIPE)


def validate_movie(path, frames, fps):
    probe = json.loads(subprocess.check_output(['ffprobe', '-v', 'error',
        '-show_entries', 'stream=nb_frames,r_frame_rate,width,height', '-of', 'json', str(path)], text=True))['streams'][0]
    assert int(probe['nb_frames']) == frames, probe
    assert probe['r_frame_rate'] == f'{fps}/1', probe
    decoded = subprocess.run(['ffmpeg', '-v', 'error', '-threads', '1', '-filter_threads', '1',
        '-i', str(path), '-f', 'null', '-'], capture_output=True, text=True)
    assert decoded.returncode == 0, decoded.stderr
    return dict(decode_passed=True, frames=frames, fps=fps, duration_s=frames/fps, sha256=digest(path))


def render_case(folder, output, factory_renderer=None, play=None, fps=30, speed=20):
    edit = json.loads((folder / 'edit.json').read_text())
    if edit.get('schema') not in ('robofrontier.relay-motion-edit.v1','robofrontier.motion-edit.v1') or not {'track_sha256','kept_intervals_source_s'} <= edit.keys():
        raise ValueError('Expected a Relay recorded-track edit; a frame-reference timeline needs its own adapter')
    assert digest(folder / 'track.npz') == edit['track_sha256']
    for name, key in [('events.jsonl','events_sha256'),('initial-rgba.json','initial_rgba_sha256')]:
        assert digest(folder / name) == edit[key]
    source = Workstation(folder / 'source')
    with np.load(folder / 'track.npz', allow_pickle=False) as z:
        track = {k:z[k].copy() for k in z.files}
    indices, _ = video_rows(track['edited_time_s'], track['duration_s'], fps, speed)
    case = edit['case_id']
    output.mkdir(parents=True, exist_ok=True)
    plan = dict(track_sha256=edit['track_sha256'], fps=fps, speed=speed,
                factory_scene_sha256=play.manifest['scene_sha256'] if play else None)
    if (output / 'render.json').exists():
        prior = json.loads((output / 'render.json').read_text())
        assert prior['plan'] == plan
        for key in prior['videos']:
            validate_movie(output / f'{key}.mp4', len(indices), fps)
        print(case, 'existing paired movies verified', flush=True)
        return prior
    paths = {'simple':output / 'simple.mp4'}
    if play:
        paths['factory'] = output / 'factory.mp4'
        place = next(p for p in play.manifest['placements'] if p['case_id'] == case)
        yaw=place['yaw_rad'];c,s=np.cos(yaw),np.sin(yaw);rot=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
        camera=source.model.camera('global').id;position=source.model.cam_pos[camera]
        from robofrontier.robot_control import quat_matrix
        rotation=quat_matrix(source.model.cam_quat[camera]);shift=np.asarray(place['translation_m'])
        factory_renderer.set_camera(dict(position_m=(rot@position+shift).tolist(),
            target_m=(rot@(position-rotation[:,2])+shift).tolist(),projection='perspective',
            vertical_fov_deg=float(source.model.cam_fovy[camera]),hidden_layers=['roof']))
    initial=json.loads((folder / 'initial-rgba.json').read_text());source.model.geom_rgba[:]=initial
    geometry_ids={g['id']:g['source_index'] for g in source.manifest['geometries']}
    if play:play.appearance(case,{name:initial[g] for name,g in geometry_ids.items()})
    events=iter(json.loads(line) for line in (folder / 'events.jsonl').read_text().splitlines())
    event=next(events,None);font=ImageFont.truetype('/System/Library/Fonts/STHeiti Medium.ttc',22)
    partials={name:path.with_name(path.stem+'.partial.mp4') for name,path in paths.items()}
    encoders={name:encoder(path,fps) for name,path in partials.items()}
    maximum_position_error=0.;maximum_rotation_error=0.;started=time.monotonic()
    mapfile=(output / 'frame-map.csv').open('w')
    mapfile.write('video_frame,video_time_s,source_time_s,edited_time_s,track_row,source_frame,segment,context\n')
    source.model.vis.quality.offsamples=0
    try:
        with mujoco.Renderer(source.model,width=1280,height=800,max_geom=max(5000,source.model.ngeom+1000)) as renderer:
            for n,row in enumerate(indices):
                if row < 0:
                    ctx=0 if row==-1 else 1
                    q=track['context_qpos'][ctx];stamp=float(track['context_source_time_s'][ctx])
                    edited=0. if ctx==0 else edit['retained_duration_s'];raw=-1;segment=-1
                else:
                    q=track['qpos'][row];stamp=float(track['source_time_s'][row]);edited=float(track['edited_time_s'][row]);raw=int(track['source_frame'][row]);segment=int(track['segment'][row])
                while event is not None and event['sim_time_s'] <= stamp+1e-9:
                    if event['event']=='rgba':
                        for name,color in event['geoms'].items():source.model.geom_rgba[geometry_ids[name]]=color
                        if play:play.appearance(case,event['geoms'])
                    event=next(events,None)
                if play:
                    error=play.apply_arrays(case,source,q,stamp,verify=True)
                    maximum_position_error=max(maximum_position_error,error['position_error_m'])
                    maximum_rotation_error=max(maximum_rotation_error,error['rotation_matrix_error'])
                else:
                    source.data.qpos[:]=q;source.data.time=stamp
                    mujoco.mj_kinematics(source.model,source.data);mujoco.mj_comPos(source.model,source.data);mujoco.mj_camlight(source.model,source.data)
                renderer.update_scene(source.data,camera='global')
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION]=False
                images={'simple':renderer.render()}
                if factory_renderer:
                    if factory_renderer.renderer_name=='filament':
                        factory_renderer.renderables.update(factory_renderer.data)
                    images['factory']=factory_renderer.render()
                for name,pixels in images.items():
                    image=Image.fromarray(pixels);draw=ImageDraw.Draw(image)
                    draw.rectangle((10,10,1268,77),fill='#f3f5f7')
                    draw.text((22,16),f'{case} · '+('工厂背景' if name=='factory' else '简单背景')+' · 20倍速 · 30fps',font=font,fill='#1d3449')
                    draw.text((22,46),f'原始仿真 {stamp:.3f}s | 同一60Hz轨迹、同一去静止剪辑；砝码独摆等待已删除',font=font,fill='#35516a')
                    encoders[name].stdin.write(image.tobytes())
                    if n==0:image.save(output / f'{name}-first.png')
                    if n==len(indices)-1:image.save(output / f'{name}-last.png')
                    if n==len(indices)//2:image.save(output / f'{name}-middle.png')
                mapfile.write(f'{n},{n/fps:.9f},{stamp:.9f},{edited:.9f},{row},{raw},{segment},{int(row<0)}\n')
                if (n+1)%300==0:print(case,'20x paired frames',n+1,'/',len(indices),'wall',round(time.monotonic()-started,1),flush=True)
    finally:
        mapfile.close()
        for process in encoders.values():process.stdin.close()
        for process in encoders.values():assert process.wait()==0,'Video encoder failed'
    checks={name:validate_movie(path,len(indices),fps) for name,path in partials.items()}
    for name,path in partials.items():path.replace(paths[name])
    result=dict(case_id=case,plan=plan,videos=checks,shared_frame_map='frame-map.csv',
        retained_duration_s=edit['retained_duration_s'],
        maximum_factory_geometry_position_error_m=maximum_position_error,
        maximum_factory_geometry_rotation_matrix_error=maximum_rotation_error,
        endpoint_context_frames=2,endpoint_context_duration_s=2/fps,
        physics_stepped=False,original_worlds_or_trajectories_changed=False,
        source_scene_sha256=edit['source_scene_sha256'],wall_s=time.monotonic()-started)
    write_json(output / 'render.json',result)
    print(case,'paired render complete',len(indices)/fps,'seconds',flush=True)
    return result


def render_all(bundle, output, factory, renderer_name, only=None):
    edits=json.loads((bundle / 'summary.json').read_text())['cases']
    factory_renderer=FactoryRenderer(factory,renderer=renderer_name,width=1280,height=800) if factory else None
    play=FactoryPlayback(factory,model=factory_renderer.model,data=factory_renderer.data) if factory else None
    try:
        reports=[]
        for edit in edits:
            if only and edit['case_id']!=only:continue
            reports.append(render_case(bundle / edit['case_id'],output / edit['case_id'],factory_renderer,play))
        write_json(output / ('summary.json' if not only else only+'-summary.json'),dict(complete=not only,cases=reports))
    finally:
        if factory_renderer:factory_renderer.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='action',required=True)
    p=sub.add_parser('prepare');p.add_argument('analysis',type=Path);p.add_argument('--output',type=Path,required=True)
    p=sub.add_parser('factory');p.add_argument('bundle',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--shell',type=Path,default=Path('outputs/robo-factory/v6-shell/scene.xml'));p.add_argument('--catalog',type=Path,default=Path('configs/suites/frontier-v2-r10.json'))
    p=sub.add_parser('render');p.add_argument('bundle',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--factory',type=Path);p.add_argument('--renderer',choices=['classic','filament'],default='filament');p.add_argument('--only')
    args=parser.parse_args()
    if args.action=='prepare':prepare(args.analysis.resolve(),args.output.resolve())
    elif args.action=='factory':build_factory(args.bundle.resolve(),args.output.resolve(),args.shell.resolve(),args.catalog.resolve())
    else:render_all(args.bundle.resolve(),args.output.resolve(),args.factory.resolve() if args.factory else None,args.renderer,args.only)


if __name__=='__main__':main()
