"""Render R11 BEST5 configurations from their source-bound 60 Hz record.

Only kinematics and appearance events are restored. No commands, IK, contact
solve or new physical integration are executed to manufacture a video.
"""
import argparse,json,subprocess,time
from pathlib import Path
import mujoco
from PIL import Image,ImageDraw
from robofrontier.cube_transport import _font
from robofrontier.run_setup import write_json
from robofrontier.motion_recording import MotionReader
from robofrontier.suite.dial_pose_timeline import EditedMotion,checksum,playback_speed,DIAL_VIDEO_FPS


def render(directory,views=('global','global_rear'),speed=None,every=None,output=None,edit=None):
    directory=Path(directory).resolve();report=json.loads((directory/'report.json').read_text())
    if not report['case_id'].startswith('dial-best5-'):raise ValueError('Use the R11 BEST5 dial entrypoint only')
    speed=playback_speed(report['case_id'],speed)
    model=mujoco.MjModel.from_xml_path(str(directory/'scene.xml'));data=mujoco.MjData(model)
    reader=MotionReader(directory/'motion')
    if not reader.meta['frames']:raise ValueError('No recorded states')
    source_hz=float(reader.meta['sample_hz']);stream=reader.frames()
    model.geom_rgba[:]=reader.meta['initial_rgba'];events=iter(reader.events());event=next(events,None)
    registry=json.loads((reader.path/'source/workstation.json').read_text())
    geom_ids={g['id']:g['source_index'] for g in registry['geometries']}
    every=max(1,round(speed*source_hz/DIAL_VIDEO_FPS)) if every is None else every
    if speed<=0 or every<1:raise ValueError('Positive playback speed and sampling stride required')
    fps=speed*source_hz/every
    edited=EditedMotion(directory/'motion',edit) if edit is not None else None
    selected=set(edited.video_indices(fps,speed).tolist()) if edited else None
    selected_frames=[];selected_times=[]
    target=Path(output) if output else directory/'video';target.mkdir(parents=True,exist_ok=True)
    processes={};started=time.monotonic();font=_font(22);rendered=0;source_frames=0;last=None;last_rendered=None
    for view in views:
        path=target/(view+'.mp4')
        if path.exists():raise FileExistsError(path)
        processes[view]=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','1280x800','-r',str(fps),'-i','-','-an','-c:v','libx264','-crf','23','-preset','fast','-pix_fmt','yuv420p','-movflags','+frag_keyframe+empty_moov+default_base_moof',str(path)],stdin=subprocess.PIPE)
    commissioning=report.get('commissioning_only') or report.get('full_task_attempt') is False
    outcome=('MECHANICAL PASS' if report.get('passed') or report.get('local_passed') else 'MECHANICAL PARTIAL / FAILED') if commissioning else 'PASS' if report.get('passed') else 'UNIT PASS' if report.get('local_passed') else 'INTERRUPTED' if report.get('interrupted') else 'PARTIAL / FAILED'
    assisted=bool(report.get('assisted',report.get('llm_used',False)))
    if assisted:outcome+=' (ASSISTED)'
    try:
        with mujoco.Renderer(model,height=800,width=1280) as renderer:
            def emit(stamp,q):
                data.qpos[:]=q;data.time=stamp
                mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data);mujoco.mj_camlight(model,data)
                for view,process in processes.items():
                    renderer.update_scene(data,camera=view);image=Image.fromarray(renderer.render());draw=ImageDraw.Draw(image)
                    draw.rectangle((12,12,1000,73),fill=(245,242,235))
                    draw.text((23,18),f'{report["case_id"]} | Oracle {outcome} | recorded-state replay',font=font,fill=(23,32,43))
                    label=' | idle edited' if edited else ''
                    draw.text((23,43),f'{view} | sim {stamp:.1f} s | {speed:g}x | {source_hz:.1f} Hz source{label}',font=font,fill=(23,32,43))
                    process.stdin.write(image.tobytes())
            for index,(stamp,q) in enumerate(stream):
                while event is not None and event['sim_time_s']<=stamp+1e-9:
                    if event['event']=='rgba':
                        for name,value in event['geoms'].items():model.geom_rgba[geom_ids[name]]=value
                    event=next(events,None)
                source_frames+=1;last=(stamp,q)
                if selected is not None:
                    if index not in selected:continue
                elif index%every:continue
                emit(stamp,q);rendered+=1;last_rendered=stamp
                selected_frames.append(index);selected_times.append(stamp)
            if last is not None and last[0]!=last_rendered:
                emit(*last);rendered+=1;selected_frames.append(source_frames-1);selected_times.append(last[0])
    finally:
        for process in processes.values():
            if process.stdin:process.stdin.close()
        codes=[process.wait() for process in processes.values()]
    if any(codes):raise RuntimeError('Video encoding failed '+str(codes))
    checks={}
    for view in views:
        path=target/(view+'.mp4');check=subprocess.run(['ffmpeg','-v','error','-i',str(path),'-f','null','-'],capture_output=True,text=True)
        checks[view]=dict(path=str(path.resolve()),decode_passed=check.returncode==0,error=check.stderr or None)
    write_json(target/'manifest.json',dict(kind='Recorded-state FK rerender; no commands, IK, contacts or integration',source_trial=str(directory),source_scene_sha256=report['scene_sha256'],source_format='robofrontier.motion.v1',recorded_source_hz=source_hz,speedup=speed,every=every,fps=fps,source_frames=source_frames,rendered_states=rendered,wall_time_s=time.monotonic()-started,trial_passed=report.get('passed'),assisted=assisted,oracle_mode=report.get('oracle_mode','pure_program'),local_unit_passed=report.get('local_passed'),commissioning_only=report.get('commissioning_only',False),edit_file=str(Path(edit).resolve()) if edit else None,edit_sha256=checksum(edit) if edit else None,video_source_frame_indices=selected_frames,video_source_timestamps_s=selected_times,views=checks))
    if not all(c['decode_passed'] for c in checks.values()):raise RuntimeError('A video could not be fully decoded')
    print(target,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trial');p.add_argument('--speed',type=float,help='Default: 10x');p.add_argument('--every',type=int);p.add_argument('--output',type=Path);p.add_argument('--views',nargs='+',default=['global','global_rear']);p.add_argument('--edit',type=Path,required=True);a=p.parse_args()
    render(a.trial,a.views,a.speed,a.every,a.output,a.edit)
