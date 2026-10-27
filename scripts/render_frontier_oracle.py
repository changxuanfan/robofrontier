"""Render stored physical configurations, including the shared chunked format.

Only kinematics and appearance events are restored. No commands, IK, contact
solve or new physical integration are executed to manufacture a video.
"""
import argparse,json,subprocess,time
from pathlib import Path
import mujoco,numpy as np
from PIL import Image,ImageDraw
from robofrontier.suite.evaluate import DIGITS
from robofrontier.cube_transport import _font
from robofrontier.run_setup import write_json
from robofrontier.motion_recording import MotionReader,digest
from robofrontier.motion_edit import MotionEdit


def render(directory,views=('global','global_rear'),speed=4,every=None,output=None,encoder_threads=1,edit_timeline=None):
    directory=Path(directory).resolve();report=json.loads((directory/'report.json').read_text())
    model=mujoco.MjModel.from_xml_path(str(directory/'scene.xml'));data=mujoco.MjData(model)
    shared=(directory/'motion/stream.json').exists();events=None;event=None;edit=None
    if shared:
        reader=MotionReader(directory/'motion')
        if not reader.meta['frames']:raise ValueError('No recorded states')
        if digest(directory/'scene.xml')!=reader.meta['source_scene_sha256']:raise ValueError('Trial render scene differs from the frozen motion source')
        source_hz=float(reader.meta['sample_hz']);edit=MotionEdit(edit_timeline,reader) if edit_timeline else None
        stream=((t,q,None) for t,q in (edit.frames(reader) if edit else reader.frames()))
        model.geom_rgba[:]=reader.meta['initial_rgba'];events=iter(reader.events());event=next(events,None)
        registry=json.loads((reader.path/'source/workstation.json').read_text())
        geom_ids={g['id']:g['source_index'] for g in registry['geometries']}
    else:
        if edit_timeline:raise ValueError('Edit timeline requires the shared source recording')
        states=np.load(directory/'frame-states.npz')
        if len(states['time_s'])<2:raise ValueError('Insufficient legacy recorded states')
        source_hz=1/float(np.median(np.diff(states['time_s'])))
        stream=((float(t),q,int(d)) for t,q,d in zip(states['time_s'],states['qpos'],states['display']))
    if edit:
        if every is not None:raise ValueError('Edit timeline defines exact sampling; omit --every')
        speed=edit.data['nominal_speedup']
    every=max(1,round(speed*source_hz/(edit.data['fps'] if edit else 30))) if every is None else every
    if speed<=0 or every<1:raise ValueError('Positive playback speed and sampling stride required')
    if encoder_threads<1:raise ValueError('Positive encoder thread count required')
    fps=edit.data['fps'] if edit else speed*source_hz/every
    target=Path(output) if output else directory/'video';target.mkdir(parents=True,exist_ok=True)
    processes={};started=time.monotonic();font=_font(22);rendered=0;source_frames=0;last=None;last_rendered=None
    for view in views:
        path=target/(view+'.mp4')
        if path.exists():raise FileExistsError(path)
        processes[view]=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-filter_threads','1','-filter_complex_threads','1','-f','rawvideo','-pix_fmt','rgb24','-s','1280x800','-r',str(fps),'-i','-','-an','-c:v','libx264','-threads',str(encoder_threads),'-crf','23','-preset','fast','-pix_fmt','yuv420p','-movflags','+frag_keyframe+empty_moov+default_base_moof','-stats_period','2','-progress',str(target/(view+'-encoding-progress.txt')),str(path)],stdin=subprocess.PIPE)
    commissioning=report.get('commissioning_only') or report.get('full_task_attempt') is False
    outcome=('MECHANICAL PASS' if report.get('passed') or report.get('local_passed') else 'MECHANICAL PARTIAL / FAILED') if commissioning else 'PASS' if report.get('passed') else 'UNIT PASS' if report.get('local_passed') else 'INTERRUPTED' if report.get('interrupted') else 'PARTIAL / FAILED'
    try:
        with mujoco.Renderer(model,height=800,width=1280) as renderer:
            def emit(stamp,q):
                data.qpos[:]=q;data.time=stamp
                mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data);mujoco.mj_camlight(model,data)
                for view,process in processes.items():
                    renderer.update_scene(data,camera=view);image=Image.fromarray(renderer.render());draw=ImageDraw.Draw(image)
                    draw.rectangle((12,12,1000,73),fill=(245,242,235))
                    draw.text((23,18),f'{report["case_id"]} | Oracle {outcome} | recorded-state replay',font=font,fill=(23,32,43))
                    draw.text((23,43),f'{view} | sim {stamp:.1f} s | {speed:g}x | {source_hz:.1f} Hz source',font=font,fill=(23,32,43))
                    process.stdin.write(image.tobytes())
            for index,(stamp,q,display) in enumerate(stream):
                while event is not None and event['sim_time_s']<=stamp+1e-9:
                    if event['event']=='rgba':
                        for name,value in event['geoms'].items():model.geom_rgba[geom_ids[name]]=value
                    event=next(events,None)
                if not shared and mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'score_0_0')>=0:
                    show=f'{display:02}' if display>=0 else '--'
                    for digit,char in enumerate(show):
                        for k,segment in enumerate('abcdefg'):
                            model.geom_rgba[model.geom(f'score_{digit}_{k}').id]=[.68,.13,.07,1 if segment in (DIGITS[int(char)] if char!='-' else 'g') else 0]
                source_frames+=1;last=(stamp,q)
                if not edit and index%every:continue
                emit(stamp,q);rendered+=1;last_rendered=stamp
            if last is not None and last[0]!=last_rendered:emit(*last);rendered+=1
    finally:
        for process in processes.values():
            if process.stdin:process.stdin.close()
        codes=[process.wait() for process in processes.values()]
    if any(codes):raise RuntimeError('Video encoding failed '+str(codes))
    checks={}
    for view in views:
        path=target/(view+'.mp4');check=subprocess.run(['ffmpeg','-v','error','-threads','1','-i',str(path),'-threads','1','-f','null','-'],capture_output=True,text=True)
        checks[view]=dict(path=str(path.resolve()),decode_passed=check.returncode==0,error=check.stderr or None)
    write_json(target/'manifest.json',dict(kind='Recorded-state FK rerender; no commands, IK, contacts or integration',source_trial=str(directory),source_scene_sha256=report['scene_sha256'],source_format='robofrontier.motion.v1' if shared else 'legacy_npz',recorded_source_hz=source_hz,speedup=speed,every=every,fps=fps,encoder_threads=encoder_threads,source_frames=reader.meta['frames'] if edit else source_frames,rendered_states=rendered,edit_timeline=str(Path(edit_timeline).resolve()) if edit else None,edit_timeline_sha256=digest(edit_timeline) if edit else None,all_source_chunks_and_selected_timestamps_verified=bool(edit),wall_time_s=time.monotonic()-started,trial_passed=report.get('passed'),local_unit_passed=report.get('local_passed'),commissioning_only=report.get('commissioning_only',False),views=checks))
    if not all(c['decode_passed'] for c in checks.values()):raise RuntimeError('A video could not be fully decoded')
    print(target,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('trial');p.add_argument('--speed',type=float,default=4);p.add_argument('--every',type=int);p.add_argument('--output',type=Path);p.add_argument('--views',nargs='+',default=['global','global_rear']);p.add_argument('--encoder-threads',type=int,default=1);p.add_argument('--edit-timeline',type=Path);a=p.parse_args()
    render(a.trial,a.views,a.speed,a.every,a.output,a.encoder_threads,a.edit_timeline)
