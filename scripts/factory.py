"""Build, view, audit or render all released Oracle workstations in one factory."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time

import numpy as np
from PIL import Image

from robofrontier.task_families import ROOT, RELEASE, read
from robofrontier.factory_playback import FactoryTimeline,current_scene
from robofrontier.factory_cameras import shots, camera_at
from robofrontier.run_setup import write_json
from robofrontier.motion_recording import digest

SCENE = current_scene()


def export_animation(args):
    """Stream source-bound poses for one retained offline Cycles scene."""
    import mujoco
    if not math.isfinite(args.duration) or args.duration<=0:
        raise ValueError('Positive finite video duration required')
    output=Path(args.output);output.mkdir(parents=True,exist_ok=False)
    timeline=FactoryTimeline(args.scene,read(ROOT/RELEASE),args.families)
    try:
        play=timeline.play;model,data=play.model,play.data
        names=play.manifest['workstation_names']
        gids=np.array(sorted({model.geom(target).id for track in timeline.tracks
            for target in names[track.spec['id']]['geom_names'].values()}),dtype=np.int32)
        plan=shots(play.manifest)
        preset=next((c for c in play.manifest['cameras'] if c['id']==args.camera),None)
        if args.camera!='tour' and preset is None:raise ValueError('Unknown camera')
        frames=[];count=math.ceil(args.duration*timeline.fps)
        with (output/'frames.jsonl').open('x') as trace:
            for index in range(count):
                trace.write(json.dumps(timeline.at(index),separators=(',',':'))+'\n')
                file=output/f'pose-{index:06d}.npz'
                np.savez_compressed(file,geom_ids=gids,position=data.geom_xpos[gids].astype(np.float32),
                    rotation=data.geom_xmat[gids].reshape(-1,3,3).astype(np.float32),rgba=model.geom_rgba[gids])
                frames.append(dict(file=file.name,sha256=digest(file),camera=preset or camera_at(plan,index/timeline.fps)))
                if index%300==0:print('pose',index,'/',count,flush=True)
        result=dict(schema='robofrontier.factory-animation.v1',source_scene_sha256=digest(args.scene),
            fps=timeline.fps,frames=frames,loop=True,stations=len(timeline.tracks),
            release_sha256=digest(ROOT/RELEASE),source_trace_sha256=digest(output/'frames.jsonl'),
            physics_steps=0,scope='Source FK and appearance; no task dynamics or commands')
        write_json(output/'animation.json',result);write_json(output/'camera-shots.json',plan)
        return dict(frames=count,stations=len(timeline.tracks),output=str(output))
    finally:timeline.close()


def build(output):
    from robofrontier.factory_scene import build_factory
    from robofrontier.factory_workstations import compose
    output=Path(output)
    shell=output.parent/(output.name+'-shell')
    if output.exists() or shell.exists():raise FileExistsError('Choose a new factory output')
    scene,_=build_factory(output=shell)
    result=compose(scene,ROOT/'configs/suites/frontier-current-oracle.json',output)
    manifest=read(output/'manifest.json')
    write_json(output/'camera-shots.json',shots(manifest))
    write_json(ROOT/'outputs/robo-factory/current.json',dict(scene=str(result),purpose='oracle',release=RELEASE))
    return result


def render(args):
    from robofrontier.factory_view import FactoryRenderer
    if not math.isfinite(args.duration) or args.duration<=0:raise ValueError('Positive finite video duration required')
    output=Path(args.output);output.mkdir(parents=True,exist_ok=False)
    started=time.monotonic();release=read(ROOT/RELEASE)
    renderer=FactoryRenderer(args.scene,args.renderer,args.width,args.height)
    timeline=None;encoder=None
    try:
        timeline=FactoryTimeline(args.scene,release,args.families,model=renderer.model,data=renderer.data)
        plan=shots(renderer.manifest)
        if args.camera!='tour':renderer.select(args.camera)
        fps=timeline.fps;count=math.ceil(args.duration*fps)
        encoder=subprocess.Popen(['ffmpeg','-v','error','-filter_threads','1','-f','rawvideo',
            '-pix_fmt','rgb24','-s',f'{args.width}x{args.height}','-r',str(fps),'-i','-',
            '-an','-c:v','libx264','-threads','1','-preset','fast','-crf','20',
            '-pix_fmt','yuv420p','-movflags','+faststart',str(output/'factory.mp4')],stdin=subprocess.PIPE)
        with (output/'frames.jsonl').open('x') as trace:
            for frame in range(count):
                row=timeline.at(frame)
                if args.camera=='tour':renderer.set_camera(camera_at(plan,frame/fps))
                pixels=renderer.render()
                encoder.stdin.write(pixels.tobytes())
                trace.write(json.dumps(row,separators=(',',':'))+'\n')
                if frame in (0,count-1):Image.fromarray(pixels).save(output/f'frame-{frame:06}.png')
                if frame%300==0:print('factory frame',frame,'/',count,flush=True)
        encoder.stdin.close();code=encoder.wait();encoder=None
        if code:raise RuntimeError('Factory video encoder failed')
        decoded=subprocess.run(['ffmpeg','-v','error','-threads','1','-i',str(output/'factory.mp4'),
            '-f','null','-'],capture_output=True,text=True,check=True)
        if decoded.stderr:raise RuntimeError(decoded.stderr)
        probe=subprocess.run(['ffprobe','-v','error','-count_frames','-select_streams','v:0',
            '-show_entries','stream=nb_read_frames,avg_frame_rate','-of','json',str(output/'factory.mp4')],capture_output=True,text=True,check=True)
        info=json.loads(probe.stdout)['streams'][0]
        if int(info['nb_read_frames'])!=count:raise ValueError('Encoded factory frame count differs')
        report=dict(renderer.report(),stations=len(timeline.tracks),fps=fps,frames=count,
            loop=True,loop_scope='independent_station',duration_s=count/fps,
            wall_s=time.monotonic()-started,full_decode_passed=True,video=info,
            source_map='frames.jsonl',camera=args.camera)
        write_json(output/'report.json',report)
        write_json(output/'camera-shots.json',plan)
        return report
    finally:
        if encoder is not None:
            encoder.stdin.close();encoder.wait()
        if timeline is not None:timeline.close()
        renderer.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    builder=commands.add_parser('build');builder.add_argument('--output',type=Path,default=SCENE.parent)
    video=commands.add_parser('video')
    video.add_argument('--scene',type=Path,default=SCENE);video.add_argument('--output',type=Path,required=True)
    video.add_argument('--duration',type=float,required=True);video.add_argument('--camera',default='tour')
    video.add_argument('--renderer',choices=['filament','classic'],default='filament')
    video.add_argument('--width',type=int,default=1280);video.add_argument('--height',type=int,default=720)
    video.add_argument('--families',nargs='+',choices=['transport','mosaic','relay','dial'])
    poses=commands.add_parser('poses',help='Export common playback for offline Cycles animation')
    poses.add_argument('--scene',type=Path,default=SCENE);poses.add_argument('--output',type=Path,required=True)
    poses.add_argument('--duration',type=float,required=True);poses.add_argument('--camera',default='tour')
    poses.add_argument('--families',nargs='+',choices=['transport','mosaic','relay','dial'])
    viewer=commands.add_parser('view');viewer.add_argument('--scene',type=Path,default=SCENE)
    args=parser.parse_args()
    if args.command=='build':print(build(args.output))
    elif args.command=='video':print(json.dumps(render(args),indent=2))
    elif args.command=='poses':print(json.dumps(export_animation(args),indent=2))
    else:
        from robofrontier.factory_view import serve
        serve(args.scene,play=True)


if __name__=='__main__':main()
