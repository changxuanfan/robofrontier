"""Build, view, audit or render all released Oracle workstations in one factory."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import shutil

import numpy as np

from robofrontier.task_families import ROOT, RELEASE, read
from robofrontier.factory_playback import FactoryTimeline,current_scene
from robofrontier.factory_cameras import shots, camera_at
from robofrontier.run_setup import write_json
from robofrontier.motion_recording import digest

SCENE = current_scene()


def orbit_camera(seconds,duration):
    angle=-2.35+2*np.pi*seconds/duration
    return dict(id='factory-orbit',position_m=[23*np.cos(angle),22*np.sin(angle),4.8+.2*np.sin(angle)],
        target_m=[0,0,.7],vertical_fov_deg=62,projection='perspective',hidden_layers=[])


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
        appearance_ids=sorted({model.geom(names[track.spec['id']]['geom_names'][name]).id
            for track in timeline.tracks for event in track._event_list for name in event['geoms']
            if name in names[track.spec['id']]['geom_names']})
        gids=np.array(sorted({model.geom(target).id for track in timeline.tracks
            for target in names[track.spec['id']]['geom_names'].values()}),dtype=np.int32)
        plan=shots(play.manifest)
        preset=next((c for c in play.manifest['cameras'] if c['id']==args.camera),None)
        if args.camera not in ('tour','orbit') and preset is None:raise ValueError('Unknown camera')
        frames=[];count=math.ceil(args.duration*timeline.fps)
        with (output/'frames.jsonl').open('x') as trace:
            for index in range(count):
                trace.write(json.dumps(timeline.at(index),separators=(',',':'))+'\n')
                file=output/f'pose-{index:06d}.npz'
                np.savez_compressed(file,geom_ids=gids,position=data.geom_xpos[gids].astype(np.float32),
                    rotation=data.geom_xmat[gids].reshape(-1,3,3).astype(np.float32),rgba=model.geom_rgba[gids],
                    body_ids=np.unique(model.geom_bodyid[gids]),
                    body_position=data.xpos[np.unique(model.geom_bodyid[gids])].astype(np.float32),
                    body_rotation=data.xmat[np.unique(model.geom_bodyid[gids])].reshape(-1,3,3).astype(np.float32))
                camera=orbit_camera(index/timeline.fps,args.duration)if args.camera=='orbit' else preset or camera_at(plan,index/timeline.fps)
                frames.append(dict(file=file.name,sha256=digest(file),camera=camera))
                if index%300==0:print('pose',index,'/',count,flush=True)
        result=dict(schema='robofrontier.factory-animation.v1',source_scene_sha256=digest(args.scene),
            fps=timeline.fps,frames=frames,loop=True,stations=len(timeline.tracks),
            release_sha256=digest(ROOT/RELEASE),source_trace_sha256=digest(output/'frames.jsonl'),
            appearance_geom_ids=appearance_ids,
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


def render_offline(args):
    """Export one source-bound clip, then render it in one persistent Blender process."""
    from scripts.export_factory_blender import export
    output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=False)
    started=time.monotonic();export(args.scene,output/'asset')
    pose_args=argparse.Namespace(**vars(args));pose_args.output=output/'poses'
    export_animation(pose_args)
    preparation_s=time.monotonic()-started
    blender=args.blender or shutil.which('blender') or '/Applications/Blender.app/Contents/MacOS/Blender'
    command=[blender,'--background','--python-exit-code','1','--python',str(ROOT/'scripts/render_factory_blender.py'),'--',
        '--asset',str(output/'asset'),'--animation',str(output/'poses/animation.json'),'--output',str(output),
        '--engine',args.renderer,'--profile',args.profile,'--device',args.device,'--samples',str(args.samples if args.samples is not None else (24 if args.renderer=='cycles'else 4)),
        '--width',str(args.width),'--height',str(args.height)]
    command.append('--denoise'if args.denoise else '--no-denoise')
    if args.save_blend:command.append('--save-blend')
    with (output/'blender.log').open('x')as log:subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
    result=read(output/'report.json');result.update(preparation_wall_s=preparation_s,end_to_end_wall_s=time.monotonic()-started)
    write_json(output/'report.json',result);return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    builder=commands.add_parser('build');builder.add_argument('--output',type=Path,default=SCENE.parent)
    video=commands.add_parser('video')
    video.add_argument('--scene',type=Path,default=SCENE);video.add_argument('--output',type=Path,required=True)
    video.add_argument('--duration',type=float,required=True);video.add_argument('--camera',default='tour')
    video.add_argument('--renderer',choices=['eevee','cycles'],default='cycles')
    video.add_argument('--profile',choices=['fast','reference'],default='fast')
    video.add_argument('--samples',type=int);video.add_argument('--device',choices=['METAL','CPU'],default='METAL')
    video.add_argument('--denoise',action=argparse.BooleanOptionalAction,default=True)
    video.add_argument('--save-blend',action='store_true')
    video.add_argument('--blender',help='Blender executable; auto-detected on macOS')
    video.add_argument('--width',type=int,default=1280);video.add_argument('--height',type=int,default=720)
    video.add_argument('--families',nargs='+',choices=['transport','mosaic','relay','dial'])
    poses=commands.add_parser('poses',help='Export common playback for offline Cycles animation')
    poses.add_argument('--scene',type=Path,default=SCENE);poses.add_argument('--output',type=Path,required=True)
    poses.add_argument('--duration',type=float,required=True);poses.add_argument('--camera',default='tour')
    poses.add_argument('--families',nargs='+',choices=['transport','mosaic','relay','dial'])
    viewer=commands.add_parser('view');viewer.add_argument('--scene',type=Path,default=SCENE)
    args=parser.parse_args()
    if args.command=='build':print(build(args.output))
    elif args.command=='video':print(json.dumps(render_offline(args),indent=2))
    elif args.command=='poses':print(json.dumps(export_animation(args),indent=2))
    else:
        from robofrontier.factory_view import serve
        serve(args.scene,play=True)


if __name__=='__main__':main()
