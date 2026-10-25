"""Export one stored-motion frame for an existing Blender factory asset.

Derived presentation data only: source FK plus the frozen workstation transform.
No IK, commands, simulation step, interpolation, or physics restart is used.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from robofrontier.factory_workstations import FactoryPlayback
from robofrontier.motion_recording import MotionReader
from robofrontier.robot_control import quat_matrix
from robofrontier.workstation import Workstation


def export(recording, factory, output, time_s):
    if not np.isfinite(time_s) or time_s<0:raise ValueError('Nonnegative finite recorded time required')
    reader=MotionReader(recording);source=Workstation(reader.path/'source')
    selected=None
    for stamp,qpos in reader.frames():
        if stamp+1e-9>=time_s:
            selected=(stamp,qpos);break
    if selected is None:raise ValueError('Requested time exceeds the recording')
    stamp,qpos=selected;play=FactoryPlayback(factory);case=reader.meta['case_id']
    ids={g['id']:g['source_index'] for g in source.manifest['geometries']}
    rgba={name:reader.meta['initial_rgba'][index] for name,index in ids.items()}
    for event in reader.events():
        if event['sim_time_s']>stamp+1e-9:break
        if event['event']=='rgba':rgba.update(event['geoms'])
    play.appearance(case,rgba);expected=play.apply(case,source,qpos,stamp)
    names=play.manifest['workstation_names'][case]['geom_names'];geoms=[];error=0.
    for name,target in names.items():
        g=play.model.geom(target);index=g.id
        error=max(error,float(np.linalg.norm(play.data.geom_xpos[index]-expected['geometries'][name]['position_m'])))
        geoms.append(dict(id=index,name=target,position=play.data.geom_xpos[index].tolist(),
            rotation=play.data.geom_xmat[index].reshape(3,3).tolist(),rgba=play.model.geom_rgba[index].tolist()))
    if error>1e-6:raise ValueError('Factory recorded pose differs from source FK')
    place=next(p for p in play.manifest['placements'] if p['case_id']==case)
    yaw=place['yaw_rad'];r=np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1.]])
    shift=np.asarray(place['translation_m']);cid=source.model.camera('global').id
    camera_pos=source.model.cam_pos[cid];camera_r=quat_matrix(source.model.cam_quat[cid])
    result=dict(schema='robofrontier.factory-recorded-pose.v1',case_id=case,
        factory_scene_sha256=play.manifest['scene_sha256'],source_scene_sha256=reader.meta['source_scene_sha256'],
        motion_index_sha256=hashlib.sha256((reader.path/'stream.json').read_bytes()).hexdigest(),
        recording=str(reader.path.resolve()),requested_time_s=time_s,selected_recorded_time_s=stamp,
        physics_stepped=False,maximum_factory_position_error_m=error,geoms=geoms,
        camera=dict(id='recorded_task',position_m=(r@camera_pos+shift).tolist(),
            target_m=(r@(camera_pos-camera_r[:,2])+shift).tolist(),projection='perspective',
            vertical_fov_deg=float(source.model.cam_fovy[cid]),hidden_layers=['roof']))
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x') as file:json.dump(result,file,separators=(',',':'));file.write('\n')
    print(json.dumps(dict(output=str(output),case_id=case,recorded_time_s=stamp,geometry_count=len(geoms),maximum_position_error_m=error)))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('recording',type=Path);p.add_argument('--factory',type=Path,default=Path('outputs/robo-factory/v6-workstations/scene.xml'))
    p.add_argument('--output',type=Path,required=True);p.add_argument('--time-s',type=float,required=True)
    a=p.parse_args();export(a.recording,a.factory,a.output,a.time_s)
