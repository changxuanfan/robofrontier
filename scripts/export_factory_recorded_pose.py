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
from robofrontier.motion_edit import MotionEdit
from robofrontier.robot_control import quat_matrix
from robofrontier.workstation import Workstation


def export(recording, factory, output, time_s=None, edit_timeline=None, output_frame=None):
    reader=MotionReader(recording);source=Workstation(reader.path/'source')
    edit=None;selected_index=None
    if output_frame is not None:
        if edit_timeline is None or time_s is not None:raise ValueError('Edited output frame requires an edit timeline and no source-time override')
        edit=MotionEdit(edit_timeline,reader)
        if not isinstance(output_frame,int) or not 0<=output_frame<edit.data['output_frames']:raise ValueError('Edited output frame outside timeline')
        row=edit.data['render_frames'][output_frame];selected_index=row['source_frame'];time_s=row['source_time_s']
    elif edit_timeline is not None:
        raise ValueError('Specify --output-frame to use the shared edit timeline')
    if time_s is None or not np.isfinite(time_s) or time_s<0:raise ValueError('Nonnegative finite recorded time required')
    selected=None
    for index,(stamp,qpos) in enumerate(reader.frames()):
        matches=(index==selected_index) if selected_index is not None else (stamp+1e-9>=time_s)
        if matches:
            selected=(stamp,qpos);break
    if selected is None:raise ValueError('Requested time exceeds the recording')
    if edit and abs(selected[0]-time_s)>1e-8:raise ValueError('Motion edit source timestamp differs')
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
    if edit:result.update(edit_timeline_sha256=hashlib.sha256(Path(edit_timeline).read_bytes()).hexdigest(),output_frame=output_frame,output_time_s=output_frame/edit.data['fps'],selected_source_frame=selected_index)
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x') as file:json.dump(result,file,separators=(',',':'));file.write('\n')
    print(json.dumps(dict(output=str(output),case_id=case,recorded_time_s=stamp,geometry_count=len(geoms),maximum_position_error_m=error)))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('recording',type=Path);p.add_argument('--factory',type=Path,default=Path('outputs/robo-factory/v6-workstations/scene.xml'))
    p.add_argument('--output',type=Path,required=True);group=p.add_mutually_exclusive_group(required=True);group.add_argument('--time-s',type=float);group.add_argument('--output-frame',type=int);p.add_argument('--edit-timeline',type=Path)
    a=p.parse_args();export(a.recording,a.factory,a.output,a.time_s,a.edit_timeline,a.output_frame)
