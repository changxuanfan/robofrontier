"""Raise an immutable workstation without scaling its manipulation geometry.

Only root world poses, absolute task metadata and table legs change. Ground
stays at Z=0. Source models and asset caches are never edited in place.
"""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image

from ..run_setup import write_json


def shift_metadata(value,delta):
    world_points={'position_m','initial_position_m','base_position_m','home_tcp_m',
                  'initial_center_m','pickup_center_m','center_world_m','center_m'}
    world_arrays={'source_positions_m','goal_positions_m','parking_positions_m'}
    if isinstance(value,dict):
        for key,item in value.items():
            if key in world_points and isinstance(item,list) and len(item)==3:
                item[2]+=delta
            elif key in world_arrays:
                for point in item:point[2]+=delta
            elif key in ('floor_z_m','rim_z_m'):value[key]+=delta
            else:shift_metadata(item,delta)
    elif isinstance(value,list):
        for item in value:shift_metadata(item,delta)


def build(case,replace_draft=False,render=True,transform=None):
    from . import v2
    from ..workstation import Workstation
    source_case=deepcopy(case);source_case['scene_revision']=case['source_scene_revision']
    source=v2.directory(source_case);frozen=Workstation(source)
    env=json.loads((source/'environment.json').read_text())
    old_height=env['physics']['table_height_m'];new_height=case['table_height_m'];delta=new_height-old_height
    folder=v2.directory(case)
    if folder.exists():
        validation=json.loads((folder/'validation.json').read_text()) if (folder/'validation.json').exists() else {}
        if not replace_draft or validation.get('frozen_input') or validation.get('oracle_verified'):raise FileExistsError('Immutable/existing source: '+str(folder))
        shutil.rmtree(folder)
    folder.mkdir(parents=True)
    cache=v2.ROOT/'runs/state-assets';cache.mkdir(parents=True,exist_ok=True)
    for asset in frozen.manifest['assets']:
        original=source/asset['path'];cached=cache/asset['sha256']
        if not cached.exists():
            temporary=cache/(asset['sha256']+f'.{os.getpid()}.tmp');shutil.copyfile(original,temporary)
            if hashlib.sha256(temporary.read_bytes()).hexdigest()!=asset['sha256']:raise ValueError('Changed source asset')
            os.replace(temporary,cached)
        target=folder/asset['path'];target.parent.mkdir(parents=True,exist_ok=True)
        try:os.link(cached,target)
        except OSError:shutil.copyfile(cached,target)
    for name in ('target-pattern.png','target-original.png'):
        if (source/name).exists():shutil.copyfile(source/name,folder/name)
    root=ET.parse(source/'scene.xml').getroot();world=root.find('worldbody')
    for element in world:
        if element.tag not in ('body','geom','camera','light','site'):continue
        if element.tag=='geom' and (element.get('type')=='plane' or element.get('name','').startswith('floor')):continue
        p=np.fromstring(element.get('pos','0 0 0'),sep=' ')
        if element.tag=='geom' and element.get('name','').startswith('table_leg_'):
            # Lengthen stationary legs towards the unchanged floor.
            size=np.fromstring(element.get('size'),sep=' ');p[2]+=delta/2;size[2]+=delta/2
            element.set('size',v2.vector(size))
        else:p[2]+=delta
        element.set('pos',v2.vector(p))
    for key in root.findall('keyframe/key'):
        q=np.fromstring(key.get('qpos'),sep=' ')
        for j in range(frozen.model.njnt):
            if frozen.model.jnt_type[j]==mujoco.mjtJoint.mjJNT_FREE:q[frozen.model.jnt_qposadr[j]+2]+=delta
        key.set('qpos',v2.vector(q))
    shift_metadata(env,delta);env['physics']['table_height_m']=new_height;env['suite_case']=deepcopy(case)
    env['source_frame_revision']=dict(source_directory=str(source.relative_to(v2.ROOT)),
        source_scene_sha256=frozen.manifest['scene_sha256'],vertical_translation_m=delta,
        floor_unchanged=True,geometry_scaled=False)
    if transform is not None:
        transform(root,env)
    path=folder/'scene.xml';ET.indent(root);ET.ElementTree(root).write(path,encoding='unicode')
    m=mujoco.MjModel.from_xml_path(str(path));d=mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m,d,m.key('initial').id);mujoco.mj_forward(m,d)
    write_json(folder/'environment.json',env)
    task=v2.task_for(case,env,folder);write_json(folder/'task.json',task);write_json(folder/'public-task.json',v2.public_task(task))
    write_json(folder/'parameters.json',dict(physics=env['physics'],mechanism=env['mechanism'],grid=env.get('grid'),source_frame_revision=env['source_frame_revision']))
    state=v2.manifest(m,d,case,folder,env)
    write_json(folder/'validation.json',dict(status='generated',oracle_verified=False,policy_verified=False,
        frozen_input=False,robot_count=case['arms'],nq=m.nq,nv=m.nv,ngeom=m.ngeom,
        physics_sha256=state['physics_sha256'],scope='Translated frozen source with taller legs; requires current engineering checks'))
    if render:
        with mujoco.Renderer(m,width=1280,height=800,max_geom=max(5000,m.ngeom+1000)) as renderer:
            for view in ('global','top'):
                renderer.update_scene(d,camera=view);Image.fromarray(renderer.render()).save(folder/('preview-'+view+'.png'))
    print(json.dumps(dict(case=case['id'],table_height_m=new_height,nq=m.nq)),flush=True)
    return folder
