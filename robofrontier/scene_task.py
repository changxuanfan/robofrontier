"""Frozen generated-scene tasks with shared goals; no allocation or planner."""
from collections import Counter
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]


def validate_scene_task(task):
    from .task_config import check_position
    if task.get('kind')!='shared_grid' or not isinstance(task['arms'],dict) or not task['arms']:
        raise ValueError('Generated scene task requires shared_grid and named arms')
    folder,env=validate_scene_asset(task)
    if task['objects']!=env['objects']:raise ValueError('Object definitions differ from generated scene')
    if len({o['id'] for o in task['objects']})!=len(task['objects']):raise ValueError('Duplicate objects')
    for obj in task['objects']:check_position(obj['initial_position_m'])
    goals=task['shared_goals']
    if not goals or len({g['goal_id'] for g in goals})!=len(goals):raise ValueError('Invalid shared goal IDs')
    for goal in goals:
        check_position(goal['position_m'])
        if set(goal['requires'])!={'color','shape'} or goal['requires']['shape']!='cube':raise ValueError('Grid goals require color and cube shape')
    available=Counter((o['color'],o['shape']) for o in task['objects'])
    required=Counter((g['requires']['color'],g['requires']['shape']) for g in goals)
    if required-available:raise ValueError('Insufficient inventory')
    completion=task['completion']
    for key in ('xy_tolerance_m','z_tolerance_m','max_linear_speed_m_s','stable_duration_sim_s','max_sample_gap_sim_s','axis_alignment_tolerance_rad'):
        if not math.isfinite(completion[key]) or completion[key]<=0:raise ValueError('Invalid completion criterion')
    if completion['max_sample_gap_sim_s']<.02:raise ValueError('Samples are at the 20 ms control tick')
    mode=completion.get('mode','pose')
    if mode not in ('pose','pocket_seating'):raise ValueError('Unknown completion mode')
    if mode=='pocket_seating':
        grid=task.get('grid',{});fixture=grid.get('fixture',{})
        if grid!=env.get('grid') or fixture.get('kind')!='recessed_square_pockets':
            raise ValueError('Pocket evaluation requires the generated fixture geometry')
        for key in ('containment_tolerance_m','upright_tolerance_rad','max_angular_speed_rad_s'):
            if not math.isfinite(completion.get(key,float('nan'))) or completion[key]<=0:
                raise ValueError('Invalid pocket completion criterion')
        if not 0<grid['cube_side_m']<fixture['opening_side_m'] or not 0<fixture['depth_m']<grid['cube_side_m']:
            raise ValueError('Invalid pocket geometry')
        for goal in goals:
            row,col=goal.get('row'),goal.get('column')
            if type(row) is not int or type(col) is not int or not 1<=row<=grid['rows'] or not 1<=col<=grid['columns']:
                raise ValueError('Pocket goals require valid row and column')
            expected=[grid['center_world_m'][0]+(col-(grid['columns']+1)/2)*grid['cell_side_m'],
                      grid['center_world_m'][1]+((grid['rows']+1)/2-row)*grid['cell_side_m'],
                      fixture['floor_z_m']+grid['cube_side_m']/2]
            if any(abs(a-b)>1e-6 for a,b in zip(goal['position_m'],expected)):
                raise ValueError('Pocket goal differs from the physical pocket center')
    return task


def validate_scene_asset(task):
    """Shared static asset checks; task-specific goals are validated separately."""
    from .task_config import check_position
    import re
    folder=(ROOT/task['scene_asset']).resolve()
    if not folder.is_relative_to(ROOT/'configs/environments') or not folder.is_dir():
        raise ValueError('Use a generated scene directory under configs/environments')
    if hashlib.sha256((folder/'scene.xml').read_bytes()).hexdigest()!=task['scene_sha256']:
        raise ValueError('Generated scene differs from the requested task')
    env=json.loads((folder/'environment.json').read_text())
    if task['arms']!={s['id']:{'base_position_m':s['base_position_m'],'yaw_rad':s['yaw_rad']} for s in env['stations']}:
        raise ValueError('Robot bases differ from generated scene')
    for name,base in task['arms'].items():
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*',name) or name in ('Meta','Coordinator','Controller','research'):
            raise ValueError('Invalid robot ID')
        check_position(base['base_position_m'])
        if not math.isfinite(base['yaw_rad']):raise ValueError('Invalid base yaw')
    lower,upper=(task['workspace_world_m'][k] for k in ('lower','upper'))
    check_position(lower);check_position(upper)
    if any(a>=b for a,b in zip(lower,upper)):raise ValueError('Invalid world command bounds')
    actual={c.get('name') for c in ET.parse(folder/'scene.xml').findall('worldbody/camera')}
    if len(task['cameras'])!=len(set(task['cameras'])) or set(task['cameras'])!=actual:
        raise ValueError('Camera list differs from generated scene')
    return folder,env


def freeze_scene(task,directory):
    """Copy the actual scene/meshes into the run; preserve relative asset paths."""
    task=deepcopy(task);source=ROOT/task['scene_asset'];dest=directory/'private/inputs/scene'
    dest.mkdir(parents=True);shutil.copytree(source/'assets',dest/'assets')
    root=ET.parse(source/'scene.xml').getroot();compiler=root.find('compiler')
    if (source/'workstation.json').exists():
        # Keep the exact canonical bytes/hash used by offline factory replay.
        for name in ('scene.xml','workstation.json','environment.json','task.json'):
            shutil.copy2(source/name,dest/name)
    else:
        meshes=(source/compiler.get('meshdir')).resolve()
        shutil.copytree(meshes,dest/'model-assets');compiler.set('meshdir','model-assets')
        ET.indent(root);ET.ElementTree(root).write(dest/'scene.xml',encoding='unicode')
    names=(('target-pattern-draft.png','initial-top.png','initial-top.json')
           if task['kind']=='shared_grid' else ('preview-board.png',) if task['kind']=='spring_dial'
           else ('preview-global.png','preview-global_rear.png','preview-top.png') if task['kind']=='frontier_suite'
           else ('preview-top.png',))
    for name in names:
        if task['kind']=='frontier_suite' and not (source/name).is_file():continue
        shutil.copy2(source/name,directory/'public/inputs'/name)
    if task['kind']=='frontier_suite' and task['family']=='mosaic':shutil.copy2(source/'target-pattern.png',directory/'public/inputs/target-pattern.png')
    for name in ('robot-layout.png','camera-guide.json'):
        if (source/name).is_file():shutil.copy2(source/name,directory/'public/inputs'/name)
    task['scene_asset']='private/inputs/scene/scene.xml'
    task['scene_sha256']=hashlib.sha256((dest/'scene.xml').read_bytes()).hexdigest()
    return task


def camera_names(task):
    return task.get('cameras',['global','global_side']+[a+'_'+s for a in task['arms'] for s in ('front','side')])


def sample_shared(world):
    import mujoco
    import numpy as np
    support=set();contact=set();floor_support=set()
    for c in world.data.contact:
        for geom,other in ((c.geom1,c.geom2),(c.geom2,c.geom1)):
            name=world.model.body(world.model.geom_bodyid[geom]).name
            if name not in world.object_names:continue
            if (world.model.geom(other).name or '').startswith('table_'):support.add(name)
            if (world.model.geom(other).name=='table_pocket_floor' and c.efc_address>=0
                    and abs(c.frame[2])>.9):floor_support.add(name)
            if world.model.geom_bodyid[other] in world.robot_bodies:contact.add(name)
    states={}
    for name in world.object_names:
        body=world.model.body(name).id;v=np.zeros(6)
        mujoco.mj_objectVelocity(world.model,world.data,mujoco.mjtObj.mjOBJ_BODY,body,v,0)
        rotation=world.data.body(body).xmat.reshape(3,3)
        # Axis-aligned cubes are invariant under quarter turns and face flips.
        alignment=math.acos(float(np.clip(np.min(np.max(np.abs(rotation),axis=0)),-1,1)))
        states[name]={'position_m':world.data.body(body).xpos.copy().tolist(),'linear_velocity_m_s':v[3:].tolist(),
                      'angular_velocity_rad_s':v[:3].tolist(),
                      'quaternion_wxyz':world.data.body(body).xquat.copy().tolist(),
                      'table_supported':name in support,'pocket_floor_supported':name in floor_support,'released':name not in contact,
                      'axis_alignment_error_rad':alignment}
    world.evaluation_samples.append({'time_s':world.time,'objects':states})


def pocket_eligible(obj, goal, state, criteria, grid):
    """Private geometry check of the full cube footprint, not a policy tool."""
    import mujoco
    import numpy as np
    if (state is None or any(obj[k]!=v for k,v in goal['requires'].items())
            or not state.get('released') or not state.get('pocket_floor_supported')):return False
    p=np.asarray(state.get('position_m',[]));v=np.asarray(state.get('linear_velocity_m_s',[]))
    q=np.asarray(state.get('quaternion_wxyz',[]));omega=np.asarray(state.get('angular_velocity_rad_s',[]));target=np.asarray(goal['position_m'])
    if (p.shape!=(3,) or v.shape!=(3,) or omega.shape!=(3,) or q.shape!=(4,) or not all(np.isfinite(a).all() for a in (p,v,q,omega))
            or np.linalg.norm(q)<1e-9):return False
    r=np.empty(9);mujoco.mju_quat2Mat(r,q/np.linalg.norm(q));r=r.reshape(3,3)
    f=grid['fixture'];half=grid['cube_side_m']/2;tol=criteria['containment_tolerance_m']
    extents=half*np.sum(np.abs(r),axis=1)
    return bool(np.linalg.norm(v)<=criteria['max_linear_speed_m_s']
        and np.linalg.norm(omega)<=criteria['max_angular_speed_rad_s']
        and np.max(np.abs(r[2]))>=math.cos(criteria['upright_tolerance_rad'])
        and np.all(np.abs(p[:2]-target[:2])+extents[:2]<=f['opening_side_m']/2+tol)
        and abs(p[2]-extents[2]-f['floor_z_m'])<=tol
        and abs(p[2]-target[2])<=criteria['z_tolerance_m'])


def evaluate_shared(world):
    from .cube_transport import evaluate_placement_window
    samples=deepcopy(list(world.evaluation_samples))
    for sample in samples:
        sample['objects']={name:state for name,state in sample['objects'].items()
                           if state['axis_alignment_error_rad']<=world.task['completion']['axis_alignment_tolerance_rad']}
    strict=evaluate_placement_window({'objects':world.task['objects']},
        {'orders':{'shared':world.task['shared_goals']},'completion':world.task['completion']},samples)
    result=strict
    if world.task['completion'].get('mode')=='pocket_seating':
        result=evaluate_placement_window({'objects':world.task['objects']},
            {'orders':{'shared':world.task['shared_goals']},'completion':world.task['completion']},
            list(world.evaluation_samples),eligible=lambda obj,goal,state,c:pocket_eligible(obj,goal,state,c,world.task['grid']))
        result.update(metric='pocket_seating',seated_goals=result['matched_goals'],strict_pose_diagnostic=strict)
    result.update(all_goals_met=result['complete'],task_status=world.task.get('status'),scope='Private stable pose/color/release check; draft target, no policy object-state feedback')
    return result
