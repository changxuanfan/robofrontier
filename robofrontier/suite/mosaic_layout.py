"""R11 mosaic corner-facing layout, cloned from immutable R10 workstations.

Only mounting poses, associated cameras/labels and necessary initial stock
clearance change. This module never steps physics or runs an Oracle.
"""
from copy import deepcopy
import hashlib
import json
import math
import shutil
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from . import v2
from ..robot_control import matrix_quat
from ..run_setup import write_json


def placements(stations):
    n=len(stations); per=n//2; result=[]
    for index,s in enumerate(stations):
        new=deepcopy(s); col=index%per; side=-1 if index<per else 1
        outer=col in (0,per-1); hand=-1 if col==0 else 1
        delta=np.zeros(3)
        if n==4:
            delta[:2]=[hand*.10,side*.10]
        elif outer:
            delta[0]=hand*.20
        elif n==8:
            delta[0]=-.05 if col==1 else .05
        turn=math.radians(30 if n==4 else 45) if outer else 0.
        yaw=s['yaw_rad']-side*hand*turn
        if outer:
            forward=np.array([math.cos(yaw),math.sin(yaw)])
            if forward[0]*(-hand)<=0 or forward[1]*(-side)<=0:
                raise ValueError('Corner robot must point inward in both tabletop axes')
        new['base_position_m']=(np.asarray(s['base_position_m'])+delta).tolist()
        new['yaw_rad']=yaw
        rotation=v2.rz(yaw-s['yaw_rad'])
        new['home_tcp_m']=(np.asarray(new['base_position_m'])+rotation@(np.asarray(s['home_tcp_m'])-s['base_position_m'])).tolist()
        result.append(new)
    return result


def transform(root,env):
    world=root.find('worldbody'); old=deepcopy(env['stations']); new=placements(old)
    changes=[]
    for a,b in zip(old,new):
        name=a['id']; rotation=v2.rz(b['yaw_rad']-a['yaw_rad'])
        origin=np.asarray(a['base_position_m']); target=np.asarray(b['base_position_m'])
        base=world.find(f"body[@name='{name}_link0']")
        base.set('pos',v2.vector(target));base.set('quat',v2.vector(matrix_quat(v2.rz(b['yaw_rad']))))
        label=world.find(f"geom[@name='{name}_label']")
        label.set('pos',v2.vector(target+rotation@(np.fromstring(label.get('pos'),sep=' ')-origin)))
        label.set('quat',v2.vector(matrix_quat(v2.rz(b['yaw_rad']))))
        for cam in world.findall('camera'):
            if not cam.get('name','').startswith(name+'_'):continue
            cam.set('pos',v2.vector(target+rotation@(np.fromstring(cam.get('pos'),sep=' ')-origin)))
            if cam.get('xyaxes'):
                axes=np.fromstring(cam.get('xyaxes'),sep=' ').reshape(2,3)
                cam.set('xyaxes',v2.vector((rotation@axes.T).T.ravel()))
        changes.append(dict(robot=name,previous_base_m=a['base_position_m'],base_m=b['base_position_m'],
            translation_m=(target-origin).tolist(),previous_yaw_deg=math.degrees(a['yaw_rad']),yaw_deg=math.degrees(b['yaw_rad'])))
    env['stations']=new
    env['mechanism']['layout_revision']=dict(name='corner-facing-r11',source_revision=10,
        rows=2,outer_turn_deg=30 if len(new)==4 else 45,
        outer_x_shift_m=.10 if len(new)==4 else .20,
        row_outward_shift_m=.10 if len(new)==4 else 0.,
        inner_pair_separation_increase_m=.10 if len(new)==8 else 0.,
        stock_policy='Retain original positions unless initial robot clearance requires relocation',changes=changes)
    # The spacing is no longer uniform within a row.
    env['mechanism'].pop('station_pitch_m',None)
    env['mechanism']['station_x_positions_m']=[s['base_position_m'][0] for s in new[:len(new)//2]]
    env['mechanism']['robot_to_grid_edge_m']=abs(new[0]['base_position_m'][1])-env['grid']['rows']*env['grid']['pitch_m']/2


def clear_stock(folder,env,reference_positions=None):
    """Static conservative robot AABBs at stock height; never simulate a grasp."""
    path=folder/'scene.xml';root=ET.parse(path).getroot();world=root.find('worldbody')
    model=mujoco.MjModel.from_xml_path(str(path));data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id);mujoco.mj_forward(model,data)
    side=env['grid']['cube_side_m'];z=env['physics']['table_height_m'];blocked=[]
    from .geometry_checks import owner_map
    owners=owner_map(model,[s['id'] for s in env['stations']])
    for g,owner in owners.items():
        if owner is None or not(model.geom_contype[g] or model.geom_conaffinity[g]):continue
        rotation=data.geom_xmat[g].reshape(3,3)
        center=data.geom_xpos[g]+rotation@model.geom_aabb[g,:3]
        extent=abs(rotation)@model.geom_aabb[g,3:]
        if center[2]+extent[2]>=z and center[2]-extent[2]<=z+side:
            blocked.append((center[:2]-extent[:2]-side/2-.008,center[:2]+extent[:2]+side/2+.008))
    hx,hy=env['tables'][0]['half_size_m'][:2]
    w=env['grid']['columns']*env['grid']['pitch_m']/2;h=env['grid']['rows']*env['grid']['pitch_m']/2
    bases=np.asarray([s['base_position_m'][:2] for s in env['stations']])
    occupied=[]
    def clear(point):
        p=np.asarray(point[:2])
        if abs(p[0])+side/2>hx-.015 or abs(p[1])+side/2>hy-.015:return False
        if abs(p[0])<w+side/2+.055 and abs(p[1])<h+side/2+.055:return False
        if any(np.all(p>lo) and np.all(p<hi) for lo,hi in blocked):return False
        if len(occupied) and np.any(np.max(abs(np.asarray(occupied)-p),axis=1)<side+.015-1e-8):return False
        return np.linalg.norm(bases-p,axis=1).min()<.74
    candidates=[np.array([x,y]) for y in np.arange(-hy+.06,hy-.059,.075) for x in np.arange(-hx+.06,hx-.059,.075)]
    positions={};changed=[];deferred=[]
    for obj in env['objects']:
        p=np.asarray((reference_positions or {}).get(obj['id'],obj['initial_position_m']))
        if clear(p):positions[obj['id']]=p;occupied.append(p[:2])
        else:deferred.append(obj)
    for obj in deferred:
        original=np.asarray(obj['initial_position_m']);valid=[p for p in candidates if clear(p)]
        if not valid:raise ValueError('No non-overlapping initial stock position: '+obj['id'])
        p=min(valid,key=lambda p:np.linalg.norm(p-original[:2]))
        pos=np.r_[p,z+side/2];positions[obj['id']]=pos;occupied.append(p)
    for obj in env['objects']:
        old=obj['initial_position_m'];pos=positions[obj['id']].tolist()
        if np.linalg.norm(np.asarray(old)-pos)>1e-9:changed.append(dict(object=obj['id'],previous_m=old,position_m=pos))
        obj['initial_position_m']=pos
        world.find(f"body[@name='{obj['id']}']").set('pos',v2.vector(pos))
        jid=model.joint(obj['id']+'_free').id;addr=int(model.jnt_qposadr[jid])
        for key in root.findall('keyframe/key'):
            q=np.fromstring(key.get('qpos'),sep=' ');q[addr:addr+3]=pos;key.set('qpos',v2.vector(q))
    ET.indent(root);ET.ElementTree(root).write(path,encoding='unicode')
    env['mechanism']['layout_revision']['relocated_stock_count']=len(changed)
    write_json(folder/'stock-relocations.json',dict(scope='Static initial layout construction only; no live object writes',changes=changed))
    return {k:v.tolist() for k,v in positions.items()}


def inward_transform(root,env):
    """Move only each row's end robots 50 mm along their existing facing."""
    world=root.find('worldbody');n=len(env['stations']);per=n//2;changes=[]
    for index,s in enumerate(env['stations']):
        outer=index%per in (0,per-1)
        delta=.05*np.array([math.cos(s['yaw_rad']),math.sin(s['yaw_rad']),0.]) if outer else np.zeros(3)
        before=s['base_position_m'][:]
        s['base_position_m']=(np.asarray(before)+delta).tolist()
        s['home_tcp_m']=(np.asarray(s['home_tcp_m'])+delta).tolist()
        name=s['id']
        nodes=[world.find(f"body[@name='{name}_link0']"),world.find(f"geom[@name='{name}_label']")]
        nodes += [c for c in world.findall('camera') if c.get('name','').startswith(name+'_')]
        for node in nodes:node.set('pos',v2.vector(np.fromstring(node.get('pos'),sep=' ')+delta))
        changes.append(dict(robot=name,previous_base_m=before,base_m=s['base_position_m'],translation_m=delta.tolist(),yaw_deg=math.degrees(s['yaw_rad'])))
    env['mechanism']['layout_revision']=dict(name='corner-facing-r12-inward50mm',source_revision=11,
        inward_distance_m=.05,direction='Existing robot forward axis in tabletop plane',
        unchanged='All yaw angles; middle robots in six/eight-arm scenes',changes=changes)
    env['mechanism']['station_x_positions_m']=[s['base_position_m'][0] for s in env['stations'][:per]]
    h=env['grid']['rows']*env['grid']['pitch_m']/2
    gaps={s['id']:abs(s['base_position_m'][1])-h for s in env['stations']}
    env['mechanism']['robot_to_grid_edge_m_by_robot']=gaps
    env['mechanism']['robot_to_grid_edge_m']=min(gaps.values())


def finish(case,folder,env):
    root=ET.parse(folder/'scene.xml').getroot();model=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'));data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id);mujoco.mj_forward(model,data)
    from scripts.export_frontier_contact_sheets import fit_camera
    for view in ('top','global','global_rear'):
        fit_camera(model,data,view)
        root.find(f"worldbody/camera[@name='{view}']").set('pos',v2.vector(model.cam_pos[model.camera(view).id]))
    ET.indent(root);ET.ElementTree(root).write(folder/'scene.xml',encoding='unicode')
    model=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'));data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id);mujoco.mj_forward(model,data)
    write_json(folder/'environment.json',env)
    task=v2.task_for(case,env,folder);write_json(folder/'task.json',task);write_json(folder/'public-task.json',v2.public_task(task))
    write_json(folder/'parameters.json',dict(physics=env['physics'],mechanism=env['mechanism'],grid=env['grid']))
    state=v2.manifest(model,data,case,folder,env)
    contacts=[]
    from .geometry_checks import owner_map
    owners=owner_map(model,task['arms'])
    for contact in data.contact:
        a,b=map(int,contact.geom)
        if contact.dist<-.0005 and (owners[a] or owners[b]):
            contacts.append(dict(a=model.body(int(model.geom_bodyid[a])).name,b=model.body(int(model.geom_bodyid[b])).name,gap_m=float(contact.dist)))
    validation=dict(status='static_layout_preview',oracle_verified=False,policy_verified=False,frozen_input=False,
        robot_count=case['arms'],physics_sha256=state['physics_sha256'],initial_robot_penetrations=contacts,
        physics_steps=0,scope='Static compilation and initial contact screen only; no manipulation or all-cell reachability certification')
    write_json(folder/'validation.json',validation)
    v2.validate_task(task)
    if contacts:raise ValueError('Initial robot penetration: '+str(contacts[:5]))
    return model,data


def build(case,reference_positions=None,replace_draft=False):
    from .reframe import build as clone
    folder=clone(case,replace_draft,False,transform=inward_transform if case['scene_revision']==12 else transform)
    env=json.loads((folder/'environment.json').read_text())
    positions=clear_stock(folder,env,reference_positions)
    model,data=finish(case,folder,env)
    source=v2.directory(dict(case,scene_revision=case['source_scene_revision']))
    if (source/'source-pattern.png').exists():shutil.copyfile(source/'source-pattern.png',folder/'source-pattern.png')
    return folder,model,data,positions
