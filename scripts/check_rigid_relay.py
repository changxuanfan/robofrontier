"""Passive fixture checks and matched-duration timings, never a task Oracle.

The elevated fixture deliberately clamps the beam root to the test stand. It
is NOT a robot grasp witness and its poses never enter a policy observation.
Actual benchmark scenes retain all free bodies and robot-only actuation.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from robofrontier.physics_health import require_valid_step
from robofrontier.robot_control import PandaArm,matrix_quat
from robofrontier.run_setup import write_json
from robofrontier.suite.v2 import ROOT, cases, directory

OUT = ROOT/f'docs/generated/frontier-v2/relay-r{max(c["scene_revision"] for c in cases() if c["family"]=="relay")}'


def hanging_fixture(case, dt, seconds=5., payload_id='payload_01', initial_hinge_rad=0.):
    source = directory(case)
    full = ET.parse(source/'scene.xml').getroot()
    root = ET.Element('mujoco')
    for tag in ('compiler', 'option', 'asset', 'default'):
        node = full.find(tag)
        if node is not None: root.append(deepcopy(node))
    # Assets remain relative to the original frozen workstation.
    meshdir=Path(full.find('compiler').get('meshdir',''))
    texturedir=Path(full.find('compiler').get('texturedir',''))
    root.find('compiler').set('meshdir', str(source/meshdir))
    root.find('compiler').set('texturedir', str(source/texturedir))
    for node in root.findall('.//*[@file]'):
        if not Path(node.get('file')).is_absolute():
            base=meshdir if node.tag=='mesh' else texturedir if node.tag=='texture' else Path()
            node.set('file', str(source/base/node.get('file')))
    root.find('option').set('timestep', str(dt))
    world = ET.SubElement(root, 'worldbody')
    for name in (payload_id,payload_id+'_weight'):
        node = deepcopy(full.find(f"worldbody/body[@name='{name}']"))
        pos = np.fromstring(node.get('pos'), sep=' ');pos[2] += .8
        node.set('pos', ' '.join(map(str,pos)))
        if name == payload_id:node.remove(node.find('freejoint'))
        world.append(node)
    if case['level']==3:
        # Clamping the root makes it static, which changes MuJoCo's default
        # parent/child collision filtering. Preserve the live joint-bearing
        # exclusion; otherwise the test housing artificially locks the hinge.
        ET.SubElement(ET.SubElement(root,'contact'),'exclude',
                      body1=payload_id,body2=payload_id+'_distal')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    data = mujoco.MjData(model);mujoco.mj_forward(model,data)
    if initial_hinge_rad:
        distal=data.body(payload_id+'_distal');p0=distal.xpos.copy();r0=distal.xmat.reshape(3,3).copy()
        w=data.body(payload_id+'_weight');wp=w.xpos.copy();wr=w.xmat.reshape(3,3).copy()
        data.joint(payload_id+'_flexion').qpos[0]=initial_hinge_rad
        mujoco.mj_kinematics(model,data)
        distal=data.body(payload_id+'_distal');rotation=distal.xmat.reshape(3,3)@r0.T
        q=data.joint(payload_id+'_weight_free').qpos
        q[:3]=distal.xpos+rotation@(wp-p0);q[3:]=matrix_quat(rotation@wr)
        mujoco.mj_forward(model,data)
    hook_geoms=[g for g in range(model.ngeom) if '_hook_' in model.geom(g).name and 'bracket' not in model.geom(g).name]
    hook_body=int(model.geom_bodyid[hook_geoms[0]])
    # Hook ring centre from the evenly spaced capsule centres (in body frame).
    hook_local=np.mean(model.geom_pos[hook_geoms],axis=0)
    max_distance=0.;max_penetration=0.;contact_steps=0;max_hinge=0.;started=time.perf_counter()
    for _ in range(round(seconds/dt)):
        before=float(data.time);mujoco.mj_step(model,data);require_valid_step(data,before)
        hook=data.xpos[hook_body]+data.xmat[hook_body].reshape(3,3)@hook_local
        max_distance=max(max_distance,float(np.linalg.norm(data.body(payload_id+'_weight').xpos-hook)))
        if case['level']==3:max_hinge=max(max_hinge,abs(float(data.joint(payload_id+'_flexion').qpos[0])))
        touched=False
        for con in data.contact:
            a,b=[model.geom(int(g)).name for g in con.geom]
            if ('_hook_' in a and '_eye_' in b) or ('_eye_' in a and '_hook_' in b):
                max_penetration=max(max_penetration,float(-con.dist));touched=True
        contact_steps+=int(touched)
    return dict(case_id=case['id'],payload_id=payload_id,initial_hinge_rad=initial_hinge_rad,
                level=case['level'],timestep_ms=dt*1000,simulation_s=seconds,
                wall_s=time.perf_counter()-started,max_eye_separation_m=max_distance,
                max_ring_contact_penetration_mm=max_penetration*1000,
                contact_step_fraction=contact_steps/round(seconds/dt),
                retained=max_distance<.065,
                hinge_angle_rad=float(data.joint(payload_id+'_flexion').qpos[0]) if case['level']==3 else None,
                max_hinge_angle_rad=max_hinge if case['level']==3 else None,
                numerical_warnings=sum(w.number for w in data.warning),
                scope='Elevated passive beam-root clamp fixture; gravity/contact only, no robot or task execution')


def timed_scene(folder, seconds=.1, repeats=3):
    model=mujoco.MjModel.from_xml_path(str(folder/'scene.xml'))
    task=json.loads((folder/'task.json').read_text());walls=[];overlaps=[]
    for _ in range(repeats):
        data=mujoco.MjData(model);mujoco.mj_resetDataKeyframe(model,data,model.key('initial').id);mujoco.mj_forward(model,data)
        if not walls:
            for con in data.contact:
                names=[model.geom(int(g)).name for g in con.geom]
                if con.dist<-.0002 and any('payload' in n for n in names):overlaps.append(dict(geoms=names,penetration_mm=float(-con.dist*1000)))
        arms=[PandaArm(model,data,a) for a in task['arms']]
        stride=round(.02/model.opt.timestep);started=time.perf_counter()
        for i in range(round(seconds/model.opt.timestep)):
            if i%stride==0:
                for arm in arms:arm.control(.02)
            before=float(data.time);mujoco.mj_step(model,data);require_valid_step(data,before)
        walls.append(time.perf_counter()-started)
    return dict(simulation_s=seconds,repeats=repeats,median_wall_s=float(np.median(walls)),
                wall_samples_s=walls,timestep_ms=model.opt.timestep*1000,nv=model.nv,
                initial_payload_overlaps=overlaps,plugins=model.nplugin,
                source_scene_sha256=hashlib.sha256((folder/'scene.xml').read_bytes()).hexdigest())


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    selected=[c for c in cases() if c['family']=='relay' and c['level']>1]
    fixtures=[]
    for case in selected[:2]:
        task=json.loads((directory(case)/'task.json').read_text());representatives={}
        for item in task['mechanism']['payload_assemblies']:
            mode=item.get('elastic_hinge',{}).get('mode','out_of_plane' if case['level']==3 else 'no_beam_hinge')
            representatives.setdefault(mode,item['id'])
        for mode,ident in representatives.items():
            for dt in (.002,.001):
                row=hanging_fixture(case,dt,payload_id=ident);row['mode']=mode
                fixtures.append(row);print(json.dumps(row),flush=True)
    timings=[]
    for case in selected:
        previous=dict(case,scene_revision=7)
        row=dict(case_id=case['id'],previous_cable=timed_scene(directory(previous)),
                 current_rigid=timed_scene(directory(case)))
        timings.append(row);print(json.dumps(row),flush=True)
    old=sum(r['previous_cable']['median_wall_s'] for r in timings)
    new=sum(r['current_rigid']['median_wall_s'] for r in timings)
    report=dict(scope='Passive engineering checks only; no Oracle or policy; timing excludes loading/rendering/recording. Old/new mechanisms and timesteps differ, so this is design comparison, not an isolated Cable-plugin benchmark.',
                oracle_started=False,fixtures=fixtures,timings=timings,
                previous_wall_sum_s=old,current_wall_sum_s=new,short_passive_speedup=old/new,
                task_success_verified=False,factory_recomposition_pending=True)
    # Keep separately recorded static checks and perturbed fixtures intact.
    write_json(OUT/'timing-checks.json',report)
    return report


if __name__=='__main__':main()
