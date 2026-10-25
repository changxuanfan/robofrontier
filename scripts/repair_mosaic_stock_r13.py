"""Minimally relocate R12 stock using the unchanged geometric coverage gate.

All searches use engineering scratch data. New immutable scene inputs are
written only after a complete supply-and-goal pass. No physics is stepped.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import shutil
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from scripts.check_mosaic_coverage_r12 import Coverage,geometric_forward
from robofrontier.run_setup import write_json
from robofrontier.suite import v2
from robofrontier.suite.mosaic_layout import finish
from robofrontier.suite.reframe import build as clone

OUT=v2.ROOT/'runs/engineering/mosaic-r13-stock-20261025'


def candidate_positions(c,obj):
    p=np.asarray(obj['initial_position_m']);table=json.loads((c.folder/'environment.json').read_text())['tables'][0]
    hx,hy=table['half_size_m'][:2];grid=c.task['grid'];w=grid['columns']*grid['pitch_m']/2;h=grid['rows']*grid['pitch_m']/2
    bases=np.asarray([s['base_position_m'][:2] for s in c.task['arms'].values()])
    other=np.asarray([o['initial_position_m'][:2] for o in c.task['objects'] if o['id']!=obj['id']])
    local=[p[:2]+[x,y] for x in np.arange(-.30,.301,.03) for y in np.arange(-.30,.301,.03)]
    global_points=[np.array([x,y]) for x in np.arange(-hx+.06,hx-.059,.06) for y in np.arange(-hy+.06,hy-.059,.06)]
    choices={}
    for xy in local+global_points:
        if abs(xy[0])>hx-.045 or abs(xy[1])>hy-.045:continue
        if abs(xy[0])<w+.10 and abs(xy[1])<h+.10:continue
        distance=np.linalg.norm(bases-xy,axis=1).min()
        if not .33<distance<.73:continue
        # More space than the original75mm packing, without changing cubes.
        if np.max(abs(other-xy),axis=1).min()<.0899:continue
        key=tuple(np.round(xy,6));choices[key]=np.r_[xy,p[2]]
    return sorted(choices.values(),key=lambda q:(np.linalg.norm(q-p),abs(np.linalg.norm(bases-q[:2],axis=1).min()-.49)))


def initial_target_clear(c,name):
    c.d.qpos[:]=c.q0;geometric_forward(c.m,c.d);bid=c.m.body(name).id
    for contact in c.d.contact:
        a,b=map(int,contact.geom);ba,bb=map(int,c.m.geom_bodyid[[a,b]])
        if bid not in (ba,bb) or contact.dist>=-.0005:continue
        other=bb if ba==bid else ba
        if other!=0:return False
    return True


def move_one(c,name,trace):
    obj=next(o for o in c.task['objects'] if o['id']==name);previous=obj['initial_position_m'][:]
    addr=int(c.m.jnt_qposadr[c.m.joint(name+'_free').id]);saved=c.q0[addr:addr+3].copy()
    positions=candidate_positions(c,obj)
    for i,point in enumerate(positions[:600]):
        c.q0[addr:addr+3]=point;c.hand_cache.clear()
        if not initial_target_clear(c,name):continue
        trial=dict(obj,initial_position_m=point.tolist());result=c.probe('source',trial)
        if not result['confirmed']:continue
        obj['initial_position_m']=point.tolist()
        trace.append(dict(object=name,previous_m=previous,position_m=point.tolist(),candidate_index=i,
            displacement_m=float(np.linalg.norm(point-saved)),witness=result['witness']))
        print('MOVE',name,'distance',round(np.linalg.norm(point-saved),3),'candidate',i,flush=True)
        return
    c.q0[addr:addr+3]=saved;c.hand_cache.clear()
    raise RuntimeError('No admitted stock placement in bounded search '+name+'; candidates='+str(len(positions)))


def full_check(c):
    result={}
    for kind,items,key in [('source',c.task['objects'],'sources'),('goal',c.task['goals'],'goals')]:
        rows=[]
        for i,obj in enumerate(items):
            rows.append(c.probe(kind,obj))
            if (i+1)%30==0:print('VERIFY',c.task['id'],kind,i+1,'/',len(items),flush=True)
        result[key]=rows
    result['passed']=all(x['confirmed'] for key in ('sources','goals') for x in result[key])
    return result


def optimize(arms):
    cases=json.loads((v2.ROOT/'configs/suites/frontier-mosaic-r12.json').read_text())['cases']
    case=next(c for c in cases if c['arms']==arms and c['level']==1)
    folder=v2.ROOT/case['directory'];c=Coverage(folder);start=time.monotonic();out=OUT/f'a{arms:02}';out.mkdir(parents=True,exist_ok=True)
    old=json.loads((v2.ROOT/'runs/engineering/mosaic-r12-coverage-20261025'/case['id']/'report.json').read_text())
    assert old['complete'] and old['source_scene_sha256']==c.scene_hash
    trace=[];pending=[x['id'] for x in old['sources'] if not x['confirmed']]
    original={o['id']:deepcopy(o) for o in c.task['objects']}
    for cycle in range(1,6):
        print('CYCLE',arms,cycle,'pending',pending,flush=True)
        for name in dict.fromkeys(pending):
            move_one(c,name,trace)
            write_json(out/'progress.json',dict(cycle=cycle,moves=trace,elapsed_s=time.monotonic()-start))
        c.hand_cache.clear();checked=full_check(c)
        write_json(out/f'cycle-{cycle:02}-coverage.json',checked)
        if checked['passed']:break
        pending=[x['id'] for x in checked['sources'] if not x['confirmed']]
        for row in checked['goals']:
            if row['confirmed']:continue
            failure=row.get('last_failure') or {}
            pending.extend(x for x in (failure.get('a'),failure.get('b')) if x in original)
        if not pending:raise RuntimeError('Unconfirmed goal route without an identified supply blocker')
    else:raise RuntimeError('Bounded layout repair did not reach full coverage')
    positions={o['id']:o['initial_position_m'] for o in c.task['objects']}
    changed=[dict(object=name,previous_m=obj['initial_position_m'],position_m=positions[name])
        for name,obj in original.items() if np.linalg.norm(np.asarray(obj['initial_position_m'])-positions[name])>1e-9]
    write_json(out/'admitted-layout.json',dict(arms=arms,source_scene_sha256=c.scene_hash,positions=positions,changes=changed,
        complete=True,geometry_passed=True,physics_steps=0,moves=trace,elapsed_s=time.monotonic()-start))
    # The same stock positions are used across all three difficulty variants.
    built=[]
    for source in [x for x in cases if x['arms']==arms]:
        new=dict(source,source_scene_revision=12,scene_revision=13)
        new['directory']=str(v2.directory(new).relative_to(v2.ROOT))
        raw=mujoco.MjModel.from_xml_path(str(v2.ROOT/source['directory']/'scene.xml'))
        def transform(root,env):
            world=root.find('worldbody')
            for obj in env['objects']:
                name=obj['id'];point=positions[name];obj['initial_position_m']=point
                world.find(f"body[@name='{name}']").set('pos',v2.vector(point))
                addr=int(raw.jnt_qposadr[raw.joint(name+'_free').id])
                for key in root.findall('keyframe/key'):
                    q=np.fromstring(key.get('qpos'),sep=' ');q[addr:addr+3]=point;key.set('qpos',v2.vector(q))
            env['mechanism']['stock_layout_revision']=dict(name='individually-graspable-r13',source_revision=12,
                changed_stock_count=len(changed),robot_poses_changed=False)
        target=clone(new,False,False,transform=transform)
        env=json.loads((target/'environment.json').read_text());finish(new,target,env)
        oldfolder=v2.ROOT/source['directory']
        if (oldfolder/'source-pattern.png').exists():shutil.copyfile(oldfolder/'source-pattern.png',target/'source-pattern.png')
        # Immutable robot/control/material and goal properties must agree.
        model=mujoco.MjModel.from_xml_path(str(target/'scene.xml'))
        for field in ['body_mass','body_inertia','geom_friction','jnt_range','actuator_gainprm','actuator_biasprm','actuator_forcerange']:
            assert np.array_equal(getattr(model,field),getattr(raw,field)),field
        oldtask=json.loads((oldfolder/'task.json').read_text());task=json.loads((target/'task.json').read_text())
        assert task['arms']==oldtask['arms'] and task['goals']==oldtask['goals'] and task['grid']==oldtask['grid']
        assert model.opt.timestep==raw.opt.timestep
        write_json(target/'stock-relocations.json',dict(scope='Initial environment construction, no physical execution',changes=changed))
        built.append(new)
    write_json(out/'cases.json',built)
    print('DONE',arms,'unique_stock_moved',len(changed),'seconds',round(time.monotonic()-start,1),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--arms',type=int,choices=[4,6,8],required=True);a=p.parse_args();optimize(a.arms)
