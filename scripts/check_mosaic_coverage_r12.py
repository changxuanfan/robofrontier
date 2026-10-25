"""Exhaustive item/cell coverage with finite sampled geometric witnesses.

Engineering scratch data only: no actuator commands, mj_step or scene edits.
An unconfirmed point is not a proof of physical impossibility. A witness is
not a dynamic grasp, release, logistics or simultaneous-motion certificate.
"""
import argparse
from copy import deepcopy
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from robofrontier.robot_control import PandaArm,matrix_quat,rotation_vector
from robofrontier.run_setup import write_json
from robofrontier.suite.geometry_checks import owner_map
from robofrontier.suite.mosaic_oracle_v2 import grasp_rotations
from robofrontier.suite.v2 import ROOT

OUT=ROOT/'runs/engineering/mosaic-r12-coverage-20261025'


def geometric_forward(model,data):
    """Collision-only position stages; no unnecessary force/constraint solve.

    Contact signatures checked against mj_forward on40 distinct full-scene
    poses before this optimization. Neither version advances time.
    """
    mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data);mujoco.mj_collision(model,data)


class Coverage:
    def __init__(self,folder):
        self.folder=folder;self.task=json.loads((folder/'task.json').read_text())
        self.scene_hash=hashlib.sha256((folder/'scene.xml').read_bytes()).hexdigest()
        root=ET.parse(folder/'scene.xml').getroot();compiler=root.find('compiler')
        for tag,key in [('mesh','meshdir'),('texture','texturedir')]:
            for asset in root.findall('asset/'+tag+'[@file]'):
                asset.set('file',str((folder/compiler.get(key,'')/asset.get('file')).resolve()))
        compiler.set('meshdir','');compiler.set('texturedir','')
        body=ET.SubElement(root.find('worldbody'),'body',name='coverage_payload',pos='0 0 -10')
        ET.SubElement(body,'freejoint',name='coverage_payload_free')
        ET.SubElement(body,'geom',name='coverage_payload_geom',type='box',size='.03 .03 .03',mass='.16875',friction='1 .005 .0001')
        for key in root.findall('keyframe/key'):key.set('qpos',key.get('qpos')+' 0 0 -10 1 0 0 0')
        # An exact duplicate of the manufactured hand can reject a blocked
        # Cartesian hand pose once, before trying different arm IK branches.
        hand=deepcopy(root.find(".//body[@name='A_hand']"))
        for node in hand.iter():
            if node.get('name'):node.set('name',node.get('name').replace('A_','coverage_hand_'))
        hand.set('pos','0 0 -10');hand.set('quat','1 0 0 0');hand.set('childclass','panda')
        hand.insert(0,ET.Element('freejoint',name='coverage_hand_free'))
        root.find('worldbody').append(hand)
        for key in root.findall('keyframe/key'):key.set('qpos',key.get('qpos')+' 0 0 -10 1 0 0 0 .04 .04')
        self.m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
        self.d=mujoco.MjData(self.m);mujoco.mj_resetDataKeyframe(self.m,self.d,self.m.key('initial').id);geometric_forward(self.m,self.d)
        self.q0=self.d.qpos.copy();self.arms={r:PandaArm(self.m,self.d,r) for r in self.task['arms']}
        self.home={r:self.q0[a.qids].copy() for r,a in self.arms.items()}
        self.owners=owner_map(self.m,self.arms);self.proxy=self.m.body('coverage_payload').id
        self.proxy_addr=int(self.m.jnt_qposadr[self.m.joint('coverage_payload_free').id])
        self.hand_addr=int(self.m.jnt_qposadr[self.m.joint('coverage_hand_free').id])
        self.hand_bodies={i for i in range(self.m.nbody) if self.m.body(i).name.startswith('coverage_hand_')}
        self.hand_tcp=self.m.site('coverage_hand_tcp').id;self.hand_cache={}
        self.samples=0;self.last_failure=None;self.ik_cache={};self.kinematic_witness=None;self.seat_clear_seen=False
        self.reach_bounds={r:sum(float(np.linalg.norm(a.ik.kinematic_model.body_pos[i])) for i in range(2,a.ik.kinematic_model.nbody))+float(np.linalg.norm(a.ik.kinematic_model.site_pos[a.ik.local_site]))+.15 for r,a in self.arms.items()}

    def ik(self,robot,target,rotation,seed):
        key=(robot,tuple(np.round(target,9)),tuple(np.round(rotation,9).ravel()),tuple(np.round(seed,7)))
        if key in self.ik_cache:return self.ik_cache[key]
        arm=self.arms[robot];q,error=arm.ik.solve(target,rotation,seed,160)
        _,actual=arm.ik.pose(q)
        if error>.0001 or np.linalg.norm(rotation_vector(rotation,actual))>.001:self.ik_cache[key]=None;return None
        if np.any(q<arm.ik.limits[:,0]) or np.any(q>arm.ik.limits[:,1]):self.ik_cache[key]=None;return None
        self.ik_cache[key]=q
        return q

    def hand_clear(self,point,rotation,width,source):
        key=(tuple(np.round(point,9)),tuple(np.round(rotation,9).ravel()),width,source)
        if key in self.hand_cache:return self.hand_cache[key]
        self.d.qpos[:]=self.q0
        self.d.qpos[self.hand_addr:self.hand_addr+3]=point-rotation@self.m.site_pos[self.hand_tcp]
        self.d.qpos[self.hand_addr+3:self.hand_addr+7]=matrix_quat(rotation)
        for suffix in ('finger_joint1','finger_joint2'):self.d.joint('coverage_hand_'+suffix).qpos[0]=width/2
        geometric_forward(self.m,self.d);self.samples+=1;clear=True
        for contact in self.d.contact:
            a,b=map(int,contact.geom);ba,bb=map(int,self.m.geom_bodyid[[a,b]])
            if ba not in self.hand_bodies and bb not in self.hand_bodies:continue
            if ba in self.hand_bodies and bb in self.hand_bodies:continue
            own,other=(ba,bb) if ba in self.hand_bodies else (bb,ba)
            og=b if ba in self.hand_bodies else a
            if self.owners[og] is not None:continue
            name=self.m.body(other).name
            if name==source and self.m.body(own).name.endswith(('_left_finger','_right_finger')):continue
            # 1mm rejection includes the 0.5mm collision tolerance, 0.1mm
            # positional and <=1mrad angular IK residual over the real hand.
            if contact.dist<-.001:
                clear=False;self.last_failure=dict(kind='hand_obstacle',a=self.m.body(own).name,b=name,gap_m=float(contact.dist));break
        self.hand_cache[key]=clear
        return clear

    def pose(self,robot,q,width,source=None,carried=None):
        """Keep every other robot/object at its exact initial pose."""
        self.d.qpos[:]=self.q0;arm=self.arms[robot];self.d.qpos[arm.qids]=q
        for suffix in ('finger_joint1','finger_joint2'):self.d.joint(robot+'_'+suffix).qpos[0]=width/2
        payload=self.proxy if self.q0[self.proxy_addr+2]>-1. else None
        if carried is not None:
            name,offset,relative=carried;tcp,rotation=arm.ik.pose(q)
            bid=self.m.body(name).id;addr=self.proxy_addr if name=='coverage_payload' else int(self.m.jnt_qposadr[self.m.joint(name+'_free').id])
            self.d.qpos[addr:addr+3]=tcp+rotation@offset
            self.d.qpos[addr+3:addr+7]=matrix_quat(rotation@relative);payload=bid
        geometric_forward(self.m,self.d);self.samples+=1
        fingers=set()
        for contact in self.d.contact:
            a,b=map(int,contact.geom);oa,ob=self.owners[a],self.owners[b]
            ba,bb=map(int,self.m.geom_bodyid[[a,b]])
            if payload is None and (ba==self.proxy or bb==self.proxy):continue
            if robot not in (oa,ob) and payload not in (ba,bb):continue
            an=self.m.body(ba).name;bn=self.m.body(bb).name
            target=source if source is not None else ('coverage_payload' if payload==self.proxy else None)
            other=bn if oa==robot else an
            own=an if oa==robot else bn
            intended=(other==target and own in (robot+'_left_finger',robot+'_right_finger'))
            if intended:
                if contact.dist<=.0006:fingers.add(own)
                # A geometric seat cannot use deep object penetration.
                if contact.dist>=-.0005:continue
            if contact.dist<-.0005:
                self.last_failure=dict(kind='collision',a=an,b=bn,gap_m=float(contact.dist));return False,fingers
        return True,fingers

    def edge(self,robot,a,b,width,source=None,carried=None):
        count=max(1,math.ceil(float(np.max(abs(b-a)))/.025))
        for fraction in np.linspace(0,1,count+1):
            ok,_=self.pose(robot,a+(b-a)*fraction,width,source,carried)
            if not ok:return False
        return True

    def cartesian(self,robot,start,target,rotation,width,source=None,carried=None):
        origin,_=self.arms[robot].ik.pose(start);count=max(1,math.ceil(float(np.linalg.norm(target-origin))/.01));q=start
        for fraction in np.linspace(0,1,count+1)[1:]:
            nxt=self.ik(robot,origin+(target-origin)*fraction,rotation,q)
            if nxt is None:self.last_failure=dict(kind='path_ik');return None
            if not self.edge(robot,q,nxt,width,source,carried):return None
            q=nxt
        return q

    def seeds(self,robot):
        yield self.home[robot]
        for amount in (-.8,.8):
            q=self.home[robot].copy();q[0]+=amount;q[2]-=amount
            yield np.clip(q,*self.arms[robot].ik.limits.T)

    def close_contact_width(self,robot,q,aperture,source=None,carried=None):
        # Actual finger collision faces need not touch a60mm cube at exactly
        # the nominal60mm joint-width setting. Search real jaw positions,
        # stopping at the first bilateral geometry without deep penetration.
        for width in np.arange(aperture,.0549,-.0001):
            ok,fingers=self.pose(robot,q,float(width),source,carried)
            if not ok:return None
            if len(fingers)==2:return float(width)
        return None

    def candidate(self,robot,point,rotation,width,source,release=False):
        if not release and not self.hand_clear(point,rotation,width,source):return None
        arm=self.arms[robot];seat=None
        for seed in self.seeds(robot):
            seat=self.ik(robot,point,rotation,seed)
            if seat is None:continue
            if self.kinematic_witness is None:self.kinematic_witness=dict(robot=robot,tcp_m=np.asarray(point).tolist(),tool_rotation=rotation.tolist(),joint_rad=seat.tolist())
            ok,_=self.pose(robot,seat,width,source)
            if ok:self.seat_clear_seen=True;break
            seat=None
        if seat is None:return None
        # A loaded goal preview has a separate virtual cube; no supply cube is
        # removed or hidden to make placement easier.
        if release:
            offset=rotation.T@np.array([0.,0.,-.012]);relative=rotation.T
            carried=('coverage_payload',offset,relative)
        else:
            origin=self.q0[int(self.m.jnt_qposadr[self.m.joint(source+'_free').id]):][:3]
            tcp,_=arm.ik.pose(seat);offset=rotation.T@(origin-tcp);relative=rotation.T
            carried=(source,offset,relative)
        for rise in (.14,.24,.34):
            hover=self.ik(robot,point+[0,0,rise],rotation,seat)
            if hover is None:continue
            if not self.pose(robot,hover,width,source)[0]:continue
            if not all(self.pose(robot,self.home[robot],w,source)[0] for w in np.linspace(.08,width,5)):continue
            if not self.edge(robot,self.home[robot],hover,width,source):continue
            held_width=self.close_contact_width(robot,seat,width,None,carried) if release else None
            if release and held_width is None:continue
            if release and not self.pose(robot,hover,held_width,None,carried)[0]:continue
            descent=self.cartesian(robot,hover,point,rotation,held_width if release else width,source,carried if release else None)
            if descent is None:continue
            if release:
                # This certifies a release pose above the cell, not an
                # impossible demand that an open hand fit at the pocket floor.
                payload_ok=self.pose(robot,descent,held_width,None,carried)[0]
                if not payload_ok:continue
                released=[(point+rotation@offset).tolist(),matrix_quat(np.eye(3)).tolist()]
                # Opening/exit are checked with the virtual cube left fixed.
                saved=self.q0[self.proxy_addr:self.proxy_addr+7].copy()
                self.q0[self.proxy_addr:self.proxy_addr+7]=np.r_[released[0],released[1]]
                try:
                    if not all(self.pose(robot,descent,w,None,carried)[0] for w in np.linspace(held_width,width,7)):continue
                    retreat=self.cartesian(robot,descent,point+[0,0,rise],rotation,width,None,carried=None)
                finally:self.q0[self.proxy_addr:self.proxy_addr+7]=saved
                if retreat is None:continue
            else:
                held_width=self.close_contact_width(robot,descent,width,source)
                if held_width is None:self.last_failure=dict(kind='no_bilateral_geometric_contact');continue
                origin=self.q0[int(self.m.jnt_qposadr[self.m.joint(source+'_free').id]):][:3]
                actual_tcp,actual_rotation=arm.ik.pose(descent)
                carried=(source,actual_rotation.T@(origin-actual_tcp),actual_rotation.T)
                retreat=self.cartesian(robot,descent,point+[0,0,.14],rotation,held_width,source,carried)
                if retreat is None:continue
            return dict(robot=robot,tcp_m=np.asarray(point).tolist(),tool_rotation=rotation.tolist(),
                approach_width_m=width,contact_width_m=held_width,hover_rise_m=rise,seat_joint_rad=descent.tolist(),hover_joint_rad=hover.tolist(),
                lift_or_withdraw_joint_rad=retreat.tolist(),initial_to_hover='Joint samples <=0.025rad',
                approach_lift='Cartesian samples <=10mm plus joint samples <=0.025rad',
                bilateral_contact_checked=not release)
        return None

    def probe(self,kind,item):
        ident=item['id'] if kind=='source' else item['goal_id'];p=np.asarray(item['initial_position_m'] if kind=='source' else item['position_m'])
        robots=sorted(self.arms,key=lambda r:np.linalg.norm(np.asarray(self.task['arms'][r]['base_position_m'])-p))
        begin=self.samples;failures=Counter();self.kinematic_witness=None;self.seat_clear_seen=False
        for robot in robots:
            base=np.asarray(self.task['arms'][robot]['base_position_m'])
            if np.linalg.norm(p-base)>self.reach_bounds[robot]:continue
            for tilt in (0,15,25):
                base=np.asarray(self.task['arms'][robot]['base_position_m']);axes=[np.array([1.,0,0]),np.array([-1.,0,0]),np.array([0,1.,0]),np.array([0,-1.,0])]
                rotations=grasp_rotations(np.array([0.,0,1.]),axes,p-base,tilts=(tilt,))
                for rotation,_ in rotations:
                    for height in ((.012,.020,.025,.028,.006) if kind=='source' else (.012,)):
                        # 30mm drop height is a finite candidate. It is not a
                        # physical claim that the cube will settle correctly.
                        point=p+[0,0,height+(0 if kind=='source' else .030)]
                        for width in (.064,.072,.080):
                            result=self.candidate(robot,point,rotation,width,ident if kind=='source' else None,kind=='goal')
                            if result:return dict(id=ident,position_m=p.tolist(),confirmed=True,kinematically_reachable=True,witness=result,samples=self.samples-begin)
                            failures[(self.last_failure or {}).get('kind','ik_or_endpoint')]+=1
        # A hand-clearance rejection is independent of arm reach. Preserve a
        # separate IK witness so it cannot be mislabeled as an unreachable item.
        if self.kinematic_witness is None:
            point=p+[0,0,.012+(0 if kind=='source' else .030)]
            for robot in robots:
                base=np.asarray(self.task['arms'][robot]['base_position_m'])
                if np.linalg.norm(point-base)>self.reach_bounds[robot]:continue
                for rotation,_ in grasp_rotations(np.array([0.,0,1.]),axes,point-base,tilts=(0,15,25)):
                    for seed in self.seeds(robot):
                        q=self.ik(robot,point,rotation,seed)
                        if q is not None:
                            self.kinematic_witness=dict(robot=robot,tcp_m=point.tolist(),tool_rotation=rotation.tolist(),joint_rad=q.tolist());break
                    if self.kinematic_witness is not None:break
                if self.kinematic_witness is not None:break
        return dict(id=ident,position_m=p.tolist(),confirmed=False,search_exhausted=True,
            kinematically_reachable=self.kinematic_witness is not None,kinematic_witness=self.kinematic_witness,collision_clear_endpoint_found=self.seat_clear_seen,
            reason='No witness in the finite tested poses/routes; not proof of physical impossibility',
            failures=dict(failures),last_failure=self.last_failure,samples=self.samples-begin)


def run(case):
    folder=ROOT/case['directory'];out=OUT/case['id'];out.mkdir(parents=True,exist_ok=True)
    c=Coverage(folder);start=time.monotonic();report=dict(case_id=case['id'],source_scene_sha256=c.scene_hash,checker_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        complete=False,physics_steps=0,scope='Finite sampled geometric coverage in exact initial scene; other arms remain at initial poses. Not a real grasp/placement or parallel planning certificate.',
        collision_penetration_tolerance_m=.0005,joint_sample_max_rad=.025,cartesian_sample_max_m=.01,
        ik_position_tolerance_m=.0001,ik_orientation_tolerance_rad=.001,
        gripper_prefilter='Exact copied manufactured hand against apparatus/supply; excludes arms, followed by full-arm checks for admitted poses. 1mm conservative prefilter threshold.',
        sources=[],goals=[])
    for kind,items,key in [('source',c.task['objects'],'sources'),('goal',c.task['goals'],'goals')]:
        for index,item in enumerate(items):
            row=c.probe(kind,item);report[key].append(row)
            report.update(wall_s=time.monotonic()-start,collision_samples=c.samples)
            write_json(out/'report.json',report)
            print(case['id'],kind,index+1,'/',len(items),'confirmed',sum(x['confirmed'] for x in report[key]),'wall',round(report['wall_s'],1),flush=True)
    report.update(complete=True,passed=all(x['confirmed'] for k in ('sources','goals') for x in report[k]))
    write_json(out/'report.json',report)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--arms',type=int,choices=[4,6,8],required=True);p.add_argument('--level',type=int,choices=[1,2,3]);p.add_argument('--catalog',type=Path,default=ROOT/'configs/suites/frontier-mosaic-r12.json');p.add_argument('--output',type=Path);a=p.parse_args()
    if a.output:OUT=a.output.resolve()
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/'checker-source.py').write_bytes(Path(__file__).read_bytes())
    cases=json.loads(a.catalog.read_text())['cases']
    selected=[case for case in cases if case['arms']==a.arms and (a.level is None or case['level']==a.level)]
    for case in selected:
        if case['level']==3 and a.level is None:
            sibling=next(c for c in selected if c['level']==2)
            source=OUT/sibling['id']/'report.json'
            from robofrontier.suite.v2 import physical_summary
            ma=mujoco.MjModel.from_xml_path(str(ROOT/sibling['directory']/'scene.xml'))
            mb=mujoco.MjModel.from_xml_path(str(ROOT/case['directory']/'scene.xml'))
            ha=hashlib.sha256(json.dumps(physical_summary(ma),sort_keys=True,separators=(',',':')).encode()).hexdigest()
            hb=hashlib.sha256(json.dumps(physical_summary(mb),sort_keys=True,separators=(',',':')).encode()).hexdigest()
            ta=json.loads((ROOT/sibling['directory']/'task.json').read_text());tb=json.loads((ROOT/case['directory']/'task.json').read_text())
            if ha==hb and all(ta[k]==tb[k] for k in ['arms','objects','goals']):
                r=json.loads(source.read_text());r.update(case_id=case['id'],source_scene_sha256=hashlib.sha256((ROOT/case['directory']/'scene.xml').read_bytes()).hexdigest(),
                    equivalent_evidence=dict(source_report=str(source),source_report_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),compiled_physics_sha256=ha,
                        arms_objects_goals_equal=True,scope='Same complete collision geometry, limits, initial joint/object states and target poses. Printed grid is non-colliding paint only.'))
                write_json(OUT/case['id']/'report.json',r);print('EQUIVALENT',case['id'],flush=True);continue
        run(case)
