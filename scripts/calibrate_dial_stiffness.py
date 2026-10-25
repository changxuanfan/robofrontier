"""Isolated controller-stiffness counterfactual, never a suite task PASS."""
import argparse,hashlib,json,math,shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
from robofrontier.suite.scenes import ROOT,catalog,case_directory
from robofrontier.suite.oracle import Trial
from robofrontier.suite.dial_oracle import Manipulator
from robofrontier.run_setup import write_json

def run(scale,name,finger_armature=None,position_drive=False,uniform_contact=False,retention=False,timestep=None,full_task=False,case_id='dial-n04-d3',dense_contact=False,seed=0):
    case=next(c for c in catalog()['cases'] if c['id']==case_id);source=case_directory(case);out=ROOT/'runs/oracle/calibration'/name
    out.mkdir(parents=True,exist_ok=False);asset=out/'source';asset.mkdir();shutil.copytree(source/'assets',asset/'assets')
    root=ET.parse(source/'scene.xml').getroot();root.find('compiler').set('meshdir',str((source/root.find('compiler').get('meshdir')).resolve()))
    if timestep is not None:root.find('option').set('timestep',str(timestep))
    model=mujoco.MjModel.from_xml_path(str(source/'scene.xml'));task=json.loads((source/'task.json').read_text());operators=[d['nominal_robots'][0] for d in task['dials']] if case['family']=='dial' else list(task['arms'])
    for robot in operators:
        for i in range(1,8):
            name=f'{robot}_actuator{i}';aid=model.actuator(name).id;e=root.find(f"actuator/*[@name='{name}']");gain=model.actuator_gainprm[aid].copy();bias=model.actuator_biasprm[aid].copy();gain[0]*=scale;bias[1]*=scale;bias[2]*=math.sqrt(scale)
            e.set('gainprm',' '.join(map(str,gain)));e.set('biasprm',' '.join(map(str,bias)))
    if finger_armature is not None:
        for joint in root.findall('worldbody/.//joint'):
            if (joint.get('name') or '').endswith(('finger_joint1','finger_joint2')):joint.set('armature',str(finger_armature))
    if position_drive:
        inertia=2*((finger_armature if finger_armature is not None else .1)+.015);stiffness=10000.;damping=2*math.sqrt(stiffness*inertia)
        for actuator in root.findall('actuator/*'):
            if (actuator.get('name') or '').endswith('_actuator8'):
                actuator.set('gainprm',str(stiffness*.04/255));actuator.set('biasprm',f'0 {-stiffness} {-damping}');actuator.set('forcerange','-70 70')
    if uniform_contact:
        for body in root.findall('worldbody/body'):
            if (body.get('name') or '').endswith('_module'):
                for geom in body.findall('.//geom'):
                    if geom.get('contype')!='0':geom.set('priority','1');geom.set('solref','.004 1');geom.set('solimp','.95 .99 .001 .5 2')
    ET.ElementTree(root).write(asset/'scene.xml',encoding='unicode')
    for filename in ('environment.json','task.json'):
        from robofrontier.suite.tasks import make_task
        value=make_task(source) if filename=='task.json' else json.loads((source/filename).read_text())
        if timestep is not None:value['physics']['timestep_s']=timestep
        value['controller_counterfactual']=dict(operators=operators,stiffness_scale=scale,damping_scale=math.sqrt(scale),finger_equivalent_drive_inertia_kg=finger_armature,position_drive_recalibrated=position_drive,uniform_contact=uniform_contact,arm_joint_force_limits_unchanged=True,position_hand_force_cap_n=70 if position_drive else None,timestep_override_s=timestep)
        if filename=='task.json':value['scene_sha256']=hashlib.sha256((asset/'scene.xml').read_bytes()).hexdigest()
        write_json(asset/filename,value)
    t=Trial(asset,out/'trial',seed=seed);report=dict(passed=False,scope='Controller counterfactual only; not a complete suite witness',operator_stiffness_scale=scale,mechanical_passed=False)
    try:
        if dense_contact:
            import numpy as np
            names=[o['id'] for o in t.task['objects'] if o['id']!='C_yellow_1'][:8];floor=t.task['grid']['floor_z_m']
            points=[np.array([x*.051,y*.051,floor+.02]) for x in (-1,0,1) for y in (-1,0,1) if x or y]
            for name,p in zip(names,points):t.data.joint(name+'_free').qpos[:]=np.r_[p,[1,0,0,0]]
            mujoco.mj_forward(t.model,t.data);t.event('dense_fixture_initialization',scope='Before first physics step only')
        t.wait(.6)
        if dense_contact:
            t.pick('C','C_yellow_1',height=.006);t.place('C',[0,0,floor+.02],np.eye(3),precise=True);t.wait(.8)
            b=t.data.body('C_yellow_1');error=float(np.linalg.norm(b.xpos[:2]));neighbor_drift=max(float(np.linalg.norm(t.data.body(n).xpos-p)) for n,p in zip(names,points));upright=float(b.xmat.reshape(3,3)[2,2]);released=t.monitor.released(t.monitor.geoms['C_yellow_1'])
            report.update(dense_xy_error_m=error,neighbor_drift_m=neighbor_drift,upright=upright,mechanical_passed=bool(error<.005 and neighbor_drift<.001 and upright>.99 and released));t.snapshot('dense-final')
        elif full_task:
            if case['family']=='dial':
                from robofrontier.suite.dial_oracle import solve
            elif case['family']=='relay':
                from robofrontier.suite.relay_oracle import solve
            else:
                from robofrontier.suite.placement_oracle import transport,mosaic
                solve=transport if case['family']=='transport' else mosaic
            solve(t);report['mechanical_passed']=bool(t.monitor.result()['task_success'])
        elif retention:
            m=Manipulator(t);m.lift()
            try:m.set_symbols([1]*4)
            except Exception as error:report['capture_event']=type(error).__name__+': '+str(error)
            t.execute('offload-red',{robot:('gripper',dict(mode='move',width_m=.08,speed_m_s=.08)) for robot in operators},allow_failure=True)
            for robot in operators:t.move_free(robot,*t.home[robot],label='offload-withdraw')
            minimum=[1e9]*4;maximum_width=[0.]*4;survived=0.;grasp_survived=0.;grasp_alive=True
            for _ in range(600):
                t.wait(.1);valid=True
                for i,a in enumerate(m.dials):
                    own=set(t.monitor.module_geoms[i]);clear=t.monitor.bottom(own)-.4;minimum[i]=min(minimum[i],clear);blue=a['nominal_robots'][1];maximum_width[i]=max(maximum_width[i],t.arms[blue].observe()['gripper_width_m'])
                    for c in t.data.contact:
                        if c.dist>.0002:continue
                        x,y=map(int,c.geom)
                        for part,other in ((x,y),(y,x)):
                            if part in own and other not in own and (t.monitor.geom_owner[other]<0 or t.monitor.robots[int(t.monitor.geom_owner[other])] in operators):valid=False
                    if clear<.02:valid=False
                if not valid:break
                survived+=.1
                grasp_alive=grasp_alive and all(q.qualified for q in t.monitor.blue_qualification)
                if grasp_alive:grasp_survived+=.1
            report.update(offloaded_survival_s=survived,retained_airborne_60s=survived>=59.99,qualified_blue_grasp_survival_s=grasp_survived,retained_qualified_blue_grasp_60s=grasp_survived>=59.99,min_clearance_m=minimum,maximum_blue_width_m=maximum_width)
        else:
            m=Manipulator(t);m.lift()
            for symbol in (1,2,3):m.set_symbols([symbol]*4);m.release(m.dials)
            report['mechanical_passed']=not t.monitor.violations
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error)
        t.snapshot('failure')
    finally:
        report['evaluation']=t.monitor.result();t.close(report)
    print(json.dumps({k:report.get(k) for k in ('mechanical_passed','error','operator_stiffness_scale','retained_airborne_60s','retained_qualified_blue_grasp_60s','qualified_blue_grasp_survival_s','offloaded_survival_s','maximum_blue_width_m')}),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--scale',type=float,required=True);p.add_argument('--name',required=True);p.add_argument('--finger-armature',type=float);p.add_argument('--position-drive',action='store_true');p.add_argument('--uniform-contact',action='store_true');p.add_argument('--retention',action='store_true');p.add_argument('--timestep',type=float);p.add_argument('--full-task',action='store_true');p.add_argument('--dense-contact',action='store_true');p.add_argument('--case',default='dial-n04-d3');p.add_argument('--seed',type=int,default=0);a=p.parse_args();run(a.scale,a.name,a.finger_armature,a.position_drive,a.uniform_contact,a.retention,a.timestep,a.full_task,a.case,a.dense_contact,a.seed)
