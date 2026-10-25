"""Engineering reference, not an Agent episode or benchmark policy.

Replays the resource witness through World.validate_action/start_action/
poll_actions, exactly the executor used by the role HTTP service. No teleport,
attachment, joint-state reset or object-truth correction is used. Object truth
is read only to assert grasp/placement and produce this private check report.
The video clock is simulation time, not inference/wall time.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import glfw
import mujoco
import numpy as np

from check_transport_schedule import ROUTES, solve
from robofrontier.pipeline import World
from robofrontier.recording import Video
from robofrontier.run_setup import write_json
from robofrontier.task_config import ROOT, load_task, load_harness


class Reference:
    def __init__(self, output, task, record=True, visible=True, speed=.06):
        self.output=output;self.task=task;self.cfg=load_harness()
        self.world=World(output,task);self.trace=[];self.number=0
        self.speed=speed
        self.contact_samples=0;self.video=None;self.window=None
        self.grasp_evidence=[];self.place_evidence=[]
        if record or visible:
            if not glfw.init():raise RuntimeError('GLFW unavailable')
            glfw.window_hint(glfw.VISIBLE,visible)
            self.window=glfw.create_window(1280,800,'RoboFrontier — engineering reference',None,None)
            if not self.window:raise RuntimeError('GL window unavailable')
            glfw.make_context_current(self.window);glfw.swap_interval(0)
            self.context=mujoco.MjrContext(self.world.model,mujoco.mjtFontScale.mjFONTSCALE_150)
            self.scene=mujoco.MjvScene(self.world.model,maxgeom=5000)
            self.options=mujoco.MjvOption();self.options.geomgroup[3]=0
            self.camera=mujoco.MjvCamera();self.camera.type=mujoco.mjtCamera.mjCAMERA_FIXED
            self.camera.fixedcamid=self.world.model.camera('global').id
            if record:self.video=Video(output/'private/outputs/reference-sim-time.mp4',1280,800,10)
        self.next_frame=0
        # Object IDs also begin with A_/B_/C_; classify by body ancestry instead.
        model=self.world.model
        self.geom_owner={}
        roots={model.body(a+'_link0').id:a for a in task['arms']}
        for geom in range(model.ngeom):
            body=int(model.geom_bodyid[geom])
            while body and body not in roots:body=int(model.body_parentid[body])
            if body in roots:self.geom_owner[geom]=roots[body]

    def frame(self, record=True):
        if not self.window:return
        glfw.poll_events()
        if glfw.window_should_close(self.window):raise RuntimeError('Reference window closed')
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN,self.context)
        viewport=mujoco.MjrRect(0,0,1280,800)
        mujoco.mjv_updateScene(self.world.model,self.world.data,self.options,None,self.camera,
                              mujoco.mjtCatBit.mjCAT_ALL,self.scene)
        mujoco.mjr_render(viewport,self.scene,self.context)
        rgb=np.empty((800,1280,3),dtype=np.uint8)
        mujoco.mjr_readPixels(rgb,None,viewport,self.context)
        if self.video and record:self.video.frame(np.ascontiguousarray(rgb[::-1]))
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW,self.context)
        mujoco.mjr_render(viewport,self.scene,self.context);glfw.swap_buffers(self.window)
        return np.ascontiguousarray(rgb[::-1])

    def tick(self):
        self.world.step();self.contact_samples+=1
        data=self.world.data;model=self.world.model
        for contact in data.contact:
            first=self.geom_owner.get(int(contact.geom1));second=self.geom_owner.get(int(contact.geom2))
            if first and second and first!=second:
                raise AssertionError(('inter-arm contact',first,second,float(data.time)))
        if not np.isfinite(data.qpos).all():raise AssertionError('Nonfinite physical state')
        if self.world.time+1e-9>=self.next_frame:
            self.frame();self.next_frame+=.1

    def commands(self, commands):
        active={}
        for arm,(op,parameters) in commands.items():
            prepared=self.world.validate_action(arm,op,parameters,self.cfg)
            self.number+=1;call_id='reference-'+str(self.number)
            self.world.start_action(arm,call_id,prepared)
            active[call_id]=arm
            self.trace.append({'call_id':call_id,'arm':arm,'op':op,'parameters':parameters,
                               'start_sim_s':self.world.time})
        while active:
            self.tick()
            for result in self.world.poll_actions(self.cfg['action_timeout_sim_s']):
                arm=active.pop(result['call_id'])
                row=next(t for t in reversed(self.trace) if t['call_id']==result['call_id'])
                row.update(result);row['finish_sim_s']=self.world.time
                if result['status']!='completed':
                    row['robot']=self.world.arms[arm].observe()
                    row['contacts']=[{'geoms':[self.world.model.geom(int(c.geom1)).name,
                                              self.world.model.geom(int(c.geom2)).name],
                                      'distance_m':float(c.dist)} for c in self.world.data.contact
                                     if int(c.geom1) in self.geom_owner or int(c.geom2) in self.geom_owner]
                    raise AssertionError((arm,row))

    def moves(self, points):
        self.commands({a:('move',{'target_m':list(map(float,p)),'speed_m_s':self.speed}) for a,p in points.items()})

    def grips(self, arms, state):
        self.commands({a:('gripper',{'state':state}) for a in arms})

    def phase(self, entries):
        # Each resource-model phase ends with the *whole* arm returned home.
        points={a:point for a,_,_,point in entries}
        self.moves({a:[p[0],p[1],.66] for a,p in points.items()})
        self.moves({a:[p[0],p[1],.423] for a,p in points.items()})
        pick={a for a,_,is_pick,_ in entries if is_pick};place=set(points)-pick
        self.commands({**{a:('gripper',{'state':'close'}) for a in pick},
                       **{a:('gripper',{'state':'open'}) for a in place}})
        self.moves({a:[p[0],p[1],.66] for a,p in points.items()})
        for arm,obj,is_pick,point in entries:
            position=self.world.data.body(obj).xpos.copy()
            evidence={'arm':arm,'object':obj,'sim_time_s':self.world.time,'object_position_m':position.tolist()}
            if is_pick:
                if position[2]<.58:raise AssertionError(('grasp failed',evidence))
                self.grasp_evidence.append(evidence)
            else:
                error=float(np.linalg.norm(position-np.array(point)))
                evidence['error_m']=error
                if error>.012:raise AssertionError(('placement error',evidence))
                self.place_evidence.append(evidence)
        self.moves({a:[.40,self.task['arms'][a],.68] for a in points})

    def close(self):
        video=self.video.close() if self.video else None
        if self.window:
            self.context.free();glfw.destroy_window(self.window);glfw.terminate()
        return video


def run(output, mode, record=True, visible=True, speed=.06):
    if output.exists():raise ValueError('Choose a new engineering output directory')
    output.mkdir(parents=True)
    task_path=ROOT/'configs/tasks/three-arm-nine-orders-v1.json';task=load_task(task_path)
    write_json(output/'private/inputs/task.json',task)
    schedule=solve(1)['schedule'] if mode=='parallel' else [[name] for name,route in ROUTES.items() for _ in range(2*len(route))]
    assignments={
        'A_local':('A_red_1','A',0), 'A_to_B':('A_red_2','B',1), 'A_to_C':('A_red_3','C',0),
        'B_local':('B_blue_1','B',0), 'B_to_C_1':('B_blue_2','C',1), 'B_to_C_2':('B_blue_3','C',2),
        'C_to_A_1':('C_green_1','A',1), 'C_to_A_2':('C_green_2','A',2), 'C_to_B':('C_green_3','B',2)}
    initial={o['id']:o['initial_position_m'] for o in task['objects']}
    counters={name:0 for name in ROUTES};reference=None
    report={'scope':'Engineering reference only; not model performance', 'mode':mode,
            'tcp_speed_m_s':speed,
            'task_sha256':hashlib.sha256(task_path.read_bytes()).hexdigest(),
            'video_clock':'simulation time; not wall-clock time',
            'collision_scope':'All robot-body contact pairs sampled every 0.02 simulated seconds; no formal swept-volume certification',
            'passed':False}
    started=time.monotonic()
    try:
        reference=Reference(output,task,record,visible,speed)
        for batch_index,batch in enumerate(schedule):
            entries=[]
            for name in batch:
                phase=counters[name];route=ROUTES[name];station=phase//2;arm=route[station]
                obj,destination,slot=assignments[name]
                if phase%2==0:
                    point=initial[obj] if station==0 else [.5,(task['arms'][route[station-1]]+task['arms'][arm])/2,.42]
                else:
                    point=task['goals'][destination][slot]['position_m'] if station+1==len(route) else [.5,(task['arms'][arm]+task['arms'][route[station+1]])/2,.42]
                entries.append((arm,obj,phase%2==0,point));counters[name]+=1
            reference.phase(entries)
            print(json.dumps({'batch':batch_index+1,'of':len(schedule),'sim_s':reference.world.time}),flush=True)
        for _ in range(100):reference.tick()
        report['evaluation']=reference.world.evaluate()
        report['final_grippers']={a:r.observe()['gripper_width_m'] for a,r in reference.world.arms.items()}
        report['passed']=report['evaluation']['all_goals_met'] and all(v>.07 for v in report['final_grippers'].values())
        if not report['passed']:raise AssertionError('Final order/release check failed')
        pixels=reference.frame(record=False)
        if pixels is not None:
            from PIL import Image
            Image.fromarray(pixels).save(output/'private/outputs/final-global.png')
    except Exception as error:
        report['error']=repr(error)
        raise
    finally:
        if reference:
            report.update(sim_time_s=reference.world.time, action_count=len(reference.trace),
                          contact_samples=reference.contact_samples,video=reference.close(),
                          grasps=reference.grasp_evidence,placements=reference.place_evidence)
            write_json(output/'private/outputs/actions.json',reference.trace)
        report['wall_time_s']=time.monotonic()-started
        write_json(output/'private/outputs/report.json',report)
        print(json.dumps({k:v for k,v in report.items() if k not in ('grasps','placements')},ensure_ascii=False),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--mode',choices=['serial','parallel'],default='parallel')
    p.add_argument('--no-record',action='store_true');p.add_argument('--hidden',action='store_true')
    p.add_argument('--speed',type=float,default=.06)
    args=p.parse_args();run(args.output.resolve(),args.mode,not args.no_record,not args.hidden,args.speed)
