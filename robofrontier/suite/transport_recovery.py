"""Engineering handback for a live Transport world, without arbitrary code exec.

The research owner may submit ordinary bounded robot commands or request a
fresh goal plan from measured state. Inputs and their provenance are archived.
No state rewind, score reset, unknown-action adoption or implicit success.
"""
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from ..run_setup import write_json
from .transport_full_oracle import execute_job


class EngineeringRecovery:
    def __init__(self,directory):
        self.path=Path(directory)/'recovery'
        for name in ('inbox','accepted','rejected'): (self.path/name).mkdir(parents=True,exist_ok=True)
        self.seen=set();self.external_inputs=[];self.next_poll=0.;self.counter=0
        self.waiting=False;self.failed_progress={};self.automatic_progress={}

    def observation(self,t,runtime):
        failed={ident:dict(goal=job.get('goal'),object=job.get('object'),error=str(job.get('error')),
            robots=sorted(job['robots']),retained=[r for r,owner in runtime.leases.items() if owner==ident])
            for ident,job in runtime.jobs.items() if job['status']=='failed' and not job.get('recovered_by')}
        return dict(schema='transport.engineering-frontier.v1',scene_sha256=t.task['scene_sha256'],
            sim_time_s=t.time,wall_s=time.monotonic()-t.started,failed_jobs=failed,
            accepted_goals=t.monitor.result().get('assignments',{}),retained_resources=runtime.leases,
            robots={r:a.observe() for r,a in t.arms.items()},
            objects={o['id']:dict(position_m=t.data.body(o['id']).xpos.tolist(),rotation=t.data.body(o['id']).xmat.tolist()) for o in t.env['objects']},
            held=dict(t.held),active_robots=sorted(runtime.running),updated_at_unix_s=time.time(),
            source='Privileged engineering observation; never a GPT benchmark policy input')

    def _retry(self,t,runtime,ident,request):
        old=runtime.jobs[ident]
        if old.get('recovered_by'):raise ValueError('This failure already has a recovery owner')
        goal=old['goal'];name=old['object'];robots=set(old['robots'])
        requested=request.get('robots')
        if requested is None:
            p=t.data.body(goal['board']).xpos
            nearby=sorted((r for r in t.arms if r not in runtime.leases),
                key=lambda r:np.linalg.norm(np.asarray(t.stations[r]['base_position_m'])[:2]-p[:2]))[:2]
            robots.update(nearby)
        else:robots.update(requested)
        if robots-set(t.arms):raise ValueError('Unknown recovery robot')
        original=dict(id=goal['goal_id'],goal=goal,object=name,path=list(robots),attempts=1)
        steps=request.get('steps',[])
        if not isinstance(steps,list) or len(steps)>64:raise ValueError('At most 64 ordinary recovery steps')
        for step in steps:
            if 'wait_s' in step:
                if set(step)!={'wait_s'} or not 0<=float(step['wait_s'])<=30:raise ValueError('Invalid bounded wait')
            elif set(step)!={'robot','operation','parameters'} or step['robot'] not in robots or step['operation'] not in ('move','gripper') or not isinstance(step['parameters'],dict):
                raise ValueError('Only leased ordinary robot commands are accepted')
        def recover():
            for step in steps:
                if 'wait_s' in step:t.wait(float(step['wait_s']))
                else:t.execute('engineering-repair',{step['robot']:(step['operation'],step['parameters'])})
            return execute_job(t,runtime,original,resume=True)
        self.counter+=1;new_id=goal['goal_id']+'-recovery-'+str(self.counter)
        if not runtime.recover(ident,new_id,robots,recover):raise ValueError('Requested recovery resources are still busy')
        return new_id

    def poll(self,t,runtime,jobs,pending,force=False):
        if not force and time.monotonic()<self.next_poll:return False
        self.next_poll=time.monotonic()+1.;state=self.observation(t,runtime)
        write_json(self.path/'frontier.json',state);changed=False
        # New accepted progress can free a previously unavailable carrier or
        # route. Admit at most once per goal/progress count, not once forever.
        # A static blocked frontier cannot trigger an identical retry loop.
        for ident,failed in list(state['failed_jobs'].items()):
            count=len(state['accepted_goals']);baseline=self.failed_progress.setdefault(ident,count)
            goal=failed['goal']['goal_id']
            if count>self.automatic_progress.get(goal,-1) and (count>baseline or goal in state['accepted_goals']):
                try:self._retry(t,runtime,ident,{});self.automatic_progress[goal]=count;changed=True
                except ValueError:pass
        for file in sorted((self.path/'inbox').glob('*.json')):
            if file.name in self.seen:continue
            try:
                payload=file.read_bytes();request=json.loads(payload)
                if set(request)-{'id','job','scene_sha256','robots','steps','reason','author'}:raise ValueError('Unsupported engineering input')
                if request['scene_sha256']!=t.task['scene_sha256']:raise ValueError('Wrong physical scene')
                ident=request['job']
                if ident not in runtime.jobs or runtime.jobs[ident]['status']!='failed':raise ValueError('No matching failed job')
                new_id=self._retry(t,runtime,ident,request)
                record=dict(id=request['id'],source_file=file.name,sha256=hashlib.sha256(payload).hexdigest(),
                    input=request,admitted_sim_s=t.time,recovery_job=new_id)
                self.external_inputs.append(record);write_json(self.path/'accepted'/file.name,record)
                t.event('transport_engineering_input',**record);self.seen.add(file.name);changed=True
            except (ValueError,KeyError,TypeError) as error:
                write_json(self.path/'rejected'/file.name,dict(error=str(error),sim_time_s=t.time));self.seen.add(file.name)
        return changed

    def __call__(self,t,runtime,jobs,pending):
        self.waiting=True
        t.event('transport_engineering_handback',frontier=str(self.path/'frontier.json'),
            scope='Live world retained; independent jobs already continue through the same runtime')
        t.snapshot('recovery-frontier')
        self.poll(t,runtime,jobs,pending,force=True)
        next_wall=time.monotonic()
        while not runtime.active:
            t.check_stop()
            if self.poll(t,runtime,jobs,pending):break
            # Keep gravity, contact and holding alive at approximately real
            # time during owner diagnosis. This waiting counts in the trial.
            runtime.step();next_wall+=float(t.model.opt.timestep)
            delay=next_wall-time.monotonic()
            if delay>0:time.sleep(min(delay,.02))
        self.waiting=False
        return runtime.active
