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
from .transport_full_oracle import execute_job,goal_coverage,candidate_graph,path_between
from .transport_runtime import PreparationComplete
from .transport_oracle_v2 import _robot_payload_contacts
from .transport_grasp_safety import requires_contact_recovery


class EngineeringRecovery:
    def __init__(self,directory,on_idle=None,inventory_check=None,on_review=None):
        self.path=Path(directory)/'recovery'
        for name in ('inbox','accepted','rejected','reviews'): (self.path/name).mkdir(parents=True,exist_ok=True)
        self.seen=set();self.external_inputs=[];self.next_poll=0.;self.counter=0
        self.waiting=False;self.failed_progress={};self.automatic_progress={}
        self.on_idle=on_idle;self.inventory_check=inventory_check
        self.on_review=on_review

    def on_terminal(self,t,runtime,ident,job,kind):
        fault=kind=='failed' and requires_contact_recovery(job.get('error'))
        review=kind=='prepared' and job.get('pause_after_prepare',False)
        if fault:job['manual_recovery_required']=True
        if not (fault or review) or self.on_review is None:return
        # The failed/prepared scope has known terminal actions. Independent
        # actions may remain live; preserve those controllers rather than
        # adopting, cancelling or replaying them during engineering review.
        if any(t.arms[r].job is not None or t.arms[r].uncertainty_reported for r in job['robots']):return
        notice=dict(job=ident,kind=kind,reason=str(job.get('error')) if fault else 'Explicit prepared-state review',
            preserved_action_ids={r:a.job for r,a in t.arms.items() if a.job is not None})
        state=self.observation(t,runtime);state['engineering_review']=notice
        write_json(self.path/'frontier.json',state)
        write_json(self.path/'reviews'/(ident+'-'+kind+'.json'),state)
        self.on_review(t,runtime,notice)

    def observation(self,t,runtime):
        def support(name):
            if not hasattr(t.monitor,'geoms') or name not in t.monitor.geoms:return None
            velocity=None
            if hasattr(t.data,'joint'):
                velocity=np.asarray(t.data.joint(name+'_free').qvel[:3]).tolist()
            target=t.monitor.geoms[name]
            return dict(geometric_bilateral=sorted(t.monitor.bilateral(target)),
                positive_force_bilateral=sorted(t.monitor.bilateral(target,positive_force=True)),
                linear_velocity_m_s=velocity)
        failed={ident:dict(goal=job.get('goal'),object=job.get('object'),error=str(job.get('error')),
            error_type=type(job.get('error')).__name__,manual_recovery_required=job.get('manual_recovery_required',False),
            measured_support=support(job.get('object')),
            prepared_reservations=job.get('prepared_reservations',[]),
            robots=sorted(job['robots']),retained=[r for r,owner in runtime.leases.items() if owner==ident])
            for ident,job in runtime.jobs.items() if job['status'] in ('failed','prepared') and not job.get('recovered_by')}
        return dict(schema='transport.engineering-frontier.v1',scene_sha256=t.task['scene_sha256'],
            sim_time_s=t.time,wall_s=time.monotonic()-t.started,failed_jobs=failed,
            accepted_goals=t.monitor.result().get('assignments',{}),retained_resources=runtime.leases,
            robots={r:a.observe() for r,a in t.arms.items()},
            objects={o['id']:dict(position_m=t.data.body(o['id']).xpos.tolist(),rotation=t.data.body(o['id']).xmat.tolist()) for o in t.env['objects']},
            held=dict(t.held),active_robots=sorted(runtime.running),updated_at_unix_s=time.time(),
            state_qpos=t.data.qpos.tolist(),state_qvel=t.data.qvel.tolist(),
            capabilities=['coherent_qpos_qvel','retain_robots_on_prepare','pause_after_prepare','known_action_fault_review','measured_payload_support','immutable_review_snapshots'],
            source='Privileged engineering observation; never a GPT benchmark policy input')

    def _retry(self,t,runtime,ident,request):
        old=runtime.jobs[ident]
        if old.get('recovered_by'):raise ValueError('This failure already has a recovery owner')
        goal=old['goal'];name=old['object']
        retained={r for r,owner in runtime.leases.items() if owner==ident}
        requested=request.get('robots')
        if requested is None:
            robots=retained|{r for r in old['robots'] if r not in runtime.leases}
            p=t.data.body(goal['board']).xpos
            nearby=sorted((r for r in t.arms if r not in runtime.leases),
                key=lambda r:np.linalg.norm(np.asarray(t.stations[r]['base_position_m'])[:2]-p[:2]))[:2]
            robots.update(nearby)
        else:robots=retained|set(requested)
        if robots-set(t.arms):raise ValueError('Unknown recovery robot')
        if not robots:raise ValueError('No available recovery robot')
        if requested is None and goal['goal_id'] not in t.monitor.result().get('assignments',{}):
            starts=[r for r in robots if t.arms[r].grasp_force is not None and t.finger_contacts(r,name)]
            if not starts:
                point=t.data.body(name).xpos
                starts=[r for r in robots if np.linalg.norm(np.asarray(t.stations[r]['base_position_m'])[:2]-point[:2])<=.90]
            destinations=set(goal_coverage(t,goal))&robots
            graph={r:[s for s in ns if s in robots] for r,ns in candidate_graph(t).items() if r in robots}
            if not any(path_between(graph,start,end) for start in starts for end in destinations):
                raise ValueError('No currently available recovery route; preserve the original failed scope')
        original=dict(id=goal['goal_id'],goal=goal,object=name,path=list(robots),attempts=1)
        steps=request.get('steps',[])
        resume=request.get('resume_goal',True)
        if not isinstance(resume,bool):raise ValueError('resume_goal must be boolean')
        keep=request.get('retain_robots',[]);pause=request.get('pause_after_prepare',False)
        if not isinstance(keep,list) or any(not isinstance(r,str) for r in keep) or set(keep)-robots:
            raise ValueError('Prepared reservations must be robots in this recovery scope')
        if not isinstance(pause,bool) or (resume and (keep or pause)):
            raise ValueError('Reservations and prepared review require resume_goal:false')
        if pause and self.on_review is None:raise ValueError('Prepared review pause is unavailable in this executor')
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
            if not resume:
                for actor,held in list(t.held.items()):
                    if actor in robots and t.arms[actor].grasp_force is None and not _robot_payload_contacts(t,actor,held):
                        t.held.pop(actor)
                        t.event('transport_measured_carrier_cleared',robot=actor,object=held)
                raise PreparationComplete(retain_robots=keep,pause_after_prepare=pause)
            return execute_job(t,runtime,original,resume=True)
        self.counter+=1;new_id=goal['goal_id']+'-recovery-'+str(self.counter)
        if not runtime.recover(ident,new_id,robots,recover):raise ValueError('Requested recovery resources are still busy')
        if new_id in runtime.jobs:
            runtime.jobs[new_id]['manual_recovery_required']=bool(not resume and old.get('manual_recovery_required'))
        return new_id

    def poll(self,t,runtime,jobs,pending,force=False):
        if not force and time.monotonic()<self.next_poll:return False
        if self.inventory_check is not None:
            lost=self.inventory_check(t)
            if lost:
                from .transport_inventory import UnreachableTransportInventory
                if t.motion is not None:t.motion.capture(t.data,force=True)
                t.event('transport_inventory_unreachable',objects=lost)
                raise UnreachableTransportInventory('Required stock left the admitted robot workspace: '+str(lost))
        self.next_poll=time.monotonic()+1.;state=self.observation(t,runtime)
        write_json(self.path/'frontier.json',state);changed=False
        # Explicit owner repairs acquire resources before heuristic retries.
        for file in sorted((self.path/'inbox').glob('*.json')):
            if file.name in self.seen:continue
            try:
                payload=file.read_bytes();request=json.loads(payload)
                if set(request)-{'id','job','scene_sha256','robots','steps','reason','author','resume_goal','retain_robots','pause_after_prepare'}:raise ValueError('Unsupported engineering input')
                if request['scene_sha256']!=t.task['scene_sha256']:raise ValueError('Wrong physical scene')
                ident=request['job']
                if ident not in runtime.jobs or runtime.jobs[ident]['status'] not in ('failed','prepared'):raise ValueError('No matching failed or prepared job')
                new_id=self._retry(t,runtime,ident,request)
                record=dict(id=request['id'],source_file=file.name,sha256=hashlib.sha256(payload).hexdigest(),
                    input=request,admitted_sim_s=t.time,recovery_job=new_id)
                self.external_inputs.append(record);write_json(self.path/'accepted'/file.name,record)
                t.event('transport_engineering_input',**record);self.seen.add(file.name);changed=True
            except (ValueError,KeyError,TypeError) as error:
                write_json(self.path/'rejected'/file.name,dict(error=str(error),sim_time_s=t.time));self.seen.add(file.name)
        # New accepted progress justifies another geometric plan, but a known
        # contact/support fault needs measured owner recovery first.
        for ident,failed in list(state['failed_jobs'].items()):
            if getattr(runtime,'jobs',{}).get(ident,{}).get('manual_recovery_required'):continue
            count=len(state['accepted_goals']);baseline=self.failed_progress.setdefault(ident,count)
            goal=failed['goal']['goal_id']
            if count>self.automatic_progress.get(goal,-1) and (count>baseline or goal in state['accepted_goals']):
                try:self._retry(t,runtime,ident,{});self.automatic_progress[goal]=count;changed=True
                except ValueError:pass
        return changed

    def __call__(self,t,runtime,jobs,pending):
        self.waiting=True
        t.event('transport_engineering_handback',frontier=str(self.path/'frontier.json'),
            scope='Live world retained; independent jobs already continue through the same runtime')
        t.snapshot('recovery-frontier')
        self.poll(t,runtime,jobs,pending,force=True)
        next_wall=time.monotonic()
        while not runtime.active:
            if self.on_idle is not None:self.on_idle(t,runtime)
            t.check_stop()
            if self.poll(t,runtime,jobs,pending,force=self.on_idle is not None):break
            if self.on_idle is not None:continue
            # Keep gravity, contact and holding alive at approximately real
            # time during owner diagnosis. This waiting counts in the trial.
            runtime.step();next_wall+=float(t.model.opt.timestep)
            delay=next_wall-time.monotonic()
            if delay>0:time.sleep(min(delay,.02))
        self.waiting=False
        return runtime.active
