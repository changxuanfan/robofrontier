"""Cooperative Oracle execution in one physics thread, with explicit leases.

Only execute/wait yield. Existing synchronous geometric planners therefore
read coherent data and can be reused without threads writing a shared world.
This runtime selects no objects, destinations, routes or recovery policies.
"""
from copy import deepcopy
import math
import threading

import numpy as np
from greenlet import getcurrent, greenlet


class TransportRuntime:
    def __init__(self, trial, admission=None):
        if any(a.job is not None for a in trial.arms.values()):
            raise ValueError('Attach at a known idle boundary')
        self.t=trial;self.owner=getcurrent();self.thread=threading.get_ident()
        self.jobs={};self.leases={};self.requests={};self.running={};self.sequence=0
        self.admission=admission;self.original=(trial.execute,trial.wait,trial.step)
        trial.execute=self.execute;trial.wait=self.wait;trial.step=self.step

    def settled(self,r):
        arm=self.t.arms[r]
        return (arm.job is None and not getattr(arm,'uncertainty_reported',False)
                and getattr(arm,'grasp_force',None) is None and r not in self.t.held)

    def acquire(self,robots):
        ident,job=self._current();robots=set(robots)
        if any(r in self.leases and self.leases[r]!=ident for r in robots):return False
        if any(r not in job['robots'] and not self.settled(r) for r in robots):return False
        for r in robots:self.leases[r]=ident
        job['robots'].update(robots);return True

    def recover(self,ident,new_ident,robots,callback):
        """Transfer a failed, known-idle frontier without clearing its history."""
        old=self.jobs[ident];robots=set(robots)
        if old['status']!='failed':raise ValueError('Recovery requires a failed job boundary')
        retained={r for r,owner in self.leases.items() if owner==ident}
        if retained-robots:raise ValueError('Recovery must account for every retained robot')
        if any(r in self.leases and self.leases[r]!=ident for r in robots):return False
        if any(self.t.arms[r].job is not None or getattr(self.t.arms[r],'uncertainty_reported',False) for r in robots):
            raise ValueError('Unknown or active actions cannot be adopted or replayed')
        for r in retained:self.leases.pop(r)
        if not self.add(new_ident,robots,callback):raise RuntimeError('Recovery admission unexpectedly changed')
        self.jobs[new_ident].update(object=old.get('object'),goal=old.get('goal'),recovery_of=ident)
        old['recovered_by']=new_ident
        self.t.event('transport_recovery_admitted',previous_job=ident,job=new_ident,robots=sorted(robots))
        return True

    def step(self):
        if getcurrent() is not self.owner or threading.get_ident()!=self.thread:
            raise RuntimeError('Only the Oracle runtime owner may step physics')
        self.original[2]()

    def _current(self):
        current=getcurrent()
        for ident,job in self.jobs.items():
            if job['fiber'] is current:return ident,job
        raise RuntimeError('Robot operation requires an assigned Oracle job')

    def execute(self,label,commands,allow_failure=False):
        ident,job=self._current()
        if not commands:return {}
        if set(commands)-job['robots']:
            raise ValueError('Job attempted to operate an unleased robot')
        # Copy at submission; later planner edits cannot alter an admitted action.
        return self.owner.switch(dict(kind='execute',job=ident,label=label,
            commands=deepcopy(commands),allow_failure=allow_failure))

    def wait(self,seconds):
        ident,_=self._current()
        if not math.isfinite(seconds) or seconds<0:raise ValueError('Finite nonnegative wait required')
        return self.owner.switch(dict(kind='wait',job=ident,until=self.t.time+seconds))

    def add(self,ident,robots,callback):
        robots=set(robots)
        if ident in self.jobs or robots-set(self.t.arms):raise ValueError('Invalid Oracle job')
        if robots & set(self.leases):return False
        def run(*_):
            try:return dict(kind='done',job=ident,result=callback())
            except Exception as error:return dict(kind='failed',job=ident,error=error)
        self.jobs[ident]=dict(robots=robots,fiber=greenlet(run,parent=self.owner),status='ready')
        for r in robots:self.leases[r]=ident
        self.t.event('transport_job_admitted',job=ident,robots=sorted(robots))
        return True

    def release(self,robots):
        ident,job=self._current()
        for r in robots:
            if r not in job['robots'] or not self.settled(r):
                raise ValueError('Only settled, unloaded job robots can be released')
            self.leases.pop(r);job['robots'].remove(r)
        self.t.event('transport_resources_released',job=ident,robots=sorted(robots))

    def _resume(self,ident,value=None):
        self._handle(ident,self.jobs[ident]['fiber'].switch(value))

    def _handle(self,ident,message):
        job=self.jobs[ident]
        kind=message['kind']
        if kind in ('done','failed'):
            job.update(status=kind,result=message.get('result'),error=message.get('error'))
            # A failed load retains its owner; independent jobs can finish,
            # but the scheduler cannot pretend this arm became available.
            for r in list(job['robots']):
                if self.settled(r):
                    self.leases.pop(r,None)
            self.t.event('transport_job_'+kind,job=ident,
                error=str(message.get('error')) if kind=='failed' else None,
                retained_robots=sorted(r for r in job['robots'] if r in self.leases))
        else:
            self.requests[ident]=message;job['status']='waiting'

    def cycle(self):
        """Admit ready work, advance one step, and deliver completed results."""
        t=self.t;t.check_stop()
        for ident,job in list(self.jobs.items()):
            if job['status']=='ready':self._resume(ident)
        changed=True
        while changed:
            changed=False
            for ident,request in list(self.requests.items()):
                if request['kind']=='wait':
                    if t.time+1e-9<request['until']:continue
                    self.requests.pop(ident);self._resume(ident);changed=True;continue
                commands=request['commands']
                if any(r in self.running for r in commands):continue
                # A denied nominal sweep waits for a running action boundary.
                # Retesting the identical request every 2 ms wastes wall time
                # without admitting it any earlier under this policy.
                running_revision=tuple(sorted((r,id(rec)) for r,rec in self.running.items()))
                if request.get('blocked_on')==running_revision:continue
                if self.admission is not None and not self.admission(request,self):
                    if self.running:
                        request['blocked_on']=running_revision;continue
                    self.requests.pop(ident)
                    error=RuntimeError('No collision-free sampled free-arm route '+request['label']+' [runtime admission]')
                    # Throw at execute(), preserving the planner's ordinary
                    # bounded alternative-route handling.
                    self._handle(ident,self.jobs[ident]['fiber'].throw(error))
                    changed=True;continue
                try:
                    prepared={r:t.arms[r].validate(op,p,t.cfg,t.task['workspace_world_m']['lower'],
                        t.task['workspace_world_m']['upper']) for r,(op,p) in commands.items()}
                except (ValueError,TypeError,KeyError) as error:
                    self.requests.pop(ident);self._handle(ident,self.jobs[ident]['fiber'].throw(error));changed=True;continue
                self.requests.pop(ident);self.sequence+=1;call=f'transport-{self.sequence}'
                record=dict(label=request['label'],commands=commands,start_sim_s=t.time,
                    pending=set(commands),results={},allow_failure=request['allow_failure'],job=ident)
                self.jobs[ident]['status']='executing'
                t.event('action_start',label=record['label'],commands=commands,oracle_job=ident)
                for r,command in prepared.items():
                    t.arms[r].start(call+'-'+r,command,t.time);self.running[r]=record
                changed=True
        if not self.active:return
        t.step()
        if t.tick%t.stride:return
        finished=[]
        for r,record in list(self.running.items()):
            result=t.arms[r].poll(t.time,15)
            if result is None:continue
            record['results'][r]=result;record['pending'].remove(r);self.running.pop(r)
            if not record['pending']:finished.append(record)
        for record in finished:
            rec={k:record[k] for k in ('label','commands','results','start_sim_s')};rec['end_sim_s']=t.time
            t.actions.append(rec);t.event('action_end',label=rec['label'],results=rec['results'],oracle_job=record['job'])
            if not record['allow_failure'] and any(r['status']!='completed' for r in rec['results'].values()):
                self._handle(record['job'],self.jobs[record['job']]['fiber'].throw(RuntimeError(rec['label']+': '+str(rec['results']))))
            else:self._resume(record['job'],rec['results'])

    @property
    def active(self):
        return any(j['status'] not in ('done','failed') for j in self.jobs.values())

    def close(self):
        if self.active or self.running:raise RuntimeError('Cannot detach with unresolved Oracle jobs')
        self.t.execute,self.t.wait,self.t.step=self.original


def sampled_motion_admission(request,runtime):
    """Private sampled whole-scene nominal joint sweep, not certification.

    A conflicting new move waits for currently running actions, rather than
    interrupting them. Actual process monitoring remains authoritative.
    """
    from .transport_oracle_v2 import _planning_copy
    from ..robot_control import matrix_quat,quat_matrix
    import mujoco
    t=runtime.t;new={r:c for r,c in request['commands'].items() if c[0]=='move'}
    if not new:return True
    commands={r:rec['commands'][r] for r,rec in runtime.running.items() if rec['commands'][r][0]=='move'}
    commands.update(new);s=_planning_copy(t);original=s.data.qpos.copy();paths={}
    for r,(op,p) in commands.items():
        arm=s.arms[r];start=t.data.qpos[arm.qids].copy()
        if 'joint_rad' in p:end=np.asarray(p['joint_rad'])
        else:
            point=np.asarray(p.get('target_m',t.data.site_xpos[arm.site]+np.asarray(p.get('delta_m',[0,0,0]))))
            rotation=quat_matrix(p['quaternion_wxyz']) if 'quaternion_wxyz' in p else t.data.site_xmat[arm.site].reshape(3,3)
            end=s.screen_ik(r,point,rotation,seed=start)
            if end is None:return False
        paths[r]=(start,end)
    count=max(2,max(math.ceil(float(np.max(abs(b-a)))/.025) for a,b in paths.values()))
    carried={};owners={}
    for ident,job in runtime.jobs.items():
        name=job.get('object')
        if name is None:continue
        for r in job['robots']:
            owners[r]=name
            if t.arms[r].grasp_force is not None and t.finger_contacts(r,name):
                body=t.data.body(name);br=body.xmat.reshape(3,3);arm=t.arms[r]
                carried[r]=(name,br.T@(t.data.site_xpos[arm.site]-body.xpos),br.T@t.data.site_xmat[arm.site].reshape(3,3))
    s.held={}
    for f in np.linspace(0,1,count+1)[1:]:
        s.data.qpos[:]=original
        for r,(a,b) in paths.items():s.data.qpos[s.arms[r].qids]=a+(b-a)*f
        mujoco.mj_kinematics(s.model,s.data)
        for r,(name,offset,relative) in carried.items():
            arm=s.arms[r];rr=s.data.site_xmat[arm.site].reshape(3,3)@relative.T
            point=s.data.site_xpos[arm.site]-rr@offset
            s.data.joint(name+'_free').qpos[:]=np.r_[point,matrix_quat(rr)]
        mujoco.mj_forward(s.model,s.data)
        for r in paths:
            if not s.pose_clear(r,s.data.qpos[s.arms[r].qids],(owners[r],) if r in owners else (),clearance_m=.001):return False
    return True
