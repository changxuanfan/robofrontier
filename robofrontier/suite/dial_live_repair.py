"""Oracle-only live repair channel. Never reads passwords or assigns state.

A recoverable reference failure retains the world and blind solver. Research
can inspect timestamped geometry/images and submit ordinary bounded actions.
All interventions and failed plans remain in the episode record. This is not
an autonomous policy harness and grants no new observations to GPT task roles.
"""
import json
import time
from pathlib import Path
from ..run_setup import write_json
from .oracle import ReferenceStopped
from .oracle_recovery import OracleRecovery,RecoverablePlanError
import hashlib


class LiveRepair:
    def __init__(self,trial,operator):
        self.t=trial;self.m=operator;self.index=0;self.path=trial.directory/'recovery';self.path.mkdir()
        self.interventions=0;self.adapter=None;self.owner="dial-reference-owner"

    def settle(self):
        t=self.t;old=t.abort_on_process_violation;t.abort_on_process_violation=False
        try:
            if self.adapter is not None and self.adapter.attached:
                self.adapter.pause(self.owner)
                for _ in range(800):
                    if not self.adapter.running:break
                    self.adapter.advance(.02)
                if self.adapter.running or self.adapter.unknown:raise RuntimeError('Recovery waits for unresolved executor feedback')
                self.adapter.resume(self.owner);return
            for arm in t.arms.values():
                if arm.job is not None:arm.request_stop(t.time,reason='reference_recovery_boundary')
            for _ in range(round(16/t.model.opt.timestep)):
                busy=[a for a in t.arms.values() if a.job is not None]
                if not busy:return
                t.step()
                for arm in busy:
                    result=arm.poll(t.time,15.)
                    if result:t.event('reference_recovery_terminal_action',robot=arm.name,result=result)
            raise RuntimeError('Recovery cannot replay unknown active actions')
        finally:t.abort_on_process_violation=old and not bool(t.monitor.violations)

    def devices(self):
        t=self.t
        devices={}
        for a in self.m.dials:
            body=t.data.body(a['id']+'_module')
            devices[a['id']]=dict(position_m=body.xpos.tolist(),quaternion_wxyz=body.xquat.tolist(),
                angle_deg=float(t.data.joint(a['id']+'_hinge').qpos[0]*180/3.141592653589793),
                blue_ready=bool(t.monitor.dial_ready[a['index']]),nominal_robots=a['nominal_robots'])
        return devices

    def snapshot(self,error,guess):
        t=self.t;folder=self.path/f'pause-{self.index:03d}';folder.mkdir()
        t.snapshot('recovery-'+str(self.index))
        state=dict(pause=self.index,phase='awaiting_repair',error=str(error),sim_time_s=t.time,requested=guess,
                   best=t.monitor.game.display,robots={r:a.observe() for r,a in t.arms.items()},devices=self.devices(),
                   violations=t.monitor.violations,private_password_included=False,
                   waiting_physics='Paused at current live state; wall budget continues; not evidence of physical holding duration',
                   command_schema='plan.json: {source:research_gpt, batches:[{actor:[move|gripper,params]}], resume:true}')
        write_json(folder/'state.json',state);write_json(self.path/'awaiting.json',dict(pause=self.index,folder=str(folder),error=str(error)))
        t.event('reference_awaiting_live_repair',pause=self.index,error=str(error),folder=str(folder),
                scope='World/solver retained; no qpos restore, no violation removal')
        return folder

    def sync(self):
        t=self.t
        for a in self.m.dials:
            grip=self.m.grips.get(a['id'])
            if grip and t.arms[grip['robot']].grasp_force is None and t.monitor.released(self.m.red_geoms(a)):
                self.m.grips.pop(a['id']);t.event('reference_forget_released_grip',dial=a['id'])
            if t.task['level']>1:
                robot=a['nominal_robots'][1]
                if t.arms[robot].grasp_force is None:t.held.pop(robot,None)
                elif robot in t.monitor.bilateral([g for g,i in t.monitor.blue.items() if i==a['index']]):t.held[robot]=a['id']+'_module'

    def recover(self,error,guess):
        if isinstance(error,ReferenceStopped):raise error
        if str(error).startswith(('Numerical physics failure','Nonfinite physical state','Planning scratch cannot')):raise error
        t=self.t;self.settle();self.index+=1;folder=self.snapshot(error,guess)
        waiting_started=time.monotonic()
        if t.monitor.violations:
            t.continued_after_violation=True;t.continue_after_recorded_violation=True
        if self.adapter is None:
            world=hashlib.sha256(str(t.directory).encode()).hexdigest()[:16]
            self.adapter=OracleRecovery(t,owner=self.owner,world_id=world,observe=lambda:{
                'task':{'id':t.task['id'],'best':t.monitor.game.display},
                'robots':{r:a.observe() for r,a in t.arms.items()},'objects':self.devices()})
        elif not self.adapter.attached:self.adapter.reattach(self.owner)
        recovery=self.adapter.recover(RecoverablePlanError(str(error)),list(t.arms),evidence={
            'task':{'reason':str(error),'requested':guess},'objects':self.devices()})
        while True:
            t.check_stop();plan=folder/'plan.json'
            if not plan.exists():
                t._heartbeat('awaiting_recovery');time.sleep(.25);continue
            self.interventions+=1;t.oracle_repair_interventions=self.interventions
            try:
                payload=plan.read_text();plan.rename(folder/f'plan-{self.interventions:03d}.json');request=json.loads(payload)
                if set(request)-{'source','batches','resume'} or request.get('source')!='research_gpt':raise ValueError('Bad repair envelope')
                t.event('reference_live_repair_plan',pause=self.index,intervention=self.interventions,plan=request,
                        scope='Online research GPT physical recovery through existing OracleRecovery; decoder remains blind')
                statuses=[]
                for i,batch in enumerate(request.get('batches',[])):
                    if not isinstance(batch,dict) or not batch:raise ValueError('Repair batch must map actors to commands')
                    identifiers=[]
                    for actor,command in batch.items():
                        if actor not in t.arms or not isinstance(command,list) or len(command)!=2 or command[0] not in ('move','gripper'):raise ValueError('Only existing robot commands are accepted')
                        ident=f'dial-{self.index}-{self.interventions}-{i}-{actor}';identifiers.append(ident)
                        self.adapter.submit(dict(owner=self.owner,world_id=self.adapter.world_id,id=ident,robot=actor,
                            sequence=[dict(op=command[0],parameters=command[1])],review='Research GPT physical recovery plan',
                            recovery_id=recovery['id'],assisted=True))
                    while self.adapter.running:self.adapter.advance(.02)
                    statuses.extend(self.adapter.status(ident) for ident in identifiers)
                    if any(s['status']!='completed' for s in statuses):raise RuntimeError('A repair action failed; feedback retained, world remains available')
                self.sync();t.snapshot('repair-applied-'+str(self.interventions))
                write_json(folder/f'result-{self.interventions:03d}.json',dict(applied=True,sim_time_s=t.time,
                    robots={r:a.observe() for r,a in t.arms.items()},devices=self.devices(),violations=t.monitor.violations,requests=statuses))
                if request.get('resume'):
                    self.adapter.acknowledge(self.owner,recovery['id'],'Owner resumes the reference from observed current state',assisted=True)
                    self.adapter.handoff(self.owner)
                    t.event('reference_live_repair_resumed',pause=self.index,repair_wall_s=time.monotonic()-waiting_started,
                        waiting_physics='Explicit engineering pause; no integration during owner reasoning')
                    write_json(self.path/'awaiting.json',dict(pause=self.index,phase='resuming',sim_time_s=t.time));return
            except ReferenceStopped:raise
            except Exception as failure:
                self.settle();write_json(folder/f'result-{self.interventions:03d}.json',dict(applied=False,error=str(failure),sim_time_s=t.time))
                t.event('reference_live_repair_failed',pause=self.index,error=str(failure))
