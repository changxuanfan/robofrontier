"""Keep one known physical world alive for typed, owner-reviewed Oracle repair.

This engineering interface exposes privileged observations. It is not a GPT
policy tool, physical restart checkpoint, or arbitrary Python execution hook.
"""
import hashlib
import json
import math
from pathlib import Path
import re
import time
import uuid

import numpy as np
from ..run_setup import write_json
from . import relay_oracle_v2 as ref


def continuation_options(value):
    fields={'future_region','future_yaw','future_fraction','future_height','future_waypoints','landing_point','require_topdown'}
    if not isinstance(value,dict) or set(value)-fields:raise ValueError('Unsupported receiving continuation field')
    if value and 'future_region' not in value:raise ValueError('Receiving continuation needs its reference region')
    def point(v):
        p=np.asarray(v,dtype=float)
        if p.shape!=(3,) or not np.isfinite(p).all():raise ValueError('Finite continuation XYZ required')
    for key in ['future_region','landing_point']:
        if key in value:point(value[key])
    for key in ['future_yaw','future_fraction','future_height']:
        if key in value and not math.isfinite(float(value[key])):raise ValueError('Finite continuation scalar required')
    if 'future_fraction' in value and not 0<=value['future_fraction']<=1:raise ValueError('Continuation fraction outside0..1')
    waypoints=value.get('future_waypoints',[])
    if not isinstance(waypoints,list) or len(waypoints)>4:raise ValueError('At most four finite continuation waypoints')
    for step in waypoints:
        if not isinstance(step,(list,tuple)) or len(step)!=2:raise ValueError('Continuation waypoint is XYZ,yaw')
        point(step[0])
        if not math.isfinite(float(step[1])):raise ValueError('Finite waypoint yaw required')
    if 'require_topdown' in value and not isinstance(value['require_topdown'],bool):raise ValueError('Topdown preference must be boolean')
    return dict(value)


class RelayLiveRecovery:
    def __init__(self,trial,report):
        self.trial=trial;self.report=report;self.directory=trial.directory/'recovery'
        for name in ('requests','results','accepted','rejected'):(self.directory/name).mkdir(parents=True,exist_ok=True)
        self.world_id=uuid.uuid4().hex;self.handled={};self.assisted=False
        write_json(self.directory/'world.json',dict(world_id=self.world_id,pid=__import__('os').getpid(),
            scope='Same live privileged Oracle world; finite typed repair; no reset/pose writes',
            operations=['carry','place','release','place_release','handover','grasp','open','stop','wait','resume']))

    def publish(self,error=None):
        t=self.trial;result=t.monitor.result()
        payloads={}
        for item in t.task['mechanism']['payload_assemblies']:
            ident=item['id'];body=t.data.body(ident)
            payloads[ident]=dict(position_m=body.xpos.tolist(),rotation=body.xmat.reshape(3,3).tolist(),
                actual_support=sorted(t.monitor.bilateral(t.monitor.relay_v2.states[ident]['rod_geoms'],positive_force=True)),
                state=result['payloads'][ident])
        write_json(self.directory/'observation.json',dict(world_id=self.world_id,sim_time_s=t.time,
            wall_elapsed_s=time.monotonic()-t.started,error=error,held=dict(t.held),
            active_jobs={a:r.job for a,r in t.arms.items() if r.job is not None},
            robots={a:r.observe() for a,r in t.arms.items()},payloads=payloads,goal=t.task['route'][-1],
            evaluation=result,visibility='Privileged engineering Oracle only'))
        try:t.snapshot('recovery-live',('global','global_rear'))
        except Exception as image_error:write_json(self.directory/'image-error.json',dict(error=repr(image_error)))

    def process(self,request):
        ident=request.get('id');digest=hashlib.sha256(json.dumps(request,sort_keys=True,allow_nan=False).encode()).hexdigest()
        if not isinstance(ident,str) or re.fullmatch(r'[A-Za-z0-9_-]{1,80}',ident) is None:raise ValueError('Invalid repair ID')
        if ident in self.handled:
            old=self.handled[ident]
            if old['request_sha256']!=digest:raise ValueError('Repair ID reused with different input')
            return old
        if request.get('world_id')!=self.world_id:raise ValueError('Request belongs to another world')
        if set(request)-{'id','world_id','op','payload','robots','target_m','yaw_rad','retain_attitude','offset_m','seconds','review','assisted','retained_side','continuation'}:raise ValueError('Unsupported repair field')
        if not request.get('review') or request.get('assisted') is not True:raise ValueError('External model repair requires review and assisted attestation')
        t=self.trial
        op=request.get('op')
        if any(a.job is not None for a in t.arms.values()) and op!='stop':raise ValueError('Original execution unresolved; request controlled stop, never replay')
        if t.monitor.violations:raise ValueError('Irreversible process history retained; repair cannot erase it')
        if op not in {'carry','place','release','place_release','handover','grasp','open','stop','wait','resume'}:raise ValueError('No state/reset/physics editing operation')
        payload=request.get('payload');team=request.get('robots',[])
        if op not in {'wait','resume','stop'}:
            if payload not in t.monitor.relay_v2.states:raise ValueError('Unknown payload')
            if len(team)<2 or len(set(team))!=len(team) or any(a not in t.arms for a in team):raise ValueError('Use actual existing support robots')
            support=t.monitor.bilateral(t.monitor.relay_v2.states[payload]['rod_geoms'],positive_force=True)
            if op not in {'grasp','open'} and not set(team)<=support:raise ValueError('Declared holders lack actual bilateral support')
        continuation=continuation_options(request.get('continuation',{}))
        if continuation and op!='handover':raise ValueError('Receiving continuation applies only to handover')
        side=request.get('retained_side')
        if side is not None and (op!='handover' or type(side) is not int or side not in (0,1) or len(team)!=2):raise ValueError('Retain one side of an existing two-holder handover')
        self.assisted=True;self.report['execution_mode']='privileged_assisted';self.report['llm_used']=True
        write_json(self.directory/'accepted'/(ident+'.json'),request)
        t.event('relay_repair_admitted',request=request,request_sha256=digest,world_id=self.world_id)
        result=dict(id=ident,request_sha256=digest,status='running',world_id=self.world_id,start_sim_s=t.time)
        try:
            if op=='carry':
                point=np.asarray(request['target_m'],dtype=float)
                if point.shape!=(3,) or not np.isfinite(point).all():raise ValueError('Finite XYZ required')
                rotation=t.data.body(payload).xmat.reshape(3,3).copy();yaw=float(request.get('yaw_rad',math.atan2(rotation[1,0],rotation[0,0])))
                if not math.isfinite(yaw):raise ValueError('Finite yaw required')
                ref.carry(t,payload,team,point,yaw,target_rotation=rotation if request.get('retain_attitude',True) else None)
            elif op in {'place','place_release'}:
                ref.place(t,payload,team);t.wait(.3)
                if op=='place_release':ref.release(t,payload,team,terminal=True);t.wait(1.2)
            elif op=='release':
                if not ref.final_support(t,payload,team):raise ValueError('Establish real in-goal beam support before releasing')
                ref.release(t,payload,team,terminal=True);t.wait(1.2)
            elif op=='handover':
                offset=request.get('offset_m',.32);body=t.data.body(payload)
                retained={} if side is None else {side:team[side]}
                receivers=ref.choose_team(t,payload,body.xpos.copy(),body.xmat.reshape(3,3).copy(),offset,exclude=team,search_budget_s=60.,retained=retained,**continuation)
                ref.grasp(t,payload,receivers,offset);t.wait(.3);ref.release(t,payload,[robot for robot in team if robot not in receivers])
                result['receivers']=receivers
            elif op=='grasp':
                state=t.monitor.relay_v2.states[payload]
                if state['dropped'] and not set(team)<=state['custody']:raise ValueError('Only original custody may recover a drop')
                body=t.data.body(payload);offset=request.get('offset_m',.44)
                continuing=set(team)&support
                if len(continuing)>1:raise ValueError('Already supported; use an explicit retained handover for regrasp')
                retained={}
                if continuing:
                    if not set(team)<=state['custody']:raise ValueError('Partial support recovery belongs to original custody')
                    robot=next(iter(continuing));arm=t.arms[robot]
                    local=body.xmat.reshape(3,3).T@(t.data.site_xpos[arm.site]-body.xpos)
                    retained={int(local[1]>=0):robot}
                selected=ref.choose_team(t,payload,body.xpos.copy(),body.xmat.reshape(3,3).copy(),offset,
                    exclude=(set(t.arms)-set(team))|continuing,retained=retained,departing=(),search_budget_s=60.)
                ref.grasp(t,payload,selected,offset);result['holders']=selected
            elif op=='open':
                state=t.monitor.relay_v2.states[payload]
                if not t.monitor.relay_v2.environment_contact(state['all_geoms']):raise ValueError('Opening recovery requires actual table support')
                t.execute('recovery-open-supported',{a:('gripper',dict(mode='move',width_m=.08,speed_m_s=.06)) for a in team})
                for a in team:
                    if not ref.arm_payload_contact(t,payload,a):t.held.pop(a,None)
            elif op=='stop':
                original={a:r.job for a,r in t.arms.items() if r.job is not None};feedback={};started=t.time
                for a in original:t.arms[a].request_stop(t.time,'Owner controlled stop for recovery')
                while any(t.arms[a].job is not None for a in original) and t.time-started<15.:
                    t.step()
                    for a in original:
                        value=t.arms[a].poll(t.time,15)
                        if value is not None:feedback[a]=value
                result['original_jobs']=original;result['original_feedback']=feedback
                if any(t.arms[a].job is not None for a in original):raise RuntimeError('Original stop remains unresolved')
            elif op=='wait':
                seconds=float(request.get('seconds',.5))
                if not 0<=seconds<=2:raise ValueError('Finite wait0..2s only')
                t.wait(seconds)
            elif op=='resume':
                if t.held:raise ValueError('Finish the held frontier before restarting remaining tabletop work')
                result['resume_remaining']=True
            result['status']='completed'
        except Exception as error:
            result.update(status='failed',error=type(error).__name__+': '+str(error))
        result.update(end_sim_s=t.time,evaluation=t.monitor.result(),unresolved=[a for a,r in t.arms.items() if r.job is not None])
        self.handled[ident]=result;write_json(self.directory/'results'/(ident+'.json'),result)
        t.event('relay_repair_result',detail=result);self.report.setdefault('repair_results',[]).append(result)
        return result

    def wait_for_repair(self,error):
        t=self.trial
        self.report.setdefault('recoverable_failures',[]).append(dict(error=repr(error),sim_time_s=t.time))
        t.event('relay_recovery_boundary',error=repr(error),world_id=self.world_id)
        self.publish(str(error));next_publish=time.monotonic()+2.;anchor_wall=time.monotonic();anchor_sim=t.time
        while True:
            t.check_stop()
            if t.monitor.result().get('task_success') and not any(a.job is not None for a in t.arms.values()):
                t.wait(.5);return 'complete'
            for path in sorted((self.directory/'requests').glob('*.json')):
                try:
                    request=json.loads(path.read_text());result=self.process(request)
                    path.unlink()
                    self.publish(result.get('error'))
                    if result.get('resume_remaining') and result['status']=='completed':return 'resume'
                except Exception as rejection:
                    write_json(self.directory/'rejected'/path.name,dict(error=repr(rejection)));path.unlink()
                anchor_wall=time.monotonic();anchor_sim=t.time
            # Keep force holds/physics/recording alive while the owner thinks.
            desired=anchor_sim+(time.monotonic()-anchor_wall)
            # A large world may integrate slower than real time. Poll between
            # short batches instead of letting accumulated wall debt starve
            # queued repairs. Every ordinary physics step is still recorded.
            for _ in range(25):
                if t.time+.001>=desired:break
                t.step()
            if time.monotonic()>=next_publish:self.publish(str(error));next_publish=time.monotonic()+2.
            time.sleep(.01)
