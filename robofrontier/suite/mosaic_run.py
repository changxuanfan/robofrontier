"""Mosaic-only, forward recovery in one live Trial and physics owner thread.

File requests contain finite ordinary commands or named frozen reference calls.
No Python is loaded from requests. Repeated reviews do not transfer ownership,
reset the world, erase feedback, or change the physical evaluator.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import re
import time

import numpy as np

from ..run_setup import write_json
from .oracle import ProcessViolation, ReferenceStopped
from . import mosaic_oracle_v2 as reference


def require_known_idle(trial):
    if trial.monitor.violations:
        raise ProcessViolation('Retained process violation prevents canonical continuation')
    if (any(arm.job is not None for arm in trial.arms.values()) or
            any(result.get('status') == 'uncertain' for action in trial.actions
                for result in action.get('results', {}).values())):
        raise RuntimeError('Unknown or in-flight execution requires original-action review')


class MosaicReview:
    def __init__(self, trial, *, review_wall_s=None, publish=None, on_enter=None):
        self.trial = trial
        self.directory = Path(trial.directory)/'review'
        for name in ('requests', 'results', 'rejected'):
            (self.directory/name).mkdir(parents=True, exist_ok=True)
        self.world_id = hashlib.sha256(str(Path(trial.directory).resolve()).encode()).hexdigest()[:24]
        self.review_wall_s = review_wall_s
        self.epoch = 0
        self.history, self.requests, self.seen_files = [], {}, {}
        self.assisted = False
        self.publish = publish if publish is not None else self.observation
        self.on_enter = on_enter

    def observation(self):
        t = self.trial
        write_json(self.directory/'observation.json', dict(
            world_id=self.world_id, review_id=self.epoch, sim_time_s=t.time,
            updated_at_unix_s=time.time(), held=dict(t.held),
            job=getattr(t, 'mosaic_current_job', None),
            active_jobs=getattr(t,'mosaic_active_jobs',[]),
            evaluation=t.monitor.result(),
            robots={r:a.observe() for r,a in t.arms.items()},
            objects={o['id']:dict(position_m=t.data.body(o['id']).xpos.tolist(),
                rotation=t.data.body(o['id']).xmat.reshape(3,3).tolist()) for o in t.env['objects']},
            last_actions=t.actions[-3:],
            scope='Private privileged engineering observation; never policy input'))

    def _status(self, phase):
        write_json(self.trial.directory/'runner-status.json', dict(
            phase=phase, world_id=self.world_id, review_id=self.epoch,
            simulation_time_s=self.trial.time, updated_at_unix_s=time.time(),
            review_history=self.history, evaluation=self.trial.monitor.result()))

    def _reconcile_released(self, robot):
        t=self.trial;name=t.held.get(robot)
        if name is None:return
        arm=t.arms[robot]
        if (arm.grasp_force is not None or arm.observe()['gripper_width_m']<t.task['cube_size_m']+.003 or
                not t.monitor.released(t.monitor.geoms[name])):
            raise ValueError('Payload is not physically released '+robot+'/'+name)
        velocity=t.monitor.velocity(t.model.body(name).id)
        if np.linalg.norm(velocity[:3])>.1 or np.linalg.norm(velocity[3:])>.01:
            raise ValueError('Released payload is not stable '+name)
        geoms=set(t.monitor.geoms[name]);supported=False
        for contact in t.data.contact:
            a,b=map(int,contact.geom)
            for own,other in ((a,b),(b,a)):
                if own in geoms and contact.dist<=.0005 and (t.model.geom(other).name or '').startswith(('table_','pocket_')):
                    supported=True
        if not supported:raise ValueError('No actual tabletop support for '+name)
        t.held.pop(robot)
        getattr(t,'mosaic_tilted_grasps',{}).pop(robot,None)
        t.event('mosaic_review_reconciled_release',robot=robot,object=name)

    def submit(self, request, *, assisted=True):
        request=json.loads(json.dumps(request,allow_nan=False))
        ident=request.get('id')
        if not isinstance(ident,str) or re.fullmatch(r'[A-Za-z0-9_-]{1,80}',ident) is None:
            raise ValueError('Use a bounded request ID')
        if ident in self.requests:
            old=self.requests[ident]
            if old['request']!=request:raise ValueError('Request ID reused with different input')
            return deepcopy(old)
        if set(request)-{'id','world_id','review_id','operation','batches','robot','object','target_m'}:
            raise ValueError('Unsupported request fields')
        if request.get('world_id')!=self.world_id or request.get('review_id')!=self.epoch:
            raise ValueError('Request belongs to another world or stale review')
        require_known_idle(self.trial)
        op=request.get('operation');t=self.trial
        if op not in ('commands','resume','finish_held','reconcile_released','adopt_hold','stop'):
            raise ValueError('Unsupported recovery operation')
        if op=='commands':
            batches=request.get('batches')
            if not isinstance(batches,list) or not 1<=len(batches)<=16:
                raise ValueError('Use 1..16 finite command batches')
            for batch in batches:
                if not isinstance(batch,dict) or not batch or not set(batch)<=set(t.arms):
                    raise ValueError('Invalid robot batch')
                for operation,parameters in batch.values():
                    if operation not in ('move','gripper') or not isinstance(parameters,dict):
                        raise ValueError('Only ordinary move/gripper commands')
        if op=='resume':
            if t.held or any(getattr(arm,'grasp_force',None) is not None for arm in t.arms.values()):
                raise ValueError('Finish or reconcile held payloads and force-held grippers before rematching')
            reference.protected_remaining_inputs(t,getattr(t,'mosaic_protected_assignments',{}))
        if op in ('finish_held','adopt_hold','reconcile_released') and request.get('robot') not in t.arms:
            raise ValueError('Select an actual robot')
        if op=='finish_held':
            robot=request['robot'];job=getattr(t,'mosaic_current_job',None)
            name=t.held.get(robot)
            if name is None:raise ValueError('No tracked held payload')
            job=next((j for j in getattr(t,'mosaic_active_jobs',[]) if j['object']==name),job)
            if 'target_m' in request:
                target=np.asarray(request['target_m'],dtype=float)
                if target.shape!=(3,) or not np.isfinite(target).all():raise ValueError('Finite 3D staging target required')
            else:
                if not job or job['object']!=name or robot not in job['path']:
                    raise ValueError('No matching current job; supply a reviewed staging target')
                index=job['path'].index(robot)
                target=(np.asarray(next(g['position_m'] for g in t.task['goals'] if g['goal_id']==job['goal_id']))
                    if index==len(job['path'])-1 else t.mosaic_edges[frozenset(job['path'][index:index+2])])
        record=dict(request=request,status='running',feedback=[])
        self.requests[ident]=record
        write_json(self.directory/'results'/f'{ident}.json',record)
        self.assisted|=assisted
        t.event('mosaic_review_request',request=request,assisted=assisted)
        try:
            if op=='stop':raise ReferenceStopped('Explicit owner stop during review')
            if op=='commands':
                for index,batch in enumerate(batches):
                    feedback=t.execute(f'mosaic-recovery-{ident}-{index}',batch,allow_failure=True)
                    record['feedback'].append(deepcopy(feedback))
                    require_known_idle(t)
                    if any(v['status']!='completed' for v in feedback.values()):
                        record['status']='failed';break
            elif op=='finish_held':
                reference.place_cube(t,robot,target)
                reference.helpers.park(t,robot)
                t.wait(.7)
            elif op=='reconcile_released':self._reconcile_released(request['robot'])
            elif op=='adopt_hold':
                robot=request['robot'];name=request.get('object')
                if name not in {o['id'] for o in t.env['objects']} or name in t.monitor.result().get('assignments',{}).values():
                    raise ValueError('Cannot adopt an absent or completed object')
                if t.held.get(robot) not in (None,name):raise ValueError('Robot already owns another payload')
                if t.arms[robot].grasp_force is None or not t.finger_contacts(robot,name):
                    raise ValueError('No actual force-held bilateral support')
                for _ in range(10):
                    t.wait(.02)
                    if not t.finger_contacts(robot,name):raise ValueError('Support lost during observation')
                t.held[robot]=name;t.event('mosaic_review_adopted_hold',robot=robot,object=name)
            record['status']='completed' if record['status']=='running' else record['status']
        except (ProcessViolation,ReferenceStopped):
            record['status']='interrupted';raise
        except (RuntimeError,ValueError) as error:
            record.update(status='failed',error=f'{type(error).__name__}: {error}')
            require_known_idle(t)
        finally:
            record['finished_sim_s']=t.time
            write_json(self.directory/'results'/f'{ident}.json',record)
        return deepcopy(record)

    def await_resolution(self, error):
        require_known_idle(self.trial)
        self.epoch+=1
        boundary=dict(review_id=self.epoch,error=f'{type(error).__name__}: {error}',
            start_sim_s=self.trial.time,protected_assignments=dict(getattr(self.trial,'mosaic_protected_assignments',{})))
        self.history.append(boundary)
        self.trial.event('mosaic_review_opened',**boundary)
        self._status('review');self.publish()
        if self.on_enter is not None:self.on_enter(self)
        start=time.monotonic();last_publish=start
        while True:
            self.trial.check_stop();require_known_idle(self.trial)
            for path in sorted((self.directory/'requests').glob('*.json')):
                payload=path.read_bytes()
                digest=hashlib.sha256(payload).hexdigest()
                if self.seen_files.get(str(path))==digest:continue
                try:
                    request=json.loads(payload)
                    # Future decisions are not consumed before their boundary.
                    if request.get('review_id',0)>self.epoch:continue
                    self.seen_files[str(path)]=digest
                    result=self.submit(request)
                except (ValueError,TypeError,KeyError,json.JSONDecodeError) as rejected:
                    write_json(self.directory/'rejected'/f'{digest}.json',dict(error=str(rejected)))
                    continue
                if (result['status']=='completed' and request['operation']=='resume' and
                        request['review_id']==self.epoch):
                    boundary.update(resolved_sim_s=self.trial.time,request_id=request['id'])
                    self.trial.event('mosaic_review_resumed',**boundary)
                    self._status('running');return
            now=time.monotonic()
            if self.review_wall_s is not None and now-start>self.review_wall_s:
                raise ReferenceStopped('Explicit review deadline exhausted')
            self.trial.wait(.02)
            if now-last_publish>=2:
                self.publish();self._status('review');last_publish=now
            # Review holds advance real physics at approximately wall rate.
            time.sleep(max(0.,.02-(time.monotonic()-now)))


def run_reference(trial, review, *, solver=reference.solve, max_goals=None, motion_profile=None, max_reviews=None, goal_ids=None):
    resume=False
    while True:
        try:
            remaining=None if max_goals is None else max_goals-len(trial.monitor.result().get('assignments',{}))
            if remaining is not None and remaining<=0:
                return dict(reference_placed=len(trial.monitor.result().get('assignments',{})),
                    total_goals=len(trial.task['goals']),commissioning_only=max_goals<len(trial.task['goals']),resumed=resume)
            extra={}
            if goal_ids is not None:
                completed=trial.monitor.result().get('assignments',{})
                extra['goal_ids']=[goal for goal in goal_ids if goal not in completed]
            return solver(trial,max_goals=remaining,motion_profile=motion_profile,resume=resume,**extra)
        except (ProcessViolation,ReferenceStopped):raise
        except RuntimeError as error:
            require_known_idle(trial)
            if max_reviews is not None and len(review.history)>=max_reviews:
                raise ReferenceStopped('Explicit review count budget exhausted')
            review.await_resolution(error)
            resume=True
