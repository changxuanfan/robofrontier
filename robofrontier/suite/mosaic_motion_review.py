"""Mosaic-only offline motion review. Proposals never change source/edit files.

Protect recorded actions, commanded force holds, progress and release events.
Inspect the remaining long intervals in physical units, retaining slow drift and
flagging repeated large object motion for review instead of calling it zero force.
"""
from bisect import bisect_left
import json
from pathlib import Path
import re

import numpy as np

from .mosaic_policy_package import ROOT,checksum,read,write

SETTINGS='configs/task_families/mosaic/motion-review-r13.json'
CRITICAL_EVENTS={'picked','placed','grip_relocalized','mosaic_review_adopted_hold',
                 'mosaic_review_reconciled_release','reference_placement_repair','trial_end'}


def merge_intervals(intervals):
    result=[]
    for first,last in sorted(intervals):
        if last<first:raise ValueError('Non-monotonic protected interval')
        if result and first<=result[-1][1]:result[-1][1]=max(last,result[-1][1])
        else:result.append([float(first),float(last)])
    return result


def gap_intervals(start,end,protected,minimum):
    cursor=start;gaps=[]
    for first,last in merge_intervals(protected):
        if last<start or first>end:continue
        first=max(start,first);last=min(end,last)
        if first>cursor and first-cursor>=minimum:gaps.append((cursor,first))
        cursor=max(cursor,last)
    if end-cursor>=minimum:gaps.append((cursor,end))
    return gaps


def protected_intervals(actions,events,progress,end,context):
    intervals=[];force_changes={};counts=dict(actions=len(actions),critical_events=0,progress_changes=0)
    for action in actions:
        first=float(action['start_sim_s']);last=float(action['end_sim_s'])
        intervals.append((first-context,last+context))
        for robot,(operation,parameters) in action['commands'].items():
            if operation=='gripper':
                mode=parameters.get('mode')
                # An unrecognized hand mode is conservatively a possible hold.
                feedback=action.get('results',{}).get(robot,{})
                held=not (mode=='move' and feedback.get('status')=='completed')
                force_changes.setdefault(robot,[]).append((first,held))
    for event in events:
        stamp=float(event['sim_time_s']);kind=event['event']
        if kind in CRITICAL_EVENTS:
            intervals.append((stamp-context,stamp+context));counts['critical_events']+=1
        if kind=='mosaic_review_adopted_hold' and event.get('robot'):
            force_changes.setdefault(event['robot'],[]).append((stamp,True))
    for changes in force_changes.values():
        held=False;since=None
        for stamp,value in sorted(changes):
            if value and not held:since=stamp
            if held and not value:intervals.append((since-context,stamp+context));since=None
            held=value
        if held:intervals.append((since-context,end))
    previous=None
    for item in progress:
        if 'sim_time_s' not in item:continue
        state=item['result'];signature=(state.get('matched_goals'),state.get('task_success'),
            json.dumps(state.get('assignments',{}),sort_keys=True),json.dumps(state.get('violations',[]),sort_keys=True))
        stamp=float(item['sim_time_s'])
        if signature!=previous:
            intervals.append((stamp-context,stamp+context));counts['progress_changes']+=1
        if state.get('task_success'):intervals.append((stamp-context,end))
        previous=signature
    return merge_intervals(intervals),counts


def _layout(joints):
    arms=[];fingers=[];objects=[]
    for joint in joints:
        name=joint['id'];address=joint['qpos_address'];kind=joint['type']
        if re.fullmatch(r'[A-H]_joint[1-7]',name):arms.append(address)
        elif re.fullmatch(r'[A-H]_finger_joint[12]',name):fingers.append(address)
        elif kind==0:objects.append(address)
    if not arms or not objects:raise ValueError('Expected Mosaic arms and free cubes')
    return np.array(arms),np.array(fingers,dtype=int),np.array(objects)


def _quaternion_angles(a,b):
    a=a/np.maximum(np.linalg.norm(a,axis=-1,keepdims=True),1e-15)
    b=b/np.maximum(np.linalg.norm(b,axis=-1,keepdims=True),1e-15)
    return 2*np.arccos(np.clip(np.abs(np.sum(a*b,axis=-1)),0,1))


class GapFeatures:
    def __init__(self,joints,angular_noise_rad=1e-6):
        self.arm,self.finger,self.free=_layout(joints);self.rotation_noise=angular_noise_rad
        self.linear=np.r_[self.arm,self.finger,np.ravel(self.free[:,None]+np.arange(3))]
        self.first=None;self.last=None;self.minimum=None;self.maximum=None;self.path=None
        self.first_quat=None;self.last_quat=None;self.rotation_path=None;self.rotation_excursion=None
        self.frames=0

    def add(self,states):
        q=np.asarray(states,dtype=float)
        if not len(q):return
        values=q[:,self.linear];quats=q[:,self.free[:,None]+np.arange(3,7)]
        if not np.isfinite(q).all():raise ValueError('Nonfinite source state')
        if self.first is None:
            self.first=values[0].copy();self.minimum=values.min(0);self.maximum=values.max(0);self.path=np.zeros(values.shape[1])
            self.first_quat=quats[0].copy();self.rotation_path=np.zeros(len(self.free));self.rotation_excursion=np.zeros(len(self.free))
        else:
            values=np.vstack((self.last,values));quats=np.concatenate((self.last_quat[None],quats))
            self.minimum=np.minimum(self.minimum,values.min(0));self.maximum=np.maximum(self.maximum,values.max(0))
        self.path+=np.abs(np.diff(values,axis=0)).sum(0)
        angles=_quaternion_angles(quats[1:],quats[:-1]);angles[angles<self.rotation_noise]=0
        self.rotation_path+=angles.sum(0)
        self.rotation_excursion=np.maximum(self.rotation_excursion,_quaternion_angles(quats,self.first_quat).max(0))
        self.last=values[-1].copy();self.last_quat=quats[-1].copy();self.frames+=len(q)

    def result(self,settings):
        if self.first is None:raise ValueError('Candidate has no source states')
        excursion=self.maximum-self.minimum;net=np.abs(self.last-self.first);arm_end=len(self.arm);finger_end=arm_end+len(self.finger)
        arm_max=float(excursion[:arm_end].max());finger_max=float(excursion[arm_end:finger_end].max(initial=0))
        object_range=np.linalg.norm(excursion[finger_end:].reshape(-1,3),axis=1)
        object_path=np.linalg.norm(self.path[finger_end:].reshape(-1,3),axis=1)
        object_net=np.linalg.norm(net[finger_end:].reshape(-1,3),axis=1)
        angular_noise=settings['angular_noise_rad'];linear_noise=settings['linear_noise_m']
        noise=np.r_[np.full(arm_end,angular_noise),np.full(len(net)-arm_end,linear_noise)]
        ratio=net/np.maximum(self.path,1e-30)
        drift=np.any((net>noise)&(ratio>=settings['monotonic_net_to_path_ratio']))
        rotation_net=_quaternion_angles(self.last_quat,self.first_quat)
        drift|=bool(np.any((rotation_net>angular_noise)&(rotation_net/np.maximum(self.rotation_path,1e-30)>=settings['monotonic_net_to_path_ratio'])))
        repeated_translation=(object_range>settings['object_excursion_m'])&(object_path/np.maximum(object_range,1e-30)>=settings['repeated_path_to_range_ratio'])&(object_net/np.maximum(object_range,1e-30)<=settings['repeated_net_to_range_ratio'])
        repeated_rotation=(self.rotation_excursion>settings['object_rotation_rad'])&(self.rotation_path/np.maximum(self.rotation_excursion,1e-30)>=settings['repeated_path_to_range_ratio'])&(rotation_net/np.maximum(self.rotation_excursion,1e-30)<=settings['repeated_net_to_range_ratio'])
        large=bool(np.any(object_range>settings['object_excursion_m']) or np.any(self.rotation_excursion>settings['object_rotation_rad']))
        thresholds=np.r_[np.full(arm_end,settings['arm_excursion_rad']),np.full(finger_end-arm_end,settings['gripper_excursion_m'])]
        robot_large=excursion[:finger_end]>thresholds
        robot_repeated=robot_large&(self.path[:finger_end]/np.maximum(excursion[:finger_end],1e-30)>=settings['repeated_path_to_range_ratio'])&(net[:finger_end]/np.maximum(excursion[:finger_end],1e-30)<=settings['repeated_net_to_range_ratio'])
        all_robot_motion_repeated=bool(robot_large.any() and np.all(~robot_large|robot_repeated))

        if robot_large.any() and not all_robot_motion_repeated:label='keep_robot_or_gripper_motion'
        elif drift:label='keep_slow_net_motion'
        elif large and (repeated_translation.any() or repeated_rotation.any()):label='review_repeated_large_object_oscillation'
        elif large:label='keep_nonrepeated_object_motion'
        elif all_robot_motion_repeated:label='review_repeated_robot_or_gripper_oscillation'
        else:label='proposed_idle_cut'
        return dict(label=label,source_states=self.frames,arm_excursion_rad=arm_max,
            gripper_excursion_m=finger_max,object_excursion_m=float(object_range.max(initial=0)),
            object_rotation_excursion_rad=float(self.rotation_excursion.max(initial=0)),
            measured_zero_force_proven=False,applied=False,
            reason='No recorded action/commanded force hold/progress transition in this candidate; pose evidence is not measured contact force')


def audit_case(trial,output,settings=None):
    trial=Path(trial);output=Path(output);settings=settings or read(ROOT/SETTINGS)
    from ..motion_recording import MotionReader
    reader=MotionReader(trial/'motion')
    if reader.meta['status']!='complete':raise ValueError('Require finalized source recording')
    if not reader.meta['case_id'].startswith('mosaic-'):raise ValueError('Mosaic recordings only')
    actions=read(trial/'actions.json')
    events=[json.loads(line) for line in (trial/'trace.jsonl').open()]
    progress=[json.loads(line) for line in (trial/'process.jsonl').open()]
    start=reader.meta['chunks'][0]['start_sim_s'];end=reader.meta['chunks'][-1]['end_sim_s']
    protected,counts=protected_intervals(actions,events,progress,end,settings['transition_context_s'])
    gaps=gap_intervals(start,end,protected,settings['minimum_candidate_duration_s'])
    registry=read(reader.path/'source/workstation.json');chunks=reader.meta['chunks'];ends=[c['end_sim_s'] for c in chunks]
    offsets=np.r_[0,np.cumsum([c['frames'] for c in chunks])];results=[];scanned=0
    for first,last in gaps:
        features=GapFeatures(registry['joints'],settings['angular_noise_rad']);first_index=last_index=None
        for index in range(bisect_left(ends,first),len(chunks)):
            chunk=chunks[index]
            if chunk['start_sim_s']>=last:break
            path=reader.path/chunk['file']
            if checksum(path)!=chunk['sha256']:raise ValueError('Source candidate chunk checksum differs')
            with np.load(path,allow_pickle=False) as data:
                times=data['time_s'];selected=(times>=first)&(times<last)
                if not selected.any():continue
                q=np.bitwise_xor(np.bitwise_xor.accumulate(data['xor'],axis=0),data['base']).view('<f4')
                features.add(q[selected]);ids=np.flatnonzero(selected)+offsets[index]
                if first_index is None:first_index=int(ids[0])
                last_index=int(ids[-1]);scanned+=int(selected.sum())
        if first_index is None:continue
        results.append(dict(source_start_s=first,source_end_s_exclusive=last,first_source_frame=first_index,
                            last_source_frame=last_index,duration_s=last-first,**features.result(settings)))
    report=dict(schema='robofrontier.mosaic-motion-review.v1',case_id=reader.meta['case_id'],
        source_scene_sha256=reader.meta['source_scene_sha256'],motion_index_sha256=checksum(reader.path/'stream.json'),
        actions_sha256=checksum(trial/'actions.json'),trace_sha256=checksum(trial/'trace.jsonl'),
        process_sha256=checksum(trial/'process.jsonl'),settings=settings,guards=counts,
        source_frames=reader.meta['frames'],candidate_source_frames_checked=scanned,protected_intervals_s=protected,
        source_data_unchanged=True,edits_applied=False,policy_input=False,results=results,
        scope='Private offline review candidates only. Active execution/commanded holding/progress changes protected. Large oscillation needs task-effect review; no measured-zero-force claim.')
    write(output,report);return report
