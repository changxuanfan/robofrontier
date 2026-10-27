"""Fresh native execution of immutable, completed Mosaic command prefixes.

The source may still be running. Only complete recorded scopes covered by
finalized motion chunks are candidates. They never constitute a full source
qualification. Every target goal requires new native dynamics and evaluation.
"""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import shutil
import time

from ..motion_recording import MotionReader
from ..run_setup import write_json
from ..task_config import physical_control_fingerprint
from .mosaic_concurrency import build_plan
from .mosaic_equivalence import check
from .provenance import environment_logic_manifest


MARKERS=('goal_attempt','mosaic_resume_from_actual_progress')


def review_deliberation(plan,events):
    """Schedule fresh candidates without copying long human thinking pauses.

    Native actions and ordinary explicit settling remain intact. Within a
    resolved source review, replace only long idle gaps with 2.4s of settling.
    Source timing/envelopes remain archived, and new dynamics must still pass.
    """
    intervals=[];opened={}
    for e in events:
        if e['event']=='mosaic_review_opened':opened[e['review_id']]=e['sim_time_s']
        elif e['event']=='mosaic_review_resumed':
            start=opened.pop(e['review_id'],None)
            if start is None:raise ValueError('Resolved source review has no recorded opening')
            intervals.append((start,e['sim_time_s']))
    saved=0.;changes=[]
    for i,job in enumerate(plan['jobs']):
        start=job['actions'][0]['end_sim_s'];end=job['end_sim_s'];gap=max(0.,end-start)
        reviewed=sum(max(0.,min(end,b)-max(start,a)) for a,b in intervals)
        if reviewed<=5.:continue
        delay=gap-reviewed+2.4;job['wait_after_s']=max(0.,delay);saved+=gap-delay
        changes.append(dict(node=i,source_gap_s=gap,target_settle_s=delay))
    plan['review_deliberation_schedule']=dict(saved_source_idle_s=saved,changes=changes,
        scope='Fresh candidate scheduling only; native command protection and whole-task timing policy unchanged')
    return plan


def completed_prefix(events,available_until,after_sim_s=-1.,max_new_goals=24,previous_pairs=None):
    """Choose an immutable completed boundary; ignore an in-flight tail."""
    pairs={};chosen=None;old=previous_pairs or {}
    for index,e in enumerate(events):
        if e['sim_time_s']>available_until+1e-8:break
        if e['event']=='goal_attempt':pairs[e['goal']]=e['object']
        elif e['event']=='mosaic_resume_from_actual_progress':pairs.update(e['protected_assignments'])
        else:continue
        if any(pairs.get(g)!=name for g,name in old.items()):
            # Before the previous cut, its later goals need not exist yet.
            if e['sim_time_s']>after_sim_s+1e-8:raise ValueError('Source prefix lost an already consumed pairing')
        if e['sim_time_s']<=after_sim_s+1e-8:continue
        if len(set(pairs)-set(old))>max_new_goals:break
        chosen=(index,e['sim_time_s'],dict(pairs))
    if chosen is None:return None
    index,cut,pairs=chosen;selected=events[:index+1]
    actions=[];pending=None
    for e in selected:
        if e['event']=='action_start':
            if pending is not None:raise ValueError('Prefix source contains overlapping action batches')
            pending=dict(label=e['label'],commands=e['commands'],start_sim_s=e['sim_time_s'])
        elif e['event']=='action_end':
            if pending is None or pending['label']!=e['label']:raise ValueError('Prefix action acknowledgement does not match its original start')
            if any(r['status']!='completed' for r in e['results'].values()):
                raise ValueError('Failed or uncertain source actions need explicit owner review')
            pending.update(results=e['results'],end_sim_s=e['sim_time_s']);actions.append(pending);pending=None
    if pending is not None:raise ValueError('Completed source marker contains unresolved execution')
    return dict(cut=cut,assignments=pairs,actions=actions,events=selected)


def snapshot(source,destination,*,after_sim_s=-1.,previous_pairs=None,max_new_goals=24):
    """Capture source evidence only, with no physics step or state restoration."""
    source=Path(source).resolve();destination=Path(destination)
    status=json.loads((source/'runner-status.json').read_text())
    if status['evaluation']['violations']:raise ValueError('Source has retained physical violations')
    reader=MotionReader(source/'motion')
    if not reader.meta['chunks']:return None
    raw=(source/'trace.jsonl').read_bytes();lines=raw.splitlines(keepends=True)
    events=[json.loads(line) for line in lines if line.endswith(b'\n')]
    prefix=completed_prefix(events,reader.meta['chunks'][-1]['end_sim_s'],after_sim_s,max_new_goals,previous_pairs)
    if prefix is None:return None
    destination.mkdir(parents=True,exist_ok=False)
    for name in ('scene.xml','task.json','environment.json','supply-plan.json'):
        shutil.copyfile(source/name,destination/name)
    os.symlink(source/'assets',destination/'assets',target_is_directory=True)
    motion=destination/'motion';motion.mkdir();os.symlink(source/'motion/source',motion/'source',target_is_directory=True)
    meta=deepcopy(reader.meta)
    # Each XOR chunk contains its own preceding full base, so a later interval
    # is independently decodable without copying earlier motion states.
    meta['chunks']=[c for c in meta['chunks'] if c['end_sim_s']>=after_sim_s and c['start_sim_s']<=prefix['cut']]
    for chunk in meta['chunks']:os.link(source/'motion'/chunk['file'],motion/chunk['file'])
    meta.update(status='prefix_candidate',frames=sum(c['frames'] for c in meta['chunks']))
    write_json(motion/'stream.json',meta)
    (destination/'trace.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in prefix['events']))
    write_json(destination/'actions.json',prefix['actions'])
    task=json.loads((source/'task.json').read_text());cfg=json.loads((source/'executor-source/effective-harness.json').read_text())
    hashes={str(p.relative_to(source/'executor-source')):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (source/'executor-source').rglob('*.py')}
    report=dict(case_id=task['id'],passed=False,source_prefix_only=True,source_prefix_cut_sim_s=prefix['cut'],
        evaluation=dict(assignments=prefix['assignments'],total_goals=len(prefix['assignments']),violations=[]),
        physical_control_sha256=physical_control_fingerprint(hashes,cfg),
        environment_logic_sha256=environment_logic_manifest(hashes,task)['sha256'],
        scene_bundle_sha256=json.loads((source/'source-assets.json').read_text())['sha256'],
        assisted=any(e['event']=='mosaic_review_request' and e.get('assisted') for e in prefix['events']),
        scope='Incomplete-source command candidate only; never source or target task qualification')
    write_json(destination/'report.json',report)
    manifest=dict(source=str(source),world_id=status['world_id'],case_id=task['id'],
        source_executor_manifest_sha256=hashlib.sha256((source/'executor-launch.json').read_bytes()).hexdigest(),
        captured_trace_sha256=hashlib.sha256(raw).hexdigest(),selected_trace_sha256=hashlib.sha256((destination/'trace.jsonl').read_bytes()).hexdigest(),
        cut_sim_s=prefix['cut'],after_sim_s=after_sim_s,source_assignments=prefix['assignments'],
        source_phase_at_capture=status['phase'],full_source_passed=False,full_target_passed=False,
        scope=report['scope'])
    write_json(destination/'prefix-manifest.json',manifest)
    return manifest


def execute(trial,source,equivalence,*,max_goals=None,prefix_goals=24,finish_after_source=False):
    if not equivalence['equivalent_for_command_candidate']:raise ValueError('Streaming requires exact physical input equivalence')
    source=Path(source).resolve();identity=json.loads((source/'runner-status.json').read_text())['world_id']
    original_manifest=hashlib.sha256((source/'executor-launch.json').read_bytes()).hexdigest()
    target_bundle=trial.source_bundle['sha256'];target_count=max_goals or len(trial.task['goals'])
    after=-1.;pairs={};chunk=0;peak=0;first=True
    trial.mosaic_stream_summary=dict(source=str(source),prefixes=[],source_assisted=False,
        scope='New actuator dynamics from completed source prefixes; source may be incomplete; target evaluation remains independent')
    write_json(trial.directory/'physical-equivalence.json',equivalence)
    from .mosaic_async import execute_plan
    while len(trial.monitor.result().get('assignments',{}))<target_count:
        trial.check_stop();status=json.loads((source/'runner-status.json').read_text())
        if status['world_id']!=identity or hashlib.sha256((source/'executor-launch.json').read_bytes()).hexdigest()!=original_manifest:
            raise ValueError('Streaming source physical owner or frozen executor changed')
        path=trial.directory/'source-prefixes'/f'prefix-{chunk:04d}'
        manifest=snapshot(source,path,after_sim_s=after,previous_pairs=pairs,
            max_new_goals=min(prefix_goals,target_count-len(pairs)))
        if manifest is None:
            if status['phase'] not in ('running','review'):
                if finish_after_source:
                    report=json.loads((source/'report.json').read_text())
                    if (report.get('failed_robot_actions') or report.get('unresolved_robot_actions') or
                            report['evaluation']['violations']):raise RuntimeError('Closed source native feedback needs owner review')
                    from .mosaic_run import require_known_idle
                    require_known_idle(trial)
                    if trial.held or any(a.grasp_force is not None for a in trial.arms.values()):
                        raise RuntimeError('Source handoff requires physically released arms')
                    trial.mosaic_stream_summary['source_terminal']=dict(phase=status['phase'],error=report.get('error'),
                        source_passed=report.get('passed'),target_progress_at_handoff=len(trial.monitor.result().get('assignments',{})))
                    write_json(trial.directory/'streaming-summary.json',trial.mosaic_stream_summary)
                    trial.event('mosaic_stream_source_exhausted',source_phase=status['phase'],source_error=report.get('error'),
                        scope='Retained source commands consumed; continue new target world from actual progress, never copy source outcome')
                    from .mosaic_oracle_v2 import solve
                    return solve(trial,resume=not first,motion_profile=trial.mosaic_motion_profile)
                raise RuntimeError('Source ended before a new usable complete prefix: '+status['phase'])
            # Waiting is actual forward simulation. Previously accepted goals
            # remain protected; no held payload or active action is abandoned.
            if trial.held or any(a.job is not None or a.grasp_force is not None for a in trial.arms.values()):
                raise RuntimeError('Source wait requires actual released, known idle arms')
            trial.wait(.2)
            current=trial.monitor.result().get('assignments',{})
            if any(current.get(g)!=name for g,name in getattr(trial,'mosaic_protected_assignments',{}).items()):
                raise RuntimeError('Streaming wait disturbed a completed pairing')
            time.sleep(.2);continue
        plan=build_plan(path,candidate_prefix=True,after_sim_s=None if first else after)
        review_deliberation(plan,[json.loads(line) for line in (path/'trace.jsonl').read_text().splitlines()])
        plan['target_scene_bundle_sha256']=target_bundle
        plan['operation_prefix']=f'mosaic-stream-{chunk:04d}'
        write_json(path/'concurrency-plan.json',plan)
        trial.event('mosaic_stream_prefix_admitted',manifest=str(path/'prefix-manifest.json'),
            source_cut_sim_s=manifest['cut_sim_s'],new_candidate_goals=plan['total_goals'],source_prefix_only=True)
        result=execute_plan(trial,plan,initialize=first)
        peak=max(peak,result['concurrent_operation_peak']);first=False;after=manifest['cut_sim_s'];pairs=manifest['source_assignments'];chunk+=1
        trial.mosaic_stream_summary['prefixes'].append(dict(path=str(path),source_cut_sim_s=after,candidate_goals=len(pairs),
            target_actual_goals=len(trial.monitor.result().get('assignments',{})),peak=result['concurrent_operation_peak'],
            review_deliberation_saved_s=plan['review_deliberation_schedule']['saved_source_idle_s']))
        trial.mosaic_stream_summary['source_assisted']|=plan['source_assisted']
        write_json(trial.directory/'streaming-summary.json',trial.mosaic_stream_summary)
    return dict(reference_placed=len(trial.monitor.result().get('assignments',{})),total_goals=len(trial.task['goals']),
        commissioning_only=max_goals is not None,streamed_prefixes=chunk,concurrent_operation_peak=peak,
        strategy='Independent native execution of source prefix dependencies; not a global optimum')
