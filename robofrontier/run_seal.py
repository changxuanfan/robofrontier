"""Engineering-only shutdown/owner-loss sealing. Never creates a physical world."""
import fcntl
import json
import math
from pathlib import Path
import shutil
import time
import uuid

from .pipeline_client import client_call
from .recording import probe_video
from .run_setup import write_json
from .scene_task import camera_names


def _unsettled(store):
    values={i for i,a in store.actions().items() if a['status'] in ('pending','running','uncertain')}
    values.update(r['id'] for r in store.db.execute(
        "SELECT id FROM calls WHERE status='accepted' AND op IN ('move','gripper','action_batch')"))
    return sorted(values)


def seal_run(store, timeout_s=30):
    timeout=float(timeout_s)
    if not math.isfinite(timeout) or not 0<=timeout<=120:raise ValueError('Finite seal timeout between 0 and120 seconds')
    if not store.get('execution_owner_attached'):raise ValueError('No execution owner was attached; seal never starts one')
    run=store.path.resolve().parents[2];output=run/'private/outputs';deadline=time.monotonic()+timeout
    task=json.loads((run/'private/inputs/task.json').read_text())
    with store.transaction():
        if not store.get('dispatch_paused',False):
            store.put('dispatch_paused',True);store.event('DISPATCH_PAUSED','Meta',{'reason':'Engineering seal requested'})
    lock=(run/'private/state/world.lock').open('a+')
    observation=None;capture_error=None;shutdown_error=None;live=False
    try:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:live=True
        if live:
            while _unsettled(store):
                if time.monotonic()>=deadline:
                    raise ValueError('Physical execution unresolved; run remains paused and owner is not closed')
                time.sleep(.02)
            cameras=[c for c in ('global','board','board_top') if c in camera_names(task)]
            connection=run/'private/state/connections'/f"{store.get('config')['manager']}.json"
            identity='seal-'+uuid.uuid4().hex
            try:
                result=client_call(connection,'observe',{'cameras':cameras},identity+'-view',
                    timeout=max(.1,deadline-time.monotonic()),log_call_id=False)
                observation=result.get('observation');capture_error=result.get('error')
                if observation:
                    for name,path in observation.get('images',{}).items():
                        source=Path(path).resolve()
                        if source.is_relative_to(run/'public/observations') and source.is_file():
                            if source.suffix.lower() not in ('.png', '.jpg', '.jpeg'):
                                raise ValueError('Unsupported observation image format')
                            target=output/('final-'+name+source.suffix.lower());shutil.copy2(source,target)
                            observation['images'][name]=str(target)
            except (OSError,ValueError) as error:capture_error=str(error)
            # This is one shutdown request, never a physical-command retry.
            try:
                result=client_call(connection,'shutdown',{},identity+'-shutdown',
                    timeout=max(.1,deadline-time.monotonic()),log_call_id=False)
                if result.get('error'):shutdown_error=result['error']
            except (OSError,ValueError) as error:shutdown_error=str(error)
            while True:
                try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);break
                except BlockingIOError:
                    if time.monotonic()>=deadline:
                        raise ValueError('Execution owner did not exit; run remains paused. '+(shutdown_error or ''))
                    time.sleep(.02)
        # The exclusive OS lock proves there is no physical owner to replay.
        report_path=output/'seal.json'
        if store.get('lifecycle') in ('ended','interrupted') and report_path.exists():
            saved=json.loads(report_path.read_text())
            return {k:saved[k] for k in ('sealed','lifecycle','fresh_observation','report_ref','unresolved_action_ids','video')}
        with store.transaction():
            if store.get('lifecycle')!='ended':
                previous=store.get('lifecycle');store.recover_execution()
                store.put('lifecycle','interrupted');store.put('physics_state','stopped')
                if previous!='interrupted':
                    store.event('RUN_INTERRUPTED','Meta',{'reason':'Execution owner absent at engineering seal; no world reconstruction or physical replay'})
        video_path=output/(task.get('recording_camera','global')+'-realtime.mp4')
        video={'path':str(video_path),**probe_video(video_path)}
        result={'sealed':True,'lifecycle':store.get('lifecycle'),'fresh_observation':observation is not None,
            'report_ref':str(report_path),'unresolved_action_ids':_unsettled(store),'video':video,
            'final_observation':observation,'capture_error':capture_error,'shutdown_error':shutdown_error,
            'owner_was_live':live,'scope':'Engineering closure only; not a successful task or fresh offline evaluation.'}
        write_json(report_path,result)
        store.event('RUN_SEALED','Meta',{k:result[k] for k in ('report_ref','fresh_observation','unresolved_action_ids','video')})
        store.export_events(output/'events.jsonl')
        return {k:result[k] for k in ('sealed','lifecycle','fresh_observation','report_ref','unresolved_action_ids','video')}
    except Exception as error:
        store.event('RUN_SEAL_INCOMPLETE','Meta',{'reason':str(error)[:1000],'note':'No automatic motion replay or forced owner termination'})
        raise
    finally:
        fcntl.flock(lock,fcntl.LOCK_UN);lock.close()
