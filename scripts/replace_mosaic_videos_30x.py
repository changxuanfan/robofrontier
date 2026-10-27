"""Replace the nine reviewed 60x clips with source-resampled 30x/30fps clips."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
from zoneinfo import ZoneInfo

import numpy as np

from robofrontier.motion_edit import MotionEdit, resample_timeline
from robofrontier.motion_recording import MotionReader, digest
from scripts.mosaic_motion_mask import frame_intervals
from scripts.mosaic_visual_motion import visual_mask, visible_frame_selection, longest_still_seconds


def load(path):
    return json.loads(Path(path).read_text())


def save(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')


def refine_timeline(data, keep, times):
    """Compose a visual selection with source windows without bridging old cuts."""
    selected = [f for f,k in zip(data['render_frames'],keep) if k]
    windows = data['keep_source_ranges']
    pieces = []
    j = 0
    stride = data['nominal_source_stride']
    for frame in selected:
        start = frame['source_frame']
        while start > windows[j]['last_source_frame']:
            j += 1
        if start < windows[j]['first_source_frame']:
            raise ValueError('Selected state lies in a removed source gap')
        end = min(start+stride-1,windows[j]['last_source_frame'])
        if pieces and start <= pieces[-1][1]+1:
            pieces[-1][1] = max(pieces[-1][1],end)
        else:
            pieces.append([start,end])
    result = dict(data)
    result.update(render_frames=[dict(f,output_frame=i,output_time_s=i/data['fps'])
                                 for i,f in enumerate(selected)],
                  output_frames=len(selected),output_duration_s=len(selected)/data['fps'],
                  keep_source_ranges=[dict(first_source_frame=a,last_source_frame=b,
                                           source_start_s=float(times[a]),source_end_s=float(times[b]))
                                      for a,b in pieces],
                  visual_refinement=dict(overlay_excluded_px=80,analysis_width=640,
                                         analysis_height=360,channel_delta=20,changed_pixels=4),
                  range_semantics='Inclusive source windows at the new stride60 grid, intersected with the previous reviewed windows. Original deleted gaps are never bridged.')
    return result


def run(folder,workers=3):
    folder=Path(folder).resolve();workspace=Path(__file__).resolve().parents[1]
    original=load(folder/'video-index.json')
    if original['ready']!=9 or any(r['speedup']!=60 for r in original['videos']):
        raise ValueError('Require all nine current reviewed 60x files')
    backup=folder/'60倍速备份';backup.mkdir(exist_ok=True)
    for row in original['videos']:
        p=Path(row['file']);target=backup/p.name
        if digest(p)!=row['sha256']:raise ValueError('Current source video checksum differs')
        if not target.exists():os.link(p,target)
        if digest(target)!=row['sha256']:raise ValueError('Backup checksum differs')
    save(backup/'video-index-original.json',original)
    shutil.copy2(folder/'目录说明.md',backup/'目录说明-original.md')
    records=folder/'30倍速导出记录';records.mkdir(exist_ok=True)
    batch=records/datetime.now(ZoneInfo('Asia/Hong_Kong')).strftime('%Y%m%d-%H%M%S');batch.mkdir()
    timeline_stage=batch/'轨迹剪辑时间表';timeline_stage.mkdir()
    active_timelines=folder/'轨迹剪辑时间表'
    previous_timelines=folder/'轨迹剪辑时间表-60倍速备份'
    env=os.environ.copy();env.update(PYTHONPATH=str(workspace),VECLIB_MAXIMUM_THREADS='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    lock=threading.Lock();status={r['case_id']:'pending' for r in original['videos']}
    def progress(case,state,**fields):
        with lock:
            status[case]=state
            save(folder/'30倍速替换进度.json',dict(ready=sum(s=='ready' for s in status.values()),total=9,
                 batch=str(batch),states=status,latest=dict(case_id=case,state=state,**fields)))
            print(state.upper(),case,fields,flush=True)
    def one(row):
        case=row['case_id'];work=batch/case;work.mkdir();reader=MotionReader(Path(row['source_trial'])/'motion')
        old_timeline=load(row['motion_edit_timeline']);MotionEdit(row['motion_edit_timeline'],reader)
        with np.load(Path(row['work_directory'])/'motion-mask.npz') as mask:times=mask['source_time_s']
        edit=resample_timeline(old_timeline,times,30,30)
        old_anchors={f['source_frame'] for f in old_timeline['render_frames']}
        if not old_anchors.issubset({f['source_frame'] for f in edit['render_frames']}):
            raise ValueError('Resampling lost a previously reviewed anchor')
        edit.update(resampled_from_edit_sha256=digest(row['motion_edit_timeline']),
                    source_recording_hint=os.path.relpath(reader.path.resolve(),active_timelines))
        plan=work/'resampled-edit.json';save(plan,edit);MotionEdit(plan,reader)
        render_dir=work/'render';progress(case,'rendering',target_frames=edit['output_frames'],work_directory=str(work))
        with (work/'render.log').open('w') as log:
            result=subprocess.run([sys.executable,'-m','scripts.render_frontier_oracle',row['source_trial'],
                 '--views','global','--speed','30','--encoder-threads','1','--edit-timeline',str(plan),
                 '--output',str(render_dir)],cwd=workspace,env=env,stdout=log,stderr=subprocess.STDOUT)
        if result.returncode:raise RuntimeError('Render failed: '+str(work/'render.log'))
        rendered=load(render_dir/'manifest.json')
        if rendered['rendered_states']!=edit['output_frames'] or rendered['speedup']!=30 or rendered['fps']!=30:
            raise ValueError('Render cadence/count differs from planned source selection')
        progress(case,'checking',rendered_frames=edit['output_frames'])
        active=visual_mask(render_dir/'global.mp4',overlay_height=80,width=640,height=360)
        if len(active)!=edit['output_frames']:raise ValueError('Visual decoder count differs')
        keep=visible_frame_selection(active);final_edit=refine_timeline(edit,keep,times)
        final_plan=timeline_stage/(case+'.json');save(final_plan,final_edit);MotionEdit(final_plan,reader)
        output=work/'final.mp4'
        if keep.all():
            shutil.copy2(render_dir/'global.mp4',output)
        else:
            terms=[f'between(n,{a},{b})' if a!=b else f'eq(n,{a})' for a,b in frame_intervals(keep)]
            script=work/'select-filter.txt';script.write_text("select='"+'+'.join(terms)+"',setpts=N/(30*TB)\n")
            subprocess.run(['ffmpeg','-v','error','-threads','1','-filter_threads','1','-i',str(render_dir/'global.mp4'),
                 '-filter_script:v',str(script),'-an','-c:v','libx264','-threads','1','-preset','fast','-crf','20',
                 '-pix_fmt','yuv420p','-r','30','-fps_mode','cfr','-movflags','+faststart',str(output)],env=env,check=True)
        subprocess.run(['ffmpeg','-v','error','-threads','1','-i',str(output),'-threads','1','-f','null','-'],env=env,check=True)
        probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-count_frames','-show_entries',
             'stream=codec_type,width,height,nb_read_frames,avg_frame_rate:format=duration','-of','json',str(output)],text=True,env=env))
        stream=next(x for x in probe['streams'] if x['codec_type']=='video');duration=float(probe['format']['duration'])
        if int(stream['nb_read_frames'])!=final_edit['output_frames'] or (stream['width'],stream['height'])!=(1280,800):
            raise ValueError('Final video dimensions/frame count differ')
        if abs(duration-final_edit['output_duration_s'])>.05:raise ValueError('Final duration differs')
        check_active=visual_mask(output,overlay_height=80,width=640,height=360)
        still=longest_still_seconds(check_active)
        if still>.5:raise ValueError('Residual static waiting remains: '+str(still))
        preview=work/'exact-final.png'
        subprocess.run(['ffmpeg','-v','error','-threads','1','-filter_threads','1','-sseof','-0.5','-i',str(output),
             '-vf','reverse','-frames:v','1','-threads','1','-y',str(preview)],env=env,check=True)
        final_name=Path(row['file']).name.replace('60倍速','30倍速')
        result=dict(case_id=case,label=row['label'],file=str(folder/final_name),source_trial=row['source_trial'],
             source_frames=row['source_frames'],speedup=30.,fps=30.,state='ready',duration_s=duration,
             kept_frames=final_edit['output_frames'],rendered_states=final_edit['output_frames'],
             pre_visual_refinement_frames=edit['output_frames'],source_stride=60,
             decode_passed=True,sha256=digest(output),final_preview=str(preview),work_directory=str(work),
             motion_edit_timeline=str(active_timelines/final_plan.name),motion_edit_timeline_sha256=digest(final_plan),
             manifest=str(work/'manifest.json'),previous_60x_file=str(backup/Path(row['file']).name),
             previous_60x_duration_s=row['duration_s'],longest_remaining_visually_still_s=still)
        save(work/'manifest.json',dict(result,source_scene_sha256=reader.meta['source_scene_sha256'],
             motion_index_sha256=digest(reader.path/'stream.json'),all_source_chunks_and_selected_timestamps_verified=True,
             all_original_deleted_gaps_preserved=True,initial_and_final_states_preserved=True,interpolation=False,
             physics_restarted=False,render_manifest=str(render_dir/'manifest.json'),visual_detection=final_edit['visual_refinement']))
        progress(case,'ready',duration_s=duration,frames=final_edit['output_frames'],max_still_s=still)
        return result,output,final_plan
    try:
        with ThreadPoolExecutor(max_workers=workers) as pool:results=list(pool.map(one,original['videos']))
    except Exception as error:
        save(batch/'error.json',dict(error=str(error),states=status));raise
    timeline_index=dict(schema='robofrontier.motion-edit-batch.v1',ready=9,total=9,fps=30,nominal_speedup=30,
        original_recordings_unchanged=True,background_independent=True,timelines=[
        dict(case_id=r['case_id'],label=r['label'],timeline=p.name,sha256=digest(p),output_frames=r['kept_frames'],
             source_frames=r['source_frames'],output_duration_s=r['duration_s'],keep_source_ranges=len(load(p)['keep_source_ranges']))
        for r,_,p in results])
    save(timeline_stage/'index.json',timeline_index)
    if previous_timelines.exists():raise FileExistsError(previous_timelines)
    active_timelines.rename(previous_timelines)
    shutil.copytree(timeline_stage,active_timelines)
    backed_up=dict(original,videos=[dict(r,file=str(backup/Path(r['file']).name),
        motion_edit_timeline=str(previous_timelines/Path(r['motion_edit_timeline']).name)) for r in original['videos']],
        motion_edit_timeline_folder=str(previous_timelines))
    save(backup/'video-index.json',backed_up)
    for row,output,_ in results:
        shutil.copy2(output,row['file'])
    for row in original['videos']:
        p=Path(row['file'])
        if digest(p)!=row['sha256']:raise ValueError('Old root file changed during replacement')
        p.unlink() # Immutable linked copy remains in 60倍速备份.
    if len(list(folder.glob('*.mp4')))!=9:raise ValueError('Main folder must contain exactly nine videos')
    final=dict(original,variant='visible_idle_removed',ready=9,total=9,status='complete',speedup=30,fps=30,
         videos=[r for r,_,_ in results],updated_at=datetime.now(ZoneInfo('Asia/Hong_Kong')).isoformat(),
         previous_60x_backup=str(backup),motion_edit_timeline_folder=str(active_timelines),
         visual_review='pending',all_exact_final_frames_visually_reviewed=False,verification=str(batch/'verification.json'))
    save(folder/'video-index.json',final)
    save(batch/'verification.json',dict(passed=True,files=9,fps=30,speedup=30,visual_review='pending',
         original_recordings_unchanged=True,all_original_deleted_gaps_preserved=True,cases=[r for r,_,_ in results]))
    progress('all-nine','complete',folder=str(folder));print('ALL_NINE_30X_REPLACED',batch,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--folder',type=Path,required=True)
    p.add_argument('--workers',type=int,default=3,choices=range(1,4));a=p.parse_args();run(a.folder,a.workers)
