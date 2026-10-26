"""Remove globally stationary replay frames using the original full-scene states."""
import argparse
import concurrent.futures
import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import threading
from scripts.mosaic_motion_mask import analyze,ALGORITHM_VERSION


def run(args):
    root=args.folder.resolve();backup=root/'完整时间线原版';backup.mkdir(exist_ok=True)
    original=backup/'video-index.json'
    if not original.exists():
        previous=json.loads((root/'video-index.json').read_text())
        if previous.get('ready')!=9 or previous.get('variant')=='idle_removed':raise ValueError('Require the nine complete original videos')
        original.write_text(json.dumps(previous,ensure_ascii=False,indent=2)+'\n')
        if (root/'目录说明.md').exists():shutil.copy2(root/'目录说明.md',backup/'目录说明.md')
    previous=json.loads(original.read_text());sources=previous['videos'];assert len(sources)==9 and len({v['case_id'] for v in sources})==9
    for row in sources:
        old=Path(row['file']);saved=backup/old.name
        if not saved.exists():
            if not old.exists():raise FileNotFoundError(old)
            old.rename(saved)
        if hashlib.sha256(saved.read_bytes()).hexdigest()!=row['sha256']:raise ValueError('Original video checksum changed')
    backup_notes=backup/'目录说明.md'
    note_text=backup_notes.read_text() if backup_notes.exists() else ''
    for row in sources:
        old_file=row['file'];row.setdefault('original_export_file',old_file);row['file']=str(backup/Path(old_file).name)
        note_text=note_text.replace(old_file,row['file'])
    original.write_text(json.dumps(previous,ensure_ascii=False,indent=2)+'\n')
    if note_text:backup_notes.write_text(note_text)
    records=root/'静止裁剪记录';records.mkdir(exist_ok=True)
    rows=[dict(case_id=v['case_id'],label=v['label'],source_trial=v['source_trial'],source_frames=v['source_frames'],source_video=str(backup/Path(v['file']).name),source_video_sha256=v['sha256'],original_duration_s=v['duration_s'],speedup=v['speedup'],file=str(root/Path(v['file']).name.replace('-完整-','-去静止-')),state='pending') for v in sources]
    env=os.environ.copy();env.update(VECLIB_MAXIMUM_THREADS='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1');lock=threading.Lock()
    def save():
        data=dict(kind='Nine whole-scene motion-only edits of audited recorded-state videos',variant='idle_removed',ready=sum(v['state']=='ready' for v in rows),total=9,status='complete' if all(v['state']=='ready' for v in rows) else 'processing',updated_at=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),workers=args.workers,source_backup=str(backup),videos=rows)
        temp=root/'video-index.json.tmp';temp.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n');temp.replace(root/'video-index.json')
        lines=['# 九任务去静止视频','',f"已就绪：{data['ready']}/9。保持原版60倍速，去掉整场机械臂、夹爪、物体均静止的等待段。动作前后保留极短衔接帧，初始与最终状态保留。",'','| 任务 | 状态 | 去静止视频 |','|---|---|---|']
        for v in rows:lines.append(f"| {v['label']} | {v['state']} | [{Path(v['file']).name}]({v['file']}) |")
        lines+=['','基于完整原始状态轨迹检测，覆盖关节、夹爪和所有自由物体的位置与旋转；抑制微小数值抖动。','原始完整时间线视频保留在“完整时间线原版”子目录。裁剪映射、阈值和检查结果保存在“静止裁剪记录”。','']
        (root/'目录说明.md').write_text('\n'.join(lines))
    save()
    def export(pair):
        row,source=pair
        try:
            index=1
            while (records/f'{row["case_id"]}-attempt-{index:03d}').exists():index+=1
            work=records/f'{row["case_id"]}-attempt-{index:03d}';work.mkdir()
            with lock:row.update(state='analyzing',work_directory=str(work));save();print('MOTION_ANALYSIS_START',row['label'],flush=True)
            manifest_path=root/'导出记录'/(Path(source['file']).stem+'-manifest.json');manifest=json.loads(manifest_path.read_text())
            def progress(done,total):
                with lock:row.update(analysis_frames=done,analysis_total=total);save()
            analysis=None
            for cached in sorted(records.glob(row['case_id']+'-attempt-*/motion-analysis.json'),key=lambda p:p.stat().st_mtime,reverse=True):
                candidate=json.loads(cached.read_text())
                if (candidate.get('algorithm_version',1)!=ALGORITHM_VERSION or candidate['source_frames']!=manifest['source_frames'] or candidate['source_scene_sha256']!=manifest['source_scene_sha256'] or candidate['fps']!=manifest['fps'] or candidate['speedup']!=manifest['speedup']):continue
                if not all((cached.parent/name).exists() for name in ('motion-mask.npz','select-filter.txt')):continue
                for name in ('motion-analysis.json','motion-mask.npz','select-filter.txt'):shutil.copy2(cached.parent/name,work/name)
                analysis=candidate;progress(candidate['source_frames'],candidate['source_frames']);print('MOTION_ANALYSIS_REUSED',row['label'],flush=True);break
            if analysis is None:analysis=analyze(row['source_trial'],manifest,work,progress)
            output=work/'motion-only.mp4';encoder_progress=work/'encoding-progress.txt'
            with lock:row.update(state='encoding',kept_frames=analysis['kept_video_frames'],removed_fraction=analysis['removed_video_fraction'],encoding_progress=str(encoder_progress));save();print('MOTION_ANALYSIS_DONE',row['label'],'remove',round(100*analysis['removed_video_fraction'],1),'%',flush=True)
            command=['ffmpeg','-hide_banner','-v','error','-threads','1','-filter_threads','1','-filter_complex_threads','1','-i',row['source_video'],'-filter_script:v',str(work/'select-filter.txt'),'-an','-c:v','libx264','-threads','1','-preset','fast','-crf','23','-pix_fmt','yuv420p','-r',str(manifest['fps']),'-fps_mode','cfr','-movflags','+faststart','-metadata','comment=Whole-scene stationary intervals removed using original recorded states; source timestamps retained','-stats_period','2','-progress',str(encoder_progress),str(output)]
            with (work/'encode.log').open('w') as log:result=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT)
            if result.returncode:raise RuntimeError('Encoding failed: '+str(work/'encode.log'))
            check=subprocess.run(['ffmpeg','-v','error','-threads','1','-i',str(output),'-threads','1','-f','null','-'],env=env,capture_output=True,text=True)
            if check.returncode:raise RuntimeError('Full decode failed: '+check.stderr)
            probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-threads','1','-count_frames','-show_entries','stream=codec_type,width,height,nb_read_frames:format=duration','-of','json',str(output)],env=env,text=True));video=next(v for v in probe['streams'] if v['codec_type']=='video');duration=float(probe['format']['duration'])
            if int(video['nb_read_frames'])!=analysis['kept_video_frames'] or (video['width'],video['height'])!=(1280,800) or abs(duration-analysis['kept_video_frames']/manifest['fps'])>.15:raise ValueError('Output frame count/size/duration mismatch')
            destination=Path(row['file']);assert not destination.exists();shutil.copy2(output,destination)
            preview=work/'exact-final.png';subprocess.run(['ffmpeg','-v','error','-threads','1','-filter_threads','1','-sseof','-0.5','-i',str(destination),'-vf','reverse','-frames:v','1','-threads','1','-y',str(preview)],env=env,check=True)
            evidence=dict(analysis,source_video=row['source_video'],source_video_sha256=row['source_video_sha256'],file=str(destination),duration_s=duration,original_duration_s=row['original_duration_s'],decode_passed=True,encoded_frame_count=int(video['nb_read_frames']),video_sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),final_preview=str(preview),physics_restarted=False)
            (work/'manifest.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+'\n')
            with lock:
                row.update(state='ready',duration_s=duration,removed_seconds=row['original_duration_s']-duration,decode_passed=True,sha256=evidence['video_sha256'],manifest=str(work/'manifest.json'),final_preview=str(preview));save();print('MOTION_ONLY_READY',row['label'],sum(v['state']=='ready' for v in rows),'/9',flush=True)
        except Exception as error:
            with lock:row.update(state='error',error=str(error));save();print('MOTION_ONLY_ERROR',row['label'],str(error),flush=True)
            raise
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as executor:
        futures=[executor.submit(export,pair) for pair in zip(rows,sources)]
        for future in concurrent.futures.as_completed(futures):future.result()
    print('ALL_NINE_MOTION_ONLY_READY',root,flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--folder',type=Path,required=True);parser.add_argument('--workers',type=int,default=3);args=parser.parse_args()
    if not 1<=args.workers<=3:parser.error('Use1..3 independent video tasks')
    run(args)
