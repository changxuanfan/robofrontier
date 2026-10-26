"""Export one verified complete recorded-state video for each of the nine cases."""
import argparse
import concurrent.futures
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import threading


def run(args):
    workspace=Path(__file__).resolve().parents[1]
    summary=json.loads(args.summary.read_text());cases=summary['final_verified_cases']
    if len(cases)!=9 or len({v['case_id'] for v in cases})!=9:raise ValueError('Require all nine unique audited complete cases')
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=True)
    supplemental=output/'补充视角与旧倍速';supplemental.mkdir(exist_ok=True)
    for old in list(output.glob('*.mp4')):
        if re.match(r'^\d{2}-',old.name):continue
        destination=supplemental/old.name
        if destination.exists():raise FileExistsError(destination)
        old.rename(destination)
    artifacts=output/'导出记录';artifacts.mkdir(exist_ok=True)
    env=os.environ.copy();env.update(PYTHONPATH=str(workspace),VECLIB_MAXIMUM_THREADS='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    lock=threading.Lock()
    rows=[]
    for i,case in enumerate(cases,1):
        label=case['label'].replace(' ','');trial=Path(case['report']).parent
        report=json.loads((trial/'report.json').read_text());audit=json.loads((trial/'independent-motion-audit.json').read_text())
        if not report['passed'] or not audit['passed']:raise ValueError('Unqualified source '+str(trial))
        rows.append(dict(case_id=case['case_id'],label=case['label'],source_trial=str(trial),source_frames=audit['frames'],source_scene_sha256=report['scene_sha256'],source_simulation_s=report['evaluation']['sim_time_s'],speedup=60.,file=str(output/f'{i:02d}-{label}-完整-全局-60倍速.mp4'),state='pending'))
    def save():
        document=dict(kind='All nine audited original-task recorded-state videos; no new simulation',updated_at=datetime.datetime.now().astimezone().isoformat(),ready=sum(v['state']=='ready' for v in rows),total=9,workers=args.workers,encoder_threads=1,videos=rows)
        temporary=output/'video-index.json.tmp';temporary.write_text(json.dumps(document,ensure_ascii=False,indent=2)+'\n');temporary.replace(output/'video-index.json')
        lines=['# 九任务完整视频','',f"已就绪：{document['ready']}/9。统一全局视角、60 倍速，包含原记录的完整时间范围及最终状态。",'','| 任务 | 状态 | 视频 |','|---|---|---|']
        for row in rows:lines.append(f"| {row['label']} | {row['state']} | [{Path(row['file']).name}]({row['file']}) |")
        lines+=['','视频从已审计的状态轨迹离线渲染，不执行控制、IK、接触求解或新物理步进。','原有额外视角与旧倍速视频保留在“补充视角与旧倍速”子目录。','']
        (output/'目录说明.md').write_text('\n'.join(lines))
    save()
    def publish(row,manifest,video):
        if manifest['source_frames']!=row['source_frames'] or manifest['source_scene_sha256']!=row['source_scene_sha256'] or manifest['speedup']!=60 or not manifest['views']['global']['decode_passed']:
            raise ValueError('Video/source evidence mismatch '+row['case_id'])
        destination=Path(row['file'])
        if not destination.exists():shutil.copy2(video,destination)
        decoder=subprocess.run(['ffmpeg','-v','error','-threads','1','-i',str(destination),'-threads','1','-f','null','-'],env=env,capture_output=True,text=True)
        if decoder.returncode:raise RuntimeError('Full video decode failed '+decoder.stderr)
        probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','format=duration:stream=codec_type,width,height','-of','json',str(destination)],env=env,text=True))
        stream=next(v for v in probe['streams'] if v['codec_type']=='video');duration=float(probe['format']['duration'])
        if (stream['width'],stream['height'])!=(1280,800) or abs(duration-manifest['rendered_states']/manifest['fps'])>.15:raise ValueError('Unexpected video geometry or duration')
        preview=artifacts/(destination.stem+'-final.png')
        subprocess.run(['ffmpeg','-v','error','-threads','1','-filter_threads','1','-sseof','-0.5','-i',str(destination),'-vf','reverse','-frames:v','1','-threads','1','-y',str(preview)],env=env,check=True)
        result=dict(manifest,file=str(destination),duration_s=duration,video_sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),final_preview=str(preview))
        (artifacts/(destination.stem+'-manifest.json')).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        with lock:
            row.update(state='ready',duration_s=duration,rendered_states=manifest['rendered_states'],decode_passed=True,sha256=result['video_sha256'],final_preview=str(preview));save()
            print('VIDEO_READY',row['label'],sum(v['state']=='ready' for v in rows),'/9',flush=True)
    def export(row):
        try:
            with lock:row['state']='running';save();print('VIDEO_START',row['label'],flush=True)
            trial=Path(row['source_trial']);destination=Path(row['file']);cached=None
            if row['case_id']=='mosaic-a06-9x18-d3':
                folder=output.parent/'six-l3-full-60x'
                if (folder/'manifest.json').exists():cached=folder
            existing=artifacts/(destination.stem+'-manifest.json')
            if destination.exists() and existing.exists():
                m=json.loads(existing.read_text());publish(row,m,destination);return
            if cached is not None:
                manifest=json.loads((cached/'manifest.json').read_text());publish(row,manifest,cached/'global.mp4');return
            index=1
            while (artifacts/f'{row["case_id"]}-attempt-{index:03d}').exists():index+=1
            target=artifacts/f'{row["case_id"]}-attempt-{index:03d}';target.mkdir()
            with lock:row['render_directory']=str(target);row['encoding_progress']=str(target/'global-encoding-progress.txt');save()
            with (target/'render.log').open('w') as log:
                result=subprocess.run([sys.executable,str(workspace/'scripts/render_frontier_oracle.py'),str(trial),'--views','global','--speed','60','--encoder-threads','1','--output',str(target)],env=env,cwd=workspace,stdout=log,stderr=subprocess.STDOUT)
            if result.returncode:raise RuntimeError('Renderer failed: '+str(target/'render.log'))
            publish(row,json.loads((target/'manifest.json').read_text()),target/'global.mp4')
        except Exception as error:
            with lock:row.update(state='error',error=str(error));save();print('VIDEO_ERROR',row['label'],str(error),flush=True)
            raise
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as executor:
        futures=[executor.submit(export,row) for row in rows]
        for future in concurrent.futures.as_completed(futures):future.result()
    if sum(v['state']=='ready' for v in rows)!=9:raise RuntimeError('Missing video exports')
    print('ALL_NINE_VIDEOS_READY',output,flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--summary',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--workers',type=int,default=3);args=parser.parse_args()
    if not 1<=args.workers<=3:parser.error('Use1..3 independent render processes')
    run(args)
