"""Collect only the 36-case suite; re-render saved states, never step physics."""
import argparse
import concurrent.futures
import html
import json
import math
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'docs/generated/frontier-suite/oracle-36-only-20261024'


def render(source,target):
    import mujoco
    import numpy as np
    from PIL import Image,ImageDraw
    from robofrontier.cube_transport import _font
    from robofrontier.suite.evaluate import DIGITS
    from scripts.export_frontier_contact_sheets import fit_camera
    source=Path(source);target=Path(target)
    states=np.load(source/'frame-states.npz');report=json.loads((source/'report.json').read_text())
    n=len(states['time_s'])
    if not n:raise ValueError('No recorded frames')
    every=max(1,math.ceil(n/600));indices=list(range(0,n,every))
    if indices[-1]!=n-1:indices.append(n-1)
    fps=15;interval=float(np.median(np.diff(states['time_s']))) if n>1 else 1.
    speed=interval*every*fps
    m=mujoco.MjModel.from_xml_path(str(source/'scene.xml'));d=mujoco.MjData(m)
    d.qpos[:]=states['qpos'][0];mujoco.mj_forward(m,d)
    for view in ('top','global','global_rear'):fit_camera(m,d,view)
    processes={};target.mkdir(parents=True,exist_ok=True)
    status='PASS' if report.get('passed') else 'PAUSED / PARTIAL' if report.get('interrupted') else 'FAILED / PARTIAL'
    try:
        for view in ('global','global_rear'):
            processes[view]=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s','1280x800','-r',str(fps),'-i','-','-an','-c:v','libx264','-threads','1','-crf','21','-preset','veryfast','-pix_fmt','yuv420p','-movflags','+faststart',str(target/(view+'.mp4'))],stdin=subprocess.PIPE)
        with mujoco.Renderer(m,width=1280,height=800) as renderer:
            for i in indices:
                d.qpos[:]=states['qpos'][i];d.time=float(states['time_s'][i]);mujoco.mj_forward(m,d)
                if mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'score_0_0')>=0:
                    value=int(states['display'][i]);show=f'{value:02}' if value>=0 else '--'
                    for digit,char in enumerate(show):
                        for k,segment in enumerate('abcdefg'):
                            m.geom_rgba[m.geom(f'score_{digit}_{k}').id]=[.68,.13,.07,1 if segment in (DIGITS[int(char)] if char!='-' else 'g') else 0]
                for view,process in processes.items():
                    renderer.update_scene(d,camera=view);image=Image.fromarray(renderer.render());draw=ImageDraw.Draw(image)
                    draw.rectangle((8,8,1271,78),fill=(245,242,235))
                    draw.text((20,12),f'{report["case_id"]} | {status} | Oracle saved-state overview',font=_font(21),fill=(23,32,43))
                    draw.text((20,43),f'{view} | sim {d.time:.1f} s | approx {speed:.1f}x | {source.parent.name}/{source.name}',font=_font(20),fill=(23,32,43))
                    process.stdin.write(image.tobytes())
    finally:
        for p in processes.values():
            if p.stdin:p.stdin.close()
        codes=[p.wait() for p in processes.values()]
    if any(codes):raise RuntimeError('Encoding failed '+str(codes))
    for view in processes:
        check=subprocess.run(['ffmpeg','-v','error','-threads','1','-i',str(target/(view+'.mp4')),'-f','null','-'],capture_output=True,text=True)
        if check.returncode or check.stderr.strip():raise RuntimeError('Decode failed '+check.stderr[:400])
    (target/'video-manifest.json').write_text(json.dumps(dict(source=str(source.relative_to(ROOT)),kind='Time-compressed saved-state overview; no new physics',source_frames=n,rendered_frames=len(indices),source_interval_s=interval,approximately_speedup=speed,video_fps=fps,views=['global','global_rear'],decode_passed=True),indent=2))


def collect():
    from robofrontier.suite.scenes import catalog
    OUT.mkdir(parents=True,exist_ok=True)
    cases=catalog()['cases'];records=[]
    for case in cases:
        raw=ROOT/'runs/oracle/frontier-v1'/case['id'];target=OUT/case['id'];target.mkdir(exist_ok=True)
        (target/'current-case.json').write_text(json.dumps(case,ensure_ascii=False,indent=2))
        link=target/'all-original-run-data'
        if raw.exists() and not link.exists():link.symlink_to(raw,target_is_directory=True)
        reports=[];candidates=[]
        for p in sorted(raw.glob('seed-*/attempt-*/report.json')):
            try:d=json.loads(p.read_text())
            except ValueError:continue
            archive=target/'all-reports'/p.relative_to(raw);archive.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,archive)
            reports.append(str(p.relative_to(ROOT)))
            if (p.parent/'frame-states.npz').exists():
                candidates.append((d.get('scene_sha256')==case.get('scene_sha256'),bool(d.get('passed')),d.get('finished_at_unix_s',p.stat().st_mtime) or 0,p,d))
        row=dict(case_id=case['id'],family=case['family'],report_count=len(reports),all_reports=reports,video_ready=False)
        if candidates:
            current,passed,_,p,d=max(candidates,key=lambda x:x[:3])
            row.update(source=str(p.parent.relative_to(ROOT)),current_scene_matches=current,reported_passed=passed,interrupted=bool(d.get('interrupted')),status='passed' if passed else 'paused' if d.get('interrupted') else 'failed')
            shutil.copy2(p,target/'selected-report.json')
            if (p.parent/'trace.jsonl').exists():shutil.copy2(p.parent/'trace.jsonl',target/'selected-trace.jsonl')
            note=f'Selected source: {row["source"]}\nSelection: current XML first, then successful trial, then most recent.\nCurrent scene XML matches: {current}.\nReport passed: {passed}. Interrupted: {row["interrupted"]}.\nVideos are time-compressed overviews of actual saved states, not a new physical run.\nAll attempts and original raw data remain under all-original-run-data.\n'
        else:
            row.update(status='not_run',current_scene_matches=None,reported_passed=None)
            note='No completed or interrupted Oracle report with saved trajectory exists for this task. No Oracle video has been fabricated.\n'
        (target/'README.txt').write_text(note)
        records.append(row)
    return records


def page(records):
    groups=[]
    labels={'transport':'物品传递','relay':'空中接力','mosaic':'火箭拼图','dial':'旋钮 / 手摇柄'}
    for family,label in labels.items():
        cards=[]
        for r in [r for r in records if r['family']==family]:
            cid=r['case_id'];state={'passed':'所选记录通过','paused':'用户暂停，未完成','failed':'失败 / 部分过程','not_run':'尚无 Oracle 轨迹'}[r['status']]
            warning=' · 较早场景版本' if r.get('current_scene_matches') is False else ''
            videos=''.join(f'<div><b>{label}</b><video controls preload="none" src="{cid}/{v}.mp4"></video><a href="{cid}/{v}.mp4" target="_blank">单独播放 / 下载</a></div>' for v,label in [('global','正面'),('global_rear','背面')]) if r.get('video_ready') else '<p>暂无可播放视频</p>'
            cards.append(f'<article><h3>{cid}</h3><p>{state}{warning} · {r["report_count"]} 份历史报告</p><div class="views">{videos}</div><p><a href="{cid}/README.txt">来源说明</a> · <a href="{cid}/all-original-run-data/">本任务全部原始运行文件</a></p></article>')
        groups.append(f'<section><h2>{label}</h2>'+''.join(cards)+'</section>')
    ready=sum(r.get('video_ready',False) for r in records)
    (OUT/'manifest.json').write_text(json.dumps(records,ensure_ascii=False,indent=2))
    (OUT/'index.html').write_text(f'<!doctype html><meta charset="utf-8"><title>仅本轮36任务 · Oracle</title><style>body{{font:17px system-ui;background:#f1f5f7;color:#182631;max-width:1500px;margin:32px auto;padding:0 24px}}article{{background:white;border-radius:12px;margin:20px 0;padding:20px}}.views{{display:grid;grid-template-columns:1fr 1fr;gap:16px}}video{{width:100%}}a{{color:#175d94}}p{{line-height:1.6}}</style><h1>仅本轮 36 个任务 · Oracle 文件夹</h1><p>不含之前其他实验。目前 {ready} 个任务已导出正反视频；未运行、失败、暂停和较早场景版本均单独标明。视频是带时间和倍速标注的快速总览，使用实际保存的状态，未启动新物理实验。每个任务保留所有历史报告，并链接本轮全部原始运行文件。</p><p><a href="frontier-v1-oracle-videos-and-reports.zip">下载视频与报告 ZIP</a> · <a href="manifest.json">索引</a></p>'+''.join(groups))


def main():
    records=collect();page(records)
    def job(r):
        if not r.get('source'):return r
        target=OUT/r['case_id']
        try:
            if not (target/'video-manifest.json').exists():
                with (target/'export.log').open('w') as log:
                    done=subprocess.run([sys.executable,'-m','scripts.collect_current_suite_oracle','--render',str(ROOT/r['source']),str(target)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
                if done.returncode:raise RuntimeError('See export.log')
            r['video_ready']=True
        except Exception as e:r['export_error']=str(e)
        return r
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
        futures={pool.submit(job,r):r for r in records}
        for future in concurrent.futures.as_completed(futures):
            r=future.result();page(records)
            print(r['case_id'],r.get('video_ready'),r.get('export_error',''),flush=True)
    with zipfile.ZipFile(OUT/'frontier-v1-oracle-videos-and-reports.zip','w',compression=zipfile.ZIP_STORED,allowZip64=True) as archive:
        for p in sorted(OUT.rglob('*')):
            if p.is_file() and not p.is_symlink() and p.suffix in ('.mp4','.json','.jsonl','.txt','.html'):
                archive.write(p,p.relative_to(OUT))
    print('DONE',sum(r['video_ready'] for r in records),'of',len(records),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--render',nargs=2);a=p.parse_args()
    if a.render:render(*a.render)
    else:main()
