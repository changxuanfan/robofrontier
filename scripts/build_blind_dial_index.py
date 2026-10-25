"""Public result/video index; does not load the private answer book."""
import html
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1];B=ROOT/'runs/oracle/dial-hold-blind-20261025';OUT=ROOT/'outputs/oracle/dial-hold-blind-20261025';OUT.mkdir(parents=True,exist_ok=True)
cards=[];rows=[]
for n in (4,6,8):
    for level in (1,2,3):
        cid=f'dial-n{n:02}-d{level}';reports=sorted((B/cid).glob('attempt-*/report.json'))
        candidates=[(p,json.loads(p.read_text())) for p in reports];winners=[x for x in candidates if x[1]['passed']]
        selected=winners[-1] if winners else candidates[-1] if candidates else None
        r=selected[1] if selected else {};passed=bool(winners);m=OUT/cid/'manifest.json';ready=m.exists() and json.loads(m.read_text()).get('views')
        active=any((d/'status.json').exists() and not (d/'report.json').exists() for d in (B/cid).glob('attempt-*'))
        status='通过' if passed else '运行中／重测中' if active else '未通过（已封存）' if reports else '待运行'
        ev=r.get('evaluation',{});queries=ev.get('valid_trials','—');actions=r.get('robot_action_count','—')
        video=''.join(f'<div><b>{label}</b><video controls preload="none" src="{cid}/{view}.mp4"></video><a href="{cid}/{view}.mp4">单独播放</a></div>' for view,label in [('global','正面'),('global_rear','反面')]) if ready else '<p>录像待导出。</p>'
        cards.append(f'<article><h2>{n} 盘 · L{level}</h2><p class="tag">{status}</p><p>有效反馈 {queries} 次 · 动作 {actions} 次</p>{video}<details><summary>运行范围</summary><p>来自全新初态物理运行，未回放旧指令。正确数量来自独立评估进程；解码进程只接收组合及计数。</p><pre>{html.escape(r.get("error","") or "")}</pre></details></article>')
        rows.append(dict(case_id=cid,passed=passed,selected_report=str(selected[0]) if selected else None,valid_trials=queries,actions=actions,video_ready=bool(ready)))
(OUT/'summary.json').write_text(json.dumps(dict(protocol='dial-continuous-grasp-v1',passed=sum(r['passed'] for r in rows),total=9,cases=rows),indent=2)+'\n')
page='''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>密码锁 · 未知答案 Oracle</title><style>body{font:16px system-ui;background:#f2f5f7;color:#193044;margin:0}main{max-width:1500px;padding:28px;margin:auto}h1{font-size:34px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(430px,1fr));gap:22px}article{padding:20px;border-radius:14px;background:white}h2{margin-top:0}.tag{display:inline-block;padding:5px 12px;background:#e7f3eb;border-radius:7px}video{width:100%;background:#e0e7eb;margin:8px 0}p{line-height:1.7}a{color:#08668d}pre{white-space:pre-wrap}</style><main><h1>密码锁 · 未知答案 Oracle</h1><p>持续夹持规则：所有旋钮在非零刻度稳定保持，才产生正确数量反馈。转到位继续夹持；必要换抓时当前组合无效，重新建立保持后再计分。没有正式解谜前的机械预演。</p><p>物理几何使用 R10，评估协议为 dial-continuous-grasp-v1；旧实验单独保留。录像为保存状态的离线重建。四盘L1为4倍速，其余为12倍速。</p><div class="grid">'''+''.join(cards)+'</div></main></html>'
(OUT/'index.html').write_text(page)
print('Whole tasks passed',sum(r['passed'] for r in rows),'/9; videos',sum(r['video_ready'] for r in rows))
