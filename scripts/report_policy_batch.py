"""Produce a portable human index from collected GPT results, not Oracle data."""
import argparse
from html import escape
import json
from pathlib import Path

LABELS={'transport':'物品传递','mosaic':'拼图','relay':'空中接力','dial':'旋钮'}


def progress(row):
    if row.get('total_goals') is not None:return f"{row['matched_goals']}/{row['total_goals']}"
    if row.get('total_payloads') is not None:return f"{row['completed_payloads']}/{row['total_payloads']}"
    if row.get('valid_trials') is not None:return '通过' if row.get('task_success') else f"{row['valid_trials']} 次有效试次"
    return '尚未收集'


def report(output):
    output=Path(output);result=json.loads((output/'results.json').read_text());rows=result['cases']
    lines=['# GPT最小规模12项试验','',
        'Adaptive；gpt-6.1-sol / high；每项从START起最多20分钟墙钟，思考、通信、工具及fork均计入。初始化时间不计入；到时停止新增动作，制动与封存另记。',
        '', '采用四类最小规模各三个Level。旋钮始终4盘：L1为4臂，L2/L3为8臂；其他为物品传递4臂、拼图4臂、空中接力8臂。',
        '', '这是用户授权的限时策略试验，不代表36项Oracle工程验收已经通过。每项从新Controller上下文启动，不复用Oracle/前一Level经验；任务内临时分身由Controller选择。',
        '', '| 类别 | Level | 臂数 | 物理验收结果 | 目标计数/反馈 | 任务墙钟 | 分身数 | 原速录像 |',
        '| --- | --- | ---: | --- | --- | --- | ---: | --- |']
    cards=[]
    for r in rows:
        state='通过' if r.get('task_success') else '到时未完整通过' if r.get('budget_exhausted') else '未完整通过' if 'evaluation_ref' in r else '待收集'
        if r.get('process_compliant') is False:state+='；存在过程违规'
        seconds=r.get('task_wall_s');duration='—' if seconds is None else f'{int(seconds)//60:02d}:{int(seconds)%60:02d}'
        filename=Path(r['video']).name if r.get('video') else None;link=f'[播放]({filename})' if filename else '—'
        lines.append(f"| {LABELS[r['family']]} | L{r['level']} | {r['arms']} | {state} | {progress(r)} | {duration} | {r['delegates']} | {link} |")
        media=f'<video controls preload="none" src="{escape(filename)}"></video>' if filename else '<p>录像尚未收集</p>'
        cards.append(f'<article><h2>{LABELS[r["family"]]} L{r["level"]} · {r["arms"]}臂</h2><p>{escape(state)} · {escape(progress(r))} · {duration}</p>{media}</article>')
    lines+=['','目标计数不等于整个任务成功；成功要求物理目标及过程规则均通过。时限结果使用制动前的budget-cutoff.json；不把停止后才稳定的物体计作准时完成。','',
        '录像保留等待与思考期间的原速过程。完整视频解码、源运动帧数、实际模型/effort/权限及fork父会话检查保存在results.json；完整60Hz源状态仍位于各run的private/outputs/motion中，便于离线迁移到工厂。float32量化及后续无损压缩的语义不变，这不是物理重启检查点。','',
        '权限不是OS文件沙箱。工具输入文字审计独立保存，不能据此声称对恶意越界有完备防护。']
    (output/'README.md').write_text('\n'.join(lines)+'\n')
    (output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>GPT12项录像</title><style>body{font:16px system-ui;background:#f5f7f9;color:#21313d;margin:28px}main{display:grid;grid-template-columns:repeat(2,minmax(300px,1fr));gap:20px}article{background:white;border:1px solid #dce3e8;padding:18px;border-radius:12px}h2{font-size:19px}video{width:100%;background:#e8edf1}</style><h1>GPT最小规模12项 · 原速录像</h1><p>Adaptive · gpt-6.1-sol / high · 每项START后最多20分钟。详细判据与审计见README.md及results.json。</p><main>'+''.join(cards)+'</main>')
    print(json.dumps(dict(index=str(output/'index.html'),readme=str(output/'README.md'))))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output',type=Path);a=p.parse_args();report(a.output)
