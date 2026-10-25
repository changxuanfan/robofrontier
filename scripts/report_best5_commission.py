"""Summarize bounded local coverage without promoting it to task success."""
from pathlib import Path
import json
import html
import shutil

ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'runs/engineering/best5-r11-commission'
cat=json.loads((ROOT/'configs/suites/dial-best5-r11.json').read_text());cases=[]
for case in cat['cases']:
    attempts=[];coverage={};released=set();complete=[];simultaneous=[]
    for p in sorted(BASE.glob('*/attempt-*/report.json')):
        r=json.loads(p.read_text())
        if r['case_id']!=case['id']:continue
        for mark in r.get('marks',[]):
            if mark['passed']:
                for dial in mark['devices']:coverage.setdefault(dial,set()).add(mark['symbol'])
        if r.get('release') and all(r['release']['released']) and r['release']['all_supported'] and max(r['release']['residual_deg'])<=4:
            released.update(r['devices'])
        local=bool(r.get('local_passed',r.get('passed',False)))
        if local and len(r.get('devices',[]))==case['dials']:
            events=(p.parent/'trace.jsonl').read_text().splitlines()
            if any(json.loads(line)['event']=='observed_best_input' for line in events):simultaneous.append(str(p.relative_to(ROOT)))
        if local and len(r.get('devices',[]))==case['dials'] and set(x['symbol'] for x in r.get('marks',[]) if x['passed'])=={1,2,3,4,5} and r.get('release'):complete.append(str(p.relative_to(ROOT)))
        attempts.append(dict(report=str(p.relative_to(ROOT)),local_passed=local,error=r.get('error'),marks=[m['symbol'] for m in r.get('marks',[]) if m['passed']],
                             devices=r.get('devices',[]),failed_robot_actions=r.get('failed_robot_actions'),unknown_robot_actions=r.get('unresolved_robot_actions',[]),
                             inter_robot_contacts=r.get('evaluation',{}).get('inter_robot_contacts')))
    cases.append(dict(case_id=case['id'],dials=case['dials'],arms=case['arms'],level=case['level'],
        tested_symbols_by_dial={k:sorted(v) for k,v in sorted(coverage.items())},passive_return_verified_dials=sorted(released),
        all_dials_five_symbols_and_release_witness=complete,all_device_ready_witness=simultaneous,full_oracle_verified=False,attempts=attempts))
summary=dict(scope='Mechanical unit coverage, often across independent fresh worlds. No complete blind code-solving episode.',
             case_count=9,scenes_with_local_hold_evidence=sum(bool(c['tested_symbols_by_dial']) for c in cases),
             scenes_with_all_dials_five_marks_and_release=sum(bool(c['all_dials_five_symbols_and_release_witness']) for c in cases),
             scenes_with_all_device_ready_evidence=sum(bool(c['all_device_ready_witness']) for c in cases),
             full_oracle_runs=0,tests_passed=55,cases=cases)
(BASE/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
OUT=ROOT/'docs/generated/frontier-v2/best5-dials-r11';(OUT/'commissioning-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
rows=[]
for c in cases:
    chosen=max(c['attempts'],key=lambda a:(len(set(a['marks'])),a['local_passed']))
    src=ROOT/chosen['report'];dest=OUT/'commissioning'/c['case_id'];dest.mkdir(parents=True,exist_ok=True)
    links=[]
    for view in ('top','global','global_rear'):
        p=src.parent/f'final-{view}.png'
        if p.exists():
            shutil.copyfile(p,dest/f'{view}.png');links.append(f'<a href="commissioning/{c["case_id"]}/{view}.png">{view}</a>')
    tested='; '.join(d+': '+','.join(map(str,m)) for d,m in c['tested_symbols_by_dial'].items())
    rows.append(f'<tr><td>{html.escape(c["case_id"])}</td><td>{"已验证" if c["all_device_ready_witness"] else "待验"}</td><td>{html.escape(tested)}</td><td>{html.escape(", ".join(c["passive_return_verified_dials"]) or "待验")}</td><td>{" / ".join(links)}<br>{html.escape(chosen["error"] or "该次局部单元通过")}</td></tr>')
page='''<!doctype html><meta charset="utf-8"><title>BEST5机械小验收</title><style>body{font:17px system-ui;color:#172d3f;background:#f3f5f6;margin:35px}table{border-collapse:collapse;background:white;width:100%}td,th{padding:15px;border:1px solid #ccd6dc;text-align:left}p{max-width:1200px;line-height:1.8}a{color:#08638c}</style><h1>BEST5 R11：机械小验收记录</h1>'''
page+=f'<p>{summary["scenes_with_all_device_ready_evidence"]}/9 种布局已实际形成整组有效 READY 输入。此项只证明至少一个全盘有效组合，不等于每盘全部五档、全部切换或完整盲解谜都已通过。完整盲解谜运行数：0。</p>'
page+='<p>不同标号和回弹证据可能来自独立初态运行；单盘局部通过不代表所有盘同时完成该档位。所有失败、中止及独立重测记录保留。八盘L2先因240秒预算中止，再在独立单独运行中通过。六盘L3/D06同一连续运行中通过1～5档保持，但后续返回1档／释放前到达300秒预算，未标为整段通过。</p><p><a href="index.html?v=panel-layout">九个场景的新布局</a> · <a href="commissioning-summary.json">详细证据索引</a> · <a href="commissioning-videos/dial-n04-d1/global.mp4">四盘L1机械录像：正面</a> · <a href="commissioning-videos/dial-n04-d1/global_rear.mp4">背面</a>（真实状态回放，12倍速，不是完整解谜）</p>'
page+='<table><tr><th>任务</th><th>整组 READY</th><th>已测盘／通过的保持档位</th><th>回零已验证的盘</th><th>代表运行的最后状态／该次结果</th></tr>'+''.join(rows)+'</table><p>55项软件回归测试通过。运动规划使用Oracle几何权限；密码仅由独立评估进程读取，解码器只接收时间、实际组合与历史BEST。GPT观察接口维持原规则。</p>'
(OUT/'commissioning.html').write_text(page)
print(json.dumps({k:v for k,v in summary.items() if k!='cases'}))
