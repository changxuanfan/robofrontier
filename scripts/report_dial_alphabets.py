"""Publish discrete BEST-feedback distributions, with sampling provenance."""
from pathlib import Path
import csv
import html
import json
import shutil

ROOT=Path(__file__).resolve().parents[1];B=ROOT/'runs/oracle/dial-feedback-alphabet-20261025'
OUT=ROOT/'docs/generated/frontier-v2/dial-alphabet-comparison';OUT.mkdir(parents=True,exist_ok=True)
summary=json.loads((B/'summary.json').read_text());rows=[]
prior=json.loads((ROOT/'runs/oracle/dial-feedback-comparison-20261025/best-smart-percentiles.json').read_text())
for p in prior['results']:
    rows.append(dict(dials=p['dials'],alphabet=3,passwords=p['cases'],tested=p['cases'],exhaustive=True,
        mean=p['mean'],percentiles={f'P{q}':p[f'P{q}'] for q in (5,20,50,80,95)},baseline=True))
rows+=summary['results'];rows.sort(key=lambda r:(r['dials'],r['alphabet']))
md=['# BEST反馈：3～7个有效档位的试次分布','',
    '有效档位不包括无效初态0。继续使用候选消元、信息熵和历史纪录组合邻域实验的参考策略；不是全局最优次数，也不是GPT或MuJoCo实测。',
    '','3档基线沿用上次全枚举。4盘全部档位、6盘4档为全枚举；其余每组4096个均匀无放回抽样。所有候选后验精确保留；仅在候选数超过8192时，用独立于测试密码的8192个候选估计查询熵。',
    '','扩展程序已在3档的81、729、4096条测试路径上逐条复现上次查询与改位次数。四条回归测试通过。']
tables=[]
for n in (4,6,8):
    md+=['',f'## {n}盘','', '| 有效档位 | 平均次数 | P5 | P20 | P50 | P80 | P95 |','|---|---:|---:|---:|---:|---:|---:|']
    html_rows=[]
    for r in [r for r in rows if r['dials']==n]:
        values=[str(r['alphabet']),f"{r['mean']:.2f}"]+[str(r['percentiles'][f'P{p}']) for p in (5,20,50,80,95)]
        md.append('| '+' | '.join(values)+' |')
        html_rows.append('<tr>'+''.join('<td>'+html.escape(x)+'</td>' for x in values)+'</tr>')
    tables.append(f'<h2>{n}盘</h2><table><thead><tr>'+''.join('<th>'+x+'</th>' for x in ['有效档位','平均次数','P5','P20','P50','P80','P95'])+'</tr></thead><tbody>'+''.join(html_rows)+'</tbody></table>')
md+=['','## 如何理解','',
    '- P95=36表示至少95%的样本密码在36次以内完成，不是恰好36次的概率。次数包含首次尝试与成功那次。抽样百分位可能浮动，JSON保留均值区间及分位数Bootstrap区间。',
    '- 例如八盘七档：平均约27.20次，均值近似95%区间27.04～27.37；P95的Bootstrap区间36～37次。这些区间只描述密码抽样误差，不包括换策略、模型或物理误差。',
    '- 在当前范围和策略下，六盘／八盘每增加一个有效档位，平均大致再增加3／4次试次。这是数值观察，不是复杂度定理。',
    '- 一种简单逐位纪录策略可以保证不超过1+N×(K−1)次：七档的4／6／8盘分别25／37／49次。它不是最优策略，平均比本表策略更慢，但说明不需要遍历K^N个密码。',
    '- 没有模拟档位具体角度、运动途中额外结算、持握失误、机械臂碰撞或GPT推理错误，不能把试次倍数当成真实耗时倍数。',
    '- 此前Oracle的24轮工程上限不宜原样用于所有新难度；需要先确定逻辑预算与机械时间预算，避免把单纯超限混成推理失败。',
    '', '本次没有修改任何实际场景、密码、评估接口或旧录像。']
(B/'report.md').write_text('\n'.join(md)+'\n')
(B/'combined-summary.json').write_text(json.dumps(dict(assumptions=summary['assumptions'],results=rows),indent=2)+'\n')
with (B/'percentiles.csv').open('w',newline='') as file:
    fields=['dials','alphabet','passwords','tested','exhaustive','mean','P5','P20','P50','P80','P95'];w=csv.DictWriter(file,fieldnames=fields);w.writeheader()
    for r in rows:w.writerow({**{k:r[k] for k in fields[:6]},**r['percentiles']})
for name in ['report.md','combined-summary.json','percentiles.csv','baseline-validation.json']:
    shutil.copyfile(B/name,OUT/name)
page='''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>BEST：档位数与试次分布</title><style>body{font:17px system-ui;background:#f3f5f6;color:#1a3041;line-height:1.8}main{max-width:1100px;padding:24px;margin:24px auto}h1{font-size:34px}table{border-collapse:collapse;background:white;width:100%}th,td{padding:12px;text-align:right;border-bottom:1px solid #d9e3e9}th{background:#e6eef2}th:first-child,td:first-child{text-align:left}h2{margin-top:36px}a{color:#0c638c}.note{padding:18px;background:#fff3d8;border-left:4px solid #c99a3b}</style><main><h1>BEST反馈：3～7档的尝试次数分布</h1><p>有效档位不包括0。延续候选消元＋历史纪录邻域实验策略；不是理论最优，也不是GPT实测。</p><p class="note">3档基线、4盘所有设置及6盘4档为全枚举。其余每组4096个均匀抽样，分位数为估计。密码过滤始终精确；大后验只对查询熵做8192候选采样。</p>'''+''.join(tables)+'''<h2>解释与边界</h2><p>百分位表示该比例的密码在多少次以内完成，包含首次与最终成功尝试。没有模拟机器人、角度布局或运动中间态自动刷新。八盘七档P95估计36，Bootstrap区间36～37。</p><p>七档时简单逐位策略有25／37／49次的保底上界，说明不需要枚举整个密码空间。这些是参考策略的表现，不能当成GPT成功率或物理时间预测。</p><p><a href="report.md">完整报告</a> · <a href="percentiles.csv">下载统计CSV</a> · <a href="combined-summary.json">统计与不确定性JSON</a> · <a href="baseline-validation.json">3档路径复现核验</a></p></main></html>'''
(OUT/'index.html').write_text(page)
print(OUT/'index.html')
