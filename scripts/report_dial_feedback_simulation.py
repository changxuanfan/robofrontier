"""Research report and public tables; no environment or evaluator mutation."""
from pathlib import Path
import json
import html
import shutil

ROOT=Path(__file__).resolve().parents[1];B=ROOT/'runs/oracle/dial-feedback-comparison-20261025'
OUT=ROOT/'docs/generated/frontier-v2/dial-feedback-comparison';OUT.mkdir(parents=True,exist_ok=True)
data=json.loads((B/'summary.json').read_text());lookup={(r['dials'],r['policy']):r for r in data['results']}
rows=[];actions=[];details=[]
for n in (4,6,8):
    c=lookup[n,'current_entropy'];b=lookup[n,'record_entropy'];a=lookup[n,'record_local_entropy'];simple=lookup[n,'record_coordinate']
    pct=lambda x,y:100*(x/y-1)
    rows.append(f"| {n} | {3**n:,} | {c['queries']['mean']:.2f} | {b['queries']['mean']:.2f} | +{pct(b['queries']['mean'],c['queries']['mean']):.1f}% | {a['queries']['mean']:.2f} |")
    actions.append(f"| {n} | {c['changed_dials']['mean']:.2f} | {b['changed_dials']['mean']:.2f} | {a['changed_dials']['mean']:.2f} | +{pct(a['changed_dials']['mean'],c['changed_dials']['mean']):.1f}% | {simple['changed_dials']['mean']:.2f} |")
    for name,title in [('current_entropy','当前数／熵优先'),('record_entropy','历史最高／同类熵优先'),('record_local_entropy','历史最高／加入纪录组合邻域实验'),('record_coordinate','历史最高／逐位比较')]:
        r=lookup[n,name];details.append(f"| {n} | {title} | {r['queries']['mean']:.2f} | {r['queries']['p95']:.0f} | {r['queries']['maximum']} | {r['changed_dials']['mean']:.2f} |")
report='''# 历史最高反馈：逻辑复杂度穷举模拟

这是离散逻辑模拟，没有启动 MuJoCo、机械臂或 GPT。4／6／8盘分别穷举81／729／6,561个密码，每个密码等权；四个策略共29,484条完整求解路径。没有使用当前九项固定答案，也没有把答案作为选下一次尝试的输入。

## 平均逻辑试次数

| 盘数 | 全部密码数 | 当前正确数 | 历史最高：同类策略 | 增幅 | 历史最高：适配邻域策略 |
|---|---:|---:|---:|---:|---:|
'''+ '\n'.join(rows)+'''

前两列求解器采用相同的候选采样上限96、熵优先和0.015改位惩罚，只按真实反馈的含义分别更新候选。适配策略额外允许在创造纪录的已知组合附近做单旋钮实验；它没有获得当前正确数或某位是否正确。

## 平均单旋钮改位数

| 盘数 | 当前数／熵优先 | 历史最高／同类策略 | 历史最高／适配邻域 | 相对当前数增幅 | 历史最高／逐位比较 |
|---|---:|---:|---:|---:|---:|
'''+ '\n'.join(actions)+'''

改位数是相邻两次提交组合之间改变了几个旋钮，包括开局从0到非零。逐位策略失败后，恢复旧位也计入下一次提交所需改位。它不是底层机器人动作数，不含转动角度、轨迹、撤离、碰撞或抓取恢复。

## 尾部与策略差异

| 盘数 | 策略 | 平均试次 | P95 | 最多试次 | 平均改位 |
|---|---|---:|---:|---:|---:|
'''+ '\n'.join(details)+'''

八盘同类熵策略的最坏57次，是该策略的缺点，不是游戏必需57次。独立的逐位纪录策略在全部密码上最多9／13／17次，给出一个可达到的上界；邻域策略平均更省试次，但不是全局最优求解器。不能把旧算法适配不好制造的失败当成任务固有难度。

## 结论

- 候选密码空间仍是3^N，没有变大。反馈丢失了“本次比纪录更差还是持平”的信息，主要增加试验设计、历史输入归因及候选约束维护的负担。
- 同类策略平均逻辑试次增加约28%／64%／86%。适配新反馈后增加约26%／51%／70%，而单旋钮改位只增加约3%／10%／15%。这适合增加推理负担而不同比增加机械运动。
- 也可以选择更多逻辑试次、少改位的逐位策略。因此试次数、改位、持握时间和总耗时要分别统计，不能只用运动并行率评判这种任务。
- 同类策略的平均候选—查询匹配量只增加约4%／12%／14%；以上是计算工作量代理，不是GPT时间或物理仿真耗时预测。模拟程序共享决策树前缀，总运行时间不代表逐个实例单独运行的时间。

## 适用边界

每个完整输入被当作一次清晰的结算。模拟不包含运动中间态自动计数、重新抓取、观察遗漏、LLM推理错误或机器人动力学。因此不能把表中倍数当作GPT成功率变化或MuJoCo墙钟时间倍数。若保留连续自动刷新机制，运动途中停留也可能刷新BEST，必须确保纪录事件与可观察输入的对应关系；否则原本想增加推理，实际增加的是信息归因歧义。

本次没有覆盖九个旧环境或改旧录像的CORRECT标签。新标签、规则、评估器和解码器必须同步切换；不能只换文字。新规则草案见feedback-contract.md。
'''
(B/'report.md').write_text(report)
contract='''# BEST反馈规则草案（未部署）

仪表标题：BEST / NN。NN是旋钮总数。数值为本轮完整有效输入曾取得的最高正确数；它不是当前输入的正确数。

1. 本轮尚无有效试次时显示BEST --。首次有效结算后显示00～NN。
2. 全部旋钮实际被保持，处于非零刻度的原±4°容差内并稳定0.25秒，才形成有效完整输入。L2/L3保持原托举条件。
3. 每次有效结算，评估器私下计算当前完整组合的正确数，并只公开更新后的历史最大值。不得公开当前较低正确数、逐位匹配或密码。
4. BEST不涨，意味着本次正确数不超过现纪录；BEST上涨，在该结算对应输入明确的前提下，新值就是这次的正确数。
5. 纪录只能来自同一次完整组合，不得拼接各旋钮在不同时间的成绩。实际同时全部正确时才解锁。
6. 无效、松手或零位时，纪录保留，READY熄灭；重新形成有效输入才可结算。READY只表示输入有效，不提供额外正确性信息。
7. 纪录仅在本轮世界内保留，新轮次清空。原有已封存记录不改写。

结算归因：部署前须明确选用离散提交边界，或为连续自动结算提供可观察的试次/创纪录事件。运动中间态也可能达到稳定条件，不能无依据把其纪录归给最后的目标组合。该选择会影响实际有效试次数，应随版本固定。

求解器更新：若纪录从B升到B2，则该明确结算的输入得分恰为B2；若仍为B，则本次输入得分≤B。保留创造纪录的输入及其证据，不把纪录当成现在仍保持正确的证明。

工程范围：新增feedback_mode=current或best_so_far、纪录整数及结算序号；复用已有状态和录制。物理参数、档位数和通用机器人控制接口不因本规则改变。任务说明和面板标题必须与评估语义一致。BEST/READY等视觉变化继续由已有外观事件录制，便于离线回放。
'''
(B/'feedback-contract.md').write_text(contract)
for name in ['summary.json','report.md','feedback-contract.md','per-password.csv']:
    shutil.copyfile(B/name,OUT/name)
table_html=''
for title,rs,heads in [('平均逻辑试次数',rows,['盘数','密码数','当前数','历史最高：同类策略','增幅','历史最高：适配策略']),
                       ('平均单旋钮改位数',actions,['盘数','当前数','历史最高：同类策略','历史最高：适配策略','增幅','历史最高：逐位'])]:
    body=''.join('<tr>'+''.join('<td>'+html.escape(v.strip())+'</td>' for v in row.strip('|').split('|'))+'</tr>' for row in rs)
    table_html+=f'<h2>{title}</h2><table><thead><tr>'+''.join('<th>'+h+'</th>' for h in heads)+'</tr></thead><tbody>'+body+'</tbody></table>'
page='''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>BEST反馈复杂度模拟</title><style>body{font:17px system-ui;background:#f3f5f6;color:#172e3f;line-height:1.8}main{max-width:1150px;margin:40px auto;padding:0 24px}h1{font-size:34px}h2{margin-top:36px}table{width:100%;border-collapse:collapse;background:white}th,td{padding:12px;text-align:right;border-bottom:1px solid #dce4e9}th:first-child,td:first-child{text-align:left}th{background:#e7eff3}a{color:#08618d}.note{background:#fff5dd;border-left:4px solid #d59c35;padding:18px}code{background:#e7eff3;padding:3px 6px}</style><main><h1>从 CORRECT 到 BEST：难度增加多少？</h1><p>三个有效档位保持不变。穷举4／6／8盘全部密码，四种求解策略共29,484条求解路径。没有读取九项真实密码，也没有运行机械臂或GPT。</p>'''+table_html+'''<p>同类策略使用相同熵优先和改位惩罚；适配策略额外允许围绕已知纪录组合做局部实验。它没有获得当前正确数。</p><p class="note">这些是离散逻辑试次与改位数，不是MuJoCo动作数或耗时倍数。运动途中自动刷新、抓取恢复和模型错误均未模拟。更多试次不必意味着同比增加机械运动。</p><h2>说明与界面要一起改</h2><p>面板应标为 <code>BEST / NN</code>，文字明确“本轮完整有效输入的历史最高正确数，并非当前正确数”。建议另有READY指示，区分有效但没进步和根本未形成有效输入。</p><p>本次仅模拟和规则草案，旧环境、旧CORRECT标签和旧录像未被覆盖。正式部署前要处理运动中间态刷新纪录的归因问题。</p><p><a href="report.md">完整分析与尾部统计</a> · <a href="feedback-contract.md">新规则草案</a> · <a href="summary.json">全部统计JSON</a> · <a href="per-password.csv">逐密码结果CSV</a></p></main></html>'''
(OUT/'index.html').write_text(page)
print(OUT/'index.html')
