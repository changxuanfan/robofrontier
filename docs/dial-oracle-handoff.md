# 密码锁 Oracle 交接：2026-10-26，用户要求 compact 前暂停

## 当前指令和状态

**已按用户要求暂停。不要自动启动实验；用户 compact 后说“继续”再恢复。**

用户最新要求：失败不能立即终止整个 Oracle；应保留现场，允许 GPT 分析失败并做实际恢复，必要时回退探索，但不能混拼不同轨迹伪装连续成功。继续解决三个 L3，不应把首轮6/9当作任务整体结束。

当前没有本任务的物理进程、求解／裁判子进程或恢复实验在运行。首轮九项已全部结束，18段正反录像全部导出且可完整解码，9项盲解码日志审计完成。

## 工作区：必须用这个 worktree

- `/Users/fanchangxuan/.codex/worktrees/dial-best5-oracle/roomscript2`
- 分支：`task/dial-best5-oracle`
- 本分支 HEAD：`1b6e5c7`，是首轮正式运行的冻结代码。
- 原始工程：`/Users/fanchangxuan/Repos/roomscript2`；初始无Git，已建立公共 main。
- 公共基线提交 `8e7872f`，忽略外部数据链接修正 `d3e5916`。
- **本任务代码没有 merge 到 main。不要修改／调度用户另外三个类别的 fork 或 worktree。**
- 大模型资产未放进Git；此 worktree 所需模型文件使用独立 APFS COW 副本，已确认不同inode。`.venv`、vendor、runs、outputs、docs/generated、私有裁判输入是外部链接。
- 公共初始化工具 `scripts/setup_worktree_data.py` 已在main；其他类别由用户在其他会话管理。
- 当前恢复代码有未提交修改，**不等同已验证版本，尤其不能直接启动默认肩部抓取策略。**

## 首轮正式 Oracle：真实结果，不要重跑已通过六项

目录：`/Users/fanchangxuan/Repos/roomscript2/runs/oracle/dial-best5-r11-worktree-20261025`

- `batch-status.json`：active=false，9项结束，6项通过。
- `audit.json`：9项均只用solver输入日志重放，结果一致；审计没有读取私有密码。
- `manifest.json`：冻结提交、工作区、配置及预算。
- 每项 `dial-best5-nXX-dY/attempt-001/` 有 report、actions、trace、motion、冻结源码、裁判日志等。

| 任务 | 结果 | 关键情况 |
|---|---|---|
| 4盘L1 | 通过 | 136.3秒，8次显式选题／20个实际有效输入，零失败动作／违规／未知；峰值4臂运动 |
| 4盘L2 | 通过 | 160.0秒，6次显式选题／9个有效输入，零失败动作 |
| 4盘L3 | 未通过 | BEST2/4；D04蓝柄支撑丢失后G仍转动，`driven_red_without_support`，发生于sim276.442秒 |
| 6盘L1 | 通过 | 149.1秒，22个有效输入，零失败动作 |
| 6盘L2 | 通过 | 441.7秒，20个有效输入，零失败动作 |
| 6盘L3 | 未通过 | `Blue grasp did not requalify before red operation: D04`；约159秒。防护阻止继续转动 |
| 8盘L1 | 通过 | 250.9秒，21个有效输入，零失败动作 |
| 8盘L2 | 通过 | 568.8秒，14个有效输入，零失败动作 |
| 8盘L3 | 未通过 | BEST6/8，约885.6秒；`No reachable small open-crank exit I`，重抓释放无法找到小幅撤离IK；6个失败动作，最终没有未结算动作 |

录像：`/Users/fanchangxuan/Repos/roomscript2/outputs/oracle/dial-best5-r11-worktree-20261025`
每项global/global_rear，60Hz实际状态离线重建，12倍速，**没有重新积分物理来制造视频**。

公开结果页（已更新为九项结束）：
`http://127.0.0.1:56719/best5-blind-oracle-worktree-20261025/index.html`
初态九任务图：`http://127.0.0.1:56719/best5-dials-r11/all-tasks-top.png`

首轮管理session85896已结束，首次导出74255、最终导出50844、最终审计33771均已结束。不要重新等待这些session或把它们当活进程。

## 盲测／物理边界必须保持

- R11九项：4/6/8盘 × L1/L2/L3；L1对应4/6/8臂，L2/L3对应8/12/16臂。
- 刻度0..5，1..5有效，步长60°。3可为±180°，4/5可为-120°/-60°。
- 反馈是历史BEST，初始0；READY灰无效／橙资格等待／绿就绪。未ready数字暗但保留记录；绿亮数字亮。
- 有效条件保持原样：非零档位±4°、250ms，真实红双指接触且有效力保持；L2/L3另有独立蓝柄支撑并整体离外部支持至少20mm。
- **自动结算包含实际中间组合**；不能只算请求目标，不能把BEST不变当当前得分。
- 只有 evaluator 进程读私有密码。solver只拿N/K、时间、实际可见输入组合、历史BEST。未知归属保留多个解释。
- 真实几何／关节／机构角度是**Oracle特权**，没有新增GPT任务角色的真值接口。
- 不读 `private-evaluator-input.json` 或 `configs/evaluations/private/...` 内容进研究上下文；不改配对密码。
- 不改环境物理、摩擦／NoSlip／力上限／验收规则来补成功。当前2ms、NoSlip20、已用pd1000-v1 hand control；正常机器人夹持命令可调整，必须真实执行和记录。
- 已有规则违规不能悄悄清除。若恢复后解锁，分别报告goal_reached和clean success；要证明无违规可从初态用修复策略重跑，不能拼接回退片段。
- 当前只做Oracle，**没有授权自动开始GPT实验或自动merge主分支**。

## 当前未提交恢复代码（需要继续完成和验证）

先读 `git status --short` 和差异。主要文件：

1. `robofrontier/suite/dial_live_repair.py`（新文件）
   - 密码锁私有参考的LiveRepair桥接，复用现有通用 `OracleRecovery`，不是另一个执行器。
   - 遇到可恢复错误先受控停止，记录现场图像、机器人反馈、机构姿态和原错误，保留当前物理世界及盲解码器。
   - 在 `trial/recovery/awaiting.json` 发布暂停目录；研究会话提交 `pause-NNN/plan.json` 的普通机器人命令，然后继续同一目标。
   - plan格式：`{"source":"research_gpt","batches":[{"A":["gripper",{"mode":"move","width_m":0.08,"speed_m_s":0.05}]}],"resume":true}`。
   - 必须原子写plan，所有请求/结果/失败方案保留；不支持qpos重设、删除违规或直接给密码。
   - 如用户STOP或ReferenceStopped，仍尊重停止。等待会计入墙钟预算。
   - 需要真失败端到端验证，当前只有循环／执行器测试。

2. `robofrontier/suite/oracle_recovery.py`（原有通用模块）
   - **原先已经存在，不要覆盖。** 开发中曾误占名字，已恢复原模块完整内容。
   - 当前只加了`reattach(owner)`、`acknowledge(owner,id,review,assisted=False)`、`resume(owner)`，支持多次已结算交回／重新接管，同一物理执行owner不变。
   - 原有uncertain fencing、独立机器人序列、源码哈希、assisted分类保留。

3. `dial_best_oracle.py`
   - solve现在在lift/set_symbols失败后进入LiveRepair，而不是立刻关闭。
   - 同一个猜测重试，盲解码历史／BEST保留；没有回退状态拼接。
   - release新增候选：红手已退出force mode且蓝支撑仍有效时，让蓝手通过普通动作平移模块12mm，生成相对撤离，避免把已伸到极限的红臂继续向外伸；最多有限尝试，尚未实测成功。
   - **危险待修：`blue_capture_mode='shoulder'`现在是默认，但肩部方案已验证在初态不成立。恢复后先改回`stem`默认，肩部只能显式实验，不能直接拿当前默认跑正式L3。**
   - shoulder选项当前是夹住两挡肩外侧的32mm跨度、闭合轴沿蓝柄长轴，抓点在两挡肩中间；这是尚未通过的尝试。

4. `dial_oracle.py` / `dial_oracle_v2.py`
   - blue_capture_options／blue_approach_aperture为旧参考留默认不变的扩展点。
   - Best L3可先把空爪开到“目标跨度+12mm”再接近；已做44mm预开口测试，但没解决肩部抓取碰撞。
   - `rule_failure()`允许在**显式恢复过已记录违规**的工程尝试中继续，但Monitor的违规和task_success判定不删除。

5. `oracle.py`
   - execute遇异常会把已经提交但未结算的calls记录为uncertain，保留原命令并推进编号，避免重用call ID；实际停止反馈另记。
   - llm_used根据在线修复干预标记；最终仍应由通用恢复适配器close提供完整assisted元数据。

6. 正式runner和batch
   - `run_blind_dial_oracle.py`协议已改为`dial-best5-continuous-grasp-repair-v2`，增加goal_reached／干预次数／continued_after_violation。
   - **未完成的重要接线：runner finally仍直接poll／trial.close。若通用恢复adapter还attached，应通过adapter.pause/advance排空，最后adapter.close(report)，不要偷poll或漏掉assisted/recovery元数据。**
   - `run_best5_dial_batch.py`首项失败后不再阻止其他项启动，符合用户“不要失败就停”。
   - 新批次要独立目录，不能覆盖首轮；建议只跑失败三个L3，并让恢复channel保留现场。

## 已验证／否定的物理线索：不要重复盲试

蓝柄D04的首轮只读诊断位于原正式目录：
- `support-geometry-diagnostic.json`、`diagnose_support_geometry.py`。
- 抬起时蓝手H夹爪约19.9mm，模块相对手姿正常。
- 4盘L3失败前模块相对H滚转约36°，夹爪被撑到约35mm；出现挡肩与H手掌接触，支撑丢失。
- 检查故障前红手G接触：只有红握把的正常手指接触，没有发现该处红掌撞盘的证据。不要未经证据就把故障归因于红手撞盘。

肩部抓取探针：
`/Users/fanchangxuan/Repos/roomscript2/runs/engineering/best5-recovery-20261026/shoulder-d04/attempt-001`、`attempt-002`。
- 两次均未开始D04转动，先在初始D01蓝侧接近失败：`No side-access blue grasp D01`。
- 第一次使用80mm开口；第二次先收至44mm，仍失败。
- 零物理步的独立FK/碰撞检查显示，闭合轴沿柄长轴的32mm肩夹姿态会让手掌碰白盘，穿透约12.7mm；不是单纯IK无解，IK位置误差本身很小。
- 另试过仅FK：在靠外挡肩（z=-.078）的翼部、y偏移±22mm抓4mm薄翼，仍有手掌／link7与盘的约10～14mm干涉；部分20/35/50/65度倾斜候选也没在初始桌面姿态找出无碰撞端点。上述翼部与倾斜搜索**仅诊断，没有写进当前实现，没有做动力学成功验证**。
- 可继续思考的方向（尚未实施／证明）：先用已验证的stem方式抬起，再让另一只手临时支撑／握住机构，蓝手在空中换更稳抓法；或者改变整个盘体呈现姿态，降低偏心重力／给后续红手留空间。任何动作仍通过普通接口，先小单元，不改物理／秘密。

## 测试状态：不要混淆版本

- 冻结提交1b6e5c7正式启动前57项软件测试通过。
- 新恢复版最近一次针对性组：`tests.test_oracle_live_repair tests.test_oracle_recovery tests.test_best5_oracle tests.test_dial_hold_oracle` 共30项通过。
- 此后有blue approach aperture等小改动，未把整套恢复版本端到端跑通；不能称“所有恢复已验证”。
- 初次测试fake world缺model导致线程等待，已修test fixture和daemon，旧test PID3730已明确停止。
- worktree为通用恢复测试补了R10四臂transport资产COW（只作测试fixture，不改其他类别代码）。
- 新测试`tests/test_oracle_live_repair.py`覆盖失败计划后继续、真实执行器命令、打断call ID不复用、肩夹旋转/跨度数学；后者不是物理抓持成功证明。

## compact后建议的恢复顺序

1. 确认用户已说“继续”，读本文件和Git差异；使用当前worktree，不要回原始main工程改代码。
2. 把未通过的shoulder默认改回stem；保留或隔离该实验，避免又从初态失败。
3. 完成runner与现有OracleRecovery的排空／close接线，确认违规不消失、unknown不重发、assisted标记准确。
4. 用一个受控的可恢复失败做真实端到端：世界保留→发布awaiting→研究会话给普通修复动作→同一解码器继续；不只测helper。
5. 对8盘L3的不可达撤离先做蓝手协同相对移开的最小验证；对D04支撑先验证更稳的控制／换抓方案。
6. 测试通过后提交当前分支，新的三个L3尝试用新目录、同一私有配对密码；过程中失败继续保留world供分析，不立刻退出。
7. 如果发生已记录违规，继续尝试可证明goal_reached，但clean success仍不能伪造；必要时用修复策略从标准初态重跑形成连续合规证据。

## compact保护备份

外部目录：`/Users/fanchangxuan/Repos/roomscript2/runs/oracle/dial-recovery-20261026/`
- `working-source.tar.gz`：11个当前修改／新源码的完整副本（创建本交接文档之前）。
- `working-tree.patch`：相对于1b6e5c7的tracked差异。
- `working-source-manifest.json`：分支、基准commit、路径和逐文件SHA256。
- 本文会复制为该目录的`HANDOFF.md`。

当前文件本身也在worktree的`docs/dial-oracle-handoff.md`，未提交但compact不会删除文件。**不要用reset/clean来“整理”这些未提交恢复工作。**
