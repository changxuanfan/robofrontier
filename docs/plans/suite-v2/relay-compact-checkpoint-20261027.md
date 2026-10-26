# Compact 后先读此文件 — 2026-10-27

这是本轮最新接续说明，覆盖旧 `relay-continuation.md` 中过期的进度、
5/25mm 标准和“尚未热加载/尚未得到许可”等状态。先读完，再核对原现场。
**用户刚要求 compact 后仍保留细节，并明确“程序不要停”。不要停止、重启、
另开原仿真，也不要把 compact 当作取消任务。继续直到9项完成。**

## Compact 后继续工作的最新增量（2026-10-27）

以下新进度覆盖下文旧前沿：**24/27根完成，仍7/9项**。原PID/world不变。
- 16L2 P3：`r16-p3-P-current-normal-escape-v44` 已真实完成分离；独立50mm验收通过，
  仅约1.56mm越线、无force/robot contact，native completed也为true，drops0。
  `r16-p3-P-separated-home-v45`已完成，P可重新参与运输。16L2现2/4。
- 16L2 P2：G/O→H/O(v41)完成；H为Side的中间抓姿不再被错误当作禁用。
  多个普通carry前移到[1.55,.8,1.2]，`r16-p2-HO-to-IO-palm-clear-Top-v51`
  已完成I/O Top；最新helper正在执行v52 I/P、v53前移、v54 J/P的有限前缀，
  **先核对原results和pending，不能重放这些ID**。P1仍D/M Top，当前local站位约-.431/+.320m。
- 16L3 P1：确认旧N Side张爪后本来已可无接触停留；把立即home当成交接条件是冗余约束。
  新`robofrontier/suite/relay_donor_staging.py`只预测完整21点普通75mm开口并要求
  停留点1mm全景无接触，不改活物理；后续carry仍包含静止donor的全部几何。
  同一52510进程已按既有用户热更新授权加载`parked-donor-v1.py`，SHA256
  `2fa937b9cbc878075ded78ae0ac6c30cc2b9584e07dad7ae3d95696a83e13126`；
  ack/queue/detach均确认，runtime source/meta在原16L3 recovery/controller-extensions。
  新loader `runs/engineering/relay-descent-20261026/relay_parked_hotload_loader.py`。
  捕获现场正常75mm开口通过、45mm未张开负例拒绝、私有诊断qpos未变；21个已有recovery单测通过。
- `r16-p1-HN-to-HO-safe-parked-N-v47`已实际完成H/O Top交接；
  `r16-p1-HO-separate-parked-N-v50`已沿实测N tool Z移动120mm，木杆与N真实分开。
  N直接home(v51)只规划失败无动作；N局部q[2]+.4的v54已真实完成。
  下一段H/O forward仍被N_hand/O_link5..7挡住；不要重跑大范围IK。
  最新helper正在验证/执行`N-compound-future-fold-v58-1..`和`HO-Ncompound-forward-v59`，
  以完整后续carry清障为目标选有限空手折叠路径。单关节v56、carry v57未提交。
  N复位并非必须，只需当前无力空手的完整实际短路径让开后续transport。
- L2和L3两套原世界继续并行，各最多物理1核+辅助1核；不新建/停止原进程。

## 一分钟恢复

1. 工作目录 `/Users/fanchangxuan/.codex/worktrees/relay-continuation/roomscript2`，
   分支 `codex/relay-continuation`；读本文件，再读旧 continuation 获取更早背景。
2. `cat /tmp/relay-r16-active-root.txt` 是唯一实际推进的原世界根目录。
3. 确认 PID **52509、52510** 活着、world ID 未变；监督进程52485。
4. 读取各 `attempt-001/recovery/observation.json`、最近原始 `results/*.json`，
   核对 requests/accepted 中没有未决物理动作，再规划。**不得重放已存在ID。**
5. 最新精确快照：`runs/engineering/relay-descent-20261026/owner-frontier-current.json`。
   文件快照也会过期，原世界最新不可变反馈优先。
6. 目前 **7/9项、23/27根** 按用户标准完成。剩4根：16L2的P1/P2/P3、16L3的P1。
   恢复后继续调度，不只回答“可以继续”。

当前完整会话JSONL（需要核对用户原话时按角色/关键词读取，不整篇灌入）：
`/Users/fanchangxuan/.codex/sessions/2026/10/26/rollout-2026-10-26T00-40-13-01a173ef-208f-74d0-ae4c-faf10e1135f1.jsonl`。

## 不可丢的用户要求

- 只完成 Oracle 九项（8/12/16臂 × L1/L2/L3），不运行GPT策略。
- 没有总时间期限；不因慢或超时封存未完成任务。
- 继续原来的物理进程/现场，禁止重跑前缀、状态回滚、活世界qpos写入、焊接。
- 最终真实松爪、落入白色Goal，**边界统一允许50mm**。不要求速度/静止时长，
  不要求木条或砝码先碰桌才松爪；允许Goal内木杆接触、碰撞、稳定叠放。
- 仍保留真实合格空中交接、实际动作和失败历史。不能伪造旧严格native通过。
- 每项最多2个计算核心：原物理主进程1 + 最多1个计算辅助进程。
  `diagnostic_world` 的 `reserve_helper_core` 用 flock 限制辅助槽。
  不要持同一槽后启动子进程并等待它再获取该槽；需要串联用同进程runpy。
- 用户反复要求提高速度。不要长时间分析而让已完成动作无人接续；及时收取反馈，
  批量执行已明确的有限动作，只在新障碍处返回判断。不同case可以并行。
- 不启动subagent，除非用户或适用指令明确要求；当前开发指令禁止主动委派。

## 世界与进程

根目录（runs实际共享到主仓库）：
`runs/oracle/relay-r16-38b5423-20261026-171651`

| 原世界 | PID | world_id |
| --- | --- | --- |
| relay-a16-d2 | 52509 | 9ecf541b82b9407988a5aeb2a58db55f |
| relay-a16-d3 | 52510 | be2204c7cc834e2792271a959a435d9e |

其余已完成原case已按用户验收关闭并完整保存。**不要恢复重复世界**：
R17监督74995/子75025–75030、重复8L2 PID66700、旧55214/99193都保留SIGSTOP。
它们不是当前有效进度。原L1证据沿用，几何未变。

## 完成情况与视频

- 三个L1全部完成：8L1 2/2、12L1 3/3、16L1 4/4。
- 原8L2、8L3各2/2，原12L2、12L3各3/3，均按用户最终Goal标准关闭。
- 原12L2最后P1约35.9mm越线，**用户50mm内允许，禁止再追修它**。
- 原12L2完整录像472,741帧；原12L3 1,094,707帧，全部逐块checksum/解码/时间核对。
  精确最终event状态另验；最后event比60Hz样本最多一个采样间隔，不能伪造stream状态。
- `USER-GOAL-COMPLETE.json`、`user-goal-report.json`、
  `RECORDING-VERIFIED-USER-GOAL.json` 是独立验收/完整性证据。
- 严格native task/stream可能仍为failed，反映旧规则/STOP/历史保护错误，不能改写成成功。
- 视频绝对目录：
  `/Users/fanchangxuan/Repos/roomscript2/outputs/oracle/relay-user-completed-20261026/`
  各 `relay-a08-d2-full/global.mp4`、`relay-a08-d3-full/global.mp4`、
  `relay-a12-d2-full/global.mp4`、`relay-a12-d3-full/global.mp4`。
  8L2 6m13s、8L3 6m36s、12L2约7m01s、12L3约17m31s。4x动作、压缩低运动等待，
  保留全部动作区间、失败及修正。12L3首末画面已视觉核对；12L2视频生成解码通过，
  首末画面仍可再查看。不是新物理模拟。当前renderer从report读取50mm，并附精确final event。

## 原16L2，1/4完成

- P4完成，旧严格框约3mm误差已被用户允许，不追修。
- P3已经Goal内、无夹持力、夹爪80mm打开，但仍搭在P手指/偶发手掌上，
  **尚未计入完成，不能只看“open”就宣布落好**。这是最关键资源P的阻塞。
- P1实际D/M，Root约[-.696,1.516,1.294]，已完成
  `r16-p1-DL-next-positive-Mfreed-v1`，M已从P2释放后真实接过P1。
- P2实际G/O（Top），Root约[1.102,1.300,1.196]，最新普通前移
  `r16-p2-GO-open-H-Top-window-v29` 已完成。接下来H54可能已可达；之前在
  Root[.9,1.4]要求H到达失败是位置问题，不能照搬旧结论。P3解除前不要让P接P2。

## 原16L3，3/4完成

- P2/P3/P4均真实松爪、Goal内，最近重新核验仍3/4。P2当前约[1.610,-.434,.871]，
  P3[1.750,-.546,.836]，P4[1.727,-.635,.842]，允许接触叠放。
- 仅P1在运。实际 **H/N**，H Top（Dot约-.999），N Side（Dot约+.095），
  负侧H约32mm站位写法是`.32m`，正侧N约`.58m`。
- 最新 `r16-p1-HN-receiver-height-room-v38` 已完成纯垂直下降到
  Root约[1.203,1.302,1.158]。保持实际全姿态，最低组件仍高于桌面；之前最低约.961，
  下降后约.886，不需要人为要求Root1.2/1.3。
- H原来有已放下P2的过期持物标签，导致choose_team完全不考虑H。
  已用正常typed `move_empty` 当前关节位置的空手保持反馈清除：
  `r16-p2-H-empty-stage-feedback-v56`。物理无接触/无力/有桌面支持的真空手，
  不是直接篡改cache。随后 `r16-p1-GN-to-HN-Hfreed-v35`真实交接完成。
- 当前剩余O Top接手问题：旧N是Side，禁止把`require_topdown=True`错误用于
  保留N的负侧换手。H已是Top，**保留H、O新接手时**可以要求两者Top。
- 最后一次有限搜索（compact前）`r16-p1-HN-to-HO-lowered-Top-v39`：
  私有候选offset [.32,.54]、[.32,.44]、[.32,.32]、[.32,.24]，
  全部未找到候选，**没有提交物理v39请求**。重新读取结果确认。
- 重点检查O endpoint、100/150mm preapproach能否达到以及N forearm/O link2，
  不要只反复扩大搜索。Root纯下降已解决部分preapproach高度问题。
- 旧H/N前移到[1.6,1.15,1.2]并改变yaw2.2在N立即IK失败；保持实际姿态的
  纯下降1.16已通过。N侧姿态/关节分支是局部限制，不是整个机械臂任务死锁。
- 一旦H/O真正都Top，可用已验证过的临近段继续I/O、I/P、J/P。
  杆端会扫过固定底座，提前用有限XY/yaw waypoint避开；不可按索引强行直走。

## 已获用户批准并已经完成的原进程热加载（重要）

用户问“热更新是否暂停”，已如实说明加载会短暂停顿、仿真时钟也暂停，
同一进程继续；随后用户明确：**“如果不是重新开始，而是能够继续，那完全没有问题”**。
这已经授权本次原进程控制逻辑热加载，不要再次当作未获批准。

- 只对原16L2 PID52509加载了**v1**，没有重启/恢复物理状态。
- 冻结源：原16L2 `recovery/controller-extensions/open-contact-withdraw-v1.py`。
- SHA256 `11c63d5f1d3e5f3e011e3a8491013740d2cb1f1f066cc3ea72719f813a57cd80`。
- `open-contact-withdraw-v1-loaded.json` 已确认 installed=true、相同PID/world ID。
- 加载方式：LLDB attach，调度 `Py_AddPendingCall`，回到安全Python边界才运行
  code-only callback，再detach。没有在调试器里发动作，没有写live qpos/物理/monitor。
  **目前没有调试器挂着，进程已恢复正常运行。**
- 可复核 loader：`runs/engineering/relay-descent-20261026/relay_hotload_loader.py`。
  已加载ID不能盲目再次排队；先查原ack与queue-result。缓冲区约2KB，无物理状态含义。
- 新增逻辑仅处理：已张开、无力、Goal有真实地面支持、初始仍接触的短距离撤离。
  private FK不把旧held标签当作物理焊接，保留真实80mm开口；只许已有接触不加深、
  无新穿透、无机器人/桌面碰撞。真实动作仍`t.execute`，原速度/关节/记录约束保留。
- 原非接触 `move_empty` 路径完全不变。实际动作后仍有接触则保留原failed反馈、
  保留标签，**不会假称成功**；真的完全分离后才按原规则清cache。
- 代码：`robofrontier/suite/relay_contact_escape.py`。3个单测 +4 empty recovery +
  14 live recovery测试通过，并用真实保存现场81点检查及mock actuator调度核对。
- 曾准备支持大角度转动的**实验v2**，所有转动候选未通过完整碰撞检查，
  **从未加载或执行**。已归档 `open-contact-withdraw-v2-uninstalled.py/.json`。
  tracked源码现已恢复为与已加载v1相同的SHA。不要把v2当成已生效。

## P3撤离已经真实执行了什么，别重复

目标是原16L2 P3，机器人P，始终force-free/open，现仍有接触。
1. `r16-p3-P-open-contact-withdraw-v16`：约-20mm X/-5mm Z，真实移动，
   物体跟随手指，后置分离未通过。world/PID/历史不变，没有碰撞violations。
2. `r16-p3-P-progressive-open-exit-v17-1..4`：又做了4段短移，均真实进展，
   全部反馈“moved but cargo contact remains”。脚本随后在没有新可用路径时停止。
3. `r16-p3-P-lower-open-exit-v19-1`：Y约-10mm、Z约-25mm，真实移动，仍有接触。
   第2段没有可验证路径，**没有提交**。不要重放任何上述ID。
4. 目前P3 Root约[1.694,-.486,.992]；准确位置以最新observation为准。
   P_tool也已离开最初的位置，旧80mm长直线IK结论不再适用。

失败报告在这里指“移动已完成但仍有接触”，不是仿真结束，也不是物体瞬移。
持续friction支持意味着固定木杆FK预测分离不保证真实木杆不跟随。
下一步应按最新接触面/腕部姿态设计退出（例如张爪换成Top方向，或让桌面接过负载），
别继续重复同一抓取RRT。旧抓取计划把80mm开口缩成65mm、产生固定木杆起点穿透，
这正是之前大量搜索无效的原因。

## 接口/证据/恢复

- Python `.venv/bin/python`；环境
  `PYTHONPATH=.:runs/engineering/relay-descent-20261026 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1`。
- 当前venv没有pytest；相关测试用 `python -m unittest discover -s tests -p 'test_relay_*.py'`，
  不要为测试误装依赖或重建环境。
- `owner_client.send(run,request)`：唯一原世界typed请求；唯一ID，不可重发。
  异常可能发生在真实动作后，先看原result +新observation，不能当作“完全没动”。
- `diagnostic_world(run)` 是只读FK，最多用最新完整motion chunk补weight/hinge，
  不是新物理仿真。不要从它的零qvel/默认force推断真实稳定或持物；用原observation。
- `scripts/check_relay_user_goal.py`独立50mm验收，真实release+ground+qualified transfer，
  不要求速度/静止；保持原strict reports/counters。后来接触会移动已放下物体，需复验。
- `user_goal_lower_open.py`：真实Goal内小步下降、实际支持后正常release/open。
  对16L2 P1、16L3 P2已有效；release失败可能**已经张爪且最终完成**，要另验。
- `original_recovery_lift.py`：仅原custody双实际持物，24mm真实抬升恢复旧接触保护，
  不清历史drop计数，不改flag；原状态若仍grounded不可让foreign接手。
- `finite_top_route.py` /`finite_owner_sequence.py` 只执行明确有限前缀，失败回模型判断。
  `16l3-p1-top-roadmap.json`的v14已执行，v15未发；后续H/N为新实际前沿，
  **不得从这个旧roadmap开头重跑**。
- 查看大JSONL只读尾部或指定ID，别打印全部stream manifest/图片base64。

## 场景几何（用户最终版）

L1完全不变；8/12臂L2/L3初始端点直线跨度1.15m，16臂1.20m；
16臂径向两排距离1.40m。L2圆弧中心角63°；L3折转31.5°、内角148.5°，
弹性关节只作垂直于初始杆面的弯折（横向轴）， nominal±25°、刚度18/阻尼.55。
木条底面、距末端100mm直接铁环连接，无短柱；同款双环+170mm木条+600g砝码。
Goal0.80×1.36m；木条截面45mm。当前R16所有世界已经是这版，不重建。

## 最后检查/对用户沟通

用户很不满意重复计算和等待，应明确承担诊断顺序/控制状态设计的问题，勿归咎于
“任务本来就很难”。下一步以直接动作和可验证物理反馈为主，少做泛泛解释。
保持约60秒内有有意义进展更新。正常权限/用户已授权的热更新不重复询问。
当前用户要compact，两个后台原物理进程继续；恢复模型先读本文件和新原result。
不要给进程发STOP/SIGTERM，不要把未完成任务标成complete/paused。
