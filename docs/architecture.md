# 目录、状态与上下文

共享模板和运行代码属于项目；输入、工作记忆和证据属于单次 run。每次实验采用固定版本，不自动把旧 run 的计划或经验注入下一次。

## 项目入口

| 路径 | 职责 |
| --- | --- |
| `prompts/` | 共用规则及 Meta、Coordinator、Worker、Controller 角色模板 |
| `.agents/skills/robot-collaboration/` | 任务 Agent 使用的 Skill；通信和工具参考各只有一个权威版本 |
| `configs/` | 工程与任务配置；单臂练习配置仍可用 |
| `robofrontier/` | 状态存储、固定调度、工具服务、MuJoCo 与 IK 接入 |
| `scripts/` | 准备、管理、离线检查、观察与导出入口；调度器也是执行脚本 |
| `tests/` | 离线测试、FakeWorld、运动学及无窗口接触回归；不接入真实会话或GUI策略试验 |
| `assets/` | 网页与贴图 |
| `docs/` | 当前架构、Meta 操作和验收范围；`history/` 保存旧计划、试点与诊断 |
| `vendor/` | 上游机器人模型与来源 |
| `runs/` | 实验和工程检查产物，名称与报告区分其用途 |

当前任务统一由`configs/task_families/current.json`按GPT/Oracle用途选择，
`scripts.tasks`负责冻结公开材料和源绑定协议；`scripts.factory`负责无物理的
多桌剪辑回放。历史生成入口保留版本兼容。参见[GPT准备](gpt-policy.md)、
[工厂说明](robo-factory.md)和[合并验收](oracle-integration.md)。

## 新运行目录：格式 v3

```text
public/inputs/task.json       固定：允许提供给策略的静态任务
public/inputs/common.md       固定：共用规则
public/inputs/roles.json      固定：角色与通信拓扑
public/inputs/roles/*.md      固定：本次角色指令及本角色连接路径
public/inputs/skill/          固定：本次采用的 Skill 与协议
public/observations/          增长：时间戳 RGB 图像
public/results/              写入：交回报告；可选本角色运行期经验文件，通知仅含引用
public/robot-models/*.json    按需：按模型摘要标识的完整静态机器人描述
private/inputs/task.json      固定：完整场景与任务输入
private/inputs/harness.json   固定：工程参数
private/inputs/meta-runbook.md 固定：管理操作说明
private/inputs/manifest.json  固定：格式、模型请求与输入/源码摘要
private/inputs/frozen-source/ 固定：复现所需代码、模板与 Skill 字节
private/inputs/scene/         固定：生成场景运行的 MJCF、贴图与机器人网格（按需）
private/state/runtime.sqlite 更新：唯一权威运行数据库
private/state/connections/   更新：角色凭证、服务地址
private/state/client-state/  更新：已提交请求、结果和收件回执
private/outputs/              导出：运行摘要、JSONL、完整场景、录像、评估等
```

`public` / `private` 是输入契约分类，不是共享用户文件系统的权限隔离。Worker 只使用自己的连接；工程侧真值不作为策略观测。历史 run 保持原路径和格式；当前工具拒绝自动迁移或重启旧世界。

## 保存什么，谁更新

| 内容 | 写入者 | 更新时机 |
| --- | --- | --- |
| 固定输入与版本快照 | 工程准备程序 | 新建 run；运行中保持固定 |
| 每角色工作笔记、状态、检查期限 | 任务 Agent 通过工具 | 重要决策变化、明确检查期限、结束一轮前 |
| 邮件、交付批次、回执、通知状态 | Harness | 相应事件发生时 |
| 动作编号、开始/终止/未知结果 | 执行服务 | 动作事件发生时 |
| 平台会话状态观察 | Meta 的固定驱动器 | 调度检查时；保存时间戳，不把旧观察当作实时事实 |
| 图片、录像 | 观察和录制服务 | 请求观察／采样时 |
| JSONL、运行摘要 | 数据库导出 | 结束、关闭、故障或明确请求；不逐秒重写 |
| 评估 | 仿真评估器 | 任务团队提出 finish 时；与 Agent 的成功主张分开 |

工作笔记在数据库中，先在同一份 work 中维护计划与本次探索经验，不另建通用 memory/plan/todo 目录。经验较长时，可由该角色自行写入本 run 的 `public/results/<actor>-experience.md`，并在 work 保留引用；这是可选派生产物，不是新的权威状态库或冻结 Skill。记录尝试、允许观测产生的证据、适用条件及不确定性；后续证据可修正旧经验。其他角色按既有通信引用选择性读取，不自动注入所有上下文。Single 保留自己经验，常驻 Worker 继续维护自己的 work，Adaptive 分身随交回报告交接有用经验，主 Agent 决定新 fork 采用哪些。

槽板任务可在冻结 completion 中声明 pocket_seating：私有验收检查颜色、唯一匹配、完整水平投影、槽底承托、基本竖直、释放及稳定时间，并附原严格姿态指标作为诊断。旧任务未声明该模式时继续使用原验收；不向策略工具返回实时物体真值。工程 seal 复用运行数据库和物理 owner 锁，产生 private/outputs/seal.json、末帧与事件导出，不新增状态库或调度角色。

默认不跨 run 自动加载经验。若研究持续学习，应在运行前明确授权保留来源和学习预算；当前没有新增自动跨任务训练/记忆系统。源码、凭证和工程真值日志属于工程资料，不是 Agent 在线学到的经验。数据库副本、JSONL 与录像不能彼此充当恢复物理世界的依据。

## 上下文读取

- 首次：读本角色、任务、共享规则、Skill 和工具参考；注册 READY。其 robot_facts 从实际机器人模型和控制配置生成，只返回紧凑的固定基座、TCP 指令界限与接口能力；各策略角色使用同一摘要。详细几何按需 robot_model，实时本体状态仍用 observe。摘要不是全臂无碰撞区域、当前占用或任务分工。
- 恢复：读自己的 work 与 inbox；必要时查询原动作、相关历史或图像。Coordinator 可读 team 的自报状态，但需区分声明与事实。
- 执行：Coordinator 和 Worker 都在工具返回时接收有界消息批次，可在同一原生轮次持续决策。按决策需要选择视图，不强制每步看全部相机。
- 边界覆盖：robotTool/robotSequence 的项目 API 自带收件；其他本地分析、图像/进程操作和允许的原生只读操作用 roleStep 包裹，在所选操作结束后刷新一次收件。裸原生工具没有项目消息 hook；这不是修改 Codex 的全局工具宿主。
- 动作期限：按请求的距离/角度/开合量和速率计算有界预算，接受时记录；超时先受控停止，停止无法确认则进入现有 uncertain/工程恢复机制，不释放执行占用。网络等待不是物理取消。
- 保存：工作笔记保留当前计划、承诺、证据引用、阻塞和下一决策；材料变化时更新，yield 前保存。完整推理、临时计算与图片解码无需另行持久化。

Codex 管理会话历史和模型上下文；项目只保存外部实验状态。内存请求队列、物理积分状态与模型临时上下文不保证在进程退出后继续存在。运行中断时保留未知动作和已有记录，另开 run，当前不提供完整物理续跑。

正常 finish 保存私有 evaluation-window.json（适配器支持时的有界最终采样窗），供独立复核；中断关闭可保存 interruption-evaluation.json 和对应窗，明确不是成功试验。证据不进入策略 API，不是周期世界检查点。START 记录仿真起始时刻，报告的仿真耗时减去该值，避免把准备阶段算入任务时间。

## 运行中的规划职责

依赖边界、授权和滚动规划的权威指导在共享 Skill；Harness不实现任务规划器。任务角色拥有独立本地执行/准备的常设范围；Coordinator协调联合分工、优先级及资源，Peer由相关Workers直接协商。共享资源/联合承诺未确定时仍须协调，已有明确限制仍有效。工作笔记沿用一份，保存变化后的依赖与下一决策，不记录固定任务解法或强制每步重新全局规划。

centralized 模式只有 Controller 一个任务 Agent，所有物理动作显式选择 robot；没有 Coordinator/Workers，也不限制其跨臂并行。同样使用图像、本体反馈、低层动作及可选动作段。数据库把请求所属角色与物理 robot 分开记录，互斥按物理臂检查。各模式共享控制器、观察限制及动作能力，Meta 均不参与任务决策。

adaptive 模式初始只有 Controller，可通过有边界的委派创建临时分身；当前assignment定义分身身份和控制范围。分身正常执行本次委派后 return_task 并结束原生轮次，不默认成为每臂常驻 Worker。期间必要的动作/消息等待使用现有工具；主动保存期限后结束回复仍是可选延续，不是完成前的必经阶段。完成报告由执行服务落盘，控制权交回与小通知由数据库提交，Controller按需读取；该报告是Agent产物，不是评估真值。协议以Skill的adaptive参考为准。

活动分身的原生轮次停止但未交回时，平台观测触发去重事实事件给 Controller，原有执行占用不变；不存在隐式成功或自动回收。Controller 用 work 的紧凑 delegate_status 或所选分身工作笔记监控，必要时只读检查自己的原生分身会话。分身之间不直接通信；已交回的会话不承接新委派。常驻层级/Peer Workers 的续轮生命周期不变。

原生线程 error 与最新轮次 failed 是不同事实，保存在同一 agent JSON 中。对于已确认 failed、物理请求均已解决的活动分身，新的父定向消息可触发一次续接；缺失终态或旧消息/期限不重试错误。该例外沿用现有 plan/claim、投递结果和去重机制，不回放动作，也不改变控制权。

dispatcher_health 仍存现有 SQLite meta 中，更新成功观测、周期、投递与脱敏错误，不另建监控数据库。UI 展示时间证据；research 轮次/工具单元有效性仍由当前工程宿主保证，不把后台工具显示运行当独立守护服务。

START 为所有初始任务角色登记中性的继续状态，使其能同步开始本角色工作，不由调度器分配动作。之后 Agent 自己选择继续、wait_events 或保存后结束；消息、可用状态、自选期限和明确保存的继续工作为必要续轮提供依据。正常等待不立即续轮；“主动”不是增加固定频率的模型轮询。wait_events 由 Agent 选择总期限（配置范围内），客户端使用原编号处理 HTTP 分片，不重置服务端截止时间；仅事件、总期限或暂停/结束返回业务结果。Codex 工具宿主的让出/续等独立于此。活动会话仍不被普通消息逐条 steer。

活动任务角色若主动选择 yield waiting/available，须填写自行选择的 after_s；这是所选原生轮次结束方式的参数约束，不要求角色每次决策后 yield。READY 前、工程暂停或结束时豁免。常驻 Worker 本地完成后保持团队可用；Adaptive 分身完成委派走 return_task，已有期限随交回清除，旧会话不承接新委派。程序不填默认周期，不解释待办内容、不代替规划。客户端锁只保护请求/回执状态读写，不覆盖网络等待或物理运动时长，以免同一 Controller 的独立工具调用被串行化。

原生 idle 会话可在正常非阻塞动作期间按既有事件/期限/待办条件续轮，以规划或通信；不产生第二条本臂动作，也不唤醒无待办的正常等待。阻塞机器人动作及未知执行结果仍受原有恢复约束。运行通知要求合并新事件并按需恢复上下文，不要求固定调用 inbox 和 work。

机器人静态摘要复用实际动作验证的界限计算，不在 task.json 手写第二套机器人配置，也不新建摘要文件或数据库。配置事实、观测/队友报告与规划假设由 Agent 在已有紧凑工作笔记中区分，不强制每轮填表或复述全部摘要。

工作笔记保存时间/版本与执行器当前或最近动作分别返回；team 不再把一整份原始 agents 记录当作当前工作真相。执行器可更新续轮状态而不改写笔记，旧笔记不会伪装成新保存；动作事实不能推断物体抓取、任务完成或共享区清空。没有新增业务状态机或数据库表。

scripts/role_tools.js 是薄入口，准备时复制并冻结。robotTool 交付一个指定操作的结果及其中已有的选定图像；job 可复用原图像并保留时间，缺图不隐式观察。robotSequence 只执行 Agent 已决定的有限 move/gripper 列表，保留各编号和结果；遇到事件、失败/未知、暂停/结束或需要解释的中间图像返回模型。Agent 选择动作段、边界和是否并行调用，没有隐藏运输/抓取规划或条件队列。

move/gripper 可选 wait:false，动作原编号保持 running，客户端不把它缓存成终态；同臂互斥，其他工具仍可使用。动作完成事件只交付给本角色；Worker 决定哪些业务里程碑应通知队友。等待器在物理线程 tick 检查持久化收件及动作状态，不持有跨 tick 数据库事务。等待器及物理执行不支持跨进程恢复。

区分动作结束、可接新工作、共享资源释放和整个任务完成。robot_model 与 check_motion 是按需分析工具，由 Agent 判断调用价值；move 不要求先检查。检查返回 advisory_only:true，不批准也不拒绝动作。robot_model 导出机器人专用配置；check_motion 在独立数据上复用控制器/IK，采样检查候选运动的跨臂碰撞几何。检查按 tick 分片，继续处理邮件和动作；不集成物理、不复制动态物体状态、不排任务、不执行候选动作。结果带源状态、时域、采样间隔和覆盖缺口；不是连续避碰证书或经验证的运动中清区事件。高级交接重叠不能仅靠 Prompt 或该采样结果保证。finish 后拒绝动作，迟到消息仍记录为收尾审计，不重新启动团队。

## 通用性的边界

共享协作规则不含当前任务的运输解法；机器人数量与任务输入可配置。当前 Panda 适配器支持 TCP 位置／四元数姿态、关节目标、夹爪开度／速率、预期宽度与目标力抓取、同臂受控停止。抓取力控制属于显式驱动模式，反馈与保持语义以 [工具参考](../.agents/skills/robot-collaboration/references/tools.md) 为准；不向策略输出接触真值。原直线任务仍可用；生成场景可带独立基座朝向、相机和共享目标。评估依具体任务定义，不把初始供料颜色当作固定机器人职责。改成导航、异形装配或不同机器人时，需要调整适配器和评估，不能只换一句 goal。

生成场景在准备运行时复制 MJCF、贴图与机器人网格，并纳入输入摘要；从 keyframe 加载后由同一物理服务执行。RGB／机器人本体反馈与私有目标评估分开。控制器按机器人模型补偿重力，保持原执行器力限、摩擦与接触参数；这不提供独立恒力夹持、触觉物体 ID 或规划器。stop 只请求当前所有者的指定动作受控停止，终态沿原动作编号返回，保留已发生的物理运动。


## Shared-payload task and finite batches

Beam handoff uses the same World, runtime, ownership, grasp driver and recording as grid tasks. `beam_task.py` validates static assets and privately evaluates airborne support, overlap-before-release, custody/recovery, inter-arm contact and stable whole-object containment. It derives phases from physics, never from an Agent's phase declaration or the Oracle relay order. Contact/retention is an explicitly declared support proxy, not true per-arm load measurement. The sampler writes a private 10 Hz process JSONL and an incremental summary; cross-arm contact is checked each 2 ms physics step, support/pose each 20 ms. Public task inputs contain declared static facts/rules and criteria; no live object/contact state crosses into observe, READY, robot_model or check_motion.

`action_batch.py` accepts current finite commands: one owner can submit a whole batch, or multiple owners can sign their own parts using the same sync_group ID/roster/deadline. The first deadline never renews. Only submitted robots are reserved; completion triggers full revalidation before the same executor starts all members. Missing signatures, withdrawal, pause, shutdown or failed commit validation cancel unstarted commands. All validate/reserve before any start; the GUI owner starts the group before its next physics advance. Database rollback cannot reverse motor side effects: uncertain startup retains execution/control and marks recovery required. Existing calls/actions store parent and deterministic member IDs; no new database or queue is introduced. A member failure brakes remaining group arm moves, preserving established grasp holds; unfinished gripper commands are individually tracked. It is not a rigid-payload controller, trajectory synchronizer, collision gate or task planner. Hierarchical/peer per-arm ownership remains unchanged; a Worker cannot author another Worker's command. Shared summaries expose other members' execution facts, not commands/mail. Pending members and unresolved signed operations block control handback; unrelated scopes remain available. Agent plans and physical timing remain separate.

Batch query uses persistent member outcomes in both active and terminal states; RPC completion is separate from physical success. Scheduler uses the immutable outer wait flag, not internal member collection flags. Unknown execution blocks automatic continuation. Cancelled pending reservations are not counted as physical action overlap; reports separate submitted and actually started actions.
