# Meta 操作说明：当前 Harness

Meta 是当前 research 会话，只做工程管理和固定调度。任务团队由配置选择 Coordinator＋Workers、Peer、Single 或 adaptive Controller＋临时分身。目录、持久化归属和上下文规则见 [架构说明](architecture.md)；任务通信和工具协议只维护在 [Skill](../.agents/skills/robot-collaboration/SKILL.md) 的参考文件中。

正式试验的初始策略角色使用干净的新会话，不 fork 含工程 Oracle 解法或历史试验答案的 research 会话。Adaptive 运行期分身仅继承本次干净 Controller 的历史；经验跨运行复用须按实验协议声明。公共输入不复制工程参考脚本、成功抓点、私有过程日志或 Oracle 路线。

## 工程准备

```sh
uv run python scripts/create_pipeline_run.py runs/NEW --task configs/tasks/three-arm-v1.json
uv run python scripts/check_harness.py --report runs/engineering-check.json
```

准备只创建格式 v3 的文件与数据库，拒绝覆盖已有目录；不启动 MuJoCo、会话、录像或动作。角色指令位于 public/inputs/roles，每个任务 Agent 读取 common、task、自己的角色和 Skill。Meta 使用 private/inputs/meta-runbook 与自己的角色说明，不需要维护任务解法。

旧 run 和旧 prepared 包不可由当前服务自动转换；需要复查历史时使用该 run 的原始记录与冻结工具。当前格式、输入来源由 manifest 确认。

工程验收用于开发或相关接口变更后的故障排查，不是每次任务前替 Agent 试出抓放参数的必经阶段。使用仿真物体真值计算夹持偏移或参考动作的小测须单独标为 oracle 工程证据，不能作为模型成绩，也不把其状态派生参数、轨迹或报告导入策略输入。公开机器人配置和已声明的标定信息依任务契约提供。正式 Agent 的试探依允许的图像/本体反馈进行，时间、动作与失败计入任务；若另设学习阶段，提前声明预算与经验保留方式。运行期经验的维护、引用和交接以 Skill 为准，Meta 不替策略总结工程真值为经验。

## 获得实验授权后的启动

1. 使用新 run。显式启动 `uv run python -m robofrontier.pipeline --run-dir runs/NEW`，一个服务拥有一个 MuJoCo 世界。启动前校验输入和源码；准备时没有世界。
2. 创建授权的独立 Coordinator/Worker 会话，使用本次角色文件，READY 后 yield。模型按配置请求，实际设置另外核实；requested 不是 verified。
3. 用 Meta 连接的 bind 操作绑定真实 thread ID，启动下面的固定循环。全部 READY、已绑定、平台状态新鲜且 idle 才 START；服务再次校验。START 给所有任务角色中性继续状态，不分配动作或任务顺序。
4. 循环只按已记录条件通知，不解释业务内容，不分配任务或操作机械臂。停止条件为结束、中断、暂停、故障或明确停止请求。

在 Meta 的 functions.exec 单元中装载冻结调度器，不能从终端直接运行：

```javascript
// @exec: {"yield_time_ms": 1000, "max_output_tokens": 2000}
const quote=s=>"'"+s.replaceAll("'",String.fromCharCode(39,92,39,39))+"'";
const source=await tools.exec_command({cmd:"cat "+quote(RUN_DIRECTORY+"/private/inputs/frozen-source/scripts/meta_dispatcher.js"),workdir:REPOSITORY,max_output_tokens:12000});
if(source.exit_code!==0)throw new Error("dispatcher unavailable");
const dispatcher=eval(source.output+"\nrunMetaDispatcher;");
await dispatcher({runDir:RUN_DIRECTORY,python:PYTHON,cwd:REPOSITORY,tools,startWhenReady:true,
  onEvent:e=>{if(e.event!=="cycle"||e.cycle%15===0)notify(e);}});
```

路径取本次准备目录的绝对路径。仅工程检查时不要装载真实 tools；测试使用 fake adapters。工具返回 Script running 后，通过 functions.wait 读取输出，同一循环继续运行。当前 Meta 工具单元结束会停止调度，未实现独立接管桌面会话的守护程序。

运行期间保持 research 管理轮次有效，通过 commentary 汇报并继续等待/驱动；只在运行结束、明确暂停或故障停报后最终回复。`Script running` 只说明工具已让出，不证明 research 最终回复后仍能调用原生平台。不要据此宣称独立后台常驻。启动小验收必须覆盖分身先交回、另一个仍工作、Controller 空闲后续轮及漏交回收尾，不先跑完整密集任务。

## 固定调度与故障

- active 会话不续轮，不 steer、不取消工具。任务角色的工具结果附带有界收件；START 后按当前模式选择工作边界。Single/Adaptive Controller、Coordinator 和常驻 Worker 可持续工作、wait_events 或保存后结束本轮；Adaptive 分身正常执行有边界的委派后 return_task，主动保存的定时等待只是可选延续。Agent 选择等待总期限，客户端用原编号处理 HTTP 分片，分片不强制业务超时；Codex 工具宿主仍按自身机制让出和续等。
- 项目 API 返回自带收件；本地 Python/图像分析、进程查询或原生只读工具须经冻结 helper 的 roleStep，在所选操作结束后刷新一次自己的收件。它不改计算结果、不重放动作、不打断原生轮次。直接使用裸 Codex 工具没有项目收件 hook，不能宣称所有原生工具自动注入；审计要检查角色是否遵循统一入口。
- 原生会话 idle 后，发给本角色的消息、系统事件、到期的自选检查或保存的 continuing/working 工作可以触发一次合并通知；正常非阻塞动作仍在运行不单独阻止续轮。阻塞动作及未知结果仍需原有结果/恢复处理，同臂动作互斥保持。
- 自报 available 产生 Coordinator 状态事件；工具完成、会话 idle、自报可用与真实任务成功分别记录。
- 发送前复核状态，claim 通知并去重。通知不含任务解法；收件和启动新轮均不表示业务接受。
- 发送结果未知时不盲目重发；核对真实会话及 wake_id。现有小型 lease 防止误开两个管理循环。
- 原生检查与发送不是原子接口，实验要求独占调度；外部输入应记录为干预并暂停核实。

## 状态与结束

SQLite 是外部运行状态的唯一权威来源；工作笔记通过 work/checkpoint/yield 更新，checkpoint 只指工作笔记。客户端保存请求和 call_id；超时后 pending／原 ID 查询，不重建新动作请求。

输入在启动、START、finish/关闭时校验；运行中按 harness 的 integrity_check_interval_s 低频检查。发现变化标记无效、暂停调度并终止已知动作。平台状态轮询周期与模型自选检查期限分开。

正常 finish 保存私有评估、关闭录像并导出日志；工程关闭保存中断状态和已有记录。物理进程丢失后不重建同一 run 的世界；未终结动作记为 uncertain，本次中断。聊天/工作笔记恢复与物理恢复分开，当前没有 --resume 或周期世界检查点。

结束本轮或修改环境前，先通知任务角色暂停并保存，再调用现有管理 CLI 的 `seal`：

```sh
.venv/bin/python -m robofrontier.runtime_cli --run-dir runs/EXISTING seal --data '{"timeout_s":30}'
```

seal 暂停新增动作，等待已有执行结算，通过原服务获取最后图像并请求关闭，再用独占 world.lock 核实物理进程已退出。有未解决执行或进程未退出则报告失败并保持暂停，不强制杀进程、不重发运动。若物理进程已丢失，只封存已有数据库和日志，把未结算动作标记 uncertain，不构造世界或补造最终评估。正常 finish 的结果保持原样。仅临时暂停并准备继续同一物理世界时不要调用 seal。

录像使用分片 MP4，正常关闭编码器后验证头信息与首帧可解码；异常退出可能保留已完成片段并丢失尾部。seal.json 分开记录封存状态、是否获得新鲜末帧、未知动作和录像可读性；文件存在不代表录像有效，首帧检查也不保证每一帧完好。工程等待期限不限制 Agent 的任务规划或动作时长。

管理端可用 `python -m robofrontier.runtime_cli --run-dir RUN export --data '{"path":"ABSOLUTE_OUTPUT.jsonl"}'` 显式导出已提交事件。工程检查、验收限制见 [验证范围](harness-v2-verification.md)。

## 按需 fork 模式

配置 topology=adaptive 时，启动只有 Controller 的新运行。Controller 经 delegate 提交有边界的工作范围；Meta 不选择范围、机械臂或顺序。Codex 引擎的分发器先从 Controller 创建不执行任务的快照会话，再从该快照创建整批分身。快照 ID 在读取原生 lineage 前持久保存；验证 Controller→快照及快照→各分身的 parent/ordinal/hash，确认所有分身同一边界后才授权。Controller 指定的各自范围仍由宿主分别发送，首次输入显式固定本轮 model/effort；原生 fork 默认档位不能当作继承证据。Claude 通用 `delegate-bind` 入口继续禁用；Claude 自有的实验性 adaptive 主机独立创建无工具快照并核对两段原生历史及快照完整性，再创建和授权分身。Claude 快照需要一次模型调用，并计入共享预算；创建结果不确定、来源不符或快照被更改时暂停，不能自动重试。该流程尚未取得正式计分资格。

初始任务会话用Main/Coordinator/Worker加运行标识命名。分发器在分身身份确认、首次指令投递后，尽力设为Fork [所控机械臂]+短身份+运行标识；多臂范围自然显示A+B等。命名只是界面元数据，工具缺失、错误或超过短期限只记录，不暂停任务、不改变控制权，也不重试创建会话。

SQLite 保存委派、Codex 快照和分身身份与机器人控制权，不新增数据库。连接为运行时状态；原始 roles.json 仅描述初始角色。主会话等待 delegate 工具结果期间原生历史仍可能变化；Codex 路径用快照固定同批来源，不等待主会话 idle，Claude 实验路径仍使用独立的父历史检查。物理继续，建立并发送分身任务后即返回，完成事件按到达交付。分身 return_task 交回范围，团队 finish 仍只由 Controller 提出。

原生创建或投递结果未知时标记 needs_review，不自动 refork 或释放所有权；已知分身 ID 与原生 lineage 用于人工/工程核实。共享工作区和可读连接仍不是 OS 安全隔离；实验角色必须遵守当前连接与证据边界，不能声称防御恶意凭据冒用。

## Adaptive 异常边界与结果

分身以当前运行时assignment替代继承的父身份，return_task先原子写入public/results中的报告，再提交归还与小通知；父Controller按需读取。写入失败不交还、不通知。返回后移出活动原生查询，不把历史分身数量累积成每轮轮询开销。

每轮实际 wait_threads 观测导入后，活动分身若已 idle/error/needsAttention 且尚未 return_task，按分身/turn ID 去重给 Controller 生成停止未交回事件，附保存的工作状态、复查期限及未解决动作数。它是事实通知，不直接判失败、不结束委派或自动释放机器人。父角色在工具边界/事件等待/idle 续轮取得事件，结合记录区分主动等待与异常停轮。明确保存的 waiting/available 期限可在范围有效、会话 idle 且执行状态允许时恢复一次；旧邮件或继承的 continuing 状态不自动恢复异常停轮。Controller 的新定向消息可要求补交收尾或继续同一范围，交回后新委派另建 fork。原生 idle 不是业务完成或路径清空，调度器不替 Agent 选择范围或空间顺序。

业务暂停沿用既有消息与本角色 stop：主 Agent 发请求，分身在消息边界处理，确认当前动作终态后报告实际状态；暂停不交回控制权。主 Agent 不能用当前 stop 接口直接停止分身的动作。该协议不保证即时抢占；系统级工程暂停与业务暂停消息是不同机制。完整约定以 Skill 的 communication 参考为准。

平台 systemError 与最新原生轮次 failed 分开保存。活动分身收到停止事件后的新 Controller 定向消息时，只有新鲜观测确认 failed 且所有物理请求已解决，才允许一次失败会话续接；claim 前再次核实。旧消息、工作状态或自选期限不重试 error。新轮次若仍失败，需新的父消息或工程处理，不循环投递同一请求。单独的 error 标签、缺失轮次终态、active、accepted/running/uncertain 动作均不满足此例外；其他角色的生命周期不变。

Meta state/面板的 dispatcher_health 保存最近成功会话观测、最近检查周期、最近确认投递和脱敏错误详情。时间陈旧就是缺少健康证据，不能由后台单元仍存在推断正常。它不保证独立进程存活，不改变 Agent 的观察/复查期限。状态检查当前按配置周期执行，延迟还包括平台耗时，未承诺零延迟回调。

状态查询每次最多尝试两次；缺测会清除旧idle可信度，禁止依据旧状态续轮。连续三个调度周期缺测则显式暂停调度及新动作，已有动作保留原结果收尾；不猜测idle、不重发fork。新运行可用可选native_setup_timeout_s配置创建期限，默认120秒；这是基础设施故障界限，不限制分身的任务时间。到期保留已知ID和控制权，返回needs_review，禁止自动重建。

fork返回的已知ID先持久记录，历史验证单独进行；只有所有分支验证完成才授权。已返回结果优先于丢失的首次投递回执，迟到回执不退回finished状态，也不恢复已结束分身。普通消息和系统事件统一预算并保留未交付项；完整报告不经通知传输。

模型与effort读取每次run配置，初始分身输入和常规续轮都显式传入；实际turn_context仍需核验。变更模型或源码后准备新run，保留旧冻结包及其记录。
