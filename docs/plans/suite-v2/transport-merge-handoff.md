# Transport 合并交接

Transport 的当前默认是 R14 真实半孔。当前规格、入口和证据边界见
[任务说明](../../tasks/transport.md)，GPT参考见 `prompts/tasks/transport.md`。
本worktree的家族准备与整理完成；公共集成、正式R14资格和联合工厂绘制待主session处理。

## 合并内容与所有权

| 内容 | 权威入口 |
| --- | --- |
| 当前场景／验收／控制绑定 | `configs/task_families/transport/half-opening-r14.json` |
| 当前九项catalog | `configs/suites/frontier-transport-r14.json` |
| 新场景元数据 | `configs/environments/cube_transport/frontier-v2-r14-transport-*` |
| 原R13九项证据 | `configs/task_families/transport/r13-qualified.json` |
| 持久交付路径／SHA／播放终止规则 | `configs/task_families/transport/delivery.json` |
| 公开输入与资产物化 | `robofrontier/suite/transport_policy_package.py` |
| 源原生剪辑与信号函数 | `robofrontier/suite/transport_motion_edit.py`、`transport_motion_signals.py` |
| 构建／生成／核验 | `scripts/build_transport_policy_package.py`、`edit_transport_recordings.py`、`check_transport_family.py` |
| 最终任务参考 | `prompts/tasks/transport.md` |

本分支也包含已完成Oracle期间的并发调度、抓取、恢复、碰撞筛查和媒体改动。
Transport的 `transport_*` 模块及测试应合并；共享文件须逐项整合，保留其他家族能力。
当前公共Skill、公共角色Prompt及全局catalog没有接入本类新任务参考。

需单独审查的共享改动：

- `suite/oracle.py`、`suite/oracle_budget.py`：参考执行、事件／暂停与预算记录。
- `motion_recording.py`、`motion_timeline.py`：选择性读取、旧交付时间线与默认引用。
- `factory_workstations.py`：冻结源覆盖、半孔展示、刚体工位变换。
- `render_frontier_oracle.py`、`replay_motion_in_factory.py`：20x／30fps及旧交付剪辑消费。
- `build_factory_workstations.py`、`export_factory_recorded_pose.py`、`render_factory_blender.py`：来源与几何展示传递。

以上共享代码在本次整理中保持逐字节不变，仍有下面C1—C5待办。不能以单一分支覆盖冲突。
不要将旧catalog批量改名，也不要改写原始报告的场景、控制或裁判SHA。

## 持久数据与合并步骤

本机持久数据根为 `/Users/fanchangxuan/Repos/roomscript2`；本worktree中的
`runs/`、`outputs/` 是链接。它们及场景 `assets/`、预览、依赖被Git忽略。
**Git合并不会迁移这些数据。**

1. 合并本分支的源码、配置、文档和测试；保留主仓库持久 `runs/`、`outputs/`。
2. 按 `delivery.json` 核对报告、stream、剪辑数组与视频SHA。整个Transport批次应保留，
   包含未选中的尝试、冻结代码、动作／过程／恢复日志、单位验证及诊断证据。
3. 在合并checkout运行 `build_transport_policy_package.py --assets-only`。
   它从绑定的原始见证恢复新R14资产，验证既有资产，保持独立inode，不需要旧worktree。
4. 需要重建公开包时使用新的 `--output`，不要覆盖已冻结的release包。
5. 完成C1—C5及验证后再根据用户指令启动正式GPT。

本类打包／剪辑使用仓库Python依赖。历史Oracle执行器另需
`requirements-oracle-transport.txt` 的greenlet；旧 `.oracle-deps/` 属本机依赖，
不应作为源码合并或新GPT默认控制来源。R13/R11/R12等冻结清单保留用于历史复现，
当前任务入口只选择R14。

## C1：公开输入加载／冻结

公共run_setup应通过统一family provider加载本类公开JSON与 `transport.md` 并冻结hash。
public包含桌面真实支撑、孔、范围、订单、机构、相机安装标定和完成契约。
Oracle索引、轨迹、工程诊断与恢复配方保持私有。通用Skill只添加一次按任务引用读取家族
参考的机制，具体规则由本类Prompt提供。

## C2：正式资格与源绑定

原R13九项独立完整见证216/216，三项程序、六项工程辅助。R14仅有编译、模型一致性和
六项轨迹1,998,263状态的新增桌面材料零接触检查；未执行新的完整物理试验。
保留preview门禁和 `formal_gpt_ready=false`，由合并负责人建立正式资格路径。

控制和裁判变更须记录新旧来源并验证。模型、effort、拓扑和预算使用正式共同配置，
不继承旧1200s probe或Oracle预算。来源一致性检查不能只比case_id。

## C3：剪辑适配

当前源原生格式是 `robofrontier.transport-source-edit.v1`，producer为transport。
每项JSON绑定原stream／场景／动作／过程／数组hash；`selection.npz` 保存原时刻和bool keep。
`TransportEdit.at_video_time(T,speed=20)` 返回原帧、原时间、有效时间与ended。

公共层须显式适配实际结构。Transport旧 `robofrontier.motion-edit.v1` 和其他worktree的
同名格式可能不同，不能仅凭schema字符串混读。背景、材质和机位变化复用同一源选择。
既有生成器快照和hash保持原样；本次只移动信号函数，未重算或替换任何已交付选择。

## C4：多桌工厂与终止

各桌有独立去静止时钟，保留桌内机器人、夹爪、物体、机构及外观事件同步。
同一工厂帧可以对应各桌不同原始时间；全部mocap／外观更新后只做一次全厂FK／绘图。
工厂 `data.time` 使用展示时间，不能被最后更新的一桌源时间覆盖。

**每桌跑完后保持精确终态；全部桌子跑完后结束，不循环。**
Transport为20倍速、30fps；其他家族倍速由其交付决定。每帧应可追溯各桌源run、帧号、
源时间、有效时间、edit hash与完成状态。验收先两桌错位等待／不同长度，再九桌及全工厂。

原Oracle使用冻结R13做FK，展示半孔相对R13为0.5；未来GPT使用物理R14，相对R14为1。
共享渲染器目前仍有0.5默认，合并时按来源消费家族元数据，避免R14变成四分之一孔。
既有单桌映射／两帧工厂预览不能当作全工厂联合视频。

## C5：合并验收

检查真实冻结public输入与READY能力、并发、录制、未知动作、超时及失败保留。
合并前应重取其他worktree当前提交和未提交文件，不能沿用历史文档中的旧HEAD。
工厂美化、相机Demo编排、Robo Factory墙字及Codex上下文设置由主session统一处理。

## 整理记录与核验

整理审计目录为持久批次的 `engineering/transport-cleanup-20261027/`：

- `source-before/` 与 `legacy-media/`：整理前源码及一次性旧媒体脚本，供追溯。
- `protected-before.json.gz`：原始实验、最终剪辑／视频和场景的逐文件hash。
- `shared-before.json`：共享代码基线；`removed.json`／`deletion-candidates.json`：删除明细。
- `verification.json`、`tests.log`、`family-audit.json`：整理后的验证。
- `merge.patch.gz` 与 `source-only-apply.json`：本分支最终差异及向主仓库的独立应用检查。

9份历史计划已移到现有 `docs/history/`，活动说明改为最终状态；一次性旧剪辑／发布脚本
从活动scripts移除，所用信号／画面核验函数保留在本类模块。7个被替代派生草稿目录删除，
原始实验、最终媒体、半孔审计、旧视频对应的audit-v4与时间线、源原生v2均保留。
根目录的一次性交接指针已移除；当前交接统一使用本文，批次内保留紧凑恢复入口。

当前原生剪辑为1,096,132保留状态、1,035,753跳过状态。既有20x视频仍使用原审核剪辑，
两者身份由delivery索引与对应manifest区分；新源剪辑尚未批量重渲。
