# 三臂共享世界：历史 v1 试验

本文记录已完成的旧试验。旧文中的 Meta 是参赛协调者，对应现名 Coordinator；当前实验管理 Meta 与新版 Harness 见 [操作说明](../meta-runbook.md)。

已实现一个 MuJoCo 进程、一个可见 GLFW 窗口、三台 Panda、九个真实接触方块、每臂独立 IK/动作状态、定向树状邮箱、视频与时间线。沿用 Menagerie 执行器、原方块摩擦与 NoSlip=10，不使用吸附或自动 pick/place。

## 复现环境

```sh
uv run python scripts/create_pipeline_run.py runs/pipeline-NEW
uv run python -m robofrontier.pipeline --run-dir runs/pipeline-NEW
```

启动后输出本机 URL，并生成每个角色的 connection 文件。Global、global_side 和每臂前/侧视全部来自同一模型和状态。Meta 不能 move/gripper，Worker 只能操作自身机械臂，消息限定 Meta ↔ Worker。

本次正式目录 `runs/pipeline-meta-20261020-trial01/`；此前 `runs/pipeline-meta-20261020/` 是工程搭建与预览目录，不能混作模型试验。

## 工程验收

```sh
uv run python scripts/verify_pipeline.py
uv run python -m unittest discover -s scripts -p 'test_pipeline.py' -v
```

参考程序只在独立临时世界使用 Oracle 做工程验收：三臂同时真实抓起方块，随后验证 A 放置、B 重新抓取。记录 `runs/pipeline-preflight.json`，不是模型成绩。测试覆盖树状路由限制、Meta 动作权限、批量收件/回复关联、等待唤醒标记，以及不同臂同时接纳动作。

## 实验会话与生命周期

四个真实 Codex 桌面会话位于 roomscript2 项目：一个 Meta、三个 Worker。统一请求 gpt-6-luna / xhigh；实际各轮 turn_context 的验证保存在 private/model-verification.json。

research 会话通过 Codex 应用接口创建会话和检查真实 active/idle 状态。全员 READY 后，research 调用 START 并通知 Meta；Worker 根据 Meta 邮箱任务恢复。

运行中，由 research 会话轮询真实会话状态与服务器邮箱/检查期限：只在 idle 时启动原会话下一轮；活跃 Worker 的结果附带邮箱，Meta 则在下一轮自行读取积累消息。research 发出的原生会话通知只说明邮箱/期限事件，不提供任务分工或运动建议，业务消息由实验 Agent 自己产生。

**当前是由 research 会话执行的受控桌面调度，不是脱离本会话还能自主操控 Codex 的常驻守护程序。** private/driver.jsonl 记录实际状态与唤醒证据。用户可在桌面侧栏查看各独立会话，在本机总览查看邮箱和动作。

## 权限与信息

公开初始物料位置、尺寸、任务目标槽坐标、工作范围；后续工具只返回 RGB、机器人本体状态及执行状态，不返回动态物体真值。Evaluator 在 Meta 提出结束后私下检查六个槽的物体属性、位置和高度；不把真值答案反馈给策略。

消息只需 to/text/可选 reply_to，编号和来源由服务器补充。消息去重与动作编号缓存限于当前进程，尚未实现进程崩溃后的完整恢复。

工程与角色文件在 START 时保存 SHA256，运行期间检测修改并显式中止。身份使用独立令牌；但所有会话仍在同一操作系统用户下，拥有一般文件访问能力。因此这是协议约束与完整性检测，**不是恶意 Agent 的操作系统安全隔离**。当前 Agent 规则要求只读本角色文件、使用本角色工具，不修改工程。

## 记录与评价范围

- `public/`：角色规则、连接配置、实际 RGB 观测。
- `private/manifest.json`：冻结的工程摘要与控制参数。
- `private/episode-start.npz`：实际 START 时刻积分状态。
- `private/events.jsonl`：原始动作、消息和工具结果；所有事件含墙钟/仿真时间。
- `private/runtime.json`：持续更新的运行快照。
- `private/threads.json`、`model-verification.json`、`driver.jsonl`：真实会话设置与调度证据。
- `private/evaluation.json`：结束时评估器结果。
- `global-realtime.mp4`：START 后的全局录像，15 FPS；编码错误和丢帧必须检查。

任务为六个目标槽的抓取摆放，不要求堆叠或方块边缘精确配准。位置验收为中心 XY 各轴偏差 <25 mm、高度偏差 <12 mm，并检查目标属性。最终释放/撤离还需结合图像和执行记录核对；当前槽位检查本身不是完整抓取稳定性评价。

只运行一次试点，不能据此比较架构总体优劣或声称已完成 benchmark。
