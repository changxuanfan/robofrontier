# 可见 Codex 双会话通信预实验

本地标准库实现，运行在 loopback；不连接 MuJoCo，不调用独立模型 API。
真实模型推理由 Codex 桌面项目中的两个全新任务执行，各固定为 `gpt-6-luna` / `xhigh`。

后续树状 Meta–Worker 与分布式 Worker–Worker 的设计规范见 [当前通信协议](../../.agents/skills/robot-collaboration/references/communication.md)。当前协议已更新；不能据此认为本历史预实验具备这些能力。

## 本轮范围

验证 READY/START 门、A 发消息时 B 的后台工作继续推进、B 实际处理并回复、B 的 Codex 轮次真正结束后由邮箱消息触发后续轮次，以及统一时间线。

**当前研发会话是驱动器**：它通过 Codex 应用接口创建任务，读取实际平台状态，并在确认邮箱存在未处理消息且 B 已 idle 后调用应用的续发消息接口。它不替 A/B 发送业务消息或决定回复内容。
因此这是已连接桌面任务的受控演示，不是已经接通桌面后端的无人值守文件监听守护程序。文件本身不会唤醒已经结束轮次的模型。

## 启动本地邮箱与总览

```sh
python3 scripts/session_mailbox_demo.py serve --run-dir runs/session-mailbox-NEW
```

输出 localhost URL，打开可看实时 A/B 状态、独立任务计时、原始消息、实际模型验证结果和完整事件。
本次创建的真实任务及实际设置以对应 run 的 `state.json` / `report.md` 为准。

每次运行使用新目录。公共规则和角色提示放在 `public/common_rules.md` 与 `public/roles/robot_A.md`、`robot_B.md`。桌面会话首次收到这些明确文件路径，不 fork 研发历史。

客户端示例：

```sh
python3 scripts/session_mailbox_demo.py call --run-dir runs/session-mailbox-NEW --agent A --op ready
python3 scripts/session_mailbox_demo.py call --run-dir runs/session-mailbox-NEW --agent A --op watch --data '{"condition":"start","timeout":15}'
python3 scripts/session_mailbox_demo.py call --run-dir runs/session-mailbox-NEW --agent A --op send --data '{"to":"B","key":"example","text":"请确认工单X7的处理顺序"}'
python3 scripts/session_mailbox_demo.py call --run-dir runs/session-mailbox-NEW --agent B --op inbox
```

## 两阶段业务流程

1. A/B 各 READY，研发驱动核对真实 turn_context 中的 model/effort 后发布 START。
2. B 启动150秒独立后台计时任务；调用立即返回，计时不等模型。
3. A 确认工作正在运行后发送 `during-work`；B 自己决定文字回复，发 `during-work-ack`，并标记原消息已处理；A 收到后确认处理。
4. B 任务自然完成，发送 final `B_IDLE_WAIT` 并结束轮次；驱动从 Codex 平台状态确认 idle，记录证据。
5. 驱动恢复 A 的同一会话，请 A 发送 `idle-wake` 后结束轮次。
6. 驱动确认邮箱事件后向原 B 会话发送“邮箱有新消息”的通知。B 恢复后必须实际读取邮箱，回复 `idle-wake-ack`。
7. A 恢复并确认收到回复，驱动执行 `verify`。

## 消息与并发语义

- 每条消息拥有独立 ID、发送者、收件人、业务 key、可选 reply_to 与处理状态。
- 读取与处理分开：inbox 读取不会丢消息；只有收件人的 handled 才确认处理。
- 同发送者同 key 同内容重试不会重复发；同 key 不同内容报错。
- 所有请求由一个 mailbox 服务串行提交状态，避免两个模型并发改写同一个文件。
- READY/START 及后台工作、群消息、agent 自报阶段和平台确认事件分别记录，不能把自报等待当作真实平台 idle。
- 业务消息只通过 mailbox；实验会话不直接调用 Codex 跨任务发送工具，不创建其他 agents。

## 证据

- `events.jsonl`：顺序编号、UTC 精确时间、事件来源与详情。
- `messages.jsonl`：原始发送记录；处理状态在 events 与 state 中保留。
- `state.json`：运行状态和 thread ID、请求/实际模型设置。
- `verification.json`：时序和生命周期验收。
- `report.md`：完整可读时间线。
- `report.html`：验收时生成的独立静态总览，即使服务器停止仍可查看。

测试：

```sh
python3 -m unittest discover -s scripts -p 'test_session_mailbox_demo.py' -v
```

本轮不验证机器人动作或严格权限隔离。后续接入机械臂时应把模型给定的初始信息、后续RGB/本体观测、任何额外状态反馈分别列在实验记录中。
