# BEST5 旋钮家族：规格、证据与 GPT 输入审查

当前阶段是家族适配与审查，尚未准备或启动 GPT 仿真。正式 readiness、公共载入和
多工位工厂合成仍由合并后的公共实现处理；没有修改原始九项环境或控制配置。

## 入口与九项规格

- 当前任务集：`configs/suites/dial-best5-r11.json`。
- 证据登记：`configs/task_families/dial/best5-r11.json`。
- policy 专属指导：`prompts/tasks/dial-best5.md`。
- 公共合并待办：`configs/task_families/dial/integration-requests.json`。

| 盘数 | L1 机械臂数 | L2 机械臂数 | L3 机械臂数 |
| --- | --- | --- | --- |
| 4 | 4 | 8 | 8 |
| 6 | 6 | 12 | 12 |
| 8 | 8 | 16 | 16 |

全部为 R11 BEST5：盘面六个刻度，密码每位 1..5，0 无效，60 度一格；桌高
0.8m，白盘直径 0.3m。实际桌形、开孔、基座、计分板位置及相机按各 case 的
冻结模型提取。L1 模块固定；L2/L3 的蓝支撑与红操作按独立真实抓持资格验证，
L3 是偏心 crank。精确规则取自该模型的 task，不另手抄一套数值。

BEST 是历史完整有效输入的最大命中数，READY 是当前输入的有效性。稳定的中间
配置也会被自动结算；BEST 不变的一次新有效输入仍是任务进展。裁判的成功锁存、
物理支撑继续存在以及过程违规是否清零是不同概念；最终解锁不会擦除过程违规。

登记文件逐项绑定原 report、motion index、模型/任务/环境 hash、Oracle 执行版本、
控制和裁判指纹。八项程序 Oracle 和八盘 L3 辅助 Oracle 分开保留。登记不是修改
历史 `oracle_verified` 字段，更不表示公共 runtime 已支持正式 GPT readiness。

## 生成与复核审查包

```bash
.venv/bin/python -m scripts.build_dial_policy_package --output /absolute/new-review
.venv/bin/python -m scripts.build_dial_policy_package --check-output /absolute/new-review
```

命令只读取现有证据并写审查材料，不创建 runtime 数据库、角色连接、密码输入，
不启动模型或物理。登记初次创建使用 `--write-registry`，已有登记不会被静默覆盖。
源模型、规则、控制或裁判与登记不符时检查失败，要求重新审查版本与证据。

每项 `public/` 包含：

- `task.json`：原 R11 公开投影，加结构化 completion、支撑要求、工作范围及
  静态几何引用。`table-geometry.json` 导出实际分块/网格，保留环形桌的孔洞，
  不用包围盒冒充实体；具体网格按需读取，避免全部挤进首次规则上下文。
- `task-guide.md`：从当前 task.rules 生成的规则正文，指向家族指导。
- `family-guidance.md`：公开任务推理、并发、物体/工具参考系与失败反馈的使用方式。
- `camera-calibration.json` 和三张初始全局参考图：静态标定及与正式输入载入相同
  来源的初始图，不是运行中的物体观测。

`private/family-evidence.json` 只供工程审查。公共 JSON 递归检查密码、Oracle
证据入口、物体初始精确姿态和实时真值等禁入字段。机器人基座、桌面和相机的
静态规格可公开，与泄漏物体实时/初始精确姿态不是同一件事。

`review.json` 固定公共文件、家族指导和当前共享角色/Skill/harness 的 hash。
复核既检查 hash，也重新生成任务投影及规则正文对照，避免一份内部自洽但
内容已变的 manifest 混过检查。公共字段与具体运行如何载入，仍待公共入口整合。

## 旋钮专属运动编辑与独立游标

```bash
.venv/bin/python -m scripts.edit_dial_recording /absolute/attempt/motion \
  --output /absolute/new-dial-edit.json
```

`robofrontier.suite.dial_motion_edit` 读取既有 60Hz qpos、动作/恢复结果、外观事件和
过程日志。它在当前 edit 协议上增加家族 profile，不改公共录制或编辑核心。

基础小幅静止判断继续可用；新逻辑先保护新的有效输入（即使 BEST 不变）、
READY/支撑变化、失败/恢复和完成上下文。对动作内部使用记录中的实际趋近进展，
避免把一个长 action 的全部等待直接保留，也避免把微量但有效的夹爪动作删掉。

对于机械臂安静、无新的有效驱动或任务事件的窗口，使用 6..30 秒多尺度，确认
物体/夹爪是重复振荡、至少三个周期且中心不漂移后，才可省略持续往复的中段。
慢速净移动、单向回位、掉落、未知执行及证据不足的片段保留。阈值与判定版本、
源码及过程/恢复文件 hash 进入 sidecar；这不是“实测力为零”的证明。

`DialReplayTrack(recording, edit, fps=30, speed=10).at(global_frame)` 是一个工位的
独立游标，返回源帧/源时间、原 float32 qpos 和截至该时间的外观变化。不同工位
各建一个对象即可独立跳时；回退读取会重置自己的外观事件游标，结束后保留最后
姿态并显式标记完成。用完调用 close 或使用 with。它不控制全局场景时钟，
不实现公共多桌 renderer，也不擅自循环；这些决策留给合并后的合成器。
首次读取或回退后的 `appearance_reset=true` 要求消费者先恢复该工位记录的
`initial_rgba`，再应用累计外观变化，防止循环/回退后保留上轮的计分板颜色。

原始记录和成绩不变。旧的 10x 视频仍是之前版本的产物，新 sidecar 不会静默
覆盖旧视频或伪装成新的物理实验。合并后应按各工位游标先更新数据，再渲染一次
工厂；不在合成后用全局视频裁剪弥补各桌不同的静止区间。

## 完成与未完成的边界

本家族交付规格/证据、专属指导、输入预览与一致性检查、进展保护的编辑 profile
及独立读取适配器。公共正式就绪状态、run_setup 载入、三家编辑接口整合、多桌
批量更新与渲染、完成后循环策略、正式 GPT 协议确认均在 integration-requests 中。
尚未启动 GPT，也未把 Oracle 私有答案、执行脚本或本工程对话提供给 policy。

## 本次验收产物（2026-10-27）

最终审查目录：`outputs/engineering/dial-family-handoff-20261027-v2/`。
`policy-review/` 提供九项公共输入预览，`motion-edits/` 提供九份新清单，
`summary.json` 保存逐项核对结果。26 项相关检查通过，14,163 个展示源样本逐个
与原始 float32 状态和时间戳一致，所有九项过程证据覆盖检查通过。

新清单合计省略 13.733 秒原始时间。相比早期仅幅度判定的 18.083 秒，一些区间
因有效尝试、执行结果或支撑上下文而恢复保留。本次九条真实记录没有找到符合
全部条件的大幅持续无进展振荡，因而该类实际省略为零；多尺度大振荡识别、
缓慢漂移保留和未知证据保守处理由独立样例验证，不伪称九项均存在该现象。
这里没有重渲染旧视频，也没有启动 Oracle 或 GPT；全厂公共合成器仍待合并整合。
