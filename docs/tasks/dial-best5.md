# BEST5 旋钮：当前规格与合并交接

旋钮家族为 R11 BEST5 九项。Oracle 证据已完成：八项程序 Oracle，八盘 L3 为辅助 Oracle。
正式 GPT readiness、家族说明的公共载入和全厂多桌合成仍待主 session 整合；本分支未启动 GPT。

## 唯一当前入口

| 内容 | 路径 |
| --- | --- |
| 九项模型目录表 | `configs/suites/dial-best5-r11.json` |
| 规格、默认值与冻结证据登记 | `configs/task_families/dial/best5-r11.json` |
| policy 专属指导 | `prompts/tasks/dial-best5.md` |
| 公共整合请求 | `configs/task_families/dial/integration-requests.json` |
| 当前审查包与进展感知剪辑 | `outputs/engineering/dial-family-current/` |
| 已发布简单背景九视频 | `outputs/oracle/dial-nine-videos-20261027/` |
| 已发布工厂九视频及对应场景 | `outputs/robo-factory/dial-best5-edited-10x-20261027/` |

## 最终规则与默认

| 盘数 | L1 机械臂数 | L2 机械臂数 | L3 机械臂数 |
| --- | --- | --- | --- |
| 4 | 4 | 8 | 8 |
| 6 | 6 | 12 | 12 |
| 8 | 8 | 16 | 16 |

密码每位 1..5，0 无效，60 度一格；桌高 0.8m，白盘直径 0.3m。具体桌面轮廓、
开孔、基座、计分板与相机按各 case 的冻结模型读取。L1 模块固定；L2/L3 要求独立
蓝支撑、真实红抓持和整模块悬空；L3 为偏心 crank。完整规则取自该 case 的 task。

BEST 是历史完整有效输入的最大命中数，READY 表示当前输入资格。中间稳定配置
也会自动结算；BEST 不变的新有效输入仍是尝试。成功锁存不会擦除过程违规。
公开输入只提供允许的静态规格、RGB 和机器人反馈，不加入 Oracle 答案或实时物体真值。

展示默认由 `robofrontier/suite/dial_pose_timeline.py::PRESENTATION` 唯一维护：
原始记录 60Hz，展示 10 倍速、30fps。每桌按自己的编辑时间线只播一遍，完成后
保持最终姿态并标记完成；全部工位完成后结束输出，不循环。

## 审查与回放工具

```bash
.venv/bin/python -m scripts.check_dial_family --output /absolute/NEW-review
.venv/bin/python -m scripts.build_dial_policy_package --check-output /absolute/EXISTING-review/policy-review
.venv/bin/python -m scripts.edit_dial_recording /absolute/attempt/motion --output /absolute/NEW-edit.json
```

检查工具不创建 runtime、角色连接或模型会话。它绑定九项 report、资产、控制与
裁判指纹；公共输入按源规格重新生成对照。历史报告和输入保持原始字节，不将
`oracle_verified` 或旧状态字段批量改写。每次新审查选择新的输出目录。

审查包的每项 `public/` 含 task、生成的 task-guide、family-guidance、相机标定、
初始参考图及按需读取的 `table-geometry.json`。环形桌使用真实分块/网格，保留孔洞。
私有证据索引只供工程审查。公共角色 prompt、Skill、harness 保持共享权威版本。

```bash
.venv/bin/python -m scripts.replay_dial_motion /absolute/attempt/motion \
  --edit /absolute/edit.json --output /absolute/NEW-video --render
.venv/bin/python -m scripts.render_dial_oracle /absolute/attempt \
  --edit /absolute/edit.json --output /absolute/NEW-views
```

两个旋钮渲染入口消费同一源帧清单；前者输出 simple/factory 配对视频，后者选择
简单背景相机。工厂场景默认取已发布 10x 目录的 scene。只恢复 FK 与外观，按
固定工位平移/旋转放置，不重新执行动作、IK 或接触积分。反馈/边界帧单独保留，
10x 为标称动作速度；每个输出帧的原始编号和时间写入报告。

`run_best5_dial_batch.py` / `run_blind_dial_oracle.py` 是显式工程 Oracle 启动工具，
统一使用 R11 与 `configs/harness-frontier-v2-servo2.json`；不传预算表示不限时。
`commission_best5_dials.py` 是有明确诊断边界的局部工程工具，不作为完整任务证据。
肩夹实验入口已退役；模型真实挡肩几何保留。历史读取器仅用于旧证据复核。

## 数据级去静止与独立游标

`dial_motion_edit.py` 结合原始 qpos、动作进展、过程和外观事件分类。保留有效
输入、支撑变化、失败、恢复、完成上下文和未知证据；对无有效驱动、中心不漂移的
长期重复振荡进行多尺度判断。力设定不是实测接触力，分类不声称“实测力为零”。
当前九条真实记录未确认可安全删除的大幅长期无进展振荡；该能力由独立样例验证。

`DialReplayTrack.at(global_frame)` 返回本工位的源帧/源时间、原 float32 qpos 和
累计外观变化。首次/回退的 `appearance_reset` 要求先恢复本工位 initial_rgba；
不同工位游标互不影响。主 session 应在更新全部工位后渲染一次，使用独立全局
展示时钟，并在全部时间线结束时停止。此公共合成器尚未实现于本分支。

## 合并与证据保护

旋钮新增编辑和渲染实现均为 dial 命名；通用编辑模块未占用，两个通用渲染脚本
恢复为 main 版本。必要公共修复仍以小补丁保留：GPU 姿态同步、工位变换与子集、
执行终态所有权和未知围栏。主 session 按 integration-requests 整合，不覆盖整份文件。

`dial_best_oracle` 仍继承 `dial_hold_oracle → dial_oracle_v2 → dial_oracle` 的活跃
持握/几何实现，并使用 `verify_floating_dial_oracle` 的几何函数；不要按名字删除。
保留两批正式原始记录、失败/中断与恢复来源、配对私有输入、冻结源码及必要资产。
已发布视频保留其真实 edit/scene/帧映射；当前新清单不冒充旧视频已经重渲染。

历史过程在 [Oracle 记录](../history/dial-oracle-continuation-20261027.md)、
[旧交接](../history/dial-oracle-handoff-20261026.md) 与
[渲染历史](../history/dial-motion-render-history-20261027.md)。历史中的暂停、待办和
旧路径描述不作为当前指令。验收明细以当前审查包和清理回执为准。
