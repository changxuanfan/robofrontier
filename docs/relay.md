# Relay：当前任务与入口

Relay Oracle已按用户终点规则完成九项、27根。原始记录、失败／恢复历史与辅助分类保留。
当前任务、默认入口及文件职责如下；历史过程不作为运行说明。

## 任务

| 机械臂数 | 布局 | 每项装置数 | L2／L3初始端点跨度 |
| --- | --- | --- | --- |
| 8 | 两排直线 | 2 | 1.15m |
| 12 | 90°圆弧，外7／内5 | 3 | 1.15m |
| 16 | 半圆，外10／内6 | 4 | 1.20m |

三种规模各有L1／L2／L3，共九项。桌面高0.80m；16臂径向排距1.40m，
12臂外／内半径3.46／2.16m。L1器材与布局保留。
L2为63°圆弧杆；L3为31.5°折转、148.5°内夹角，全部为±25°的垂直弯折被动弹簧。
L2／L3下表面距末端沿杆100mm处直接连接固定环，无短柱，
配170mm木质摆杆和600g砝码。基座、区域与几何细节以各项冻结模型为准。

最终规则：

- 各装置从自身初始位置向公共GOAL运输；不必先回START。团队、交接点、调度由规划者选择。
- 空中运输至少两只真实木杆抓持；先建立接收支持再释放供方，每根必须有真实合格交接。
- GOAL允许任意摆放、互相接触和堆叠。整套装置的几何边界允许最多50mm向外误差。
- 允许低位先松爪：最低装置部件距桌面不超过30mm时可开始释放，无需先碰桌。
  完成仍须实际落地／获得合法货物支撑、全部机器人松开、实际位置符合范围。
- 不增加速度门槛或静止时长要求。保留过程违规与恢复历史；禁止状态写入、焊接、回滚或重生。

规则唯一来源为`robofrontier/suite/relay_contract.py`，版本`relay-final-goal-50mm-v1`。
`task.json`和`public-task.json`由此生成完整说明；不再追加版本更新条款。
旧记录自身没有此版本标记时保持原判定，不能自动重命名为当前native通过。

## 默认入口

唯一批次入口：`scripts/run_relay_nine_oracles.py`。
输入为`configs/suites/frontier-relay.json`九项，运行配置为
`configs/harness-frontier-v2-servo2.json`，不依赖某次旧run中的配置文件。
默认无总时间截止，单动作期限、显式STOP和未知执行保护保留。
单项执行由`run_relay_live_oracle.py`承接，默认流水与灵活交接；不需要安装运行时补丁。

只读输入检查，不启动物理任务／GPT：

```sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=. \
  .venv/bin/python -m scripts.run_relay_nine_oracles --check-inputs
```

Oracle已经结束；整理过程没有重跑。新入口的静态／局部回归不等于
从初态重新证明九项全程自主成功，也不能消除历史工程辅助标签。
任务仍保留预览就绪标记，正式GPT入口及公共加载／就绪接线由主session集成；本轮没有启动GPT。

## 代码职责

| 路径 | 职责 |
| --- | --- |
| `suite/relay_contract.py` | 最终规则、任务投影、输入检查与默认路径 |
| `suite/relay_oracle_v2.py`、`relay_pipeline.py` | 私有参考规划及并行执行前沿 |
| `suite/relay_live_recovery.py` | 原现场类型化恢复，保留请求与执行事实 |
| `suite/relay_user_goal_policy.py`、`relay_contact_escape.py`、`relay_donor_staging.py` | Goal释放、受限接触退出和可清空供方的正常路径 |
| `suite/relay_observation_state.py` | 只读精确状态证据 |
| `suite/relay_scene.py`、`relay_underslung.py`、`rigid_relay.py` | 当前几何及仍被使用的构建能力 |
| `relay_video_motion.py`、`relay_motion_edit.py` | 背景无关运动判定、时间映射 |
| `scripts/analyze_relay_motion.py` | 一次生成剪辑判定，不再另输出16fps旧版影片 |
| `scripts/export_relay_edited.py` | 派生60Hz轨迹及20倍速／30fps简单与工厂视频 |

## 数据与交接

所有数据位于主仓库共享`runs/`、`outputs/`，不随Git合并自动携带。

- 完成选择：`runs/oracle/relay-r16-38b5423-20261026-171651/USER-GOAL-ALL-NINE-COMPLETE.json`。
- L1原记录：`runs/oracle/relay-nine-ab1abec-20261026-032833/`。
- L2／L3原记录：`runs/oracle/relay-r16-38b5423-20261026-171651/`。
- 视频与逐帧映射：`outputs/oracle/relay-20x-20261027/report.md`。
- 独立剪辑轨迹：`outputs/oracle/relay-edited-tracks-20261027/`。
- 保留依赖、原始任务和退役源码：`runs/engineering/relay-cleanup-20261027/manifest.json`。

素材、原始60Hz流、外观事件、失败／恢复／控制修正hash、报告、视频均保留。
详细媒体说明见[记录与回放](relay-recorded-video.md)，
主session只需再读[合并交接清单](plans/suite-v2/relay-merge-handoff.md)。
