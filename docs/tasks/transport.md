# Transport 任务

当前默认物理规格为 **R14 半孔**，九项清单为
`configs/suites/frontier-transport-r14.json`。规格与验收索引是
`configs/task_families/transport/half-opening-r14.json`，GPT任务参考是
`prompts/tasks/transport.md`。公共加载与正式就绪资格待合并接入；GPT尚未启动。

## 最终规格

| 规格 | 四臂 | 八臂 | 十二臂 |
| --- | --- | --- | --- |
| 每个等级目标数 | 12 | 24 | 36 |
| 桌面外边长 | 1.938m | 2.720m | 3.536m |
| 桌高 | 0.8m | 0.8m | 0.8m |
| 中央桌孔长×宽 | 实心 | 0.476×0.476m | 0.833×0.833m |

桌面支撑由实际盒体定义，中央孔属于无支撑区域。公开任务包含支撑盒体、桌面高度、
工作范围、机器人基座、静态相机安装标定、库存、订单、机构和完成条件。
相机安装位姿声明 world 或 body 局部参考系，不能把活动相机的局部安装坐标当实时世界坐标。

物料是60mm方块或直径／高度60mm圆柱。白板为143×396mm，目标中心间距120mm，
84mm颜色／形状图案表示身份。双金属片厚×宽×长为2.5×12×24mm、中心距24mm；
插接盲槽开口4.5×15mm、深26mm，板厚32mm。插槽与中央桌孔是不同结构。

- L1：正确来源身份的物体完整落在目标允许占位范围内，释放、直立且稳定。
- L2：固定白板；两金属片完整插入对应真实盲槽，物体落座、释放且稳定。
- L3：相同插口位于被动 z／roll／pitch 弹簧板，按板的当前局部参考系判断插入与落座。

全部目标均需连续稳定1s，并满足过程合规。容差由家族配置的 `acceptance` 和冻结裁判
绑定。来源和白板字母表示库存与订单，机械臂没有永久所有权；空中交接、稳定桌面暂存、
等待与独立任务并行均可自主选择。接收支撑建立后再释放，并保护已完成目标。

## 证据与就绪状态

九项原 R13 独立完整物理见证通过216/216，三项冻结程序、六项工程辅助。
证据索引为 `configs/task_families/transport/r13-qualified.json`；原报告、失败记录、
计时、冻结执行器和完整60Hz状态流保留。

R14是新的真实碰撞场景，已编译并核对仅桌面开孔变化。六项有孔源轨迹的1,998,263个
状态没有接触新增桌面材料；该几何兼容检查没有推进物理时间。
**R14没有新的完整物理执行结果**，保持preview门禁、`formal_gpt_ready=false`。
公共harness接线与正式资格见 [合并交接](../plans/suite-v2/transport-merge-handoff.md)。

## 构建与资产恢复

新包只生成可审阅的家族资料，不创建运行、角色、凭证或GPT会话。输出须为新目录。
R14资产由绑定的Oracle冻结来源复制，具有独立inode；已有资产校验失败时停止，不覆盖。

```sh
PYTHONPATH=. .venv/bin/python scripts/build_transport_policy_package.py --assets-only
PYTHONPATH=. .venv/bin/python scripts/build_transport_policy_package.py \
  --output outputs/transport-r14-review-001 --no-previews
```

`--assets-only` 适用于合并后已有R14元数据、尚未物化Git忽略资产的checkout。
完整构建可为尚未存在的场景生成静态预览；`--no-previews` 省略预览。
公共 `build_frontier_v2.py` 尚未注册本类R14，请使用本类入口。

## 源剪辑与工厂播放

`robofrontier/suite/transport_motion_edit.py` 直接读取原始60Hz状态、动作和过程事件；
`transport_motion_signals.py` 提供运动信号与可选画面核验函数。生成入口为
`scripts/edit_transport_recordings.py`，格式为 `robofrontier.transport-source-edit.v1`。

整桌任一机械臂、夹爪或物体存在有效运动，整桌片段保留。静止及只有无进展反复晃动的
等待会被去除，包括明显、长周期晃动；慢速定向运动、连续转动、真实落物过渡、动作边界
与任务进展保留。判定来自状态和事件，不要求先渲染视频；原轨迹保持完整。

播放规格：**20倍速、30fps**。每桌有独立有效时间，通过
`TransportEdit.at_video_time(T,speed=20)` 查询原帧和原时间。各桌更新后绘制同一工厂帧。
**单桌完成后保持精确终态；全部桌子完成后结束，不循环。** 公共多桌绘制仍待接入。
整工位允许刚体平移和绕竖直轴旋转；相对布局与尺寸保持源兼容。

R14已经是最终半孔，渲染比例相对R14来源为1。原Oracle录制绑定R13，显示同样最终孔
尺寸时相对其来源为0.5。公共渲染器尚需消费此区别，避免对R14重复减半。

## 持久交付

以下路径相对仓库数据根；本worktree的 `runs/` 与 `outputs/` 链接到主仓库。

| 内容 | 路径 |
| --- | --- |
| 最终公开任务包及家族审计 | `outputs/oracle/transport-continuation-20261026/transport-policy-r14-release/` |
| 当前源原生剪辑与生成器快照 | `outputs/oracle/transport-continuation-20261026/transport-source-edits-v2/` |
| 原始物理见证与全部尝试 | `runs/oracle/transport-continuation-20261026/full/` |
| 已交付20倍速正反面18部视频 | `outputs/oracle/transport-continuation-20261026/motion-only-20x/` |
| 九项正面视频入口 | `outputs/oracle/transport-continuation-20261026/全部九项任务视频/` |

已交付视频使用其审核过的剪辑，所依赖的 `motion-timelines/` 与 `motion-only-nine/audit-v4/`
仍保留。当前源原生剪辑保留1,096,132状态、跳过1,035,753状态，供后续工厂使用；
其20x／30fps预计九项正面总时长约15分14秒，尚未重渲这套视频。
一次性旧剪辑／发布脚本与历史计划已归档；历史路径和共享合并事项见交接。
