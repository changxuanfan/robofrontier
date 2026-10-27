# Mosaic 合并交接

当前任务规则以[任务说明](../../tasks/mosaic.md)为准。此文件供主session接收本分支，
区分已交付能力和公共整合工作。当前未启动新GPT或Oracle，也没有改共享harness。

## 接收入口

| 内容 | 入口 |
|---|---|
| 当前用途与交付命令 | `configs/task_families/mosaic/delivery.json`、`python -m scripts.mosaic` |
| GPT九项 | `configs/task_families/mosaic/r14.json`、`configs/suites/frontier-mosaic-r14.json` |
| 原Oracle九项 | `configs/task_families/mosaic/r13.json`、`configs/suites/frontier-mosaic-r13.json` |
| 当前公开GPT材料 | `outputs/policy-packages/mosaic-r14-local-colours-20261027/public/` |
| 原公开审查包 | `outputs/policy-packages/mosaic-r13-20261027/public/` |
| 原Oracle结果汇总 | `runs/oracle/mosaic-continuation-20261026/FINAL.md`、`evidence-summary.json` |
| 视频/源剪辑 | `outputs/oracle/mosaic-continuation-20261026/videos/video-index.json`、`轨迹剪辑时间表/index.json` |
| R14私有配色证据 | `runs/engineering/mosaic-local-colours-r14-20261027/` |
| 私有去无用段候选 | `runs/engineering/mosaic-motion-review-20261027/` |
| 整理保护与变更清单 | `runs/engineering/mosaic-merge-cleanup-20261027/` |

R13完整通过1566格，保留7次失败动作与原世界恢复；三个4臂选中运行是pure_program，
其余六个为privileged_assisted。R14只有颜色变体设计/几何等价证据，不继承R13
完成颜色的Oracle资格。正式policy_launch_ready仍为false。

日常检查默认gpt；复查原Oracle显式指定purpose：

```sh
python -m scripts.mosaic check
python -m scripts.mosaic check --purpose oracle
python -m scripts.mosaic check --verify-recorded-chunks --verify-protected \
  --report runs/engineering/NEW-report.json
```

命令只检查输入。只有显式`--report`才创建新报告，拒绝覆盖旧报告。
`package --output NEW_DIRECTORY`导出已经核对的冻结包，不改颜色、不重算Oracle。
颜色设计构建器与原R13包构建模块保留原字节用于版本核验，不是日常默认入口。
历史说明在`docs/history/mosaic-*`，不作为新运行协议。

## Git之外的数据与资产

本worktree的`runs/`、`outputs/`、`.venv`、`vendor`指向主仓库持久目录。合并提交
不复制这些目录，也不自动带入被忽略的`assets/`网格与贴图。R14及R13共18个
场景目录的JSON/MJCF随Git交接，资产需单独物化。

合并后在主仓库执行：

```sh
python -m scripts.mosaic assets \
  --destination /Users/fanchangxuan/Repos/roomscript2 --dry-run
python -m scripts.mosaic assets \
  --destination /Users/fanchangxuan/Repos/roomscript2
```

物化器逐项校验workstation清单与资产SHA，复用相同目标文件，遇到不同字节拒绝
覆盖。源按当前资产、原Oracle的自包含`motion/source/assets`、持久内容缓存查找。
R14仅改XML/metadata颜色，故可从原R13冻结资产恢复相同网格和贴图；无需依赖
即将退役worktree的资产目录。没有启动模型、IK或物理积分。

私有R14设计文件有绑定的绝对路径，当前都位于主仓库持久`runs`，不在worktree。
迁往另一台机器时，需要明确的路径映射/新交付清单；保留原文件的hash和出处。

`_executor_launches`中的启动别名有部分指向旧worktree。这些是历史启动缓存；
选中trial的`executor-source`、`motion/source`和状态块才是受保护的复核/回放源。
不以启动缓存恢复旧物理世界。原实验路径字符串保留为出处，清单记录需注意的
别名；本分支不删除worktree、不重写旧冻结源或别名。

## 本分支整理范围

当前说明收敛为任务说明和本交接。旧R13协议、任务包形成过程、R14配色设计、
初期GPT/工厂规划移入既有`docs/history`。R13/R14的版本绑定prompt与共用skill
保持原字节，使已交付审查包仍能通过验证。

已完成的诊断、故障注入、并行探测和旧分阶段视频导出入口从工作树撤下；原字节
在Git提交`a66e183`及相关trial的冻结源码中。`retired-code.json`逐项记录hash和
冻结副本。仍被R14导入的Coverage、恢复执行器与有效回归测试保留。

删除候选仅限被正式包替代的初稿、重复重导出时间表及可再生缓存。视频按完整
内容SHA去重成硬链接，保留文件路径、编码字节、历史视频和所有现有索引；有用
的恢复日志、失败反馈、source chunks、原场景、剪辑表和审核报告继续保留。

`protected-files.json`绑定103823个原文件，含选中完整Oracle trial、原496文件
保护集合、R14场景/资产/已交付图集及其版本依据；SHA由delivery.json固定。
整理报告记录实际删除和去重结果。早期物理单元约77GiB的路径累计大小包含
大量共享硬链接，不作为可释放空间，不整目录删除。

## 共享接入：主session统一实施

主session的接收顺序为：合并及数据完整性验收、公共接线与工厂整合、Oracle多桌
演示/镜头验收、冻结新GPT协议并并行运行。GPT批次需隔离各run的数据库、凭证、
物理owner和输出；模型并发、CPU与费用预算由新协议明确。

1. **正式门禁与版本选择。** 通用`v2.cases()`的Mosaic仍默认R10，旧minimum
   probe脚本固定R10/1200s。改由用途registry显式选择九项，识别R14颜色变体证据
   范围；新的模型、effort、topology、费用和运行协议另行冻结。旧pilot期限不继承。
   `validate_task`/`World.start_trial`的preview限制按共同协议接入，不能伪装旧probe。
2. **公开材料装载与封存。** `run_setup`目前只显式复制通用角色/skill，尚未
   装载Mosaic专项材料。按白名单合并公开completion、静态几何和guidance，在封存
   前登记文件/hash；家族公开task是补充材料，不能用其preview字段覆盖新正式门禁。
   使用干净policy上下文，工程registry、配对、真值、修复经验不进入策略。
3. **控制/裁判等价。** 原基线servo2/pd1000-v1与各冻结记录一致。现有物理指纹
   hash整个`pipeline.py`，仅接入门禁也会改变hash；先做影响分析与兼容证明。
   保留历史指纹，不能抹平差异或自动认定必须重跑全部Oracle。
4. **统一剪辑协议。** Mosaic/Dial/Transport存在同名v1但结构和区间边界不同。
   设计新版本和旧格式适配器，保留case/scene/stream/events身份、精确输出帧及
   全采样率保留区间。源时间、剪后时间、输出帧时钟分别处理，不跨已删区间补帧。
5. **多桌独立时钟。** 每桌按自己的剪辑映射/倍速消费源状态与事件；全部工位
   更新后统一设置展示时钟、只渲染一次。完成桌保持精确终态，最长桌结束则结束
   视频，不循环。允许工位实例绑定相同case_id的不同scene/run，避免Oracle与GPT
   颜色混用。当前`FactoryPlayback.apply`会设置全局data.time，公共编排器须处理。
6. **场景与拍摄。** Oracle匹配场景为`outputs/robo-factory/v6-mosaic-recorded-20261027`；
   旧v6的Mosaic R10基座不匹配。整合其他家族时逐桌读取最终冻结源，工位仅整体
   平移/水平旋转，不缩放。GPT使用其自身冻结R14源。工厂美化、墙面Robo Factory
   字样移除、总览/局部环视/逐桌分难度镜头留作公共工作；保留材质/资源复用和
   近远景能力。复杂度与渲染成本以相同设置实测，用户期望成本增幅在100倍以内。
7. **语义去无用段。** 保留现有Mosaic审查标签的提议身份。它保护动作、commanded
   grasp lifetime、进展和完成，当前全窗口汇总偏保守；明显长期振荡的任务作用
   仍待审查。力设定、姿态不等于实测接触力。135个idle提议未自动应用。

现有公共差异需按功能整合：`robofrontier/motion_edit.py`、
`scripts/render_frontier_oracle.py`、`scripts/replay_motion_in_factory.py`、
`scripts/export_factory_recorded_pose.py`及相关回归测试。不能按某个分支最后提交
直接覆盖其他家族实现。其他worktree可能继续更新，合并前重新固定提交和未提交数据。

共享README的Mosaic状态可直接采用：

> Mosaic九项R13完整Oracle及独立动作记录审计通过，共1566格。后续GPT使用
> R14附近混色供料，位置和物理保持不变；其正式共享接线尚待完成。Oracle工厂
> 回放继续绑定原R13轨迹。展示默认30倍速/30fps，各桌独立去等待、保持终态，
> 全部完成后结束。当前规格和数据入口见docs/tasks/mosaic.md。

## 合并验收

本分支整理验证通过：58项相关测试、R13/R14两份九项审查包、原始动作块复查、
103823文件保护核对，以及脱离worktree资产的18场景/1332资产绑定恢复与编译。
详细报告在`runs/engineering/mosaic-merge-cleanup-20261027`。

- R13原任务/轨迹/失败恢复/审计不变，R14九项颜色与目标数量守恒；用途不串版。
- 18场景的资产可以在缺少原worktree资产的目标目录恢复，重复执行可复用，冲突拒绝覆盖。
- 两份审查包通过，公开边界与对应prompt/skill一致；保护清单全部通过。
- 历史九个30倍速视频及源映射仍可读取，所有旧视频路径/hash保留。
- 不引入新Oracle/GPT启动或旧实验重放；共享控制、裁判、READY与角色权限保持原样。
- 公共阶段再验证：等待区间错开的多桌并播、完成后hold_final、最后一桌完成结束，
  源/版本错配拒绝，以及不同机位与背景下的源状态映射。
