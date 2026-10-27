# Relay合并交接

当前任务与默认入口以[docs/relay.md](../../relay.md)为准。
无需重读数十轮接续日志来推断默认状态；历史全文保留在`docs/history/`。

## 已完成的本类整理

- 增加Relay专属九项catalog，入口读取仓库内正式servo2配置，无旧run隐式配置依赖。
- 本类规则唯一来源为`relay_contract.py`；九项任务／公开任务同步为最终说明。
  50mm容差、允许堆叠、低位30mm松爪、无额外静止／速度门槛已经接入Relay判定。
  只对有新规则版本的任务生效；旧冻结记录保留原语义与分数。
- Goal、受限退出、停放供方和精确状态修正接入正常函数调用，去除4套install包装。
- 退役4个脚本：旧限时批次、一次性追加规则、旧进度影片、装置提案预览。
  原文件在`runs/engineering/relay-cleanup-20261027/retired/`逐字节保留。
- 分析入口更名为`analyze_relay_motion.py`，只产出剪辑表；唯一新版视频出口仍是20x／30fps。
- Relay剪辑实现与测试改为`relay_motion_edit.py`／`test_relay_motion_edit.py`，
  不再与Mosaic的通用同名文件碰撞。新包schema为`robofrontier.relay-motion-edit.v1`；
  已交付的旧包不改写，读取时必须验证Relay轨迹字段，不能只凭旧v1字符串认格式。
- 两批关键executor的16个worktree依赖链接已改为稳定依赖归档；21份相关场景保存。
  `manifest.json`保留旧目标、新目标、原hash；原报告和冻结Python字节不改写。
- 长接续文档移至`docs/history/`，旧入口为简短导航；没有清掉场景演进依赖或其他家族目录。

## 合并范围

独立内容：Relay契约、任务／公开任务、专属catalog、参考／恢复／剪辑模块、测试和当前说明。

需要主session审查的共享文件改动：

| 文件 | 本类改动与合并要求 |
| --- | --- |
| `suite/evaluate_v2.py` | 仅Relay类读取显式规则参数；保留其他家族与旧任务语义 |
| `suite/v2.py`、`rigid_relay.py` | 本分支既有Relay版本选择／公开投影／构建变更；不覆盖其他分支的新catalog |
| `factory_workstations.py` | 精确源绑定与向量化只读放置；需与其他家族播放器共同集成 |
| `.gitignore` | 仅本地／生成数据忽略，不清除未跟踪素材 |

不要直接覆盖共享catalog或以ours/theirs整文件选边。
各worktree的最新专属catalog才是输入来源；公共R10目录不代表其他家族当前状态。

## 正式GPT前仍需主session完成

1. 合并四类最终任务／Prompt／Skill输入与加载方式；正式就绪状态应绑定规则、控制、源与证据。
   当前Relay仍`environment_preview_only`；不批量翻转oracle_verified，也不自动继承旧限时probe预算。
2. 通用harness、角色／并发／冻结／观测回归；本轮没有改通用控制器或启动GPT。
   通用`freeze_executor`未来仍会创建worktree别名；本轮修复的是已选两批历史依赖，
   公共后续运行的持久打包策略还需在合并时统一。
3. 统一各家族剪辑格式。Mosaic引用原帧的格式与Relay完整派生轨迹包不同；
   文件名冲突已避开，语义适配仍不能省略。
4. 多桌同屏播放器：每桌按自己的倍速和剪辑时钟取原始帧，全部更新后一次绘图；
   完成后保持终态。当前只有逐项视频，未宣称九桌／36桌联合回放已实现。
5. 扩充对明显而长期无进展的主杆／弹簧被动振荡识别，保留真实释放、掉落和恢复证据。
   现有960段复核和时间表仍可直接复用；不能把所有被动物体运动都删除。
6. 工厂丰富度、桌子精确源替换、去墙面Robo Factory展示字、近远景／镜头Demo及成本对照。
   素材、来源和许可均保留。上下文／compact设置属于Codex宿主，不作为仓库已实现功能。

## 数据不能遗漏

`runs/`、`outputs/`、`vendor/`、`.venv/`是共享链接，不属于Git提交。
两批原始轨迹、完成截止、用户／native报告、辅助标签、控制扩展、请求结果和hash全部保留。
旧用户验收9/9不是native原评分9/9，也不是当前整理代码的一轮新自主Oracle结果。

集中视频目录各直接9个MP4：
`outputs/oracle/relay-nine-videos-20x-20261027/`与
`outputs/robo-factory/relay-nine-videos-20x-20261027/`。
剪辑轨迹／完整冻结源在`outputs/oracle/relay-edited-tracks-20261027/`。
原始包保留源帧号、源时间、编辑时间、cut边界、外观事件与hash，不需要从视频重新找静止。

运行时依赖归档保留在共享`runs/engineering/relay-cleanup-20261027/`。
它专为已选择Relay输入复核保存，不是可执行其他家族所有历史构建的完整仓库副本。
任何物理新运行仍需显式授权与独立目录，不自动重复旧ID或旧控制请求。

本次164项Relay回归、40项相关检查通过（两组有重叠）；九项输入只读检查通过。
18段视频完整解码及哈希、九份剪辑轨迹哈希、43,032份证据文件校验通过；
当前45份几何／物理／注册表文件与整理前逐字节一致。
删除95份本地pyc缓存，约1.8MiB。未宣称全项目所有历史测试都通过；
已有部分非Relay环境的缺失／外链贴图问题仍由主session处理。

本次检查结果和删减清单见`runs/engineering/relay-cleanup-20261027/cleanup-result.json`。
