# Frontier V2 任务入口

四类任务各有三个规模、三个等级，共36项。各类最终规格由所属 worktree 的家族清单
及任务说明确定；合并负责人统一接入全局 catalog、公共输入加载与 readiness。
当前 `configs/suites/frontier-v2.json` 是待更新的组合清单。

| 家族 | 规模 | 等级 | 当前入口 |
| --- | --- | --- | --- |
| Transport | 4／8／12 臂 | 平面放置／固定插板／弹簧插板 | [最终任务说明](tasks/transport.md)；`configs/suites/frontier-transport-r14.json` |
| Mosaic | 4／6／8 臂 | 三种拼图约束 | Mosaic worktree 最终交付，合并时登记 |
| Relay | 8／12／16 臂 | 三种载荷／机构约束 | Relay worktree 最终交付，合并时登记 |
| Dial | 4／6／8 旋钮 | 三种锁机构 | Dial worktree 最终交付，合并时登记 |

Transport 当前物理场景是 R14：中央桌孔为四臂实心、八臂0.476×0.476m、
十二臂0.833×0.833m。九项原 R13 Oracle 已完成；R14保留独立的新场景资格状态。
合并前的共享准备与回放边界见 [Transport交接](plans/suite-v2/transport-merge-handoff.md)。

历史场景修订、局部验证与预览见 `docs/history/`；完整记录保留在各批次 `runs/`。
[旧环境说明快照](history/frontier-v2-environments-before-cleanup-20261027.md) 用于追溯原始输入。
