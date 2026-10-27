# 当前入口与数据保留

| 要做的事 | 入口 |
| --- | --- |
| 选择36项当前任务／版本 | `configs/task_families/current.json` |
| 浏览四类任务说明 UI／下载离线包 | `outputs/task-atlas/v1-20261027/index.html`、`outputs/exports/task-atlas-four-families-20261027.zip` |
| 配置不同模型与上下文 | `configs/models.json` |
| 运行 Codex CLI 基准 | `python -m scripts.benchmark`；见 [运行说明](gpt-policy.md) |
| 保留的 Oracle 构建／运行／验证 | `scripts/run_oracle_family.py` 与各家族当前 runner；见 [任务入口](frontier-v2-environments.md) |
| 查看最终简单背景 Oracle 视频 | `outputs/final/oracle/{transport,mosaic,relay,dial}/` |
| 查看当前工厂图／成片 | `outputs/final/factory/` |
| 查看保留的历史 GPT 对照 | `outputs/gpt/frontier-v2-minimum-20m-20261025/` |
| 查看最终产物原路径、倍速、哈希 | `outputs/final/manifest.json` |
| 原始动作与运行证据 | `runs/oracle/`、`runs/policy/` |
| 工程核验／清理回执 | `runs/engineering/` |

`outputs/final` 是统一浏览入口，通过相对链接引用保留的媒体，不重复拷贝视频。原始60Hz源流、冻结场景、动作／过程证据和源帧剪辑保留原路径，避免破坏已发布数据的引用。Transport 有正反两视角18条，另外三类各9条，共45条简单背景视频；Dial 的 assisted 案例保持原标识。

任务图谱的网页、图像、公开规格摘要、生成脚本和离线ZIP作为最终资料保留，使用当前GPT任务来源；它不是实时运行面板，也不包含Oracle解法或私有密码。上述目录列入显式保留清单。

工厂保留当前 V10 场景、Blender 导出、top／side 图、环视成片，以及最后发布的任务展示。静止／细抖动裁剪表与原始源流分开保存，展示剪辑不覆盖原数据。已编码的整段PNG缓存、未选用的旧试验／校准、旧工厂试图和过期 prepared 包按 `configs/retention.json` 清理。本轮盘点之后新生成或更新的渲染目录排除在清理之外。

18个未被现有代码／配置／测试引用的一次性探针、概念图脚本及被CLI替代的旧20分钟Desktop批量入口已移除。原提交仍在 Git 历史中；仍被现有 Oracle、场景构建或回归引用的模块保留。四个旧任务 worktree 已移除，分支提交保留；39个独有忽略文件已归档并核对哈希。main 和当前开发 worktree 保留。

本轮回执在 `runs/engineering/cli-migration-cleanup-20261027/`：保留清单、删除清单、旧worktree归档、原生CLI协议检查与回归检查分别记录。清理前对174917个当前源／交付文件建立哈希清单；清理后的检查以实际回执为准。旧历史保护清单记录当时的数据全集，不再要求已获授权退役的试验继续存在。

`runs/`、`outputs/`、大型场景资产和 `vendor/` 不在 Git 内。Git 历史只能恢复源码，不能恢复已删除的未跟踪试验数据。当前仍保留的原始源和最终交付需随数据根备份；单独推送源码不能恢复实验。
