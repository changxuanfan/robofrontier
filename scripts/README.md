# 工程入口

从仓库根使用`.venv/bin/python -m scripts.<名称>`。

| 入口 | 用途 |
| --- | --- |
| `tasks list/check/prepare/prepare-batch` | 当前36项的版本、输入审计与独立GPT准备 |
| `factory build/view/video/poses` | 当前工厂、独立循环、30fps视频与Cycles动画状态导出 |
| `restore_scene_assets` | 按原SHA恢复忽略的场景资产，拒绝覆盖漂移资产 |
| `check_harness` | 全部离线Python/JS/文档与协议回归 |
| `check_oracle_preservation` | 保护清单与冻结Oracle绑定校验 |
| `benchmark_robo_factory` | 同分辨率、同镜头、交替顺序的成本比较 |
| `export_factory_blender`、`render_factory_blender` | 高质量离线出片 |

`create_pipeline_run`保留用于旧任务/显式配置；`meta_dispatcher.js`是冻结Meta执行器。
家族构建、审核和Oracle脚本保留其原名以支持历史证据、测试和可复现性。它们不负责
选择当前GPT版本。不要把带oracle、commission、probe的脚本当成正式GPT启动入口。

迁移工具`archive_worktree_dependencies`保留归档回执；`prepare_factory_edits`
产生新源剪辑版本，不修改原录制或历史剪辑。
