# RoboFrontier

RoboFrontier 用 MuJoCo 多机械臂任务研究 GPT 的分工、规划与协作。Frontier V2
包含 Transport、Mosaic、Relay、Dial 四类，每类三个规模、三个等级，共36项。
在线任务使用真实接触物理；离线展示读取已保存状态。

## 当前任务与工程入口

- [各类任务与版本入口](docs/frontier-v2-environments.md)
- [Transport 最终规格、Oracle 证据与视频](docs/tasks/transport.md)
- [Transport 合并交接与共享待办](docs/plans/suite-v2/transport-merge-handoff.md)
- [架构、目录与运行数据](docs/architecture.md)
- [Meta 操作说明](docs/meta-runbook.md)
- [Coordinator／Worker／Controller Skill](.agents/skills/robot-collaboration/SKILL.md)
- [Harness 验收范围](docs/harness-v2-verification.md)
- [工厂展示与渲染](docs/robo-factory.md)

Transport 的当前物理规格是 R14 半孔，九项原 R13 Oracle 已完成216/216。
R14 的公开任务包与源剪辑已准备；正式 GPT 的公共加载、就绪资格和多桌工厂播放器
待合并接入。其他三类由各类 worktree 的最终交付确定版本。
`configs/suites/frontier-v2.json` 尚待统一合并，不能用它推断各类最终默认版本。

## 安装与工程检查

Python 与依赖由 `.python-version`、`pyproject.toml`、`uv.lock` 管理。
机器人上游资产与许可证由 `scripts/fetch_robot.py` 物化，来源记录在 `vendor/`。

```sh
uv sync --locked
uv run python scripts/fetch_robot.py
uv run python scripts/check_harness.py
```

准备运行文件使用 `scripts/create_pipeline_run.py`；实际启动与模型配置遵循 Meta
操作说明和所选任务的 readiness。Transport 的包构建／资产恢复入口见任务说明。

## 目录

| 目录 | 内容 |
| --- | --- |
| `configs/` | 任务场景、家族规格、catalog 与控制配置 |
| `prompts/`、`.agents/skills/` | 任务参考、公共规则与角色协议 |
| `robofrontier/` | 仿真、运行服务、任务实现与离线数据接口 |
| `scripts/`、`tests/` | 工程入口与回归验证 |
| `assets/`、`vendor/` | 展示资产与上游模型 |
| `docs/` | 当前说明；`docs/history/` 保存历史过程 |
| `runs/`、`outputs/` | 持久实验记录与媒体交付，Git 忽略，需独立保管 |

当前 worktree 的 `runs/`、`outputs/` 链接到主仓库数据；模型资产和依赖也有忽略项。
合并代码与移交数据是两项工作，具体清单见本类合并交接。

## 历史练习与实验

单臂键盘 Demo 仍可由 `uv run robofrontier` 或 `Launch Demo.command` 启动；操作、
配置与早期实验保存在 [历史说明](docs/history/readme-before-transport-cleanup-20261027.md)。
其他试点与修订记录见 `docs/history/`、`benchmarks/`。旧成绩绑定原场景和控制来源。
