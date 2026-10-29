# RoboFrontier

> **Transport physics revision:** This branch uses the rolling-friction candidate. See [validation and limitations](docs/transport-cylinder-stability.md) before collecting results; results differ from the original R14 physics.

RoboFrontier 用 MuJoCo 多机械臂任务研究 GPT 的分工、规划与协作。Frontier V2
包含 Transport、Mosaic、Relay、Dial 四类，每类三个规模、三个等级，共36项。
在线任务使用真实接触物理；离线展示读取已保存状态。

## 当前入口

- [四类任务说明图谱 UI](outputs/task-atlas/v1-20261027/index.html)（[离线 ZIP](outputs/exports/task-atlas-four-families-20261027.zip)）
- [各类任务与版本入口](docs/frontier-v2-environments.md)
- [Codex CLI 多模型、并行运行与公开材料](docs/gpt-policy.md)
- [多模型 worktree 交接、本机小测与调度 prompt](docs/multimodel-handoff.md)
- [Claude Code 工程版交接与当前验证范围](docs/claude-handoff.md)；
  [Claude 工程批次主 session prompt](prompts/claude/engineering-manager.md)
- [Linux 服务器策略基准准备](docs/server-benchmark.md)
- [最终视频、数据保留与目录](docs/repository-layout.md)
- [本次合并、数据保全与验收](docs/oracle-integration.md)
- [架构、目录与运行数据](docs/architecture.md)
- [Meta 操作说明](docs/meta-runbook.md)
- [Coordinator／Worker／Controller Skill](.agents/skills/robot-collaboration/SKILL.md)
- [Harness 验收范围](docs/harness-v2-verification.md)
- [工厂展示与渲染](docs/robo-factory.md)

四个Oracle分支已合并。当前用途与版本统一由`configs/task_families/current.json`
选择：GPT使用Transport R14、Mosaic R14、Relay最终Goal规则和Dial R11 BEST5；
Oracle工厂展示使用各录制自己的冻结来源。旧`frontier-v2.json`保留历史生成兼容。

36桌工厂采用独立源剪辑、独立循环、各自倍速与30fps。新GPT使用全新公开上下文，
不继承Oracle答案。当前已加入CLI多模型宿主；本轮没有启动正式模型试验。四个已合并旧任务worktree已清理，当前开发worktree与main保留。

## 安装与工程检查

新 clone 后，安装 Python 环境并运行 `.venv/bin/python -m scripts.benchmark_handoff setup-assets`，
可自动下载、校验并恢复当前 R14 任务资产；离线可使用 `setup-assets --archive /path/to/policy-assets-transport-rolling-20261029.tar.gz`。
资产包和校验说明见[多模型交接](docs/multimodel-handoff.md)。不要将密钥或运行数据提交到 Git。

Python 与依赖由 `.python-version`、`pyproject.toml`、`uv.lock` 管理。
机器人上游资产与许可证由 `scripts/fetch_robot.py` 物化，来源记录在 `vendor/`。

```sh
uv sync --locked --extra oracle
uv run python scripts/fetch_robot.py
uv run --extra oracle python -m scripts.check_harness
```

```sh
.venv/bin/python -m scripts.tasks list
.venv/bin/python -m scripts.tasks check --purpose gpt
.venv/bin/python -m scripts.benchmark prepare --models astra --cases transport-a04-d1 --output runs/policy/my-new-batch
.venv/bin/python -m scripts.factory video --renderer cycles --camera orbit --duration 8 --output outputs/robo-factory/my-gpu-demo
```

准备命令仅冻结输入。并行准备与真实启动见GPT说明；更多入口见
[scripts目录](scripts/README.md)。历史场景资产可按记录哈希恢复：
`python -m scripts.restore_scene_assets --report runs/engineering/asset-restore.json`。

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

当前有效的历史worktree依赖已转到主仓库持久归档。`runs/`、`outputs/`、
`vendor/`和大型场景资产需要随数据根独立备份；仅复制Git仓库不能恢复实验。

## 历史练习与实验

单臂键盘 Demo 仍可由 `uv run robofrontier` 或 `Launch Demo.command` 启动；操作、
配置与早期实验保存在 [历史说明](docs/history/readme-before-transport-cleanup-20261027.md)。
其他试点与修订记录见 `docs/history/`、`benchmarks/`。旧成绩绑定原场景和控制来源。
