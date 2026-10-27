# GPT任务准备与并行运行

入口为`python -m scripts.tasks`，统一清单为`configs/task_families/current.json`，
控制配置为`configs/harnesses/policy.json`。默认adaptive、gpt-6.1-sol/high、pd1000-v1；
初始只有Controller，后续委派由它自己选择。Coordinator/Worker及peer模式仍可通过
明确的harness选择。角色协议见[Skill](../.agents/skills/robot-collaboration/SKILL.md)。

## 准备

从仓库根运行，输出目录必须不存在：

```sh
.venv/bin/python -m scripts.tasks check --purpose gpt
.venv/bin/python -m scripts.tasks prepare transport-a04-d1 --output runs/policy/new-transport
.venv/bin/python -m scripts.tasks prepare-batch --output runs/policy/new-batch
# 可选家族：--families transport mosaic
```

这些命令不启动物理、原生会话或模型。每项有独立run目录、SQLite、连接凭据、
工作笔记、记录器和服务；并行启动时各服务采用独立自动分配端口。不要把多个任务
绑定到同一Controller或同一Meta调度循环。`batch.json`记录已准备项，失败保留现场，
不用同目录重试覆盖。

## 公开材料和资格

`configs/task_families/<family>/public/<case>/`保存经过审查的公开材料副本及原哈希。
准备时复制到`public/inputs/family/`，公共任务指向家族说明、静态几何与相机材料。
Meta使用私有输入和固定调度器；策略仅使用自己的公开输入、被分配工具和运行反馈。
Oracle答案、配对、轨迹、私有评估和旧实验经验不加入策略上下文。

原始场景task与readiness保持原字节；新run的`prepared_policy_trial`协议明确绑定
release、源scene/task、证据资格与全新经验边界。START再次校验冻结release与源元数据；
正常完整性检查还校验公开材料、harness和执行源码。Oracle验证标签不改为true。
原20分钟probe预算不继承；本配置没有总试验墙钟期限，动作及基础设施期限仍有效。
实际试验预算由启动协议明确决定。

## 真实启动条件

依据[Meta操作说明](meta-runbook.md)，使用干净新会话，固定model/effort并验证
实际宿主支持。Meta需要原生create、wait、send能力，adaptive还需要fork与谱系核验。
本次工程会话未提供这些原生会话工具，工具发现记录在
`runs/engineering/oracle-integration-20261027/native-capabilities.json`；因此没有声称
原生往返或真正并行GPT已经验收，也没有启动模型。

离线检查覆盖四类冻结场景的START、评估器和60Hz记录器，以及FakeWorld/
调度/委派/执行占用回归；这些结果不证明真实模型表现或并行协作收益。
源级回放循环只属于展示，正式策略试验不会循环重开。

Codex本机配置为400000上下文、350000自动压缩；当前会话显示的有效窗口为380000。
配置上限不等于服务端对任意模型承诺的可用窗口。
