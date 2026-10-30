# OpenLux GPT-6.1 Terra

此工作树提供 `openlux-terra` profile：模型 `gpt-6.1-terra`、推理强度 `high`、OpenLux `https://api.openlux.ai/v1`、Codex Responses API。默认 Luna profile 保留。OpenLux 模型名和连接配置不证明实际上游模型、视觉、工具、fork 或上下文压缩可用；必须先通过真实预检。当前上下文 272000、压缩阈值 245000 是工程配置，未验证极限。

## 配置 Key

在 OpenLux 控制台为需要的分组设置 API token，在本地 `.env` 添加：

```dotenv
OPENLUX_API_KEY=在本机填写
```

不要把密钥写入 Git、命令行参数或聊天。进程环境的同名变量优先于 `.env`。这个 profile 使用一个 token；分组范围、优先级和跨组重试取决于该 token 的 OpenLux 设置，本程序不负责在两个组间切换，也不能验证实际上游模型。不要把一个分组切换误认为本程序实现了主备。

## 本机检查和短测

运行前安装环境和资产，见 [Luna CPU 指南](luna-cpu.md)。`doctor` 不调用模型；`preflight` 发起付费请求，验证图片、工具、手动压缩与 fork，但不证明自动压缩极限、API 真实上游身份或正式任务可用。

```bash
.venv/bin/python -m scripts.luna_cpu doctor --profile openlux-terra
.venv/bin/python -m scripts.luna_cpu preflight --profile openlux-terra --output runs/policy/terra-smoke
.venv/bin/python -m scripts.luna_cpu submit --profile openlux-terra --output runs/policy/terra-smoke --level 1 --min-arms --wall-limit-s 600 --parallel 4
```

预检通过后才能提交；`submit` 默认 36 项并发且会付费，缩小测试必须显式指定过滤和并行数。CPU 渲染、隔离 Codex home、Motion 和 session 收集方式沿用 [Luna CPU 指南](luna-cpu.md)。现有 Slurm 脚本固定 Luna profile；本配置不能通过直接提交现有脚本切换为 OpenLux。SQLite 节点本地暂存与回传尚未验证，暂不要提交正式集群批次。

也可不调用模型先准备任务，之后 run 从 batch.json 识别 profile：

```bash
.venv/bin/python -m scripts.luna_cpu prepare --profile openlux-terra --output runs/policy/terra-plan --level 1 --min-arms --wall-limit-s 600
.venv/bin/python -m scripts.luna_cpu preflight --profile openlux-terra --output runs/policy/terra-plan
.venv/bin/python -m scripts.luna_cpu run runs/policy/terra-plan --parallel 4
```

预检输出位于 `runs/engineering/model-compatibility/openlux-terra/`。批次中的 `codex-home.json` 给出独立 session 目录；用 `scripts.luna_cpu collect` 收集已停止批次。以上目录含实验记录，不应直接公开分享。

## macOS 原生与 Linux/Docker 一条命令提交

在仓库已安装依赖、资产和 Codex、已配置 `.env` 并通过本模型预检后，以下命令自动检查渲染和沙箱、建立本批次 Codex home、准备并提交任务，不需要 LLM 主 session。默认 OpenLux Terra high、6 个指定 Level 1 任务、并发 6、每任务 START 后 300 秒，实际请求会收费。批次目录必须是新的；命令不会重启已有批次。预检本身也会产生模型费用。

macOS 原生渲染（使用本机原生 OpenGL，不保证 CPU 渲染）：

```bash
.venv/bin/python -m scripts.portable_benchmark --output runs/policy/mac-six-example
```

Docker/Linux CPU 软件渲染：

```bash
docker run --rm --cap-add SYS_ADMIN --security-opt seccomp=unconfined \
  -v "$PWD:/workspace" -w /workspace --entrypoint python robofrontier-luna-cpu \
  -m scripts.portable_benchmark --output runs/policy/docker-six-example
```

上面的容器权限设置会降低隔离，仅在已确认需要的环境中使用。可用 `--parallel`、`--wall-limit-s` 和 `--cases` 调整；执行前须检查服务端并发限制与可用内存。命令不会安装依赖、下载资产或自动执行收费预检；失败时检查提示。Codex home 按批次隔离，批次内任务仍共享；并发配置文件风险尚未完全排除。任务返回 `needs_review` 时必须查看日志，不能自动重放结果不确定的物理动作。
