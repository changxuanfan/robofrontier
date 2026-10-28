# Linux 策略基准准备

此说明针对策略任务；工厂 Cycles 渲染由工厂工作区单独准备。每个任务使用自己的世界、SQLite、会话、日志和60Hz源状态。先准备输入及检查，再单独执行真实模型调用。

## 机器与数据

安装 Python 3.12、uv、Node.js、Codex CLI、MuJoCo 所需的 OpenGL/GLFW 和 ffmpeg。无显示器 Linux 仍需能为 GLFW 建立 OpenGL 上下文；例如在有 Mesa 软件渲染支持的机器上使用 Xvfb。实际 GPU 驱动和显示服务器配置取决于服务器镜像，先做真实截图检查。`--hidden` 只隐藏窗口，不保证不需要显示服务。检查 `codex --version`、`node --version` 和 `ffmpeg -version`，并使用需要访问的 OpenAI 账号登录 Codex。

```sh
uv sync --locked --extra oracle
uv run python scripts/fetch_robot.py
cp .env.example .env
# 在 .env 中手动填写所需厂商凭证和千问工作区地址。
uv run python -m scripts.benchmark models
uv run python -m scripts.tasks check --purpose gpt --assets
```

仓库中的 `.env` 不提交。已有环境变量优先，文件支持 `KEY=value` 和带引号的字面值，不执行 shell。不要把凭证填入模型 JSON、命令行或日志。

Git 不包含实验数据、vendor 和被忽略的场景资产。将保留数据按清单和哈希复制到服务器，特别是 `runs/state-assets/` 及所需 source recording；随后运行 `uv run python -m scripts.restore_scene_assets --report runs/engineering/assets-restore.json`。此恢复命令报告全部历史场景的缺失；随后运行 `uv run python -m scripts.tasks check --purpose gpt --assets` 对当前36项逐个核对工作站资产哈希。所需场景的任何缺失或哈希不符都必须修复后才启动。不要把 Mac 的 `.venv` 拷到 Linux。准备及运行使用同一个服务器目录和源码，冻结包里的绝对路径不可直接从另一台机器迁移。

仅运行当前策略任务时，也可使用[交接指南](multimodel-handoff.md)中的 `benchmark_handoff assets/restore`；该资产包不需要携带历史录制与旧实验数据。

## 预检与启动

```sh
# Paid API calls: image/tool/manual compact and fork test, no robot trial.
uv run python -m scripts.benchmark preflight --models astra luna
# Add deepseek qwen glm kimi after setting those credentials.
uv run python -m scripts.benchmark prepare --models astra luna --families mosaic --output runs/policy/my-batch
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  uv run python -m scripts.benchmark run runs/policy/my-batch --parallel 9
uv run python -m scripts.benchmark status runs/policy/my-batch
uv run python -m scripts.benchmark stop runs/policy/my-batch
```

不指定家族时每个模型准备全部36项，`--parallel` 是同一批次并发任务数，最大36；每个任务内部的 Adaptive fork 会进一步增加模型请求。先以小规模实测 CPU、内存、磁盘、截图响应和厂商额度，再选择9或36。图片调用、60Hz录制和多进程各自消耗资源；当前没有长期36任务负载保证。

每个预检通过后才允许对应配置启动。预检检查图片、工具、手动 compact 和 fork 后继承的信息与工具；不证明自动 compact 阈值、完整272K窗口、实际模型返回身份或长期稳定性。检查配置、账号、CLI或模型变化后重做预检。预检会使用少量 API 额度；`prepare` 不调用模型或推进物理。

日志和轨迹位于各 run 的 `private/outputs/`，工作状态和消息位于 SQLite。START 起每项24小时；容量错误的已确认失败只在没有未决物理请求时续接。未知动作使用原 ID 查询，不重发动作。物理 owner 丢失后不恢复旧世界；保留中断证据并创建新的 run。完整状态流和外观事件供之后在工厂离线回放。
