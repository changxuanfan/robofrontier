# Luna CPU benchmark：Linux / Slurm 交接

本分支使用 **GPT-6 Luna、high、OpenAI API Key、adaptive**。默认选择全部 36 项、最多 36 项并发，每项开始后限时 86400 秒（24 小时）。Controller 可自行增减子代理，模型请求并发可能高于任务数。没有监督模型的主 session 也可以通过命令运行；运行会产生 API 费用。当前任务资产是 **Mosaic R14**。

36 项持续并发尚未在目标 Linux 服务器验证。运行前确认 CPU、内存、磁盘、API 额度及费用预算；全部任务并发不等于它们一定同时开始或一定成功。后台保持完整记录不等于自动备份。

## 1. 获取代码并准备环境

取得本分支后，在集群允许存放大文件的 Lustre 目录 clone 并进入仓库根目录。不要将大型数据写入有配额的 home。准备 Python 3.12、Node.js、Codex CLI、Mesa OSMesa（Debian/Ubuntu 的 `libosmesa6`），并确认允许 Codex Linux sandbox 使用所需用户命名空间。依赖和网络环境由集群管理员决定。

```sh
python3.12 -m venv .venv
.venv/bin/pip install .
cp .env.example .env
chmod 600 .env
# 在 .env 中自行填写 OPENAI_API_KEY，不要提交、发消息或打印密钥。
```

自动获取并验证 GitHub Release 的 R14 资产（约 186 MB），恢复后再次校验各资产；已有正确资产则直接复用：

```sh
.venv/bin/python -m scripts.luna_cpu setup-assets
.venv/bin/python -m scripts.luna_cpu doctor
```

`setup-assets` 也可不通过 Luna 包装器独立使用：`python -m scripts.benchmark_handoff setup-assets`。离线机器可先从 [Release](https://github.com/changxuanfan/robofrontier/releases/tag/handoff-20261028) 下载 `policy-assets.tar.gz`，再传 `setup-assets --archive /path/to/policy-assets.tar.gz`。脚本固定检查发布包 SHA-256、任务发布清单和所有文件；不同内容或损坏时停止，不覆盖已有不同资产。缓存放在忽略 Git 的 `runs/asset-cache/`。下载需要网络，离线资产要由用户提前传入。与 Release 清单不匹配的其他分支不能使用本版本配置。

## 2. 在 Slurm 排队

准备阶段执行 `setup-assets`，以便计算节点无法联网 GitHub 时仍可复用资产。以下仅是提交格式，资源数值是**示例，不是 36 项所需配置**；请根据集群规则、排队限制和测试结果修改。每项的 24 小时从 START 开始，Slurm 作业时限还必须留出预检、启动和收尾时间，过短会中断任务。

```sh
sbatch --partition=YOUR_PARTITION --cpus-per-task=YOUR_CPUS --mem=YOUR_MEMORY \
  --time=YOUR_JOB_TIME scripts/slurm_luna_cpu.sh
squeue -u "$USER"
```

作业启动后脚本检查 / 恢复资产，运行环境 doctor，进行**付费模型预检**，然后启动**付费任务**。输出默认是 `runs/policy/luna-slurm-<Slurm作业ID>/`；Slurm 标准输出默认由站点配置决定（通常在提交目录的 `slurm-<作业ID>.out`）。不设置 GPU 申请，OSMesa / llvmpipe 用 CPU 渲染，MuJoCo 物理用 CPU。使用 `scancel JOB_ID` 取消作业时可能中断尚未写完的数据。

脚本可通过环境变量覆盖：`RF_BATCH_OUTPUT`、`RF_WALL_LIMIT_S`、`RF_PARALLEL`。例如小额工程测试可设 `RF_WALL_LIMIT_S=600 RF_PARALLEL=4`，但并发 4 默认仍会选 36 项；若只想跑 4 个最简单 Level 1 任务，使用下面手动命令的 `--level 1 --min-arms`。集群对环境变量传递有不同配置，请确认 `sbatch --export` 的站点规则。

## 3. 手动运行及查询

每批需使用一个未存在的输出目录。`preflight --output`、`submit --output` 使用相同路径，自动选择相同的私有 Codex home。更换 Codex CLI 或模型配置后必须重新预检。预检需调用 API；`doctor` 不调用模型。

```sh
.venv/bin/python -m scripts.luna_cpu preflight --output runs/policy/luna-all-36
.venv/bin/python -m scripts.luna_cpu submit --output runs/policy/luna-all-36
```

要做 4 项、每项 10 分钟的工程短测：

```sh
.venv/bin/python -m scripts.luna_cpu preflight --output runs/policy/luna-smoke
.venv/bin/python -m scripts.luna_cpu submit --output runs/policy/luna-smoke --level 1 --min-arms --wall-limit-s 600 --parallel 4
```

另一终端可查看、统计或请求优雅停止：

```sh
.venv/bin/python -m scripts.luna_cpu status runs/policy/luna-all-36
.venv/bin/python -m scripts.luna_cpu usage runs/policy/luna-all-36
.venv/bin/python -m scripts.luna_cpu performance runs/policy/luna-all-36
.venv/bin/python -m scripts.luna_cpu stop runs/policy/luna-all-36
```

## 4. 保存和收集

每个新批次自动使用 `.codex-luna/<批次名>-<路径哈希>/sessions/`；`runs/policy/<批次名>/codex-home.json` 记录确切位置。它不同于服务器账号默认 `~/.codex`。运行数据位于批次目录，其中包括截图、SQLite 状态、宿主日志、评估和运动记录；默认不录实时视频。以上目录均被 Git 忽略，必须单独备份。

批次已关闭后，可复制批次数据和按角色关联的 session 到新目录，输出 `collection.json` 标识缺失 session 或评估：

```sh
.venv/bin/python -m scripts.luna_cpu collect runs/policy/luna-all-36 --output runs/exports/luna-all-36
```

收集目录包含实验私有内容；不要公开分享，且仍需另存备份。该检查不保证评分正确，也无法识别运行中从未登记的 session。真实多子代理及 36 项长期运行还需实测。`usage` 根据 session 去重统计 token，报告缺失 trace，不等于账单；`performance` 的模型轮次包含工具等待，不能当作纯 API 延迟。没有按进程连续采集 CPU/RSS；可在 Slurm 配置支持时通过 `sacct -j JOB_ID --format=JobID,State,Elapsed,AllocCPUS,MaxRSS,TotalCPU` 查看站点报告，具体统计范围由集群决定。

## 5. Docker（可选）

如果服务器支持 Docker，可在代码目录构建镜像：

```sh
docker build -f docker/luna-cpu.Dockerfile -t robofrontier-luna-cpu .
```

以下将整个工作区挂载到容器，**自动创建的批次 Codex home 位于该挂载下**，不会使用服务器默认 Codex home；容器退出后数据仍在工作区。必须为所有命令使用同一个容器内路径 `/workspace`，否则已冻结的绝对路径可能失效。API key 从挂载的 `.env` 读取。

```sh
docker run --rm -v "$PWD:/workspace" -w /workspace robofrontier-luna-cpu setup-assets
docker run --rm -v "$PWD:/workspace" -w /workspace robofrontier-luna-cpu doctor
docker run --rm -v "$PWD:/workspace" -w /workspace robofrontier-luna-cpu preflight --output runs/policy/luna-all-36
docker run --rm -v "$PWD:/workspace" -w /workspace robofrontier-luna-cpu submit --output runs/policy/luna-all-36
```

在 macOS 的 Docker，默认 seccomp 可能阻止 Codex sandbox：此前测试需给上述各命令增加 `--security-opt seccomp=unconfined`。这会降低容器隔离；Linux 集群先用默认配置检查，若失败由管理员确认可用方案。容器用户可能在宿主机创建不同所有者的文件。并非所有 Slurm 站点提供 Docker；上面的 Slurm 脚本使用原生 Linux Python 环境。
