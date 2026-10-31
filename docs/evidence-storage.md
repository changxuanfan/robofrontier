# 实验数据收集、压缩与恢复

Codex、Claude、Luna 和多通信分支使用相同的离线收集、归档命令。下列命令不调用模型、不推进仿真、不修改评分或源数据。运行实验仍使用各分支原来的命令。

已执行的测试、真实数据还原结果和验证边界见 [2026-10-30 验证记录](evidence-storage-validation-20261030.md)。

## 停止写入后再收集

在运行任务的同一台机器、同一个容器和相同挂载路径下执行。批次监督程序及各任务必须已结束；不要从 Mac 宿主机读取容器内仍在写入的 SQLite，也不要在 Slurm 作业仍运行时打包。

```sh
.venv/bin/python -m scripts.collect_benchmark_batch runs/policy/MY_BATCH \
  --output runs/exports/MY_BATCH
```

Claude 批次也使用此命令，将输入换为其实际批次目录（例如 `runs/engineering/MY_CLAUDE_BATCH`）。Luna/多通信原来的 `python -m scripts.luna_cpu collect ...` 仍可使用，转调相同收集器。

收集器检查监督程序和 world 的已有写入锁、Codex 宿主 PID、终止状态，复制后验证源文件是否变化；SQLite 检查在一次性副本中进行，保留原始 DB/WAL/SHM 字节。运行时未结束、未决物理动作、缺少任务 Motion、Motion chunk 或源资产哈希异常会列入 issues，不静默忽略。Motion 清单检查不代替完整解码和渲染验证。所有工具都应在任务原运行环境内执行，这些检查无法跨机器或容器可靠判断 PID 和文件锁。

Codex session 优先从批次 `codex-home.json` 定位；无标记的旧批次使用 `CODEX_HOME` 或 `~/.codex`，可显式指定 `--codex-home /path/to/home`。复制登记角色及委派快照关联的 session，不打包整个 Codex home。快照记录在 `jobs[].snapshot_sessions`，不是策略角色；缺少快照时收集结果标记不完整。

Claude 收集原生 `private/claude-config/projects/*/<session-id>.jsonl`、关联的辅助 transcript 和批次内的 `private/evidence/*.jsonl`。原生 transcript 同时保留便于查找的 `sessions/` 索引副本及批次内原有的相对布局，归档时按内容去重。Claude 配置目录的其他内容（包括认证文件）不收集。若需恢复原生 CLI 会话，应另行配置凭据；本功能不保证跨 CLI 版本或迁移路径后的 resume。

新收集格式是 `robofrontier.policy-collection.v1`。`collection.json` 记录文件哈希、模型、拓扑及角色 session 对应关系。Controller、Coordinator、常驻 Worker 和 adaptive delegate 均从运行时角色登记读取，角色名字不硬编码。任务未成功或超时不影响其证据被收集；`complete: true` 表示收集检查通过，不表示任务成功或正式计分资格。

文件夹已存在时不会覆盖。收集退出码 1 表示已生成需要复核的收集结果，请查看 `jobs[].issues`。仍在运行、路径不合法或复制期间有变化会报错，不发布半成品收集目录。源目录里的软链接不会被自动跟随。

## 创建和验证归档

```sh
.venv/bin/python -m scripts.archive_policy_batch create runs/exports/MY_BATCH \
  --output runs/archives/MY_BATCH
.venv/bin/python -m scripts.archive_policy_batch verify runs/archives/MY_BATCH
```

默认要求完整收集。已停止但缺少 session、数据库异常或其他需复核证据也值得保留，可以在检查 `collection.json` 后显式使用：

```sh
.venv/bin/python -m scripts.archive_policy_batch create runs/exports/MY_BATCH \
  --output runs/archives/MY_BATCH --allow-incomplete
```

这不会补齐数据、清除错误或改变评分；归档保持 `collection_complete: false`。不允许借此绕过运行中批次检查。

归档按 SHA-256 将相同内容保存一次，使用 zlib 压缩。JSONL 中图片 URI 以及已知 MCP/Anthropic image 字段的 Base64 图片转为可逆引用；实际图片字节保留。不同 JPG 编码即使看起来相同也不会被当作同一文件。未知格式、非标准或转义编码回退为原文保存，不假设识别成功。原生 transcript、CLI 逐轮事件、评分、过程日志、Motion 和冻结输入都保留。

创建时逐文件验证恢复后的大小和 SHA-256。文件内容、相对路径、普通权限和文件修改时间保留；inode/硬链接身份、文件所有者、访问时间和扩展属性不在保证范围内。归档哈希用于检测损坏，不是第三方签名或来源认证。

旧 `robofrontier.luna-collection.v1` 收集目录及旧图片 URI 归档仍可读取。新归档使用 `jsonl_images_v2` 转换，需要本版本或更新版本的恢复工具；旧工具不能读取新转换。不要混用旧恢复脚本。

## 恢复和使用数据

```sh
.venv/bin/python -m scripts.archive_policy_batch restore runs/archives/MY_BATCH \
  --output runs/restored/MY_BATCH
```

目标必须不存在。全部哈希校验成功才发布恢复目录，失败时移除临时恢复目录。归档的 `objects/` 不能直接当作普通 session 或 Motion 文件夹使用，必须先 restore。

```text
runs/restored/MY_BATCH/
├── collection.json                  # 模型、角色、session、文件哈希清单
├── sessions/<profile>/<case>/        # 角色关联的原生 session JSONL
└── batch/<profile>/<case>/
    ├── public/observations/          # 原观测图片，旧 PNG 或新 JPG
    └── private/
        ├── inputs/                  # 冻结场景、源代码、配置
        ├── evidence/                # Claude CLI 逐轮事件（如有）
        ├── claude-config/projects/  # Claude 原生 transcript 布局（如有）
        ├── state/                   # 原始运行时数据库等证据
        └── outputs/
            ├── motion/              # stream.json、NPZ、事件、source/
            ├── codex-host.jsonl     # Codex 宿主事件（如有）
            └── suite-process.jsonl  # 评分过程数据（如有）
```

无 profile 的历史批次使用 `sessions/<case>/`。记录中的绝对路径保持原值，用于审计；跨机器分析时根据清单中的相对路径定位数据。恢复不是自动重启实验的命令。

## Motion、图片与工厂背景

Motion 保持既有 `robofrontier.motion.v1`：采集时 float64→float32 有既定量化，此后的 XOR 差分与 ZIP 压缩精确还原 float32 状态。本功能不增加量化，不删静止帧，不删时间戳、颜色事件或过程记录；`suite-process.jsonl` 使用普通无损压缩。

新运行观测图默认 JPEG 896×560、quality 85；缺少 profile 的旧冻结运行仍按 PNG 1280×800 解释。公共标定随新观测分辨率生成派生副本，保留原始标定。归档不把历史 PNG 转 JPG、不二次编码 JPG。图片格式变更属于新运行观测设置，不属于无损归档。

恢复后的 Motion 使用已有读取器和回放命令。简单背景使用 `motion/source/`；工厂背景必须另备与录制场景匹配的工厂 scene、manifest 及其纹理/网格资产。资产 release、Mosaic R14/R13、颜色事件及物理版本兼容性由回放器校验，不绕过源场景检查。归档不会自动生成工厂场景，也不会自动打包收集目录外的工厂资产。

可对原收集目录与恢复目录执行额外回放验证：

```sh
.venv/bin/python -m scripts.verify_evidence_replay runs/exports/MY_BATCH \
  runs/restored/MY_BATCH --output runs/engineering/MY_BATCH-replay-check
```

如需工厂检查，增加 `--factory /path/to/matching-factory/scene.xml`。这会逐文件比对哈希，逐帧比较 Motion 的状态和时间戳、比较外观事件，再在同一渲染器内检查简单背景首/中/末三帧；工厂检查比较三帧的几何映射、首末图片并检查生成的短预览视频。预览是分布抽样，不代表连续录像或任务成功。输出 `report.json` 列出实际覆盖的任务和帧数，不应把一个模型或少量采样结果解释为所有模型、所有画面的验收。

Docker 内运行离线命令时可替换默认 entrypoint，例如：

```sh
docker run --rm -v "$PWD:/workspace" -w /workspace --entrypoint python \
  robofrontier-luna-cpu -m scripts.collect_benchmark_batch \
  runs/policy/MY_BATCH --output runs/exports/MY_BATCH
```

归档与恢复命令同样在 `--entrypoint python` 后使用 `-m scripts.archive_policy_batch ...`，保持相同挂载路径。无需向这些离线命令传入模型 API Key。

## 空间与隐私

归档不会自动删除原批次、原 session 或 exports，也不改变 API 输入、缓存或计费。压缩节省体现于最终归档，不减少运行中数据，也不保证源数据和收集副本同时保留时总占用下降。

需要预留收集副本、归档、系统临时文件的空间；恢复还需要完整原始大小。归档按文件流式压缩，但 JSONL 图片处理的内存取决于最大单行及图片大小，校验临时磁盘取决于最大单文件。36 项批次先完成一次 collect，再顺序归档整个批次，避免同时启动 36 个归档进程。

所有证据含模型轨迹、私有任务输入和运行时连接信息，应私有存储。不要上传到公开仓库；CLI 认证目录排除不等于整个数据已经匿名化。只有独立备份、归档验证和恢复使用均确认后，才自行决定清理哪些源副本；这些命令本身不删除源数据。
