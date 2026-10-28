# Claude harness: development status

This branch is under development. It does **not** admit scored tasks. Its
experimental engineering runner can start a simulator and paid inference when
explicitly invoked; provider profiles are candidates, not compatibility claims.
The existing Codex benchmark remains separate.

For an agent-independent orchestration prompt, see
[Claude engineering manager](../prompts/claude/engineering-manager.md).
The older [multimodel handoff](multimodel-handoff.md) describes the Codex
runner and its asset-bundle recovery, not Claude command names or readiness.

After `uv sync --locked --extra oracle`, use:

```sh
.venv/bin/python -m scripts.claude_benchmark models
.venv/bin/python -m scripts.claude_benchmark doctor --models haiku opus kimi qwen glm deepseek
.venv/bin/python -m scripts.claude_benchmark plan --models kimi --level 1 --min-arms --parallel 4
.venv/bin/python -m scripts.claude_benchmark prepare-draft --models kimi --level 1 --min-arms --output runs/engineering/claude-draft
.venv/bin/python -m scripts.claude_benchmark prepare-draft --models opus --level 1 --min-arms --record-video --output runs/engineering/claude-video-draft
.venv/bin/python -m scripts.claude_offline_probe
```

Real precheck (spends money; needs a credential and an explicit cap):

```sh
.venv/bin/python -m scripts.claude_precheck --profile haiku --max-budget-usd 0.5
.venv/bin/python -m scripts.claude_precheck --profile haiku --max-budget-usd 0.5 --confirm-spend
.venv/bin/python -m scripts.claude_precheck --profile haiku --max-budget-usd 0.5 --check-fork --confirm-spend
```

Without `--confirm-spend` it only prints the plan. With it, it makes three
calls on one explicit session: a text reply, a robot tool call whose returned
image must be identified, and a resumed turn. The receipt is `receipt.json`
in the output directory; it records CLI version, requested and reported model,
cost and per-check results, never key values. A profile passes only if all
three checks pass within the cap. Bench runs use `opus`, not the Haiku
precheck profile.

`--check-fork` adds a fourth paid call: a tool-free native fork of the idle
session that must recall the image color and have a matching native transcript
prefix. The receipt includes a profile fingerprint and whether the fork was
checked. This idle-session check does not establish compatibility with a parent
blocked in a tool call, concurrent delegate execution, or a real robot task.
The cap is based on CLI-reported cost and is not a provider billing limit.

The doctor checks locally only. It reports whether credentials are present,
not whether they work. If a sandbox denies the provider host or writes under
`runs/`, allow it in your own environment rather than bypassing the sandbox.
For local credentials copy `.env.claude.example` to
`.env` with permissions 600; existing environment variables take priority.
Do not commit credentials. The Anthropic-compatible Qwen endpoint must match
your actual account and region. Live preflight status:

- `haiku` (`claude-haiku-5-5`, CLI 2.1.312) passed on 2026-10-28 with a total
  cost of $0.000541 under a $0.5 cap. Its receipt is local at
  `runs/engineering/claude-precheck/haiku-20261028-194259/receipt.json`
  (`runs/` is gitignored). This covers only that profile, CLI version and cap.
- `opus` and the other providers have not been preflighted.

`robofrontier.claude.robot_mcp` is an experimental role-bound MCP server. Its
offline probe checks Claude CLI tool discovery, simulated tool-image forwarding and
fork history against local stub servers. It does not validate real provider image
handling, trusted session provenance, dynamic delegation or production physical runs.
Do not launch it against an actual scored world. The draft prepare command freezes inputs for engineering inspection only; it is
not a scored batch. Experimental engineering run/status/stop commands exist, but
the real-provider preflight and full runner still require qualification; the Codex-to-Claude
component map and verified CLI facts are in [claude-codex-parity.md](claude-codex-parity.md).
Mosaic remains the released R14 scene in this branch.

## 接手进度：离线策略主机开发

本轮继续仅使用本地假模型与假机器人服务，没有调用真实 API 或启动正式试验。
`robofrontier/claude/session_host.py` 管理一个角色的原生 CLI 回合：固定角色 MCP、
唯一会话 ID、明确续接、相互排斥、回合证据和停止请求。`robofrontier/claude/dispatch.py`
使用现有运行时的调度规则处理已绑定的 centralized Controller。目前已有实验性
启动、批次、停止和封存代码，**尚未完成正式 runner 的实际环境验证**。

Claude CLI 2.1.312 使用本地假服务通过了角色会话启动、工具图片返回和续接测试；
这些不是模型能力或厂商兼容验证。任务世界丢失、进程崩溃后的监督恢复和真实物理
集成也尚未通过端到端测试。Claude adaptive 委派在 runtime 中明确禁用：转录前缀
相同只是历史匹配证据，不能单独证明原生 fork 创建过程或角色授权，必须先完成可信
绑定验证才可开放。Codex 的现有委派入口不受此限制。

Claude 工程草稿现会在 harness 与 manifest 均标记 `engine: claude`，但仍是
`prepared_not_qualified`，不可用于正式运行。上一个会话报告的两组测试因缺失的旧
`mosaic/four-arm-10x10/assets` 而失败；本机从 main 工作树的同一场景版本恢复该目录后，
`test_execution_contract.py` 与 `test_robot_actions.py` 已通过。这不证明历史资产全部齐备。

目前还新增了集中式工程接入 `robofrontier/claude/policy_host.py`：只接受已准备且输入
通过校验的 centralized Claude 世界，绑定 Controller、加载公开材料并检查 READY/idle。
调用方仍须负责启动世界、明确发起 START、停止和封存；没有暴露正式批次运行命令。
策略会话关闭了文件读取工具，因此 `bootstrap.py` 将公开文本材料打包，并通过
`robot` 的 `public_image` 操作返回 task.json 显式列出的初始参考图。该操作限制在
当前 run 的 public 目录，仍需实际任务级验证。

离线测试还将集中式策略主机接到真实 `RuntimeService` 和调度器，使用合成世界及
本机回环服务验证绑定、READY、显式 START 和调度；这不等于真实 MuJoCo 或真实模型测试。
`prepare-draft` 可选 `--topology centralized` 来准备集中式工程草稿，默认仍是 adaptive。
两种草稿都不具备正式运行资格，也不会自动启动任务。四个 Level 1 最少臂集中式草稿
已在临时目录通过冻结输入校验和公开材料加载检查。

`robofrontier/claude/supervisor.py` 新增已有世界上的集中式工程监督循环：等待 READY、
显式 START、持续调度，遇到启动超时、停止请求、世界丢失、未知会话状态或墙钟上限
时停止模型并请求现有 seal。seal 不成功时保留错误，不能报告运行已安全关闭。它仍不
负责创建世界或调度批次，尚不是正式运行入口。当前 runner 使用 `--bare`；本机 Claude
CLI 的帮助说明该模式不读取 OAuth 或钥匙串，所以目前只能使用 API 密钥路线，不能声称
可以直接复用 Claude Desktop 登录。订阅登录需另行设计、隔离和验证。

`robofrontier/claude/engineering_run.py` 进一步连接了工程草稿校验、一次性启动记录、
独立物理进程与监督循环；它只接受未认证的 centralized 工程草稿，尚无正式批次 CLI。
异常不重启世界，也不强杀可能仍在执行的物理进程；进程仍活着或封存未确认时标记待检查。
`turn_budget_usd` 是 Claude CLI 每个回合的限制。任务还设置一个 CLI 报告费用的
累计阈值：后续回合按报告的剩余额度启动；费用缺失、零值或无效时停止并要求检查。
这是估算控制，第三方厂商可能不提供可靠成本，账单可能与 CLI 报告不同，也可能超出
阈值；它不是服务商的硬性扣费上限或整批共享预算。调用此代码可能产生 API 费用，
本轮测试仅使用替身，没有实际调用该启动过程或付费 API。

`robofrontier/claude/engineering_batch.py` 增加了实验性集中式工程批次的内部接口：
全部任务预检后按设置的数量并发，支持只读状态和停止请求，监督者失联及异常任务
标记待检查，批次和单任务都不自动重启。相关测试用替身验证了这些调度行为。它还没有
开放正式计分批次 CLI，也没有证明真实模型、MuJoCo、多角色委派或服务商账单控制可用。

工程命令（支持 centralized 和 adaptive 草稿；需要本机 API Key；可能产生费用）：

```sh
.venv/bin/python -m scripts.claude_benchmark run-engineering RUN_DIR --parallel 2 --turn-budget-usd 0.1 --task-budget-usd 1 --wall-limit-s 1200
.venv/bin/python -m scripts.claude_benchmark status-engineering RUN_DIR
.venv/bin/python -m scripts.claude_benchmark stop-engineering RUN_DIR
```

第一条缺少 `--confirm-engineering-spend` 时只报告不会启动；加上该选项才实际调用。
示例额度不是价格预测；四个任务的每任务额度各自独立，不能当成共享的批次总上限。

自适应工程路径现已连接启动器和批次接口；父角色、fork 设置和子角色共用单任务
SQLite 费用估算账本。真实 Claude CLI 通过本机合成模型与合成世界测试了 READY、
START、委派、fork、子角色返回及机器人控制权恢复。原生记录写入有时晚于 MCP 请求，
现在创建 fork 前等待对应委派调用被记录，超时则暂停并要求人工检查。
这些测试不验证真实模型、服务商兼容性、MuJoCo 物理、计分、账单或整个批次的总费用；
adaptive 仍属于未认证工程运行，不能用作正式 benchmark 结果。

另外，可在已恢复 R14 资产且支持本机图形的环境中运行可选物理烟雾检查：

```sh
ROBOFRONTIER_PHYSICS_SMOKE=1 .venv/bin/python -m unittest discover -s tests -p test_claude_physics_smoke.py -v
```

该检查使用真实 MuJoCo 进程和 Claude CLI，但模型服务只绑定本机回环，使用合成密钥，
不访问真实 API。默认检查 transport Level 1 工程草稿；可用环境变量
`ROBOFRONTIER_PHYSICS_CASE` 选择其他已发布 case。它验证自适应委派与返回、墙钟停止、
封存和进程退出。它还检查相机图片到达模拟模型端，并通过子角色对 A 臂执行 5 mm 的
仿真位移；不验证复杂操作、模型能力或正式计分。2026-10-28 本机分别对四类
最少臂 Level 1（transport、mosaic R14、relay、dial）通过了该测试，默认关闭录像。
要额外在 transport 草稿检查录制与封存时的首帧解码，设置 `ROBOFRONTIER_PHYSICS_VIDEO=1`
并运行相同命令；本机已通过。这只检查首帧，并不证明视频全程完整。

`prepare-draft --record-video` 在新草稿中开启录像，需要 FFmpeg 和 ffprobe。工程封存
会尝试验证视频流并解码首帧；录像被要求而无法验证时，任务标为 `needs_review`。
默认草稿不录像，封存报告提示视频缺失是预期结果。

四项并发工程物理烟雾检查（需要更多 CPU、内存和图形资源）：

```sh
ROBOFRONTIER_PHYSICS_BATCH=1 .venv/bin/python -m unittest discover -s tests -p test_claude_parallel_physics_smoke.py -v
```

该检查在同一批中使用四个相互独立的物理进程，检查它们确实同时存活，并分别核对
委派、短位移和封存。模型仍是本机合成服务；测试不证明厂商并发、速率限制或成本。
