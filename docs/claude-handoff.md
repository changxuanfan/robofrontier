# Claude harness: development status

This branch is under development. It does **not** admit scored tasks. Its
experimental engineering runner can start a simulator and paid inference when
explicitly invoked; provider profiles are candidates, not compatibility claims.
The existing Codex benchmark remains separate.

For an agent-independent orchestration prompt, see
[Claude engineering manager](../prompts/claude/engineering-manager.md).
The older [multimodel handoff](multimodel-handoff.md) describes the Codex
runner and its asset-bundle recovery, not Claude command names or readiness.

## Claude 分支接手步骤

当前 `codex/claude-code-harness` **尚未发布到 GitHub**，所以现在不能直接从 GitHub
检出它；下列 clone 命令在该分支发布、接收者获得仓库访问权限后才可用。其他分支
可能仍使用 Codex 路线，不能把它们当成这个 Claude 分支。

```sh
git clone --branch codex/claude-code-harness https://github.com/changxuanfan/robofrontier.git robofrontier-claude
cd robofrontier-claude
git rev-parse --short HEAD
```

在 macOS 或 Linux 准备 Python 3.12、`uv`、Claude Code CLI 和可用的图形环境；
录像还需 FFmpeg 和 ffprobe。已验证的本机 CLI 为 2.1.312，版本变化后需要重新
验证。Claude 路线本身不需要安装 Codex CLI；外层调度 session 可以使用具备终端
访问的其他 coding agent。下面先安装依赖，并从已发布的 R14 资产包恢复资源：

```sh
claude --version
uv sync --locked --extra oracle
mkdir -p ../robofrontier-assets
gh release download handoff-20261028 --repo changxuanfan/robofrontier --pattern 'policy-assets.tar.gz*' --dir ../robofrontier-assets
(cd ../robofrontier-assets && shasum -a 256 -c policy-assets.tar.gz.sha256)
.venv/bin/python -m scripts.benchmark_handoff restore ../robofrontier-assets/policy-assets.tar.gz
```

Linux 可将 `shasum -a 256` 换成 `sha256sum`。没有 `gh` 时可从同仓库的
`handoff-20261028` Release 下载两个同名资产文件，核对 SHA-256 后执行 restore。
包内资产版本为 R14，Mosaic R15 不在本分支。恢复程序还校验资产清单和文件哈希。

在仓库根目录创建个人 `.env`，以下命令不会覆盖已有文件；只在自己的机器填写
相应变量，不要在聊天中粘贴密钥或把密钥提交到 Git：

```sh
.venv/bin/python -c 'import os; from pathlib import Path; fd = os.open(".env", os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600); os.write(fd, Path(".env.claude.example").read_bytes()); os.close(fd)'
```

`opus` 使用 `ANTHROPIC_API_KEY`；国产模型分别读取模板中的对应变量，千问还需
按账号和区域填写 Anthropic 兼容 endpoint。`opus_subscription` 是尚未使用真实订阅
验证的候选配置。模板的空值不构成登录或可用性证明。环境变量优先于 `.env`。
如果 `.env` 已存在，检查并编辑原文件，不要覆盖；不要将创建命令的失败当成认证失败。

百炼 [Coding Plan 使用说明](https://help.aliyun.com/zh/model-studio/coding-plan) 明确限制
非交互式批量 API 调用；本 harness 会由外层进程自动并发启动多个 CLI，会话用途与
套餐允许范围须向供应商确认。不要因为它支持 Claude Code 就假定可用于此批量任务；
应使用允许该用途的账号与服务，例如核实后采用按量计费接口。套餐和按量计费的
密钥、endpoint 不能混用，见[百炼 Claude Code 接入说明](https://help.aliyun.com/zh/model-studio/claude-code)。
DeepSeek profile 使用的 Anthropic 兼容地址与 [DeepSeek 官方接入说明](https://api-docs.deepseek.com/quick_start/agent_integrations/claude_code/)一致；
模型映射、视觉和工具往返仍需对实际账号和所选 profile 预检，不能只凭地址判断通过。
`kimi` 保留原 Moonshot API 候选设置；新增 `kimi_code` 单独使用
`KIMI_CODE_API_KEY`、国内 Kimi Code 端点和 `k3.1-pro`。根据 [Kimi Code 官方说明](https://www.kimi.com/code/docs/en/third-party-tools/claude-code.html)，
该模型与可用上下文取决于会员等级，并且仅支持思考模式；海外账号应核实官方区域端点。
这两个候选 profile 均未在本工程做真实预检，也不能假定 Kimi Code 套餐允许此类批量
benchmark；请先确认账号权限、用途和额度，再选择 profile 进行授权预检。
API Key 模式将所选厂商密钥传给 Claude 的 `ANTHROPIC_API_KEY`；本机 CLI 2.1.312
的回环测试确认请求使用 `x-api-key`。百炼的[兼容接口文档](https://help.aliyun.com/zh/model-studio/anthropic-api-messages)
和 [DeepSeek 兼容接口文档](https://api-docs.deepseek.com/zh-cn/guides/anthropic_api/)
均列出对该请求头的支持；这不构成真实账号认证、其他厂商兼容性或完整任务通过的证明。

以下离线命令不启动世界、不调用模型：

```sh
.venv/bin/python -m scripts.claude_benchmark doctor --models opus
.venv/bin/python -m scripts.claude_benchmark plan --models opus --level 1 --min-arms --parallel 4
```

doctor 只检查凭据是否存在及本机资产等条件；plan 只列出四项任务。先确认这些
结果，再根据后文进行已获授权的付费预检。无论预检结果如何，当前版本仍不能生成
正式计分 benchmark。可把 [Claude 工程批次主 session prompt](../prompts/claude/engineering-manager.md)
交给外层 agent，明确模型、任务、并发和预算。

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

For an additional paid check of the adaptive fork boundary:

```sh
.venv/bin/python -m scripts.claude_blocked_precheck --profile opus --max-budget-usd 1
.venv/bin/python -m scripts.claude_blocked_precheck --profile opus --max-budget-usd 1 --confirm-spend
```

It uses a synthetic local robot service and three CLI turns: initialize the
parent, block the parent at an image tool call, and create a tool-free fork
while that call is blocked. It verifies the native history prefix, releases the
parent, and checks its image response. One turn can generate multiple provider
requests. The two concurrent turn allowances share the remaining estimated
budget. A receipt with `cost_complete: false` does not account for every turn;
neither this check nor the CLI-reported budget proves a provider billing cap,
physical execution, or scored-run qualification. The full path has passed
only against a local fake provider; real profiles remain unverified.

After obtaining both real receipts, compare them with the current profile and
CLI version without making model calls:

```sh
.venv/bin/python -m scripts.claude_readiness --profile opus --precheck /path/to/receipt.json --blocked-precheck /path/to/blocked/receipt.json
```

This checks required observations, session relationships, cost completeness,
model/profile fingerprint and CLI version. `precheck_evidence_matches` is only
receipt consistency: it cannot authenticate a receipt, distinguish accounts
sharing the same profile, establish provider model identity, or authorize a
scored benchmark. `formal_execution_available` remains false.

The doctor checks locally only. It reports whether credentials are present,
not whether they work. If a sandbox denies the provider host or writes under
`runs/`, allow it in your own environment rather than bypassing the sandbox.
For local credentials copy `.env.claude.example` to
`.env` with permissions 600; existing environment variables take priority.
Do not commit credentials. The Anthropic-compatible Qwen endpoint must match
your actual account and region.

The runner redacts exact occurrences of the selected credential (including
JSON-escaped forms) from its event file, summary and stderr tail, and writes
event files with owner-only permissions. This does not sanitize Claude's native
session files or detect transformed/partial secrets; treat the entire run
directory as private and inspect it before sharing.

Live preflight status:

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

On 2026-10-28, a fresh local clone at commit `aa0473d` independently created
its Python environment with `uv sync --locked --extra oracle`, downloaded the
published `handoff-20261028` asset archive, verified its SHA-256 and restored
3,467 files. It planned and prepared the four minimum-arm Level 1 Opus drafts;
all four frozen-input checks returned no changes. The Claude test suite ran
104 tests successfully with two opt-in physical tests skipped. The clone came
from the local branch, not GitHub, and no real provider or scored run was tested.
The same clone subsequently passed the opt-in four-way physical smoke: four
distinct MuJoCo owners overlapped, camera images reached a synthetic loopback
model, each delegate completed its synthetic motion and returned control, and
all four worlds sealed. This is an engineering integration check, not evidence
of real model capability or scored benchmark qualification.

At Claude commit `85f9b91`, a Git merge preview against GitHub `main` at
`c8b4137` reported no textual conflicts; this is not a completed merge and does
not prove behavioral compatibility. The branch descends from the existing
`codex/multimodel-handoff` branch, so its eventual merge also includes that
branch's changes. Shared engine-neutral and Codex CLI benchmark tests passed
(8 and 10 tests), as did 24 Node dispatcher scenarios. Recheck the actual
target branch and rerun relevant tests before merging.

## 接手进度：离线策略主机开发

本轮继续仅使用本地假模型与假机器人服务，没有调用真实 API 或启动正式试验。
`robofrontier/claude/session_host.py` 管理一个角色的原生 CLI 回合：固定角色 MCP、
唯一会话 ID、明确续接、相互排斥、回合证据和停止请求。`robofrontier/claude/dispatch.py`
使用现有运行时的调度规则处理已绑定的 centralized Controller。目前已有实验性
启动、批次、停止和封存代码，**尚未完成正式 runner 的实际环境验证**。

Claude CLI 2.1.312 使用本地假服务通过了角色会话启动、工具图片返回和续接测试；
这些不是模型能力或厂商兼容验证。任务世界丢失和进程崩溃后的监督恢复仍未完成
端到端验证。Claude 的通用 `delegate-bind` 入口保持禁用；实验性的内部 adaptive
委派已连接并经过本机模拟模型和 MuJoCo 烟雾测试。转录前缀相同只是历史匹配证据，
不能单独证明原生 fork 创建过程或角色授权。Codex 的现有委派入口不受此限制。

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
负责创建世界或调度批次，尚不是正式运行入口。

API Key profile 使用 `--bare`；Claude CLI 不会在该模式读取 OAuth。现有候选 profile
`opus_subscription` 使用 `CLAUDE_CODE_OAUTH_TOKEN`，非 bare 启动时关闭用户/项目设置
源、MCP 自动发现和工具搜索；仅允许凭据发送到 Anthropic 官方 endpoint。它使用独立
会话目录，不会自动接管 Claude Desktop 的登录。按照 [Anthropic 认证文档](https://code.claude.com/docs/en/authentication)，
用户可自行在安全终端运行 `claude setup-token`，把生成的 token 作为敏感凭据配置到
`.env` 或环境变量中，不要贴到聊天或提交 Git。托管策略仍可能影响 CLI；不同启动方式
的系统提示可能不同。候选路径只在真实 CLI 加本地假 token/假模型下验证了图片、续接、
fork 和项目配置隔离；真实订阅及其成本报告仍未验证。费用不明时账本会暂停，不能将
CLI 报告视为订阅额度或实际账单的硬上限。

`robofrontier/claude/engineering_run.py` 进一步连接了工程草稿校验、一次性启动记录、
独立物理进程与监督循环；它只接受未认证的 centralized 工程草稿，尚无正式批次 CLI。
异常不重启世界，也不强杀可能仍在执行的物理进程；进程仍活着或封存未确认时标记待检查。
封存证据必须明确列出空的未解决动作列表，否则即使进程已退出也标记待检查。
`turn_budget_usd` 是 Claude CLI 每个回合的限制。任务还设置一个 CLI 报告费用的
累计阈值：后续回合按报告的剩余额度启动；费用缺失、零值或无效时停止并要求检查。
这是估算控制，第三方厂商可能不提供可靠成本，账单可能与 CLI 报告不同，也可能超出
阈值；它不是服务商的硬性扣费上限或整批共享预算。调用此代码可能产生 API 费用，
本轮测试仅使用替身，没有实际调用该启动过程或付费 API。

`robofrontier/claude/engineering_batch.py` 增加了实验性集中式工程批次的内部接口：
全部任务预检后按设置的数量并发，支持只读状态和停止请求，监督者失联及异常任务
标记待检查，批次和单任务都不自动重启。相关测试用替身验证了这些调度行为。它还没有
开放正式计分批次 CLI，也没有证明真实模型、MuJoCo、多角色委派或服务商账单控制可用。
如果批次已有启动占用记录但没有状态文件，查询会显示
`claim_without_status_needs_review`；它可能在启动过程中中断，不能作为未启动批次重跑。
停止发生在启动前时，状态为 `stopped_before_start`；停止导致有任务未启动或已启动
任务按停止请求封存时，批次为 `stopped`。若任何任务需要检查，批次优先显示
`needs_review`。`closed` 只描述工程批次结束，不代表任务成功或正式计分通过。
隔离测试中实际终止过运行替身任务的监督进程：查询能显示
`supervisor_lost_needs_review`，保留领取记录并拒绝重跑；此测试没有启动物理世界，
不证明监督进程崩溃后真实物理子进程已停止或完成封存。
`status-engineering` 同时展示批次和各任务的 `stop_requested`，表示停止标记是否
存在；监督者失联时，标记存在不代表物理进程已经接收请求、退出或封存。

工程命令（支持 centralized 和 adaptive 草稿；需要相应 profile 的凭据；可能产生费用）：

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
CLI 非零退出时，即使输出含费用数字，共享账本也将该回合标为不确定并停止新的预算分配。
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
