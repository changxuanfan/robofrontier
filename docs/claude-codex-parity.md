# Codex 到 Claude Code：对照表与迁移状态

状态日期：2026-10-28。本文记录 Codex 路线每个环节在 Claude 路线中的对应物及
迁移限制。表格反映当前工程实现；后文阶段记录包含当时的历史状态。真实 Opus
已通过基础及阻塞 fork 预检，但一次真实模型物理工程测试在启动阶段超时。完整
多角色、并发与物理路径仍只经过本机合成模型的烟雾验证。
完整的当前状态和边界以 [Claude 交接文档](claude-handoff.md) 为准。

## 1. 环节对照

| 环节 | Codex 路线 | Claude 路线 | 状态 |
| --- | --- | --- | --- |
| 模型与厂商配置 | `configs/models.json`、`launch_config` 生成 `codex.json` | `configs/claude-models.json`、`scripts/claude_benchmark.py` | doctor、plan、工程草稿；正式准备与真实模型资格尚未完成 |
| 任务选择与冻结 | `benchmark.select_cases`、`prepare` | 同一套 `select_cases`、`prepare`；harness 增加 `engine: claude` | 共享层已支持 `engine`（阶段 1） |
| 单个策略回合 | Codex app-server 的 `turn/start` | `claude -p`，`robofrontier/claude/runner.py` | 已有，离线测试通过 |
| 机器人工具 | app-server dynamic tool `robot` | 角色绑定 MCP `robofrontier/claude/robot_mcp.py` | 真实 Opus 通过图像工具预检；物理任务的完整多角色路径仍未验证 |
| 每任务策略主机 | `scripts/codex_policy_host.js` | `robofrontier/claude/policy_host.py`、`adaptive_policy_host.py` | 实验性 centralized 与 adaptive 主机 |
| 调度循环 | `scripts/meta_dispatcher.js` | Claude 的 `dispatch.py`、`adaptive_dispatch.py` | 实验性独立调度；共享 Node 适配器也存在，但不是 Claude 工程主机的运行入口 |
| 会话状态来源 | `codex.wait_threads`、`codex.app-server` | `claude.host` | `SESSION_SOURCES` 按 engine 限定（阶段 1） |
| fork 历史一致性 | `runtime_cli.fork_provenance`：`CODEX_HOME` 中的 ordinal 与前缀哈希 | `robofrontier/claude/provenance.py`：转录行的 uuid 序列与内容 | 前缀比较已离线验证，不独立证明来源或权限 |
| 冻结源文件 | `benchmark.py` 与 `codex_*.js` | `claude_benchmark.py`、模型配置及 `robofrontier` Python 源码 | 工程草稿冻结和校验，仍非正式运行资格 |
| 监督命令 | `benchmark.py` 的 preflight、run、status、stop | `claude_benchmark.py`、`claude_precheck.py` | 工程 run/status/stop 与预检已有；正式计分入口未开放 |
| 交接 doctor | `benchmark_handoff.doctor` 要求 `codex` | `claude_benchmark doctor` | 只做离线检查，不能证明真实模型或账号可用 |

## 正式运行前仍需核对

以下是尚未完成的验收工作，不是声称现有命令会自动授予资格：

1. 对每个拟使用的真实 profile、账号和当前 CLI，取得文本、图片工具往返、续接及
   fork 预检证据；adaptive 还需阻塞父会话时的 fork 预检。确认厂商允许所选批量用途，
   并核实模型映射、视觉、工具调用、并发和实际计费。回执匹配本身不能证明模型身份。
2. 用已授权的真实模型完成任务级工程验证，检查观察图片、动作与角色委派、handback、
   任务完成请求、世界终态及共享评估输出。单次合成位移和进程封存不等于任务成功。
3. 对照 Codex 路线确认任务选择、冻结输入、策略权限、录像和评估口径；记录 Claude
   无工具 fork 创建回合等可能影响公平比较的差异。验证模型报告与运行证据的归属，
   不把 CLI 声明的模型名直接作为厂商实际模型身份的证明。
4. 验证启动、停止、未知动作、进程异常与 supervisor 失联的恢复流程，包括并发下的
   状态和证据保留。SIGTERM 烟雾检查不覆盖 SIGKILL、断电或所有动作时机。
5. 在上述证据充分后设计并验证正式准备、资格检查与批次入口，确保不把工程草稿或
   仅有本地回执的运行误标为正式结果。当前 `formal_execution_available` 仍为 false。

真实 API 验证及任何真实实验的费用和范围须先由操作者明确授权；测试不需要把密钥
放进仓库。当前分支使用发布的 Mosaic R14，不包含 R15。

以提交 `188fff6` 的共享代码运行了现有 run setup、runtime store、report、delegation、
execution contract 和 robot actions 测试，合计 55 项通过。report 测试需设置
`PYTHONPATH=scripts`；这些回归检查不能证明完整 Codex 或 Claude 正式任务行为。

## 2. 阶段 1 引入的共享约定

- harness 配置可选 `engine`：`codex`（缺省）或 `claude`。缺省时不写入配置，因此已有 Codex 运行的
  哈希不变。manifest 新增 `engine` 字段。
- `RuntimeStore.observe_session` 只接受该 run 引擎的来源：Codex 为 `codex.wait_threads` 与
  `codex.app-server`，Claude 为 `claude.host`。来源不是任意字符串。
- `runtime_cli.native_fork_provenance` 按 engine 分发。Claude 读取
  `<run>/private/claude-config/projects/*/<session>.jsonl`，每个 run 共用一个隔离的配置目录。
- `meta_dispatcher.js` 的会话控制统一通过适配器：`observe`、`fork`、`send`、可选 `title`。
  `codexAdapter(tools)` 是默认值，Codex 主机与原有 23 个场景不变。Claude 主机提供自己的适配器，
  不构造伪造的 Codex 工具对象。

## 3. 离线实测事实（CLI 2.1.312）

1. **fork 转录。** `--resume P --fork-session` 生成的子转录包含父会话的全部消息行，顺序、uuid
   与内容一致，只有 `sessionId` 与 `promptId` 不同；没有显式的 `forkedFrom` 字段。历史内容比对
   只能证明前缀匹配，不能独立证明可信的创建过程或角色授权。
2. **fork 可以预先指定子会话 ID。** `--resume P --fork-session --session-id C` 返回的会话 ID 就是 C，
   所以主机可以先记录子 ID，再运行回合。
3. **fork 只能由一个回合创建。** CLI 没有“只创建分支不运行”的方式。Codex 路线是
   `thread/fork`（排除回合）后绑定、激活、再发首条提示；Claude 路线在子会话首个回合运行之前
   无法产生转录，也就无法先验证来源。阶段 4 必须解决：首个回合需要使用子角色自己的
   MCP 配置并且不带机器人权限，或者先做一个无工具的创建回合再验证；两者都要显式记录为
   与 Codex 不同的条件。
4. **工具结果图像存放在父会话目录。** 转录行引用 `<父会话>/tool-results/…`，子会话读取历史图像
   依赖父会话目录保留，运行期间不得清理。
5. **MCP 工具超时可配置。** `MCP_TOOL_TIMEOUT`（毫秒）生效：2 秒超时的工具在 6 秒休眠时报
   `timed out after 2s`，20 秒超时的工具 4 秒休眠正常返回。`wait_events` 最长 600 秒，主机必须
   在子进程环境里设置它。默认值未实测。
6. **`--bare` 加 `--tools ''` 加 `--strict-mcp-config` 时没有后台模型请求。** 假端点只收到指定模型
   的请求，工具集恒为 `mcp__robot__robot`。
7. **`claude -p` 在 stdin 未重定向时等待 3 秒并警告。** `run_turn` 应显式使用 `stdin=DEVNULL`
   （阶段 2 修改）。

## 4. 基线测试

以下为早期阶段基线记录，不代表当前资产或最新测试结果。

在本工作树运行（沙箱下 Bash 不能写入工作树，因此设置 `PYTHONDONTWRITEBYTECODE=1`）：

```sh
PYTHONPATH=. .venv/bin/python -m unittest discover -s tests -p 'test_x.py'
node tests/test_meta_dispatcher.cjs
```

阶段 0 基线：`test_scheduling`、`test_delegation`、`test_delegate_callbacks`、`test_cli_benchmark`、
`test_pipeline`、`test_centralized`、`test_action_batch`、`test_run_setup`、`test_runtime_store`、
`test_sync_groups`、`test_benchmark_handoff` 与全部 Claude 测试通过；`test_execution_contract` 与
`test_robot_actions` 各有一个用例因本机缺少场景资产（`scene_asset` 的 `assets/` 目录）而失败，
需要先按 `docs/multimodel-handoff.md` 恢复运行资产，与本次迁移无关。

## 5. 接手后发现的 fork 限制

以下记录最初发现的问题；随后实现了下文所述的实验性内部委派路径，通用入口及
正式计分入口仍保持关闭。

Claude 转录的前缀比较证明内容匹配，不能独立证明该子会话由可信宿主在特定时刻
通过原生 fork 创建，亦不能证明子角色 MCP 权限已正确绑定。`delegate-bind` 对
Claude 因此暂时拒绝执行；只有建立受信创建证据并通过角色权限测试后才可开放。
当前单 Controller 会话宿主与调度器为开发模块，不构成已完成的正式实验 runner。

## 6. 父会话阻塞期间的无工具 fork 测试

本机 CLI 2.1.312 和回环假模型、假机器人服务的测试中，父会话停在一个未返回的
MCP 调用时，可以指定子 UUID 执行 `--resume P --fork-session --session-id C`。
该创建回合使用空 MCP 配置与 `--tools ''`；假模型接收的工具列表为空，父会话
仍保持活动，子历史前缀与父历史匹配。这只验证可行性，不等于身份授权或生产安全性：
仍须完成可信的创建记录、运行时委派绑定、角色隔离、子角色调度和共享预算等验证。

`robofrontier/claude/fork_setup.py` 新增实验性创建步骤：核对已领取的委派和父会话
待处理工具调用，在调用前持久化唯一子 ID，以无工具回合创建，再核对父历史不变、
子历史前缀与 CLI 返回的会话和费用。异常时保留 uncertain 记录，不自动重试。该步骤
本身不绑定角色或授予权限；记录只能作为宿主侧证据，不能防止同一系统账户的其他进程
篡改文件。该步骤已用于实验性的受信内部创建流程；故障恢复及真实任务级测试仍未
完成，通用 Claude `delegate-bind` 继续关闭。

`robofrontier/claude/budget.py` 增加持久化任务预算预留账本；会话和 fork 创建步骤
可以共用该账本，并在并发时先预留再调用。费用不明或超过预留则停止发放新额度，
进程失联后的未结算预留不会自动释放。账本是 CLI 报告的费用估算，不是实际账单或
支付平台硬上限；adaptive 工程主机已使用它，但不能据此开放正式多角色运行。

`ClaudeSessionHost.adopt_verified_fork` 增加了一个实验性子角色接管原语：要求冻结
输入和模型配置一致、fork 记录与运行时已激活的角色及父历史匹配、父委派调用仍在
等待，并使用运行目录内共享预算账本。成功后才为该角色创建 MCP 配置并续接原生子 ID；
一次性接管标记阻止自动重复创建宿主。实验性内部路径已连接多角色调度；通用
Claude `delegate-bind` 和正式真实任务仍未开放。

`robofrontier/claude/adaptive_setup.py` 将上述组件串成实验性的受信宿主创建流程：
领取委派、创建无工具 fork、记录并核对 ID 和历史、绑定范围、激活以及发送子任务。
合成运行时的测试覆盖成功路径和费用不明失败路径，失败时暂停并保留机器人所有权。
通用 `delegate-bind` 仍对 Claude 关闭；该内部流程已接入 adaptive 工程主机，但没有
经过真实任务、供应商和故障恢复的端到端验证。

`robofrontier/claude/adaptive_dispatch.py` 新增实验性的角色状态观察和调度循环，使用
同一个宿主所有权和现有运行时调度器，将通知路由给匹配的父或子会话；角色缺失、状态
不明、发送结果不明或残留创建领取时暂停。

`robofrontier/claude/adaptive_policy_host.py` 新增供现有世界使用的实验性策略主机，
接入 adaptive 公开材料、共享预算、角色绑定、READY、调度和多角色停止。四类 Level 1
最少臂的 adaptive 工程草稿通过了冻结输入和启动材料加载检查。fork 创建现在线程中
进行，停止请求会取消创建回合并阻止新子角色启动；离线测试覆盖了阻塞时的停止。
当前 adaptive 已接入未计分的工程世界/批次启动入口，真实 CLI 进程异常、多角色物理封存
与供应商端到端行为仍需验证。上述合成测试不能代表真实任务或供应商兼容性。
