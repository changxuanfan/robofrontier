# Codex 到 Claude Code：对照表与迁移状态

状态日期：2026-10-28。本文记录 Codex 路线每个环节在 Claude 路线中的对应物、已实现的
共享中立接口，以及阶段 2–4 必须遵守的已验证事实。所有验证均离线：本地假模型端点、
假机器人服务、锁定的 Claude Code CLI 2.1.312，没有调用任何厂商 API。

## 1. 环节对照

| 环节 | Codex 路线 | Claude 路线 | 状态 |
| --- | --- | --- | --- |
| 模型与厂商配置 | `configs/models.json`、`launch_config` 生成 `codex.json` | `configs/claude-models.json`、`scripts/claude_benchmark.py` | profile 与 doctor、plan、prepare-draft 已有；可运行的 prepare 待做 |
| 任务选择与冻结 | `benchmark.select_cases`、`prepare` | 同一套 `select_cases`、`prepare`；harness 增加 `engine: claude` | 共享层已支持 `engine`（阶段 1） |
| 单个策略回合 | Codex app-server 的 `turn/start` | `claude -p`，`robofrontier/claude/runner.py` | 已有，离线测试通过 |
| 机器人工具 | app-server dynamic tool `robot` | 角色绑定 MCP `robofrontier/claude/robot_mcp.py` | 已有，未经多角色验证 |
| 每任务策略主机 | `scripts/codex_policy_host.js` | 待建 `scripts/claude_policy_host.js` | 阶段 2 |
| 调度循环 | `scripts/meta_dispatcher.js`（原写死 Codex 工具名） | 同一文件，经中立适配器 | 适配器已完成（阶段 1），含 24 个 Node 场景 |
| 会话状态来源 | `codex.wait_threads`、`codex.app-server` | `claude.host` | `SESSION_SOURCES` 按 engine 限定（阶段 1） |
| fork 来源证明 | `runtime_cli.fork_provenance`：`CODEX_HOME` 中的 ordinal 与前缀哈希 | `robofrontier/claude/provenance.py`：转录行的 uuid 序列与内容 | 已实现并用真实 CLI 加假端点验证（阶段 1） |
| 冻结源文件 | `benchmark.py` 与 `codex_*.js` | `claude_benchmark.py` 与 `claude_policy_host.js`，按 engine 选择 | 已完成（阶段 1） |
| 监督命令 | `benchmark.py` 的 preflight、run、status、stop | `claude_benchmark.py` 与 `claude_precheck.py` | 预检已有；run、status、stop 待做（阶段 3） |
| 交接 doctor | `benchmark_handoff.doctor` 要求 `codex` | 待按 engine 区分 | 阶段 6 |

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
   与内容一致，只有 `sessionId` 与 `promptId` 不同；没有显式的 `forkedFrom` 字段。来源证明因此
   依据历史内容比对，而不是模型自述。
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
