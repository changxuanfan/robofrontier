# Claude Code 多模型 Harness 迁移设计

2026-10-28，Asia/Hong_Kong。状态：设计提案，尚未实现或调用模型。

**1. 目标与分支基线**

在 `codex/claude-code-harness` 分支设计可独立运行的 Claude 实验系统。
基线为 GitHub `codex/multimodel-handoff` 的 `b3259aa817197f9bcb26ed7ccda02229b7c049ca`。
当前本地 worktree 为 `/Users/fanchangxuan/Repos/roomscript2-worktrees/claude`。

- 实验内部所有策略模型统一通过 Claude Code CLI 运行，包括 Opus 5.5、其他 Claude 模型和国产模型。
- 最上层工程主 session 可为 Claude Desktop、Claude Code、Codex 或其他有权限执行管理命令的 agent；工程模型不参与策略决策。
- doctor、prepare、run 和交接环境不依赖 `codex` 可执行文件或登录状态。
- 目标覆盖 `claude-opus-5-5`、其他 Claude、原有 GPT 配置及 Kimi、Qwen、GLM、DeepSeek；再逐步纳入豆包、MiMo、MiniMax、Step。每个具体型号必须通过预检。
- 已发布场景继续使用基线版本，Mosaic 九项均为 R14、81 mm，无 R15 的15%颜色交换。
- 原 GitHub 交接分支和其他 Codex/R15 worktree 不因本方案而变更。
- 复用物理、评估、公开任务和动作协议；Claude 会话、工具、prompt、skill 和调度单独实现。

本机已由 `2.1.96` 更新至 npm latest **`2.1.312`**，版本与 `--help` 已检查；
尚未调用认证或推理。正式实验将记录并固定经预检的 CLI/SDK 版本，升级后重新验证。

**2. 先区分四种入口**

| 入口 | 原生提供什么 | 本项目定位 |
| --- | --- | --- |
| Claude Code CLI / Agent SDK | Claude Code 的循环、会话、工具、权限、压缩和子代理能力 | 第一条完整自动化执行路线；推荐用 API Key 计费 |
| Claude Desktop 的 Code 模式 | 项目内的 Claude Code 交互体验，支持项目指令、skills、MCP 等 | 交互开发和实验管理入口；策略批量运行由独立 Claude Code CLI/Agent SDK 进程承担 |
| Claude Desktop 普通聊天或其他主 session | 执行允许的管理命令或通过 MCP 调用管理功能 | 管理入口；可启动和查询，但不是策略团队本身 |
| Claude Messages API / Client SDK | 原生模型请求、图片、tool_use/tool_result | Claude Code 连接模型服务的协议；本分支不额外实现一套绕过 CLI 的策略执行器 |

API Key 是认证/计费选择；Messages API 和 Agent SDK 是不同执行路线。
Agent SDK 实际运行 Claude Code binary；直接 Messages API 不自动获得其会话、fork、
skills、hooks 或自动压缩。国产厂商需提供 Anthropic Messages 兼容服务或经过明确
验证的转换服务；原本的 Responses 地址不能直接当成 Claude Code 地址。
[官方比较](https://code.claude.com/docs/en/agent-sdk/overview)

Code 模式与 CLI 可共享项目配置，但桌面配置发现顺序有差异；只加载本次冻结的材料。
普通聊天的 MCP 能力也不等于可通过官方接口自动创建、唤醒和监控任意聊天会话。
[Code Desktop](https://code.claude.com/docs/en/desktop)、
[Desktop 本地 MCP](https://support.claude.com/en/articles/10949351-getting-started-with-local-mcp-servers-on-claude-desktop)

**3. 推荐架构**

```text
Claude Desktop / Claude Code / Codex / 任意获授权的工程主会话
                  |
       实验管理命令或 management MCP
       prepare / start / status / stop / report
                  |
          Claude experiment supervisor
                  |
          Claude Code CLI / 官方 Agent SDK
                  |
     所选服务商的 Claude 或国产模型 API
                  |
       robot MCP：认证操作与真实图像返回
                  |
  公共任务 / 工作笔记 / 邮箱 / 权限 / 动作编号 / 调度规则
                  |
          物理 owner / MuJoCo / 私有评估与记录
```

工程主 session 可换，实际策略执行器与模型由 batch 的冻结配置决定；切换主 session
不等于改变任务内模型。管理工具只返回工程状态、计费与错误等获授权摘要，不给策略动作建议或隐藏评估真值。
策略从全新会话开始，不继承工程聊天、其他实验或 Oracle 历史。
原 runtime 的 SQLite 仍是实验状态权威；Claude 的 transcript 是模型会话证据。
会话 resume 不代表物理世界支持进程崩溃后的恢复。

**4. 可以直接采用的 Claude 原生能力**

| 能力 | 采用方式 | 必须验证的部分 |
| --- | --- | --- |
| 非交互执行与日志 | CLI `-p`、stream-json，或官方 Agent SDK | 原生 init/result/error、session ID、完整 usage、异常退出 |
| 多轮与持久会话 | 明确 session ID 的 resume；需要分支历史时原生 fork | 多任务禁止依赖“最近会话”；恢复不重放已提交物理动作 |
| 机器人函数与截图 | 原生 MCP，返回 text 与 image content | 模型读取的是本轮真实截图，路径文字不能代替图片 |
| 指令与知识 | run 内冻结的 Claude prompts、skills 与 agent definitions | 子代理实际加载了哪些内容；压缩后如何恢复规则 |
| 临时分工 | 普通子代理或原生 fork，取决于所需历史 | 普通子代理默认不继承完整历史；fork 与工具权限继承不同 |
| 生命周期观察 | SDK 事件、原生 session/agent ID、适用版本 hooks | 原生结束只表示模型轮次结束，不表示任务完成或控制权交回 |

官方来源：[非交互执行](https://code.claude.com/docs/en/headless)、
[会话与 fork](https://code.claude.com/docs/en/agent-sdk/sessions)、
[MCP 图像](https://code.claude.com/docs/en/mcp#images-in-tool-results)、
[skills](https://code.claude.com/docs/en/skills)、
[子代理](https://code.claude.com/docs/en/agent-sdk/subagents)、
[hooks](https://code.claude.com/docs/en/hooks)。

Agent Teams 当前仍为实验能力，存在队友恢复与任务状态等限制，第一版不以其作为
无人值守机器人物理实验的唯一执行基础；后续可单列实验模式。
[官方限制](https://code.claude.com/docs/en/agent-teams#limitations)

**5. 单独编写 prompt、skill 和角色配置**

以下均为拟建文件，并非已经存在或可执行：

```text
prompts/claude/manager.md              工程主会话：准备、启动、监控、报告
prompts/claude/controller.md           任务 Controller 的初始化和交回责任
prompts/claude/delegate.md             有边界的临时任务与角色切换
prompts/claude/worker.md               可选常驻 Worker 模式
.claude/skills/robot-experiment/       Claude 专用执行规则与恢复入口
.claude/agents/                       Claude 原生角色定义模板
configs/claude-models.json             Claude 与国产模型的精确型号、端点和预算配置
configs/claude-harness.json            Claude 引擎、拓扑和并行配置
scripts/claude_benchmark.py            独立工程命令入口
robofrontier/claude/                  runner、监督、MCP、会话证据
docs/claude-handoff.md                 安装、凭证、Desktop、服务器交接
```

具体语言与文件拆分在协议验证后收敛；同一职责只保留一份实现。
Agent SDK 优先评估 Python 以复用本项目 Python runtime；如果必要的原生 agent
身份或事件能力只在锁定的 TypeScript 版本可靠提供，再选择 TypeScript runner，
不同时维护两套 Claude Code 循环。

规则分工：manager 只做工程操作；Controller 自主决定机器人策略与委派；Delegate
执行自己的 robots/goal/done_when/constraints。共享动作语义保持一份权威定义，
Claude 文档补充其原生工具名称、图像机制与恢复行为，不复制长期漂移的协议正文。
每个 run 把实际采用的文件冻结并记录摘要，避免读取工作目录后来改过的模板。

项目级 CLAUDE.md 与技能加载路径只在工程/策略的明确范围内生成；策略 workspace
不得自动加载工程主会话的私人 memory、hook、插件或未冻结提示词。

**6. 机器人 MCP 是实接口，权限不是 prompt 约定**

MCP 调用复用既有 observe、move、gripper、job/pending、work、inbox、通信和
return_task 等操作语义。采用固定 JSON Schema，真实截图以 image content 返回。
同一业务操作的参数含稳定 call_id；未知回执时查询原编号。

MCP 进程/连接在宿主侧绑定 run 与 actor，并由 runtime 验证 robots 所有权。
不接受模型自由填写任意 actor 或私有连接路径来切换角色。原生子代理继承 MCP
连接不等于获得新的独立角色权限；fork 也不会自动撤销父角色权限。

第一轮协议验证必须证明一种可靠的角色绑定：可以采用原生 role-scoped MCP
配置，或独立角色进程与各自认证连接。动态分身的原生身份、授权范围和生命周期
必须在受信宿主侧建立对应关系；不能用模型自报身份、prompt 注入或普通工具参数
伪造此对应关系。失败时阻止该拓扑启动，不静默降级权限要求。

MCP 与工具白名单不能单独作为恶意策略的操作系统文件隔离。初版沿用原基准的
合作式证据限制；若需要强隔离，另加受限 workspace/进程权限，并独立声明验收范围。

**7. 多代理、fork 与调度的独立设计**

- 先验收单 Controller 的工具、截图、续轮与生命周期；这是接口检查，不冒充
  adaptive 正式结果。Controller 原本可以对多个不同机械臂提交并行动作。
- adaptive 仍要求 Controller 决定委派范围，runtime 检查范围互斥与未决动作，
  记录委派，绑定子会话，验证历史来源与身份，再激活权限。
- 原生普通子代理从新上下文开始；需要继承本任务完整历史时采用原生 fork。
  skill 的 `context: fork` 并不是完整会话历史 fork。
- 单独定义 Claude provenance：原生 session/agent/parent ID、已知源消息边界、
  请求和确认事件。不得填造 Codex ordinal 或用一条“创建成功”日志代替来源验证。
- 必须验证父执行挂起/空闲、子会话真正创建、源历史一致、图像历史可用和子角色
  权限重新绑定。若原生接口无法提供原基准要求的历史证明，记录能力缺口；
  另设带显式上下文包的 delegation 模式，作为不同实验条件，不冒称等价 adaptive。
- 生命周期区分 starting、active、waiting、idle、failed、unknown、ended；会话状态
  由原生证据推进，业务完成以 return_task/finish 的 runtime 结果为准。
- hook 用于已验证的生命周期与事件采集，不替代动作幂等、授权或私有评估。

这是目前最大的设计验证点。会话原生支持 fork，并不足以证明实验中的授权和
同一历史边界已经满足。普通子代理与完整历史 fork 的区别见
[官方子代理](https://code.claude.com/docs/en/sub-agents)及
[skill 的 context: fork](https://code.claude.com/docs/en/skills#run-skills-in-a-subagent)。

**8. 并行、模型与预算**

分别配置 batch 任务并行、每任务策略/子代理数量、账户 API 请求并发与速率预算。
例如“四项最少臂 Level 1”表示四个独立世界，不表示四个机械臂或四个 API 请求。
每个世界仍有自己的记录、邮箱、工作笔记与动作编号。

模型 profile 保存 engine、精确 model ID、requested/effective effort、context 能力、
压缩设置、输出与时间/费用预算、CLI/SDK 版本和认证来源类型；密钥值不进入记录。
其他 Claude 与国产模型按能力单独配置，不假定支持相同窗口、effort、图像限制或缓存规则。

厂商接入候选均以独立 profile 配置，均未完成本工作树的真实 API 预检：

| 模型 | 官方 Claude Code / Anthropic 入口证据 | 需核实 |
| --- | --- | --- |
| OpenAI GPT（原配置 `gpt-6-astra`、`gpt-6-luna`） | 本次尚未核实可由 Claude Code 直接使用的模型端点 | 确认模型/API 权限和 Anthropic 兼容路径；可能需要独立验证的协议转换网关，不预先宣称支持 |
| Kimi `kimi-k3.1-pro` | [Kimi Anthropic Messages 与 Claude Code](https://platform.kimi.com/docs/guide/claude-code-kimi) | 精确型号、视觉与图片工具回传 |
| Qwen `qwen3.8-max-0902` | [百炼 Anthropic Messages 模型表](https://help.aliyun.com/zh/model-studio/anthropic-api-messages) | 地域、按量密钥、图片工具回传；Coding Plan 禁止非交互批量脚本，不能用于本实验：[套餐规则](https://help.aliyun.com/zh/model-studio/coding-plan-faq) |
| GLM `glm-5.3-flash` | [智谱 Claude Code 文档](https://docs.bigmodel.cn/cn/coding-plan/tool/claude) | 文档示例为 Coding Plan；标准按量密钥与精确型号的可用性须验证 |
| DeepSeek `deepseek-flash` | [DeepSeek 官方 Claude Code](https://api-docs.deepseek.com/quick_start/agent_integrations/claude_code/) | 固定 Flash，避免 Claude 档位别名被映射到不支持视觉的 V4 Pro |
| 豆包 `doubao-seed-2-1-pro-260915` | [方舟按量 Claude Code 入口](https://docs.volcengine.com/docs/ark/integrate-third-party-tools?lang=en) | 官方示例使用旧快照；验证260915及图像/工具链 |
| MiMo `mimo-v2.6-pro` | [小米按量 Claude Code](https://mimo.mi.com/docs/zh-CN/tokenplan/integration/claudecode) | 图像、子代理、真实模型与工具回传 |
| MiniMax `MiniMax-M3.1-Flash` | [MiniMax Claude Code](https://platform.minimax.cn/docs/m-plan/claude-code) | 示例为订阅接入；按量密钥及图片工具链需核验 |
| Step `step-5-preview` | [Step Claude Code](https://platform.stepfun.com/docs/zh/guides/developer/integrations/claude-code.md) | 区分 Step Plan 和普通按量端点；验证真实图片和工具链 |

每个 profile 启动时仅向该子进程传递其端点和密钥环境变量，不修改用户全局设置；
覆盖或隔离已有 Claude 配置中的模型与端点，避免跨厂商串线。实际设置优先级和
密钥冲突必须按锁定的 CLI 版本核实。模型别名、辅助请求、子代理和 fallback 均需
核对实际调用型号；响应缺少可信实际型号时标记“未验证”，不得冒称已验证。

Opus 5.5 使用 `claude-opus-5-5`；自适应思考始终启用，以其支持的 effort 控制。
不能照搬手工 thinking token budget 或强制 tool_choice 的旧参数。
[Opus 5.5 说明](https://platform.claude.com/docs/en/models/opus-5-5/whats-new-opus-5-5)

严格模型实验关闭自动降级；发生其他模型调用或型号不可确认时在报告中明确记录，
不能声称全程 Opus。Claude Code 自带辅助/子代理调用也要纳入每模型 usage 审计，
不能只依据顶层 `--model` 判断所有后台请求的模型。
缓存读取、5m/1h写入、普通输入、输出与辅助模型分别计量；记录 API usage 与
客户端估价的区别，resume 后避免重复累加累计账单。账户配额需开跑前核实。
之前 Codex/Responses 的缓存价格与实际命中逻辑不能原样套用到 Anthropic 兼容端点；
每个供应商、地区、套餐和协议需复核计价及实际 usage 字段。

首选 API Key 的自动化环境，普通 Claude 登录可作为明确选择的交互路径；不复制
用户 Desktop 登录文件或把订阅额度宣称为 API 余额。认证检查只报告状态与变量名。

**9. 主 session 与模型服务的交付范围**

Desktop Code 模式读取 Claude 专用项目材料，可以通过同一命令准备/运行/查询。
普通 Desktop 聊天接 management MCP 后可接受“跑指定四类任务，并行N项”等指令，
由监督进程持有实验生命周期；聊天关闭不被视作正式任务已完成。

任何有权运行管理命令或调用 management MCP 的主 session 均可负责工程管理。
正式任务内策略统一由独立 Claude Code CLI/Agent SDK 会话承担，模型可以是 Claude
或通过 Anthropic 兼容接口提供的国产模型。管理与策略会话分开。

本地 Desktop 可用 stdio MCP；Slurm/远程环境优先 SSH 隧道连接本机 MCP bridge。
Remote connector 从 Anthropic 云端连接，不能直接指向仅本机可见的 localhost。
需要远程 connector 时另行部署可访问且有认证的 endpoint，不自动暴露集群服务。
[官方网络说明](https://support.claude.com/en/articles/11175166-get-started-with-custom-connectors-using-remote-mcp)

每家按量 API 的 key、端点、模型与额度使用独立 profile。订阅套餐的使用范围、
视觉支持和非交互运行许可须逐项核对，不能把可在交互式 Claude Code 使用视为
自动授权批量实验。厂商未原生兼容时先记录缺口；自建转换服务只有在协议、图片、
工具续轮和成本都经过独立验证后才能作为明确标注的可选实现。

**10. 实施阶段与可验收产物**

| 阶段 | 工作与产物 | 完成条件 |
| --- | --- | --- |
| A：冻结设计与能力验证 | 锁定 CLI/SDK；核实 session/fork、role-scoped MCP、截图和权限 | 发布能力矩阵；未通过项明确阻止对应模式 |
| B：独立入口与运行包 | Claude models/harness 配置、prepare/doctor、冻结的 prompt/skill | 无模型/物理调用可准备四项L1；源与资产hash可验证 |
| C：原生工具与单角色闭环 | Claude MCP、原生事件、真实图像往返与稳定编号 | 合成图辨认→工具返回另一张图→继续调用；不依靠文字泄露颜色答案 |
| D：会话和委派 | resume、压缩后恢复、fork、角色绑定与回收、通信 | 来源证据成立；子代理越权与重复/未知调用测试拒绝；同一物理世界无重置 |
| E：完整监督 | START、等待/唤醒、容量失败、停止、封存、usage | 不向 active 会话重复投递；截止后不再动作；异常留存证据且不重开 |
| F：小额联网预检 | 指定模型与预算检查，独立receipt | 对实际版本、配置、prompt/skill与工具协议绑定通过记录 |
| G：正式最小批次 | 用户明确启动时，四类各一项最少臂L1 | 四个独立run、截图、原始usage、动作和私有评估完整 |
| H：Desktop与交接 | Code模式、management MCP、其他工程主session验收 | 干净clone可复现；未安装Codex也能启动；配置和资产齐全；不依赖现有登录文件 |
| I：各供应商接入 | 各自 Anthropic 兼容端点、凭证、精确型号与预检 | 每个型号单独验证图片、工具、实际型号、限额、计费与并行；失败不影响其他型号 |

A/C/D 中的付费检查只在实施阶段按明确预算启动，本设计阶段不调用模型。
Gate F 不以一句文字回应通过；必须验证本项目实际使用的图片工具循环。

**11. 必须改动的位置与保留的不变量**

当前 Codex 耦合实际分布在 `scripts/benchmark.py`、`codex_policy_host.js`、
`codex_rpc.js`、`codex_preflight.js`、`meta_dispatcher.js`，以及
`robofrontier/runtime_cli.py` 的 Codex fork 文件校验、`runtime_store.py` 的
Codex 状态来源白名单、`run_setup.py` 的冻结源清单、handoff 的 Codex doctor。

Claude 路线新建原生执行器与监督，不向原有 dispatcher 塞伪造的 Codex 工具对象。
runtime 中真正共享的会话观察/委派验证可以抽出与平台无关的实验契约，分别检验
Claude 原生证据与历史 Codex 证据；拒绝简单把来源白名单放宽成任意字符串。

继续保留：冻结公开输入、动作幂等与未知结果、角色授权、同臂互斥、控制权交回、
24小时实验截止、START/READY、一个世界一个物理 owner、终止封存、私有评估、
R14 场景hash，以及同批不同模型共用新生成 Dial 题目但世界独立的比较方式。
Claude 的自动重试、模型 fallback、文件 checkpoint 都不能绕过这些规则。

测试覆盖接口和物理副作用：真实图像工具结果、异步多臂、权限、同源委派、
未知回执、429等待、成本/时间截止与封存；适当复用已有 FakeWorld 和协议测试。
文档与可逆配置不额外写逐字镜像测试。最终还需要原有 Codex 核心回归通过，
共享逻辑变化才可合并；不依赖此 worktree 的忽略文件或其他任务运行产物。

**12. Git 与交接**

新功能只在 `codex/claude-code-harness` 实施。按“独立准备/配置 → MCP与原生
执行 → 委派与监督 → Desktop/交接”形成可独立审查的提交，避免同步场景改版。
不默认导入 R15、Oracle 输出、旧策略历史、`.env`、登录状态、绝对路径或其他
worktree 的虚拟环境。依赖与 R14 资产使用可复现安装和校验。

完成后从交接分支重建/更新 PR，通过共享核心回归再合并。目标是普通 Git 提交
与明确资产包；同事 clone 分支、初始化环境与凭证后，可从任意获授权的工程主会话通过
一条任务描述选择 model、families、level、min_arms、parallel、预算并启动。
CLI命令与Desktop工具的最终名称以实现为准，本提案中的路径不代表入口已存在。

**14. 代码审查后的实施拆分与验收门槛（2026-10-28）**

基线调查以 `b3259aa817197f9bcb26ed7ccda02229b7c049ca` 为准。下面是实施计划，
并非已实现功能；本计划不保证未经验证的第三方服务支持。

| 现有文件或目录 | 核查结果 | 实施动作 |
| --- | --- | --- |
| `scripts/benchmark.py` | 同时负责选择任务、模型 profile、冻结、锁、批次调度；生成 `codex.json` 并检查 Codex preflight、版本和 host | 提取与引擎无关的选择及批次逻辑；另建 Claude 命令和配置。现有入口在回归通过前保持可用。 |
| `scripts/codex_policy_host.js`, `scripts/codex_rpc.js` | 使用 app-server 的 turn/thread 和 robot dynamic tool；宿主持有角色映射、动作调用、截图 | Claude 侧原生 runner + MCP，保留真实身份绑定和一次性动作语义；不伪装 app-server 协议。 |
| `scripts/meta_dispatcher.js` | 状态观察、fork、发送和唤醒直接调用 Codex API | 保留调度规则，Claude 单独实现可信事件观察和原生会话控制；未知状态停在需要检查。 |
| `robofrontier/runtime_cli.py`, `runtime_store.py`, `delegation.py` | fork 读取 `CODEX_HOME` 的 ordinal 与前缀哈希，状态观察只接受两个 Codex 来源；委派时事务性分配所有权 | 设计带类型及版本的实际证据验证，保留所有权及事务规则；无可验证 Claude 来源则 adaptive 不开放。 |
| `robofrontier/runtime_service.py`, `pipeline.py`, `pipeline_client.py` | 角色连接 token、世界、观测、动作队列、私有评分和幂等请求 | 以公共接口复用，受信宿主绑定每个角色；保持未决动作不重复和世界丢失不恢复的原则。 |
| `robofrontier/run_setup.py`, `run_seal.py` | 创建并冻结公共、私有输入；源文件列表和角色模板目前含 Codex 元素 | 纳入 Claude runner、MCP、prompt、skill、profile、依赖和版本指纹；防止把工程和私有内容带进策略上下文。 |
| `configs/models.json`, `configs/harnesses/policy.json`, `configs/task_families/current.json` | 原 profile 是 Responses，物理和任务发布配置与引擎部分耦合 | 新建 Claude profile，逐型号核实；沿用任务发布清单和物理参数，不改 R14。 |
| `scripts/benchmark_handoff.py`, `docs/multimodel-handoff.md` | 资产恢复可以复用，doctor 和安装说明要求 Codex | 独立 Claude doctor、密钥/环境说明、资产校验和全新 clone 文档。 |
| `tests/test_cli_benchmark.py`, `test_runtime_store.py`, `test_delegation.py`, `test_meta_dispatcher.cjs`, `test_codex_preflight.js`, `test_execution_contract.py`, `test_run_setup.py` | 已有测试覆盖现有行为，但不证明 Claude 的画像、fork、身份及原生事件 | 原测试保留；新增具有真实 Claude 协议形态的离线集成测试、故障注入及经授权的厂商预检。 |

实施顺序及进入下一阶段的最低条件：

1. **基线和能力清单**：冻结当前 commit、CLI/SDK 版本及配置；核对构建依赖和已发布资产哈希。以本机帮助和官方说明核实 MCP 图片、事件、resume/fork、原生设置隔离和版本边界。输出功能矩阵及未知项。
2. **干净的 prepare/doctor**：实现独立 Claude 模型配置和工程命令；对四家族选择最小臂 Level 1 得到四项；冻结全部运行输入。doctor 对缺少 assets/依赖/凭证给出可操作诊断，不请求模型。无 Codex 可执行文件的隔离环境也可通过非模型准备检查。
3. **离线端到端最小路线**：构建受控 stub 来测试启动、真实 MCP tool result 图片内容、权限绑定、session/turn 事件和停止；请求与回应的外部接口必须与锁定版本匹配。离线 stub 只证明宿主行为，不能证明厂商适配。
4. **物理与故障集成**：在本地测试世界里验证单 Controller、多机械臂动作与并发、截图、消息、真实状态和结束语义；注入断链、回执不明、子进程失败和重复请求。世界 owner 丢失不得自动恢复并继续计分。所有测试限定可恢复的测试资产与非正式结果。
5. **角色、历史和动态委派**：用原生会话验证继承边界、可信来源、实际子角色权限、父子可观察状态、手回动作及异常处理；对重复 fork/启动不明等故障做测试。不能验证原有条件时保持该模式不可用，并提出明确标注的其他实验条件，不能伪报等价。
6. **真实模型小额预检**：只有在获得可用的个人凭证与额度后，分别验证指定型号：文字、输入图、工具调用、工具回传新图片、原生会话恢复、需要时的 fork/压缩；记录型号、路径、版本与费用，不输出密钥。供应商或型号没有通过则禁止正式启动。
7. **交付和真实实验**：完成并发/资源/费用限制、可中断的监督、操作文档和全新 clone 恢复演练；先展示每模型预检结论。真实四项 Level 1 只有在用户明确要求时启动，且不会把任务数量等同 API 并发数。
8. **回到主线**：Claude 分支仅包含代码、测试和不含密钥文档；在独立 worktree 验证，先提交再推送到独立远端分支，之后经 PR 与现有 main 的改动比较、解决冲突及回归。工作树、`.env`、资产和运行结果各自独立，不通过 Git 合并。

每个阶段的交付状态必须区分：已完成、仅离线验证、真实厂商已验证、受限/不可用；
不得把工具已启动、CLI 退出码 0 或模型一句“完成”当作任务成功。
