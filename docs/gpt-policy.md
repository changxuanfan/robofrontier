# 用 Codex CLI 运行模型基准

入口是 `python -m scripts.benchmark`。Linux 配置与数据准备见[服务器指南](server-benchmark.md)。一个 Desktop 主会话可以执行这个命令并查询状态；实际策略团队由独立的 Codex CLI `app-server` 进程运行，不依赖 Desktop 主聊天一直占用工具轮次。现有 Desktop 固定调度器仍可使用。

跨机器交接、当前价格、Level/最少臂筛选及可复制的调度 prompt 见[多模型交接指南](multimodel-handoff.md)。

每个模型／任务组合拥有独立 run、MuJoCo 世界、SQLite、连接、策略会话和工作笔记。默认 adaptive；只由 Controller 自行选择任务内 fork 范围，宿主不规划抓放路线。9／36 指并行任务数，同一批次同一Dial题目在各模型间共享新生成的私有密码，仍使用独立世界与工作笔记。任务内分身可能增加模型请求并发，不能把任务数当作厂商配额保证。

## 模型与 API

配置在 `configs/models.json`，默认 `astra`（`gpt-6-astra/high`），另有 `luna`（`gpt-6-luna/high`）。所有配置只作为该 CLI 子进程的参数，不修改用户全局 Codex 默认模型或登录。

截至 2026-10-28 的官方文档：

| 厂商 | 当前旗舰／相关型号 | 视觉任务使用 | 官方依据 |
| --- | --- | --- | --- |
| DeepSeek | V4 Pro；最新视觉版 V4.1 Flash | `deepseek-flash`；Pro 仅文本 | [型号与能力](https://api-docs.deepseek.com/quick_start/pricing/)、[Codex 接入](https://api-docs.deepseek.com/quick_start/agent_integrations/codex/) |
| 千问 | Qwen3.8-Max | 固定快照 `qwen3.8-max-0902` | [视觉能力](https://help.aliyun.com/zh/model-studio/vision-model)、[Responses](https://help.aliyun.com/zh/model-studio/qwen-api-via-openai-responses) |
| 智谱 | GLM-5.3 仅文本；GLM-5.3-Flash 为原生多模态 | `glm-5.3-flash` 候选，须验证该模型／账号的 Responses 图像与工具支持 | [旗舰](https://docs.bigmodel.cn/cn/guide/models/text/glm-5.3)、[视觉版](https://docs.bigmodel.cn/cn/guide/models/vlm/glm-5.3-flash)、[Responses](https://docs.bigmodel.cn/cn/guide/develop/responses/introduction) |
| Kimi | Kimi K3.1 Pro | `kimi-k3.1-pro` | [型号](https://platform.kimi.com/docs/guide/kimi-k3-1-pro-quickstart)、[Codex 接入](https://platform.kimi.com/docs/guide/codex-kimi) |

旗舰是厂商定位，不是我们的实测排名。纯文本型号不会被偷偷加上另一个视觉模型后混入同一成绩。订阅或 Coding Plan 不代表具有本基准所需的按量 API 权限；具体模型、Responses、图片和工具支持以账号实际预检为准。

复制仓库的 `.env.example` 为 `.env`，填入需要的凭证；benchmark 读取该文件，已有进程环境变量优先。文件只解析字面赋值，不执行 shell。`.env` 不纳入 Git。密钥只通过对应环境变量进入 CLI：`DEEPSEEK_API_KEY`、`DASHSCOPE_API_KEY`、`ZAI_API_KEY`、`KIMI_API_KEY`。不要写入仓库或聊天。千问另设 `DASHSCOPE_BASE_URL`，填写你在官方控制台得到的工作空间／地域地址，例如 `https://你的WorkspaceId.cn-beijing.maas.aliyuncs.com/compatible-mode/v1`。

底层等价的 Codex provider 配置如下，实际 benchmark 会自动按进程生成，无需覆盖全局配置：

```toml
[model_providers.deepseek]
name = "DeepSeek"
base_url = "https://api.deepseek.com"
env_key = "DEEPSEEK_API_KEY"
wire_api = "responses"
requires_openai_auth = false
supports_websockets = false

[model_providers.kimi]
name = "Kimi"
base_url = "https://api.moonshot.cn/v1"
env_key = "KIMI_API_KEY"
wire_api = "responses"
requires_openai_auth = false
supports_websockets = false

[model_providers.zhipu]
name = "Zhipu"
base_url = "https://open.bigmodel.cn/api/v1"
env_key = "ZAI_API_KEY"
wire_api = "responses"
requires_openai_auth = false
supports_websockets = false

[model_providers.qwen]
name = "Qwen"
base_url = "https://你的WorkspaceId.cn-beijing.maas.aliyuncs.com/compatible-mode/v1"
env_key = "DASHSCOPE_API_KEY"
wire_api = "responses"
requires_openai_auth = false
supports_websockets = false
```

Codex 当前只支持 `wire_api="responses"`。支持 OpenAI Chat Completions 不等于支持 Codex；若某账号只有 Chat API，不能把 `wire_api` 改成 `chat` 来解决。[OpenAI 配置说明](https://learn.chatgpt.com/docs/config-file/config-reference)

第三方模型还需要正确的本地模型 catalog，声明图片输入、工具模式和上下文。脚本自动生成并绑定；不能只改 `-m` 后就认定图片已经传入。现有配置含文档能力，不代表已通过外部 API 测试。

## 预检、准备与启动

```sh
# 查看配置；不调用模型
.venv/bin/python -m scripts.benchmark models

# 真实 API 小预检：图片辨色、函数工具、手动上下文压缩、fork及分身工具；会消耗少量 API 额度
.venv/bin/python -m scripts.benchmark preflight --models astra
# 配好各自环境变量后，可用 --models deepseek qwen glm kimi

# 冻结全新输入；不启动模型或物理。可同时给多个 --models
.venv/bin/python -m scripts.benchmark prepare --models astra --families mosaic --output runs/policy/my-mosaic-batch
# 不指定 --families／--cases 则每个模型准备全部36项

# 真正开始；不能恢复或重建曾启动过的物理世界
.venv/bin/python -m scripts.benchmark run runs/policy/my-mosaic-batch --parallel 9
# 全36项可用 --parallel 36，实际受本机与厂商配额限制
.venv/bin/python -m scripts.benchmark status runs/policy/my-mosaic-batch
.venv/bin/python -m scripts.benchmark stop runs/policy/my-mosaic-batch
```

run 会检查预检结果与实际模型、provider 地址、上下文、effort、生成的模型 catalog 和 Codex CLI 版本一致。修改执行源码／冻结输入后必须重新 prepare。preflight 是小样本接口兼容性检查，包含手动 compact 和 fork 后继续调用工具；它不验证自动 compact 阈值、完整 272K 窗口或机器人任务成绩。

任务版本继续由 `configs/task_families/current.json` 选择：本发布清单的 GPT Mosaic 是 R14，Oracle 展示仍使用自己的 R13 冻结录制。其他 worktree 的新草案不会自动进入此清单。`scripts.tasks` 的纯准备／来源检查入口仍保留；正式 CLI 启动使用 benchmark 入口生成的包。

## 时间、容量错误与上下文

从 START 起每项最多24小时，包括模型思考、工具、通信、fork 和容量等待。仿真服务自己执行截止：禁止新动作、请求制动、保存截止时评估、封存60Hz记录。停止后的稳定不能算作准时成功。Codex 宿主不再施加从进程创建起算的固定截止；物理进程连接仍有独立的工程启动检查；Slurm 作业时限覆盖启动、实验和收尾。已结束的运行不会循环重开。

已明确失败、属于容量／速率限制的模型轮次，等待至少10秒，在没有未解决物理调用时续接同一会话；不重发机械臂动作、不重置物理、不延长期限。身份／配额不足／普通任务错误不当作容量错误盲目循环。创建、fork 或投递回执未知时保留现场并停止自动重试。原调用编号的客户端恢复规则保持不变。

统一请求 `model_context_window=272000`、`model_auto_compact_token_limit=245000`。模型必须原生支持这个窗口，且有足够输出／系统空间；配置数值不会扩大服务端窗口。CLI catalog 的95%有效窗口预留仍可能使显示值约为258400。第三方各自的压缩行为、tokenizer 和 effort 档位需记录，不能把同名 effort 当作相同计算预算。厂商滚动别名可能升级，结果要保留实际返回型号与日期。

## 图像、动作与后续工厂展示

正式 CLI 包使用简单任务场景，按需通过 MuJoCo OpenGL Classic 返回策略所选相机的真实 RGB。机器人工具同时返回 JSON 与选定图片；路径不等于已经查看图片。隐藏服务不持续绘制预览，也默认不编码实时视频，避免36项一起占满渲染资源。

从 START 到结束的物理状态由原记录器按60Hz保存，包含机械臂与物体运动，以及材质事件；不会因为没有截图就丢动作。记录器在物理子步采样，60Hz之外的更高频变化受采样带宽限制。静止／细抖动裁剪只产生派生剪辑，不覆盖原流。之后可以离线以30fps在工厂 Cycles 中渲染 top/front，完全独立于模型取图。

## 会话与 Desktop

CLI 宿主使用持久化的原生 thread/start、turn/start、thread/fork；绑定真实 session ID，按父ID、ordinal 与历史哈希验证 fork。一个初始任务会话保持全新，只有任务内 delegate 可以继承它自己的历史。

会话保存在同一 CODEX_HOME，且有可读标题。status 输出每个任务／角色的 thread ID、真实观测状态；可用匹配 provider 配置的 CLI resume 查看。独立 app-server 的实时状态属于该宿主，不能把 Desktop 显示的 notLoaded 推断为停止，也不要在它运行时手工续轮。Desktop 列表可能按来源、provider、登录与项目过滤；目前未验收它自动显示全部第三方会话及实时状态，不改写 Desktop 私有数据库来伪造分身。官方 thread/list 也默认只列交互来源，审计时须显式包含 appServer／exec 等来源。[OpenAI App Server](https://learn.chatgpt.com/docs/app-server)

本机真实 CLI 的离线协议检查支持工具往返、返回图片与同边界 fork，使用本地 Responses stub，外部模型调用为0：

```sh
.venv/bin/python -m scripts.check_codex_cli --output runs/engineering/my-cli-protocol-check
.venv/bin/python -m scripts.check_codex_snapshot
```

当前工程未启动正式策略试验，未凭离线检查声称第三方厂商 API 已验收。物理／文件证据边界依旧是角色协议和工具认证，不是防恶意策略的操作系统隔离。
