# 八家中国视觉模型：Codex 适配、缓存价格与并发限制

核查日期：2026-10-28，Asia/Hong_Kong。采用厂商官方来源，针对本项目的机器人
截图、动态函数工具、多轮工具结果、长历史、压缩与 fork 流程。以下是普通实时按量
API，不混用 Coding Plan、M Plan、Token Plan、Step Plan、Batch 或第三方转发价格。
未调用付费推理、未读取密钥、未验证当前账户私有配额。

## 适配评价与当前工程状态

“适配程度”评价的是接入证据与工程改造量，不是机器人成功率排名。
当前配置只有 Kimi、Qwen、GLM、DeepSeek 四家；新增四家尚未加入运行配置。
这八家均没有在本交接工作树完成真实 API preflight，不能称为已端到端跑通。

| 模型 | Codex 接入评价 | 本 Harness 的状态与待验证点 |
| --- | --- | --- |
| `kimi-k3.1-pro` | 高：厂商有原生 Responses + Codex 指南 | 已有 `kimi` profile；验证图片工具回传、长程续轮、压缩/fork、实际 reasoning 参数。[官方接入](https://platform.kimi.com/docs/guide/codex-kimi) |
| `deepseek-flash` | 高：厂商有原生 Responses + Codex 指南 | 已有 `deepseek` profile；当前别名是 V4.1-Flash-1017，支持视觉，不能替换为当前不支持视觉的 V4 Pro。[官方接入](https://api-docs.deepseek.com/quick_start/agent_integrations/codex/)、[型号](https://api-docs.deepseek.com/quick_start/pricing/) |
| `MiniMax-M3.1-Flash` | 高：有详细 Codex 模型目录配置 | 尚未配置；按官方声明 text/image、reasoning 支持；支持 low/medium/high/xhigh/max，不支持 none。工具循环与图像结果仍须测。[官方接入](https://platform.minimax.cn/docs/m-plan/codex) |
| `qwen3.8-max-0902` | 较高：有 Responses 图片、函数接口 | 已有 `qwen` profile；需要账户地域/workspace 对应地址；Session 缓存需另加 header；超约80%上下文可能自动截断。[官方接口](https://help.aliyun.com/zh/model-studio/qwen-api-via-openai-responses) |
| `glm-5.3-flash` | 较高：有 Responses 图片、函数接口 | 已有 `glm` profile；Responses 地址为 `/api/v1`，不是 Chat 的 `/api/paas/v4`。核实账户可用型号与多模态工具结果。[官方接口](https://docs.bigmodel.cn/cn/guide/develop/responses/introduction) |
| `doubao-seed-2-1-pro-260915` | 较高，但需区分订阅与按量端点 | 尚未配置；厂商 Codex 指南用 Coding Plan 端点，不能据此把订阅配额套入按量 API。多轮保留 reasoning 加密内容，测试完整请求历史。[官方 Codex](https://docs.volcengine.com/docs/ark/coding-plan-personal-ai-codex?lang=zh)、[深度思考](https://docs.volcengine.com/docs/ark/deep-thinking?lang=zh) |
| `mimo-v2.6-pro` | 官方支持强，但当前模板存在明确差异 | 尚未配置；官方目录 `use_responses_lite=true`，当前 Harness 强制 false。厂商提示 custom 工具需要 lite，必须先核对动态工具路径。[官方指南](https://mimo.mi.com/docs/en-US/tokenplan/integration/codex-configuration)、[官方目录](https://example-files.cnbj1.mi-fds.com/example-files/configs/model-catalogs.json) |
| `step-5-preview` | 有官方接入，但机器人流程适配有待验证 | 尚未配置；Codex 教程用 Step Plan，普通按量也有 `/v1/responses`。按量文档只承诺 function、tool_choice=auto、字符串工具结果；未证明图片工具回传与 custom/freeform 支持。[官方 Codex](https://platform.stepfun.com/docs/zh/guides/developer/integrations/codex.md)、[按量 Responses](https://platform.stepfun.com/docs/zh/api-reference/responses/responses-create.md) |

这些评价不承诺八家采用相同协议子集。OpenAI 支持自定义 provider、环境变量密钥与
请求头，但服务商是否实现具体图片/工具字段仍由其接口决定。
[Codex 配置文档](https://learn.chatgpt.com/docs/config-file/config-advanced)

工程审计基于 [launch_config](../scripts/benchmark.py)、[模型配置](../configs/models.json)、
[模型目录模板](../configs/codex-model-template.json) 和 [preflight](../scripts/codex_preflight.js)：

- 当前 smoke 检查是用户附图 → 函数调用 → 文本工具结果；完整检查再加手动压缩和 fork。
  **尚未实测供应商接受工具返回的图片，并据此继续动作**。离线 stub 验证不能代替这项。
- 当前模板没有显式声明 `supports_reasoning_summaries=true` 或对应配置强制项。
  MiMo/MiniMax 指南指出该标记关系到是否发送 `reasoning.effort`；应检查实际请求，
  不能只凭本地 `high/max` 标签认定推理强度已经生效。
- 当前每请求上下文预算272,000，自动压缩阈值245,000；不是每轮一定发送272K。
  尚未压测完整窗口或自动压缩行为。

建议接入顺序属于工程判断：先验证已有四家；新增优先 MiniMax；MiMo 先处理 lite
与动态工具兼容；Doubao 核对按量端点；Step 先攻克图片工具结果。最终任务能力只能
通过同一冻结输入、相同预算与记录完整 usage 的实验比较。

## 普通按量价格

单位：每 **1,000,000 tokens**。除 DeepSeek 单独标明美元外，其余为中国端人民币；
不做汇率换算。图片编码以及推理输出也可能产生费用。

| 模型／地域 | 未缓存输入 | 缓存命中输入 | 输出 | 官方价格 |
| --- | ---: | ---: | ---: | --- |
| Kimi K3.1 Pro／中国 | ¥13 | ¥1.30 | ¥68 | [Kimi](https://platform.kimi.com/) |
| Qwen3.8-Max-0902／北京 | ¥12 | ¥1.5 隐式；¥1 显式 | ¥36 | [Qwen](https://help.aliyun.com/zh/model-studio/qwen3-8-max) |
| GLM-5.3-Flash／中国 | ¥0.8 | ¥0.23 | ¥2.8 | [智谱](https://docs.bigmodel.cn/cn/guide/start/pricing) |
| DeepSeek Flash／非高峰 | US$0.15 | US$0.003 | US$0.60 | [DeepSeek](https://api-docs.deepseek.com/quick_start/pricing/) |
| DeepSeek Flash／高峰 | US$0.30 | US$0.006 | US$1.20 | [DeepSeek](https://api-docs.deepseek.com/quick_start/pricing/) |
| Doubao Seed 2.1 Pro **260915**／北京 | ¥6 | ¥1.20 | ¥30 | [方舟](https://docs.volcengine.com/docs/ark/model-pricing?lang=zh) |
| MiMo V2.6 Pro／中国 | ¥3 | ¥0.025 | ¥6 | [MiMo](https://mimo.mi.com/docs/pricing) |
| MiniMax M3.1 Flash／中国 | ¥1 | ¥0.10 | ¥4 | [MiniMax](https://platform.minimax.cn/docs/pricing/overview.md) |
| Step 5 Preview／中国 | ¥7 | ¥0.70；合资格老用户¥0.35 | ¥20 | [StepFun](https://platform.stepfun.com/docs/zh/guides/pricing/details.md) |

DeepSeek 高峰：周一至周五香港09:00–12:00、14:00–18:00，中国公共假期除外。
其余时间适用非高峰价。[时段规则](https://api-docs.deepseek.com/quick_start/pricing/)

Step 缓存老用户条件：2026-10-14 00:30 北京时间前已通过普通按量 API 调用过
Step 5 Preview；不是只要注册较早即可。[调价公告](https://platform.stepfun.com/docs/zh/announcements/step-5-preview-price-adjustment.md)

这里不套用旧型号的长度加价：Doubao 260915 输入0–1024K同价，旧260628是256K；
MiMo V2.6 Pro 没有列长上下文加价；MiniMax M3 的512K阶梯不属于 M3.1 Flash。
[方舟型号](https://docs.volcengine.com/docs/ark/model-list?lang=zh)、
[MiMo 价格](https://mimo.mi.com/docs/pricing)、
[MiniMax 价格](https://platform.minimax.cn/docs/pricing/overview.md)

## 缓存写入、存储与生效条件

| 厂商 | 写入／存储与期限 | 当前 Harness 的边界 |
| --- | --- | --- |
| Kimi | 自动前缀缓存，默认5分钟，支持1小时；首页写入¥13/M未注明TTL，1小时 Pro 写入价本次未核实 | 普通输入、写入、读取 token 互斥，同一 token 不重复收¥13+¥13。[缓存计费](https://platform.kimi.com/docs/guide/context-caching) |
| Qwen | 隐式自动、期限不固定；显式创建¥15/M、命中¥1/M，5分钟命中续期 | 未配置 Session header，仍可能命中隐式缓存。`x-dashscope-session-cache: enable` 才开启 Session；该型号优先显式缓存。[缓存](https://help.aliyun.com/zh/model-studio/context-cache)、[Session](https://help.aliyun.com/zh/model-studio/compatibility-with-openai-responses-api) |
| GLM | 自动隐式缓存；存储限时免费；未公布固定TTL，没有单列写入价 | 建议重复前缀≥500 tokens；检查真实 usage。[缓存](https://docs.bigmodel.cn/cn/guide/capabilities/cache)、[计费](https://docs.bigmodel.cn/cn/guide/start/pricing) |
| DeepSeek | 自动缓存；未单列写入/存储费用；不活跃缓存通常数小时至数天清理 | best effort，不保证每次命中或固定TTL。[缓存](https://api-docs.deepseek.com/guides/kv_cache/) |
| Doubao | 隐式自动，存储免费；显式存储¥0.017/M tokens/小时，不足1小时按1小时，最长7天 | 新增输入按普通输入价；显式需配置 `caching`，与隐式互斥。[缓存](https://docs.volcengine.com/docs/ark/context-cache?lang=zh) |
| MiMo | 缓存写入限时免费；未公布固定TTL或单独存储价 | 不能承诺永久免费。[定价](https://mimo.mi.com/docs/pricing) |
| MiniMax | 自动前缀缓存，输入≥512 tokens；新增输入普通价，没有单列写入/存储项 | 未公布固定TTL，只有实际命中才能按缓存价计算。[缓存](https://platform.minimax.cn/docs/api-reference/text-prompt-caching.md) |
| Step | 自动前缀缓存，256-token块；首次写入计普通输入价，无额外缓存购买费 | LRU淘汰，无固定TTL；相同图片/视频、工具及结果可参与缓存。[缓存](https://platform.stepfun.com/docs/zh/guides/developer/prompt-cache.md) |

以上八家均未找到适用于本表公共按量 API 的明确“缓存 token 不计 TPM”保证。
不能把价格折扣代入 TPM，也不能把专属 PTU、吞吐预留或订阅折算套过来。
实际命中应查原始响应 usage；仅看到相同系统提示词不能证明有缓存折扣。

## RPM、TPM 与同时请求数

RPM=每分钟请求数，TPM=每分钟 token 配额，“并发请求”=同时尚未结束的请求。
“未公布/未核实”不表示无限，也不代表当前账户必然获得表中基线。

| 模型／普通按量账户 | 同时请求数 | RPM | TPM | 额度范围与来源 |
| --- | ---: | ---: | ---: | --- |
| Kimi K3.1 Pro | 按充值等级，数值未核实 | 同左 | 同左，另有TPD | 用户级，全部模型/Key共享；公开动态表本次不可读取，以控制台为准。[官方](https://platform.kimi.com/docs/pricing/limits) |
| Qwen3.8-Max-0902／北京 | 未公布 | 未给数字 | **1,500,000** | 主账号+模型，所有workspace/Key共享；三个月消费档的该快照基线相同。[官方](https://help.aliyun.com/zh/model-studio/quota-management) |
| GLM-5.3-Flash | 账户等级+模型，以控制台为准 | 未公布 | 未公布 | 不使用论坛数字或 Coding Plan 项目数替代。[官方](https://docs.bigmodel.cn/cn/api/rate-limit) |
| DeepSeek Flash | **2,500/账户** | 未公布 | 未公布 | 从发出到响应结束，所有Key共享；超额429。[官方](https://api-docs.deepseek.com/quick_start/rate_limit/) |
| Doubao 260915／北京 | 未公布，低充值可能受限 | **500** | **1,000,000** | 主子账号及同模型接入点共享；公开最大值不构成容量保障。[模型表](https://docs.volcengine.com/docs/ark/model-list?lang=zh)、[FAQ](https://docs.volcengine.com/docs/ark/modelark-faq?lang=zh) |
| MiMo V2.6 Pro | 未公布数字 | **100** | **10,000,000** | 同账号同模型全部Key合计。[官方](https://mimo.mi.com/docs/en-US/quick-start/model) |
| MiniMax M3.1 Flash／免费 | 未公布 | **20** | **1,000,000** | 主子账号共享。[官方](https://platform.minimax.cn/docs/guides/rate-limits.md) |
| MiniMax M3.1 Flash／充值 | 未公布 | **200** | **10,000,000** | TPM含输入+输出，可申请提高。[官方](https://platform.minimax.cn/docs/guides/rate-limits.md) |
| Step 5 Preview／V0个人 | **5** | **100** | **500,000** | 其他充值档见下表。[官方](https://platform.stepfun.com/docs/zh/guides/pricing/rate-limits.md) |

千问固定快照不能套用滚动别名 `qwen3.8-max` 的500/1000/2000万TPM。
不同地域也不能混用额度；例如其部分海外/全球部署另列30,000 RPM、150,000 TPM。
[型号地域表](https://help.aliyun.com/zh/model-studio/qwen3-8-max)

Step 中国个人账户按累计现金充值分级，代金券不计入；企业联系商务。
平台可随负载调整，账户需要实名认证。[官方限流](https://platform.stepfun.com/docs/zh/guides/pricing/rate-limits.md)

| Step 等级 | 累计现金充值人民币 | 同时请求 | RPM | TPM |
| --- | --- | ---: | ---: | ---: |
| V0 | <100 | 5 | 100 | 500,000 |
| V1 | 100–499 | 20 | 400 | 2,000,000 |
| V2 | 500–1,999 | 30 | 600 | 3,000,000 |
| V3 | 2,000–9,999 | 40 | 800 | 4,000,000 |
| V4 | ≥10,000 | 130 | 2,600 | 13,000,000 |

## 怎样把限额转成实验并行数

一个 Level 1 任务内部可能产生多个 Codex 会话。四任务、四条 CLI 进程、四个机械臂、
四个同时 API 请求是四种不同计数；不能从API并发额度直接推出可开多少实验。

工程建议是按供应商账户统一限制同时请求，并对 RPM/TPM 做预算与429退避。
当前代码主要限制任务并行度，不应把它宣传成已经实现账户级API限流。
多个任务各自重试也可能一起冲击共享限额；增加同账户Key通常不能扩容。

可粗估 `每分钟请求数 ≤ min(RPM, TPM / 每请求计入限流的平均token)`，
同时请求数还取决于响应耗时。实际供应商可能预扣最大输出、按秒限制或动态限流，
因此这不是吞吐承诺；Kimi 与 Doubao 尤其要核实预扣规则。
[Kimi](https://platform.kimi.com/docs/introduction)、
[Doubao](https://docs.volcengine.com/docs/ark/activation-management?lang=en)、
[Qwen](https://help.aliyun.com/zh/model-studio/rate-limit)

例如假设每次请求计入限流100K tokens，500K TPM只能支持约5次请求/分钟，
即使账户标称100 RPM也不能按100 RPM调度。这是算术示例，不是实测token用量。

建议先每家单任务完成图片工具回传、continuation、压缩/fork检查，再测四项最少臂
Level 1，记录每轮token、延迟、cache hit与429后逐步加并行。Step V0 的5连接与
500K TPM尤其需要注意；其他未公布连接数的厂商也不能直接承诺12任务稳定并行。
选型时，GLM/MiniMax 的人民币输入和输出价格低；MiMo 的命中输入价格低，但仍
要合计未命中、输出、缓存写入与实际成功率。DeepSeek 保留美元报价单独核算。

操作步骤见[交接指南](multimodel-handoff.md)。本次只更新研究资料，不改变默认模型、
CLI配置、正式实验或前一个 Harness session 的运行状态。
