# 四家视觉模型与 Codex 接口

八家模型的 Codex 适配评价、缓存写入/存储费用、RPM/TPM与并发限制，见
[完整比较报告](model-compatibility-20261028.md)。本页保留现有四个 profile 的快速配置说明。

核对日期：2026-10-28。以下选择面向本项目需要的原生图片理解、函数工具和长程
agent 调用。各家官方定位不能构成跨厂商准确率排名；谁更适合机器人任务要由
本基准验证。价格为实时按量 API 原价，每百万 token，输入列为非缓存价格。
订阅、Coding Plan、优惠或 Batch 价不能直接代替此表。

| 厂商 / 本项目 profile | 所选视觉模型 API ID | 输入 / 1M | 输出 / 1M | 缓存命中输入 / 1M |
| --- | --- | ---: | ---: | ---: |
| Kimi / `kimi` | `kimi-k3.1-pro` | ¥13 | ¥68 | ¥1.30 |
| 千问 / `qwen` | `qwen3.8-max-0902` | ¥12 | ¥36 | ¥1.5 隐式；¥1 显式 |
| 智谱 / `glm` | `glm-5.3-flash` | ¥0.8 | ¥2.8 | ¥0.23 |
| DeepSeek / `deepseek`，非高峰 | `deepseek-flash` | US$0.15 | US$0.60 | US$0.003 |
| DeepSeek / `deepseek`，高峰 | 同上 | US$0.30 | US$1.20 | US$0.006 |

Kimi 官方将 K3.1 Pro 定位为迄今最强旗舰并声明原生视觉；另有同等智能的 Highspeed
版本，输入¥26、输出¥136、缓存命中¥1.30。缓存写入基础展示价为¥13/1M；
普通输入、写入、命中 token 互斥，同一token不重复收费。默认缓存5分钟；
1小时 Pro 写入单价本次未核实。[Kimi 官方模型与价格](https://platform.kimi.com/)、
[缓存计费](https://platform.kimi.com/docs/guide/context-caching)

千问采用固定快照而非滚动别名。表中为北京地域；新加坡等地域价格可能不同，
例如新加坡该档输入¥14.988、输出¥44.965。图片编码也消耗输入 token。
[千问视觉模型](https://help.aliyun.com/zh/model-studio/vision-model)、
[百炼价格](https://help.aliyun.com/zh/model-studio/model-pricing)

智谱5.3系列中，普通 GLM-5.3 当前列为文本输入；Flash/FlashX 是原生多模态系列。
本项目采用 Flash，FlashX 是更快的服务档，价格输入¥2、输出¥7、缓存命中¥0.57，
不能仅因名字多一个 X 就推断准确率更高。缓存存储目前限时免费。
[GLM-5.3-Flash 模型](https://docs.bigmodel.cn/cn/guide/models/vlm/glm-5.3-flash)、
[智谱价格](https://docs.bigmodel.cn/cn/guide/start/pricing)

DeepSeek 的 `deepseek-flash` 当前对应 DeepSeek-V4.1-Flash-1017，支持视觉；
V4 Pro 当前未列视觉支持。高峰为周一至周五 UTC 01:00–04:00、06:00–10:00，
即香港09:00–12:00、14:00–18:00，中国公共假期除外，其余为非高峰。
此行单位是美元，其余三家为人民币，没有直接进行汇率换算。
[DeepSeek 型号、视觉与价格](https://api-docs.deepseek.com/quick_start/pricing/)

费用取决于每次请求的历史输入、图片编码、缓存和推理/输出用量，不能用“4任务”直接
推算固定金额。本项目会保留 native usage 事件；账单和最新价格以供应商控制台为准。
外部调度 session 的费用还需另算。

## 配置映射

| profile | base URL | Key 环境变量 | reasoning effort |
| --- | --- | --- | --- |
| `kimi` | `https://api.moonshot.cn/v1` | `KIMI_API_KEY` | `high` |
| `qwen` | 从 `DASHSCOPE_BASE_URL` 读取 workspace/region URL | `DASHSCOPE_API_KEY` | `xhigh` |
| `glm` | `https://open.bigmodel.cn/api/v1` | `ZAI_API_KEY` | `max` |
| `deepseek` | `https://api.deepseek.com` | `DEEPSEEK_API_KEY` | `high` |

这些配置沿用 Harness 的 `configs/models.json`，全部使用 Responses。
支持 Chat Completions 或兼容 OpenAI 格式不足以证明适配本 Harness；还需要
图片输入、动态函数工具、工具续轮、compaction 与 fork 历史可用。

Kimi 与 DeepSeek 已提供 Codex 官方接入文档；千问提供 Responses 的地域/workspace
地址；智谱的 Responses 地址与普通 `/api/paas/v4` Chat 地址不同。账号是否开通
该具体模型与接口必须通过本机 preflight 验证；不要在未验证时宣称四家都已跑通。
[Kimi Codex](https://platform.kimi.com/docs/guide/codex-kimi)、
[千问 Responses](https://help.aliyun.com/zh/model-studio/qwen-api-via-openai-responses)、
[智谱 Responses](https://docs.bigmodel.cn/cn/guide/develop/responses/introduction)、
[DeepSeek Codex](https://api-docs.deepseek.com/quick_start/agent_integrations/codex/)

运行步骤、最少臂选择、并发含义与离线交接见[交接指南](multimodel-handoff.md)。
