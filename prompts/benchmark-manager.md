# 多模型基准调度 prompt

把下面的内容交给具备本机终端访问能力的 coding agent，并按需要改参数。
首次接手先读 `docs/multimodel-handoff.md`；查询型号/费用/endpoint 时再读
`docs/model-pricing-20261028.md`。此 prompt 不依赖特定外部模型或 Desktop 应用。

```text
你负责本仓库的 Codex CLI 基准工程调度。请先读取 docs/multimodel-handoff.md。

本次参数：
- models: [kimi]
- level: 1
- families: [transport, mosaic, relay, dial]
- scale: min-arms
- parallel: 4
- mode: smoke
- output: runs/policy/kimi-l1-small-001
- next_level: none

mode=plan：只输出计划；mode=smoke：完成本机检查和指定模型小型 API 测试后结束；
mode=run：完成完整 preflight，准备并执行上述批次，持续观察到终态。
mode=run 会产生 API 费用，参数中的授权只覆盖指定模型/任务/阶段。

步骤与完成条件：
1. 核对当前分支、Git提交和 configs/task_families/current.json，报告选中的发布版本。
   仅读取当前checkout和本次新批次；其他session的运行由其原操作者负责。
2. 用 scripts.benchmark_handoff doctor 检查工具、资产、所选模型的环境配置。
   若没有.env则用 init-env 创建；报告缺少的变量名，密钥内容保留在本机。
   缺密钥时完成 plan 并明确列出待用户填写项，结束付费测试部分。
3. 执行 scripts.benchmark plan，传入models、families、level、min-arms和parallel。
   展示准确case ID、臂数、模型、任务总数；只有mode=plan时到此结束。
4. mode=smoke：逐一运行指定模型的 preflight --smoke-only，报告每个真实回执路径
   与成功/失败，明确它不能代替完整检查；然后结束。
5. mode=run：逐一执行完整 preflight，只有全部所选模型通过后才prepare到全新output。
   完整回执必须匹配当前profile、catalog与CLI；失败保留证据并报告，不能改写passed。
6. 从持久终端执行 benchmark run <output> --parallel <parallel>，保存进程句柄。
   每30–60秒轮询本批次status和宿主日志，报告终态、等待/运行状态及工程异常。
   仅在有新进展时更新用户；不要反复将未知状态猜成完成或失败。
7. 全部任务终结后汇总各case结果、错误和输出位置。next_level=none则结束；若参数
   显式指定下一等级，使用同样模型和家族、对应最少臂选择、全新output完成下一批。

实验边界：任务决策、图片理解、机器人操作由内部被测模型完成；外层只使用官方
benchmark与工程检查入口。保留冻结输入、原始日志和私有评分隔离。外层不读取
Oracle答案/私有Dial谜底，不给任务内agent注入动作策略，不修改运行中的任务或
替模型做机器人动作。报告模型profile与effort，不以外层session的模型名代称。

遇到API/配额/协议错误时保留原批次并报告，不擅自换模型、降低实验要求、重启世界
或自动升级CLI。需要停止本批次时使用benchmark stop。并行上限针对当前批次任务，
不要额外再启动一组相同任务来“加速”。
```

只需把 `models` 改为 `[qwen]`、`[glm]` 或 `[deepseek]` 即可单模型试跑。
`models: [kimi, qwen, glm, deepseek]` 配合 Level 1/min-arms 会生成16项；
parallel=4 表示最多同时4项。若希望四类合计4项，只选择一个模型。
填 `mode: run` 才会执行机器人实验；默认示例只做连通小测。
