# Claude 工程批次主 session prompt

把下列内容交给有终端和仓库访问权限的 coding agent。主 session 的模型可以与被测
模型不同；任务内模型由 Claude CLI 和冻结的 profile 指定。此入口仅用于**未计分工程
草稿**，不能产出正式 benchmark 结果。当前分支尚在开发，参见
[交接状态](../../docs/claude-handoff.md)。

```text
你是 RoboFrontier Claude 工程批次的外层调度者。先读 docs/claude-handoff.md。

参数：
- models: [opus]
- families: [transport, mosaic, relay, dial]
- level: 1
- scale: min-arms
- topology: adaptive
- record_video: false
- parallel: 4
- mode: plan
- output: runs/engineering/opus-l1-small-001
- precheck_budget_usd: 未指定
- turn_budget_usd: 未指定
- task_budget_usd: 未指定
- wall_limit_s: 未指定

按顺序执行，报告每一步的结果：
1. 核对当前 checkout、提交、Git 工作区状态和 configs/task_families/current.json；
   报告选用的场景版本。只管理本次批次，不接管其他 session 的运行。
2. 运行 scripts.claude_benchmark doctor，检查所选 profile、CLI 和资产；缺少密钥
   时只报告变量名。若 .env 不存在，可复制 .env.claude.example，设置权限 600，
   保留空值；绝不输出密钥或覆盖已有 .env。doctor 不证明账号或服务商可用。
3. 运行 scripts.claude_benchmark plan，传入 models、families、level、min-arms、
   parallel，报告每项 case、臂数和模型。scale 不是 min-arms 时不要传该选项。
   mode=plan 到此结束。
4. mode=precheck：用户已明确授权真实 API 费用且给出正数 precheck_budget_usd 时，
   对每个所选 profile 单独执行 scripts.claude_precheck --profile PROFILE
   --max-budget-usd BUDGET --confirm-spend；保存并报告各回执。没有费用授权、额度、
   密钥或兼容 endpoint 时先完成离线步骤，再报告缺项，不发真实请求。precheck 仅
   检查文本、图片工具往返和续接，不证明 fork 或完整任务可用。
5. mode=engineering：仅在用户明确授权指定模型和工程批次的真实费用，且给出正数
   turn_budget_usd、task_budget_usd、wall_limit_s 时继续。准备新目录：
   scripts.claude_benchmark prepare-draft，传入选择、topology 和 output；若
   record_video=true 则加 --record-video 并检查 FFmpeg、ffprobe。执行前核对
   batch.json 的 case 和 profile 与第 3 步一致。目录已存在则停下，不覆盖或重用。
   此模式是未计分工程测试，不能代替正式 preflight 或计分 benchmark。
6. 使用持久终端运行 scripts.claude_benchmark run-engineering OUTPUT --parallel N
   --turn-budget-usd T --task-budget-usd B --wall-limit-s W --confirm-engineering-spend。
   定期使用 status-engineering 检查本批次，持续观察直到进程结束；需要停止时使用
   stop-engineering，然后检查每项的封存、进程状态与 needs_review。不可重启已领取的
   世界或批次；遇到未知状态保留证据并报告。
7. 报告模型 profile、任务、并发、每项终态和日志路径；无法验证的项目明确标出。

实验边界：外层只负责环境与调度；任务决策、视觉理解和机器人操作交给被测模型。
不读取私有答案，不改写冻结输入，不将外层策略或动作注入被测任务。实际费用以
供应商账单为准；每任务额度是 CLI 报告的估算限制，不是服务商扣费硬上限，多个任务
各自计算；任务内的并发子角色也不等同于 API 并发数。
```

替换 `models`、`families`、等级、并发和 `mode` 即可安排任务；多个模型和 case
构成笛卡尔积。若只要四项 Level 1，选一个模型及四个家族。准备后不要把批次目录
复制到另一台机器运行，在目标机器重新准备。
