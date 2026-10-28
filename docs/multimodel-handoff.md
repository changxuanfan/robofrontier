# 多模型交接与本机启动

本分支从 Harness 的 `codex/server-benchmark-prep`、提交
`3638e1c351b6e885ac94e3256f5782f8aecd46ee` 创建，分支名为
`codex/multimodel-handoff`。聊天 session 的 fork 不会复制 Git 工作树；本次已单独
核对并切换到上述 Harness 提交。原 Harness 的运行由原 session 管理。

## 交接范围与版本

代码通过 Git 交接；大型运行资产另行交接；每台机器独立建立 `.env`、Python 环境
和新实验目录。此分支沿用 Harness 的 `configs/task_families/current.json`：
Transport R14、Mosaic R14、Relay 最终 Goal 规则、Dial R11 BEST5。
其他 worktree 的 Mosaic R15 或旧 Terra 实验不在此分支。若实验要求 R15，先在
发布清单中完成相应变更、验证和资产更新，再准备新批次。

这里的“主 session”负责准备、启动、检查工程错误和报告；任务内的模型由
`configs/models.json` 与冻结的 `codex.json` 决定。主 session 可以使用其他 coding
agent，只要具备本机命令行和文件访问能力并遵守[调度 prompt](../prompts/benchmark-manager.md)。
它仍可能通过修改输入或注入策略影响结果，所以实验边界也必须执行：任务决策、
看图和机器人动作全部由内部被测模型完成，外部 session 只做工程调度。

## 从 GitHub 或离线包接手

Git 提交记录当前分支的已暂存改动。worktree 是该分支的工作目录，`main` 是另一个
分支；在这里 commit 不会自动提交到 `main`。交接这个分支无需先合并 main。
GitHub 交接仓库为私有的 [changxuanfan/robofrontier](https://github.com/changxuanfan/robofrontier)。
仓库保留原 main 与本交接分支；后续在自己的分支开发、push，再通过 PR 合并。
原机器已配置 origin；更新交接分支时：

```sh
git push -u origin codex/multimodel-handoff
```

仓库所有者先在 GitHub Settings → Collaborators 邀请同事。同事接受邀请、获得写权限后，
可以 clone、建立自己的分支并 push：

```sh
git clone --branch codex/multimodel-handoff https://github.com/changxuanfan/robofrontier.git roomscript2
cd roomscript2
git switch -c experiments/my-model
# 修改完成后 git add、git commit，再发布自己的分支
git push -u origin experiments/my-model
```

无需 GitHub 也可交接 `roomscript2.bundle` 和 `policy-assets.tar.gz`：

```sh
git clone --branch codex/multimodel-handoff /path/to/roomscript2.bundle roomscript2
cd roomscript2
```

bundle 是仓库历史快照，不是持续同步服务。日常协作用上述 GitHub 仓库即可；
bundle 供离线交接或备份使用，后续更新可通过 GitHub 或新 bundle 传递。
资产包只包含当前36项任务需要的场景资产和 Panda 来源文件；不包含 `.env`、Codex
认证、实验结果、已准备世界或 Dial 私有答案。机器人上游许可证随 Panda 资产保存。
接收者核对随包提供的 SHA-256 后执行恢复命令；恢复也会检查发布清单和每个文件哈希，
拒绝覆盖内容不同的现有资产。

## 安装与环境变量

需要 Python 3.12、uv、Node.js、Codex CLI、ffmpeg；图形驱动按操作系统配置。
本次本机测试 CLI 为 `codex-cli 0.164.0-alpha.11.1`。建议首次复现采用相同版本；
换 CLI 后必须重新做完整 preflight。

```sh
uv sync --locked --extra oracle
# 安装指定版本的 CLI；已有合适版本可跳过
npm install -g @openai/codex@0.164.0-alpha.11.1
codex --version
.venv/bin/python -m scripts.benchmark_handoff restore /path/to/policy-assets.tar.gz
.venv/bin/python -m scripts.benchmark_handoff init-env
```

`init-env` 仅在文件不存在时创建，权限为600。若 `.env` 已存在，直接编辑即可。
填写要用的模型，空着的模型不做联网测试：

```dotenv
KIMI_API_KEY=
DASHSCOPE_API_KEY=
DASHSCOPE_BASE_URL=
ZAI_API_KEY=
DEEPSEEK_API_KEY=
```

`.env` 已被 Git 忽略。程序按字面量读取，不执行 shell；现有进程环境变量优先于
`.env`。不必 `source .env`。同事使用自己的 API Key，无需接收你的 ChatGPT 登录。
如果在服务器运行，Key 只在执行 CLI 的那台机器配置；本地编排 SSH 不会自动把本地
环境变量传到远程。当前工具以 macOS/Linux 为目标，使用 POSIX 文件锁。

四家的型号、价格、官方接口与注意事项见[模型与价格](model-pricing-20261028.md)。
千问必须填写实际 workspace/region 的 **Responses base URL**，例如
`https://<WorkspaceId>.cn-beijing.maas.aliyuncs.com/compatible-mode/v1`。
其他三家的 endpoint 已写入模型配置，密钥仍从环境读取。
生成的 provider 配置使用 `wire_api="responses"`，为该模型声明图片输入；不会修改
用户全局 Codex 默认模型或登录。OpenAI 的配置说明见
[自定义 provider](https://learn.chatgpt.com/docs/config-file/config-advanced)。

## 先小测，再正式运行

先运行不调用付费模型的检查：

```sh
.venv/bin/python -m scripts.benchmark_handoff doctor --models kimi qwen glm deepseek
.venv/bin/python -m scripts.check_codex_cli --output runs/engineering/local-cli-check
.venv/bin/python -m scripts.benchmark plan --models kimi --level 1 --min-arms --parallel 4
```

doctor 只报告密钥是否存在，不输出内容；它不是账号权限或联网成功证明。
本地 CLI 检查使用本机 Responses stub，验证 CLI 接口、图片回传及 fork，不计为各家
API 通过。`plan` 输出任务数、臂数、来源与 qualification，不创建世界或请求模型。

填好一个 Key 后先做小型真实 API 测试，例如：

```sh
.venv/bin/python -m scripts.benchmark preflight --models kimi --smoke-only
```

它给模型一张合成红蓝图片，要求调用检查工具并回复。通过才证明本账号在该配置下
能完成这一小段图片/函数工具往返，不代表整个机器人实验已经适配。输出在
`runs/engineering/model-smoke/kimi/report.json`。可依次替换 `kimi` 为
`qwen`、`glm`、`deepseek`；这些命令会产生真实 API 费用。

正式运行要求另做完整检查：

```sh
.venv/bin/python -m scripts.benchmark preflight --models kimi
```

完整检查增加手动 compaction 和 fork 后的历史/工具使用，成功回执位于
`runs/engineering/model-compatibility/kimi/report.json`。smoke 回执不能代替它。
完整检查仍未压测272K上下文或自动压缩，也不能独立证明服务器背后的实际模型身份。
更改型号、endpoint、effort、模型 catalog 或 CLI 版本后重新检查。

## 分配任务和控制并行

`--models` 接受 profile：`kimi qwen glm deepseek`。`--families` 可选
`transport mosaic relay dial`，`--cases` 指定准确任务 ID；`--level` 选择等级，
`--min-arms` 在每个选定家族中读取任务元数据，挑选该等级最少臂的规模。

| 选择 | 每个模型任务数 | 四个模型合计 |
| --- | ---: | ---: |
| Level 1 + 每类最少臂 | 4 | 16 |
| Level 1 + 全部规模 | 12 | 48 |
| 全部等级、规模 | 36 | 144 |

Level 1 最小四项为 Transport 4臂、Mosaic 4臂、Relay 8臂、Dial 4臂；Level 2
对应4、4、8、8臂。`dial-...-n04` 的数字是拨盘数，并不总是臂数。

下面只运行 **一个模型的四项 Level 1**：

```sh
.venv/bin/python -m scripts.benchmark prepare --models kimi --level 1 --min-arms --output runs/policy/kimi-l1-small-001
.venv/bin/python -m scripts.benchmark run runs/policy/kimi-l1-small-001 --parallel 4
```

`prepare` 冻结代码、公开任务、配置和独立世界输入，不启动模型。`run` 在前台等待
整个批次完成。主 session 应用其持久终端执行/轮询功能保存进程，并从另一命令读取
状态；服务器长期作业可放 tmux 或使用现有 Slurm 作业入口。不要依赖 SSH 断开后
普通前台进程继续运行。本文不为每个机器人 fork 单独提交 Slurm 作业。

`--parallel N` 是本批次同时运行的独立任务上限，范围1–36，默认9，建议首次显式写4。
每个任务可因内部机器人分工产生多个会话；它不是 API 请求数或供应商并发限额。
多个批次同时跑时，各批次的 N 会相加。CPU、内存、渲染与账号 RPM/TPM 均可能
限制吞吐；先小规模测试，再按实际占用增加。

若要四模型对照且共用同批次生成的 Dial 随机题，可在同一个 prepare 中使用
`--models kimi qwen glm deepseek`。得到16项后 `--parallel 4` 会分批执行；所有任务
运行状态隔离。若一个模型先单独运行，再在另一批次运行其他模型，Dial 会生成新题。
当前 Harness 的上下文为272000，自动压缩阈值245000；各家 effort 等级不同，
同一字符串不代表相同计算量，比较时保留这些配置。

查看与停止只针对自己的批次：

```sh
.venv/bin/python -m scripts.benchmark status runs/policy/kimi-l1-small-001
.venv/bin/python -m scripts.benchmark stop runs/policy/kimi-l1-small-001
```

STOP 由宿主按协议处理，不是立即 kill；未启动任务会跳过。批次和世界不自动重启，
运行已启动后再次执行 run 会拒绝。失败保留原目录，检查工程原因；需要新实验时
换新输出目录。默认每任务最多24小时；这不是金额上限。单项日志与输出在
`<batch>/<profile>/<case>/private/outputs/`，包括宿主日志和协议证据；图片来自
任务工具的观察/初始公开图，Codex CLI 不是自动截屏桌面的程序。查看结果不等于
给任务内模型补充策略。

Level 2 使用新目录和 `--level 2 --min-arms`。只有用户要求下一阶段时才接续；
调度 prompt 可以表达“Level 1全部结束后再跑Level 2”，但该能力由外层调度执行，
并非 run 命令隐含升级。

## worktree 协作与回到 main

一个 worktree 绑定一个独立分支，同一分支不能同时被两个 worktree 检出。同事在
自己的 clone 内建立分支；同一机器的额外实验可这样创建：

```sh
git worktree add -b experiments/kimi-l1 ../roomscript2-kimi codex/multimodel-handoff
```

共享 Git 历史，隔离代码改动、`.env`、`runs/` 和工作目录。Python 环境可按机器重建，
大型只读资产可复制；运行输出和准备世界始终各自独立。不要把已 prepare 的批次
目录挪到另一机器执行，它含冻结的绝对路径；在目标机器重新 prepare。

完成审查和必要测试后，在 main 所在工作树合并分支（也可通过 GitHub PR）：

```sh
git status --short
git merge --no-ff codex/multimodel-handoff
```

执行前确认当前分支是 main 且工作区干净。main 后续有新提交时，先在本分支合并
main 并解决冲突、验证，再提交 PR。不要强推覆盖同事分支。Git 合并只合并已跟踪
代码；资产包、密钥与实验结果仍单独保管。
