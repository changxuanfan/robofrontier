# RoboFrontier

当前主线是四类、36 个环境的 Frontier V2：物品传递、拼图、空中接力和旋钮密码锁。当前环境草稿为9项Transport R13＋其余27项R10（80 cm桌高、2 ms步长）；旧版保留。R13已做局部操作验证与双臂串／并行对照，尚未运行完整Oracle。R10现有同配置证据为**18/36完整通过**，不代表效率合格，也不能继承给新版。不能把静态构建或局部成功当成整项通过。

另按用户授权完成了最小规模的 **12项限时GPT试验**：Adaptive、gpt-6.1-sol/high，每项START后最多20分钟，2项完整通过。所有物理进程与模型轮次均已结束，12段原速录像已完整解码检查。该探测试验不等于36项Oracle认证，也未把Oracle抓点/经验输入策略。见[结果与录像目录](outputs/gpt/frontier-v2-minimum-20m-20261025/README.md)。

- [当前 36 环境目录与说明](docs/frontier-v2-environments.md)
- [Transport R11修改范围与静态检查](docs/plans/suite-v2/transport-r11.md)
- [Transport R12目标间隔调整](docs/plans/suite-v2/transport-r12.md)
- [Transport R13白板缩短20 mm](docs/plans/suite-v2/transport-r13.md)
- [Transport R13最小单元、真实并行对照及后续计划](docs/plans/suite-v2/transport-r13-units.md)
- [R10／工厂 V6／离线回放与运行目录](docs/plans/suite-v2/implementation-r10.md)
- [冻结的 36 项输入清单](configs/suites/frontier-v2-r10.json)
- [当前 R10 pd1000-v1 Oracle 结果索引](runs/oracle/suite-v2-r10-closure-20261025/coverage-current.json)
- [成功与未通过Oracle录像](outputs/oracle/review-20261025/index.html)
- [本轮并行 Oracle 复盘与剩余缺口](runs/oracle/suite-v2-r10-servo2-20261025/review.md)
- [原 legacy 驱动工程结果](runs/oracle/suite-v2-r10-20261025/summary.json)
- [工厂当前预览](outputs/robo-factory/v6-cycles-final/interior_diagonal.png)

下列链接包含早期多臂试点与原单臂键盘练习；其通过记录不自动适用于当前 36 环境。

- [下一阶段计划与验收顺序](docs/next-iteration-plan.md)
- [trial04 效率诊断](runs/trial04-efficiency-audit-20261020/report.md) 与 [针对性修复](runs/efficiency-fixes-20261020/report.md)
- [目录、状态与上下文](docs/architecture.md)
- [Meta 启动与管理](docs/meta-runbook.md)
- [当前验收范围](docs/harness-v2-verification.md)
- [Coordinator / Worker Skill](.agents/skills/robot-collaboration/SKILL.md)
- [四臂 10×10 方块拼图环境](configs/environments/mosaic/four-arm-10x10/README.md)
- [实验三：八臂面对面传递长木条（环境预览）](configs/environments/beam_handoff/facing-08/README.md)
- [真机命令语义与抓取力审计](docs/robot-command-reality-audit-20261022.md)：共享多臂工具已增加 grasp 模式；原单臂键盘练习仍沿用旧位置夹爪。
- [3～12 臂方块＋圆柱场景与离线任务包（历史独立系列）](benchmarks/cube_transport_cylinders/README.md)
- [原方块＋球体版本与历史参考](benchmarks/cube_transport/README.md)
- [历史试点与诊断](docs/history/)

仅准备文件：`uv run python scripts/create_pipeline_run.py runs/NEW`。离线检查：`uv run python scripts/check_harness.py`。二者不会创建模型会话或操作机器人。物理启动是单独的显式步骤，见操作说明。

## 原单臂键盘练习

本机 MuJoCo 小 Demo：固定在桌面的 Panda 7 DoF、双指夹爪、三个自由方块。默认由键盘控制末端位移，机器人运动通过 IK 和上游位置执行器实现；抓取依靠真实接触与摩擦。

当前启动默认使用 `configs/noslip.json`（`noslip-10-v1`）：仅启用 10 次 NoSlip 接触后处理，保留原摩擦、夹爪执行器强度、质量和速度。原配置保存在 `config.json`，可用 `uv run robofrontier --config config.json` 复现原始设置。不同接触配置的结果需要分开记录；NoSlip 解决了本案例的慢滑，不保证任意抓取姿态成功。

新随机布局的实际 RGB 操作已抓起蓝色方块并保持超过 38 s 未掉落，原速视频与验证记录见 `docs/history/noslip-validation-20261019.md`。这与此前默认配置下的失败、固定坐标参考程序的成绩分别记录。

## 打开

本机环境和模型已准备好。双击 **`Launch Demo.command`**，或在本目录运行：

```sh
uv run robofrontier
```

点击仿真窗口，再使用英文键盘输入。macOS 功能键若被系统用于亮度/媒体控制，请使用 `Fn + F9` 等组合。

首次在其他机器安装：先安装 [uv](https://docs.astral.sh/uv/) 和 FFmpeg，然后：

```sh
uv sync --locked
uv run python scripts/fetch_robot.py
uv run robofrontier
```

此版本在 Apple Silicon macOS、Python 3.12.6、MuJoCo 3.16.0 上验证。独立虚拟环境位于 `.venv`，具体依赖由 `uv.lock` 固定。

## 键位

| 操作 | 按键 |
| --- | --- |
| 桌面内平移 | 方向键；↑ 是 -X（远离正面相机），↓ 是 +X，← / → 是 -Y / +Y |
| 抬高 / 降低末端 | W / S |
| 夹爪 yaw | A / D |
| 打开 / 关闭夹爪 | O / P |
| 慢 / 正常 / 快 | `[` / `]` |
| 切换末端 / 七关节模式 | Tab |
| 切换正面 / 侧面相机 | C；切换后方向键仍跟随世界坐标 |
| 恢复初始场景 | Backspace（Mac 键盘的 Delete） |
| 暂停 / 恢复整个物理世界 | Space，仅供调试 |
| 开始 / 停止录屏 | F9 |
| 保存截图 | F8 |
| 显示 / 隐藏帮助 | F1 |
| 保存并退出 | Esc |

默认末端平移速度 3 cm/s，慢档 1 cm/s，快档 6 cm/s。按住方向键持续运动，松开后受限减速并保持，物理世界仍继续运行。失焦会清空运动输入并保持，**不会冻结重力**。多轴同时输入按总平移速度限幅。

黄色小点是 TCP（夹指夹持区域中心）。建议先从正面调整左右和前后，再按 C 从侧面检查高度；降低夹爪，让两指分别位于方块两侧，按 P 闭合，然后 W 抬起。失败时 Backspace 从完全相同的初态重来。

七关节模式使用：

| 关节 | 正向 / 负向 |
| --- | --- |
| J1 | Q / A |
| J2 | W / S |
| J3 | E / D |
| J4 | R / F |
| J5 | T / G |
| J6 | Y / H |
| J7 | U / J |

支持多个关节同时输入，同一关节的正负键相互抵消。键盘硬件可能限制特定多键组合。各关节默认参考速度 15°/s，慢 / 快为 5 / 30°/s，参考加速度初值 60°/s²。实际反馈速度显示在右上角；模型采用位置伺服，不能把参考速度当作实际速度的硬保证。

## IK 与机器人导入

- **机器人不用重新建模**：`scripts/fetch_robot.py` 从 Google DeepMind MuJoCo Menagerie 获取 Panda 的 MJCF、67 个引用网格和许可证。当前固定 commit 为 `62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35`；文件 SHA-256 保存在 `vendor/panda/SOURCE.json`。
- `robofrontier/scene.py` 读取该 MJCF，在内存中设置固定底座位置，并加入桌子、方块、相机和 TCP site；不改写上游机器人文件。每个 episode 会保存完整组装后的 `scene.xml`。
- **MuJoCo 提供正运动学和末端雅可比等基础计算；这版的数值 IK 是我们写的小型阻尼最小二乘求解器**，位于 `robofrontier/sim.py`。它只使用机器人关节配置、运动学模型和期望 TCP 位姿；不读取运行中方块的真值坐标。
- 键盘 → 小步末端目标 → IK → 受速度/加速度/关节范围限制的关节参考 → 原模型位置执行器 → MuJoCo 接触物理。
- 没有吸附、抓取 weld 或运行时直接修改物体位置。IK 不会替操作者识别目标、对齐方块或选择闭合时机。

## 配置、日志和视频

`config.json` 集中保存桌面高度、方块尺寸与质量、摩擦、速度档、工作区域、初始 TCP 和相机等设置。默认 2 ms 物理步长、100 Hz 控制，初始化后先稳定运行 1 s，再保存初始状态。reset 同时恢复完整积分状态与控制器参考。

每次启动或 reset 创建一个 `runs/日期-时间/` 目录，包括：

- `metadata.json`：配置、模型 hash、版本、上游来源、初始关节参考。
- `scene.xml`：完整组装的场景。
- `initial.npz`：初始仿真状态及控制器状态。
- `trace.jsonl`：约 10 Hz 的动作输入、物理状态、机器人和物体状态，以及录像等事件。这是练习诊断日志，并非正式的逐动作研究数据协议。
- `recording-XX.mp4`：F9 录制，固定 1280×800、30 FPS，包含当前相机画面及状态提示。直接编码 MuJoCo 像素，不需要 macOS 屏幕录制权限。

视频按墙钟时间采样，仿真和墙钟时间分别写入日志。编码异步、队列有界，编码失败和丢帧会写日志。reset 结束旧录像，避免不同初态混在同一视频里。当前不含独立回放界面。

## 已验证与范围

运行物理验收：

```sh
uv run python scripts/verify_demo.py
```

验证当前默认配置和长时间保持：

```sh
uv run python scripts/verify_demo.py --config configs/noslip.json --hold-seconds 30
```

该配置已从同一初态连续 3 次抓起蓝色方块，抬高约 12 cm，各保持 30 s，保持期间位移约 0.05 mm。结果存于 `runs/verification-noslip.json`，属于参考程序的物理验收。

该参考程序使用已知坐标验证控制器和接触物理，**不是视觉模型或 GPT 的成绩**。验证结果写入 `runs/verification.json`：

- 空闲运行 30 s，物体无明显漂移或数值发散。
- 完整初态恢复；双关节同时运动、松键保持、模式切换。
- 连续向下输入后目标没有无限积累。
- 从同一初态连续 3 次双指抓取蓝色方块，抬高约 9.5 cm，持续接触保持约 4.6 s。
- 原生窗口、渲染截图和 H.264 MP4 编码已检查。

当前包含 Human 键盘练习版与本地 RGB + 本体状态工具入口，仿真持续运行。未加入独立 GPT API 客户端、MCP 注册、实验矩阵和统计评估。工具接管调用同一个 `Simulation.control()`，不更换机器人或另建物理世界。

## 供 Codex 操作的本地入口

```sh
uv run robofrontier --bridge --blind --record
```

`--blind` 在初始化时重新随机摆放方块，不把物体坐标通过入口返回。控制使用 RGB + 机器人本体状态；两台固定 RGB 相机与键盘 C 切换的相机相同。内部完整状态仅用于物理与诊断日志，策略操作时不得读取 `metadata.json`、`scene.xml`、`initial.npz` 或内部 `trace.jsonl` 来获取物体真值。该约束是实验协议，不是操作系统文件权限隔离。

另一个终端或 Codex 的命令工具可以调用：

```sh
uv run python -m robofrontier.client observe
uv run python -m robofrontier.client move --dy -0.002 --duration 0.5
uv run python -m robofrontier.client move --dx 0.06 --duration 1 --speed fast
uv run python -m robofrontier.client gripper close
uv run python -m robofrontier.client wait --duration 1
uv run python -m robofrontier.client record stop
```

每次 move 都可以按动作选择 `--speed slow|normal|fast`；速度档对应键盘的 TCP 速度和关节速度，控制器仍施加 60°/s² 的关节加速度限制。move 持续时间可设为 0.1–4 s，位移上限按 `TCP 速度 × 持续时间` 计算；最长单步约为 4、12、24 cm。夹爪与等待动作仍限于 2 s。yaw 每次最多 10°。例如高位长距离接近可选 fast，最后对准可选 slow。返回的是实际位置，不保证请求位移被精确执行。运动结束后留出 0.35 s 的减速/稳定时间，再返回正面与侧面截图。等待模型决策期间物理仍运行。此版本是有界速度请求，不是精确位置到达规划器。

仅绑定本机回环地址，令牌保存在忽略提交的 `.robot-bridge.json`；HTTP 线程排队，GUI 主线程执行物理和渲染。`--request-id` 可显式指定请求编号，同编号同参数重试返回缓存结果，不重复运动。Space 可中断正在运行的工具动作。没有 object-state、depth、自动 pick 或隐式重置入口。后续启动会把策略可见的请求与结果另存 `policy.jsonl`；内部真值 trace 与它分开。

首次 RGB 尝试的说明见 `docs/history/rgb-attempt-20261019.md`。参考控制器的固定坐标成功记录与该 RGB 尝试是两个不同结果。

关节模式允许探索姿态，只有基本限位与目标偏差限制，不提供全局避障。碰撞会真实发生；遇到难以恢复的姿态可 reset。此 Demo 的仿真设置不能直接当作真机控制配置。

## 一手资料

- [Menagerie Panda](https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_emika_panda)
- [MuJoCo Python API](https://mujoco.readthedocs.io/en/stable/python.html)
- [MuJoCo visualization](https://mujoco.readthedocs.io/en/stable/programming/visualization.html)
- 进一步说明见 `docs/mujoco-official-notes.md` 和 `docs/vla-action-spaces.md`。


八臂木条的正式任务入口为 [task.json](configs/environments/beam_handoff/facing-08/task.json)，复用现有运行服务；任务私有验收与通用同步提交分别位于 beam_task/action_batch。新增能力属于工程接入，不能把 Oracle 回放当作 GPT 成绩。详见[环境说明](configs/environments/beam_handoff/facing-08/README.md)。
