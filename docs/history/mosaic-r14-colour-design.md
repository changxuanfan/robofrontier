# 历史：Mosaic R14 供料颜色设计

本页保存颜色设计与检查方法。当前规格见[任务说明](../tasks/mosaic.md)，
当前命令和共享待办见[合并交接](../plans/suite-v2/mosaic-merge-handoff.md)。

R14用于接下来的GPT任务；历史Oracle视频及未来工厂Oracle回放仍使用原R13。
本次只改变初始方块的颜色和对应RGB，九个任务的方块位置、尺寸、机器人基座、
初始关节、桌面、目标图案、口袋/印刷线、相机与物理参数均不变。各颜色的总数量
仍与目标图案完全一致。同尺度L1/L2/L3采用相同供料配色。

用户最新要求是让附近供料更贴合附近拼图，约60%–80%，不是80%的硬门槛。
设计取70%，按整数方块向上取整；每臂实际有70.0%–72.7%的最近供料具有不同
同色目标格的同臂取放采样几何证据。九项合计1119/1566个配对，71.46%。

| 规模（各适用L1/L2/L3） | 每臂已确认本地配对 / 最近供料 |
|---|---|
| 4臂 | A13/18，B13/18，C13/18，D13/18 |
| 6臂 | A14/20，B23/32，C21/29，D15/21，E23/32，F20/28 |
| 8臂 | A23/32，B21/29，C30/42，D29/41，E24/33，F21/30，G28/40，H29/41 |

这里“最近供料”按世界XY中与基座的距离划分，等距按机器人ID确定，与历史source
标签无关，也不建立所有权。“本地配对”必须是同一个机器人具有原R13对应供料的
采样抓取/抬升证据，以及对应目标的采样接近、开爪和撤离证据。不同供料匹配不同
目标格，不把同一格重复计入多个方块的可用数量。

严格按最近基座划目标区时，8臂两端某些机器人只有24个格、附近却有41个方块。
因此允许同侧相邻、相对最近基座距离额外不超过240mm、机器人到格中心XY距离
不超过860mm的候选格；这些距离只筛选候选，**不能代替真实几何证据**。本次
选中的额外格逐一运行已有Coverage采样检查；L2/L3仅在编译后的完整物理/初态
身份一致时复用证据。工程scratch没有推进动力学或修改现场。

分配使用最小费用一一匹配，优先最近的目标区，跨区部分优先邻近机械臂区域；
然后按运输距离和少量保留原颜色的偏好分配到具体供料位置。不是全局时间最优
规划，也没有证明同时运动时无碰撞。连接取放两端的完整载荷路径、放满后的
抓取/撤退、实际抓稳/落稳及并行效率仍需要GPT运行观察。

剩余约三成是设计中留给跨区协作的空间，**不是规定必须交接的次数**。有些同色
块可与其他目标重配，GPT也可以直接跨区抓取；实际交接可更少。工程配对、关节
见证和比例不进入GPT公开包，policy自行选择分工、次序与路径。

## 版本入口

- `configs/task_families/mosaic/selection.json`明确区分用途。
- `configs/task_families/mosaic/r14.json`绑定新版九项资产和私有设计证据，用于未来GPT。
- `configs/suites/frontier-mosaic-r14.json`列出完整九项环境目录。
- `prompts/tasks/mosaic-r14.md`为新版公开说明，沿用几何/操作通用的mosaic-r13 skill。
- `configs/task_families/mosaic/r13.json`继续绑定原Oracle和工厂回放，不能把旧动作
  套到R14颜色上冒充新版Oracle成功。

selection是家族声明，当前共享运行器不会自动读取它。公共门禁、加载与封存仍按
合并交接中列明的公共接线在合并阶段实施；不得退回通用默认R10，也不得因
几何等价就写入R14 oracle_verified=true。新版本未启动Oracle或GPT。

## 生成和验证

```sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m scripts.build_mosaic_local_colors \
  --arms 4 --engineering runs/engineering/mosaic-local-colours-r14-20261027
# 分别用 --arms 6、--arms 8 生成其余尺度的工程设计。
python -m scripts.build_mosaic_local_colors \
  --engineering runs/engineering/mosaic-local-colours-r14-20261027 \
  --output outputs/policy-packages/mosaic-r14-local-colours-20261027
python -m scripts.check_mosaic_local_colors \
  --engineering runs/engineering/mosaic-local-colours-r14-20261027 \
  --package outputs/policy-packages/mosaic-r14-local-colours-20261027
```

构建拒绝覆盖现有R14目录或输出包。复查读取现有输入，不生成动作。验证包括：
去掉供料RGBA后完整XML相同；完整编译物理/初态相同；颜色数量守恒；每个本地
配对对应源证据；新公开任务不含私有配对；九份配色图和18张初态截图；原R13
资产、任务包、视频、时间表等496个历史文件SHA不变。29项相关测试通过。

## 用户可看产物

主目录：`outputs/policy-packages/mosaic-r14-local-colours-20261027/`。
`说明.md`解释两个版本的用途，`previews/index.html`包含九项完整图集。
`previews/all-nine-colour-layouts.jpg`是九项总图；中间彩格是目标参考示意，
不是初态已经完成。各case的`top.png`与`global.png`是真实MuJoCo初态截图。
`previews/before-after-colours.jpg`对比三个尺度的新旧供料颜色。
