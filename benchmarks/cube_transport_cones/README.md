# Cube Transport：方块＋圆锥 v3

此版本把原 v2 球体换成完整尖顶圆锥。原场景、订单和视频保留在 `cube_transport` 目录；当前生成器输出到 `cube_transport_cones`，不会覆盖原版本。v2 生成源码另存于 `benchmarks/cube_transport/history/v2-generator-source.tar.gz`。

## 场景与任务

3～6 臂仍只使用六色实心方块。7～12 臂复用这六色：A～F 初始各有三件红、蓝、绿、橙、紫、青方块；G～L 初始各有三件同色圆锥。圆锥尖端朝上、平底向下，底径 4 cm、高 4 cm、质量 50 g。视觉与碰撞共用封闭凸网格，底部采用 64 边形近似圆。惯性由网格计算；物体坐标原点是包围盒中心，重心位于它下方 1 cm。

布局、物料供需、初始位置、目标位置、相机与全场运输距离分布保持 v2 配置；订单只把 sphere 要求换成 cone。颜色＋形状相同的物品可以互换。每臂最终三个目标，槽位不规定执行顺序。完整清单见 [tasks.md](tasks.md)。十个规模各三个订单种子，仅订单变化，初始几何不变。

背景为浅蓝、桌面为暖木色。每臂 front / side，加 global / global_side / top，共 2N+3 台固定相机。基座标签及预览边框 A～L 标识机器人；静态图片不能替代执行中的观测。各场景 startup.md / startup.json 包含初始化说明，不含运输解法。初始坐标允许公开；运行策略只应取得 RGB、机器人本体状态与动作状态。

## 物理与验证范围

桌高 0.4 m；步长 0.002 s，重力 9.81 m/s²，原摩擦 `[1, 0.005, 0.0001]`、condim 4、NoSlip 10 和 Panda 夹爪执行器保持不变。没有新增滚动阻力、吸附、隐藏支撑或物体固定。

十个规模通过加载、桌面支撑、初始穿透、点位 IK / 终点接触以及一秒静置检查；圆锥静置最大位移约 0.075 mm。这不证明动态抓取、交接或多臂避碰成功。

**当前参考抓法未通过圆锥抓取。** 已知位姿的三臂 A 方块抓放通过；七臂 G、十二臂 L 圆锥未被稳定提起。改变夹持高度及较慢抬升的探测也有失败，结果和视频保存在 validation。原质量、摩擦和夹持强度未因此调整。形状替换完成不等于完整运输任务可可靠完成；后续需要验证合适抓姿或另选物体。

参考夹持检查额外要求圆锥放下后倾角小于 15°。任务计分仍沿用 v2 的位置、线速度、支撑、释放及一秒稳定窗口要求，**没有新增任务姿态评分**。失败抓取也不会被视为通过。

本地夹持检查使用已知位姿程序和 IK，没有 LLM，不是纯视觉模型成绩。没有启动新的模型试验或修改通信 Harness / 规划 Skill。离线调度只检查容量和依赖，不含真实运动时间；其阶段数和 v2 一致，不能据此推断物理运行加速。

## 目录与复现

```text
configs/environments/cube_transport_cones/<环境>/
  environment.json / scene.xml / assets/
  startup.json / startup.md
configs/tasks/cube_transport_cones/<环境>/seed-0.json…seed-2.json
benchmarks/cube_transport_cones/
  README.md / tasks.md / manifest.json
  previews/     初始化图与编号、相机元数据
  references/   离线调度参考、下界与距离统计
  validation/   几何、点位、静置及本地抓放结果和视频
```

```sh
.venv/bin/python -m scripts.generate_cube_transport
.venv/bin/python -m scripts.verify_transport_objects --record
.venv/bin/python -m unittest tests.test_cube_transport -v
.venv/bin/mjpython -m scripts.view_cube_transport --count 7
```

夹持检查遇到失败会退出为非零，这是当前预期结果。查看器加载 initial keyframe 并运行真实物理；尚未绑定旧 pipeline 的动作服务，不提供键盘控制或录屏。生成器部分生成会覆盖该新系列的汇总清单，完整发布应生成全部规模。旧系列的程序回放脚本仍指向旧目录，避免改写历史结果。
