# Cube Transport：方块＋圆柱 v4

当前生成器输出此系列。3～6 臂仍为六色方块；7～12 臂将原球体换成同色圆柱。A～F 初始各有三件红、蓝、绿、橙、紫、青方块，G～L 初始各有三件同序颜色圆柱。每个工位最终三个目标，颜色＋形状相同的实物可互换。完整订单见 [tasks.md](tasks.md)。

圆柱使用 MuJoCo 原生 cylinder，同时用于可视和碰撞，直径 4 cm、高 4 cm、质量 50 g，竖直放置。坐标原点和重心均在几何中心。布局、订单分配、供需距离比例、初始坐标、槽位和相机保留原设置，仅替换形状要求。十个规模各三个订单种子，种子只改变订单。

浅蓝背景、暖木色桌面；每臂 front / side 加 global / global_side / top，共 2N+3 台相机。startup.md / startup.json 描述编号、邻接、初始化物料与相机，不包含任务解法。预览使用固定基座位置标注 A～L，文字位于渲染视口之外。

## 物理与验证

桌高 0.4 m，时间步长 0.002 s，重力 9.81 m/s²；摩擦 `[1, 0.005, 0.0001]`、condim 4、NoSlip 10、Panda 执行器和夹爪强度保持原配置。没有吸附、物体固定、隐藏托槽或新增滚动阻力。圆柱若倒下仍可能滚动，平底不会保证所有抓放都成功。

全部规模通过加载、桌面支撑、间距、初始穿透、点位 IK / 终点接触与一秒静置检查，静置最大位移约 0.075 mm。这些不是完整运动轨迹或多臂避碰证明。

本地已知位姿参考抓放检查使用程序＋机器人 IK，无 LLM：三臂 A 方块、七臂 G 红圆柱、十二臂 L 青圆柱。三个检查均通过；七臂和十二臂圆柱最终 XY 误差分别约 3.08 mm、0.14 mm，保持双指接触提起、释放后直立稳定。结果见 validation/object-fixtures.json；录制文件在同目录。这只验证对应工位的一次本地抓放，不证明全任务运输、多臂并行或 RGB 模型表现。

任务完成判定沿用原规则：颜色＋形状正确、每槽不同实物、XY 误差 ≤2 cm、Z 误差 ≤1 cm、线速度 ≤1 cm/s、有桌面支撑且脱离机器人，并连续稳定一秒；采样间隔 ≤25 ms。任务没有增加姿态评分；本地抓放检查另外记录朝向，并要求圆柱最终倾角 <15°。

离线 references 仍是容量／依赖抽象中的可行调度上界与下界，不是连续物理时间最优解，阶段数不等于工具调用或仿真秒数。该形状替换未修改通信 Harness 或 Agent 规划 Skill，也没有启动新的模型实验。新场景尚未接入原 pipeline 的实时动作服务。

## 文件与复现

```text
configs/environments/cube_transport_cylinders/<环境>/
  environment.json / scene.xml / assets/
  startup.md / startup.json
configs/tasks/cube_transport_cylinders/<环境>/seed-0.json…seed-2.json
benchmarks/cube_transport_cylinders/
  README.md / tasks.md / manifest.json
  previews/       初始化图、相机与编号元数据
  validation/     点位、静置、本地抓放结果和视频
  references/     离线调度、下界和需求距离统计
```

```sh
.venv/bin/python -m scripts.generate_cube_transport
.venv/bin/python -m scripts.verify_transport_objects --record
.venv/bin/python -m unittest tests.test_cube_transport -v
.venv/bin/mjpython -m scripts.view_cube_transport --count 7
```

查看器应用保存的 initial keyframe，仅展示和运行物理，不提供键盘控制或自动录屏。生成器部分生成会覆盖该系列汇总清单，完整发布请生成全部规模。相机原图为 1280×900，带边框 1536×1152；标定与视口偏移见 startup.json。

原球体配置和视频仍在 cube_transport；圆锥配置和失败记录仍在 cube_transport_cones。圆锥版生成源码归档在 benchmarks/cube_transport_cones/history/v3-generator-source.tar.gz，球体版源码归档在 benchmarks/cube_transport/history/v2-generator-source.tar.gz。旧视频与回放脚本继续使用旧系列，不覆盖历史结果。

## 6～12 臂完整 Oracle 程序参考

本次 seed 0 的七个规模均取得整场通过记录，分别为 18/18 至 36/36；所有圆柱最终接近竖直。6 臂使用原运动时序，7～12 臂在默认时序出现失持／抓取失败后，改用更缓的运动时序通过。质量、摩擦、执行器增益和力限未改变，失败尝试保留。这不是统一速度下的规模性能比较，也不是模型成绩。

结果与视频：[本次参考运行](../../runs/program-transport-cylinders-v4-20261021/report.md)。录像为 12 倍速真实轨迹回放，逐份核验编码和场景／轨迹来源。尚未重跑其他订单种子。
