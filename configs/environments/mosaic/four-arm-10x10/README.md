# 四臂方块拼图环境

四台 Franka Panda 7 DoF 固定在 1.6×1.6 m 实体木色桌面的四边，A 南、B 东、C 北、D 西，均朝内。桌高 0.4 m，中央 10×10 网格占地 40×40 cm，格距与方块边长均为 4 cm；网格和行列编号仅为视觉印刷，没有碰撞分隔、吸附或自动对齐。

100 个 50 g 自由方块，按目标草案颜色计数：A 旁蓝色 40、B 旁红色 30、C 旁黄色 14、D 旁白色 16。整齐单层供料，方块间保留间距。供料位置不限制机器人只能操作某种颜色。

浅蓝背景、暖木色桌面。共 12 个固定相机：global、global_side、top、board_top，加 A～D 各 front / side。board_top 是网格近景；top 上北右东，行号由北往南、列号由西往东。initial-* 是初始化截图，不是实时观测。

## 打开和复位

在仓库根目录运行：

```sh
.venv/bin/mjpython -m scripts.view_mosaic
```

F6 依次切换 global / top / board_top / global_side；F7 恢复 initial keyframe。Mac 如需可用 Fn+F6 / Fn+F7。也可启动时指定 `--camera board_top`。鼠标观察操作使用 MuJoCo 原生功能。

物理持续运行，机器人保持初始执行器目标。上述命令是环境查看器，不接入动作服务。Agent 服务现可通过下述单独准备／启动步骤加载同一场景；尚未接入四臂键盘运动，也没有自动摆放策略。场景已保存 initial keyframe，原始 qpos0 不是可直接使用的初始化姿态；加载后需应用 keyframe。

## 目标草案与验证边界

assets/reference-original.png 保存用户参考图。task-draft.json 与 target-pattern-draft.png 是规整的四色 10×10 草案：原图第 7 行有半格错位，这版把中间蓝块放在第 6 列；深浅蓝视作同一颜色。运行评测前需确认目标矩阵。矩阵暂存在 robofrontier/mosaic.py 的 PATTERN，修改后重新生成；重新生成会覆盖本目录派生文件。

validation.json 记录初始化穿透、一秒静置和机器人点位 IK 检查。当前无超过 1 mm 的初始化穿透，方块静置最大位移约 0.073 mm；各臂自己的供料中心点通过 IK，四臂可达目标格的并集覆盖全部 100 格，每臂固定朝下姿态点位覆盖 86 格。

这只是几何端点检查，没有证明抓取、接近／撤离、夹指避障、密集零间隙插入、连续轨迹或多臂并行安全。动作服务的私有评分检查共享目标、2 mm XY 中心误差、3 mm 高度误差、约 2° 轴对齐、释放和桌面支撑，并要求连续一秒稳定；目标仍是草案，没有验证完整密集摆放可行性。

重力、0.002 s 时间步、NoSlip 10、摩擦和 Panda 位置执行器沿用现有环境，不改夹持强度或加入吸附。scene.xml 的网格素材通过相对路径引用 vendor/panda/assets，复制整个仓库才保留可加载性。

生成命令：

```sh
.venv/bin/python -m scripts.build_mosaic --reference /path/to/reference.png
```

## 接入 Agent 动作服务

生成器同时更新 configs/tasks/mosaic-four-arm-v1.json，使用共享目标，不按机器人分配目标格。以新目录准备一次运行：

```sh
.venv/bin/python -m scripts.create_pipeline_run runs/NEW --task configs/tasks/mosaic-four-arm-v1.json
.venv/bin/python -m robofrontier.pipeline --run-dir runs/NEW
```

第一条仅冻结输入、场景、网格贴图及机器人网格，创建角色连接；第二条才启动共享物理世界和本地认证服务。角色 READY／会话绑定／START 仍沿用现有门控，没有 START 就不能移动机械臂。一个运行只由一个物理进程拥有；其他角色通过各自连接控制同一份世界，不各自启动模拟器。

支持位置＋世界坐标四元数姿态、关节目标模式、连续夹爪开度与速度、原动作 ID 上的受控停止。关节速度／加速度、位置跟踪和原执行器力限仍受控制器约束。用机器人模型做重力补偿；不读取物体状态来选动作。夹爪宽度控制并非恒定夹持力控制。完整参数以 READY 和冻结 Skill 的工具契约为准。

运行 RGB 为 1280×800，READY 返回对应相机配置；本目录静态预览原图为 1280×900，带边框预览另有视口偏移，见 initial-top.json。两者不是同一套像素坐标。

原会话调度和通信拓扑保持不变；更新了机器人动作适配器、运行准备、共享目标评估及通用 Prompt / Skill。没有启动新的模型试验，也没有写入摆图顺序或自动抓放解法。
