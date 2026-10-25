# VLA 的动作空间与 RGB / Oracle：一手资料核对

日期：2026-10-19。只调查与解释，未实施仿真。下述论文版本和官方仓库说明对应特定实现，不代表所有 VLA。

## 已核实的实例

| 系统 | 模型输入 | 动作表示 | 一手证据 |
| --- | --- | --- | --- |
| RT-2 | 相机图像、文字任务 | 末端 3D 平移增量、3D 旋转增量、夹爪、终止标志 | [原始论文 §3.2](https://arxiv.org/html/2307.15818v1#S3.SS2) |
| 原始 OpenVLA | 图像、语言指令 | 7 维机器人动作；数据筛选为单臂末端控制，不是七个关节 | [论文 Fig.2、§3.2–3.3](https://arxiv.org/html/2406.09246v3) |
| π0 论文 | 多路 RGB、语言、关节角本体状态 | 作者内部数据使用关节位置；混合训练也含不同动作表示 | [论文 §IV](https://www.physicalintelligence.company/download/pi0.pdf)、[作者 Kevin Black 在官方仓库的解释](https://github.com/Physical-Intelligence/openpi/discussions/302) |
| 后续 openpi 相关模型 | 随具体模型和数据适配器定义 | 作者说明支持关节、世界坐标末端、末端相对动作；不能一概视为关节输出 | [官方作者讨论 #302](https://github.com/Physical-Intelligence/openpi/discussions/302) |

OpenVLA 的 7 维在常见末端接口中对应平移 3、旋转 3、夹爪 1；RT-2 另外还有终止标志。机器人本身是 7 DoF，与动作向量恰好有 7 维是不同概念。原始 OpenVLA 的图像输入结论不能自动套用后续 OpenVLA-OFT 等扩展。

π0 论文明确观察包含关节角。官方 ALOHA 适配代码又具体列出 state 的内容为左右臂角度和夹爪，并把它与图像一起送入模型：[aloha_policy.py](https://github.com/Physical-Intelligence/openpi/blob/main/src/openpi/policies/aloha_policy.py)。因此，视觉机器人不等于模型和底层都只能读取相机。

结论：这些一手实现并未采用唯一动作空间。末端命令与关节命令都是正常设计；要报告具体表征、坐标系、单位、频率、归一化和执行器，而不能把“VLA 标准”理解为必需输出每个电机。

## IK 与世界物体真值是独立的问题

LeRobot 官方文档直接介绍关节和末端两种表示，并提供基于机器人 URDF 的 `RobotKinematics`、`ForwardKinematicsJointsToEE`、`InverseKinematicsEEToJoints`：[Action Representations](https://github.com/huggingface/lerobot/blob/main/docs/source/action_representations.mdx)。它给出的运动学转换依据是机器人模型及关节状态，不是方块的模拟真值。

以下是据此作出的系统设计解释：

1. **感知 / 决策**回答：方块在哪里、应选择哪个、末端下一步向哪里走？视觉模式由图像决定；Oracle 可以获得精确物体坐标。
2. **IK**回答：给定期望末端位姿及机器人几何关系，关节应采用什么配置？数值求解通常利用当前关节配置做初值，选择连续、可达的解。
3. **低层控制**根据实际关节反馈跟踪参考。模型不接收关节状态，并不意味着底层控制器也不能读关节编码器。

所以 RGB 与 Oracle 都能使用相同 IK：它们的区别在目标/动作由什么信息决定，不在是否存在运动学求解。IK 不会自动识别蓝色方块，也不会把某个错误估计的目标修正成正确方块坐标。

真机的直接证据：Franka 官方 `Robot` API 同时支持 CartesianPose、CartesianVelocities、JointPositions、JointVelocities；`RobotState.q` 是机器人测得的关节位置，单位 rad。[Robot API](https://frankarobotics.github.io/libfranka/latest/classfranka_1_1Robot.html)、[RobotState API](https://frankarobotics.github.io/libfranka/latest/structfranka_1_1RobotState.html)。官方示例明确把 Cartesian pose goals 送入接口以求解 IK：[joint_impedance_control.cpp](https://github.com/frankarobotics/libfranka/blob/main/examples/joint_impedance_control.cpp)。这些接口不要求传入被抓方块的真实位置。真机使用已知机器人结构和自身测量，不需要知道整个世界的状态。

Franka FCI 的控制结构与可读机器人状态另见[官方 overview](https://frankarobotics.github.io/docs/doc/libfranka/docs/overview.html)。

“看图后向下移动 2 mm”可以不显式重建整个世界；相反，“从一张 RGB 得出方块精确三维坐标”并非 IK 的能力，需要视觉估计、标定、深度、多视角或任务先验等额外条件。单目视觉闭环不代表任意场景都能精确恢复物体三维状态。

## 不能把末端控制全部称作 IK

robosuite 官方区别了 WholeBodyIK 与 OSC_POSE：前者把末端目标变为关节角目标；后者根据末端误差和机器人模型计算关节力矩。[Controller 文档](https://robosuite.ai/docs/modules/controllers.html)。因此，证明某 VLA 输出末端动作，不等于证明它底层必定采用某个 IK 实现。

## 对本 Demo 的建议（设计判断）

- RGB 条件可精确定义为 **RGB + 相同的机器人本体状态**；Oracle 条件为前者再加物体位置、姿态和属性。
- 二者共用末端增量与夹爪动作接口、相同 IK 和轨迹限制。不要一个采用关节控制、另一个采用末端控制，否则改变了两个因素。
- RGB 路径的 IK / controller 不读取方块 body pose、目标 site 或仿真深度；只读机器人模型和本体关节状态。环境求解器本身当然仍拥有完整物理状态，这不等于给操作者泄露真值。
- 机器人本体状态必须按机器人关节 / site 名称明确筛选，不能把包含自由方块位姿的整个 `qpos` 数组传入 RGB 策略或 IK。完整状态只留在物理引擎与 evaluator / logger。
- `move(dx,dy,dz)` 与 `grasp(object_id)` 属于不同抽象层：后者若内部查询物体真值并规划抓取，会绕过视觉决策，不能被包装成 RGB 条件下的普通动作。
- 若研究“没有本体反馈的纯像素策略”，应另外声明 policy 是否能看关节状态，以及 controller 是否能读取编码器；这和 RGB 对比 object-state Oracle 是不同实验问题。

## 待实现核验

仍需实测 Panda 的 IK 连续性、限位、真实接触、动作尺度及视觉可操作性。查到真实机器人论文使用末端动作，并不意味着任意通用 GPT 在未训练情况下能可靠抓取；可行的控制接口与模型执行能力需要分开验证。
