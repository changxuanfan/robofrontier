# MuJoCo / Panda 官方接口核对

核对日期：2026-10-19。本文是计划阶段的资料核对，未运行仿真、安装软件或验证抓取。`stable` 文档和 Menagerie `main` 会变化，实施时必须固定 MuJoCo 版本及 Menagerie commit，随后以实际编译的模型数组复核。下文将模型事实与工程建议分开。

## 1. Panda 模型事实

来源：[Menagerie Panda XML](https://github.com/google-deepmind/mujoco_menagerie/blob/main/franka_emika_panda/panda.xml)、[模型说明](https://github.com/google-deepmind/mujoco_menagerie/blob/main/franka_emika_panda/README.md)。

- 标准 `panda.xml` 是 7 个机械臂旋转关节、2 个夹指滑动关节和 8 个执行器；第 8 个执行器通过 tendon 控制双指，并由 equality 约束保持双指同步。
- 七个手臂 `general` 执行器通过 `gainprm`、`biasprm` 实现位置 PD 伺服，控制输入是关节位置参考，单位 rad；不是输入 rad/s 的速度伺服。
- 该 XML 没有“键盘默认速度”或通用关节硬速度上限。速度必须由应用层设定参考轨迹；参考轨迹限速不等于实际 `qvel` 必然不超速，接触力和跟踪误差仍会影响实际速度。
- 关节位置范围来自模型，必须由编译后的 `jnt_range` / `actuator_ctrlrange` 读取，而不是只硬编码在人机输入层：

| 关节 | 下限 rad | 上限 rad | 执行器力矩范围 N·m |
|---|---:|---:|---:|
| J1 | -2.8973 | 2.8973 | -87 … 87 |
| J2 | -1.7628 | 1.7628 | -87 … 87 |
| J3 | -2.8973 | 2.8973 | -87 … 87 |
| J4 | -3.0718 | -0.0698 | -87 … 87 |
| J5 | -2.8973 | 2.8973 | -12 … 12 |
| J6 | -0.0175 | 3.7525 | -12 … 12 |
| J7 | -2.8973 | 2.8973 | -12 … 12 |

以上是 Menagerie 模型的参数，不代表最新 Franka 真机规格或真机安全认证。

- 夹指单侧滑动范围为 0 … 0.04 m，最大双指开口约 0.08 m；夹爪控制范围是 0 … 255，0 对应关闭位置参考、255 对应打开位置参考。夹爪执行器 `forcerange="-100 100"` 是该 tendon actuator 的力限，不能直接描述成“每个指尖施加 100 N”。
- 模型 `home`：手臂 `q=[0,0,0,-1.57079,0,1.57079,-0.7853]` rad，双指位置各 0.04 m，夹爪 ctrl=255。可以作为初始化候选，桌面任务仍应验证可达性及碰撞后选择任务 home。
- 模型使用 `implicitfast`；未显式写 timestep。MuJoCo option 的默认 timestep 是 0.002 s。实施时显式写 `timestep="0.002"`，不依赖隐式默认。来源：[MJCF option](https://mujoco.readthedocs.io/en/stable/XMLreference.html#option)。
- Menagerie 的 `scene.xml` 只有地面、照明等基础场景，没有实验所需桌子和三个自由方块。应增加自己的场景 XML，保持机器人上游模型和许可证可追溯。标准 Panda 与 `mjx_panda.xml` 的夹爪、执行器、控制范围有差异，不应混用。

## 2. 原生 passive viewer 与键盘限制

来源：[MuJoCo Python viewer](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer)、[Python C++ 键盘封装](https://github.com/google-deepmind/mujoco/blob/main/python/mujoco/simulate.cc)、[viewer.py](https://github.com/google-deepmind/mujoco/blob/main/python/mujoco/viewer.py)。

事实：macOS 的 `launch_passive` 必须使用随 MuJoCo 安装的 `mjpython` 启动，原因是主线程渲染限制。应用负责推进物理；viewer 的 `sync()` 用于同步 GUI 和仿真状态，修改 viewer 可视化结构时要遵循 `lock()` 约定，不能任意跨线程写共享数据。

事实：`key_callback` 的公开签名只有一个 `int keycode`，C++ 封装仅在 `IsKeyDownEvent(act)` 为真时调用；Python 回调不包含 key-up、action 或 modifier 参数。仅靠此回调不能可靠维护“当前按住的多个键”，用超时猜测松开也不能做到严格一致。不能拿到 passive viewer 的私有 GLFW 窗口后覆盖回调。

工程建议：如只做离散按钮，passive viewer 加每键固定时长动作很快；本任务要求按住松开、同时多关节和失焦停止，直接用公开渲染接口自建一个小 GLFW 窗口更直接，避免依赖全局键盘监听或 OS 辅助功能权限。

## 3. 推荐窗口路径：MuJoCo mjv/mjr + GLFW

官方支持依据：[MuJoCo 可视化层](https://mujoco.readthedocs.io/en/stable/programming/visualization.html)、[MuJoCo rendering](https://mujoco.readthedocs.io/en/stable/programming/rendering.html)、[官方 basic.cc 示例](https://github.com/google-deepmind/mujoco/blob/main/sample/basic.cc)、[GLFW 输入指南](https://www.glfw.org/docs/latest/input_guide.html)、[GLFW 线程要求](https://www.glfw.org/docs/latest/intro_guide.html#thread_safety)。

事实：MuJoCo 提供与窗口库解耦的 `mjvScene`、`mjvCamera`、`mjv_updateScene` 以及经典 OpenGL `mjrContext`、`mjr_render`。官方 basic 示例使用 GLFW 创建窗口与 OpenGL 上下文，再推进 MuJoCo 物理并渲染。GLFW 的按键回调给出 key/scancode/action/mods，其中 action 包含 PRESS、RELEASE、REPEAT，另有窗口 focus 回调；`glfwGetKey` 也可查询已处理事件所记录的按键状态。GLFW 提醒 REPEAT 不应作为驱动动画的时钟。

工程建议：窗口创建、事件轮询、渲染都在进程主线程；用 PRESS/RELEASE 维护按键集合，以固定控制周期计算方向输入，忽略 OS REPEAT。失焦、切模式、暂停和退出时清空集合并撤销运动命令。固定物理步长与输入/显示频率分离。多键以同一个 7 维关节向量进入 executor，冲突正负键相互抵消。默认 Cartesian DLS IK 和关节 jog 共用轨迹限制与 executor；IK 可由 `mj_jacSite` 获取末端雅可比。来源：[mj_jacSite](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-jacsite)。

Cartesian 多轴输入需整体归一化，避免斜向比单轴快；关节模式则逐关节限制并声明是否使用全局缩放。两种模式都记录请求值和实际 `qvel`；接触/奇异位姿/跟踪失败返回实际结果，不宣称请求即达成。

## 4. 初始状态与恢复

官方接口：[mj_getState / mj_setState / mj_stateSize](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-getstate)、[状态位标志](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjtstate)。

工程建议：初始化时显式设置机器人、夹爪和所有自由物体，先 settle，再保存 `mjSTATE_INTEGRATION` 状态向量。恢复时校验模型 hash、版本和状态长度，用 `mj_setState` 恢复后 `mj_forward` 重建派生状态。另行保存控制器内部的参考位置/速度、夹爪参考、模式、速度档、相机参数和随机种子；只保存 qpos 不足以恢复完整操作起点。加入自由方块后重新生成完整模型的初始化，不盲目沿用机器人子模型的 keyframe 长度。

reset 不是缓慢回 home，而是恢复完整初始快照；home 动作则是运行中的有界轨迹，两者明确分开。跨 MuJoCo 版本/平台不承诺逐 bit 一致。

## 5. 录屏

事实：MuJoCo classic 渲染可用 `mjr_readPixels` 获取 framebuffer；Python 提供离屏 Renderer，OpenGL context 同一时刻只能在一个线程 current，其后渲染必须遵守该线程归属。来源：[Python OpenGL 上下文](https://mujoco.readthedocs.io/en/stable/python.html#classic-rendering)、[mjr_readPixels](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mjr-readpixels)。MuJoCo 输出的是像素，不自动生成完整 MP4 文件。

工程建议：最小版本从自建窗口的已渲染 framebuffer 读 RGB，修正上下方向，再交 FFmpeg 编码 MP4；这是场景窗口录制，不依赖 macOS 系统屏幕录制权限。录制期间固定分辨率，处理 Retina framebuffer 与窗口逻辑尺寸差异，编码队列有界，不能让编码器卡住物理循环。每帧记录仿真时间与帧编号，明确 MP4 采用仿真时间还是墙钟时间；另存 JSONL/状态，不以视频代替 trace。后续若需要固定相机录制可转为独立离屏相机流，必须先验证 macOS 上的上下文与性能。

## 6. 尚未验证的内容

上述资料支持 API 选择，不保证桌面场景可抓。方块尺寸/质量/摩擦、夹指接触、home 位姿、IK 增益和轨迹速度都需落地后实测。推荐先用真实接触验证开夹—接近—闭夹—抬起，不使用焊接、吸附或 teleport 作为抓取实现。第一阶段接受单任务单场景，把调好的参数冻结后再做 Human/Agent 比较。
