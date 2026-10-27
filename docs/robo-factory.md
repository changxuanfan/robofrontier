# 工厂展示与渲染

V10使用居中区域标签的场景，最新静态图索引为
`outputs/robo-factory/v10-centered-labels-review/README.md`。区域标签固定在各矩形上边中间，
通过移动供给车避让；工位标签放在桌面投影外。固定灯光/相机配置为
`configs/backgrounds/robo_factory_lookdev.json`与`robo_factory_lookdev_camera.json`。
大面积柔和补光替代离散光池；低角度暖日光从真实窗洞进入，室外道路已关闭。
机械臂深色关节与外壳、木制任务件和桌面印刷分别校准展示材质，不改写源状态。

视频现已恢复：16秒、30fps、480帧、1080p、Cycles Metal GPU、完整360°环绕。
同一组10帧的加速比较见`runs/engineering/factory-orbit360-20261027/benchmark-review.md`；
正式视频输出为`outputs/robo-factory/v10-orbit16-cycles/`。
以下V9成片与耗时是历史验收，不能用作V10最终画质或速度结论。

当前为V10、55×50米、四区36桌。配置`configs/backgrounds/robo_factory.json`；
布局输入`configs/suites/frontier-current-oracle.json`；场景
`outputs/robo-factory/v10-centered-labels/scene.xml`，由`outputs/robo-factory/current.json`指向。
离线展示没有物理自由度或执行器。[V7验收记录](history/robo-factory-v7-20261027.md)
及[V6早期说明](history/robo-factory-before-integration-20261027.md)保留。

## 源、剪辑和独立循环

`configs/task_families/current.json`绑定每桌自己的冻结场景、录制和剪辑。
Transport 20x、Mosaic 30x、Relay 20x、Dial 10x，输出30fps，原记录60Hz。
每桌独立跳过自己的无用区间，完成后回到原始起点继续循环；循环时恢复初始RGBA，
再累积该周期的外观事件。每帧更新所有工位后只提交一次工厂FK、渲染一次。
旧交接中的“全部完成后停止”由2026-10-27最新要求覆盖。

新的`robofrontier.station-edit.v2`在
`outputs/robo-factory/source-edits-v3-20261027/`。它在历史家族剪辑之上，以2/8/32秒
窗口复核静止、微抖及反复被动振荡，保护定向慢移动、连续旋转、动作和结果/违规/
失败/恢复/外观事件附近的源上下文。新表绑定selection、源stream和语义日志的哈希。
原始流、原剪辑、原视频不改写。运动分类为10Hz审查、源选择仍用原60Hz索引；
不声称测出了接触力，也不把展示删帧作为物理通过证据。

工位采用整体平移/水平旋转，不缩放机械臂或载荷，不执行动作、IK或物理积分。
Transport原Oracle为R13，桌面开孔显示比例0.5；GPT的R14本身已是半孔，比例1。
桌面显示编辑与源物理几何分开记录，避免重复减半。

## 外观和镜头

保留四墙、屋顶、结构梁、窗门、区域文字、原桌面和相机能力；去掉北墙ROBO FACTORY字。
蓝灰Transport、灰绿Mosaic、陶土Relay、赭黄Dial使用齐平无重叠地坪，柜体与边线同色系，
共用灰色十字通道和安全黄线。木台面、白色机械臂、灰色结构件统一，顶棚采用涂装金属。
桌下增加柜体标签、状态灯和服务线圈，原有电源/线槽保留；间隙新增6个滚动检修/控制架，
角落增加8组悬挂工具供给。检修架避开规划工位且留出至少0.4m余量；不是全动作扫掠体积认证。
共享网格、重复材质、贴图和静态批处理保留；构建使用输出内资产，不改写任务原资产。

窗外日间厂区图由内置imagegen生成，项目文件为
`assets/robo_factory/textures/exterior-daylight-v1.png`，同目录`.prompt.txt`记录提示词。
它只作为窗外景物；工厂、机械臂及视频本身均由Blender三维渲染。
两张工厂美术参考与提示词保存在`outputs/robo-factory/v8-study/references/`。

V10保留42组灯具模型；照明采用宽幅顶棚漫反射补光，避免离散灯源在地坪产生一圈圈亮斑。
窗侧面积光、低角度太阳及环境光分工，顶棚补光不贡献镜面高光。
Cycles玻璃保留相机方向的反射/透射，阴影射线使用薄玻璃透光近似，允许日光进入真实窗洞。
Eevee采用混合透明薄玻璃，fast模式4盏主灯计算阴影，其余灯补光；两引擎效果需要实测比较。
台面/硬表面使用平面法线，机械臂保留平滑表面；避免弧形桌面错误平滑造成波浪条带。

`camera-shots.json`先总览6秒、角落小幅环视6秒，再按Transport→Mosaic→Relay→Dial，
每类L1→L2→L3拍各规模工位。`orbit`默认在厂内缓慢环绕60°，可用`--orbit-degrees 360`选择整圈，导出固定30fps。
镜头时钟与各桌源时钟独立；可选固定镜头或自行剪辑。
这是可用的拍摄基础，尚未制作完整讲解、字幕与成片。

## 构建、观看与导出

从仓库根执行，生成目录必须是新目录：

```sh
.venv/bin/python -m scripts.factory build --output outputs/robo-factory/my-factory
.venv/bin/python -m scripts.factory video --renderer cycles --camera orbit --duration 8 --output outputs/robo-factory/my-cycles
.venv/bin/python -m scripts.factory video --renderer eevee --camera orbit --duration 8 --output outputs/robo-factory/my-eevee
```

`--scene`选择新场景，`--families`选择家族，`--camera`可选tour、orbit、desk-front或相机预设。
默认Cycles/Metal GPU、fast、64 samples、Noise Threshold 0.10、Min Samples 4，
Adaptive Sampling与GPU降噪开启；Eevee默认6 samples与GPU降噪。
`--samples`、`--noise-threshold`、`--min-samples`可单独覆盖。
两种引擎均启用GPU降噪，CPU用于源FK、文件与场景更新。`--save-blend`可保存可检查场景。
需安装Blender和ffmpeg；macOS自动发现Blender，也可传`--blender`指定程序。
输出包含asset、poses、逐帧源映射、PNG、MP4、日志与report；完整解码并校验视频帧数/30fps。
动画PNG默认为临时文件：MP4完整解码与帧数校验通过后，保留`stills/`内首、中、末三张，
清理其余PNG；失败时不清理。`--keep-frames`显式保留全序列，`--no-encode`抽样比较也保留全序列。
保留PNG到编码完成可以避免JPEG中间帧带来的额外有损压缩，长久保存以MP4与源姿态为主。
清理数量、回收字节数及仍保留的帧路径写入`report.json`的`frame_storage`与`views[].retained_file`。
30fps是影片播放帧率，生成每帧耗时另行统计。

逐桌固定front使用冻结任务的`global`整桌正面镜头，按工位平移/旋转放入工厂，
保持源视场角，固定相机，保留周围工厂及其他工位：

```sh
.venv/bin/python -m scripts.factory video --camera desk-front --station transport-a04-d1 --duration 1 --width 1920 --height 1080 --output outputs/robo-factory/my-desk-front
```

`poses`也支持相同的`--camera desk-front --station CASE_ID`组合。
2026-10-28的Transport A04-D1测试为连续10帧，默认64/0.10/4，1080p，
预热后9帧平均3.444秒/帧（0.290帧/秒），首帧含准备10.337秒，P95 3.647秒。
该测试包含PNG写盘，编码另计；视频验证后仅保留三张无损检查图。
36个源`global`镜头均确认存在，本轮实际画面/FK验证覆盖A04-D1，尚未逐桌检查遮挡。
图像、视频与验证回执由`current.json.presentation.desk_front_preview`索引；
记录在`runs/engineering/factory-desk-front-10frames-20261028/`。

要复用同一批姿态进行两个引擎的严格对比，可分步导出：

```sh
.venv/bin/python -m scripts.export_factory_blender --scene outputs/robo-factory/v9-workstations/scene.xml --output outputs/robo-factory/my-blender
.venv/bin/python -m scripts.factory poses --duration 8 --camera orbit --output outputs/robo-factory/my-poses
/Applications/Blender.app/Contents/MacOS/Blender --background --python-exit-code 1 --python scripts/render_factory_blender.py -- --asset outputs/robo-factory/my-blender --animation outputs/robo-factory/my-poses/animation.json --output outputs/robo-factory/my-cycles --engine cycles --device METAL --profile fast --samples 24
```

第二次渲染使用相同asset/animation、独立output，改为`--engine eevee --samples 4`。
`--benchmark-frames 3 --views interior_diagonal`用于固定机位冷/热帧测试。
动画加速比较使用`--frame-indices 0 1 96 97 192 193 288 289 384 385 --no-encode`，
只抽取相同10个源状态；完整相机路径仍决定固定LOD。`--metalrt ON`启用Metal硬件光线追踪，
`--png-compression 0`减少无损PNG的压缩开销（增加磁盘占用，不降低像素质量）。
报告区分场景准备、首帧、预热后平均/中位/P95/最慢帧以及编码时间；动画计时包含姿态更新和PNG。
AGX采样记录整机GPU利用率，包含桌面等其他进程，不能视作Blender独占利用率。

fast模式仅对机械臂外观使用三档LOD，按整条镜头路径的最近位置固定细节，避免跨档跳变。
原始网格保留，`--profile reference`关闭LOD。任务装置、开孔、载荷、原轨迹与物理输入不简化。
几何按刚体合并并用Geometry Nodes实例复用；逐帧批量更新点位置/朝向，变色只切换对应实例材质。
没有材质索引变化的实例云不重复刷新静态网格；运动位置和源外观事件照常更新。
变色采用绑定在网格数据上的缓存材质副本；物体级材质覆盖不能作为实例变色依据。
Blender集成检查读取最终求值几何的面材质，验证独立变色、运动后位置和立即复位。
状态及源映射有哈希验证；初始和动画首/中/末帧审计刚体转换，位置误差约1微米。
`--bake-gi`提供静态结构间接光烘焙实验，排除全部任务物体；当前不是默认方案。
导出仍为可视化，不证明Isaac/USD动力学转换。

## V10完整环视验收

- 当前视频：`outputs/robo-factory/v10-orbit16-cycles/factory.mp4`，16秒、480帧、30fps、1920×1080、完整360°。
- Cycles Metal GPU、MetalRT ON、24 samples、GPU降噪；所有36桌源轨迹独立循环，源物理与动作不变。
- Apple M5 Pro 16核GPU，预热后479帧平均2.126秒/帧，P95 2.279秒。
  场景准备4.913秒，首帧纯渲染3.120秒，进程启动至首帧8.290秒；
  编码5.568秒，总耗时1037.374秒。逐帧时间含更新、降噪与PNG，编码另计。
- 同一组10帧比较64→24 samples及更新/写盘优化：2.517→2.121秒/帧，耗时减少15.72%，
  平均SSIM 0.99694；完整480帧仅正式渲染一次，没有缩图或补帧。
- 渲染窗口整机AGX利用率均值94.79%、峰值100%；包含桌面等其他进程。
- 完整解码、480张PNG、时长/帧率/帧数、480份源姿态哈希及首中末FK通过。
- 完整记录与限制：`outputs/robo-factory/v10-orbit16-cycles/README.md`和
  `runs/engineering/factory-orbit360-20261027/final-validation.json`。

## 64 samples短片及存储策略复核

`outputs/robo-factory/v10-orbit4-cycles64/factory.mp4`为此前16秒轨迹的前4秒，120帧，
原生1080p，GPU 64 samples（自适应阈值沿用0.15）。预热后平均2.476秒/帧，P95 2.884秒。
视频通过完整解码后自动清理117张PNG；此前16秒视频也清理477张PNG。
各保留首、中、末三张无损图，累计释放约4.94GB。

直接编码实验位于`runs/engineering/factory-samples64-4s-20261027/`。
同一组10帧测得PNG流程2.539秒/帧，Blender内置FFmpeg直出2.441秒/帧，约减少3.83%。
内置编码采用CRF17，原流程CRF18；测试规模小，不保证长片同幅加速。
直出已验证源像素与AgX一致，仍作为实验入口，生产默认保留编码后清理流程。
完整限制和质量指标见该目录README与`native17-comparison.json`。

## V9历史验收

- V9有34583个视觉geom，实例复用约392份刚体网格；环绕LOD后约725万三角形，原外观约4390万。
- 两引擎使用相同36桌姿态、相机路径和源级剪辑；本轮只测试GPU。
- 最终交付为`v2`，修正了Geometry Nodes实例的动态材质传递；旧视频保留并标注superseded。
- Cycles视频：`outputs/robo-factory/v9-orbit-cycles-v2/factory.mp4`。
- Eevee比较视频：`outputs/robo-factory/v9-orbit-eevee-v2/factory.mp4`，以对应report为实测依据。
- 两视频均8秒/240帧/30fps；时间包含姿态更新与PNG写盘，编码另计。
- Apple M5 Pro 16核GPU：Cycles 24 samples平均1.38秒/帧、P95 1.74秒；Eevee 4 samples平均1.43秒/帧、P95 1.56秒。
  场景准备分别4.37/4.58秒，首帧渲染2.13/2.09秒；新进程首次可用帧含准备和更新为6.85/6.99秒。
  系统着色器缓存保留；GPU采样整机均值约93%/69%，不是独占利用率。
- 全部240帧中，6个新增检修架的保守包围盒均未与任何源任务geom包围盒重叠；不是帧间连续碰撞认证。
- 片段包含353次Dial源RGBA变化，其余家族在这段时间内无RGBA变化。
- 性能、几何/外观隔离及源绑定审计保存在`runs/engineering/factory-v9-20261027/`。
- 1008项离线测试通过，Blender最终求值几何的独立变色/立即复位检查通过；217184份受保护历史文件及36项源绑定通过校验。
- 对照图、打包贴图的Blender文件与索引在`outputs/robo-factory/v9-review/`。
- 本轮已重新prepare 36项至`runs/policy/prepared-factory-v9-20261027/`，模型调用0次、仿真未启动。
  后续再改执行源码时需新prepare，旧准备目录保留。

## 数据清理后的交付入口

`outputs/final/factory/` 提供当前 top、side 和环视 MP4 的链接。当前场景、Blender导出、光照配置、36条源流和v3剪辑保留；旧版本试图和编码后的逐帧PNG缓存按保留策略清理。旧验收中的帧数／解码结果是当时检查记录，不能据此声称PNG缓存现在仍存在。更新后的独立渲染输出不纳入本轮清理。
