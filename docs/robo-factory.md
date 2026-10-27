# 工厂展示与渲染

当前为V7、55×50米、四区36桌。配置`configs/backgrounds/robo_factory.json`；
布局输入`configs/suites/frontier-current-oracle.json`；场景
`outputs/robo-factory/v7-workstations/scene.xml`。离线展示没有物理自由度或执行器。
原V6及早期设计保留在[历史说明](history/robo-factory-before-integration-20261027.md)。

## 源、剪辑和独立循环

`configs/task_families/current.json`绑定每桌自己的冻结场景、录制和剪辑。
Transport 20x、Mosaic 30x、Relay 20x、Dial 10x，输出30fps，原记录60Hz。
每桌独立跳过自己的无用区间，完成后回到原始起点继续循环；循环时恢复初始RGBA，
再累积该周期的外观事件。每帧更新所有工位后只提交一次工厂FK、渲染一次。
旧交接中的“全部完成后停止”由2026-10-27最新要求覆盖。

新的`robofrontier.station-edit.v2`在
`outputs/robo-factory/source-edits-v2-20261027/`。它在历史家族剪辑之上，以2/8/32秒
窗口复核静止、微抖及反复被动振荡，保护定向慢移动、连续旋转、动作和结果/违规/
失败/恢复/外观事件附近的源上下文。新表绑定selection、源stream和语义日志的哈希。
原始流、原剪辑、原视频不改写。运动分类为10Hz审查、源选择仍用原60Hz索引；
不声称测出了接触力，也不把展示删帧作为物理通过证据。

工位采用整体平移/水平旋转，不缩放机械臂或载荷，不执行动作、IK或物理积分。
Transport原Oracle为R13，桌面开孔显示比例0.5；GPT的R14本身已是半孔，比例1。
桌面显示编辑与源物理几何分开记录，避免重复减半。

## 外观和镜头

保留四墙、屋顶、结构梁、窗门、区域文字、原桌面和相机能力；去掉北墙ROBO FACTORY字。
新增桌下服务连接、线槽、控制盒、电源组件，墙边压缩机/电柜/管线、料箱和防撞柱，
以及空闲区域边缘的补给车。设备避开规划工位和通道，未声称验证了所有搬运姿态的扫掠体积。
共享网格、重复材质、贴图和静态批处理保留；构建使用输出内资产，不改写原资产目录。

`camera-shots.json`先总览6秒、角落小幅环视6秒，再按Transport→Mosaic→Relay→Dial，
每类L1→L2→L3拍各规模工位。镜头时钟与各桌源时钟独立；可选固定镜头或自行剪辑。
这是可用的拍摄基础，尚未制作完整讲解、字幕与成片。

## 构建、观看与导出

从仓库根执行，生成目录必须是新目录：

```sh
.venv/bin/python -m scripts.factory build --output outputs/robo-factory/my-factory
.venv/bin/python -m scripts.factory view
.venv/bin/python -m scripts.factory video --duration 12 --output outputs/robo-factory/my-preview
```

`--scene`可选择新场景，`--families`可选择家族，`--camera`可选择tour或原相机预设。
viewer支持漫游、平移、缩放、俯视和四周cutaway；交互刷新上限15fps，按30fps时间线
跳采样。导出固定30fps，必须指定时长，写出逐帧源映射并完整解码验证。

交互Filament预览仍有大场景阴影/锯齿和原生arena回退警告。正式出片使用Cycles：

```sh
.venv/bin/python -m scripts.export_factory_blender --scene outputs/robo-factory/v7-workstations/scene.xml --output outputs/robo-factory/my-blender
.venv/bin/python -m scripts.factory poses --duration 12 --camera tour --output outputs/robo-factory/my-poses
/Applications/Blender.app/Contents/MacOS/Blender --background --python scripts/render_factory_blender.py -- --asset outputs/robo-factory/my-blender --animation outputs/robo-factory/my-poses/animation.json --output outputs/robo-factory/my-cycles --samples 48
```

Cycles复用场景、网格、材质及persistent render数据，逐帧更新源绑定的姿态、外观与相机。
状态文件及源映射有哈希验证，变色后恢复原材质；浮点可视化转换位置误差约1微米。
它是可视化导出，未证明Isaac/USD动力学转换。

## 本次验收

- 36桌均通过独立时钟、长时间跳转、回退、各自循环边界和RGBA复位检查。
- 36桌4秒、120帧Filament预览完整编码/解码通过；15帧Cycles动画也完成真实出片。
- 新厂33042个视觉geom、786份复用mesh，保留36桌正确源。
- 1280×720五镜头、三轮交替基准：静态帧耗时增加9%–22%，峰值RSS约1756→1805MiB。
  比较包含工位源更新和外观新增，不能把差异全部归因于背景。
- Cycles材质预览、动画及逐帧源映射位于`outputs/robo-factory/v7-cycles*`；
  性能原始数据位于`outputs/robo-factory/v7-performance/`。离线出片速度不等于实时30fps。
