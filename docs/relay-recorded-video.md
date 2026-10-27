# Relay：同一剪辑轨迹迁移背景并以20倍速播放

2026-10-27。使用原来已完成的九项任务，不重跑Oracle、不积分物理、不重置原世界。

## 三种频率

- 原仿真积分步长2ms，即500Hz。
- 原位姿记录60Hz，float32 qpos；记录包含所有关节、夹爪、主杆和独立砝码状态。
- 去静止判定沿用已审查的20Hz样本与0.25s/2s窗口；视频新输出30fps、按有效仿真时间20倍速。

旧去静止视频为16fps、名义4倍速，不是30fps。新版从原60Hz记录重新选帧，
没有把旧视频简单再加速5倍，因此不会继承旧版每段额外边界帧造成的速度误差。

## 先剪轨迹，再换场景

`outputs/oracle/relay-edited-tracks-20261027/`每个case包含：

| 文件 | 内容 |
| --- | --- |
| `edit.json` | 源场景hash、完成截止、保留区间、既有去静止规则/阈值/画面复核来源 |
| `track.npz` | 保留区间内原60Hz状态，原始时间、原始帧号、剪辑时间、每行时长、区间编号 |
| `source/` | 与这些qpos精确匹配的冻结模型、关节/几何注册表和本地资源 |
| `events.jsonl` | 完成截止前全部外观事件；即使事件落在删去等待里，也在下个显示帧前应用 |
| `initial-rgba.json` | 初始外观 |
| `source-chunks.json` | 原始chunk清单、hash以及本次抽取时已校验的chunk |

没有重新判定工厂像素是否静止。既有20Hz最终布尔剪辑表先转成以原仿真秒计的
半开时间区间，每个判定样本管理其相邻样本中点之间的时间格，再取出落在保留格里的
原60Hz记录行。qpos直接复制，未根据控制命令重建，也未在剪切边界插值或跑物理。
原记录的所有等待仍留在原始文件中；派生包才移除了这些等待。

全场机械臂/夹爪有效动作按OR保留；极小数值噪声、全场静止、只有砝码摆动的等待，
继承此前经过960段画面检查的同一剪辑结果。主杆滑落等真实事件仍保留。
这是带阈值的工程运动识别，不保证识别任意小于记录/采样分辨率的动作。

`track.npz`中的`edited_time_s`从零开始，等待被切掉，`source_time_s`保留原时间跳跃。
`duration_s`累加为有效时长，原采样落在2ms时钟上，实际相邻记录约16/18ms，
因此按原时间差而非固定帧号计算速度。不同区间边界不桥接。
`context_qpos`/`context_source_time_s`另保留原始首末证据姿态。

## 场景迁移的边界

`source/workstation.json`给出稳定body/geom身份以及qpos布局。工厂模型由每个包的
精确冻结源组成，旧版本杆件不混用。机器人未命名几何使用注册表的`source_index`，
不能把诸如`A_link0:geom0`当作MuJoCo原生名字查询。

源FK产生每个body位姿；工厂给工作台、全部机械臂、物体及相机施加同一个平移和
绕竖直轴旋转，尺寸不缩放。世界坐标自由体和机器人根部都正确转换；关节角仍使用
本地值。桌子材质、光照、工厂静态背景可以不同，剪辑与原始时间不变。
只改变部分基座/桌面相对布局、缩放器材或改变物理，不属于可直接复用的刚体迁移。

工厂是零task动力学的视觉模型，以mocap body呈现已记录的FK位姿。
不把它说成“工厂物理重新验证成功”，也不把其他静态工作站说成并行完成任务。
每个输出帧校验源/目标全部可见几何的位置和旋转矩阵，误差限1e-6；源场景hash不符则拒绝。

## 20倍速与逐帧对应

`video_rows`在连续剪辑时钟上每`20/30`秒选一个原记录状态，未作qpos插值。
任意背景使用完全相同的行号。首末各增加一帧证据姿态，总额外时长2/30秒。
因此视频时长为`ceil(有效仿真秒×30/20)/30 + 2/30`。
快放时短于显示采样间隔的动作可能只在轨迹包里完整保留；需要慢放时可以用同一包重新渲染。

每项`outputs/oracle/relay-20x-20261027/<case>/frame-map.csv`由两种背景共享，包含
视频帧号/播放时间、原始仿真时间、剪辑时间、轨迹行号、原始60Hz帧号、区间号。
`render.json`记录同一派生轨迹hash、工厂hash、倍速、帧率、几何误差、完整解码与视频hash。

## 可重用命令

```sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=. \
  .venv/bin/python -m scripts.export_relay_edited prepare \
  outputs/oracle/relay-active-only-20261027 \
  --output outputs/oracle/relay-edited-tracks-20261027

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=. \
  .venv/bin/python -m scripts.export_relay_edited factory \
  outputs/oracle/relay-edited-tracks-20261027 \
  --output outputs/robo-factory/relay-recorded-20261027

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=. \
  .venv/bin/python -m scripts.export_relay_edited render \
  outputs/oracle/relay-edited-tracks-20261027 \
  --output outputs/oracle/relay-20x-20261027 \
  --factory outputs/robo-factory/relay-recorded-20261027/scene.xml
```

新场景用新的输出目录并重新组成精确源。输出编码先写partial文件，完整解码和帧数
校验后再作为最终影片；复用已有结果时核对渲染计划。所有旧影片与原轨迹均保留。
