> 历史归档。当前规格与状态见 [Transport任务说明](../tasks/transport.md)；合并操作见 [交接](../plans/suite-v2/transport-merge-handoff.md)。本文保留当时的过程与证据。

# Transport R12：相邻目标间隔增加20 mm

仅调整白板目标间隔并重新静态渲染九个Transport场景，不运行Oracle。

- 标识边长／直径仍84 mm。
- 相邻中心距100→120 mm，边缘净间隔16→36 mm。
- L2／L3的真实插孔、编号和目标坐标与标识同步移动；孔的尺寸和深度不变。
- 白板外尺寸仍143×416 mm，两端标识到板边各留46 mm。
- 白板安装、机器人、供料、订单、弹簧、材料与验收标准沿用[R11](transport-r11.md)。模型重新编译质量和惯量。
- R11场景和预览保留；当前目录选用R12 Transport，其余27项仍R10。新清单是`configs/suites/frontier-transport-r12.json`。

新版预览：`docs/generated/frontier-v2/transport-r12/index.html`。
只做模型编译、静态接触／支撑检查和渲染，没有机械臂动作或物理积分。
