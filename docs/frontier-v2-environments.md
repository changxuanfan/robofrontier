# Frontier V2任务入口

四类任务各有三个规模、三个等级，共36项。统一清单为
`configs/task_families/current.json`，显式区分`gpt`和`oracle`用途。

| 家族 | GPT输入 | Oracle展示 | 倍速 | 规格与证据 |
| --- | --- | --- | --- | --- |
| Transport | R14真实半孔 | 原R13冻结录制；显示半孔 | 20x | [任务说明](tasks/transport.md) |
| Mosaic | R14附近混色 | 原R13配色录制 | 30x | [任务说明](tasks/mosaic.md) |
| Relay | 最终50mm Goal、低位释放、可堆叠 | 每次录制自己的冻结源 | 20x | [任务说明](relay.md) |
| Dial | R11 BEST5 | R11原始记录 | 10x | [任务说明](tasks/dial-best5.md) |

Transport/Mosaic R14是新输入；R13完整Oracle结论不自动成为R14物理通过。
Relay的最终Goal口径与历史native通过分开报告。辅助和纯程序证据保留原标签。

公共材料随新run冻结；原任务文件的预览标记不改写，新运行通过源绑定协议
进入评估和记录。见[GPT准备](gpt-policy.md)。工厂使用源级剪辑、独立循环、30fps；
此规则覆盖旧交接中的播完停止。见[工厂说明](robo-factory.md)。

历史catalog、生成器和包保留用于解释旧实验，不能直接选择当前版本。
[旧环境说明](history/frontier-v2-environments-before-cleanup-20261027.md)保留修订过程。
