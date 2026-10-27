# 四类Oracle整合验收

本页记录四分支合并时的验收快照。后续已清理四个旧任务worktree、退役旧prepared包并加入CLI宿主；当前运行方式见[GPT说明](gpt-policy.md)，保留数据见[目录说明](repository-layout.md)。下文的旧运行状态、数量和路径描述不作为当前启动配置。

2026-10-27，从main `d3e5916`合入Mosaic、Transport、Relay、Dial四个continuation
分支。原提交、失败/恢复证据、冻结源码、原轨迹、剪辑、视频和辅助标签保留。
没有删除worktree，没有启动正式GPT。

## 当前结构

日常使用[任务入口](frontier-v2-environments.md)、[GPT准备](gpt-policy.md)、
[工厂展示](robo-factory.md)。代码入口为`scripts.tasks`与`scripts.factory`。
公开家族材料移入`configs/task_families/<family>/public/`并保留原哈希；
`current.json`明确选择GPT/Oracle用途。旧脚本和catalog保留其复现职责，不再作为默认选择。
家族历史、旧计划与旧工厂说明保留于`docs/history/`，当前说明不再要求去worktree找版本。

## 数据保全

回执根为`runs/engineering/oracle-integration-20261027/`，保护检查为
`preservation-final.json`。Mosaic/Transport/Relay原保护清单约21.7万条文件检查通过；
其中一份已更新共享源文件的原字节保存在原分支Git对象。Dial另核对64项绑定，
包括原报告、选择、源元数据、stream索引和原渲染视频。36条录制的原始chunk在
生成新时间线时逐块验证SHA并解码，不修改原流。

清点了640条指向worktree的历史依赖：592条有效链接已改指持久归档，约648MiB
唯一内容；runs/outputs/vendor链接改指主数据根。归档复用内容但不与即将删除的
worktree文件共用inode。48条链接在整合前已断，来源为更早的transport-oracle、
mosaic-r13-oracle、relay-oracle目录；原路径和断链事实保留，没有猜造历史版本。
当前36项选定Oracle源、资产和流均已独立验收；48条断链不被写成已经恢复。

Git忽略资产按原哈希从state-assets恢复；恢复扫描复用33159项、恢复550项、缺项0。
这是场景资产扫描，不代表Git包含全部实验数据。备份必须包含runs、outputs、vendor、
state-assets及依赖归档；删除worktree仍由用户执行。

## 共享代码和运行准备

不同家族的旧edit结构由显式适配器识别，不能只凭相同schema字符串猜格式。
FactoryTimeline统一逐桌时钟、源帧、外观复位和一次全厂FK。独立循环规则进入
统一release，旧家族停止合同保留为历史。

新策略run冻结家族公开材料和私有release，通过独立协议进入评估与记录；原task
不改资格标记。完整源指纹仍记录实际当前字节。历史Mosaic/Dial审查需要的兼容记录
同时固定旧/新哈希，并验证未变部分的AST；只排除明确新增的prepared START检查和
与这两类无关的Relay类。不把Relay新评分说成旧评分，也不更改原证据哈希。

四类真实冻结场景都通过离线START检查，创建评估器与60Hz记录器，期间禁止物理步进。
真实Codex会话工具在本次宿主未提供；原生往返、模型表现和真正并行策略运行尚未验收。
新batch准备只产生独立运行目录，不创建模型会话或开始仿真。

## 工厂和验收范围

V7保留55×50米布局，采用36项正确冻结源，增加桌下/墙边/区域边缘设施并移除墙上
品牌字。静态批处理、网格/材质复用和近远景相机保留。Transport桌面半孔属于
明确的显示编辑，不缩放机器人或物体，不重跑物理。

原生源剪辑的新版本保留动作、进展、违规、失败、恢复及RGBA事件附近上下文，
并保护慢速进展、连续旋转和叠加在振荡上的定向漂移。分类在10Hz审查，取帧用原
60Hz索引。原家族剪辑与新版本都保留；分类不声称具备实测力或完整因果证明。

36桌经长时钟跳转、回退和逐桌循环边界检查，姿态/外观复位通过；显示模型的刚体
位置误差低于1e-13米。4秒/120帧预览及15帧Cycles动画完整编码解码通过。
Cycles静态预览和动画状态导出已接通；完整讲解Demo、剪辑字幕和正式GPT留待后续。

历史同机1280×720三轮交替、五镜头Filament静态基准（该渲染路径现已移除；当前简单任务取图使用OpenGL Classic）：耗时增加9%–22%，RSS约
1756→1805MiB。Cycles同设置的两镜头单次对照为约1.10x、1.55x；这是单次观察，
不是统计保证。原始数据与范围见工厂说明。没有据此声称实时30fps或任意机器成本上限。

最终完整离线harness为1008项Python测试，失败/错误/跳过均为0，JS及文档检查通过；
报告为`harness-release.json`。36项已准备到`runs/policy/prepared-integration-20261027/`，
输入完整性、未绑定会话、未启动世界的复核为`prepared-batch-verification.json`。
这是可检查的准备产物，实际启动前仍需核实原生宿主与预算。
物理工程回归、无物理回放与正式模型运行是不同验收范围。
