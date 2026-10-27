# Mosaic 家族提交给共享整合的接口清单

2026-10-27版本选择已更新：未来GPT使用
`configs/task_families/mosaic/r14.json`；原Oracle及工厂回放继续使用`r13.json`。
选择声明在`configs/task_families/mosaic/selection.json`。R14仅改初始供料颜色，
物理/初态几何等价，但不是一次新的Oracle成功；详细规格与证据范围见
`mosaic-r14-local-colours.md`。以下原R13证据继续保留，不能覆盖新版选择。

G1接入需区分“原R13完整Oracle资格”和“R14颜色变体的几何等价/本地供料设计”。
不能把原Oracle的完成颜色资格复制到新场景；共享门禁应明确颜色变体接入协议，
登记新scene/task/公开材料身份及证据范围。G2为新GPT加载R14公开task和
mosaic-r14.md prompt，沿用Mosaic操作skill；G4仍绑定原R13冻结轨迹/配色。
正式启动前继续保持policy_launch_ready=false，不自动启动或重跑Oracle。

本分支提供Mosaic R13家族包与私有轨迹审查器。以下公共修改仅登记需求，未在本轮
实施。共享文件应由整合负责人统一修改，避免各worktree分别覆盖相同模块。

## 已有可接入的家族产物

- registry：`configs/task_families/mosaic/r13.json`，九项独立绑定实际R13资产、task、
  public-task、成功report、动作审计、stream、action/trace/process日志及控制/裁判。
- 数据构建/检查：`scripts/build_mosaic_policy_package.py`。只生成review bundle，
  不调用prepare_run，不创建连接/角色/数据库，也不启动仿真或模型。
- 待加载公开材料：包内九项public/cases/<case>/task.json及target-pattern.png；
  public/guidance/mosaic-r13.md与public/guidance/mosaic-r13/SKILL.md。
- 私有轨迹审查：`scripts/audit_mosaic_motion_intervals.py`，输出
  `robofrontier.mosaic-motion-review.v1`，不是通用motion-edit协议或自动编辑器。
- 当前跨背景精确映射仍是已交付的30倍速motion-edit表；本轮没有改动该共享实现。

## G1：正式就绪登记与门禁

当前validate_task只接受preview/显式probe，World.start_trial拒绝preview。
建议新门禁消费家族登记和新run协议，核对case、资产、控制、裁判和证据范围，
再创建新的冻结输入；不修改历史motion/source，不伪装为旧bounded probe。

Oracle模式和已知失败应原样保留：三个4臂选中案例为pure_program，六个6/8臂
选中案例为privileged_assisted；七次已知失败已物理恢复，不是零失败或GPT策略
经验。该证据说明原工位可完成，不代表未来GPT已获验证。

验收：错误case/旧R10、缺失证据、控制/裁判差异或运行协议未冻结时不能启动。
正确通过的数据review bundle也不能单独置policy_launch_ready=true。

## G2：公开投影、家族prompt/skill和封存

公共装载器需显式选择家族材料，在新run封存前加载，写明public输入清单/hash，
通过一条清晰的上下文指针使policy读取。只新增文件不会让当前run_setup自动加载。

家族包的task.json是补充公开投影，不是完整private task或可执行启动配置。
接入时按白名单合并completion、静态规格和guidance等补充；新run的正式readiness
由共同门禁决定，不能被审查包保留的历史preview字段覆盖。正式policy使用干净的新
上下文，不继承包含Oracle私有证据的工程聊天历史。
公开补充completion、静态桌面/相机等；完整对象真值、Oracle配对/修复/动作/经验
仍在工程侧。工程manifest/registry不进入policy上下文。

当前servo2文件完整内容作为合格Oracle基线保存。未来policy的模型、effort、
topology与费用/运行协议单独冻结；改变模型并不自动等于改变物理控制，也不能
把旧harness内的模型值当成用户已经选择的新实验模型。

现有physical_control_fingerprint对pipeline.py等整份文件做hash；readiness等
公共代码变化也会改变该指纹。应先做影响分析与兼容证据，不应一律声称物理变了
必须全套重跑，也不能直接抹掉指纹差异。未证明等价前保持门禁阻断。

## G3：统一轨迹编辑协议

Mosaic/Dial/Transport目前存在同名v1但不同结构；部分区间inclusive，部分half-open，
采样/事件校验/源定位方式也不同。确定一个明确新版本与旧格式适配器，不直接用
一份文件覆盖另一家族实现。

应绑定原始recording、scene、stream和events、编辑决定与审查来源；区分真实
源时间、剪后内容时间、输出帧时间；保留精确已交付帧序列与完整采样率保留窗口。
重新采样宜使用连续剪后时钟，定义端点/剪辑边界/不足一帧的处理，不累积切段误差。
历史精确帧模式仍需可重现，不能在转换时悄悄改变旧视频语义。

Mosaic审查报告时间区间为[source_start_s,source_end_s_exclusive)，同时给出的
first_source_frame/last_source_frame均为inclusive。公共适配器应显式转换。
`proposed_idle_cut`仍是提议，`review_repeated_*`仍待作用审查，`keep_*`需要保留；
不能把任何候选报告当作已批准删帧表。measured_zero_force_proven始终不作真值推断。

## G4：多工位时钟和统一渲染

每桌一个源记录/编辑映射/速度/工位变换；每个输出帧分别解析各桌源状态和事件，
更新全部工位后，只渲染一次工厂。FactoryPlayback当前会改全局data.time，公共
编排器应最后设置展示时钟，不能让最后一桌的源时钟成为全场时间。

本用户已选择hold_final：每桌完成后保持成品，允许完成后静止；最长桌结束时
结束视频，不循环，不通过全局剪辑跳掉其他桌子的动作。Mosaic默认30x/30fps，
其他家族默认需显式读取各自协议，不在本分支强行统一倍速。

旧v6 Mosaic工位版本不符；匹配源为v6-mosaic-recorded-20261027，但整合所有
家族时仍应逐桌核对其最终冻结源，整体刚体放置并拒绝缩放/版本错配。

## G5：语义审查和必要遥测

Mosaic审查器保护实际动作区间、commanded grasp lifetime、释放/拾取/恢复等
关键事件、进展变化和完成之后的状态。只分析其余持续≥4s窗口，检测小幅颤动、
缓慢净移动和有界重复明显振荡。当前汇总窗口策略偏保守：一个长候选中发现有效
运动时可能保留整段；不是已经验证无遗漏的通用语义分类器。

force_n是设定，现有measured_grasp_force_n可能为空，qpos不能证明实际零接触力。
候选审查不重做接触/动力学求解，也不补造历史力数据。需要真实力/支持资格遥测
时，另定义只读通道的单位、采样和来源，合并后实现。

## 共同验收

1. 两桌等待错开，各自删留互不影响；不同源时间的姿态可出现在同一工厂帧。
2. 已完成桌保持精确最终成品；没有新的物理启动/重置或循环实验声明。
3. 有效极慢运动/夹爪操作/支撑/真实失败恢复不被删；无用大振荡可以提出审查，
   不能仅由幅度决定，更不能把缺失的力测量解释为零。
4. 源/编辑/事件/变换错配被拒绝，历史记录hash不变，所有参与家族各自通过回归。
5. 新GPT实际加载的公开任务/角色/skill与封存清单一致；未知读入路径不能绕过合同。
6. 真实任务效率只用原始时间/动作/失败记录统计，展示剪后时长不作为算法加速证据。
