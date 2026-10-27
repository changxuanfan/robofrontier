# Compact 后先读此文件 — 2026-10-27

这是本轮最新接续说明，覆盖旧 `relay-continuation.md` 中过期的进度、
5/25mm 标准和“尚未热加载/尚未得到许可”等状态。先读完，再核对原现场。
**用户刚要求 compact 后仍保留细节，并明确“程序不要停”。不要停止、重启、
另开原仿真，也不要把 compact 当作取消任务。继续直到9项完成。**

## 2026-10-27最新：可迁移剪辑轨迹、简单/工厂18段20倍速全部完成

用户要求去静止必须绑定动态轨迹，以便原背景与工厂背景同步剪切；所有视频改20倍速。
现已完成，不需恢复任何控制任务或媒体进程。没有重跑Oracle，没有向原世界发送命令。
原物理任务仍使用历史9/9、27/27的验收记录，native分数/原始轨迹/旧MP4均未改。

最新总览：`outputs/oracle/relay-20x-20261027/report.md`，summary.json complete=true、
completed_tasks=9/completed_videos=18。新片均30fps、按剪辑后的有效仿真时间20倍速；
首尾各仅增加一帧证据姿态。旧16fps名义4x片保留，下面旧章节中的4x仍属历史版本。

集中交付目录（每个目录直接9个MP4，按臂数/Level命名）：
- `outputs/oracle/relay-nine-videos-20x-20261027/`：简单背景九段。
- `outputs/robo-factory/relay-nine-videos-20x-20261027/`：工厂背景九段。

九段对应时长（两个背景一致）：8臂L1/L2/L3为17.667/24.467/23.033秒；
12臂50.300/45.733/58.033秒；16臂87.500/79.500/88.867秒。
所有18段完整解码、30fps与帧数检查、交付文件SHA256复制核验通过。
三张qa-08/12/16-paired-first-last.jpg已实际查看；8L1中间帧和12L3/16L2工厂末帧也已查看。

独立可复用数据包：`outputs/oracle/relay-edited-tracks-20261027/`。
每case含精确冻结source模型与资源、edit.json保留源时间区间/阈值/既有960段图像复核来源、
track.npz保留区间内的原60Hz qpos，共570746行；每行原始帧号/源时间/剪辑时间/
时长/segment均保存；另有首末证据状态、外观事件、初始RGBA及源chunk清单hash。
原始记录60Hz，仿真2ms=500Hz，去静止判定沿用已审查20Hz mask（0.25/2秒窗口），
不在新背景重复识别像素，不跨剪切插值或积分物理。track-validation.json九包全部通过。

工厂重建：`outputs/robo-factory/relay-recorded-20261027/scene.xml`，9个relay视觉工作站，
每个源都从上述精确冻结记录组成，拒绝旧版本同ID混用。FactoryPlayback.apply_arrays
向量化FK/共同平移和绕竖直轴旋转；不缩放、不改变相对布局。材料背景可不同。
所有输出帧验证全部可见几何的源/目标位置与旋转，最大位置差5.03e-15米。
两种背景共用每case的frame-map.csv。是原记录展示，不是工厂重新跑物理成功。

实现：robofrontier/motion_edit.py、scripts/export_relay_edited.py、
FactoryPlayback.apply_arrays；格式/命令详见docs/relay-recorded-video.md。
19项剪辑/运动/刚体迁移测试通过；额外7项旧综合测试由于已有非relay生成环境的
身份贴图缺失/外链，未进入测试主体；没有修改那些环境或降低资源校验。
原因见新输出test-results.json，不要声称全库测试通过。

进程提醒：本轮只新开离线抽取与渲染辅助进程，现均完成，没有给原仿真发STOP或重启。
本轮只读ps检查时旧52509/52510未找到，不能根据下面历史章节继续声称它们当前仍活着；
不要因此重启、重复执行旧控制ID。物理任务在原记录中的9/9完成结论不变。

## 九个视频也已全部去静止：L1三段已补齐（历史4倍速版）

用户追问“9个视频都去掉静止了嘛”，如实说明原先只有6段L2/L3，然后补齐三段L1。
现在同一 `outputs/oracle/relay-active-only-20261027/report.md` 已为**九段新版总览**。
8臂L1 1:33（1482帧/16fps），12臂L1 4:26（4256帧），16臂L1 7:49（7501帧）。
L2/L3六段时长仍2:11/2:06/4:08/5:42/7:20/8:06，旧版都保留。

L1使用 unchanged_l1 记录里的真实原生成功源，来源是
`runs/oracle/relay-nine-ab1abec-20261026-032833/relay-aXX-d1/attempt-001`，没有重跑物理。
旧L1 schema没有beam_bodies和user-goal-report，离线脚本已支持beam_body回退以及
native report.passed=true的验收源，末帧保留原关闭event精确状态。
九段全部完整解码、帧数匹配，九段首末图已查看，累计960个较长待删段做图像复核。
L1三段图见qa-l1-first-last.jpg，全部汇总summary.json complete=true/completed_videos=9。
所有剪辑辅助进程均已完成，原52509/52510仍运行，原始轨迹、旧MP4与原场景未修改。
后续用户再问9视频去静止：可以直接答“九段现在全部按同一套规则完成”，链接新版总览。

## 视频追加要求也已完成：请优先交付去静止新版

用户随后明确要求：全场检查所有臂关节/夹爪/主物体位姿；0.25s和2s累计窗口过滤
数值抖动并识别慢动；删除全场静止；砝码单独摆动但没有有效臂动作的等待也删除，
有臂动作时仍保留；增加图像复核。

原版不完全满足，已在**不碰原仿真与原轨迹**的前提下统一重剪全部六段L2/L3。
新报告与全部新视频：
`/Users/fanchangxuan/Repos/roomscript2/outputs/oracle/relay-active-only-20261027/report.md`
各 `relay-aXX-dY/global.mp4`：8L2 2:11，8L3 2:06，12L2 4:08，12L3 5:42，
16L2 7:20（7037帧/16fps），16L3 8:06（7768帧/16fps）。原版保留对照，旧报告已加新版入口。

实现 `robofrontier/relay_video_motion.py` + `scripts/render_relay_active_only.py`，
13个规则单测通过，包括单臂动、慢动、数值抖动、砝码单独摆动/与臂同时动、
主物体滑落、砝码大幅独立落下、四元数符号、画面重量遮挡不能伪造成臂运动等。
原60Hz记录提取20Hz判定样本，比较0.25/2s净变化与方向一致性；所有臂/夹爪
按各自关节检测后取全场OR；主木杆和L3被动关节运动保留，疑似脱落的大幅重物位移也保留。
不再把“存在动作指令”当成整段必须保留。首尾各只留一帧定位，没有静止等待片尾。

对771个>=0.4s较长/可疑待删段做实际RGB+geom-ID分割复核，排除前后两帧砝码遮挡，
用轮廓/可见几何中心检查有效臂/主物体运动；0段需要补回。短数值间隙按轨迹判定。
源时间映射见frame-map.csv；阈值/样本/删除检查见manifest、state-summary、visual-audit和audit JPG。
六段均全视频解码成功、帧数匹配、首末图QA完成，qa-first-last.jpg已看；旧MP4未覆盖。

六段重剪所有辅助进程均已完成。原物理任务仍9/9、27/27，52509/52510仍运行，没有STOP、
没有新开物理世界、重启或状态回滚。这次只做离线剪辑，不需要恢复任何旧控制请求。

## 最终交付已完成（物理、校验、两段影片全部完成）

**9/9、27/27；没有剩余控制任务或媒体任务。** 两个原物理进程52509/52510按用户要求仍运行，
本轮从未给它们发送STOP。不要再执行历史ID，不要因为旧native评分3/9而重跑已通过用户标准的任务。

最终报告：
`/Users/fanchangxuan/Repos/roomscript2/outputs/oracle/relay-user-completed-20261027/completion-report.md`

最终新增完整影片（均从原任务初始0秒到通过验收附近，动作4x、等待压缩）：
- `relay-a16-d2-full/global.mp4`：15255帧/15fps，16:57；完整解码通过，首末PNG已看。
- `relay-a16-d3-full/global.mp4`：47618帧/15fps，约52:55；完整解码通过，首末PNG已看。
目录同上输出根，各manifest和video-inspection.json、呈现优化sidecars已保存。
视频不是新物理模拟、不是闭合episode；原进程继续，截止来自真实原记录，末帧另验实际落地/分离/框内。
旧四段8/12臂L2/L3完整影片仍在20261026输出目录，最终报告已统一链接。

媒体PID32831/32832及其编码器已完成，不再需要轮询sessions22221/84382。
L3后段在第22844帧后关闭导出阴影/反射提速，只影响绘图；原动作/姿态/源数据不改。
等待判定优化只去掉执行区间外机器人关节/夹爪微抖动；全部执行区间和payload自由体/被动关节阈值不变。
已将生效帧/源时间写入最终manifest，早期画面保留更多等待；没有删失误或真实物体运动。

最终验收根文件：`USER-GOAL-ALL-NINE-COMPLETE.json`、`user-goal-batch-status.json`（9/9），
原严格native batch-status.json未改写。最近相关检查36项通过，源码/控制修正已commit，
后续只需根据用户新指令行动，不恢复任何旧重复世界。

## 已完成：9/9 项、27/27 根（最新，覆盖下文全部未完成前沿）

**物理任务已经全部完成，不再推进任何 Oracle 控制或重跑任务。**
两个原PID 52509、52510依用户“程序不要停”仍运行；没有给它们写STOP。
L3最后P1完成最终J/P(v155)、I短撤离(v156)、Goal前移(v158)、下降v159-1..10、
距表面约13mm时真实完整80mm开口v159-11和观察v160；4/4全部分离落在框内，
第一验收时L3四根0越线。L2也4/4。根目录user-goal-batch-status.json已独立9/9，
USER-GOAL-ALL-NINE-COMPLETE.json已保存，原native严格评分与历史不变（汇总仍3/9）。

最终P抓姿问题用O24cm暂接、实际P空手撤离、I/O恢复level，再用完整后续运输
筛选P58cm Top分支解决。真实v149 P空手free-wrist撤离、v150 level、v151新Top P
交接、v152/v153 O撤离home、v154实际驶入最终窗、v155 J/P交接均已完成。
之前v124 Side P及可选整杆roll耗尽腕限位的历史保留，绝无物理checkpoint回滚。

最后安装的contact escape为**revision5**，同一52510里已ack，frozen
`open-contact-withdraw-v5.py` SHA256
`6d761f2e3ca1755bd4b5160e124da020d5ee6cbdc76f74aecd7a5f67872137bb`。
仅当另外两臂有实际双边力支撑时，空手短距6cm撤离可转动最多0.20rad；
仅Goal接地无双接收臂时仍0.04rad。81+点原几何非加深/无新穿透/机器人世界
碰撞检查与真实分离后置检查不变，未改physics或monitor。该模块6个单测通过。
最后Goal组合installer `final-goal-recovery-v2.py` SHA256
`491e2b39120c79b7bb44d1f96967f7079a5ded87530bbb7313cfcbba499569e3`，
只安装已测试Goal50、真实支持/低位80mm开口及contact withdrawal；同一实例/world。
实验性的world-topdown/Top-only新语义从未装入活世界，源码已恢复原语义，不依赖它。

**仅剩媒体导出/交付（物理已完成）**：
- 只读前缀校验脚本 `scripts/verify_relay_completion_prefix.py` 已验证L2 565086帧、
  L3 1517398帧（source从0起），每个chunk checksum/shape/time/finite，最后真记录帧
  另验inside50/released/grounded/qualified handover；不新增物理episode。
- 原进程继续运行，不使用close/STOP换视频。renderer现在支持--completion-sim-s，
  使用真实记录截止帧而不伪造trial-end事件：L2截止9418.100001001016，
  L3截止25289.960004234537，原始完整motion数据仍持续保留。
- 输出 `/Users/fanchangxuan/Repos/roomscript2/outputs/oracle/relay-user-completed-20261027/`
  的 `relay-a16-d2-full/global.mp4`、`relay-a16-d3-full/global.mp4` 正在导出。
  exec sessions **22221 / 84382**，media PIDs32831/32832，子编码器32835/32836。
  VideoToolbox硬件编码已smoke通过，ffmpeg线程/filters为1；观察总CPU每case<200%。
- 仅媒体进程应用等待判定优化：去掉指令执行区间外机器人关节/夹爪微抖动的idle
  检测，所有指令区间与所有payload自由体/被动关节运动阈值保持原值；原物理进程未触碰。
  各输出目录 `export-idle-motion-policy.json`保存生效帧/时间，完成后并入manifest。
  没有删去失败动作或物体真实运动；早期画面保留更多等待。
- 完成后看首末PNG、ffprobe时长并确认manifest decode_passed，再给用户这两段完整录像。
  报告 `completion-report.md` 已写在输出根目录；需要把最后两行“导出中”更新为可播放。
- 还未最终commit的改动：contact escape revision5/6 tests；视频cutoff/hardware选项及
  verify-prefix新脚本；本checkpoint。不要将9/9重新标成未完成或再次执行旧ID。

## 最后L3卡点已解开的最新动作

仍26/27、8/9；16L2四根实际完成，52509依用户要求继续活着，未STOP。
16L3最后P1：`r16-p1-N-local-lattice-v117-1` 已实际完成空N局部姿态调整，
`r16-p1-IO-past-lattice-N-v118` 已实际完成向前250mm，原来N臂阻塞已穿过。
成功的只读规划器：`runs/engineering/relay-descent-20261026/finite_local_empty_lattice.py`。
只在N四个相关关节的625/750候选局部网格中检查186个clear状态，135个有完整后续运输，
局部visibility检查185条边，展开1个状态，用3.229秒找到可执行1段普通空手路线；
不是再扩大百万次全局IK。原强求home和很多单关节直线未解决，禁止重复那些旧搜索。

实际I/O双持P1 Root约[1.5022,.7573,1.2570]，测得yaw1.87037，仍倾斜/roll；
**当前有限helper**正在执行 `IO-level-after-N-v119`（正常恢复level）→
`IO-open-P-window-v120`（到[1.5,.65,1.2]等有限候选）→
`IO-to-IP-final-Top-v121`。先看pending/results，不重放ID。
然后I/P到J/P，Goal放置，独立50mm复验全部4根和所有原记录。

用户尚未看到16L2完整成功影片；视频要保留全原episode动作/失误/修正，允许压缩静止段。
当前两个原case都仍活，原renderer --user-goal-complete 要关闭stream后精确final event，
不要冒充已完成封存/完整影片。最后9/9实际验收后可正常结束并封存记录，用户说九项完结束。

## 最新：26/27根，8/9项（覆盖后文所有早期前沿）

16L2已真实4/4完成，`user-goal-report.json` passed=true，`USER-GOAL-COMPLETE.json`已保存。
为遵守用户“程序不要停”，**目前没有给16L2写STOP，52509仍运行**；剩余L3继续。
16L2最后P1完整I/P→J/P(v78)、Goal(v79)、正常下降(v80)后遇到旧Goal5mm规则，
新user50接触策略v81允许P1砝码接触P3，release开口P失败但无力；完整80mm开口v85
及P最终+30mm正常撤离v86已完成，真正松开/无接触，全部Goal50内。旧native/drop历史未改。

新控制/观察修正（均同一原进程已获授权热加载，记录在各case controller-extensions）：
- `relay_user_goal_policy.py`：Goal50接触名单、真实Goal堆叠支持/组件离桌30mm低释放。
  **只是操作策略，不修改native monitor/评分/历史/几何**。
- 16L2 `goal-stack-open-v1-loaded.json`：49448a4f94d66c3c6788d902d5a54e1dc2dd7a7e4e6419f5f7e6e8e452b5d425，typed open可接受真实支持的
  user50堆叠并开至80mm；不能打开标记为其他cargo的手，接触存在时保留held标签。
  新process保留同实例/world和所有原guard；16个live_recovery测试通过。
- `relay_contact_escape.py`现在revision3：相同短距/81采样/非加深接触约束，接地支持图
  改用已批准user50 Goal名单，避免框边缘已完成P3被旧strict排除。
  16L2 frozen `open-contact-withdraw-v3.py`；组合installer
  `user-goal-withdraw-native-v1.py` SHA256
  `3c34d1bec3a46a6ca3b021b3a56663b1af0474d707187d192295996edb84557b`。
- `relay_observation_state.py`：只读native double joints + cached free-body poses，
  包含砝码与关节，不再依赖最多2simsec旧的float32影片块；没有live qpos写入。
  两个原case均已加载。`native-kinematic-state.required`要求diagnostic等待发布的
  rendering窗口结束，**不能在原publisher先写普通JSON后暂缺native字段时fallback旧chunk**。
  3个单测 + 真实场景全几何FK重建误差0通过。Goal策略4个测试/contact3个测试通过。

**现在仅16L3 P1剩余，实际 I/O Top 相对木杆，已经倾斜并绕跨度滚转。**
- v87空间倾斜约17deg已完成，v88-1/2前移完成；v93/v94继续空间转向，
  v95 I/O交接已真实完成；v96空闲H已home。
- v99绕长跨度roll -0.2已完成，v100前移80mm、v102前移至Root约x1.50已完成，
  v103前移y-50mm已完成；v104再roll -0.15已完成，v105被原guard提前拒绝（无动作）。
- 最重要的新解：**空N臂不该强求home，也不是只动手腕，应把forearm降到悬挂砝码下面**。
  旧单关节折叠v54曾把N q[2]抬高.4，导致N_link5挡住O_link2；
  现在真实v108已把N q[2]降低.15（约-1.74），frame路径4mm终点/10mrad完整扫掠已过。
  这不是state rollback，是记录完整的普通空手动作。N仍force-free/open/cargo-separated。
- 当前helper `runs/engineering/relay-descent-20261026/finite_spatial_local_progress.py`
  是旧局部v92脚本（已结束），**不要重跑**。
  新正在执行的有限前缀是 `r16-p1-IO-past-N-v110-1..3`，必要时
  `r16-p1-N-under-cargo-v109-1..3`。先看immutable results/pending，不能重发。
  目标让Root y降到约.8，再把杆恢复较平姿态，I/O→I/P→J/P→Goal。
- 其他L3 P2/P3/P4在Goal，之后最终必须复验；大多数v69..107规划未提交，
  不从编号推断执行。实际原world仍be2204c7cc834e2792271a959a435d9e/PID52510。

## 更晚的原现场前沿（覆盖后文早期增量）

**25/27根、7/9项**。16L2 P2已完成，Top J/P最终正常release通过，框内0越界，drops0。
16L2现在3/4，只余P1；16L3仍3/4，只余P1。原PID/world不变。
- L2 P1：D/M先正常前移，D/M→E/M(v63)、E/M→F/M(v65)、F/M→F/N(v66)、
  F/N→G/N(v68)均实际完成。负侧站位已交替32/54cm避免相邻手掌重叠。
  最新有限helper：v69 G/N前移已完成，v70 G/O交接已选择并在原进程执行；
  随后v71前移、v72 H/O Top待实时反馈。必须先看pending/results，不能重发。
- L3 P1：3个N空手fold(v65-1..3)和H/O前移v66已完成。
  实际H/O Top，Root约[1.269,1.181,1.190]，N空手仍停在拥挤肩肘区。
  大量只读的home、单关节fold、位置IK、整段carry未通过；这些未产生物理请求，
  不是掉落、不是进程结束。不要重复这些静态搜索。
- L3新增空间姿态typed接口已按现有用户授权热加载到同一52510的
  RelayLiveRecovery.process方法；只有已测试target_quaternion_wxyz分支，
  不替换wait循环或重新构建实例，不改状态/历史。14个相关单测通过。
  frozen extension `spatial-carry-v1.py` SHA256
  `93d3e04dab818bb67a7214b360354cd1a06ef93e70a6a0c012b9e6b76b88fbc5`，
  ack installed=true，同PID/world；LLDB已detach。
  loader `runs/engineering/relay-descent-20261026/relay_spatial_hotload_loader.py`。
- 最新L3实际请求v87：绕H负侧握点倾斜17deg抬起正端，完整杆/机器人/硬件 sweep已过；
  helper随后v88-1..8每段最多5cm，真实反馈后重建自由砝码姿态，不把同一砝码刚性预测
  到整段远端。**这是当前执行前沿，先读结果**。普通执行/掉落/碰撞guard保持。
- 普通物理反向退让允许保存真实历史；禁止live状态回滚。v84/v85退让完整方案
  未通过，只读，无物理动作。其他v69..86静态L3编号多数未提交，勿从编号推断完成。

## Compact 后继续工作的最新增量（2026-10-27）

以下新进度覆盖下文旧前沿：**24/27根完成，仍7/9项**。原PID/world不变。
- 16L2 P3：`r16-p3-P-current-normal-escape-v44` 已真实完成分离；独立50mm验收通过，
  仅约1.56mm越线、无force/robot contact，native completed也为true，drops0。
  `r16-p3-P-separated-home-v45`已完成，P可重新参与运输。16L2现2/4。
- 16L2 P2：G/O→H/O(v41)完成；H为Side的中间抓姿不再被错误当作禁用。
  多个普通carry前移到[1.55,.8,1.2]，`r16-p2-HO-to-IO-palm-clear-Top-v51`
  已完成I/O Top；最新helper正在执行v52 I/P、v53前移、v54 J/P的有限前缀，
  **先核对原results和pending，不能重放这些ID**。P1仍D/M Top，当前local站位约-.431/+.320m。
- 16L3 P1：确认旧N Side张爪后本来已可无接触停留；把立即home当成交接条件是冗余约束。
  新`robofrontier/suite/relay_donor_staging.py`只预测完整21点普通75mm开口并要求
  停留点1mm全景无接触，不改活物理；后续carry仍包含静止donor的全部几何。
  同一52510进程已按既有用户热更新授权加载`parked-donor-v1.py`，SHA256
  `2fa937b9cbc878075ded78ae0ac6c30cc2b9584e07dad7ae3d95696a83e13126`；
  ack/queue/detach均确认，runtime source/meta在原16L3 recovery/controller-extensions。
  新loader `runs/engineering/relay-descent-20261026/relay_parked_hotload_loader.py`。
  捕获现场正常75mm开口通过、45mm未张开负例拒绝、私有诊断qpos未变；21个已有recovery单测通过。
- `r16-p1-HN-to-HO-safe-parked-N-v47`已实际完成H/O Top交接；
  `r16-p1-HO-separate-parked-N-v50`已沿实测N tool Z移动120mm，木杆与N真实分开。
  N直接home(v51)只规划失败无动作；N局部q[2]+.4的v54已真实完成。
  下一段H/O forward仍被N_hand/O_link5..7挡住；不要重跑大范围IK。
  最新helper正在验证/执行`N-compound-future-fold-v58-1..`和`HO-Ncompound-forward-v59`，
  以完整后续carry清障为目标选有限空手折叠路径。单关节v56、carry v57未提交。
  N复位并非必须，只需当前无力空手的完整实际短路径让开后续transport。
- L2和L3两套原世界继续并行，各最多物理1核+辅助1核；不新建/停止原进程。

## 一分钟恢复

1. 工作目录 `/Users/fanchangxuan/.codex/worktrees/relay-continuation/roomscript2`，
   分支 `codex/relay-continuation`；读本文件，再读旧 continuation 获取更早背景。
2. `cat /tmp/relay-r16-active-root.txt` 是唯一实际推进的原世界根目录。
3. 确认 PID **52509、52510** 活着、world ID 未变；监督进程52485。
4. 读取各 `attempt-001/recovery/observation.json`、最近原始 `results/*.json`，
   核对 requests/accepted 中没有未决物理动作，再规划。**不得重放已存在ID。**
5. 最新精确快照：`runs/engineering/relay-descent-20261026/owner-frontier-current.json`。
   文件快照也会过期，原世界最新不可变反馈优先。
6. 目前 **7/9项、23/27根** 按用户标准完成。剩4根：16L2的P1/P2/P3、16L3的P1。
   恢复后继续调度，不只回答“可以继续”。

当前完整会话JSONL（需要核对用户原话时按角色/关键词读取，不整篇灌入）：
`/Users/fanchangxuan/.codex/sessions/2026/10/26/rollout-2026-10-26T00-40-13-01a173ef-208f-74d0-ae4c-faf10e1135f1.jsonl`。

## 不可丢的用户要求

- 只完成 Oracle 九项（8/12/16臂 × L1/L2/L3），不运行GPT策略。
- 没有总时间期限；不因慢或超时封存未完成任务。
- 继续原来的物理进程/现场，禁止重跑前缀、状态回滚、活世界qpos写入、焊接。
- 最终真实松爪、落入白色Goal，**边界统一允许50mm**。不要求速度/静止时长，
  不要求木条或砝码先碰桌才松爪；允许Goal内木杆接触、碰撞、稳定叠放。
- 仍保留真实合格空中交接、实际动作和失败历史。不能伪造旧严格native通过。
- 每项最多2个计算核心：原物理主进程1 + 最多1个计算辅助进程。
  `diagnostic_world` 的 `reserve_helper_core` 用 flock 限制辅助槽。
  不要持同一槽后启动子进程并等待它再获取该槽；需要串联用同进程runpy。
- 用户反复要求提高速度。不要长时间分析而让已完成动作无人接续；及时收取反馈，
  批量执行已明确的有限动作，只在新障碍处返回判断。不同case可以并行。
- 不启动subagent，除非用户或适用指令明确要求；当前开发指令禁止主动委派。

## 世界与进程

根目录（runs实际共享到主仓库）：
`runs/oracle/relay-r16-38b5423-20261026-171651`

| 原世界 | PID | world_id |
| --- | --- | --- |
| relay-a16-d2 | 52509 | 9ecf541b82b9407988a5aeb2a58db55f |
| relay-a16-d3 | 52510 | be2204c7cc834e2792271a959a435d9e |

其余已完成原case已按用户验收关闭并完整保存。**不要恢复重复世界**：
R17监督74995/子75025–75030、重复8L2 PID66700、旧55214/99193都保留SIGSTOP。
它们不是当前有效进度。原L1证据沿用，几何未变。

## 完成情况与视频

- 三个L1全部完成：8L1 2/2、12L1 3/3、16L1 4/4。
- 原8L2、8L3各2/2，原12L2、12L3各3/3，均按用户最终Goal标准关闭。
- 原12L2最后P1约35.9mm越线，**用户50mm内允许，禁止再追修它**。
- 原12L2完整录像472,741帧；原12L3 1,094,707帧，全部逐块checksum/解码/时间核对。
  精确最终event状态另验；最后event比60Hz样本最多一个采样间隔，不能伪造stream状态。
- `USER-GOAL-COMPLETE.json`、`user-goal-report.json`、
  `RECORDING-VERIFIED-USER-GOAL.json` 是独立验收/完整性证据。
- 严格native task/stream可能仍为failed，反映旧规则/STOP/历史保护错误，不能改写成成功。
- 视频绝对目录：
  `/Users/fanchangxuan/Repos/roomscript2/outputs/oracle/relay-user-completed-20261026/`
  各 `relay-a08-d2-full/global.mp4`、`relay-a08-d3-full/global.mp4`、
  `relay-a12-d2-full/global.mp4`、`relay-a12-d3-full/global.mp4`。
  8L2 6m13s、8L3 6m36s、12L2约7m01s、12L3约17m31s。4x动作、压缩低运动等待，
  保留全部动作区间、失败及修正。12L3首末画面已视觉核对；12L2视频生成解码通过，
  首末画面仍可再查看。不是新物理模拟。当前renderer从report读取50mm，并附精确final event。

## 原16L2，1/4完成

- P4完成，旧严格框约3mm误差已被用户允许，不追修。
- P3已经Goal内、无夹持力、夹爪80mm打开，但仍搭在P手指/偶发手掌上，
  **尚未计入完成，不能只看“open”就宣布落好**。这是最关键资源P的阻塞。
- P1实际D/M，Root约[-.696,1.516,1.294]，已完成
  `r16-p1-DL-next-positive-Mfreed-v1`，M已从P2释放后真实接过P1。
- P2实际G/O（Top），Root约[1.102,1.300,1.196]，最新普通前移
  `r16-p2-GO-open-H-Top-window-v29` 已完成。接下来H54可能已可达；之前在
  Root[.9,1.4]要求H到达失败是位置问题，不能照搬旧结论。P3解除前不要让P接P2。

## 原16L3，3/4完成

- P2/P3/P4均真实松爪、Goal内，最近重新核验仍3/4。P2当前约[1.610,-.434,.871]，
  P3[1.750,-.546,.836]，P4[1.727,-.635,.842]，允许接触叠放。
- 仅P1在运。实际 **H/N**，H Top（Dot约-.999），N Side（Dot约+.095），
  负侧H约32mm站位写法是`.32m`，正侧N约`.58m`。
- 最新 `r16-p1-HN-receiver-height-room-v38` 已完成纯垂直下降到
  Root约[1.203,1.302,1.158]。保持实际全姿态，最低组件仍高于桌面；之前最低约.961，
  下降后约.886，不需要人为要求Root1.2/1.3。
- H原来有已放下P2的过期持物标签，导致choose_team完全不考虑H。
  已用正常typed `move_empty` 当前关节位置的空手保持反馈清除：
  `r16-p2-H-empty-stage-feedback-v56`。物理无接触/无力/有桌面支持的真空手，
  不是直接篡改cache。随后 `r16-p1-GN-to-HN-Hfreed-v35`真实交接完成。
- 当前剩余O Top接手问题：旧N是Side，禁止把`require_topdown=True`错误用于
  保留N的负侧换手。H已是Top，**保留H、O新接手时**可以要求两者Top。
- 最后一次有限搜索（compact前）`r16-p1-HN-to-HO-lowered-Top-v39`：
  私有候选offset [.32,.54]、[.32,.44]、[.32,.32]、[.32,.24]，
  全部未找到候选，**没有提交物理v39请求**。重新读取结果确认。
- 重点检查O endpoint、100/150mm preapproach能否达到以及N forearm/O link2，
  不要只反复扩大搜索。Root纯下降已解决部分preapproach高度问题。
- 旧H/N前移到[1.6,1.15,1.2]并改变yaw2.2在N立即IK失败；保持实际姿态的
  纯下降1.16已通过。N侧姿态/关节分支是局部限制，不是整个机械臂任务死锁。
- 一旦H/O真正都Top，可用已验证过的临近段继续I/O、I/P、J/P。
  杆端会扫过固定底座，提前用有限XY/yaw waypoint避开；不可按索引强行直走。

## 已获用户批准并已经完成的原进程热加载（重要）

用户问“热更新是否暂停”，已如实说明加载会短暂停顿、仿真时钟也暂停，
同一进程继续；随后用户明确：**“如果不是重新开始，而是能够继续，那完全没有问题”**。
这已经授权本次原进程控制逻辑热加载，不要再次当作未获批准。

- 只对原16L2 PID52509加载了**v1**，没有重启/恢复物理状态。
- 冻结源：原16L2 `recovery/controller-extensions/open-contact-withdraw-v1.py`。
- SHA256 `11c63d5f1d3e5f3e011e3a8491013740d2cb1f1f066cc3ea72719f813a57cd80`。
- `open-contact-withdraw-v1-loaded.json` 已确认 installed=true、相同PID/world ID。
- 加载方式：LLDB attach，调度 `Py_AddPendingCall`，回到安全Python边界才运行
  code-only callback，再detach。没有在调试器里发动作，没有写live qpos/物理/monitor。
  **目前没有调试器挂着，进程已恢复正常运行。**
- 可复核 loader：`runs/engineering/relay-descent-20261026/relay_hotload_loader.py`。
  已加载ID不能盲目再次排队；先查原ack与queue-result。缓冲区约2KB，无物理状态含义。
- 新增逻辑仅处理：已张开、无力、Goal有真实地面支持、初始仍接触的短距离撤离。
  private FK不把旧held标签当作物理焊接，保留真实80mm开口；只许已有接触不加深、
  无新穿透、无机器人/桌面碰撞。真实动作仍`t.execute`，原速度/关节/记录约束保留。
- 原非接触 `move_empty` 路径完全不变。实际动作后仍有接触则保留原failed反馈、
  保留标签，**不会假称成功**；真的完全分离后才按原规则清cache。
- 代码：`robofrontier/suite/relay_contact_escape.py`。3个单测 +4 empty recovery +
  14 live recovery测试通过，并用真实保存现场81点检查及mock actuator调度核对。
- 曾准备支持大角度转动的**实验v2**，所有转动候选未通过完整碰撞检查，
  **从未加载或执行**。已归档 `open-contact-withdraw-v2-uninstalled.py/.json`。
  tracked源码现已恢复为与已加载v1相同的SHA。不要把v2当成已生效。

## P3撤离已经真实执行了什么，别重复

目标是原16L2 P3，机器人P，始终force-free/open，现仍有接触。
1. `r16-p3-P-open-contact-withdraw-v16`：约-20mm X/-5mm Z，真实移动，
   物体跟随手指，后置分离未通过。world/PID/历史不变，没有碰撞violations。
2. `r16-p3-P-progressive-open-exit-v17-1..4`：又做了4段短移，均真实进展，
   全部反馈“moved but cargo contact remains”。脚本随后在没有新可用路径时停止。
3. `r16-p3-P-lower-open-exit-v19-1`：Y约-10mm、Z约-25mm，真实移动，仍有接触。
   第2段没有可验证路径，**没有提交**。不要重放任何上述ID。
4. 目前P3 Root约[1.694,-.486,.992]；准确位置以最新observation为准。
   P_tool也已离开最初的位置，旧80mm长直线IK结论不再适用。

失败报告在这里指“移动已完成但仍有接触”，不是仿真结束，也不是物体瞬移。
持续friction支持意味着固定木杆FK预测分离不保证真实木杆不跟随。
下一步应按最新接触面/腕部姿态设计退出（例如张爪换成Top方向，或让桌面接过负载），
别继续重复同一抓取RRT。旧抓取计划把80mm开口缩成65mm、产生固定木杆起点穿透，
这正是之前大量搜索无效的原因。

## 接口/证据/恢复

- Python `.venv/bin/python`；环境
  `PYTHONPATH=.:runs/engineering/relay-descent-20261026 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1`。
- 当前venv没有pytest；相关测试用 `python -m unittest discover -s tests -p 'test_relay_*.py'`，
  不要为测试误装依赖或重建环境。
- `owner_client.send(run,request)`：唯一原世界typed请求；唯一ID，不可重发。
  异常可能发生在真实动作后，先看原result +新observation，不能当作“完全没动”。
- `diagnostic_world(run)` 是只读FK，最多用最新完整motion chunk补weight/hinge，
  不是新物理仿真。不要从它的零qvel/默认force推断真实稳定或持物；用原observation。
- `scripts/check_relay_user_goal.py`独立50mm验收，真实release+ground+qualified transfer，
  不要求速度/静止；保持原strict reports/counters。后来接触会移动已放下物体，需复验。
- `user_goal_lower_open.py`：真实Goal内小步下降、实际支持后正常release/open。
  对16L2 P1、16L3 P2已有效；release失败可能**已经张爪且最终完成**，要另验。
- `original_recovery_lift.py`：仅原custody双实际持物，24mm真实抬升恢复旧接触保护，
  不清历史drop计数，不改flag；原状态若仍grounded不可让foreign接手。
- `finite_top_route.py` /`finite_owner_sequence.py` 只执行明确有限前缀，失败回模型判断。
  `16l3-p1-top-roadmap.json`的v14已执行，v15未发；后续H/N为新实际前沿，
  **不得从这个旧roadmap开头重跑**。
- 查看大JSONL只读尾部或指定ID，别打印全部stream manifest/图片base64。

## 场景几何（用户最终版）

L1完全不变；8/12臂L2/L3初始端点直线跨度1.15m，16臂1.20m；
16臂径向两排距离1.40m。L2圆弧中心角63°；L3折转31.5°、内角148.5°，
弹性关节只作垂直于初始杆面的弯折（横向轴）， nominal±25°、刚度18/阻尼.55。
木条底面、距末端100mm直接铁环连接，无短柱；同款双环+170mm木条+600g砝码。
Goal0.80×1.36m；木条截面45mm。当前R16所有世界已经是这版，不重建。

## 最后检查/对用户沟通

用户很不满意重复计算和等待，应明确承担诊断顺序/控制状态设计的问题，勿归咎于
“任务本来就很难”。下一步以直接动作和可验证物理反馈为主，少做泛泛解释。
保持约60秒内有有意义进展更新。正常权限/用户已授权的热更新不重复询问。
当前用户要compact，两个后台原物理进程继续；恢复模型先读本文件和新原result。
不要给进程发STOP/SIGTERM，不要把未完成任务标成complete/paused。
