# NoSlip 配置与实际 RGB 抓取验证

2026-10-19。用户要求启用 NoSlip 并观看不再滑落的视频后，新增 `configs/noslip.json`（profile: `noslip-10-v1`）。应用启动默认采用此配置；原 `config.json` 保留为 NoSlip=0 的基线。

## 配置变化

只新增 `noslip_iterations=10` 和 profile 标签。摩擦、方块质量、夹爪 PD 参数及力上限、机械臂控制速度、重力均与原配置相同。NoSlip 是接触求解后处理，没有增加吸附、weld 或物体位姿覆盖。

## 参考程序验证

```sh
uv run python scripts/verify_demo.py --config configs/noslip.json --hold-seconds 30
```

同一初态连续 3 次双指抓取：最终抬升 11.99 cm；每次额外保持 30 s，期间位移约 0.051 mm。空闲稳定性、重置、多关节输入、松键和模式切换检查通过。该结果使用固定坐标参考程序，只证明物理控制闭环，不是 RGB 策略成绩。

## 实际 RGB 策略操作

重新启动并随机布置方块：

```sh
uv run robofrontier --config configs/noslip.json --bridge --blind --record --camera side
```

新的布局未通过入口返回，也没有读取它的内部真值日志或初始化文件来决定动作。使用正面与侧面 RGB，以及机械臂本体反馈。策略在对话中逐步选择动作，通过同一 IK 与键盘控制器执行；思考期间物理仍运行。

- 机器人动作 14 条，闭合尝试 1 次。
- 闭合后 TCP 高度 0.4250 m；最后抬升结束 TCP 高度 0.5216 m，增加 9.66 cm。
- RGB 画面显示蓝色方块随夹爪上升并保持，不是仅有机械臂上升。
- 最后抬升结束为 sim time 219.27 s；稳定性截图为 257.26 s，相隔 37.99 s，方块仍在夹爪中。
- 没有通过物体真值测量本轮精确抬升高度；9.66 cm 指的是机器人本体 TCP 高度变化。
- 原速录像继续录至约 sim time 290 s，包含闭合、抬升和完整保持过程。相机视角可切换，不影响物理。

这是一个成功案例，不是多布局总体成功率，也不保证任意抓取姿态都会成功。

## 产物

episode：`runs/20261019-222926-883522/`。

- `success-realtime.mp4`：从完整录像第 150 秒截取到末尾，原速 140.27 s。起始画面仍是夹爪张开，随后闭合、抬起、保持；没有拼接或加速。
- `recording-01.mp4`：完整原速录像 290.27 s。
- `policy.jsonl`：策略实际可见的请求、返回状态和 RGB 路径，与内部真值日志分开。
- `public-summary.json`：只依据 `policy.jsonl` 的机器人自身状态及动作统计生成。
- `rgb-0015-*.png`、`rgb-0016-*.png`：保持开始和约 38 s 后的两路 RGB。

## 使用

现在双击 `Launch Demo.command` 或运行 `uv run robofrontier`，默认都使用 NoSlip 配置。需要原基线时显式传 `--config config.json`。比较 Human 与 GPT 时必须使用同一配置，旧的失败记录不应与新配置成绩混算。
