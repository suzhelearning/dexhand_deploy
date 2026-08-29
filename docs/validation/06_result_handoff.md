# 06 — 结果交付与失败门

## 前置与命令

每个 case 必须在 G0、matrix prerequisite 通过后执行；real 还要 `--confirm-real --robot-ip`。运行结束后立即执行：

```bash
pixi run validation-analyze -- ROOT
```

不要覆盖已有 bundle 或手工编辑 `session.h5`。操作者事件只能通过 `validation-run --operator-event EVENT=DETAIL` 写入单调 `time_ns`。

## 步骤

1. 检查 bundle 是否包含 `manifest.yaml`、`session.h5`、`status.jsonl`、`operator_events.jsonl`、`logs/<component>.log`、`operator_result.yaml`、`checksums.sha256`；正常 analyzer 会在同一目录生成 `analysis.json` 与 `analysis.md`。
2. 核对 manifest：两个仓库 commit/dirty、runtime/config/ACL hash、router endpoint/ZID、publisher instance、机器/OS、case/profile、robot IP/model、hand side/mode、Motive rigid ID、H5 SHA256、IK backend、比例、UTC 起止、exit reason。
3. 核对 status/protocol：schema、source/target/proposal/command/state rate/drop、instance/router、fault/soft-stop 原因；危险 stop 必须每个启用 executor matching ack、同 tick ack/unhealthy、无新增 motion command、lockout。
4. 核对 HDF5 session-v1：`complete=true`、nullable source time 无哨兵损失、names/order、rad 单位、raw/target/joint/state 流；异常中断保留 `complete=false` 且默认 analyzer 拒绝。
5. 核对 analysis：target→solved error、joint step/velocity、saturation/rejection、command→feedback tracking、Home/hand-zero time、fault/soft-stop 和全部 operator events。判据只能来自本次 robot/coordinator/executor config，禁止放宽 limit/timeout/error。
6. 填 `operator_result.yaml`，`outcome` 只能 `pass|fail|aborted`；记录 `emergency_stop`、`abnormal_direction`、`jitter`、`noise`、`collision_risk`、`notes`。fake/headless、无设备、ack timeout、未完成实体步骤必须 `aborted` 或 `fail`，不能写 pass。
7. 重新生成并核对 SHA256（任何修改 operator result、manifest、日志后都要用工具重新生成 bundle，而非手工改 checksum）；将整个目录打包，保留命令、stdout/stderr 和现场事件。

## 立即停止与拒绝交付

checksum、schema、status、ack、config/runtime/ACL hash 任一不一致；缺 log/event/H5；router ZID 不一致；发现 recorder teardown/headless config/calibration instance/overlay 第二 authority 或 Marvin reconnect race 未有明确安全证据；出现 physical limit、方向错误、碰撞、feedback stale、tracking、servo/device error、重复 authority、急停，均拒绝交付为 pass。危险 stop 未 ack 时提示保持/按下物理急停，不得先 return/自动 clear。

## 通过判据与交付包

一个 case 只有在 `validation-analyze` exit 0、checksums/schema/config 一致、operator result 为 pass、实体动作与 status/HDF5 时序一致、所有前置 case 最新代码/config hash 且为 pass 时才能进入下一个 gate。交付包至少包含：

- case bundle（含 `analysis.json`/`analysis.md`）；
- 完整运行命令和 endpoint/ZID（不交付凭据）；
- 两仓库 commit/dirty、runtime/config/ACL hash；
- 设备/SDK/firmware、robot IP/model、hand mode/side、Motive/H5 SHA256；
- 异常、急停、方向、抖动、噪声、碰撞风险与操作者签名备注。

失败 case 的依赖 gate 全部暂停，先定位 acquisition/source/protocol/producer/coordinator/executor/config 的时序证据，修复后重新跑自动测试和受影响 case。禁止通过扩大 physical limits、关闭 freshness、吞 fault、降低 readiness 或伪造物理结果“通过”。
