# 天机遥操作设备验收

本目录是 `config/validation/test_matrix.yaml` 的操作者说明。验收工具只记录事实，**不会自动按键、不会自动运动、不会把仿真或 fake 结果标为 pass**。没有物理设备时可以执行 `--fake --headless` 验证 bundle、协议和安全记录链路，但 `operator_result.yaml` 必须保持 `aborted`，不能作为设备验收依据。

## 阶段门

1. **G0 代码与配置门**：`pixi run validation-run -- --list`、`pixi run doctor`、runtime/config/ACL hash 与两个仓库 dirty 状态记录完整。
2. **G1 采集与仿真门**：依次完成 `acquisition_live`、`mocap_live_sim`、`h5_sim`、三个 IK、target/joint replay、policy hold。每个 case 的 bundle 由 `validation-analyze` 校验通过；失败 case 不得继续后续 gate。
3. **G2 Marvin 真机门**：G1 全部依赖通过，且每一条 real case 显式 `--confirm-real`。从 10% velocity/acceleration 开始，只有 tracking、feedback、Home 都稳定才允许按 runbook 提升比例。
4. **G3 Wuji 门**：retarget dry → retarget real → direct real；hand zero、限位、watchdog 和模式互斥全部通过。
5. **G4 故障门**：先 `fault_recovery_sim`，再 `fault_recovery_real`。危险停止没有所有 executor matching ack、出现 router ZID 变化、重复 authority 或急停未保持时，立即停止，不得进入下一 gate。

## 设备矩阵与统一命令

| Case | 必需设备/前置 | 能力 | hand | 比例 |
|---|---|---|---|---|
| `acquisition_live` | router、acquisition、aligned mocap | simulation | disabled | 1.0/1.0 |
| `mocap_live_sim` | acquisition、aligned mocap、headless MuJoCo | simulation | disabled | 1.0/1.0 |
| `h5_sim` | H5、headless MuJoCo | simulation | auto | 1.0/1.0 |
| `ik_pinocchio_cpp`, `ik_pinocchio_qp`, `ik_tianji_official` | G1 `mocap_live_sim`、相应 IK backend | simulation | disabled | 1.0/1.0 |
| `target_replay_sim` | session-v1 HDF5、headless MuJoCo | simulation | auto | 1.0/1.0 |
| `joint_replay_sim` | session-v1 HDF5、headless MuJoCo | simulation | direct | 1.0/1.0 |
| `policy_hold_sim` | G1 `mocap_live_sim`、policy runner | simulation | disabled | 1.0/1.0 |
| `marvin_{mocap_live,h5}_real_10pct` | 对应 G1、Marvin、真实输入 | real | 按 profile | 0.1/0.1 |
| `wuji_retarget_dry` | `h5_sim`、Wuji dry | simulation | retarget | 0.1/0.1 |
| `wuji_retarget_real` | dry + H5 sim、Wuji real | real | retarget | 0.1/0.1 |
| `wuji_direct_real` | `joint_replay_sim`、Wuji real | real | direct | 0.1/0.1 |
| `fault_recovery_sim` | `mocap_live_sim`、故障注入器 | simulation | disabled | 0.1/0.1 |
| `fault_recovery_real` | `fault_recovery_sim`、Marvin real | real | disabled | 0.1/0.1 |

先启动 `/home/current/syz/mocap/acquisition` 的唯一 router，并让两个仓库使用同一 `TIANJI_ROUTER_ENDPOINT`。统一运行形式：

```bash
pixi run validation-run -- --case CASE_ID --output ROOT [--input INPUT]
# 仅本地 schema/preflight smoke，不是验收：
pixi run validation-run -- --case mocap_live_sim --output /tmp/tianji-validation --fake --headless
pixi run validation-analyze -- /tmp/tianji-validation
```

real case 还必须提供 `--confirm-real --robot-ip IP`，并由工具检查 matrix 中所有 prerequisite bundle 的 `operator_result.outcome == pass`。禁止使用 `--fake` 代替 real。工具不会替操作者发 `A`、`s`、Enter、`r` 或任何危险运动命令。

## Bundle 与结果交付

每次运行创建 `ROOT/<UTC>_<case_id>_<nonce>/`：`manifest.yaml`、`session.h5`、`status.jsonl`、`operator_events.jsonl`、`logs/<component>.log`、`operator_result.yaml`、`checksums.sha256`。manifest 保存两个仓库 commit/dirty、runtime/config/ACL hash、router endpoint/ZID、所有 publisher instance、机器、robot/hand、Motive rigid ID、H5 SHA256、IK backend、比例、起止时间和退出原因。操作者只使用 run_case 写事件，例如：

```bash
pixi run validation-run -- --case mocap_live_sim --output ROOT --fake --headless \
  --operator-event started='operator acknowledged preflight' \
  --operator-event note='observed stable Home'
```

事件时间由工具写入 `time.monotonic_ns()`，不得手工修改 HDF5。危险停止只能由操作者显式执行：

```bash
pixi run validation-run -- --case CASE_ID --output ROOT --confirm-real \
  --robot-ip IP --danger-stop collision_risk
```

危险 stop 必须列出每个启用 executor 的 matching ack；缺 ack 或 ack timeout 时保持/按下物理急停，不得先 return、不得自动 clear。`operator_result.yaml` 只允许 `pass|fail|aborted`，并填写急停、异常方向、抖动、噪声、碰撞风险和备注。先运行 `validation-analyze`，再把整个 case 目录（含 `analysis.json`、`analysis.md`）和执行命令交付，不得只交截图或手工摘要。

## 立即停止总则

方向/side 错误、physical limit、碰撞风险、feedback stale、tracking threshold、设备/servo error、重复 authority、router ZID 变化、急停均为危险停止：保持硬件急停，发布锁存 `tianji/safety/stop`，等待所有 matching `tianji/safety/ack/{executor_id}`，确认同一 control tick 内 ack/unhealthy、无新增 SDK 运动命令、coordinator Home command 不能解锁，重启前不能再次 teleop。正常结束仅在 feedback fresh/healthy 时允许 bounded return。Marvin reconnect race、recorder teardown/headless config、calibration instance、overlay 第二 authority 均在相应 case 观察并记录；发现未决行为按失败处理。
