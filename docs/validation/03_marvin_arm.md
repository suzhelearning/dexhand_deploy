# 03 — Marvin 双臂真机（10%）

## 安全前置

G0/G1 中对应 source 仿真 case、IK/policy contract、`marvin` readiness 通过；Marvin 型号/IP、SDK/firmware、物理急停、工作区和 Home 已确认。必须由安全员在现场；首次仅允许 matrix 固定的 `velocity_ratio=0.1`、`acceleration_ratio=0.1`，不得凭感觉改 config。每一条命令都显式 `--confirm-real --robot-ip`，不能 `--fake`。

连接前要求 source ready/healthy/real-capable、arm producer loaded/healthy、coordinator idle且 final command at Home；不要求 policy observation ready。连接后必须观察 fresh 14-joint state/ready，再由操作者发 start。Marvin reconnect race：returning/fault 中断只允许 bounded Home 的 `fault_return`，完成 Home 仍保持 fault，不得重新 teleop；记录该分支。

## marvin_pico_real_10pct

- **前置设备**：`pico_sim`、IK/backend pass；PICO controller、Marvin 双臂、router。
- **命令**：`pixi run validation-run -- --case marvin_pico_real_10pct --output ROOT --confirm-real --robot-ip ROBOT_IP --robot-model MODEL`。
- **步骤**：确认 manifest 比例为 0.1；检查 source/producer/coordinator/executor 唯一 instance 与 router ZID；先连接并 MoveHome，确认 fresh feedback；操作者按 A 后小范围移动左右 controller，观察 command→feedback，再 release/return。
- **预期**：rad wire 仅在 Marvin SDK 边界转 degree；双臂 names/side 正确；feedback fresh、tracking 在阈值内；bounded return 到 Home，`return_complete` 后 idle。SDK 发送日志无额外运动。
- **立即停止**：方向/side 错、feedback stale/tracking error、限位/碰撞、servo/device error、router/authority变化、reconnect 后可 teleop、recorder teardown 未完成。
- **记录/通过**：bundle、SDK/servo log、Home/feedback 状态、operator events；analysis 的 step/velocity/tracking/fault 均不超当前 config，操作者确认无异常后 outcome 才可写 pass。

## marvin_mocap_live_real_10pct

- **前置设备**：`mocap_live_sim`/`acquisition_live` pass、aligned mocap、Marvin、router；robot marker 不得作为 live source。
- **命令**：`pixi run validation-run -- --case marvin_mocap_live_real_10pct --output ROOT --confirm-real --robot-ip ROBOT_IP --robot-model MODEL`。
- **步骤**：确认 aligned wrist 连续且 instance 稳定；连接 MoveHome；按 `s` 冻结 valid reference、再发 start；先单侧小范围，再双侧；拔掉/模拟 acquisition stream 后观察 immediate return/armed，再用新 start 恢复。
- **预期**：0.5 s watchdog；stale/invalid 不继续运动；instance 改变清 reference 并 return；coordinator 每 tick 检查 real capability；Home/feedback/return latch 完整。
- **立即停止**：marker 影响 live target、另一侧 valid 被误杀、stream 旧值复活、tracking/feedback/stale、异常方向/碰撞风险、reconnect race。
- **记录/通过**：aligned/raw/target/command/state、stream instance/sequence、SDK feedback、status 和事件；只有故障分支与 10% 动作均满足矩阵判据才 pass。

## marvin_h5_real_10pct

- **前置设备**：`h5_sim` pass、有效 H5 输入和 SHA256、marker/安装外参、Marvin；hand preflight 按输入自动固定 retarget/direct。
- **命令**：`pixi run validation-run -- --case marvin_h5_real_10pct --output ROOT --confirm-real --robot-ip ROBOT_IP --robot-model MODEL --input INPUT.h5`。
- **步骤**：检查 speed≤0.25、yaw=0、deadman pre-start released；连接 Home；`s` 仅发 start_pending；观察 frame0 solved 关联后 approach，再低幅 replay；`r`/completed 触发 return。
- **预期**：real capability 持续包含 speed/yaw/preflight 条件；左 arm 保持 Home；每个 frame0 target 有独立 sequence 且旧 solved 不解锁；rad→degree 仅 SDK 边界；回 Home 后完成 latch。
- **立即停止**：speed/yaw 不合规仍连 real、Enter 自动运动、world keypoints 上 wire、solved sequence 不匹配、feedback stale/tracking/limit/碰撞、Marvin reconnect race。
- **记录/通过**：输入 SHA256、H5/raw/target/proposal/command/state、SDK/feedback、phase、Home/hand status、事件；analyze 严格使用本 bundle 配置，操作者实体动作和日志一致后才能 pass。

## 提速规则

每次提速前必须有当前比例的最新 `operator_result=pass`、`validation-analyze` 成功、无未解释 saturation/rejection/fault/tracking error，且安全员重新确认工作区与急停。任何失败回到上一安全比例或停止，不得直接提高参数，不得通过扩大 limits/timeout 或关闭 freshness 通过。
