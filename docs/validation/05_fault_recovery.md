# 05 — 故障与危险停止

## 通用安全规则

安全员在现场，先 sim 后 real。工具只在操作者显式要求时发布 stop，不自动按键或运动。危险原因固定为：`wrong_direction_or_side`、`physical_limit`、`collision_risk`、`feedback_stale`、`tracking_threshold`、`device_or_servo_error`、`duplicate_authority`、`router_zid_change`、`emergency_stop`。

危险流程：立即保持/按下物理急停（急停本身必须保持）；`run_case --danger-stop REASON` 发布锁存 `SafetyStopRequest`；等待所有启用 executor 的 matching `SafetyStopAck`。ack timeout 时继续保持物理急停并记录 timeout，不得先 return。必须观察同一 control tick ack/unhealthy、无新增 SDK motion command、coordinator bounded Home 不能解锁；清除只允许人工确认后重启 executor/session，不能由 return/shutdown 自动 reset。

受控流程仅适用于操作者正常结束且 feedback fresh/healthy：发 return，等待 bounded Home、fresh state 和 hand zero，再完成 `return_complete`。任何 fault/soft-stop 混同、自动 clear 或 restart 前 teleop 都是失败。

## fault_recovery_sim

- **前置设备**：`pico_sim` pass、headless MuJoCo、IK/coordinator、可控故障注入器；准备独立 output ROOT。
- **命令**：`pixi run validation-run -- --case fault_recovery_sim --output ROOT`；危险验证在明确操作员指令下加 `--danger-stop collision_risk`。
- **步骤**：分别注入 source 断流/deadman release、arm producer unhealthy、executor state stale、malformed/越限/rollback proposal、重复 authority、router 重连；每次恢复都先观察 returning/fault 和 bounded Home。对危险 stop 记录 request/每侧 ack、同 tick command 计数、lockout；重启 executor 后检查仍不能 teleop，重启 session 后才可重新 start。
- **预期**：source/producer 故障走 controlled returning；executor/liveliness/state malformed 和重复 authority 锁存 fault；fault 持续 bounded Home；safety ack matching 且 executor unhealthy；coordinator command 不解锁。
- **立即停止**：危险 stop ack 缺失、同 tick 后仍 SDK 发 motion、Home 解锁、自动 clear、重启前 teleop、router ZID 变化未 fault。
- **记录/通过**：完整 bundle、fault injection 时间线、status/ack、command/state、executor logs、operator events；每条分支与预期严格对应，analysis 通过后 case 才 pass。

## fault_recovery_real

- **前置设备**：`fault_recovery_sim` pass、`marvin_pico_real_10pct` pass、Marvin/Wuji（若启用）、安全员、硬件急停、router；显式 `--confirm-real` 和 robot IP。
- **命令**：`pixi run validation-run -- --case fault_recovery_real --output ROOT --confirm-real --robot-ip ROBOT_IP`。
- **步骤**：从 10% 连接 Home；逐项按安全员指令模拟 source/producer/executor 断流、deadman、重复 authority、router 重连；对 feedback stale、tracking threshold、servo/device error 立即物理停并用 `--danger-stop`；等待所有 matching ack 后保持急停；记录 Marvin reconnect race 的实际结果，任何 fault reconnect 只能 fault_return 到 Home，仍保持 fault。
- **预期**：真实 SDK 在 stop 同 tick 后没有新增运动命令，soft-stop/servo-disable；coordinator 不能以 Home command 解锁；重启 executor/session 前不能 teleop；正常健康 return 与危险 stop 明确分支；Wuji hand 必须 zero/at_zero/tracking false。
- **立即停止**：方向/side 错、碰撞风险、physical limit、feedback stale、tracking/servo/device error、ack timeout、router/token/authority变化、SDK继续发送、reconnect 后错误恢复 teleop。
- **记录/通过**：manifest 设备/IP/model、SDK/servo logs、status/ack、liveliness、protocol、session HDF5、operator events/result、checksums/analysis；所有危险分支 ack/no-motion/restart-lockout 和实体反馈均通过，才可写 pass。没有物理结果只能 aborted，绝不伪造。

## 各类注入后的处理表

| 注入 | 期望 authority 状态 | 是否允许自动 return | 恢复条件 |
|---|---|---|---|
| source/producer 正常断流 | returning | 允许 bounded return | fresh source/producer、Home 后新 intent |
| executor/liveliness/state stale | fault | 只允许受控 bounded Home | 安全重连、fresh state、人工确认 |
| malformed/越限/rollback/重复 authority | fault | 不以 return 清除 fault | 修复后重启 executor/session |
| router ZID 变化/急停/碰撞 | safety lock + fault | 禁止先 return | 物理确认、matching ack、重启 |
