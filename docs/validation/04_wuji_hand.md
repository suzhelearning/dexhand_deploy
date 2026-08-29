# 04 — Wuji Hand retarget/direct

## 通用前置

G0 通过；Wuji 型号、firmware、zero position/tolerance、20-joint names/order 与 URDF/firmware 序一致；硬件急停可触及。retarget 和 direct 是互斥 profile，不得同时有两个 hand command publisher。先 dry，再 real；real 仅显式 `--confirm-real`，比例固定 0.1/0.1。

危险 stop（方向/side 错、physical limit、碰撞风险、feedback stale、tracking threshold、device/servo error、重复 authority、router ZID 变化、急停）必须保持硬件急停，发锁存 `tianji/safety/stop`，等待每个 hand/arm executor matching ack；确认同 tick ack/unhealthy、SDK 无新 command、coordinator Home 不能解锁，重启前不能 teleop。正常结束仅 fresh/healthy feedback 允许 bounded return。

## wuji_retarget_dry

- **前置设备**：`h5_sim` pass、H5 keypoints、headless/realistic Wuji dry、router；H5 有无 `wuji2_joints` 必须先确认 profile 选择。
- **命令**：`pixi run validation-run -- --case wuji_retarget_dry --output ROOT --input INPUT.h5`（retarget 输入不能携带 `wuji2_joints` direct path）。
- **步骤**：检查 `HandTargetCommand.frame_id=wrist_relative_mediapipe` 与 wrist=0；在 session teleop 后输入新鲜 keypoints；平移 world 手但不改变相对点；让输入 stale/invalid；return。
- **预期**：retarget 唯一发布 HandJointCommand；手部 world 平移不改变输出；invalid/stale 不刷新 watchdog；20-joint names/limits 严格；returning/fault 拒绝新 target，slew 回零，`at_zero=true && tracking_allowed=false`。
- **立即停止**：第二 hand publisher、world frame 上 wire、越限进入 SDK、stale 仍 tracking、pinky alias 上 wire、zero 未确认却 ready。
- **记录/通过**：target/command/state、instance/mode、zero/limits/watchdog、stop ack、operator events；dry 只能验证行为，不写成 physical pass。

## wuji_retarget_real

- **前置设备**：`wuji_retarget_dry` 和 `h5_sim` pass、Wuji real、H5 preflight、router；speed/yaw real admission 合规。
- **命令**：`pixi run validation-run -- --case wuji_retarget_real --output ROOT --confirm-real --robot-ip ROBOT_IP --input INPUT.h5`。
- **步骤**：连接前确认 Wuji at_zero、tracking false；进入 teleop 后从 10% 小幅 keypoints 运动；检查 watchdog 仅被授权 source/producer 新鲜输入刷新；触发 return/fault；等待 zero。
- **预期**：exactly-one producer token（`wuji_retarget_<side>`）；strict side/names/sequence/limits；returning/fault 立即拒绝输入并回零；zero tolerance 内才 return complete；Marvin/hand recorder teardown 正常。
- **立即停止**：错误方向/side、限位/碰撞、stale 输入刷新 watchdog、instance/mode 运行时切换、servo/device/tracking error、zero 失败。
- **记录/通过**：完整 bundle、Wuji SDK/servo log、hand target/command/state、zero 时间、ack 和 operator result；必须由实体反馈稳定且 analysis 通过才 pass。

## wuji_direct_real

- **前置设备**：`joint_replay_sim` pass、包含 20-joint direct command、Wuji real、router；不启动 IK，不启用 retarget publisher。
- **命令**：`pixi run validation-run -- --case wuji_direct_real --output ROOT --confirm-real --robot-ip ROBOT_IP --input SESSION.h5`。
- **步骤**：检查 `joint_replay` 是唯一 hand command publisher，Wuji 仅 executor；验证每个 direct command names/order/strict sequence；pause 时 wire 继续刷新但 recorded frame 不推进；return zero。
- **预期**：direct command 不被 executor 重发；未知 publisher/rollback/运行中切换停止 tracking、回零、unhealthy；zero/limits/watchdog 状态可追踪。
- **立即停止**：IK 被启动、第二 publisher、rollback 被接受、越限发 SDK、pause stale、zero/feedback/servo error、重启前恢复 teleop。
- **记录/通过**：role token、direct command/state、SDK log、zero/stop ack、analysis；满足所有 strict contract 且实体反馈安全后 pass。
