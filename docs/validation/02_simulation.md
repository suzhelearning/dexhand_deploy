# 02 — G1 仿真、采集与回放

## 通用前置与停止

G0 通过；managed router 已在 acquisition 端运行；headless MuJoCo、唯一 coordinator、对应 producer/executor 能启动。H5 case 需要只读 external v4 输入，replay case 需要完整 session-v1 HDF5。先在结果根目录建立独立 case 目录，禁止覆盖已有 `session.h5`。

任何方向/side 错误、physical limit、碰撞风险、feedback stale、tracking threshold、设备/servo error、重复 authority、router ZID 变化或急停均为危险 stop：保持硬件急停，显式发布锁存 stop，等待所有启用 executor matching ack，确认同 tick ack/unhealthy、无新增 motion command、coordinator Home 不能解锁且重启前不能 teleop。仅健康 feedback 下操作者正常结束才允许 bounded return。每项的记录是对应 bundle 全部文件、控制台日志和 operator event；通过要求 `validation-analyze` 成功且 operator 观察与下列判据一致。

## acquisition_live

- **前置设备**：router、acquisition StreamHub、aligned mocap；不连接 Marvin，不使用 robot marker 生成 live target。
- **命令**：`pixi run validation-run -- --case acquisition_live --output ROOT`（先按 G0 启动唯一 router）。
- **步骤**：持续订阅 `mocap/aligned/hands`；采集端 RECORDING/IDLE 都发布 latest-only；触发 timeline reset，再重启一次 acquisition。记录 `stream_instance_id/stream_sequence/router_zid`、valid/null 两侧和关闭顺序。
- **预期**：固定 key、容量 1、不阻塞；invalid side 为 null/不前向填充；timeline reset 不回退 stream sequence；重启后 instance 改变；ACL/唯一 router 保持。
- **立即停止**：旧值冒充 valid、NaN、sequence 回退、router ZID 改变、第二 router 或发布阻塞。
- **通过**：两次生命周期和 reset 数据在 H5/status 中可追溯，实例/sequence 单调且 analyze 无 schema/hash 错误。

## pico_sim

- **前置设备**：router、headless MuJoCo；PICO 可用则使用真实 controller，否则仅可做 fake/preflight smoke。
- **命令**：`pixi run validation-run -- --case pico_sim --output ROOT`；无设备时 `... --fake --headless`，结果必须是 `aborted`。
- **步骤**：先 subscriber/query snapshot，再确认 idle/Home；操作者按 controller A（若使用真实 PICO）启动，观察双臂 canonical target → IK proposal → coordinator command → MuJoCo state；随后正常 return。
- **预期**：A 仅产生 intent；匹配 teleop state 后才出 target；左右 command 同 tick；inactive side 保持 Home；return 后 `at_home/return_complete` 闭环；headless executor 立即 ready。
- **立即停止**：A 在拒绝/断流后残留 teleop、frame/side 错误、proposal 越限/回退、headless 未 ack stop、重启前可再次 teleop。
- **通过**：status、target/proposal/command/state、Home 和 latch 的时序可分析，且未产生自动危险按键。

## mocap_live_sim

- **前置设备**：`acquisition_live` pass、aligned mocap、headless MuJoCo；robot marker 只允许 H5/diagnostics。
- **命令**：`pixi run validation-run -- --case mocap_live_sim --output ROOT`。
- **步骤**：订阅 aligned hands，确认 watchdog；按 `s` 冻结 valid wrist/reference 并由操作者发 start；观察 `mocap_to_robot → Base/TCP target`；让单侧 invalid/stale，再模拟 acquisition instance 变化，最后新 start。
- **预期**：0.5 s watchdog；instance 变化立即 return、清 reference、armed；root invalid 不误杀另一侧；live target 只来自 aligned wrist/keypoints；return 等待 fresh state。
- **立即停止**：robot marker 改变 live target、stale 仍运动、另一侧被误杀、旧 instance 恢复 target、router/authority 变化。
- **通过**：aligned stream、target、return/armed/new-start 的 status 和 source/instance 可关联，analyze 完整。

## h5_sim

- **前置设备**：只读 H5 v4、headless MuJoCo；确认 marker/安装外参，若有 `wuji2_joints` 预先标记 direct，否则 retarget。
- **命令**：`pixi run validation-run -- --case h5_sim --output ROOT --input INPUT.h5`。
- **步骤**：Enter deadman 必须 released；`s` 仅冻结 marker/reference 并发 intent；匹配 teleop 后观察 frame0 approaching 与独立 target sequence；再 Enter replay、`r` return/completed。
- **预期**：approach 前无 target；solved pose 必须同 target_sequence 且 fresh 才累计 ready；左 inactive arm 维持 Home；hand target 是减 wrist 的 relative keypoints；speed/yaw 在构造后不可在线改。
- **立即停止**：Enter 自动运动、旧 solved 解锁、world keypoints 上 wire、frame/side 错误、real 参数混入 sim、异常 recorder teardown。
- **通过**：H5 source/target/optional hand path、phase、frame0 overlay、return/latch 均可追溯且分析器不放宽配置。

## ik_pinocchio_cpp / ik_pinocchio_qp / ik_tianji_official

- **前置设备**：`pico_sim` pass、对应 backend 已 build/deploy；三 case 分开运行，禁止切换中复用 instance。
- **命令**：分别运行 `pixi run validation-run -- --case ik_pinocchio_cpp --output ROOT`、`... ik_pinocchio_qp ...`、`... ik_tianji_official ...`。
- **步骤**：启动指定 backend；检查 target frame `Base_L/Base_R`、proposal names/order、proposal/solved `target_sequence`；运动左右臂，停止 backend 再观察 bounded Home 和 idle。
- **预期**：producer 只订阅 canonical target/final command，solver reject 不发布 accepted 占位；strict names、finite、step/limits；inactive side Home；三 backend 都完成 return_complete。
- **立即停止**：调用 solver 前接受非法 frame/sequence、proposal 越限/回退、出现任何旧 IK authority、双 producer、同 logical ID 多 instance 未 fault。
- **通过**：每个 backend 单独 bundle 中 proposal/solved/command/state 时序和 backend identity 一致，分析 rate/drop/error 可读。

## target_replay_sim

- **前置设备**：session-v1 HDF5 有 active arm/hand 流，缺 side 必须由 profile 显式 inactive；headless MuJoCo/IK/Wuji retarget。
- **命令**：`pixi run validation-run -- --case target_replay_sim --output ROOT --input SESSION.h5`；不得加 `--record`。
- **步骤**：启动 target replay source；start 后 pause，观察 recorded clock/frame 不推进而 wire timestamp/sequence 持续新鲜；resume，结束后等待 Home/idle。
- **预期**：只有一个 source role；hand target 走 retarget；pause 不冻结 freshness；结束发 return，Home 与 hand zero latch 闭合。
- **立即停止**：replay profile 录制、缺 side 静默忽略、pause 时 wire stale、replay source 直接 final command、未 ack stop。
- **通过**：recorded/source 与 live sequence 可区分、pause/resume 时序正确、analysis 通过。

## joint_replay_sim

- **前置设备**：session-v1 HDF5 的 arm+20-joint hand command/state、headless MuJoCo/Wuji direct；不启动 IK。
- **命令**：`pixi run validation-run -- --case joint_replay_sim --output ROOT --input SESSION.h5`；不得加 `--record`。
- **步骤**：确认 source 和 producer role 是同一进程的独立 token；pause/resume；观察 direct hand executor names/order、zero、return。
- **预期**：producer 只输出 proposal/hand command，coordinator 仍唯一 final authority；pause 保持 recorded frame 但持续刷新 wire；direct command 不被 Wuji 重发；return zero/at_zero。
- **立即停止**：启动 IK、第二 hand command publisher、pinky alias 上 wire、pause stale、越限或 rollback。
- **通过**：arm/hand command/state、role token、pause/resume、return/latch 可分析，`--record` 明确 exit 2。

## policy_hold_sim

- **前置设备**：`pico_sim` pass、policy runner；仅 hold runner，不连接 solver API。
- **命令**：`pixi run validation-run -- --case policy_hold_sim --output ROOT`。
- **步骤**：确认 fresh arm state；运行 hold；注入 velocity 缺失/过 stale、shape/nonfinite action，再恢复并正常 return。
- **预期**：Hold 输出当前 position；velocity 用有限差分且超阈值 not ready；非法 action 令 producer unhealthy、停 proposal、coordinator controlled return；finite 越限 wire proposal 才 fault；policy 不发布 final command。
- **立即停止**：无 fresh state 仍 ready、adapter 吞掉 malformed wire、policy 订阅/调用 ArmIkSolver、fault 可自动解锁。
- **通过**：observation/action/proposal/status/fault 分流有证据，analyze 不把 rejected action 当成功运动。
