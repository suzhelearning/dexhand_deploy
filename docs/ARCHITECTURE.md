# Tianji Teleop Architecture

## 五层 authority

1. **Source**：aligned mocap、H5 或 session replay 只发布 target/raw 和
   `SessionIntent`，等待 coordinator 的匹配 `teleop` state。
2. **Producer**：IK、policy 或 direct replay 接受 target/current command，发布
   有限的 arm proposal、solved pose 或 hand command；不发布 final arm command。
3. **Coordinator**：唯一发布 `SessionState`、`LatchedBool` 和双臂
   `ArmJointCommand`，每个 tick 同序列发送两侧 command，inactive side 保持 Home。
4. **Executor**：MuJoCo/Marvin/Wuji 只接受授权 identity、正确 sequence、names、
   frame 和 limits，持续发布 typed state/status。
5. **Recorder/Diagnostics**：被动记录 session v1，诊断只观察权威 state/status 或
   发送 intent，不成为 authority。

## Router 与身份

唯一 endpoint 是环境变量 `TIANJI_ROUTER_ENDPOINT`，默认显式为
`tcp/127.0.0.1:7447`。所有 client 使用 `session.info.routers_zid()`，router
必须 exactly one；组件携带 launcher 注入的 `router_zid`，不一致 fail closed。

liveliness 使用完整 token，不折叠 instance：

```text
tj/live/source/<logical_id>/<instance_id>
tj/live/producer/{arm|hand}/<logical_id>/<instance_id>
tj/live/coordinator/arm/<logical_id>/<instance_id>
tj/live/executor/{arm|hand}/<logical_id>/<instance_id>
tj/live/recorder/<logical_id>/<instance_id>
```

## 数据契约

Arm frame 只允许 left/`Base_L` 与 right/`Base_R`，姿态为 Base 到 TCP 的
transform；quaternion 与 elbow direction 必须 finite。Hand target 上 wire 前先
减 0 号 wrist，frame 固定 `wrist_relative_mediapipe`。关节统一 rad，20-joint
hand names 只允许 canonical `l_|r_` 前缀顺序。

## 配置与运行

`config/robot` 是 names/Home/limits 唯一 authority；session YAML 只引用
component YAML、capability、active/inactive sides 和 hand mode，IK backend 只在
`config/producers/ik.yaml`。`run_session.sh` 启动顺序为 recorder、coordinator、
executor、producer、source；real profile 需要 `--confirm-real`，H5 preflight
固定 direct/retarget。任一步失败都按反序停止受管 process group 并释放 guard。

## 记录与诊断

session HDF5 v1 的 complete、nullable source time、publisher instance 和
chunked append contract 由 `recording/session_h5.py` 管理。replay 重新生成 live
sequence/timestamp，暂停只冻结 recorded clock；replay profile 禁止 `--record`。
标定轨迹、trace metrics、real readiness 和 H5 frame0 overlay 位于 diagnostics，
其中 frame0/robot marker 仅用于 H5 或诊断，live target 只来自 aligned hand wrist。
