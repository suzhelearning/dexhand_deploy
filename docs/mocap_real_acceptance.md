# mocap 真机 50mm 位移验收（动捕实测）

用 Motive 动捕系统实测天机右臂末端刚体（`right_arm`，id=10）的位移，
验收“命令 +x 移动 50mm，真机实际移动 50mm”。验收基于**位移范数在
坐标系旋转下不变**：不需要先标定 Motive↔机器人系，命令位移 50mm ↔
动捕实测位移 50mm 直接对比即可；方向核对与坐标换算需要标定
（见下文“标定”）。

## 数据链路

```
命令（合成台阶 h5，+x 50mm，1:1 目标整形）
  → mocap_h5_replay（主机输入，--hold-arm 等真机桥就绪）
  → tianji_kinematic_sim（IK → 关节命令）
  → marvin_hardware_bridge（真机安全桥，--confirm-real）
  → 天机右臂实际运动
        └─ Motive 动捕：right_arm 刚体（id=10，120Hz）
              → zenohd Router（Ubuntu 7447）
              → track_rigid record（JSONL）
              → track_rigid measure（位移报告，mm）
```

## 前提

- 真机安全检查表全部满足（急停在手边、运动空间清空、FxStation 已关、
  48V 已开等，见 README「真机安全检查表」）；
- `zenohd` Router 运行中（`ss -ltn | grep :7447`）；
- `bash windows_pub.sh` 已运行且 Motive 中 `right_arm`（id=10）
  `tracking_valid=true`（实测 mean_error ~0.05mm）；
- 已构建部署新版本（`build-ik` + `deploy-ik`），`sim_mocap`、
  `real_controller_only` 可用；
- 生成 50mm 台阶轨迹：
  ```bash
  pixi run mocap-step-h5 -- --output /tmp/robot_forward_50mm.h5 \
    --axis z --dir neg --mm 50        # 机器人 chest +x（前）方向 50mm
  ```

## 验收步骤

回放开始/结束由**键盘 's'** 控制（替代 PICO A 键）：回放终端按 s
开始，回放中再按 s 结束并回 Home。

1. **终端 1 — 启动回放主机（保持 idle 等待真机桥）**：
   ```bash
   pixi run sim_mocap -- /tmp/robot_forward_50mm.h5 --topics-only
   ```
   默认 keyboard 控制：回放保持 idle，直到按 s。

2. **终端 2 — 启动动捕记录**（`/home/current/syz/mocap` 目录）：
   ```bash
   pixi run track-rigid -- --names right_arm --output /tmp/right_arm.jsonl
   ```

3. **终端 3 — 启动真机桥（低速首次验收）**：
   ```bash
   pixi run real_controller_only -- --confirm-real \
     --velocity-ratio 20 --acceleration-ratio 20
   ```
   真机桥会校验主机链路（接受 `/mocap_h5_replay` 主机）、回零并进入
   `phase=armed_idle`。**确认 `armed_idle` 后再继续**。

4. **按 s 开始**：回到终端 1 按 **s**，回放进入 teleop，真机桥自动
   跟随（无 A 键，桥直接跟随 `/pico_body/teleop_state`）。机械臂
   前移 50mm、保持；确认到位后按 **s** 结束，回放请求回 Home，
   桥缓慢回零后软停止。

5. **测量**：结束后停止 `track-rigid`（Ctrl-C），报告位移：
   ```bash
   pixi run track-rigid measure /tmp/right_arm.jsonl \
     --start 2 --end 4        # 窗口须覆盖保持段（依实际时间调整）
   ```
   输出 `displacement_mm` 即动捕实测位移，与命令 50mm 对比。

6. **判定**：`displacement_mm ∈ [49, 51]`（±2 mm：动捕噪声 ~0.1mm +
   IK 跟踪 ~1mm + 机械/标定余量）视为通过。记录三层数据：
   命令 50mm → 目标 50.0mm（sim `/pico_body/right_arm_target_pose`）
   → 动捕实测 X mm。

## 安全说明

- 回放主机身份（`mocap_h5_replay`）经 `host_readiness` 显式接受，
  要求 idle 状态、Home 位姿与 preview-only 字段齐全；真机桥的
  回零、命令新鲜度、跟踪误差、软限位、急停等保护**全部不变**；
- 键盘 's' 是唯一的启停手段：先等真机桥 `armed_idle`，再按 s 开始；
  回放中随时按 s 结束回 Home；
- 首次验收务必低速（velocity/acceleration ratio 20），并在动捕
  记录中确认全程 `tracking_valid=true`；
- 结束时真机终端 Ctrl+C 等待 `Robot released`，再关回放终端。

## 标定（Motive ↔ 机器人系，方向核对用）

位移范数验收不需要标定；需要把动捕位移换算到机器人系（方向核对）
时，用 `/home/current/syz/mocap` 的 `calibrate-tcp`：

1. 真机/仿真中让右臂依次停在 K≥4 个不同位姿，每个保持 ≥5 s；
2. 同时记录两路：动捕 `track-rigid record`（right_arm）与机器人侧
   末端位姿。机器人侧来源：
   - 仿真：`/pico_body_sim/right_arm/solved_pose`（FK，right_chest 系）；
   - 真机：反馈关节角（`/right_arm/joint_states`）的 FK（需要
     反馈 FK 工具，后续补充），暂时可用命令关节的 solved_pose
     （误差=跟踪误差，会进入标定残差）。
3. 两路文件转成同构 JSONL（每行 `{t_ns, position, quaternion_xyzw}`），
   运行：
   ```bash
   pixi run calibrate-tcp -- --motive right_arm.jsonl \
     --robot solved.jsonl --pair-count 4
   ```
   输出 T=(R,t)（`p_robot = R @ p_motive + t`）与位置残差（mm）。

标定残差应 < 10mm；残差过大说明窗口对齐或机器人侧参考不准。
