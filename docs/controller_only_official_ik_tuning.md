# 纯手柄天机官方 IK：结构、调参与回放

本模式不是把手柄目标直接交给 `libKine`。为避免工作空间边界冻结、
七自由度换支和真机目标积压，链路分为四层：

```text
PICO 相对位姿
→ TargetConditioner（尺度、软工作空间、速度/加速度）
→ TianjiOfficialArmIk（ZSP、多候选、软限位、可达边界）
→ 公共 0.72°/帧契约（90 Hz，64.8°/s）
→ Marvin 真机桥 90 Hz 独立限速（默认约 0.778°/帧，70°/s）
→ Marvin 反馈安全检查
```

## 1. 输入目标整形

参数位于
`config/mode/controller_only/controller_only_ik.yaml` 的
`pico_controller_only_input.ros__parameters`：

| 参数 | 含义 | 当前保守初值 |
| --- | --- | --- |
| `translation_gain` | 手柄 XYZ 相对位移增益 | `[0.90, 0.90, 0.90]` |
| `rotation_gain` | 相对旋转增益 | `1.0` |
| `workspace_relative_radii_m` | 以 Home TCP 为中心的相对椭球半径 | `[0.32, 0.28, 0.28]` |
| `workspace_soft_zone_ratio` | 从椭球利用率何处开始渐近压缩 | `0.80` |
| `maximum_linear_speed_m_s` | 送入 IK 的最大末端线速度 | `0.30` |
| `maximum_angular_speed_rad_s` | 最大末端角速度 | `1.40` |
| `maximum_linear_acceleration_m_s2` | 最大线加速度 | `3.0` |
| `maximum_angular_acceleration_rad_s2` | 最大角加速度 | `9.0` |

软工作空间不是把目标硬裁到一个平面，而是在利用率超过阈值后连续压缩，
并渐近接近椭球边界。输入节点在 `/pico_body/status` 的
`target_conditioning.left/right` 中发布限制是否激活、请求速度和实际速度。

调参时先定工作空间和位移增益，再定速度，最后才调 One Euro
`min_cutoff/beta`。否则滤波延迟容易掩盖真实的速度或可达性问题。

## 2. 固定 ZSP 与多候选连续选解

纯手柄没有人体肘部信息，但配置保存了厂商 `FK_NSP` 在左右 Home
返回的参考平面方向。`official_use_zsp: true` 时，官方 IK 用该方向固定
七自由度冗余支路。

适配器始终保留厂商 `output_joint`，并尝试解释 `output_all_joint` 的行/列
布局。附加候选必须经过官方 FK 对目标位姿的复核才会进入候选集合，避免
错误 SDK 布局被静默采用。合法候选按下面三项加权：

```text
continuity_weight × 与上一帧关节距离
+ limit_weight × 软限位接近惩罚
+ posture_weight × 与 Home 姿态距离
```

对应参数是：

- `official_candidate_continuity_weight`；
- `official_candidate_limit_weight`；
- `official_candidate_posture_weight`。

通常先保持连续性权重为 `1.0`。只有 replay 显示最小限位裕度长期下降时，
才逐步提高 limit 权重；不要同时大幅调整三项。

## 3. 软关节限位

`official_joint_limit_soft_margin_deg` 默认是 `5°`。候选选择先惩罚靠近
软限位的解；最终单步还会阻止关节继续向软限位外运动，但允许已经位于
软限位带内的关节向安全方向退出。厂商硬限位标志仍然作为不可绕过的拒绝
条件。

`joint_limit_margin_deg` 是公共 IK 配置；官方后端的明确执行参数是
`official_joint_limit_soft_margin_deg`。两者建议保持一致。

## 4. 不可达和奇异目标恢复

官方精确目标失败后依次尝试：

1. 保持请求位置，逐级放松目标姿态；
2. 在当前已实现位姿和完整请求位姿之间二分；
3. 采用最远的可达中间位姿；
4. 所有降级仍失败时保持上一帧关节。

`official_orientation_relaxation_steps` 和
`official_workspace_backoff_iterations` 控制尝试次数。降级结果仍受公共
单步限制和软关节限位约束。状态中的 `orientation_relaxed`、
`workspace_backoff_active/fraction` 和 `consecutive_rejections` 用于区分
连续边界跟踪与真正的整帧拒绝。

## 5. worker deadline 与恢复

ROS 主进程通过 Unix `SOCK_SEQPACKET` 调用隔离 worker。每次调用先用
`poll()` 等待，超过 `official_worker_timeout_ms` 后关闭 socket、终止旧
worker，并最多按 `official_worker_restart_attempts` 重建后重试。

主进程启动 worker 时，所有浮点参数必须用 `double` 的 `max_digits10`
精度序列化。不能使用只有约 6 位小数的 `std::to_string`：例如纯手柄
`0.20°` 的弧度限幅会被向上舍入，导致 worker 输出略微超过主进程契约，
继而被公共安全检查连续拒绝。官方 probe 固定以 `0.20°` 覆盖该边界。

默认 `25 ms + 1 次重试` 是故障边界，不是期望耗时。90 Hz 下应重点观察：

- `solve_time_ms`：厂商库内部及回退求解时间；
- `transport_time_ms`：包含 IPC 的主进程端到端时间；
- `transport_restart_count/recovered`：本帧是否发生恢复。

正常轨迹的 transport P95 应明显低于一个控制周期 `11.1 ms`。若长期超过，
应减少回退次数或降低输入目标激进程度，而不是盲目增大 worker timeout。

## 6. JSONL 记录、指标和 preview-only replay

先启动正常纯手柄仿真，在另一个终端只读记录：

```bash
pixi run controller-only-trace -- record \
  --output tests/logs/controller_only_official.jsonl
```

按 `Ctrl+C` 停止后生成指标：

```bash
pixi run controller-only-trace -- metrics \
  tests/logs/controller_only_official.jsonl
```

建议固定录制六组动作：XYZ 单轴、腕部旋转、组合运动、手臂伸直、跨身体、
工作空间边界进入/退出。对比参数时重点看：

- solve/transport P95 和最大值；
- 请求关节单步与实际单步；
- TCP 位置/姿态误差；
- 最小关节限位裕度；
- saturation、backoff、soft-limit 帧比例；
- 连续拒绝峰值和 worker 重启次数。

关闭正在运行的 sim/real 后，可离线回放：

```bash
pixi run replay-controller-only -- \
  tests/logs/controller_only_official.jsonl --topics-only
```

去掉 `--topics-only` 可启动 RViz。Replay 使用独立
`controller-only-replay` 运行锁，发布 `scope=controller_only_replay` 和
`source=offline_replay`，且启动前拒绝与真机桥或实时输入节点共存；它不能
满足真机 readiness，禁止把 replay 当作真机输入源。

### 真机在线只读诊断

保持纯手柄 Sim 和 Real 正常运行，另开终端执行：

```bash
pixi run controller-only-real-diagnostic -- --duration 60
```

采集期间依次复现左臂、右臂、双臂同时运动和到达不了的位置。工具不会发布
任何控制消息，结束时根据累计计数区分输入整形、椭球工作空间、IK 单步、
奇异/回退、软关节限位、真机输出斜坡、周期漏期与实际跟踪误差，并把原始
JSONL 写入 `diagnostics/`。使用 `--duration 0` 时按 `Ctrl+C` 结束并打印。

## 推荐验收顺序

1. 运行官方 probe 和完整测试；
2. replay 对比默认参数与候选参数；
3. RViz/MuJoCo 做低速边界测试；
4. 真机保持默认 `velocity/acceleration ratio=60/80`，只做小幅、空载动作验收；
5. 确认实际循环稳定、没有连续拒绝、换支或小于 5° 的限位裕度后，
   再逐步放大工作空间和手柄增益；软件输出仍由 70°/s 默认限幅约束。

每次只改变一组参数并保留 trace。`official_dgr1/2/3` 使用厂商原始单位，
在没有对应 SDK 定义和 sweep 证据前保持 `0.05/0.05/0.0`。
