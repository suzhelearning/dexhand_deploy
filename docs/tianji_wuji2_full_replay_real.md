# Tianji + Wuji2 Hand 完整真机 H5 回放操作说明

机械臂（Tianji/Marvin 双臂）与 Wuji2 Hand 手部**一起**回放采集的 H5
数据：IK 驱动双臂沿 Manus 右腕轨迹运动，`wuji_hand2_bridge` 驱动右手
手指，viewer 同步动画。

本文对应 README「Tianji + Wuji2 Hand 完整真机 H5 回放」一节，整理为
可直接执行的现场操作单。

## 1. 适用场景

- 需要**机械臂 + 手同时回放**的完整链路（hand-only 回放只动手指）。
- H5 数据来自 Manus 动捕采集（含右手 21 键点、右腕位姿、物体轨迹）。
- 真机：Tianji 双臂 + Wuji2 右手，经 `enp129s0` 控制网。

## 2. 前提条件

### 2.1 网络（先检查，路由不符禁止启动）

```bash
ip route get 192.168.1.111
ping -I enp129s0 192.168.1.190   # Tianji
ping -I enp129s0 192.168.1.110   # Wuji Left
ping -I enp129s0 192.168.1.111   # Wuji Right
```

三台设备路由结果必须都是 `dev enp129s0 src 192.168.1.165`，且该连接
**不得产生 default route**。Zenoh 控制话题全部在本机默认 scouting；
现有 Router 只负责 Motive/Manus 数据。

### 2.2 设备与数据

| 项 | 要求 |
| --- | --- |
| Motive 动捕 | 运行中，`tianji_wrist` 刚体可见（frame0 骨架定位） |
| Tianji 双臂 | 上电、`192.168.1.190` 可达、急停释放 |
| Wuji2 右手 | 供电/网线正常、`192.168.1.111` 可达、序列号 `WH2KA01260814006` |
| H5 数据 | 含 `hands/right/*`（键点 + wrist 位姿）的 take 文件 |

本次实测数据：

```
/home/current/data/20260826/20260826_165624_519878_take006.h5
（514 帧，含锤子物体轨迹；`--speed 0.1` 约 40~50 秒）
```

## 3. 三终端启动

### 终端 1：Motive/H5/IK/MuJoCo 主机

```bash
pixi run sim_mocap_h5 -- /path/to/take.h5 \
  --complete-wuji2-real-preview --speed 0.1 --yaw-deg 0 \
  --right-rigid-id 3
```

- viewer 显示两套 wrist 坐标轴：Manus wrist W（frame0 固定，细、半透明）
  与当前 FK Wuji r_wrist B（粗、纯 RGB），到达 frame0 时检查两原点重合。
- 每 0.5s 输出数值诊断：
  `frame0 r_wrist 目标↔FK：位置误差=…mm 姿态误差=…°`。

### 终端 2：Tianji/Marvin 双臂真机桥

```bash
pixi run real_mocap_h5 -- --confirm-real
```

等待输出 `phase=armed_idle` 再进终端 3。首次固定 10% 速度/加速度。

### 终端 3：Wuji2 Hand 真机桥

```bash
pixi run wuji_hand2_real -- --confirm-real \
  --side right --serial WH2KA01260814006 \
  --rate 100 --keypoint-timeout 0.5 --command-slew-rate 1.0 \
  --tracking-slew-rate 6.0 --teleop-grace-s 0.3
```

无 SN 时可用 `--address 192.168.1.111:50001`（端口以 SDK scan 为准）。

## 4. 启动顺序（必须遵守）

```
① 终端 1：确认 Motive ID / IK / Home 正常
② 终端 2：等 phase=armed_idle
③ 终端 3：手桥就绪（idle/unknown 保持零位）
```

## 5. 操作流程

终端 1 键盘操作：

```
s → Enter（定零 / 冻结 frame0 骨架）
r → Enter（开始回放）
s → Enter（回 Home，随时可中断）
```

## 6. 状态门控与安全机制

| 机制 | 行为 |
| --- | --- |
| 手桥门控 | 仅 `teleop` 且键点新鲜（0.5s 内）时跟踪；`idle/unknown` 保持零位 |
| 键点超时 | 键点超过 0.5s 未更新 → 自动按 1 rad/s 缓速回零 |
| 回 Home | 终端 1 按 `s` → 机械臂回 Home，手桥进入 `returning_zero` |
| 命令保持 | `--teleop-grace-s 0.3`：离开 teleop 后 0.3s 窗口内零扰动 |
| 跟踪限速 | `--tracking-slew-rate 6.0` 仅限跟踪段爬升，与回零速率解耦 |

终端 3 status 字段：`teleop_state`、`tracking_allowed`、
`keypoint_timed_out`、`command_max_abs_rad`。

## 7. 安全退出（顺序很重要）

```text
1. 终端 1 按 s → 确认机械臂回 Home 且手桥 phase=zero_hold
2. 终端 3 Ctrl+C（disable 手）
3. 终端 2 Ctrl+C → 看到 Robot released
4. 终端 1 按 q
```

## 8. 注意事项与常见错误

| 错误 | 说明 |
| --- | --- |
| 用 `--complete-wuji2-replay` | 它会自己启动 dry 手桥，与终端 3 真机手桥**双发布冲突**。完整真机必须用 `--complete-wuji2-real-preview` |
| 跳过终端 2 直接起手桥 | 手桥需等机械臂 `armed_idle`，且发布端缺失时 runner 拒绝连接 |
| 忽略网络路由检查 | 路由带 default route 或 src 不对 → 控制话题不可达 |
| 终端 3 用全局 teleop_state key | 完整链路使用默认全局 key 即可（H5 主机是唯一发布者）；hand-only 链路才必须 `--teleop-state-key` 绑定专属 key |

## 9. 参考

- 仿真验收（无真机）：`pixi run sim_mocap_h5 -- <take.h5> --wuji2 --frame0-skeleton`
  + `pixi run wuji_hand2_dry -- --rate 100`
- 仅手回放：`docs/README` 的 `sim_mocap_h5_replay + --hand-only-replay` 一节
