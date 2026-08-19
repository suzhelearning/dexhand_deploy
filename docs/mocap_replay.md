# mocap HDF5 手腕轨迹回放（轨迹跟踪仿真）

`mocap` 分支新增的离线回放入口：把 mocap-acquisition 采集程序
（`/home/current/syz/mocap/acquisition`）录制的 HDF5 里**左右手腕位姿**
按录制节奏回放成轨迹跟踪仿真输入。机器人（RViz/MuJoCo 中的纯运动学
模型）会像在线 PICO 遥操作一样跟踪这条轨迹，可用来离线验证 IK 的
轨迹跟踪能力、工作空间与速度整形行为，不连接真机。

## 数据来源与文件格式

采集文件位于 `/home/current/data/<日期>/<时间>_take<N>.h5`，例如：

```
/home/current/data/20260819/20260819_151737_102784_take003.h5
```

本回放只支持 v4.0 紧凑 60 Hz 布局（根属性 `h5_version == "4.0"`、
`schema_layout == "compact-aligned-60hz-v1"`），读取内容：

| HDF5 路径 | 内容 |
| --- | --- |
| `time_ns` | 公共时间轴（int64，linux-clock-monotonic，固定 60 Hz） |
| `hands/left/wrist_position` | 左手腕位置 (N,3) 米制 |
| `hands/left/wrist_quaternion_xyzw` | 左手腕姿态 (N,4) xyzw 序 |
| `hands/left/valid` | 左手腕该帧是否有效 |
| `hands/right/*` | 右手腕同构数据 |

手腕位姿位于 **Motive 动捕系**（y-up 右手系、米制，与 PICO 手柄的
y-up 世界系同族）。注意根级 `valid` 标记整帧（含物体刚体）是否有效；
纯动捕会话里物体通常从未跟踪，根级 `valid` 几乎恒为 False，**不作为
手腕回放门控**，只有 `hands/<side>/valid` 有意义。

## 运行方式

```bash
# 进入已部署运行时的项目目录后（源码仓库需先 build-ik + deploy-ik）

# MuJoCo 轨迹跟踪仿真（默认，与 sim_controller_only 一致）
pixi run sim_mocap -- /home/current/data/20260819/20260819_151737_102784_take003.h5

# 同时启动 RViz 与 MuJoCo
pixi run sim_mocap -- TAKE.h5 --both

# 只启动 RViz
pixi run sim_mocap -- TAKE.h5 --rviz-only

# 无界面验证话题链路
pixi run sim_mocap -- TAKE.h5 --topics-only

# 常用回放参数
pixi run sim_mocap -- TAKE.h5 --speed 1.0 --yaw-deg 0 --reference-frame -1
```

参数说明：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `--speed N` | `1.0` | 回放倍速（按 h5 时间轴缩放） |
| `--yaw-deg N` | `0` | 整条轨迹绕竖直轴旋转的朝向标定（度）。录制时人的朝向与机器人正前方有夹角时使用；经 `pico_to_robot` 映射后等价于绕机器人世界 Z 轴旋转 |
| `--reference-frame N` | `-1` | 参考帧下标，等效于在线链路“按右手柄 A”的时刻；`-1` = 第一个有效帧 |

回放状态机与 `replay-controller-only` 一致：`arming`（1 s，idle）
→ `replaying`（发布 `teleop`，按录制时间轴发布目标）→ `returning`
（3 s，请求回 Home）→ 自动退出。回放身份为 preview-only，**不能**用于
启动真机桥；启动时会拒绝存在真机桥/实时输入节点的 ROS 图。

## 数据链路

```
HDF5 手腕位姿（Motive 系，60 Hz）
  → [--yaw-deg] 绕竖直轴朝向标定
  → ControllerOnlyTeleopMapper（与在线 PICO 完全相同的链路：
     增量相对参考帧 → pico_to_robot → world→chest
     → One-Euro 滤波 → 尺度/椭球工作空间/速度加速度整形）
  → /pico_body/{left,right}_arm_target_pose（{side}_chest 系）
  → tianji_kinematic_sim（可配置 IK 后端）
  → /pico_body_sim/{left,right}_arm/joint_commands（度）
  → /pico_body_sim/model_joint_states（弧度，RViz/MuJoCo 镜像）
```

参考帧语义与在线链路一致：参考帧处相对增量为零，映射目标为机器人
Home（`tianji_robot.yaml` 的 `init_pos`/`init_quat`，即 FK 安全初始位）；
之后的每帧只回放相对参考帧的增量。因此回放开始时机器人处于安全初始
位，随后沿录制轨迹运动，结束时平滑回 Home。

## 单侧数据缺失

采集时单侧手腕可能完全没有跟踪（例如 take003 左手全 NaN）。加载器把
该侧全部标记为无效，回放时该侧每帧使用其参考位姿（合成单位四元数），
增量恒为零，映射结果恒为机器人 Home——即**缺失侧机械臂保持在安全初始
位，不阻碍另一侧回放**。启动日志会警告缺失侧，`/pico_body/status` 的
`recording.hands.<side>.valid_frames` 可查看有效率。

## 1:1 目标整形与 50mm 位移验收

mocap 回放默认 **1:1 目标整形**（`mocap_h5_replay.translation_gain =
[1.0, 1.0, 1.0]`）：命令位移 → IK 目标位移严格 1:1。例如命令
“+x 移动 50mm”，目标位移即 50.0mm（实测 solved/FK 位移 49.9–50.0mm，
误差来自 IK 位置容差 ~1mm）。映射链中唯一的尺度因子就是
`translation_gain`（`pico_to_robot` 与 world→chest 均为无缩放正交
变换）；工作空间软区在目标利用率 >0.90 时才压缩，速度/加速度整形只在
快速移动时介入，50mm 量级渐变台阶均不触发。在线 PICO 链路刻意保留
0.90 安全缩放，需要复现其行为时把该参数改回 `[0.90, 0.90, 0.90]`。

### 合成台阶轨迹生成与验收

用内置生成器制作一条“移动 N mm → 保持 → 回程”的合成轨迹：

```bash
# 输入（手腕）系 +x 移动 50mm
pixi run mocap-step-h5 -- --output /tmp/step50mm_x.h5 --axis x --mm 50

# 机器人 chest +x（前）方向移动 50mm：输入 −z
pixi run mocap-step-h5 -- --output /tmp/forward50mm.h5 \
  --axis z --dir neg --mm 50
```

然后回放并观测目标/求解位移：

```bash
pixi run sim_mocap -- /tmp/step50mm_x.h5 --topics-only
# 另开终端查看 /pico_body/{left,right}_arm_target_pose 与
# /pico_body_sim/{left,right}_arm/solved_pose 的位移
```

输入（手腕/Motive）系 → 机器人 chest 系的轴映射（默认
`pico_to_robot`）：

| 输入轴 | 机器人世界 | left_chest | right_chest |
| --- | --- | --- | --- |
| +x | −y（右） | (0, 0, −1) | (0, 0, +1) |
| +y | +z（上） | (0, −1, 0) | (0, +1, 0) |
| +z | −x（后） | (−1, 0, 0) | (−1, 0, 0) |

参数：`--axis {x,y,z}`、`--dir {pos,neg}`、`--mm N`、`--ramp-s` /
`--hold-s` / `--return-s` / `--rate`。生成器无 ROS 依赖，产物与采集端
v4.0 布局一致，`load_mocap_h5` 可直接读取。

### 已验收结果（本机实测）

| 命令 | 目标位移峰值 | solved 位移峰值 | 方向 |
| --- | --- | --- | --- |
| 输入 +x 50mm | 50.0mm | 49.9mm | left (0,0,−50) / right (0,0,+50) |
| 机器人 +x 50mm（输入 −z） | 50.0mm | 50.0mm | 双侧 (+,0,0) |

单元测试 `tests/test_mocap_step_h5.py` 固化了两项：gain=1.0 时
50mm 命令 → 50mm 目标；gain=0.90 时 → 45mm（回归证明 gain 是
唯一尺度因子）。

## 观测与验证

- `/pico_body_sim/status`（0.5 Hz）：IK 每侧位置/姿态误差、软限位、
  奇异、回退与连续拒绝等诊断；
- `/pico_body/status`：回放阶段、帧进度、目标整形诊断；
- 另开终端 `pixi run controller-only-joints` 查看 14 关节命令；
- 另开终端 `pixi run controller-only-trace -- record --output t.jsonl`
  可把本次回放的目标/求解结果记录为 JSONL，再
  `pixi run replay-controller-only -- t.jsonl` 复现。

## 部署说明

本功能新增了 Python 节点 `mocap_h5_replay`、launch 文件
`mocap_replay.launch.py` 与 `h5py` 依赖。源码仓库需重新构建部署后，
运行时的 `pixi run sim_mocap` 才会存在：

```bash
pixi install --locked -e ik-build
pixi run -e ik-build build-ik
pixi run -e ik-build deploy-ik
```

普通用户使用独立压缩包时，应使用包含本功能的新版本压缩包。
