# Tianji Teleop

这是一个由 `mocap/aligned/hands` live 人手或 acquisition v4 H5
回放产生 canonical target 的遥操作系统。IK producer、唯一 command coordinator、
MuJoCo/Marvin/Wuji executor 均通过 versioned Zenoh protocol 通信。

## 当前实现状态

当前分支已经完成**初版 PICO2 遥操仿真机械臂闭环**：

```text
PICO2 头显/双腕位姿
  → 头部坐标系位姿映射
  → pico_ee_dexhand_qp（v131 velocity QP）
  → arm IK producer
  → coordinator
  → MuJoCo 天机机械臂
```

初版支持使用 PICO2 双腕相对头显的位姿控制 MuJoCo 中的双臂；手部 retarget
可以关闭，因此可以先单独观察机械臂 IK。系统也支持显示 PICO 原始头显、双腕
和 26 点手部骨架，以及实际送入 IK 的期望 TCP。手腕追踪短暂丢失时，仿真机械臂
保持上一帧关节命令，识别恢复后继续追向新的手腕目标。

推荐的初版 PICO2 仿真机械臂启动命令：

```bash
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile hand_tracking_sim \
  --viewer \
  --disable-hands \
  --ik-backend pico_ee_dexhand_qp \
  --arm-target-processor passthrough \
  --joint-trajectory passthrough \
  --command-step-clipping false \
  --joint-limit-source urdf \
  --pico-overlay \
  --ik-target-overlay
```

这条命令只控制 MuJoCo 中的双臂，不发布舞肌手控制命令；需要同时验证舞肌 2
代手时，去掉 `--disable-hands`，并保持 `hand_mode: retarget`。当前已通过隔离
的合成 PICO2 多进程仿真验证；真实 PICO2、真实天机机械臂和真实舞肌 2 代手
尚未完成联机验收。

## 唯一入口

先由操作者启动外部 router，再共享同一个 endpoint：

```bash
export TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447
pixi run doctor
pixi run mocap_live_sim
pixi run h5_sim -- --h5 TAKE.h5
pixi run target_replay_sim -- session.h5
pixi run joint_replay_sim -- session.h5
```

`h5_sim` 默认打开 MuJoCo viewer；自动化或无显示环境必须显式追加
`--headless`：

```bash
pixi run h5_sim -- --h5 TAKE.h5 --headless
```

真实设备必须明确确认，且由 profile 的 real capability、HostReadiness 和设备
preflight 共同放行：

```bash
pixi run mocap_live_real -- --confirm-real
pixi run h5_real -- --confirm-real --h5 TAKE.h5
pixi run h5_real -- --confirm-real --h5 TAKE.h5 --speed 1
```

手部输入的 Phase A 观察入口只接收、发布和记录，不启动 IK、coordinator、机械臂
或 Wuji 执行器。启动时由 `input_profile` 选择一种输入组合：

```bash
# PICO_2：TCP/ADB 接收双手 26 点及头显/双腕
pixi run bash scripts/run_source.sh --source pico_capture

# Manus 25 点骨架 + TJVR 旧 PICO 掌心 UDP（默认 127.0.0.1:15000）
pixi run bash scripts/run_source.sh --source manus_capture

# 集成入口，同样只观察/记录；也可用 --observation-config 指定另一份配置
pixi run bash scripts/run_session.sh --profile hand_tracking_observation
pixi run bash scripts/run_session.sh --profile hand_tracking_observation_manus
```

PICO2 默认监听本机 TCP `10002`。配置中的 `auto_adb_forward: true` 会在接收器
启动时自动执行 ADB 转发；连接头显后可以检查设备并手动确认转发状态：

```bash
adb devices
adb forward tcp:10002 tcp:10002
```

手动执行转发是可选的；如果由其他进程管理转发，可在独立观察入口追加
`--no-adb-forward`。集成仿真入口使用
`sources/hand_tracking_observation.yaml` 中的 PICO 配置。

仿真遥操作入口把观察进程、目标桥接、现有 arm IK、MuJoCo 和 Wuji Hand 2
retarget 串起来；启动时只选择一种输入 profile：

```bash
pixi run bash scripts/run_session.sh --profile hand_tracking_sim --viewer
# 只测试双腕→双臂 IK，不启动手部 retarget 或手部控制：
pixi run bash scripts/run_session.sh --profile hand_tracking_sim --viewer --disable-hands
pixi run bash scripts/run_session.sh --profile hand_tracking_sim_manus
```

`pico_ee_dexhand_qp` 是当前工程暴露的双臂 IK 选项，内部使用迁移后的
`pico_ee_v131_velocity_qp`，不再调用原 `DexhandVelocityQpIk7`。在下面的
`passthrough` 配置下，它绕过 IK 外层的关节 Ruckig 和协调器逐步裁剪，但保留
v131 内部的 Cartesian OTG 与 QP 约束：

```bash
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh --profile hand_tracking_sim --viewer \
  --disable-hands --ik-backend pico_ee_dexhand_qp \
  --arm-target-processor passthrough \
  --joint-trajectory passthrough --command-step-clipping false
```

这些覆盖参数目前只用于 `hand_tracking_sim` / `hand_tracking_sim_manus`。
选择新后端时上述三项默认即为直通、直通、关闭；现有默认后端和原来的
Ruckig/裁剪默认行为不变。退出旧会话后重启，保持双腕有效，按 `s` 开始。

### PICO 头坐标直接映射

默认 `relative_home` 将头相对手腕位姿的变化量映射到机器人 Home，按 `s`
建立参考。新增 `head_direct` 不减启动位姿，按 `s` 仅授权运动：

```bash
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile hand_tracking_sim --viewer --disable-hands \
  --arm-pose-mapper head_direct \
  --ik-backend pico_ee_dexhand_qp \
  --arm-target-processor passthrough --joint-trajectory passthrough \
  --command-step-clipping false --joint-limit-source urdf \
  --pico-overlay --ik-target-overlay
```

关系为 `T_base_tcp = T_base_head * T_head_wrist * T_wrist_tcp`。同一头相对
手腕位姿始终产生同一目标，与按 `s` 时的姿态无关。人整体平移或转向而
手相对头不变时，目标不变；只转头、手不跟随时，目标会变（并非躯干坐标系）。
启动时可能需要从 Home 追向当前手腕目标，先观察 IK 目标标记并预留空间。
已有丢失保持/恢复策略、IK 及轨迹选项不变。

固定几何在 `src/tianji_teleop/config/sources/hand_tracking_target.yaml` 的
`head_direct_mapper_config` 中配置：`input_to_base_rotation`、
`input_origin_in_base_m` 分别定义头到左右 Base 的旋转和头原点位置；
`tracked_to_tcp_pose` 定义手腕到原始 IK TCP 的固定变换（四元数 xyzw）。
默认是**仿真假定标定**：虚拟头位于模型两侧 Base 原点中点上方 0.30 m，
中性手腕姿态对应机器人 Home 姿态，并保留 36.5 mm 手心控制点到 TCP 的偏移。
它不是参考工程的实测掌心标定，真实佩戴时应按目标显示调校位置与姿态。

`--arm-pose-mapper` 仅支持 PICO 仿真。省略该参数或指定 `relative_home`
恢复默认模式；状态 `arm_pose_mapper` 可核对实际选择。单独启动目标源时，
可设置 `TIANJI_ARM_POSE_MAPPER=head_direct`，使用同一份目标配置。

### 独立姿态/高度补偿映射 head_palm_direct

此模式仍使用 PICO2 **头相对 Wrist 位姿**，不从骨架提取 Palm 点。
在原固定映射的独立副本上增加可调的姿态和高度补偿，不改变 `relative_home`
或 `head_direct` 的参数及输出。启动时仅替换：

```bash
--arm-pose-mapper head_palm_direct
```

配置位于同一目标 YAML 的 `head_palm_direct_mapper_config`：

```yaml
head_forward_offset_m: 0.10
head_height_offset_m: -0.10
tcp_local_z_correction_deg:
  left: 90.0
  right: -90.0
```

按用户确认的方向，补偿为机器人手掌局部 Z 轴左 +90°、右 -90°。
实际对齐效果仍需实测验证。姿态修正为右乘局部 Rz，
不是绕世界 Z 轴旋转。前向偏移沿固定虚拟头坐标系 +X，默认前移 10 cm，
不随手腕转动改变方向；设 `head_forward_offset_m: 0.0` 取消前移。
高度偏移沿固定虚拟头坐标系 Z 轴，负值向下，
先降低 10 cm；不能把它直接加到左右臂 Base 的 Z 分量。
局部 Z 补偿不改变手心控制点位置；若之后改变 TCP/control-frame 约定，
需重新核对固定偏移。前向、高度、姿态补偿均设为 0 且保留相同固定几何时，输出与
`head_direct` 一致。仍需双腕有效时按 `s` 启动，恢复识别不重新标定。

### 水平伸臂高度标定（head_palm_direct）

启动指令不变。在启动终端（不是 MuJoCo 窗口）操作：

1. 戴好 PICO2，头部保持自然朝前，双臂水平伸直，保持手腕稳定且均被识别。
2. 按 `c`，保持约 2 秒；终端提示 `Height calibration succeeded` 后再放松。
3. 按 `s` 开始遥操。标定只改高度，前移 10 cm 和左右姿态补偿保留。

每侧分别使用 `机器人控制点高度 = 模型水平参考高度 + 当前头相对手腕Z
− 标定平均手腕Z`。成功标定**替代**固定 `head_height_offset_m: -0.10`，
不与它叠加。坐标计算投影到模型竖直方向，并补偿控制点到原始 IK TCP 的
36.5 mm 偏移；不是把左右 Base 的 Z 分量直接当作高度。

模型参考来自当前 URDF 的零关节姿态 FK：本模型为双臂水平侧向伸展。
程序核对肩、肘、腕及控制点的世界高度差不超过 0.1 mm，否则拒绝标定。
**仅计算参考，不发送零关节姿态命令。** 标定不会修改 IK、关节限位或原始数据。

采样要求每侧至少 30 个不同时间戳、覆盖至少 1.5 秒；超过 0.25 秒断流、
识别失效、非法数据或手腕位置任一轴变化超过 6 cm，都会使本次标定失败。
仅凭腕部位姿无法自动确认你是否真的伸直双臂，需自行保持正确姿势。
采样中禁止 `s` 启动，遥操/回零中拒绝 `c`。首次标定失败后需按 `c` 重试；
重新标定失败则保留已有标定，可继续使用旧结果。

标定结果保留在当前进程内，按 `s` 退出再开始不丢失，重启会话需重新标定。
状态 `height_calibration` 提供采样数量、成功/失败及左右平均高度。
未按过 `c` 时仍保留此前的固定偏移行为；其他映射不支持此标定且不受影响。

隔离合成数据验收：

```bash
pixi run python scripts/pico_sim_smoke.py --disable-hands \
  --arm-pose-mapper head_palm_direct --ik-backend pico_ee_dexhand_qp \
  --arm-target-processor passthrough --joint-trajectory passthrough \
  --command-step-clipping false --joint-limit-source urdf \
  --height-calibration-test --tracking-loss-test
```

### PICO 追踪丢失保持

PICO 仿真 `--disable-hands` 下，持续收到合法数据包但某只手腕未被识别时，
该臂保持协调器最后下发的关节位置，暂停 IK 积分；另一臂继续跟踪，会话不退出。
恢复识别后**沿用本次启动时的原映射，不重新标定**，从保持位置重新运行 IK，
追向新识别到的手腕目标，不需要再按 `s`。恢复目标可能较远，请预留仿真运动空间。
状态 `tracking_hold_sides` 标明正在保持的侧别。头部追踪无效导致双腕输入无效时，
双臂保持；断流/超时、非法数据、IK 故障和硬限位保护仍然有效。
此行为不扩展到 Manus、启用手部 retarget 的入口或实机。

隔离仿真回归（合成 PICO 数据，无需连接头显）：

```bash
pixi run python scripts/pico_sim_smoke.py --disable-hands \
  --ik-backend pico_ee_dexhand_qp --arm-target-processor passthrough \
  --joint-trajectory passthrough --command-step-clipping false \
  --joint-limit-source urdf --tracking-loss-test
```

在上述命令末尾追加 `--pico-overlay`，同一 MuJoCo 窗口同步显示原始 PICO
头显、双腕 XYZ 坐标轴以及双手各 26 点骨架。兼容 `--disable-hands`，未按
`s` 或控制停止时也能观察有效输入；不会启动手部 retarget 或发布控制命令。

显示区标为 `PICO raw FLU`：X 前、Y 左、Z 上；红/绿/蓝分别为 XYZ，左手
青色、右手橙色。保留 PICO 初始参考系、米制尺度和原始相对位置，仅向
MuJoCo 的 +Y 平移 1.2 m、+Z 平移 0.8 m 以分开显示，**不是机械臂 IK 目标或机器人坐标配准**。
因此原始点不必和机器人 TCP 重合，转头也不会把原始场景重新归零。
拖动/缩放 MuJoCo 相机可查看机器人侧面的显示区。

无输入显示 `waiting`；超过 0.5 秒无新鲜数据会清除头显/手部几何并显示
`stale`，无效关节及其相连骨段不绘制。执行器状态的
`diagnostics.pico_overlay` 提供接收计数、帧关联 ID、状态及拒收原因；这些
诊断不改变控制健康判定。只缓存最新帧，不保证每个原始帧都渲染；完整原始
帧请使用 `--record PATH`。此 session 参数仅支持 `hand_tracking_sim`。

独立 MuJoCo 入口也可转发 `--pico-overlay`，或在其 YAML 配置中设置
`pico_overlay: true`。无窗口模式下仅接收和输出诊断，不绘制。

可再追加 `--ik-target-overlay`，在机器人所在区域绘制真正送入 IK 的双臂
**期望 TCP 位姿**：`IK desired TCP left/right`。订阅 `tianji/target/arm/left`
和 `right`，将消息的 `Base_L/Base_R` 坐标通过当前仿真 URDF 的固定基座链
转换为 MuJoCo 世界坐标。包含完整的目标位置和姿态；轴长 0.12 m，红绿蓝
分别为局部 XYZ。不是把原始 PICO 手腕直接画到机器人旁，也不重复施加
36.5 mm 手腕中心到 TCP 补偿（target 中已经完成）。

与原始 PICO 显示可同时开启，兼容 `--disable-hands`，不发布任何控制指令。
按 `s` 后有实际 IK target 才显示；未接收到目标时不虚构 Home 目标。
超过 0.5 秒无新鲜目标，保留最后位置并变灰、标注 `[stale]`，用于检查停机
位置，不代表机械臂还在跟踪该目标。执行器状态的
`diagnostics.ik_target_overlay` 提供两侧接收状态及最后序号。
此 session 参数仅支持 hand_tracking 仿真；独立 MuJoCo 入口也可使用该
参数或 YAML `ik_target_overlay: true`，但必须提供 `TIANJI_SOURCE_INSTANCE_ID`。
它是诊断显示，不会修复 IK 求解失败或回位状态问题。

| 处理层 | 参数 | 配置位置 |
|---|---|---|
| IK 前目标整形 | `--arm-target-processor passthrough` 或 `conditioned` | source 的 `arm_target_processor` / `arm_target_processor_config` |
| IK 后轨迹处理 | `--joint-trajectory passthrough` 或 `ruckig` | producer 的 `joint_trajectory_processor` / `ruckig_*` |
| 协调器增量裁剪 | `--command-step-clipping false` 或 `true` | coordinator 的 `command_step_clipping_enabled`，省略默认 true |

例如仅增加平滑：将 `--joint-trajectory passthrough` 改为
`--joint-trajectory ruckig`，不需要开启另外两层。该选项使用本工程现有
`JointTrajectoryLimiter7`，与 v131 算法内部的 Cartesian OTG（Ruckig<3>）独立。
直出仍保留 QP 内部速度/关节约束和下游安全拒绝；动态诊断比例为 0 不表示
直出已满足 Ruckig 加速度/jerk 限制。普通回 Home 仍使用受限回位轨迹。

新后端配置入口仍为 `config/producers/ik_dexhand_qp.yaml`。现在保留原版完整
速度控制链：带时间戳的目标历史 → Cartesian OTG → 自适应增益参考伺服 →
单级 qpOASES 速度 QP。包含静止保持、超时处理、零姿态偏好、任务缩放、
连续性/jerk 趋势、奇异性规避，以及位置 margin、速度、制动、加速度和 jerk 包络。
PICO 坐标映射、36.5 mm TCP 补偿和机器人 Home 不变；不移植实体设备驱动。
该后端现在按原版 `model_state_only` 推进内部 `q_ref/qdot/qddot`：普通反馈
不覆盖内部状态；启动/退出遥操、断流及硬安全拒绝时才重置对齐。主运动学
使用随工程提供的原版 MuJoCo fast 模型，在世界坐标系计算 TCP/Jacobian，
接口边界与当前工程 Base 坐标转换。可视化场景及舞肌手模型不替换。
`producer/status` 中 `backend` 保留兼容名称，`algorithm` 为
`pico_ee_v131_velocity_qp`。旧 `qp_*` Dexhand 增益/权重/nominal 字段不再控制
v131；原版参数集中在 `include/tianji_v131/velocity_profile.hpp`（修改后重新编译）。
TCP 线/角速度上限为 3 m/s、12 rad/s；该仿真后端使用 4 rad/s 关节速度和
200 Hz 下 0.02 rad 的单步契约，单独选择 `coordinator/arm_v131.yaml`。
其他后端的配置、步长和默认处理方式不变。`--joint-trajectory passthrough`
仅绕过额外的关节 Ruckig，不关闭原版 Cartesian OTG。
`--ik-target-overlay` 仍显示送入后端的目标，不是 OTG 中间参考位姿。
移植范围和验证记录见 `src/tianji_teleop/src/ik/v131_qp/PORTING.md`。

### 使用 URDF 配置下游关节保护（v131 仿真）

```bash
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile hand_tracking_sim --viewer --disable-hands \
  --ik-backend pico_ee_dexhand_qp \
  --arm-target-processor passthrough \
  --joint-trajectory passthrough --command-step-clipping false \
  --joint-limit-source urdf --pico-overlay
```

`--joint-limit-source urdf` 从当前工程的 `TIANJI_ARM_URDF`（省略时使用
工程默认机械臂 URDF）提取限位，为当前 session 生成 `RUN_ID-arm-limits.yaml`，
通过 `TIANJI_ARM_CONFIG` 传给 native producer、协调器和 MuJoCo。为忠实复现
原版，v131 QP 内部使用原版 MuJoCo fast 模型的左右臂限位（含 0.05 rad margin），
不再从此 URDF 替换内部约束。producer 的输出检查、可选 Ruckig 和协调器
使用生成的配置作为额外保护；下游不再混用更窄的 YAML 限位。原始 `robot/arm.yaml` 不改动，
Home 和关节顺序保留。Python 配置加载器支持该环境变量，显式路径优先。

该选项仅支持 `hand_tracking_sim` / `hand_tracking_sim_manus`，且 URDF 模式
要求显式指定 `--ik-backend pico_ee_dexhand_qp`，不开放给实体机器人。
缺少关节、非有限限位、左右臂范围不一致、Home 超界时启动失败，不静默回退。
当前共享配置只支持左右臂相同的七关节限位。

省略参数或选择 `--joint-limit-source yaml` 保持原来的配置来源，不修改 QP
内部现有实现。URDF 模式不是关闭检查：非有限值、越界及异常步长仍会拒绝，
也不会自动开启 Ruckig 或裁剪。需退出旧 session 后重新启动才能生效。
此选项不修复独立的浮点回位完成判断问题。

离线验证：

```bash
pixi run -e ik-build build-ik-sim
pixi run python scripts/pico_sim_smoke.py --disable-hands \
  --ik-backend pico_ee_dexhand_qp --joint-trajectory passthrough \
  --command-step-clipping false
```

仿真会话可追加 `--record PATH`。它使用一个被动 schema 1.1 recorder，同一文件
保存选定的 raw/observation、arm/hand target、joint 和 session event 流；观察
入口仍不启动机器人控制。该 profile 只允许 simulation，不接受 `--confirm-real`。

PICO 仿真使用 `sources/hand_tracking_observation.yaml` 连接 PICO2（默认 ADB
转发 TCP 10002），使用 `sources/hand_tracking_target.yaml` 配置双腕到机械臂
TCP 的映射。等待输入与 coordinator 就绪后，在 source 终端按 `s` 开始，
再次按 `s` 请求回零。Wuji dry-run 产生双手关节命令，MuJoCo 的
`hand_overlay: mujoco` 接收这些命令并更新左右手；无需连接实体舞肌手。

PICO 机械臂映射现在以手腕中心为控制点：`ik_tcp_to_control_pose` 配置
原 IK TCP 到手腕中心沿局部 +Z 的 36.5 mm 偏移。`home_pose` 仍填写原
`TCP_Link_L/R` 的 Home；映射器先推导手腕中心 Home，再按 PICO 位移和
旋转生成手腕中心目标，最后转换回原 TCP 送入 IK。纯旋转时手腕中心保持
不动，原 TCP 按目标朝向补偿位置；初次启动不因新增偏移改变 Home。

完整启动需要外部管理的 Zenoh router，以及当前工程的原生 IK 产物
`staging/ik/lib/tianji_teleop/arm_ik_producer` 或
`runtime/tianji_teleop/lib/tianji_teleop/arm_ik_producer.bin` 和对应共享库。
本机仿真可独立编译现有 C++ IK，无需 Wuji 实体 SDK 和便携 runtime/ABI 包：

```bash
pixi run -e ik-build build-ik-sim
```

该任务增量构建到 `build/ik-sim`，安装到 `staging/ik`，并运行双臂 QP
探针。使用本机 GCC、Pixi 的 Pinocchio 4.0 和 Ruckig v0.19.4。
Zenoh 依赖为官方 `zenoh-c` 1.10.0 standalone 包的 `include/`、`lib/`
（放入 `vendor/zenoh/`），以及 `zenoh-cpp` 1.10.0 的头文件
（放入 `vendor/zenoh-cpp/include/`）。当前工作区已下载这两项；vendor
和构建产物不入 Git，新机器需另行准备。原有完整部署仍使用 `build-ik`。

`build-ik-sim` 显式启用 `TIANJI_ENABLE_V131=ON`，使用本机 Pixi 的
MuJoCo/qpOASES，不能作为便携包复制到其他机器。`build-ik` 显式关闭该选项，
旧 IK 后端不引入这两项动态依赖；部署脚本会在写入前拒绝误放入 staging 的
v131 仿真二进制。当前不提供 v131 的便携 runtime 包。

v131 配合 `--joint-trajectory ruckig` 时，额外关节 Ruckig 跟踪内部模型的
**位置目标**，限速后仍会补齐位移；其他 IK 后端保留原来的速度模式。
`--joint-trajectory passthrough` 不经过这一层，不改变原版内部 Cartesian OTG。

v131 专用 coordinator 使用 `command_step_time_window_s: 0.05`：步长保护按
上次实际采用的 proposal 时间戳计算累计位移额度（4 rad/s，最多计入 50 ms），
不再把漏收多个 200 Hz proposal 的合法位移都当成单步跳变。原有两周期容差
仍保留；硬限位、异常跳变、时间倒退及过期检查不关闭，其他后端默认使用原校验。
若再次触发步长故障，`tianji/coordinator/status` 的 `diagnostics.step_rejection`
包含侧别、实际位移、允许位移、proposal 时间差和序号。修改前已 fault 的会话
仍需先退出再启动，重复按 `s` 不会解除故障锁存。

额外轨迹回归运行 `pixi run -e ik-build check-ik-trajectory`。当前本机验证中，
该探针的 Regrind 移动目标断言失败（约 14.4 mm，要求小于 5 mm），尚未修复；
双臂 QP 探针通过不代表这项动态跟踪测试或完整 PICO 会话已通过。

可独立验证 PICO 协议→双臂 target、Python dry-retarget→真实 MuJoCo 双手模型：

```bash
PYTHONPATH=src/tianji_teleop pixi run python -m unittest tests.test_pico_mujoco_pipeline -v
```

该测试使用合成输入，不运行原生 IK，也不等同于头显到仿真双臂的完整联调。

完整多进程合成输入验证（不连接头显或实体机器人）：

```bash
pixi run python scripts/pico_sim_smoke.py
```

需先构建上述原生 IK，并将官方 Zenoh 1.10.0 standalone 的 `zenohd`
放到 `vendor/zenoh-router/zenohd`（当前工作区已准备）。脚本启动仅监听
loopback 独立端口的路由器和合成 PICO TCP 服务，通过实际 `run_session.sh`
启动接收器、目标映射、原生 IK、协调器、Wuji dry-run 和 headless MuJoCo。
自动检查双臂/双手关节反馈变化、断流后停止目标发布并回到 idle、HDF5 原始
26 点数据回读。退出时清理本次启动的进程；日志、`result.json` 和 `session.h5`
保留在打印的 `/tmp/pico-sim-smoke-*` 目录。此验证不替代真实头显轴向、比例、
跟踪丢失与佩戴体验验收，也不覆盖上面的 Regrind 轨迹探针。

可选 `--record PATH` 写入 schema 1.1 session HDF5。PICO 原始帧位于
`raw/pico_hand_tracking`，Manus 和旧 TJVR 输入分别位于
`raw/manus_hand_tracking`、`raw/legacy_pico_palm`；统一 21 点骨架和 arm-input
观察位于 `observation/`。默认配置为 `sources/hand_tracking_observation.yaml`，
Manus 配置为 `sources/hand_tracking_observation_manus.yaml`。

`run_session.sh` 在 spawn 前为每个 component 分配 UUID，并注入
`TIANJI_COMPONENT_INSTANCE_ID`、`TIANJI_COORDINATOR_INSTANCE_ID` 和实际
`TIANJI_ROUTER_ZID`。组件不会自行生成身份；同 logical id 的多 instance 会被
拒绝。session profile 不保存 router endpoint 或 IK backend。

## 配置

唯一配置树位于 `src/tianji_teleop/config/`：

- `robot/`：双臂和 Wuji Hand 2 的 names、Home、rad limits、zero；
- `sources/`、`producers/`、`executors/`：组件参数；
- `coordinator/arm.yaml`：状态机和回 Home 参数；
- `recording/`、`replay/`、`diagnostics/`：session v1、回放、标定；
- `sessions/`：只组合 component config、capability、active/inactive sides 和 hand mode。

关节 wire 单位固定为 rad；Marvin 只在 SDK 边界转换为 degree。外部
acquisition H5 v4 与 session HDF5 v1 是两个不同 schema。

## 安全边界

MuJoCo、Marvin 和 Wuji 都只消费 coordinator final command 或受授权 hand
command publisher。diagnostics 仅订阅权威 state/status 或发送 intent，不发布
第二份 state/final command。普通 return 是 bounded Home；feedback、router、
identity、hard limit 或 safety stop 异常进入 fault，不能由 return 清除。

IK 构建与 runtime 部署：

```bash
pixi run -e ik-build build-ik
pixi run -e ik-build deploy-ik
```

安装产物只包含 `arm_ik_producer`、canonical source/producer/executor、recorder、
replay、policy 和 diagnostics 入口；过时入口不会保留 alias。
