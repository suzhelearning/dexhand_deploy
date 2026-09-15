# Tianji Teleop

这是一个由 `mocap/aligned/hands` live 人手或 acquisition v4 H5
回放产生 canonical target 的遥操作系统。IK producer、唯一 command coordinator、
MuJoCo/Marvin/Wuji executor 均通过 versioned Zenoh protocol 通信。

## 当前实现状态

两条路线均已有用户真实输入设备遥操 MuJoCo 的成功反馈：PICO2 裸手，以及
PICO＋双 VR 手柄＋Manus 手套（机械臂与舞肌二代手）。以下指令均为**仿真**操作；
真实设备长时间完整录制、Ctrl-C 后录制完整性和两路反复切换仍需补充验收。

| 操作 | PICO2 裸手 | PICO＋VR 手柄＋Manus |
| --- | --- | --- |
| 会话 profile | `pico2_hands_sim` | `pico_vr_manus_sim` |
| 头显应用 | PICO_2 手追踪采集 APK | `com.PICO.wholebody_stream.unity` |
| USB 数据端口 | 手动转发 TCP 10002 | 启动器管理 TCP 9999 |
| 机械臂输入 / IK | 头相对双腕 / v131 QP | 原版 driver/M0/TJVR / SPARK（默认）或 mapped-palm QP |
| 手指输入 | PICO 裸手骨架 | Manus 手套骨架 |
| 启动终端按键 | `c` 仅 Z 标定，`s` 开始/回 Home，`q` 退出 | `s` 开始，`h` 回 Home，`r` 在 Home 重置，`q` 退出 |

两路共用一个 Zenoh router，每次只启动一路。切换前退出会话并等待终端返回提示符，
再切换头显 APK；不需要重启正常运行的 router。键盘操作在**启动会话的终端**进行。

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

## PICO2 裸手遥操快速测试（机械臂＋舞肌二代手）

以下命令在工程根目录执行，使用 `pico2_hands_sim` 和 `head_palm_direct`，控制
MuJoCo 中的天机双臂与舞肌二代手。PICO2 中需运行原有 PICO_2 手追踪采集 APK，
启用手部追踪和数据发送，并通过 USB 连接电脑、允许 USB 调试。

### 首次准备

```bash
pixi install --locked
pixi install --manifest-path tools/wuji_hand_native/pixi.toml --locked
pixi run -e ik-build build-ik-sim
```

需要系统 GCC/G++、ADB 和可用的桌面显示环境。`build-ik-sim` 编译本机 v131 仿真 IK。
Git 会提交内嵌的 `vendor/pico_tracker` 原版 PICO 源码 bundle；其生成的 Pixi 环境、
build/install、厂商二进制运行时以及 `runtime/` 仍由 Git 忽略。头显 APK 和编译产物
也不随仓库提交。新电脑还需准备本工程配套运行依赖（包括 Zenoh C/C++ 依赖和下述
`zenohd`），不能仅凭 `git pull` 就假定依赖齐全。详见 [厂商运行时说明](VENDOR_RUNTIME.md)、
[官方手独立环境](docs/wuji-hand-porting.md)。此 PICO2 路线不需要 Manus 或 XRoboToolkit。

### 终端一：启动 Zenoh router

```bash
./vendor/zenoh-router/zenohd \
  -l tcp/127.0.0.1:7447 \
  --no-multicast-scouting
```

保持此终端运行；如果该地址已有可用 router，不要重复启动。
设置 `TIANJI_ROUTER_ENDPOINT` 只是指定连接地址，不会启动 router。

### 终端二：连接头显并启动完整仿真

```bash
adb devices -l
adb forward tcp:10002 tcp:10002
adb forward --list

mkdir -p recordings/device_acceptance
TELEOP_TEST_ID=$(date +%Y%m%d_%H%M%S)
RECORDING="recordings/device_acceptance/pico_${TELEOP_TEST_ID}.h5"

TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile pico2_hands_sim \
  --viewer \
  --arm-pose-mapper head_palm_direct \
  --ik-backend pico_ee_dexhand_qp \
  --arm-target-processor passthrough \
  --joint-trajectory passthrough \
  --command-step-clipping false \
  --joint-limit-source urdf \
  --pico-overlay \
  --ik-target-overlay \
  --record "$RECORDING"
```

`adb devices` 中目标头显应为 `device`；`unauthorized` 时在头显内确认授权。
只测试机械臂时，在上述命令追加 `--disable-hands`；完整手部测试不要加此参数。
每次重启重新生成时间戳，录制文件不允许覆盖。

### 标定、开始与结束

1. 等待 `session_startup_complete`，确认可视化中的左右腕和手骨架随真人更新。
2. 佩戴好头显，双臂水平伸直，在启动会话的终端按 `c`，保持双腕稳定约 2 秒。
   看到 `Height calibration succeeded` 后再放松；失败则保持稳定重新按 `c`。
   此标定只修改 Z 高度，不修改 X/Y 映射；重启会话后需要重新标定。
3. 双手保持可识别，在同一终端按 `s` 开始遥操，依次测试腕部运动、张手、握拳和捏合。
   再按 `s` 请求回 Home；按 `q` 回 Home 并退出，等待录制和进程清理结束。

普通张手/握拳默认只作为手骨架输入和手势观察，不自动启动或停止。
若要单独测试手势启动，可追加 `--operator-input gesture`：完成标定后，先释放双手
张开姿态，再双手稳定张开约 0.8 秒触发一次启动请求；键盘 `c/s/q` 仍可用。

视觉短暂丢失且 TCP 仍持续有新帧时，失效侧保持，恢复后沿原映射继续；
TCP 断连或组件异常退出需排查并重启会话。`--joint-trajectory passthrough`
绕过额外关节 Ruckig，v131 内部 Cartesian OTG 仍保留。

退出后可在同一终端检查录制：

```bash
pixi run python scripts/check_dual_recording.py --mode pico --input "$RECORDING"
pixi run python scripts/check_dual_recording.py --mode pico --input "$RECORDING" \
  --retarget-hand-commands
```

当前已通过合成输入 MuJoCo 闭环和专项回归，用户已确认本分支两条路线均可遥操，
包括 Manus 控制舞肌二代手，且手指抖动已有改善。尚缺的是带完整 H5 的正式
长时间验收、Ctrl-C 退出复测及两路切换压力记录。出现问题请保留本次 H5
及终端异常信息。完整步骤见[真实输入→仿真验收操作单](docs/real-input-simulation-acceptance.md)。

## 双输入迁移状态（当前目标与历史兼容）

原有`hand_tracking_sim`及其映射、v131、轨迹选项保持原路径。当前两条设备入口是
`pico2_hands_sim`（PICO2裸手）和`pico_vr_manus_sim`（内嵌原版PICO头显＋双VR手柄＋Manus）。
`vr_manus_xr_sim`仍保留为需要XRoboToolkit/PC-Service的另一种实验入口；旧的
`vr_manus_sim`继续用于外部TJVR数据兼容，但不自动启动旧PICO_tracker。

### 内嵌原版 PICO＋VR手柄＋Manus：`pico_vr_manus_sim`

该入口把参考工程的 `pico_bridge`、M0 掌心/骨架修正和 TJVR bridge 作为源码内嵌到
`vendor/pico_tracker`，使用独立的 Python 3.11/ROS2 Pixi 环境；下游仍复用当前工程
已验证的 `vr_manus_sim` Spark、Manus Hand2、coordinator 和 MuJoCo。不会读取外部
`pico-manus-teleop` checkout，也不会启动 PICO2 的 `10002` 接收器。

以下命令均在工程根目录执行。首次准备主环境、SPARK、官方手环境，或相关源码更新后构建：

```bash
pixi install --locked
pixi install --manifest-path tools/spark_native/pixi.toml --locked
pixi run --manifest-path tools/spark_native/pixi.toml configure
pixi run --manifest-path tools/spark_native/pixi.toml build
pixi install --manifest-path tools/wuji_hand_native/pixi.toml --locked
pixi run build-embedded-pico
pixi run build-hdf5-recorder
```

`build-hdf5-recorder` 需要系统 `g++` 和 `libhdf5-dev`，产物位于
`build/hdf5_recorder/`；VR/Manus 的 `--record` 自动使用 C++ 写入后端，PICO2 不变。
`build-embedded-pico` 只操作 `vendor/pico_tracker` 的独立 Pixi 环境和 `pico_bridge` build/install；
个人标定仍从默认的 `~/.config/pico_tracker` 读取，也可用
`--pico-calibration-dir PATH` 指定包含以下六个文件的目录：
`pico_left_arm_geometry.yaml`、`pico_right_arm_geometry.yaml`、
`pico_left_palm_tcp.yaml`、`pico_right_palm_tcp.yaml`、
`pico_left_wrist_pivot.yaml`、`pico_right_wrist_pivot.yaml`。

#### 可选：人体骨长对称化（VR 手柄路线）

当前内嵌标定菜单在每次骨长 candidate 成功激活后，默认自动执行 `symmetric_max` 收尾。
两侧 TCP、腕心和骨长文件齐全并通过校验后，生成新的只读用途快照
`原始目录/symmetric_profiles/<版本>/`，再原子更新 `原始目录/runtime_symmetric` 链接。
旧版本保留，原始测量文件不改写成对称值。缺件时提示 pending；生成失败时保留旧链接，
并明确提示“本侧原始骨长已保存，但对称运行配置未更新”（退出码 3）。
快照沿用各侧当前有效标定，不表示两侧都是本次重新测量。

在已完成标定所需设备和 PICO 原始数据准备后，按侧执行；TCP、腕心未完成时先使用
`left all` / `right all` 依次完成，而不是直接进行 geometry：

```bash
pixi run --manifest-path vendor/pico_tracker/pixi.toml bash vendor/pico_tracker/scripts/calibrate_pico_arm.sh left geometry
pixi run --manifest-path vendor/pico_tracker/pixi.toml bash vendor/pico_tracker/scripts/calibrate_pico_arm.sh right geometry
```

之后 VR 遥操命令使用固定入口（自定义原始目录时替换路径）：

```bash
--pico-calibration-dir "$HOME/.config/pico_tracker/runtime_symmetric"
```

未选择该目录的遥操仍使用原始配置。若只想保留原始独立骨长，标定命令追加
`--geometry-policy original`，遥操选择原始目录；旧对称快照不会删除或更新。
`status` 只读显示测量及已有快照路径，不重新生成。禁止在派生目录中采集标定。

已有有效双侧标定也可仅执行收尾，不采集、不启动设备：

```bash
pixi run python vendor/pico_tracker/src/pico_bridge/scripts/pico_calibration_finalize.py \
  --source "$HOME/.config/pico_tracker"
```

下面仍保留手动指定新目录的入口，适合独立快照和 A/B 对照：

先完成原始骨长标定，再生成独立派生目录。该策略对上臂、前臂分别取左右最大值，
让 M0 使用同一组长度重建左右人体手臂；不修改硬件驱动、原始标定和质量报告。
最长值不一定是真值，明显异常时仍应重新标定，而不是用该策略代替测量。

```bash
PICO_SYMMETRIC_DIR="$PWD/recordings/calibration_profiles/symmetric_$(date +%Y%m%d_%H%M%S_%N)"
pixi run python scripts/create_pico_symmetric_profile.py \
  --source "$HOME/.config/pico_tracker" \
  --output "$PICO_SYMMETRIC_DIR"
pixi run build-embedded-pico
```

在下方 VR 遥操启动命令中追加 `--pico-calibration-dir "$PICO_SYMMETRIC_DIR"`。
未指定派生目录时保留原行为，PICO2 裸手路线不受此选项影响。
派生目录中的 `pico_geometry_policy.json` 记录原始长度、文件指纹及有效长度；
M0 会打印 `Explicit symmetric_max runtime geometry`，并在状态中标明运行时覆盖。
原始标定文件的副本仍保留测量值，不应把它们误认为有效对称长度。
文件缺失、指纹变化或生成未完成时拒绝加载。不要在派生目录内重新标定；
原始标定更新后另建派生目录。工具拒绝覆盖已有目录。

这只统一人体骨架几何，不保证左右掌心位移或 IK 跟踪误差相同。
骨长策略本身不需要 `--mapped-palm-common-x-reference`；可不带该选项做对照。
`c` 的 X/Z 工作空间对齐仍是独立功能，不由骨长对称化代替。

启动原版 PICO wholebody APK 后，确认设备为 `device`。该路线使用原版 `9999` 端口和
`com.PICO.wholebody_stream.unity`；不要把 PICO2 裸手的 `10002` 转发混用：

单台头显可直接使用 `adb`。如果电脑同时连接多台 Android/PICO 设备，请把同一个
设备 serial 通过 `ADB_SERIAL=...` 或启动参数 `--adb-serial SERIAL` 传给内嵌入口；
supervisor、driver 和 `9999` forward 会绑定到该设备，避免误启动或误转发到其他设备。

```bash
adb devices -l
adb forward --list
```

启动器会启动 wholebody APK 并建立需要的 `9999` 转发，无需手动启动外部
PICO_tracker、M0、TJVR bridge 或 XR PC-Service。头显需已连接左右控制器并正常追踪。

**终端一：启动 router（已有可用 router 则跳过）。** 保持此终端运行：

```bash
./vendor/zenoh-router/zenohd \
  -l tcp/127.0.0.1:7447 \
  --no-multicast-scouting
```

**终端二：启动会话。** 下面选择“仅机械臂”或“完整 Manus”其中一条。

仅测试 PICO 头显＋双VR手柄对 MuJoCo 双臂的控制（不需要 Manus）：

```bash
mkdir -p recordings/device_acceptance
TELEOP_TEST_ID=$(date +%Y%m%d_%H%M%S)
RECORDING="recordings/device_acceptance/pico_vr_arms_${TELEOP_TEST_ID}.h5"

TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile pico_vr_manus_sim \
  --viewer \
  --disable-hands \
  --tjvr-bind 127.0.0.1 \
  --tjvr-port 15000 \
  --spark-overlay \
  --pico-startup-timeout-s 30 \
  --record "$RECORDING"
```

完整 Manus 测试使用以下独立命令块，每次重启都重新生成录制文件名。
把 `MANUS_RAWVIZ` 改为本机实际路径；路径通过参数传入，不要求参考工程位于固定位置。
`--manus-user gjy` 对应 rawviz 同级 `calibration/` 下的
`gjyLeftMetaglovePro.mcal` 和 `gjyRightMetaglovePro.mcal`，换人时使用对应人员标定。

```bash
MANUS_RAWVIZ=/实际路径/manus/rawviz.out
TELEOP_TEST_ID=$(date +%Y%m%d_%H%M%S)
mkdir -p recordings/device_acceptance
RECORDING="recordings/device_acceptance/pico_vr_manus_${TELEOP_TEST_ID}.h5"

adb devices -l

TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile pico_vr_manus_sim \
  --viewer \
  --tjvr-bind 127.0.0.1 \
  --tjvr-port 15000 \
  --manus-rawviz "$MANUS_RAWVIZ" \
  --manus-user gjy \
  --spark-overlay \
  --pico-startup-timeout-s 30 \
  --record "$RECORDING"
```

**开始与结束：**

1. 佩戴好头显、连接双控制器及 Manus dongle，开启左右手套。确认 USB 访问权限可用，
   不要另外启动占用 dongle 的 rawviz 进程；本会话会管理它。
2. 等待 `PICO raw streams ready`、`PICO m0 streams ready`、`PICO bridge streams ready`，
   然后观察下游输入与按键提示；上述 ready 仅说明 PICO 链路就绪，不代表 Manus 已有有效数据。
3. 在启动终端按 `s` 开始。先缓慢移动双控制器检查机械臂，再张手、握拳检查舞肌二代手。
   此路线读取原版个人标定，不使用 PICO2 的 `c` 水平伸臂 Z 标定。
4. 按 `h` 回 Home；需要重置时在健康 Home 状态按 `r`，等待新输入，再按 `s`。
   按 `q` 请求结束，等待清理完毕。使用 Ctrl-C 时也应等待返回提示符，避免连续强杀。

退出后检查同一个 H5 中的机械臂原始输入与手套数据：

```bash
pixi run python scripts/check_dual_recording.py --mode tjvr --input "$RECORDING"
pixi run python scripts/check_dual_recording.py --mode manus --input "$RECORDING"
```

仅机械臂录制只执行 `--mode tjvr`。录制检查不能替代现场观察；若会话自动退出、
按 `s` 无反应或 H5 为 `complete=false`，保留录制及本次日志，先检查 Manus 有效输入、
USB 权限和标定路径。不要删除运行锁来启动第二个会话；确认旧会话退出后重启，
启动器会恢复清理已登记的遗留进程。

### 可选：正式 bandwidth mapped-palm QP＋Manus

新增 `pico_ee_mapped_corrected_palm_velocity_qp` 后端，来源为参考工程
`feature/pico-manus-teleop-experiments` 的正式 bandwidth baseline，tag
`pico-mapped-wuji-hand2-bandwidth-smoothness-baseline-20260906`。
不指定 `--ik-backend` 时，上述 VR/Manus 命令继续使用原 SPARK；PICO2 指令保持不变。

新后端直接使用 corrected skeleton 的 mapped 掌心及肩/肘/腕平面臂角，
经过原版 LF/HF Cartesian 前馈、Headroom、臂角/零空间和约束 Velocity QP。
不执行 SPARK IK 或 SPARK 骨长缩放，但仍需要完整 corrected 上肢骨架。
控制周期 200 Hz，使用 `model_reference` 和 `hand_tcp_frame_L/R`。
原版内部平滑项保留；不额外叠加通用目标整形、关节 Ruckig 或逐步命令裁剪。

首次使用或新后端源码更新后，在工程根目录构建：

```bash
pixi install --manifest-path tools/mapped_palm_native/pixi.toml --locked
pixi run --manifest-path tools/mapped_palm_native/pixi.toml configure
pixi run --manifest-path tools/mapped_palm_native/pixi.toml build
```

前述内嵌 PICO、官方 Hand2、C++ HDF5 依赖和个人标定仍需准备好。
保持 router 终端运行，在另一个终端启动（填写本机 Manus 路径及人员名称）：

```bash
MANUS_RAWVIZ=/实际路径/manus/rawviz.out
TELEOP_TEST_ID=$(date +%Y%m%d_%H%M%S)
mkdir -p recordings/device_acceptance
RECORDING="recordings/device_acceptance/mapped_palm_manus_${TELEOP_TEST_ID}.h5"

adb devices -l

TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile pico_vr_manus_sim \
  --viewer \
  --ik-backend pico_ee_mapped_corrected_palm_velocity_qp \
  --arm-target-processor passthrough \
  --joint-trajectory passthrough \
  --command-step-clipping false \
  --joint-limit-source urdf \
  --tjvr-bind 127.0.0.1 \
  --tjvr-port 15000 \
  --manus-rawviz "$MANUS_RAWVIZ" \
  --manus-user gjy \
  --spark-overlay \
  --pico-startup-timeout-s 30 \
  --record "$RECORDING"
```

`--spark-overlay` 在此仍是共享的 corrected 骨架/机械臂目标显示开关，不代表使用 SPARK IK。
新 mapped 后端显示的是 IK 已采用的 corrected 骨架和实际目标，隐藏未使用的 packet
目标及骨架原始姿态轴；模型自带的目标标记同步到实际目标，不再停留在 XML 默认位置。
未开始/回 Home 时不显示已采用骨架，模型标记跟随当前渲染 TCP；失效骨架会隐藏。
该显示行为仅作用于 mapped 后端，不改变原 SPARK 和 PICO2 的显示或控制流程。
只测双臂时移除 `--manus-rawviz`、`--manus-user` 两项并加 `--disable-hands`。
按键仍为启动终端 `s` 开始、`h` 回 Home、`r` 在 Home 重置、`q` 退出；不使用裸手路线的 `c` 标定。
原版 mapped 后端的输入超时为 50 ms，过期由原版 hold/约束处理；断连或组件故障仍按会话规则处理。

#### 可选：先按 c 做仅 Z 高度标定

原版映射仍是默认。仅在上述 mapped 后端命令中增加
`--mapped-palm-height-calibration`，即可使用独立的高度标定入口；不适用于 SPARK、XR 或
PICO2，不会改变它们的标定/遥操流程。更新后先重新构建该原生 worker：

```bash
pixi run --manifest-path tools/mapped_palm_native/pixi.toml build
```

保持 router 运行，使用 wholebody APK。仅测试 PICO＋VR 手柄、不接 Manus：

```bash
adb devices -l
mkdir -p recordings/device_acceptance
TELEOP_TEST_ID=$(date +%Y%m%d_%H%M%S)

TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile pico_vr_manus_sim --viewer --disable-hands \
  --ik-backend pico_ee_mapped_corrected_palm_velocity_qp \
  --mapped-palm-height-calibration \
  --arm-target-processor passthrough --joint-trajectory passthrough \
  --command-step-clipping false --joint-limit-source urdf \
  --tjvr-bind 127.0.0.1 --tjvr-port 15000 \
  --spark-overlay --pico-startup-timeout-s 30 \
  --record "recordings/device_acceptance/mapped_palm_height_${TELEOP_TEST_ID}.h5"
```

在启动终端按 `c`，双臂水平伸直、手柄稳定保持约 2 秒；看到
`state: calibrated` 的标定结果后按 `s`。采集中或首次未成功时拒绝开始；移动过大、
数据过期或 epoch 变化时本次失败，可重新按 `c`，上次成功结果保留。
运行中不能标定；需先 `h` 回健康 Home。`r` 重置保持已成功的偏移；重新启动会话需重新标定。

它沿用“水平姿势仅校正 Z”的操作方式，并非切换成 PICO2 头相对坐标映射：
目标仍来自当前 corrected 骨架。左右偏移分别为当前模型水平 TCP 高度减去采样掌心高度；
X/Y、姿态、比例和肩肘腕臂角保持原样，不会让机器人真的伸臂执行标定。
HDF5 保留原始 TJVR，另记录标定事件、Home reset ack 和原生周期的
`target_height_offsets_m`；这条校准路线不声明与原版默认目标数值等价。
青色 `Applied corrected` 骨架保留原始已应用点位，不随高度标定整体移动。
紫色 `Z calibrated palm position` 单独显示加 Z 偏移后的掌心位置；黄色
`Mapped-palm IK target` 显示 native 输出的实际 IK 目标。前两者仅在输入新鲜时显示。
高度标定仍会改变实际掌心目标 Z；原始采集和臂角输入不变，不能视为与原版输出完全等价。
启动终端仅显示运行摘要，完整配置和资源哈希保存在录制的 metadata 中。
VR 会话可视化刷新上限为 60 Hz，机械臂控制仍按 200 Hz 调度。
退出报告的 `native_ticks` 是最后一轮 reset 后的计数，`native_ticks_total` 是全程累计；
`late_cycles` 是错过计划截止时间的次数，并非 IK 失败次数。
`timing` 分别统计输入/健康检查、控制 step、快照及整个周期工作的耗时，单位 ms。
`exit_trigger` 区分信号、关闭窗口、时长结束及操作退出；只有按 `q` 完成回位才报告
`home_return_completed=true`。Ctrl-C 直接停止会话，不表示已回 Home。
移除新增选项即回到原版映射。加入 Manus 时移除 `--disable-hands` 并填写前述 Manus 参数。

录制检查继续使用 `check_dual_recording.py --mode tjvr` 和 `--mode manus`。
软件已完成原版短轨迹逐周期对照和合成 Manus＋官方 Hand2＋C++ HDF5 的仿真联调；
新后端的真实 PICO＋手柄＋Manus 现场验收待执行。
详见 [mapped-palm 移植验证记录](docs/mapped-palm-porting.md)。

VR/TJVR 两个原生 IK 后端可在原启动命令上追加 `--native-result-format binary`，
试用 C++ 固定布局周期结果；默认仍为 `json`。先重新构建所选后端的 native worker。
该选项保留完整诊断和标定/复位行为，不适用于 PICO2 裸手入口。
目前仅完成周期结果传输，Python 控制调度尚未迁移；离线首轮平均收益有限。
构建、测量方式与后续范围见 [C++ 控制迁移记录](docs/native-control-migration.md)。

另可运行 `pixi run build-native-control`，然后在 VR/TJVR 命令追加
`--execution-guard cpp`，将每周期的执行回执检查交给 C++；默认仍为 `python`。
此选项可独立使用，保留身份/超时/双臂一致性/故障锁存与 Home/rearm 语义，
不适用于 PICO2 裸手入口。它尚未迁移整个协调器和控制调度；局部提速不代表整条遥操提速。

MuJoCo 关节应用与反馈读取也提供可选 C++ 实现：先运行
`pixi run build-native-mujoco`，再在上述 VR/TJVR 命令追加 `--simulation-backend cpp`。
默认仍为 `python`；它可与 `--execution-guard cpp`、`--native-result-format binary`
独立组合。C++ 使用同一 MuJoCo 模型/TCP，执行批量关节写入、`mj_forward` 和反馈复制；
Viewer、控制调度和授权状态机尚未整体迁移。当前离线输出对照通过，但未证明整链路
提速，也未做本选项的真实输入验收；PICO2 裸手继续使用原路径。更新 pixi 环境后需重新构建。

协调器关节命令数值模块可追加 `--coordinator-math cpp`（先重新运行
`pixi run build-native-control`，默认 `python`）。迁移范围为硬限位、步长/时间窗检查、
可选裁剪和 Home 插值；会话状态机、身份/新鲜度门控、故障保持及发布权仍由原协调器管理。
仅支持 VR/TJVR，与上述三个选项独立组合，不是完整 C++ 调度器。离线数值对照结果及
正常壁钟预算下的差异说明见 [迁移记录](docs/native-control-migration.md)。

官方 Hand2 低通滤波也提供可选 C++ 实现，两条启用手部的仿真路线均可使用：
先运行 `pixi run build-native-hand`，再在原启动命令前添加环境变量
`TIANJI_HAND_FILTER_BACKEND=cpp`（与 `TIANJI_ROUTER_ENDPOINT=...` 同级）。
移除该变量或设为 `python` 即使用原实现。缺少库或加载失败会报错，不静默回退。
此选项只替换每侧 20 关节低通滤波，保留原系数、单精度舍入、丢帧 reset、限位及关节顺序；
此滤波选项本身不替换优化求解器。驱动/SDK 均不变，尚未做该选项的真实设备验收，不宣称整链路提速。
启用 C++ 时录制元数据会记录滤波库、源码与适配器摘要。

手骨架坐标预处理可独立添加 `TIANJI_HAND_GEOMETRY_BACKEND=cpp`，也可与上述滤波变量
组合。先重新执行 `pixi run build-native-hand`（需要项目 `ik-build` 或 Hand2 环境中的
Eigen 头文件）。此选项迁移轴反射、SVD 腕坐标系、左右手坐标变换、配置旋转和指根偏移；
此几何选项本身不替换优化求解器。配置在启动时固定，库加载失败不回退；腕/食指根/中指根三点共线等
退化输入会显式报错，而不采用不唯一的坐标轴。默认仍为 `python`，移除变量即可恢复。
录制元数据包含 `hand_geometry` 实现摘要；离线对照不等于真实输入验收或整链路性能保证。

Hand2 优化器现在另有独立 C++ 选项，**原版 Python 源码及默认入口保留不变**：

```bash
pixi run build-native-hand-optimizer
```

需要已安装项目 `ik-build` 和 `tools/wuji_hand_native` 的锁定环境。然后在原来的
PICO2 或 VR＋Manus 启动命令前添加 `TIANJI_HAND_OPTIMIZER_BACKEND=cpp`，与 router
环境变量同级；移除它或设为 `python` 即恢复原版。它独立于上述几何/滤波开关，不会自动
启用其他 C++ 选项。当前仅支持原 `AdaptiveOptimizerAnalytical` 的 20-DOF Hand2 配置。
C++ 执行 FK/Jacobian、目标与解析梯度、SLSQP 回调及 warm-start；保留原参数、50 次评估、
`ftol_abs=1e-4`、float32 输出和 reset 语义。缺少库、ABI/配置错误会拒绝启动，不静默切回 Python。
库只在隔离手部 worker 中加载；HDF5 元数据记录 `hand_optimizer` 实现/依赖锁摘要。
已有两路录制的离线数值对照通过，但尚未进行该新选项的真实设备验收。Python worker、
输入封装及整体现场调度仍未全部迁移，不能把这个选项当作完整 C++ 现场运行时。

#### 可选：C++ Hand2 worker + 固定频率调度链

如果要把驱动输出之后的 Hand2 几何、优化、滤波、限位、关节排列和固定频率调度放进同一
C++ 子进程，可显式启用：

```bash
pixi run build-native-hand-scheduler
# 在启用 hands 的 pico2_hands_sim、pico_vr_manus_sim 或 vr_manus_sim 命令中追加：
--hand-scheduler-backend cpp
```

该选项使用固定 little-endian `TJHS/TJHI/TJHO` 协议，C++ scheduler 自己维护 session、
epoch、generation、新鲜度、latest-input、输出队列和故障锁存；Python 只提交已校验的输入，
由独立 reader 消费固定频率结果，提交不会等待 Hand2 求解。PICO 双侧只接受同一输入序列的
匹配结果，VR/Manus 结果在控制 loop 没有新回调时也会被消费。PICO/Manus 原有驱动、SDK、
原始输入适配、Python 授权发布及 MuJoCo 手部 executor 保持不变。它不能与
`--hand-worker-backend cpp` 同时使用。默认仍是 Python，缺少原生产物会在启动前报错，不会
静默回退；启用时录制 metadata 会保存 scheduler/optimizer、协议 ABI 及源码摘要。这样是
手部 worker/调度子链的 C++ 化，不等于整个现场 session、输入解析或 HDF5 热路径已经
全部 C++ 化；原版 Python 实现仍可直接用于 A/B 对照。

原生异步手部链路使用会话输入序号屏障隔离旧结果；PICO 左右手每次共同处理一帧，
期间只保留最新等待帧。过期帧不生成有效关节命令，worker 超时或读取失败会上报故障。
该入口已增加离线边界回归，真实设备性能和长时间稳定性仍需单独测试。

### C++ 组合链路现场试用：PICO＋VR 手柄＋Manus

更新代码后构建（本机本轮已构建）：

```bash
pixi run build-native-hand-scheduler
pixi run build-native-control
pixi run build-native-mujoco
```

沿用上文 router、wholebody APK 和 Manus 标定准备。下面启用原生手部调度器、
SPARK 二进制周期结果、C++ 执行检查、协调器数值模块及 MuJoCo 关节接口：

```bash
MANUS_RAWVIZ=/实际路径/manus/rawviz.out
TELEOP_TEST_ID=$(date +%Y%m%d_%H%M%S_%N)
mkdir -p recordings/device_acceptance

TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile pico_vr_manus_sim \
  --viewer \
  --manus-rawviz "$MANUS_RAWVIZ" \
  --manus-user gjy \
  --tjvr-bind 127.0.0.1 \
  --tjvr-port 15000 \
  --hand-scheduler-backend cpp \
  --native-result-format binary \
  --execution-guard cpp \
  --simulation-backend cpp \
  --coordinator-math cpp \
  --spark-overlay \
  --pico-startup-timeout-s 30 \
  --record "recordings/device_acceptance/cpp_vr_manus_${TELEOP_TEST_ID}.h5"
```

启动摘要应包含 `hand_scheduler=cpp; simulation=cpp`。在启动终端按 `s` 开始，
`h` 回 Home，`q` 回 Home 后退出；Ctrl-C 直接停止，不表示回位完成。
这是可选原生模块组合，现场会话接线仍含 Python。完整原生主调度入口见下节，
不要直接与本命令的四个机械臂适配选项叠加。也不要加 `--disable-hands` 或
`--hand-worker-backend cpp`，它们与本测试的原生手部调度器不兼容。

PICO2 裸手试用继续采用前文 `pico2_hands_sim` 完整命令，仅追加
`--hand-scheduler-backend cpp`；上述四个机械臂适配选项仅用于 VR/TJVR 路线。
首次试用建议完成静止手指、逐指弯曲、双手同时动作、`h → r → s`，最后按 `q` 保存录制。
原始输入保持录制；状态诊断中的 `coalesced_output_callbacks` 表示应用端未采用的中间结果，
不表示优化器从未处理该输入。`pending_output_callbacks` 是仍待关联的输入，
`unresolved_output_callbacks` 是退出时未得到可关联结果的输入。

#### 实验选项：Manus 原生解析器

在原有 **启用手部** 的 `pico_vr_manus_sim` 或 `vr_manus_sim` 命令中追加
`--manus-parser-backend cpp`，先运行：

```bash
pixi run build-native-hand
```

启动摘要包含 `manus_parser=cpp`。它只替换 rawviz 输出之后的解析、语义 21 点选择和
双手回调组装；rawviz、SDK、USB 配置、接收时间戳和原始行录制不变。
可与 `--hand-scheduler-backend cpp` 组合。省略选项或设为 `python` 保留原解析器。
不能与 `--disable-hands` 组合，也不是给 PICO2 裸手入口使用的参数。

保留 float32、Y 翻转、右手在前、重复帧拒绝、失效后等待新帧及显式手套绑定。
原生解析资源有界：单行 65536 字节、64 个手套流、每流 512 个元数据节点；超界明确失败，
不静默丢掉记录。支持正常 rawviz ASCII 数值协议，源序号/时间为 int64；SDK 发布时间等
未参与映射的整数元数据保留在原始记录中。二进制及源码哈希写入会话来源记录。
这个参数用于 Python 主调度器中的可选解析模块；下节原生联合主调度器内置 C++ Manus
解析，不需要此 ctypes 解析器选项。PICO2 和 Python 路线默认行为不变。

#### 实验入口：完整 C++ VR＋Manus 主调度链路

内嵌 PICO bridge 的世界 X 偏移可用 `--pico-world-x-offset-m 0.20` 覆盖，
表示总偏移 0.20 m（比默认 0.10 m 增加 0.10 m），不是在默认值上再叠加 0.20 m。
仅用于 `pico_vr_manus_sim`，接受 [-1,1] m 的十进制数，重启生效。
它在 PICO 世界坐标系中应用，经过映射后不一定等于机器人基坐标系 X 的同向平移；
不改变 Z 标定。省略参数保持原版默认值，不改变 PICO2 裸手路线。

Python 与 C++ 的 VR＋Manus **现场仿真入口**统一从 `src/tianji_teleop/config/robot/arm.yaml`
读取 Home：左臂 `[55,-65,-70,-60,60,0,0]°`，右臂 `[-55,-65,70,-60,-60,0,0]°`。
启动姿态、`h` 返回姿态及原生 IK 内部初始／重置状态使用同一组值。
SPARK、mapped-palm 的原版参考 YAML 和离线原版对照默认初始姿态不修改。
因此现场启动姿态已不等同于原版默认姿态；Home 也不是关节全零。

Manus 联合入口按 **200 ms 接收时间间隔**维护手部滤波／优化器连续性：
调度器正常跳过旧帧不会重置历史，超过该间隔及显式 epoch 重置才清空。
原始输入序号仍用于关联、发布及录制；左右手分别维护连续性。
此设置仅由原生 Manus 联合入口启用，PICO2 和独立 worker 默认行为不变。
更新后执行 `pixi run build-native-hand-scheduler` 并重启会话生效。

SPARK 的同 epoch 输入恢复策略默认保持原版。若跳变门控恢复后反复进入固定目标
接管，可在下方 C++ 启动命令追加 `--spark-resync-policy resume`：从当前模型状态
继续追踪，首次接管及 tracking epoch 变化仍执行原版接管。省略该参数或使用
`--spark-resync-policy reference` 可作原版对照。仅支持 C++ 调度器＋SPARK IK，
不适用于 mapped-palm 或 PICO2 裸手；不放宽输入新鲜度、跳变或关节约束。
更新此功能需先执行 `pixi run --manifest-path tools/spark_native/pixi.toml build`。
该选项的软件测试不替代真实输入体验及安全验收。

软件离线组合测试已通过（SPARK、mapped-palm 两种 IK）；**真实 PICO＋VR＋Manus 联合
验收尚未完成**。只用于 MuJoCo，不控制真机。保留上文 Python 入口用于回归。

先完成上文 wholebody APK、router、PICO 标定和 Manus 人员标定准备，再构建：

```bash
pixi run build-native-hand-scheduler
pixi run build-hdf5-recorder
pixi run build-native-session-gateway
```

```bash
MANUS_RAWVIZ=/实际路径/manus/rawviz.out
TELEOP_TEST_ID=$(date +%Y%m%d_%H%M%S_%N)
mkdir -p recordings/device_acceptance

TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile pico_vr_manus_sim \
  --viewer --viewer-backend cpp \
  --scheduler-backend cpp \
  --hand-scheduler-backend cpp \
  --publication-backend cpp \
  --recording-adapter cpp \
  --manus-rawviz "$MANUS_RAWVIZ" --manus-user gjy \
  --ik-backend spark_upper_qpoases_headroom_feedforward_velocity_qp \
  --arm-target-processor passthrough --joint-trajectory passthrough \
  --command-step-clipping false --joint-limit-source urdf \
  --tjvr-bind 127.0.0.1 --tjvr-port 15000 \
  --spark-overlay --pico-startup-timeout-s 30 \
  --record "recordings/device_acceptance/native_joint_${TELEOP_TEST_ID}.h5"
```

摘要应显示 `scheduler=cpp`、`hands=True`，并使用原生发布、录制和 Viewer。
`s` 接管；`h` 回 Home，自动重置成功且新输入有效后可再 `s`；`r` 保留手动重置；
`q` 回 Home 并排空录制后退出。启动时要求双手结果年龄不超过 50 ms（且受原新鲜度配置约束）；
不要求处理序号恰好等于持续到来的最新输入。结果尚未就绪时，`s` 会被拒绝，等待后重试。
双手均需有效，不支持此入口单手降级；禁止叠加 `--disable-hands` 或旧四个机械臂适配选项。

此入口原生处理 TJVR/Manus 解析、Hand2、控制调度、MuJoCo、发布及逐帧录制；Python
仍负责冷启动、schema 创建、进程监督和终端按键。PICO driver/M0/TJVR 发送、rawviz/SDK
及所有硬件驱动不变。控制计算只保留最新等待样本，不中断正在计算的帧；原始回调仍全部录制，
因此计算结果序号允许跳过中间输入。联合录制使用有界批次，不丢弃原始行；异常退出不能标记完整。

#### mapped-palm：双臂前伸 X/Z 标定（C++）

**PICO＋VR 手柄、只测机械臂的快捷入口：**设备和依赖已准备好、Zenoh router 已运行时，
在工程根目录直接执行：

```bash
bash scripts/run_pico_vr_arms_cpp.sh
```

此脚本复用本节已测的 C++ mapped-palm、X/Z 标定、PICO 世界 X 总偏移 0.20 m，
关闭 Manus 和手部控制；权重读取当前 `bandwidth.yaml`（当前位置惩罚 90,000）。
默认加载 PICO 标定目录，不额外指定 `runtime_symmetric`，不启用左右 X 取小值。
每次自动在 `recordings/device_acceptance/` 新建录制。可用 `--record-dir PATH` 改保存目录，
相对路径以工程根目录为基准；`--dry-run` 只查看命令，不创建录制或连接设备。
脚本不自动构建、不启动 router，也不修改标定、ADB 配置或系统权限；
实际头显启动和端口转发仍由原有内嵌会话管理器处理。首次构建步骤见下方。
`TIANJI_ROUTER_ENDPOINT` 可指定已有 router，未设置时使用 `tcp/127.0.0.1:7447`。
没有 router 时先在另一终端运行：

```bash
./vendor/zenoh-router/zenohd -l tcp/127.0.0.1:7447 --no-multicast-scouting
```

点击 MuJoCo 窗口：双手水平前伸按 `c`、保持约两秒，成功后按 `s`；`h` 回 Home，
`r` 手动重置，`q` 回 Home 并保存退出。此快捷脚本只用于 MuJoCo，不控制真机。

2026-09-15 现场更新：真实 PICO＋VR 手柄、仅机械臂的 C++ 仿真已测试，
用户反馈位置权重 **90,000** 的效果不错。此次 `c` 标定、`s` 接管、回 Home 和正常退出
均收到成功结果，HDF5 `complete=true`，退出码 0。
本轮没有 Manus 输入，不代表 90,000 配置的联合手套、长时稳定性或真机验收完成。

当前配置 `src/tianji_teleop/src/ik/mapped_palm/config/bandwidth.yaml`：

| 参数 | 当前值 | 原版 baseline |
| --- | ---: | ---: |
| `hierarchical_qp.slack_weight_position` | 90,000 | 30,000 |
| `hierarchical_qp.slack_weight_orientation` | 30,000 | 30,000 |
| `slack_position_scale` / `slack_orientation_scale` | 1 / 2 | 1 / 2 |

这是明确的本地调参，不再宣称该配置与原版完全一致；来源清单保留原版哈希及修改记录。
仅位置松弛惩罚改变，其他 bandwidth 参数未变；配置在启动时加载，重启生效。
更高权重不保证所有动作更准确，限位、不可达目标和姿态取舍仍然存在。

本机默认 `~/.config/pico_tracker` 原始骨长 YAML 已按用户要求直接修改：
左右上臂均为 `0.2763150140848072 m`，前臂均为 `0.24751718659563757 m`。
所以本次不需要 `--pico-calibration-dir`，也没有启用 `--mapped-palm-common-x-reference`。
这是本机外部配置，不随 git 分发；其他电脑需准备自己的有效标定。
重新进行骨长测量可能改写原始长度：自动对称化收尾生成的是独立快照，不回写原始 YAML，
不能据此保证重新标定后的默认目录仍然对称。

本次录制：`recordings/device_acceptance/native_arms_position90k_20260915_060300_742965540.h5`，
run ID `c4b114e5-6fac-478d-b475-1cabe8f164d8`。退出摘要为 7,768 个控制周期、21 个 late cycles；
`real_time_qualified=false`，不视为硬实时保证。此前一次 60,000 测试因电脑卡死后手动重启，
录制未完整关闭，不应作为完整对照；卡死原因尚未确定，不能归因于权重。

使用 PICO＋VR 手柄＋Manus 控制 MuJoCo 天机机械臂和舞肌二代手，
不控制机器人真机。头显运行 VR 手柄路线的 wholebody APK，不是裸手路线的 APK。
先完成上文设备、SDK、PICO/Manus 标定准备；首次使用或更新代码后构建：

```bash
pixi run --manifest-path tools/mapped_palm_native/pixi.toml build
pixi run build-embedded-pico
pixi run build-native-hand-scheduler
pixi run build-hdf5-recorder
pixi run build-native-session-gateway
```

终端一：启动 router，已有可用 router 时不要重复运行。

```bash
./vendor/zenoh-router/zenohd -l tcp/127.0.0.1:7447 --no-multicast-scouting
```

终端二：在工程根目录执行完整启动命令。`MANUS_RAWVIZ` 改为本机实际路径；
本机此前使用 `/home/zj/current_robotics/pico-manus-teleop/manus/rawviz.out`。

```bash
adb devices -l
MANUS_RAWVIZ=/实际路径/manus/rawviz.out
TELEOP_TEST_ID=$(date +%Y%m%d_%H%M%S_%N)
mkdir -p recordings/device_acceptance
RECORDING="recordings/device_acceptance/native_joint_mapped_xz_${TELEOP_TEST_ID}.h5"

TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile pico_vr_manus_sim \
  --viewer --viewer-backend cpp \
  --scheduler-backend cpp --hand-scheduler-backend cpp \
  --publication-backend cpp --recording-adapter cpp \
  --manus-rawviz "$MANUS_RAWVIZ" --manus-user gjy \
  --ik-backend pico_ee_mapped_corrected_palm_velocity_qp \
  --mapped-palm-xz-calibration \
  --arm-target-processor passthrough --joint-trajectory passthrough \
  --command-step-clipping false --joint-limit-source urdf \
  --tjvr-bind 127.0.0.1 --tjvr-port 15000 \
  --pico-world-x-offset-m 0.20 \
  --spark-overlay --pico-startup-timeout-s 30 \
  --record "$RECORDING"
```

本命令沿用此次测试的上游 PICO 世界 X **总偏移 0.20 m**，不是再加 0.20 m；
它和标定得到的机器人目标 X 偏移不是同一参数。改变上游偏移后需重启并重新标定。

如需双臂使用同一个标定 X 基准，在上述命令追加
`--mapped-palm-common-x-reference`（必须同时启用 `--mapped-palm-xz-calibration`）。
取左右机器人参考 TCP 的 `X0=min(Xref_left,Xref_right)`，后续分别使用
`Xtarget_left=X0+(Xhuman_left-Xcal_left)`、`Xtarget_right=X0+(Xhuman_right-Xcal_right)`。
因此在标定采样均值处，双臂期望 X 相等；左右补偿量仍可不同。Z、Y、姿态和 Home 不变。
窗口会显示 `X reference: shared minimum robot TCP X`。省略此选项保留原独立参考。
当前模型左右参考 X 本就接近，效果可能与原模式接近；不保证实际 TCP 无跟踪误差，
也不进行骨长尺度归一化。重启并重新按 `c` 后生效。
这里不需要裸手路线的 `adb forward tcp:10002 tcp:10002`。
仅测试机械臂时，删除 `--manus-rawviz`、`--manus-user`、
`--hand-scheduler-backend cpp`，追加 `--disable-hands`，其余参数不变。

看到 `Teleop ready`，确认摘要包含 `calibration=XZ`、`scheduler=cpp`、
`publication=cpp`、`recording_adapter=cpp`、`viewer=cpp`；联合模式应为 `hands=True`。
点击 MuJoCo 窗口使其获得键盘焦点，按以下顺序操作：

C++ Viewer 左上角会显示标定状态与最近一次按键回执：`SAMPLING` 为采样中
（显示左右样本数），`Waiting for reset / calibration ACK` 为等待确认，
`SUCCESS` 后才尝试按 `s`。`FAILED` 后显示具体原因；若保留旧标定，会提示
`Previous calibration retained`。`Last action ... REJECTED` 表示按键已收到但条件未满足。
窗口使用英文提示；`tracking identity/epoch changed` 表示采样期间追踪身份或 epoch 变化，
应保持追踪稳定后重新标定，不是 QP 求解失败。更新状态栏后需重新构建并重启会话。

- 在 Home 且输入正常时，双手水平向前伸直，按 `c` 并保持稳定约 2 秒。
- 参考关节为左右臂均 `[0,-90,0,0,0,0,0]°`，通过当前 mapped-palm 模型的
  `hand_tcp_frame_L/R` 正运动学计算参考 X/Z；不是改变 Home。
- 左右分别计算 `参考 TCP X/Z − 采样掌心 X/Z`。只加 X/Z 常量偏移，Y 和姿态不变；
  不修改 PICO 原始骨架，也不重复叠加上游世界 X 偏移。
- `c` 不驱动机械臂伸直；标定确认完成且新输入有效后按 `s` 接管。
  `h` 回 Home 后保留标定，自动重置成功后可再 `s`；`r` 仍可手动重置。
- `q` 回 Home、排空录制并退出。每次重新启动使用新录制文件，并重新按 `c` 标定；
  正在遥操时不要按 `c`，需先按 `h` 等待回 Home。
- 该参考只对齐位置，**不强制 IK 解等于参考关节角**；伸直附近仍可能受奇异性、
  姿态目标和关节约束影响，需实际输入仿真验收。

此模式要求 `--scheduler-backend cpp --publication-backend cpp --viewer --viewer-backend cpp
--recording-adapter cpp --record <新文件>`。原 `--mapped-palm-height-calibration` 仍为 Z-only，
两者互斥；不加标定参数保持原映射。PICO2 裸手、SPARK、Python 入口及硬件驱动不变。
录制的 native cycle 增加 `target_x_offsets_m`，原始 TJVR 不变。

2026-09-15 已确认真实 PICO/双 Manus 接入及全 C++ X/Z 会话启动成功。
此前 28 项回归和三种配置的离线联合流程通过。真实输入现场结果见本节顶部的
2026-09-15 记录；该仅机械臂测试不替代所有配置、联合手套及长时验收。

#### 实验选项：C++ 双臂调度器的原生消息发布

仅用于 VR/TJVR、`--disable-hands --scheduler-backend cpp` 路线。先构建：

```bash
pixi run build-native-session-gateway
```

在已有 **C++ 仅机械臂**测试命令中追加 `--publication-backend cpp`，启动摘要应显示
`scheduler=cpp; publication=cpp`。消息编码和 Zenoh put 在独立 C++ 有界输出队列的消费线程
执行，与 Python 快照通道解耦，不在控制线程执行；Python 不再重复发布这些机械臂主题。
任一路溢出或输出失败会结束会话；成功完成回执须等待所有消费者排空。
保持显式 `TIANJI_ROUTER_ENDPOINT`。旧可执行文件没有对应启动确认时会拒绝启动，不能
仅更新 Python 文件而不重新构建。省略该参数或设 `--publication-backend python` 保留原发布路径。

仅选择这个选项时仍不是全链路 C++：默认由 Python 处理录制适配、显示和键盘。
完整手部需使用上节的显式联合配置。PICO2 与 Python VR+Manus 默认命令不变。已完成离线字段、
故障回归，未完成新发布选项的真实输入仿真验收。
同录制局部性能对照及剩余工作见 [C++ 迁移记录](docs/native-control-migration.md)。
联合重置必须等待机械臂和手部 worker 都确认后才推进 epoch；PICO2 独立左右手的
完整原生主会话接线不包含在这个 VR＋Manus 入口内。

#### 实验选项：C++ 双臂调度器的原生录制接线

仅用于 VR/TJVR、`--disable-hands --scheduler-backend cpp` 路线，需 `--record`。
先构建：

```bash
pixi run build-hdf5-recorder
pixi run build-native-session-gateway
```

在已有 C++ 仅机械臂测试命令中追加 `--recording-adapter cpp`，启动摘要应显示
`recording_adapter=cpp`。原始 TJVR、完整周期、操作结果和录制生命周期进入独立 C++
录制队列；Python 仅创建 schema、移交文件描述符和管理进程，不重复逐帧录制。
可与 `--publication-backend cpp` 组合。省略选项或设为 `python` 保留现有录制接线
（现有接线的磁盘写入器本身也可能是 C++，两者不要混淆）。

文件独占创建，不覆盖历史记录；队列/写入/关闭失败不能标为完整。
已完成两种 IK 网关的离线合成输入与 Home 退出录制验证，尚未完成真实设备长时验收
及录制端到端性能对照。联合手部按上节配置启用；所有驱动不变。

#### 实验选项：C++ 双臂调度器的原生窗口

先运行 `pixi run build-native-session-gateway`，构建环境需有 GLFW 开发头文件和链接库，
运行环境需有可用图形显示。在已有 C++ 仅机械臂命令中追加：

```bash
--viewer --viewer-backend cpp --spark-overlay
```

仅机械臂使用 `--scheduler-backend cpp --disable-hands`；联合手部使用上节完整配置。摘要包含 `viewer=cpp`，
Python 不再创建 MuJoCo 窗口。默认仍为 `--viewer-backend python`；该参数不用于 PICO2 裸手入口。
窗口中按 `s/h/r/q` 通过同一状态机启动、回 Home、手动 rearm 和回 Home 后退出；开启高度标定时
支持 `c`。关闭窗口也请求正常 Home 退出；Ctrl-C 仍是直接停止。终端按键继续可用。
左键拖动旋转视角，右键拖动平移，滚轮缩放。

C++ 调度器更新后须重新运行 `pixi run build-native-session-gateway`。
`h` 回到精确 Home 后会自动执行原生重置；看到 `auto_rearm` 的 `accepted: true` 后，
保持输入有效即可再按 `s` 接管，无须先手动按 `r`。重置不会自动开始运动。
`r` 仍保留为 Home 下的手动重置，自动重置成功后也可使用；重置期间不重复启动重置事务。
输入不新鲜时自动重置会等待满足条件；重置失败进入故障，不绕过保护。
此行为仅用于 C++ 调度器，Python 默认路线不变。

独立 C++ model/data 消费最新显示快照，不修改执行器关节。骨架保持原世界坐标，
mapped-palm 仅显示实际采用的输入骨架，Z 标定点单独显示，不给整条骨架增加偏移。
显示不可用时明确报错，不静默回退。可与 `--publication-backend cpp --recording-adapter cpp`
组合；三项原生时 Python 不再重建完整周期快照，但仍管理进程、身份和接收诊断协议。
当发布和窗口均为 cpp，且未录制或录制接线也为 cpp 时，自动协商
`diagnostics=summary`：Python 只接收约 10 Hz 状态摘要，状态/epoch 变化立即通知，
退出前补发最终计数；操作回复、故障和完成回执不节流。原始包、完整 IK 周期仍交给
原生录制/发布消费者。任一消费者仍需 Python 时保留完整协议，旧二进制不支持摘要握手则拒绝启动。
**不是全链路零 Python**，也不代表完整 Manus 主调度已迁移。
当前已完成离线几何对照、按键/状态机和无显示失败测试；实际 GLFW 画面、真实输入组合
验收及端到端性能尚待验证。

**已知限制（2026-09-13）：** mapped-palm 已完成真实 PICO＋VR 手柄的仅机械臂仿真测试，
包括 Z 标定、`s` 启动、`q` 回 Home 和完整录制。大幅转腕时仍可能出现明显位置误差；
同一转腕输入的原版独立回放也复现了该现象，两边逐周期输出对照通过。
这不是“固定位置、只转姿态”模式，不能以求解成功代替跟踪精度验收。
详情见 [转腕对照报告](docs/mapped-palm-rotation-comparison.md)。
新后端＋真实 Manus 的完整现场验收、实时性能和真机控制仍未通过本轮验证。

### PICO VR 多设备与启动检查

多设备示例（将 `PICO_SERIAL` 替换为 `adb devices -l` 中的实际 serial）：

```bash
PICO_SERIAL='设备serial'
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile pico_vr_manus_sim \
  --viewer \
  --disable-hands \
  --adb-serial "$PICO_SERIAL" \
  --tjvr-bind 127.0.0.1 \
  --tjvr-port 15000 \
  --spark-overlay
```

supervisor 按 `adb/APK → driver → raw readiness → M0 → M0 readiness → TJVR → 当前下游`
顺序启动；退出或异常时反向清理，并只移除本次自己创建的 `tcp:9999`。`--resolve-only`
可只检查 contract，不触碰设备或进程：

```bash
pixi run bash scripts/run_session.sh \
  --profile pico_vr_manus_sim --disable-hands --resolve-only
```

PICO2 路线使用前文“PICO2 裸手遥操快速测试”的 `pico2_hands_sim` 命令和 `tcp:10002`，两条入口不能
同时启动；切换前先退出当前会话。

### 历史兼容入口：`vr_manus_sim`（非当前目标）

以下入口依赖外部PICO_tracker输出TJVR修正上肢数据；当前PICO＋手柄＋Manus路线
请跳到上面的`pico_vr_manus_sim`章节，不需要启动外部Tracker。

```bash
# 首先按 docs/spark-porting.md、docs/wuji-hand-porting.md 构建独立依赖。
# 由操作者设置 Manus 参考工程根目录，并启动外部 router、原版 PICO_tracker
# （默认 TJVR UDP 127.0.0.1:15000）后：
export PICO_MANUS_TELEOP_ROOT="${PICO_MANUS_TELEOP_ROOT:?set PICO_MANUS_TELEOP_ROOT before running}"
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh --profile vr_manus_sim --viewer \
  --manus-rawviz "${PICO_MANUS_TELEOP_ROOT}/manus/rawviz.out" \
  --manus-user gjy \
  --spark-overlay
```

人员名称须对应rawviz目录下左右手的`.mcal`标定。只测机械臂时，删除两个
Manus参数并加`--disable-hands`。可追加`--record recordings/device_acceptance/vr_manus_TIMESTAMP.h5`，
拒绝覆盖。按`s`显式启动、`h`回Home、`r`在健康Home联合重置后等待新输入及
再次`s`；故障锁存仍需重开会话。

新`pico2_hands_sim`已接通PICO2裸手输入、现有v131双臂IK和官方舞肌二代手retarget。
它只启动一个PICO接收器，不启动Manus或旧几何retarget手节点；MuJoCo同时拥有机械臂和手的仿真反馈。
先启动外部router、在PICO2中运行原有手追踪APK，并确认ADB连接：

```bash
adb devices
adb forward tcp:10002 tcp:10002
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile pico2_hands_sim --viewer \
  --arm-pose-mapper head_palm_direct \
  --ik-backend pico_ee_dexhand_qp \
  --arm-target-processor passthrough --joint-trajectory passthrough \
  --command-step-clipping false --joint-limit-source urdf \
  --pico-overlay --ik-target-overlay
```

需先按`docs/wuji-hand-porting.md`构建官方手独立环境。只测双臂可加`--disable-hands`。
默认映射为`relative_home`；也可显式选择`--arm-pose-mapper head_palm_direct`，沿用原有仅Z标定操作。
PICO键盘操作沿用原target source：`s`启动/回Home，`q`退出；与VR入口的`h/r`操作不同。
可加`--record recordings/device_acceptance/pico2_TIMESTAMP.h5`，保存schema 1.2原始26点、关节命令/反馈及解析配置。
新PICO入口还发布只读open/pinch/fist/unknown几何手势观察，并写入HDF5的`meta/dual_audit`。
默认动作绑定关闭，普通张手/抓握不会自动启动或停止；几何标签不代表真实手型准确率已验收。
仅新PICO仿真可显式加`--operator-input gesture`：armed状态下先释放双手张开姿态，再双手稳定张开0.8秒，
只发送一次start请求，仍须通过原有输入/标定及coordinator授权检查；长按不重复，被拒绝不排队重试。
键盘`s/c/q`仍保留，手势不会执行暂停/回Home/标定；TCP重连后需重开会话，不沿用旧手势参考。
已授权且TCP持续有新帧时，视觉丢失侧保持，恢复沿原映射继续；断连、权限丢失或处理故障仍停止。

`vr_manus_xr_sim`是本工程保留的 XR controller-only 实验入口（不是当前默认的内嵌原版
PICO driver/M0/TJVR 路线）：它只把头显和两个控制器作为 XR 输入，控制器负责
start/Home/clutch，Manus 回调进入同一舞肌二代 retarget；机械臂使用当前
`pico_ee_dexhand_qp` 的 pose-only v131 IK，不要求、不绑定 Motion Tracker。运行时必须提供
XRoboToolkit Python SDK/PC-Service，
Manus rawviz 和用户标定仍通过参数注入。参考工程的 `install_sdk.sh` 默认把 Python 扩展放在
`~/.local/lib/python3.10/site-packages`、`libPXREARobotSDK.so` 放在 `~/.local/lib`；
当前 Pixi 环境可用下面两个参数显式注入：

```bash
XR_SDK_PYTHONPATH="$HOME/.local/lib/python3.10/site-packages"
XR_SDK_LIBRARY_DIR="$HOME/.local/lib"
```

如果只有参考工程的 Pybind 源码和 `libPXREARobotSDK.so`，可以在当前工程内构建可移植的
本地运行时。构建脚本不会修改参考源码，产物放在 Git 忽略的 `vendor/xr_sdk`；之后
`vr_manus_xr_sim` 会自动发现它，不需要再填写 SDK 路径：

```bash
export XR_SDK_SOURCE="${XR_SDK_SOURCE:?set XR_SDK_SOURCE to the XRoboToolkit Pybind source root}"
pixi run build-xr-sdk -- --source "$XR_SDK_SOURCE"
# 若 companion library 不在 source/lib，可显式指定：
# pixi run build-xr-sdk -- --source "$XR_SDK_SOURCE" --library-dir "$XR_SDK_LIBRARY_DIR"
```

这一步只生成 Python 扩展和 companion library，不包含 XRoboToolkit PC-Service。PC-Service
仍须单独启动；当前工程提供 `xr-service` 前台包装器，可使用已部署的服务根目录，也可
直接使用当前 Git 忽略目录中的迁移运行时：

```bash
pixi run xr-service -- --check
pixi run xr-service
```

若服务根目录不在默认位置，设置 `XR_PC_SERVICE_ROOT` 或传入 `--root PATH`。包装器只设置
服务所需的库/Qt 环境并前台执行，不由 `run_session.sh` 自动启动；不会从其他 checkout
写死路径或修改 PICO2 流程。

如果 PICO XR 设备通过 USB 连接并由 PC-Service 使用 ADB，可用 XR 专用端口映射：

```bash
pixi run xr-adb       # tcp:60061、tcp:63901；不会操作 PICO2 的 tcp:10002
pixi run xr-adb list
```

命令会自动选择 `adb devices` 中第一个状态为 `device` 的设备；同时连接多台设备时，
请设置 `ADB_SERIAL=...`，让 `up/list/down` 都绑定同一台头显。
`pixi run xr-adb-down` 只移除上述两个 XR reverse 规则。PICO2 裸手仍单独使用
`adb forward tcp:10002 tcp:10002`，两套 ADB 配置互不替代。

启动时追加 `--xr-sdk-pythonpath "$XR_SDK_PYTHONPATH" --xr-sdk-library-dir
"$XR_SDK_LIBRARY_DIR"`（或只对 XR 采集进程设置对应的
`TIANJI_XR_SDK_PYTHONPATH` / `TIANJI_XR_SDK_LIBRARY_DIR`）；这些路径不会传给 IK、
coordinator 或执行器。入口会在导入 Pybind 扩展前加载 companion library，并在缺失时
提前失败。
若按参考工程安装了 PC-Service，先在独立终端保持安装后的 `runService.sh` 运行；
也可以通过环境变量指定实际安装路径：

```bash
export XR_PC_SERVICE_SCRIPT="${XR_PC_SERVICE_SCRIPT:?set XR_PC_SERVICE_SCRIPT to the installed PC-Service launcher}"
"$XR_PC_SERVICE_SCRIPT"
```

XRoboToolkit Pybind 通过 PC-Service 的本地共享内存取数。PICO2 裸手路线的
`adb forward tcp:10002 tcp:10002` 不属于这条 XR 路线，不能替代 PC-Service。
追加 `--xr-overlay` 可在 MuJoCo 诊断空间只读显示原始头显和两个控制器；即使 SDK 返回
额外 Tracker，也不会参与映射或命令。该入口与已验证的`pico2_hands_sim`互斥，不会启动
PICO2 接收器。
XR 会话启动前会运行 `scripts/check_xr_sdk.py`，只检查 Pybind 必需接口，不调用设备连接；
缺少 SDK 或接口时会在创建 router/执行器前失败，不会自动回退到 PICO2 或旧 TJVR。

也可先用只读探针确认真实 XR 输入已经在线：

```bash
pixi run python scripts/probe_teleop_input.py --mode xr \
  --xr-sdk-pythonpath "$XR_SDK_PYTHONPATH" \
  --xr-sdk-library-dir "$XR_SDK_LIBRARY_DIR" \
  --arm-input xr_controller --duration-s 10 --minimum-frames 30
```

该探针只检查 SDK、PC-Service、HMD、双控制器和输入新鲜度，不连接 router、不启动 Manus、
不执行操作事件或机器人命令；`passed=true`不等于标定、映射或真机验收通过。

没有 XR SDK 或设备时，可先运行离线目标层 smoke，验证控制器-only 绑定：

```bash
pixi run python scripts/xr_manus_sim_smoke.py --arm-input xr_controller --frames 100
```

该 smoke 使用合成 XR 帧和 Manus assembler 形状的 126-float callback，经过 canonical
observation、显式 controller start 边沿和 `xr_incremental`/手目标桥；不启动 SDK、rawviz、
router、IK 或机器人命令。它只证明输入到目标层的接线，不替代 MuJoCo、官方 Hand2
retarget 或真实设备验收。

两条目标入口均通过本机合成输入、原生算法和录制联调。PICO2 裸手已有当前分支的
真实设备人工试运行反馈，原版 PICO＋VR 手柄也已完成真实设备人工验证；当前分支
内嵌入口尚需补齐带 Manus 手套的正式 H5 验收、完整模式切换和长期实时性能记录。
真实输入设备的准备、只接收探测、两路仿真启动和人工验收步骤见[真实输入→仿真验收操作单](docs/real-input-simulation-acceptance.md)。Manus标准SDK库目录现在仅注入采集子进程；自定义路径使用`--manus-library-dir`，不需要全局修改IK/retarget运行环境。
停止会话后才能切换模式，不自动回退来源；`--resolve-only`可先只读检查对应配置。

录制结束后，可纯离线核验原始输入到骨架观测/回调的一致性：

```bash
pixi run python scripts/check_dual_recording.py --mode pico --input recordings/device_acceptance/pico2_TIMESTAMP.h5
pixi run python scripts/check_dual_recording.py --mode pico --input recordings/device_acceptance/pico2_TIMESTAMP.h5 \
  --retarget-hand-commands
pixi run python scripts/check_dual_recording.py --mode manus --input /已有目录/manus.h5
pixi run python scripts/check_dual_recording.py --mode manus --input /已有目录/manus.h5 \
  --retarget-hand-commands
pixi run python scripts/check_dual_recording.py --mode tjvr --input /已有目录/vr_manus.h5
pixi run python scripts/check_dual_recording.py --mode tjvr --input /已有目录/vr_manus.h5 \
  --check-native-resets
pixi run python scripts/check_dual_recording.py --mode xr --input /已有目录/vr_manus_xr.h5
```

输出JSON及首个差异，退出码0/1/2分别表示一致/差异/输入错误。不会连接router或执行录制中的操作。
Manus核验要求录制包含原始rawviz行和解析绑定配置；旧的仅callback/PKL文件不冒充raw重建。
TJVR核验要求新VR录制中的门控参数、reset初态和逐包决策审计；重新计算重复/跳变拒绝、epoch变化及重同步代数，按原记录顺序精确比较，不重排或插值。若录制含`native_attempt`，还核验SPARK实际记录的消费帧是否通过门控、原包/接收时钟/重同步代数一致，以及tick和执行代次的顺序；无新帧tick保持`null`，不补零或插值。`native_input_check`明确区分已检查、无原生调用、未录制或旧审计缺字段。不包含手指，不能搭配`--retarget-hand-commands`；不能证明实时调度选中了理论最新帧，也不重建IK/授权状态。缺少合同/审计的旧录制仍可读取，但不猜测门控或消费历史。
XR核验要求schema 1.2录制包含`raw/xr_input`；逐帧核对HMD、控制器、按键、连接代次/序号及展平列与完整JSON的内容。若兼容SDK返回Tracker，则只核对其被动记录，不作为必需输入；同时核验`meta/dual_audit`中的XR操作观察，不连接SDK、router或执行器。
显式`--retarget-hand-commands`还要求录制资产摘要与本地官方手环境一致，启动离线worker重建已有手关节命令；不会重放授权或驱动设备。
PICO模式额外要求新录制中的原始接收时钟及实际retarget消费记录；缺少这些边界的旧文件不会猜测滤波初态。
这是录制一致性检查，不是完整IK/retarget等价性或真机验收。

TJVR可显式追加`--check-native-resets`，按当前受管VR初始执行代次1的合同，核验成功rearm的14关节reset回执、零速度/加速度、执行代次递增及后续原生调用顺序。拒绝的rearm不推进代次，其他运行实例的回执不采用；缺失回执不能支撑新的执行代次。此选项不执行reset，不验证实际Home/静止反馈、内部新鲜输入截止时刻或启动授权；普通检查默认不启用这一更严格阶段。

已有原版与迁移版算法轨迹时，可分阶段比较：

```bash
pixi run python scripts/compare_dual_input_reference.py \
  --spark-reference /轨迹目录/reference.jsonl --spark-migrated /轨迹目录/migrated.jsonl \
  --hand-reference /轨迹目录/original_hand.json --hand-migrated /轨迹目录/migrated_hand.json
```

两组参数可只提供其中完整一组。它比较已有轨迹，不启动设备；通过仅代表所提供阶段一致，不代表同步联合或真机验收。

详细依赖、可运行命令、测试证据及未完成项见
[双输入接入进度](docs/dual-input-integration-progress.md)。

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
