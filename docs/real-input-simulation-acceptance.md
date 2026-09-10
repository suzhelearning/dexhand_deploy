# 真实输入设备 → 仿真验收操作单

目标是验证真实PICO2，或外部PICO_tracker＋控制器/Tracker＋Manus输入，驱动**仿真**天机双臂与舞肌二代手。不是机器人真机验收；不启用real profile，不连接机器人执行器。

本机软件已具备三路受管 sim 入口：PICO2 裸手、旧版 PICO_tracker＋Manus，以及 XRoboToolkit 控制器/Tracker＋Manus。XR 路线的 SDK/PC-Service 由操作者在运行环境中提供，Manus rawviz 与人员标定路径通过启动参数注入；当前工程不写死任何外部 checkout。两种 VR 路线都支持会话 start/Home/rearm 和控制器绑定；真实设备测试仍需按本文执行，尚未在本机完成实物验收。

2026-09-10状态：按用户安排，实物测试暂缓；下面是待执行操作，不是已通过的验收记录。当前 Python 全量回归为 905 项通过、53 项按环境条件跳过；当前 `build` 目录未发现可运行的 CTest 用例，不把 CTest 作为本轮通过证据，也不替代真实设备测试。

## 0. 通用准备

- 停止已有遥操会话；不要同时运行探测器和会话，它们会占用同一输入。
- 确认人手侧别、人员标定和机器人模型/TCP；不要把历史标定文件视为当前设备已经加载的证明。
- 在工程根目录执行下面命令。每次使用新的录制路径，启动器拒绝覆盖。
- 首轮关闭手部，只验证机械臂；通过后退出会话，移除`--disable-hands`验证双手。

单独终端启动router（若该端口已有正确router，不要重复启动）：

```bash
./vendor/zenoh-router/zenohd -l tcp/127.0.0.1:7447 --no-multicast-scouting
```

会话终端准备：

```bash
mkdir -p recordings/device_acceptance
TELEOP_TEST_ID=$(date +%Y%m%d_%H%M%S)
```

## 1. PICO2裸手路线

只连接一台目标头显，打开当前PICO_2头显端采集应用并启用发送；检查USB调试授权。

```bash
adb devices -l
adb forward tcp:10002 tcp:10002
pixi run python scripts/probe_teleop_input.py --mode pico --duration-s 10
```

探测只接收数据，不自动执行ADB、不连接router、不启动IK。PICO通过条件是头显、双腕及双手26节点有效，源时间戳持续更新；结束时每侧有效数据不超过200ms。`passed=true`只代表短时双侧输入可见，不代表坐标、标定、手势或硬件验收通过。失败时先解决应用发送/USB/识别问题，不进入运动步骤。

探测自动结束后启动仿真：

```bash
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile pico2_hands_sim --viewer --disable-hands \
  --arm-pose-mapper head_palm_direct \
  --ik-backend pico_ee_dexhand_qp \
  --arm-target-processor passthrough --joint-trajectory passthrough \
  --command-step-clipping false --joint-limit-source urdf \
  --pico-overlay --ik-target-overlay \
  --record "recordings/device_acceptance/pico_${TELEOP_TEST_ID}.h5"
```

先观察overlay左右和坐标轴。根据界面提示完成高度标定（需要时按`c`并保持双手水平稳定），再按`s`显式启动；按`q`回Home并退出。不要把VR的`h/r`操作套用到PICO入口。

退出后重新生成`TELEOP_TEST_ID`，移除`--disable-hands`验证舞肌二代手。首轮不启用手势START，先用键盘确认动作与模型映射。

## 2. 外部PICO_tracker＋Manus路线

确认原PICO_tracker当前配置及标定已在实际运行进程中生效，并输出TJVR v4到本机`127.0.0.1:15000`。若上游在另一台机器，需显式配置发送目标为本机IP；不能用远端的127.0.0.1代替。

上游参考入口在`${PICO_MANUS_TELEOP_ROOT}/PICO_tracker/scripts/start_tianji_pico_teleop.sh`。**该脚本会清理历史driver/M0/bridge进程**，不要在其他实验运行时直接启动；先按原工程操作说明确认清理范围。当前工程不自动运行、修改或替换这个启动器。运行前由操作者设置`PICO_MANUS_TELEOP_ROOT`，当前工程不假设其具体目录。

在上游已启动且没有本工程会话占用输入时，分别探测：

```bash
pixi run python scripts/probe_teleop_input.py --mode tjvr --duration-s 10
pixi run python scripts/probe_teleop_input.py --mode manus --duration-s 10 \
  --manus-rawviz "${PICO_MANUS_TELEOP_ROOT:?set PICO_MANUS_TELEOP_ROOT}/manus/rawviz.out" \
  --manus-user gjy
```

`gjy`仅适用于佩戴者确实使用该人员标定的情况；其他人员必须替换。探测Manus会启动并在结束时关闭自己的rawviz采集子进程，不要另起第二个rawviz。双侧各自的新源序号才计数，缓存的另一侧数据不刷新其新鲜度。

SDK默认从rawviz旁的`ManusSDK/lib`解析；自定义布局可在探测器和会话中显式加`--manus-library-dir "${MANUS_SDK_LIB_DIR}"`。库路径只传给Manus采集子进程，不全局export，不影响SPARK和官方手worker。未找到标准布局且没有显式指定时，预检标为`inherited_unverified`，需自行确认动态依赖并通过输入探测。

先验证双臂：

```bash
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile vr_manus_sim --viewer --spark-overlay --disable-hands \
  --record "recordings/device_acceptance/vr_arms_${TELEOP_TEST_ID}.h5"
```

退出后更新`TELEOP_TEST_ID`，验证双臂与手套联动：

```bash
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile vr_manus_sim --viewer --spark-overlay \
  --manus-rawviz "${PICO_MANUS_TELEOP_ROOT:?set PICO_MANUS_TELEOP_ROOT}/manus/rawviz.out" \
  --manus-user gjy \
  --record "recordings/device_acceptance/vr_manus_${TELEOP_TEST_ID}.h5"
```

`s`启动，`h`回Home；健康idle/Home后`r`联合重置，等待新输入后再`s`；`q`回Home退出。故障或任意姿态下不强行reset，不删除保护参数绕过拒绝。

## 3. XRoboToolkit控制器/Tracker＋Manus路线

这条路线是当前工程新增的 PICO＋VR 手柄/Tracker＋Manus 入口，不启动 PICO2 裸手接收器，也不依赖旧 TJVR 上肢转换。运行前必须让当前 `pixi` Python 能导入 `xrobotoolkit_sdk`，并启动对应的 XR PC-Service；Tracker 序列号、头显坐标系和控制器左右绑定来自 `sources/xr_manus_observation.yaml`，可在本地配置副本中调整。

先确认 XR 服务和四个设备均在线，再启动仿真。`xr_tracker` 使用腕部 Tracker 做机械臂输入；需要控制器位姿时改成 `xr_controller`。控制器绑定为右手 grip=`start`、左手 grip=`home`、右手 trigger=`clutch`，不会绕过现有 coordinator 授权。

如果当前 Pixi Python 不能直接导入 `xrobotoolkit_sdk`，将 SDK 的 Python 包父目录通过
`--xr-sdk-pythonpath PATH` 注入 XR 采集进程；也可在启动前设置
`TIANJI_XR_SDK_PYTHONPATH`。两者都不写入工程配置，也不会污染 IK、coordinator 或
MuJoCo 进程。需要核对头显、控制器、腕部/前臂 Tracker 时追加 `--xr-overlay`。

正式启动仿真前，先停止其他 XR/遥操进程，运行只读输入探针。它会预检 SDK 的
必需接口并连接 PC-Service 读取短时数据，但不连接当前工程 router、不启动 Manus、
不执行 start/Home/clutch、不发布 IK 或机器人命令：

```bash
XR_SDK_PYTHONPATH="${XR_SDK_PYTHONPATH:?set XR_SDK_PYTHONPATH to the xrobotoolkit_sdk package parent}" \
pixi run python scripts/probe_teleop_input.py \
  --mode xr \
  --xr-sdk-pythonpath "$XR_SDK_PYTHONPATH" \
  --xr-config src/tianji_teleop/config/sources/xr_manus_observation.yaml \
  --arm-input xr_tracker \
  --duration-s 10 \
  --minimum-frames 30 \
  --minimum-trackers 4
```

只有输出 JSON 中 `passed=true` 才进入下面的仿真步骤；它只证明短时设备数据活跃，
不证明 Tracker/控制器标定、Manus 手套、坐标映射、IK 或真机安全条件。探针会临时
独占 XR SDK，结束后再启动仿真；若选择 `xr_controller`，仍建议保留四个 Tracker
用于当前肘部/绑定诊断。

没有 XR SDK 或设备时，可以先做输入到目标层的离线接线检查：

```bash
pixi run python scripts/xr_manus_sim_smoke.py --arm-input xr_tracker --frames 100
pixi run python scripts/xr_manus_sim_smoke.py --arm-input xr_controller --frames 100
```

该检查使用合成 XR 帧和 126-float Manus callback，验证 canonical observation、显式
controller start 边沿、`xr_incremental` 映射和双手目标输出；它是 receive-only，不连接
SDK/PC-Service、rawviz、router，不启动 IK、retarget 或机器人命令。通过后仍需执行下面
的真实 SDK 探针和 MuJoCo 仿真。

在真实设备接入前，还可以运行受管的全链路仿真 smoke。它会临时生成 XR SDK
兼容模块和 rawviz 双手流，但仍使用当前工程的 observation、21 点转换、官方
Wuji2 worker、双侧 hand executor、v131 IK、coordinator 和 MuJoCo overlay；这一步
不连接设备、不打开机器人、不代表实物验收：

```bash
# 仅验证 XR 手臂/控制器生命周期
pixi run python scripts/xr_mujoco_sim_smoke.py \
  --arm-input xr_tracker --frames 100 --disable-hands

# 验证 XR 手臂 + Manus 双手 → 官方 Wuji2 → MuJoCo 舞肌二代手
pixi run python scripts/xr_mujoco_sim_smoke.py \
  --arm-input xr_tracker --frames 100 --with-manus

# 需要控制器位姿作为机械臂输入时
pixi run python scripts/xr_mujoco_sim_smoke.py \
  --arm-input xr_controller --frames 100 --disable-hands
```

带 `--with-manus` 的报告应同时包含 `mujoco_hand_overlay_verified=true`、
`ik_algorithm=pico_ee_v131_velocity_qp`、`raw_manus_callbacks` 和
`recorded_hand_command_frames`；报告中的 `robot_commands_enabled` 与
`hardware_acceptance_complete` 必须保持 `false`。

```bash
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile vr_manus_xr_sim --viewer \
  --arm-input xr_tracker \
  --xr-overlay \
  --manus-rawviz "${PICO_MANUS_TELEOP_ROOT:?set PICO_MANUS_TELEOP_ROOT}/manus/rawviz.out" \
  --manus-user gjy \
  --record "recordings/device_acceptance/xr_manus_${TELEOP_TEST_ID}.h5"
```

只测 XR 机械臂时可加 `--disable-hands`；双手验证时保留 Manus 参数。新 schema 1.2 录制会保存完整 `raw/xr_input`（头显、控制器、Tracker、按键和原始 JSON）、`raw/manus_callbacks` 及 `meta/dual_audit/operator_observation`，历史 PICO2 录制格式不变。若 SDK/PC-Service 不可用，启动应在采集边界报错，不能自动回退到 PICO2 或旧 TJVR。

## 4. 必须人工完成的检查

按顺序执行，每项记录实际结果和对应H5，不预填“通过”：

1. idle时无运动；左右手无交换，头/掌位置与overlay一致。
2. 分别小幅移动六个方向、旋转手掌，确认坐标轴、TCP及期望IK位姿；再逐步扩大范围。
3. 开合手、捏合与双手合拢，观察手关节侧别、限位及机械臂动作；不以探测器通过代替本项。
4. PICO短时遮挡单侧/双侧：观察保持和恢复；断开传输后不得继续接受旧输入。
5. VR输入失效、Home/rearm、重新启动按当前明确状态执行；不假定与PICO视觉丢失策略相同。
6. 退出后切换另一输入；确认旧采集进程、发布者和操作事件没有残留。
7. 短程通过后至少做一次30分钟联合运行，记录延迟、掉帧、错误退出和温升现象。该时长是本次人工测试步骤，不是已证明的实时性能等级。

## 5. 录制核验

将下列路径替换为本次实际文件：

```bash
pixi run python scripts/check_dual_recording.py --mode pico --input /实际/pico.h5
pixi run python scripts/check_dual_recording.py --mode pico --input /实际/pico双手.h5 --retarget-hand-commands
pixi run python scripts/check_dual_recording.py --mode tjvr --input /实际/vr_manus.h5 --check-native-resets
pixi run python scripts/check_dual_recording.py --mode manus --input /实际/vr_manus.h5 --retarget-hand-commands
pixi run python scripts/check_dual_recording.py --mode xr --input /实际/xr_manus.h5
```

禁用手的录制不运行手命令重建。检查只读、不执行事件；通过也不等于完整IK状态回放或机器人真机安全验收。

## 当前未完成的设备条件

本轮本机ADB未列出头显，未进行实际头显/Manus联合采样；外部PICO_tracker当前加载标定和设备绑定未核实。因此现在交付的是可执行的真实输入→仿真测试入口与操作单，不是设备验收通过，也不是两路真机profile已经开放。
