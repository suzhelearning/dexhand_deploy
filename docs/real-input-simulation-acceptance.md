# 真实输入设备 → 仿真验收操作单

日常启动请先阅读 [README 操作指令](../README.md)：其中“PICO2 裸手遥操快速测试”
提供裸手路线的 router、ADB、仅 Z 标定和启动步骤；“内嵌原版 PICO＋VR手柄＋Manus”
提供另一条路线的构建、Manus 人员配置、完整启动及 `s/h/r/q` 操作。本文用于验收记录。

目标是验证真实PICO2裸手，或PICO头显＋左右VR手柄＋Manus输入，驱动**仿真**天机双臂与舞肌二代手。不是机器人真机验收；不启用real profile，不连接机器人执行器。Motion Tracker不属于本阶段设备范围。

本机软件的目标受管 sim 入口为两路：PICO2 裸手，以及内嵌原版 PICO driver/M0/TJVR 的
PICO头显＋VR手柄＋Manus。另保留需要 XRoboToolkit/PC-Service 的实验入口。内嵌路线的
Manus rawviz 与人员标定路径通过启动参数注入，PICO个人标定从
`~/.config/pico_tracker` 或 `--pico-calibration-dir` 读取；当前工程不写死任何外部
checkout。PICO2（包括兼容的 `hand_tracking_sim`）、XR、历史 TJVR/Manus 和内嵌 legacy PICO
sim 入口共用设备路由 guard，只允许其中一路
持有输入；内嵌入口还保留 `embedded-pico.lock` 兼容状态文件，不再额外持有第二把内核锁。
正常退出会清理本次登记的
子进程组，监督进程被强杀后下一次启动会依据 PID 启动时刻和持久化 `children` 记录
回收遗留 driver/M0/bridge/downstream。内嵌入口在派生进程前写入 `process_tokens`，
用于恢复再次调用 `setsid` 的 M0/ROS 后代；恢复后才允许新会话启动。
启动时必须确认子进程 PGID 等于 PID，管理锁或记录写入失败则拒绝启动。
两路均支持会话 start/Home/rearm。

2026-09-13状态：用户已确认 PICO2 裸手路线可用，也确认当前内嵌 PICO＋VR 手柄＋
Manus 能遥操仿真机械臂和舞肌二代手，且 Manus 滤波连续性修复后抖动改善。
此前长时间录制仍出现队列溢出；现已接入 C++ 增量写入后端并通过离线压力测试。
新后端的真实设备长时间完整录制、两路切换压力验收仍待补齐。
用户随后确认遥操测试正常，但 `pico_vr_manus_20260913_015227.h5` 在 Ctrl-C 后
仍为 complete=false，退出日志包含核心转储，不能作为完整录制通过的依据。
审查已复现整组 TERM 提前终止录制子进程的问题，现改为先通知会话主进程、等待有界
排空，再执行原有进程组兜底清理；真实 SDK 下 Ctrl-C 退出仍待复测。
提交前 Python 全量回归共 **995 项，940 项执行通过、55 项按环境条件跳过**，包含
记录写入失败、真实 SIGKILL 后代恢复测试；守卫相关 **9 项全部通过**。
守卫测试在加载公共脚本前设置临时运行目录，并校验记录位置，不使用现场默认守卫目录。
内嵌 PICO bundle
此前 ROS2 CTest 为 29/29 通过，本轮未重跑。软件回归不替代带完整记录的设备验收。

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

## 2. 历史兼容：外部PICO_tracker＋Manus（当前不验收）

> 本节仅保留旧 `vr_manus_sim` 的 TJVR/Tracker 兼容说明，不属于当前 PICO2 裸手或
> PICO 头显＋双VR手柄＋Manus 目标路线。当前设备验收请跳到第3节，且不需要 Tracker。

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

## 3. 内嵌原版PICO＋VR手柄＋Manus路线

这条路线是当前工程用于真实 PICO＋VR 手柄＋Manus 输入控制仿真机械臂和舞肌二代手的
正式入口。它完整内嵌原版 `PICO_tracker` 的 `pico_bridge`、M0 骨架/掌心修正及 TJVR
UDP bridge；不依赖外部 `PICO_tracker`、XR SDK 或 Motion Tracker。首次构建：

```bash
pixi run build-embedded-pico
pixi run build-hdf5-recorder
```

使用原版 wholebody APK（`com.PICO.wholebody_stream.unity`），ADB 端口是 `9999`，
与 PICO2 裸手的 `10002` 完全分离。先启动 Zenoh router，再连接设备并确认 calibration：

```bash
adb devices -l
adb forward tcp:9999 tcp:9999
adb forward --list
```

先仅验证机械臂：

```bash
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile pico_vr_manus_sim --viewer --disable-hands \
  --tjvr-bind 127.0.0.1 --tjvr-port 15000 --spark-overlay \
  --pico-startup-timeout-s 20 \
  --record "recordings/device_acceptance/pico_vr_arms_${TELEOP_TEST_ID}.h5"
```

默认标定目录为 `~/.config/pico_tracker`，也可追加
`--pico-calibration-dir /path/to/pico_tracker_config`。完整双手测试时移除
`--disable-hands`，追加 `--manus-rawviz PATH --manus-user gjy`（人员标定必须与实际
佩戴者一致）。supervisor 会依次检查 raw、M0 corrected 和 TJVR diagnostics topic，
再启动当前 `vr_manus_sim` 下游；退出时仅清理它自己登记的 PICO 进程和 ADB forward。
它与 PICO2、XR 和另一个 VR/Manus sim 会话共享设备路由 guard；强制退出后下一次
相关入口会先恢复遗留子进程，避免 driver/M0/TJVR 重复启动。

VR/Manus 手部回调在进入 retarget 前超过 200 ms 时，会拒绝该帧并继续等待新鲜输入，
不会仅因这一帧过期退出整个会话。拒绝计数见手部状态的 `expired_callbacks`，
逐帧原因保存在 HDF5 `meta/dual_audit` 的 `manus_expired_input` 事件中；原始回调仍保留。
离线手部命令核验会跳过这些未参与在线 retarget 的帧。该策略只由 SPARK/Manus 路线启用，
不更改 PICO2 裸手入口。未收到新鲜双手输入时不能启动；遥操中持续失去新鲜输入仍按现有
coordinator 规则回 Home，不自动恢复遥操。设备身份、时钟乱序、队列溢出和求解器异常仍报故障。

VR/Manus 在线手部求解采用最新帧调度：每次求解前有界排空已排队的旧回调，选择最新帧。
被替代的回调逐帧记录为 `manus_superseded_input`，计数见 `superseded_callbacks`。
这是在线求解调度的变更，不再要求每个原始回调都运行一次 retarget；原始回调仍全部录制，
求解器、人员标定、关节映射以及输入时间戳保持原定义，离线命令核验遵循实际选择序列。

最新帧调度下，Manus worker 使用独立的连续处理序号，避免正常跳过积压帧时反复
清空原版低通滤波器（`lp_alpha: 0.2`）。相邻已处理输入的接收时间间隔超过 200 ms
时仍制造一次处理序号间断，由原版 bridge 重置状态；单侧缺失后恢复也保留原版重置语义。
原始回调序号仍用于发布和录制，不被处理序号替换。录制配置中的
`manus_filter_continuity_ns: 200000000` 供离线核验复现相同策略；旧录制无此字段时
沿用原始序号策略。此修复不修改第三方算法，不给机械臂加平滑，也不启用在 PICO2 裸手入口。

VR/Manus 的持续 HDF5 写入由独立 C++ 进程负责。Python 只初始化 schema、校验消息及
打包列块，初始化完成后关闭自己的 HDF5 句柄；C++ 缓存 dataset 并批量写入，包括
原始 UDP 变长字节列。每列达到 128 行或估算缓冲达到 8 MiB 时在消息边界发送，
刷新周期为 0.5 秒；批量消息可能略超过行数阈值，IPC 总包上限为 64 MiB。
退出排空，原始数据和控制周期不降采样。`complete=true` 仅在关闭前排空及 HDF5
flush 成功后设置；这不等于断电保护或磁盘 fsync 保证。异常仍按原策略停止会话。
PICO2 和原有 session writer 不切换后端。

构建命令为 `pixi run build-hdf5-recorder`，需要系统 `g++` 和 `libhdf5-dev`（提供 `h5c++`）。
产物位于 `build/hdf5_recorder/tianji_hdf5_recorder`，不能仅复制源代码而跳过构建。
找不到产物时启动失败，不静默退回 Python。退出摘要 `recording` 包含 backend、
queue_high_water、queue_capacity、accepted、processed、blocks_written 和 max_block_latency_s；
processed 表示交给后端的记录数，完整写盘还需以成功关闭及 complete 标记为准。

退出建议：`q` 走回 Home 后退出；Ctrl-C 走停止当前仿真并排空录制，不保证先回 Home。
VR/Manus launcher 最多给 Python owner 40 秒收尾，内嵌 supervisor 最多等待下游 shell
50 秒后才进入进程组 TERM/KILL 兜底。重复 Ctrl-C 不打断正在进行的清理。
强制 KILL、掉电或写盘失败仍可能留下 complete=false；不会事后将不完整文件改为成功。

离线压力测试（输出文件不能已存在；不连接设备、不发布控制命令）：

```bash
pixi run python scripts/benchmark_native_recording.py \
  --template recordings/device_acceptance/<已有联合录制>.h5 \
  --output recordings/device_acceptance/<新压力测试文件>.h5 \
  --duration-s 120 --speed 2
```

这只验证混合流录制吞吐，不代表真实设备联合长时间验收。

2026-09-13 本机两轮 120 秒、2 倍速测试通过。使用活动 IK 审计样本的一轮保存
47,999 个控制快照、57,599 个 Manus 回调、21,599 个 TJVR 包和 11,999 次双手输出，
生成与保存数量一致；队列高水位 147/16384，退出后为零，文件 complete=true。
测试产物 `native_hdf5_stress_20260913_active.h5` 位于 device_acceptance 目录，但其数据
为合成压力负载，不能作为真实设备或算法准确性验收。

只读确认入口：

```bash
pixi run bash scripts/run_session.sh \
  --profile pico_vr_manus_sim --disable-hands --resolve-only
```

该入口与 `pico2_hands_sim` 互斥；切换前退出当前会话。不要在新入口同时运行外部
`PICO_tracker/scripts/start_tianji_pico_teleop.sh`，否则会产生重复 driver/M0/TJVR。

## 4. XRoboToolkit控制器＋Manus实验路线

这是一条独立的 XR controller-only 实验入口，不是当前默认的 PICO＋VR 手柄＋Manus
主路线；当前主路线见第 3 节。它不启动 PICO2 裸手接收器，也不依赖旧 TJVR 上肢转换。
运行前必须让当前 `pixi` Python 能导入 `xrobotoolkit_sdk`，并启动对应的 XR PC-Service；
头显坐标系、控制器左右绑定和控制器→腕部外参来自 `sources/xr_manus_observation.yaml`
与 `hand_tracking_target_xr_manus.yaml`，可在本地配置副本中调整。Motion Tracker 不属于
本路线的前置设备。

先确认 XR 服务和输入设备均在线，再启动仿真。机械臂使用左右手柄位姿，并使用固定的参考肘部方向；不读取、不要求 Tracker。控制器绑定为右手 grip=`start`、左手 grip=`home`、右手 trigger=`clutch`，不会绕过现有 coordinator 授权。

若 PC-Service 已按参考工程安装，在独立终端保持服务运行。当前工程提供前台包装器，
优先使用 `--root`，其次使用 `XR_PC_SERVICE_ROOT`，最后使用 Git 忽略目录中的迁移运行时：

```bash
pixi run xr-service -- --check
pixi run xr-service
# 外部安装目录：
# export XR_PC_SERVICE_ROOT="${XR_PC_SERVICE_ROOT:?set XR_PC_SERVICE_ROOT}"
# pixi run xr-service
```

包装器以前台方式执行 `RoboticsServiceProcess`，设置旧 `runService.sh` 使用的库/Qt 环境，
便于退出和查看错误；它不会由 `run_session.sh` 自动启动。PC-Service 通过本地共享内存
向 Pybind 提供头显和控制器数据。

PICO2 裸手路线才使用 `adb forward tcp:10002 tcp:10002`；它不是 XRoboToolkit 路线的
连接方式，不能替代 PC-Service。

如果当前 Pixi Python 不能直接导入 `xrobotoolkit_sdk`，将 SDK 的 Python 包父目录通过
`--xr-sdk-pythonpath PATH` 注入 XR 采集进程；参考工程默认的 companion library 目录通过
`--xr-sdk-library-dir PATH` 注入。也可在启动前设置
`TIANJI_XR_SDK_PYTHONPATH` 和 `TIANJI_XR_SDK_LIBRARY_DIR`。例如参考工程安装脚本的
默认布局为：

```bash
export XR_SDK_PYTHONPATH="$HOME/.local/lib/python3.10/site-packages"
export XR_SDK_LIBRARY_DIR="$HOME/.local/lib"
```

如果部署机只有 XRoboToolkit Pybind 源码，可以使用当前工程提供的构建入口。它会在临时
目录编译，不修改外部源码，并将 Python 扩展和 `libPXREARobotSDK.so` 输出到被 Git 忽略的
`vendor/xr_sdk/{python,lib}`；XR session 会自动发现这两个目录：

```bash
export XR_SDK_SOURCE="${XR_SDK_SOURCE:?set XR_SDK_SOURCE to the XRoboToolkit Pybind source root}"
pixi run build-xr-sdk -- --source "$XR_SDK_SOURCE"
# library 不在 source/lib 时：
# pixi run build-xr-sdk -- --source "$XR_SDK_SOURCE" --library-dir "$XR_SDK_LIBRARY_DIR"
```

构建 Python 扩展不等于安装 PC-Service。服务进程及其厂商运行库仍由部署环境单独提供，
当前工程不会把其他 checkout 的绝对路径写入代码，也不会自动复制/启动服务。若 XR 设备
通过 USB 接入并由 PC-Service 使用 ADB，可执行：

```bash
pixi run xr-adb
pixi run xr-adb list
```

XR ADB 工具会自动选择第一个 `adb devices` 状态为 `device` 的设备；同时连接多台设备时，
设置 `ADB_SERIAL=...`，保证端口映射、查看和移除操作始终针对同一台头显。
上述入口只映射 XRoboToolkit 的 `60061`、`63901`；PICO2 裸手的 `adb forward tcp:10002`
仍是另一条独立链路。需要清理时运行 `pixi run xr-adb-down`。

两者都只作用于 XR 采集进程，不会污染 IK、coordinator 或 MuJoCo 进程；入口会在
导入 Pybind 扩展前加载 `libPXREARobotSDK.so`。需要核对头显和控制器时追加 `--xr-overlay`。

正式启动仿真前，先停止其他 XR/遥操进程，运行只读输入探针。它会预检 SDK 的
必需接口并连接 PC-Service 读取短时数据，但不连接当前工程 router、不启动 Manus、
不执行 start/Home/clutch、不发布 IK 或机器人命令：

```bash
export XR_SDK_PYTHONPATH="${XR_SDK_PYTHONPATH:?set XR_SDK_PYTHONPATH to the xrobotoolkit_sdk package parent}"
export XR_SDK_LIBRARY_DIR="${XR_SDK_LIBRARY_DIR:?set XR_SDK_LIBRARY_DIR to libPXREARobotSDK.so directory}"
pixi run python scripts/probe_teleop_input.py \
  --mode xr \
  --xr-sdk-pythonpath "$XR_SDK_PYTHONPATH" \
  --xr-sdk-library-dir "$XR_SDK_LIBRARY_DIR" \
  --xr-config src/tianji_teleop/config/sources/xr_manus_observation.yaml \
  --arm-input xr_controller \
  --duration-s 10 \
  --minimum-frames 30
```

上面是“头显＋左右手柄”主路径，不要求腕部/前臂 Tracker。只有输出 JSON 中
`passed=true` 才进入下面的仿真步骤；它只证明短时设备数据活跃，
不证明控制器标定、Manus 手套、坐标映射、IK 或真机安全条件。探针会临时
独占 XR SDK，结束后再启动仿真。若选择 `xr_controller`，省略 `--minimum-trackers`
即可按控制器-only 输入检查。

没有 XR SDK 或设备时，可以先做输入到目标层的离线接线检查：

```bash
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
  --arm-input xr_controller --frames 100 --disable-hands

# 验证 XR 手臂 + Manus 双手 → 官方 Wuji2 → MuJoCo 舞肌二代手
pixi run python scripts/xr_mujoco_sim_smoke.py \
  --arm-input xr_controller --frames 100 --with-manus
```

带 `--with-manus` 的报告应同时包含 `mujoco_hand_overlay_verified=true`、
`ik_algorithm=pico_ee_v131_velocity_qp`、`raw_manus_callbacks` 和
`recorded_hand_command_frames`；报告中的 `robot_commands_enabled` 与
`hardware_acceptance_complete` 必须保持 `false`。

```bash
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile vr_manus_xr_sim --viewer \
  --xr-sdk-pythonpath "$XR_SDK_PYTHONPATH" \
  --xr-sdk-library-dir "$XR_SDK_LIBRARY_DIR" \
  --arm-input xr_controller \
  --xr-overlay \
  --manus-rawviz "${PICO_MANUS_TELEOP_ROOT:?set PICO_MANUS_TELEOP_ROOT}/manus/rawviz.out" \
  --manus-user gjy \
  --record "recordings/device_acceptance/xr_manus_${TELEOP_TEST_ID}.h5"
```

只测 XR 机械臂时可加 `--disable-hands`；双手验证时保留 Manus 参数。新 schema 1.2 录制会保存完整 `raw/xr_input`（头显、控制器、按键和原始 JSON；兼容字段中若 SDK 返回 Tracker 也只作被动记录）、`raw/manus_callbacks` 及 `meta/dual_audit/operator_observation`，历史 PICO2 录制格式不变。若 SDK/PC-Service 不可用，启动应在采集边界报错，不能自动回退到 PICO2 或旧 TJVR。

## 5. 必须人工完成的检查

按顺序执行，每项记录实际结果和对应H5，不预填“通过”：

1. idle时无运动；左右手无交换，头/掌位置与overlay一致。
2. 分别小幅移动六个方向、旋转手掌，确认坐标轴、TCP及期望IK位姿；再逐步扩大范围。
3. 开合手、捏合与双手合拢，观察手关节侧别、限位及机械臂动作；不以探测器通过代替本项。
4. PICO短时遮挡单侧/双侧：观察保持和恢复；断开传输后不得继续接受旧输入。
5. VR输入失效、Home/rearm、重新启动按当前明确状态执行；不假定与PICO视觉丢失策略相同。
6. 退出后切换另一输入；确认旧采集进程、发布者和操作事件没有残留。
7. 短程通过后至少做一次30分钟联合运行，记录延迟、掉帧、错误退出和温升现象。该时长是本次人工测试步骤，不是已证明的实时性能等级。

## 6. 录制核验

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

此前已完成 PICO2 裸手和原版 PICO＋VR 手柄的真实设备人工试运行；当前分支内嵌入口也
完成过启动及机械臂动作冒烟。当前工作区本轮没有重新进行头显/VR手柄/Manus联合采样，
因此尚缺当前分支带 Manus 手套的完整 H5 验收、两路切换压力及长期运行记录；这不等于
前述已完成的人工试运行不存在。Tracker 不属于本阶段设备条件。
