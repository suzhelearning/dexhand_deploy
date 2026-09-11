# 完整内嵌原版 PICO+VR 手柄输入链路设计

## 状态

- 日期：2026-09-10
- 范围：将原版 PICO driver、M0 掌心/骨架修正和 TJVR bridge 的 PC 端运行链路内嵌到当前工程。
- 目标 profile：新增 `pico_vr_manus_sim`，用于 PICO 头显 + VR 手柄 +（可选）Manus 控制当前工程 MuJoCo。
- 现有 `pico2_hands_sim`：保持接口、默认值、进程组合和端口不变。
- 现有 `vr_manus_sim`：保持手动接收外部 TJVR 的历史兼容行为不变。

## 目标与非目标

### 目标

1. 当前工程仓库中包含运行目标链路所需的原版 PICO PC 端源码、ROS 包、launch 文件、脚本和可复现构建元数据。
2. 由当前工程统一入口按原版顺序启动并管理：

   ```text
   PICO wholebody_stream APK
     -> original pico_bridge (tcp:9999)
     -> original M0 corrected skeleton
     -> original TJVR UDP bridge (127.0.0.1:15000)
     -> current TJVR receiver
     -> current Spark IK/coordinator/MuJoCo
   ```

3. 启动时只启用一个输入 profile；旧 PICO 输入不能与 PICO2 TCP 输入同时被误接入。
4. 保留原版数据语义、坐标系、M0 标定、TJVR 编码、重置和按键状态处理，不重新实现一套近似算法。
5. 在没有 Manus 时通过 `--disable-hands` 先验收双臂；启用 Manus 时复用当前工程已存在的 Manus/Wuji2 下游接口。
6. 退出、启动失败和上游掉线时只清理本次嵌入链路的进程和 ADB forward，不影响 PICO2 或其他用户进程。

### 非目标

- 不将 PICO APK 复制进 Git；APK 仍安装在头显设备中。
- 不提交个人 `~/.config/pico_tracker` 标定文件、Manus `.mcal`、rawviz、SDK 私有库或录制数据。
- 不复制 PICO_tracker 的 `build/`、`install/`、`.pixi/`、日志和可选硬件数据产物。
- 不在本阶段重新移植 XRoboToolkit/PC-Service；PICO2 裸手 TCP 和 XR 路线继续独立。
- 不把可选 Odin、鱼眼相机、IMU900 采集链路纳入当前 PICO+VR 手柄验收入口；需要时仍保留独立原版源码边界。
- 不修改 IK 算法；当前旧 PICO 路线继续使用与原版 TJ_arm_control 对齐的 Spark backend。

## 方案选择

### 方案 A：只增加外部工程包装器

当前工程通过 `PICO_TRACKER_ROOT` 调用另一个 checkout 的脚本。改动最小，但运行结果仍依赖外部目录、外部构建状态和绝对路径，不满足完整内嵌要求。

### 方案 B：内嵌目标链路源码，使用隔离的嵌套 Pixi 环境（采用）

在当前工程的 `vendor/pico_tracker/` 中保存原版 `pico_bridge` 包、必需 launch/脚本、测试、`pixi.toml` 和锁文件。构建和运行使用嵌套 Pixi 环境，与当前 Python 3.10、MuJoCo、IK 环境隔离；当前工程只负责调用、健康检查和生命周期管理。

优点：保留原版 C++/Python 实现和 Python 3.11 ROS 环境，避免将 ROS 依赖污染 PICO2 和当前 IK 环境；源码可随当前工程一起检出。代价是首次需要下载 ROS 依赖并构建嵌套工作空间。

### 方案 C：把 ROS 包直接并入当前工程顶层环境

将 ROS2 依赖加入当前 `pixi.toml`，把 `pico_bridge` 与当前 Python/IK 组件一起构建。环境表面上统一，但会混合 Python 3.10/3.11、ROS 生成代码和当前依赖，增加 ABI、锁文件和 PICO2 回归风险，不采用。

## 内嵌目录与来源

```text
vendor/pico_tracker/
  pixi.toml
  pixi.lock
  src/pico_bridge/
    CMakeLists.txt
    package.xml
    include/
    src/
    launch/
    config/
    scripts/
    test/
  scripts/
    start_pico_driver.sh
    start_pico_m0.sh
    start_tianji_pico_teleop.sh
    stop_tianji_pico_teleop.sh
    cleanup_tianji_pico_processes.py
  source_manifest.json
```

只内嵌目标链路所需的完整 `pico_bridge` 包和其原版启动/清理脚本；不把原工作空间的可选 `imu_ros2`、相机、Odin 源码和构建产物复制进来。`pico_bridge` 包内部原有的可选脚部节点保留源码，但不由新 profile 启动。

`source_manifest.json` 保存来源仓库提交号、文件相对路径和 SHA-256。除路径可移植性修正外，嵌入文件必须与来源文件逐字一致；任何修正都记录变更原因和原始哈希，避免将“迁移”误认为重新实现。

## 配置与路径

- 嵌入源路径由当前仓库相对路径解析，代码中不出现开发机 checkout 绝对路径。
- 原版标定目录默认仍为 `${HOME}/.config/pico_tracker`，同时支持 `PICO_TRACKER_CONFIG_DIR` 覆盖。
- PICO APK 包名默认 `com.PICO.wholebody_stream.unity`，通过配置覆盖；启动器只检查/启动设备端 APK，不保存 APK。
- 原版 PICO 端口固定语义保持不变：ADB `9999`；当前 TJVR 接收默认 `127.0.0.1:15000`，可显式覆盖。
- 新录制、诊断和日志使用当前工程的相对路径或运行时目录，不写入来源 checkout。
- 个人标定只读加载；缺少或校验失败时 fail-closed，不从 `recordings/` 猜测候选文件。

## Profile 与启动接口

新增 `pico_vr_manus_sim`，配置语义如下：

```yaml
input_mode: pico_vr_manus
arm_input: embedded_tjvr_corrected_palm
hand_input: manus
required_capability: simulation
reference_execution_mode: reference_direct
```

默认启动命令为：

```bash
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile pico_vr_manus_sim \
  --viewer
```

没有 Manus 时追加 `--disable-hands`。启用 Manus 时沿用当前工程的 Manus 参数注入，不改变 PICO arm 输入。`vr_manus_sim` 仍只接收外部已生成的 TJVR；`pico2_hands_sim` 仍只连接 PICO2 TCP `10002`。

## 生命周期与健康检查

新启动器按以下顺序执行：

1. 解析 profile 和所有路径，检查 router、ADB 设备、APK 包和嵌入构建产物。
2. 建立或复用仅属于旧 PICO 路线的 `tcp:9999` forward；禁止使用 `adb forward --remove-all`。
3. 在嵌套 Pixi 环境启动原版 driver，等待 `/pico/smpl_raw` 和双侧 controller pose 持续更新。
4. 启动原版 M0，等待 `/pico/smpl_palm_corrected`、`/pico/smpl_palm_corrected_ik` 和状态 topic ready。
5. 启动原版 TJVR bridge，等待 UDP 输出配置和当前接收器建立。
6. 启动当前工程的 arm source、Spark producer、coordinator 和 MuJoCo executor。
7. 上游任一关键阶段失败时，按反序停止已经启动的进程并报告具体阶段；不让当前工程在无输入时假装已就绪。
8. 正常退出或异常退出均释放当前 session guard、TJVR UDP socket、旧 PICO tmux/process group 和 `tcp:9999` forward。

为兼容原版 tmux 启动脚本，嵌入链路可以继续使用独立 tmux session，但 session 名称、PID、日志目录和清理 token 必须由当前启动器记录并校验。清理只接受本次启动实例的身份，不能按宽泛进程名杀掉其他会话。

## PICO2 隔离保证

1. 不修改 `pico2_hands_sim` 的 observation、official PICO producer、26→21 手部转换和当前 v131 IK 默认值。
2. 新 profile 不导入 `official_pico.py`、`pico_official_hand.py` 的运行期模块，也不订阅 PICO2 TCP `10002`。
3. profile resolver 在同一运行目录发现另一输入 profile 的活动 guard 时拒绝启动，避免一台设备的两条输入链路抢占。
4. 嵌套 ROS Pixi 环境不加入当前根环境；PICO2 继续使用当前根环境 Python 3.10。
5. ADB 清理只操作 `tcp:9999`；不删除 PICO2 使用的 `tcp:10002`。
6. 新增测试锁定 PICO2 resolve 输出、端口、组件列表和受保护启动命令；实施前后均运行 PICO2 合成闭环回归。

## Manus 下游边界

本设计内嵌的是 PICO arm 输入上游。舞肌二代手仍由当前工程的 Manus/Wuji2 producer/executor 负责：

- `--disable-hands`：只验证 PICO+VR 手柄到 MuJoCo 双臂。
- 不加 `--disable-hands`：必须提供当前工程要求的 Manus rawviz/SDK 和人员标定；PICO 上游不再需要额外改变。
- PICO driver/M0/TJVR 不读取 Manus 数据，也不把 arm 输入伪装成手指数据。

## 测试与验收

### 自动化

- 嵌入源 manifest：文件集合、SHA-256 和禁止绝对路径检查。
- 嵌套 Pixi：锁文件可安装，`pico_bridge` 可构建，原版单元/CTest 可执行。
- 启动器：阶段顺序、参数转发、失败反序清理、重复启动拒绝和仅删除本次 `9999` forward。
- Profile：`pico_vr_manus_sim` resolve；`--disable-hands` 不加载 Manus；现有 `pico2_hands_sim` 和 `vr_manus_sim` resolve 不变。
- TJVR：原版录制/合成输入经过嵌入 bridge 输出后，当前接收和 Spark smoke 结果一致。
- PICO2：运行现有受保护命令、PICO2 producer/recording/hand2 回归，确认无新增 ROS/SDK 导入。

### 设备仿真验收

1. 安装/启动原版 `pico_wholebody_stream.apk`，确认 ADB 为 `device`。
2. 只执行当前工程新 profile，不再手动开启外部 PICO_tracker checkout。
3. 确认日志依次出现 raw ready、M0 ready、TJVR ready、session startup complete。
4. 按 `s`，移动左右 VR 手柄，确认 MuJoCo 双臂运动；断开/恢复输入时观察安全保持和重新接纳。
5. `--disable-hands` 验收通过后，再注入 Manus 参数验证舞肌二代手；两者分别记录日志和 HDF5。
6. 退出后检查没有残留 driver/M0/TJVR、`7447`/`15000`监听或不应保留的 `9999` forward。

## 风险与回滚

- ROS 依赖体积较大：通过嵌套 Pixi 和锁文件隔离，不把依赖加入当前默认环境。
- 原版源码许可/来源：随包保存许可和来源 manifest；不复制设备私有资产。
- PICO APK 与 Linux 源码版本不匹配：启动器在 raw readiness 阶段明确报错，不继续启动 IK。
- ROS domain 或 localhost 配置不一致：嵌入启动器统一注入 domain，健康检查失败时停止，不静默降级。
- 回滚只需停用新 profile/删除新增 vendor 和启动器；现有 PICO2、历史 TJVR 和当前 IK 文件不修改默认路径。

## 完成定义

- 当前工程能在不引用外部 PICO_tracker checkout 绝对路径的情况下，从一个新 profile 启动完整嵌入的 driver/M0/TJVR→当前工程 MuJoCo 链路。
- 原版 PICO+VR 手柄机械臂动作与已验证外部链路一致。
- `pico2_hands_sim` 的既有测试和设备命令仍通过。
- `--disable-hands` 可以在无 Manus 时完成双臂仿真验收；Manus 参数存在时可继续进入现有二代手 retarget。
- 所有新增代码、脚本、配置和测试无开发机绝对路径；未提交 APK、个人标定、SDK 私有库或构建产物。
