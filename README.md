# PICO → Marvin 天机双臂遥操作（Pixi 单文件便携包）

本包适用于 Ubuntu 22.04 x86_64，可在同一台电脑上完成：

- PICO 左右手柄和 SMPL 上肢数据读取；
- 可切换的 C++ 双臂 IK（Pinocchio / 天机官方 `libKine`）；
- RViz 与 MuJoCo 纯运动学仿真；
- Marvin SDK 真机关节位置遥操作；
- 真机状态监控和安全停机。

运行时不需要 Docker、不要求预装 ROS 2，也不需要现场编译。本包不包含
Wuji 手部资产或描述包；只保留 PICO 输入和天机坐标转换所需的最小
白名单模块。

![PICO 到 Marvin 天机双臂遥操作数据流](docs/data_flow.svg)

正确运行关系是终端 1 持续运行一套 `sim`，终端 2 只运行一套
`real`。`real` 复用 `sim` 的 PICO、SMPL 和 IK 输出，不会再
启动第二套输入或 IK 节点。

## 是否可以独立控制天机

可以。本仓库已经携带运行所需的 ROS 2、Pinocchio、RViz、MuJoCo、
PICO Python/原生 SDK、Marvin Python SDK 和本项目节点。在支持的电脑
上克隆后，不依赖 Docker、系统预装 ROS 2、官方 Wuji 仓库、FxStation
或官方 `libKine` 即可启动完整链路：

```text
PICO + SMPL → 可配置 IK → 真机安全桥 → Marvin SDK → 天机双臂
```

这里的“独立”不代表不需要外部设备和服务。运行时仍然需要：

- Ubuntu 22.04 x86_64 电脑和 Pixi；
- 本机正在运行的 PXREA Unity/XRoboToolkit 数据服务；
- 已唤醒并持续发送手柄、A 键和 SMPL 数据的 PICO；
- 与电脑有线直连、已上电并释放实体急停的 Marvin 控制器。

`pixi run sim` 独立产生 PICO/SMPL/IK 目标；`pixi run real` 只连接
Marvin 并复用该目标。因此二者属于同一个独立项目，但真机运行时必须
保持一套 `sim` 与一套 `real` 同时运行。

默认使用 `pinocchio_cpp`。工程已提供稳定的 `ArmIkSolver` 接口、速度级
`pinocchio_qp` 和 `tianji_official` 运行时适配器，可通过 YAML 切换且无需
改动 ROS 话题和真机安全桥。各后端配置和离线验证方法见
[IK 后端接口与切换](docs/ik_backends.md)。

## 运行前提

- Ubuntu 22.04 x86_64；
- 可以访问软件源的网络，用于首次安装 Pixi 环境；
- PXREA Unity/XRoboToolkit 服务和 PICO 在同一台电脑可用；
- 真机测试时，电脑有线网卡与 Marvin 控制器位于同一网段。

先检查 Pixi：

```bash
pixi --version
```

如果命令不存在，可按
[Pixi 官方安装说明](https://pixi.sh/latest/installation/)安装：

```bash
curl -fsSL https://pixi.sh/install.sh | sh
source ~/.bashrc
pixi --version
```

## 第一次使用：推荐顺序

### 1. 获取简化版项目并进入目录

源码开发可从 GitHub 克隆：

```bash
git clone git@github.com:suzhelearning/tianji_teleop.git
cd tianji_teleop
```

Git 仓库只跟踪源码、脚本、配置和 Pixi 锁文件，不跟踪 `.pixi/`、
`runtime/`、`vendor/`、`build/`、`log/`、`staging/` 等本地环境或
生成物。因此源码克隆不能直接运行 `doctor`、仿真或真机任务。

运行仿真或真机时应使用独立压缩包，其中包含经过校验的运行时和厂商
二进制。只需要复制这一份压缩包到目标电脑，不需要同时复制源码仓库、
Docker 镜像、ROS 工作区或外置校验文件。执行：

```bash
tar -xzf pico_tianji_teleop_standalone_*.tar.gz
cd pico-tianji-real-teleop
```

后续命令都在 `pico-tianji-real-teleop` 目录中执行。运行
`pixi run doctor` 时会使用包内的 `VENDOR_SHA256SUMS` 和
`RUNTIME_TREE_SHA256` 自动检查厂商文件及运行时树，不依赖压缩包旁边
的第二个文件。

### 2. 安装锁定环境并做离线检查

```bash
pixi install --locked
pixi run doctor
pixi run test
```

- `pixi install --locked`：安装包内锁定的 Python、NumPy、SciPy 和
  MuJoCo 版本；
- `pixi run doctor`：检查 ROS、Pinocchio、RViz、MuJoCo、URDF、mesh
  和厂商 SDK 文件；
- `pixi run test`：短暂启动纯运动学节点并检查 ROS 话题链路。

这三个步骤不会连接或驱动实体机械臂。`doctor` 会离线加载 Marvin SDK
检查文件与 ABI，但不会建立控制器连接。

普通使用者不需要在系统中安装 ROS 2。本项目从
`runtime/ros/humble` 加载随包 ROS 运行时；前提是拿到完整项目，不能只
复制 `src`、`pixi.toml` 和 `pixi.lock`。完整项目至少应保留
`runtime/ros/humble`、`runtime/pico_body_tianji`、`runtime/pin`、
`runtime/abi` 和 `vendor`。

`pixi install --locked` 一次只安装 `default` 环境，不会安装或检查
`ik-build`。直接运行 `pixi run sim` 也会自动安装缺失的默认环境，但首次
使用仍建议显式执行上面的安装和检查命令。两套 Pixi 环境的用途如下：

| Pixi 环境 | 安装命令 | 是否要求系统 ROS 2 |
| --- | --- | --- |
| 普通运行 `default` | `pixi install --locked` | 否，使用项目内置 ROS |
| 重新编译 `ik-build` | `pixi install --locked -e ik-build` | 安装本身不要求；执行 `build-ik` 时要求系统 ROS Humble |

普通使用者不要执行 `pixi install --all`，也不需要执行 `build-ik`。只有修改
C++ IK 源码并准备重新编译的开发者，才需要下一节。

### 3. 可选：替换 IK、重新编译并运行

本节是 IK 开发流程，普通使用者可以直接跳到“准备 PICO 输入”。所有 IK
切换、编译、部署和启动命令集中放在这里。不要直接修改 `runtime` 下的
YAML 或二进制：`runtime` 是实际运行目录，其 IK 内容由 `src` 编译并通过
`deploy-ik` 部署。

重新编译要求 Ubuntu 22.04 x86_64，并在系统中准备：

- `/opt/ros/humble` 下的 ROS Humble 开发环境；
- GCC/G++ 11、CMake 和 colcon；
- 与工程检查脚本匹配的 ROS 开发包版本。

Pixi 只负责锁定 Python 和 Pinocchio 构建依赖，不会替代系统 ROS。没有
安装系统 ROS 时，普通运行不受影响，但执行 `build-ik` 会明确报告缺少
`/opt/ros/humble/setup.bash`。

运行模式的公共配置如下：

| 运行模式 | 修改的配置文件 | Sim 命令 | Real 命令 |
| --- | --- | --- | --- |
| PICO + SMPL 全身链路 | `src/pico_body_tianji/config/mode/full_body/preview.yaml` | `pixi run sim` | `pixi run real` |
| 纯手柄链路 | `src/pico_body_tianji/config/mode/controller_only/controller_only_ik.yaml` | `pixi run sim_controller_only` | `pixi run real_controller_only` |

配置先按运行模式分组；每种模式各有一个 Sim/IK 配置和一个 Real 配置：

```text
src/pico_body_tianji/config/mode/
├── full_body/
│   ├── preview.yaml
│   └── real.yaml
└── controller_only/
    ├── controller_only_ik.yaml
    └── controller_only_real.yaml
```

`preview.yaml` 和 `controller_only_ik.yaml` 已包含三种 IK 的全部参数。
程序只使用当前后端需要的部分。IK 不在命令行选择，只需在对应模式的
YAML 中修改一行：

```yaml
tianji_kinematic_sim:
  ros__parameters:
    ik_backend: pinocchio_qp  # 或 pinocchio_cpp / tianji_official
```

因此运行命令始终只有
`sim`、`real`、`sim_controller_only`、`real_controller_only` 四个，
不会把模式名与 IK 名组合成新命令。

官方库路径和机型配置保持空字符串即可，运行时包装器会使用项目内的
`runtime/tianji_official`。修改完成后，在项目根目录依次执行：

```bash
# 首次配置 ik-build 或 pixi.lock 更新后执行
pixi install --locked -e ik-build

# 编译 src 下的 C++ IK，产物先进入 staging/ik
pixi run -e ik-build build-ik

# 检查 QP 左右臂收敛、不可达恢复和求解耗时
staging/ik/lib/pico_body_tianji/pinocchio_qp_ik_probe \
  src/pico_body_tianji/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf

# 把新二进制、官方 SDK 和完整 config 目录部署到 runtime
pixi run -e ik-build deploy-ik

# 检查部署后的运行环境和 ROS 数据链路
pixi run doctor
pixi run test
```

如果只修改了 YAML，且 `staging/ik` 中已有上一次的完整编译产物，可以
跳过 `build-ik`，只执行 `pixi run -e ik-build deploy-ik`。该命令
会递归同步整个 `src/pico_body_tianji/config/` 到 runtime，包括两种模式的
四个 YAML。部署时还会移除 runtime ELF 的 DWARF
调试信息，完整 `RelWithDebInfo` 产物继续保留在 `staging/ik`，
避免将调试符号作为大文件提交到 Git。修改了
`src/pico_body_tianji/src/`、头文件或 `CMakeLists.txt` 时，必须重新执行
`build-ik` 和 `deploy-ik`。

全身模式编译部署完成后，终端 1 和终端 2 分别执行：

```bash
# 终端 1
pixi run sim

# 终端 2：确认仿真正常后再执行
pixi run real -- \
  --confirm-real \
  --velocity-ratio 20 \
  --acceleration-ratio 20
```

纯手柄模式使用：

```bash
# 终端 1
pixi run sim_controller_only

# 终端 2：确认仿真正常后再执行
pixi run real_controller_only -- \
  --confirm-real \
  --velocity-ratio 20 \
  --acceleration-ratio 20
```

低速验收完成后，常规跟随 profile 可直接使用脚本默认的 65% 速度、85%
加速度；纯手柄 IK 限制为 75.6°/s，真机桥限制为 80°/s：

```bash
pixi run real_controller_only -- --confirm-real
```

验证 QP 时，在对应的公共 YAML 中把 `ik_backend` 改为
`pinocchio_qp`，部署后仍使用上面的同一条 Sim 命令。QP profile 是基于
90 Hz、现有 0.68° 公共单步契约和离线轨迹得到的保守初值；连接真机前
仍必须完成手柄 replay、低速和小工作空间验证。

Real 不会再启动一套 IK，只复用对应 Sim 发布的 14 关节目标。真机前必须
确认只有一套 Sim 在运行、FxStation 已关闭、实体急停已释放，并等待 Real
显示 `phase=armed_idle` 后再单击右手柄 A。结束时先在 Real 终端按
`Ctrl+C`，看到 `Robot released` 后再关闭 Sim。

### 4. 准备 PICO 输入

1. 在同一台电脑启动 PXREA Unity/XRoboToolkit 服务；
2. 戴上并唤醒 PICO；
3. 确认左右手柄、右手柄 A 键和 SMPL Body 持续发送；
4. 保持双手在舒适位置，暂时不要按 A。

XRoboToolkit 默认由本机 `127.0.0.1:60061` 提供数据。若日志一直停在
`Waiting for device discovery`，先处理 PICO/XRoboToolkit 连接，不要
启动真机。

下一步运行 `pixi run sim` 后，输入正常时，启动终端会出现设备发现/
连接信息，RViz 中的 SMPL 骨架、左右目标末端会随人体和手柄运动。
若骨架或目标保持不动，即使服务进程仍在，也不能视为数据正在持续刷新。

如果要先确认“不佩戴腿部 Motion Tracker 时，左右手柄是否仍能到达
MiniPC”，关闭正在运行的 `sim`/`real`，关闭两枚 Motion Tracker，保持
PICO、左右手柄和 XRoboToolkit 服务在线，然后执行：

```bash
pixi run pico-probe -- --duration 15
```

检测过程中分别移动左右手柄。该命令只读取 SDK，不启动 IK、不发布 ROS
控制目标，也不连接 Marvin。最终出现
`controller_link_live: true` 表示双手柄有效位姿和递增时间戳持续到达；
`body_data_received: false`、`motion_tracker_count: 0` 是关闭腿部 Tracker
后的预期结果，不会导致检测失败。

### 5. 运行纯手柄到 IK 输出

确认 `pico-probe` 通过后，继续保持 XRoboToolkit 的 Controller 和 Send
开启，关闭 Body Tracking。确认没有运行 `sim`、`real` 或其他 SDK
客户端，在终端 1 执行：

```bash
pixi run sim_controller_only -- --topics-only
```

该命令只启动 `pico_controller_only_input` 和
`tianji_kinematic_sim`，不启动 RViz/MuJoCo，也不连接 Marvin。
看到安全初始位就绪后，松开再单击右手柄 A，然后小幅移动左右手柄。

在终端 2 查看 IK 输出的左右臂 14 个关节角（弧度）：

```bash
pixi run controller-only-joints
```

主要输出话题为：

```text
/pico_body_sim/left_arm/joint_commands   # 左臂 7 关节，单位为度
/pico_body_sim/right_arm/joint_commands  # 右臂 7 关节，单位为度
/pico_body_sim/model_joint_states        # 双臂 14 关节，单位为弧度
```

要同时查看 RViz 和 MuJoCo，先关闭上述无界面任务，再执行：

```bash
pixi run sim_controller_only
```

松开再单击右手柄 A 后，小幅移动左右手柄即可观察对应机械臂。

### 6. mocap 手腕轨迹回放仿真（可选）

不需要 PICO/真机时，可以把 mocap-acquisition 采集的 HDF5（v4.0）里
录制的左右手腕位姿按录制节奏回放成轨迹跟踪仿真，机器人会像在线
PICO 遥操作一样跟踪这条轨迹：

```bash
pixi run sim_mocap -- /home/current/data/20260819/20260819_151737_102784_take003.h5
```

回放经过与在线 PICO 相同的映射链路（增量相对参考帧、One-Euro 滤波、
工作空间/速度整形），默认 **1:1 目标整形**（`translation_gain=1.0`）：
命令 +x 移动 50mm，目标位移即 50.0mm、solved 位移 49.9–50.0mm；
在线 PICO 链路刻意保留 0.90 安全缩放。参考帧等效于按右手柄 A 的
时刻；**键盘 's' 控制开始与结束**（按 s 开始回放，回放中再按 s
结束并回 Home；`--control auto` 恢复自动流程）。支持 `--speed`
倍速、`--yaw-deg` 朝向标定和 `--topics-only` 等模式；单侧手腕
完全无效（如该 take 左手全 NaN）时该侧机械臂保持 Home。可用
`pixi run mocap-step-h5 -- --output step.h5 --axis z --dir neg --mm 50`
生成“机器人 +x 移动 50mm”的合成验收轨迹。
详细说明见 [docs/mocap_replay.md](docs/mocap_replay.md)。

### 7. 启动真机

保持已经验收的仿真任务运行：普通模式使用 `pixi run sim`，纯手柄模式
使用 `pixi run sim_controller_only`。真机任务只启动 Marvin 安全桥，
复用对应的主机侧 PICO + IK 链路，不会再启动第二套 PICO/IK。
同时确认没有其他旧版 PICO 天机任务、官方控制节点、FxStation 或
Marvin SDK 会话在运行。

仿真主机链路和真机桥分别持有跨解压目录运行锁，因此可以同时运行，
但各自不能重复启动。正常退出、`Ctrl+C` 或子节点异常退出时，脚本只
停止自己管理的进程；如果主脚本曾被强制杀死，同类任务下次启动会根据
受管 PID 和进程启动时间自动清理遗留进程。真机桥启动前还会确认 ROS
图中恰好只有一个 PICO 输入节点和一个 IK 节点。外部旧节点
不会被冒险误杀，而会触发拒绝启动。

#### 7.1 配置天机有线网口

调试网线连接控制器 ETH1 或 ETH3。官方调试网段使用：

| 设备 | IPv4 地址 |
|---|---|
| 电脑有线网卡 | `192.168.1.165/24` |
| Marvin 控制器 | `192.168.1.190` |

Wi‑Fi 可以继续用于互联网，但机器人有线连接不要配置默认网关。先查找
有线网卡名：

```bash
nmcli device status
ip -brief link
```

以下假设网卡是 `enp3s0`，必须按本机输出修改：

```bash
WIRED_IF="enp3s0"

nmcli connection show tianji-static >/dev/null 2>&1 || \
  sudo nmcli connection add \
    type ethernet \
    con-name tianji-static \
    ifname "$WIRED_IF"

sudo nmcli connection modify tianji-static \
  connection.interface-name "$WIRED_IF" \
  ipv4.method manual \
  ipv4.addresses 192.168.1.165/24 \
  ipv4.gateway "" \
  ipv4.dns "" \
  ipv4.never-default yes \
  ipv6.method disabled

sudo nmcli connection up tianji-static
```

验证地址和路由：

```bash
WIRED_IF="enp3s0"
ip -4 -brief address show dev "$WIRED_IF"
ip route get 192.168.1.190
ping -c 3 192.168.1.190
```

`ip route get` 必须显示从所选有线网卡发出，并包含
`src 192.168.1.165`。如果控制器禁止 ICMP，ping 失败不一定表示 SDK
不可连接；应先用 FxStation 验证连接，随后**完全关闭 FxStation**再
启动本项目。若路由走 Wi‑Fi 或其他网卡，不要启动真机。

需要恢复普通 DHCP 时：

```bash
sudo nmcli connection modify tianji-static \
  ipv4.method auto \
  ipv4.addresses "" \
  ipv4.never-default no \
  ipv6.method auto
sudo nmcli connection up tianji-static
```

#### 7.2 启动真机桥

首次在新电脑联调，推荐从较低控制器比例开始：

```bash
pixi run real -- \
  --confirm-real \
  --velocity-ratio 20 \
  --acceleration-ratio 20
```

确认空载低速运行正常后，可使用当前默认值：

```bash
pixi run real -- --confirm-real
```

默认控制器速度为 50%，加速度为 70%。也可明确指定控制器地址：

```bash
ROBOT_IP="192.168.1.190"  # 如控制器地址不同，请修改这里
pixi run real -- \
  --confirm-real \
  --robot-ip "$ROBOT_IP" \
  --velocity-ratio 50 \
  --acceleration-ratio 70
```

启动后程序会自动执行：

```text
连接 Marvin
  → 检查并尝试清除已经释放的历史错误
  → 检查控制器、双臂和 14 个伺服轴
  → 读取当前实测关节角
  → 以实测角无跳变进入 state=1 关节位置模式
  → 以不高于 10°/s 缓慢到达安全零位
  → 等待主机状态和反馈稳定
  → 进入 phase=armed_idle
```

因此，执行真机命令后、按 A 之前，机械臂也会自动缓慢移动到安全零位。
必须提前清空双臂运动空间并握住实体急停。

只有看到以下日志后才能按右手柄 A：

```text
真机链路已就绪：保持安全零位，按右手柄 A 开始。
```

或者在状态中确认：

```text
phase=armed_idle
```

此时单击右手柄 A 开始遥操作；再次单击 A，或 PICO/SMPL 数据中断，
机械臂会缓慢回安全位。真机终端按 `Ctrl+C` 会安全停止并释放 Marvin
SDK 会话，但不会关闭仿真。结束全部工作时，先关闭真机终端并等待
`Robot released`，再到仿真终端按 `Ctrl+C`。

#### 7.3 启动纯手柄真机桥（不使用 Body/Tracker）

终端 1 保持纯手柄仿真运行：

```bash
pixi run sim_controller_only
```

此时不要按 A，先确认仿真双臂已经处于安全初始位。终端 2 首次联调建议
以较低比例启动纯手柄真机桥：

```bash
pixi run real_controller_only -- \
  --confirm-real \
  --velocity-ratio 20 \
  --acceleration-ratio 20
```

它复用终端 1 的纯手柄输入、IK 和 14 关节命令；不会启动第二
套输入或 IK，也不要求 SMPL Body 和腿部 Motion Tracker。原有的回零、
命令新鲜度与同步、反馈、关节硬限位、输出斜坡、跟踪误差和安全停机检查
保持不变。

只有看到下面的日志，或状态显示 `phase=armed_idle` 后才能按 A：

```text
真机链路已就绪：保持安全零位，按右手柄 A 开始。
```

此后右手柄 A 同时控制仿真显示和真机；再次按 A 或手柄数据中断时，真机
缓慢回安全位。结束时先在终端 2 按 `Ctrl+C` 并等待 `Robot released`，
再关闭终端 1 的仿真。

### 8. 另开终端监控真机状态

进入同一解压目录后执行：

```bash
pixi run status
```

重点查看：

- `phase`：当前真机阶段，正常待机应为 `armed_idle`；
- `robot_connected`：是否已连接控制器；
- `error_codes`：左右臂控制器错误码；
- `servo_error_reports`：左右臂 14 个伺服轴错误明细；
- `tracking_error_detail`：发生跟踪误差时的机械臂、关节和角度；
- `error`：启动失败或安全停机的直接软件判据；
- `readiness`：主机链路尚未就绪时的原因。

复现纯手柄真机跟随迟滞、左右臂不同步或空间边界问题时，另开终端执行：

```bash
pixi run controller-only-real-diagnostic -- --duration 60
```

在 60 秒内完成单臂、双臂同时运动和问题位置复现。诊断器只订阅 ROS
话题，不发布控制命令；结束后会打印输入速度/加速度、椭球工作空间、IK
单步、奇异/回退、软关节限位、真机输出斜坡、90 Hz 漏期和逐轴跟踪误差，
并将原始 JSONL 保存到 `diagnostics/`。传入 `--duration 0` 可一直采集到
`Ctrl+C`。

## 四个运行入口

| 命令 | 作用 | 是否连接真机 |
|---|---|---|
| `pixi run sim` | 同时启动 RViz 和 MuJoCo | 否 |
| `pixi run real -- --confirm-real` | 复用正在运行的 sim，启动真机桥 | **是** |
| `pixi run sim_controller_only` | 纯手柄 IK 的 RViz + MuJoCo 仿真 | 否 |
| `pixi run real_controller_only -- --confirm-real` | 复用纯手柄仿真，启动真机桥 | **是** |
| `pixi run sim_mocap -- TAKE.h5` | mocap HDF5 手腕轨迹回放为轨迹跟踪仿真 | 否 |
| `pixi run sim_mocap_step` | 键盘步进控制（动捕系 10mm/键，s 启停） | 否 |

`doctor`、`test`、`pico-probe`、`status`、`controller-only-joints` 和
`controller-only-real-diagnostic` 是检查/观测工具，不是新的运行模式。
`sim` 两个入口均可追加
`-- --rviz-only`、`-- --mujoco-only` 或 `-- --topics-only`，但入口名不变。
`sim_mocap` 默认是 preview-only 离线回放仿真，**键盘 's' 控制开始与
结束**（替代 PICO A 键；`--control auto` 可恢复自动）；`sim_mocap_step`
是键盘步进控制（上/下/左/右/1/0 = 动捕 ±z/∓z/±x/∓x/±y，每次 10mm，
**默认仅右臂**，左臂保持 Home；`--side both` 双臂同步）。
两者都可作为真机桥主机输入做确定性轨迹真机验收（见
[docs/mocap_real_acceptance.md](docs/mocap_real_acceptance.md)）。

## 控制原理

```text
PICO 左右手柄 + SMPL 上肢
  → 实时躯干坐标系下的相对末端变化
  → SMPL 肩—肘—腕臂角参考
  → ArmIkSolver（Pinocchio 或天机官方 IK）
  → 双臂关节命令安全门
  → Marvin SDK state=1 关节位置控制
```

PICO 手柄决定末端相对位置和姿态。SMPL 不直接决定末端位置，只提供
实时躯干坐标系和肩—肘—腕臂角参考，帮助选择更符合人体肘平面的冗余
关节解。

RViz 显示完整 Marvin URDF/mesh、SMPL 骨架、目标末端、当前 FK 和
肘平面参考。MuJoCo 只镜像
`/pico_body_sim/model_joint_states`，当前不执行动力学控制，也不会向
真机回写。

## 真机安全检查表

运行 `pixi run real` 或 `pixi run real_controller_only` 前必须逐项确认：

- 双臂 48V 动力电源已开启；
- 实体急停已释放、功能正常并在操作者手边；
- 双臂运动空间无人、无障碍，首次测试保持空载；
- 控制柜允许远程/自动控制；
- PICO 左右手柄持续刷新；普通模式还要求 SMPL Body 持续刷新；
- 已停止 FxStation、官方天机控制节点和其他 Marvin SDK 会话；
- 对应的 `pixi run sim` 或 `pixi run sim_controller_only` 已运行，且只有
  一套 PICO/IK 主机链路；
- 没有第二套旧仿真、旧真机桥或其他本项目控制进程；
- 仿真中左右臂方向、末端姿态、肘平面和回零均已验收；
- 当前控制器没有未释放的安全链或急停错误。

程序不会绕过仍然生效的实体急停、安全回路或控制器错误。反馈超时、
反馈序号停滞、实测越过物理硬限位、控制器/伺服报错、状态异常或跟踪
误差过大时，会锁存软件急停并释放 SDK 连接。锁存后应先查明原因，再
重新启动真机任务。

## 常见问题

### `pixi run doctor` 报 ROS/Pinocchio/ABI 失败

确认系统为 Ubuntu 22.04 x86_64，并从完整解压目录运行，不要只复制
`pixi.toml` 或 `scripts/`。若压缩包校验失败，应重新复制压缩包。

### RViz 或 MuJoCo 无法打开

本地图形桌面应执行：

```bash
echo "$DISPLAY"
pixi run sim
```

SSH 或无显示器环境使用：

```bash
pixi run sim -- --topics-only
```

远程启动 GUI 需要正确配置 X11/Wayland 转发；`--topics-only` 本身不需要
图形环境。

### RViz 打开但没有机械臂

先看启动终端是否有 `robot_state_publisher` 或 mesh 加载错误，再运行：

```bash
pixi run doctor
```

不要移动或删除解压目录中的 `runtime/`、`src/` 和 `vendor/`。

### 按右手柄 A 没有反应

常见原因：

- PICO 或 SMPL 数据没有持续刷新；
- 仿真尚未到安全初始位；
- A 键一直处于按下状态，没有形成“松开后单击”的上升沿；
- 真机尚未进入 `phase=armed_idle`；
- 运行了两套控制节点，导致话题或 SDK 会话冲突。

先松开 A，确认输入和状态正常，再单击一次。

### 出现 `not_at_home`

说明当前仿真关节还没有回到安全初始位。松开 A，等待回零完成，不要
连续按键。若长期不恢复，停止进程并重新运行仿真排查。

### 出现错误 13 或 `controller_emergency_stop`

错误 13 属于控制器/外部急停安全链，不能通过放宽 PICO 软件关节限位
解决。检查实体急停按钮、急停插头、安全输入回路和控制柜状态，并用
FxStation 或控制器事件日志定位。原因未解除前不要反复使能。

### 出现 `tracking_error`

查看 `pixi run status` 中的 `tracking_error_detail`，确认是哪一侧、
哪个关节以及目标角和实测角。常见原因包括关节实际跟随不足、机械负载
过大、目标变化过快或接近不可达/限位姿态。不要直接扩大误差阈值。

### 日志提示存在其他控制节点或 SDK 会话

关闭 FxStation、官方控制程序、重复的旧 `pixi run real`/`pixi run
sim` 终端及其他 Marvin SDK 程序。正常组合是一套 `sim` 主机链路加
一套 `real` 真机桥；不能有第二套 PICO/IK，也不能有第二个真机桥。
新版本会自动清理自己登记的残留进程；不受本包管理的 Docker/官方节点
必须在其原终端或原运行环境中关闭。

## 文件校验与来源

厂商二进制校验值保存在 `VENDOR_SHA256SUMS`，随包 ROS 运行时树校验值
保存在 `RUNTIME_TREE_SHA256`。厂商运行文件的来源与最小白名单说明见
`VENDOR_RUNTIME.md`。
