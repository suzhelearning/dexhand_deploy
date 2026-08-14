# IK 后端接口与切换

`tianji_kinematic_sim` 只依赖 `ArmIkSolver`（接口版本
`arm_ik_solver_v1`），不再直接依赖某一种 IK 实现。当前注册了三个后端：

- `pinocchio_cpp`：默认后端，使用项目 URDF 和 Pinocchio；
- `pinocchio_qp`：Pinocchio 速度级 box-QP，将末端任务、连续性、自然姿态
  和奇异恢复统一优化，并把关节位置与速度作为硬约束；
- `tianji_official`：由隔离 worker 加载天机官方 `libKine.so` 和对应
  `*.MvKDCfg`，主 ROS 进程通过本机 Unix socket 调用。

三种后端复用完全相同的 ROS 输入、关节输出、回零轨迹和真机安全桥。
切换后端不需要修改这些节点。

## 接口契约

公共接口位于
`include/pico_body_tianji/ik/arm_ik_solver.hpp`，固定约定如下：

- 左右臂关节顺序均为 `Joint1` 到 `Joint7`，接口内单位为弧度；
- 位姿是对应 `Base_L/Base_R` 到 `TCP_Link_L/TCP_Link_R` 的变换，
  平移单位为米；
- 臂角方向使用现有 `zsp_para` 参考平面方向约定，不是物理肘偏移方向；
- `accepted=true` 的输出才会被上层采用；
- 单帧关节变化不得超过 `maximum_joint_step_rad`；节点还会独立复核这一
  限制，后端输出 NaN 或越过公共步长限制时会保持上一帧关节角；
- 后端无法提供的诊断量使用 NaN，发布状态时转换为 JSON `null`。

新增其他实现时，只需继承 `ArmIkSolver`，再在
`src/arm_ik_factory.cpp` 注册名称。ROS 节点和真机桥不应包含厂商专用逻辑。

## 目录与 profile

公共接口与各后端头文件按同一规则组织：

```text
include/pico_body_tianji/ik/
├── arm_ik_factory.hpp
├── arm_ik_solver.hpp
├── pinocchio_cpp/pinocchio_arm_ik.hpp
├── pinocchio_qp/pinocchio_qp_arm_ik.hpp
└── tianji_official/
   ├── tianji_official_arm_ik.hpp
   └── tianji_official_ipc.hpp
```

纯手柄的三个后端也各自使用独立 profile：

```text
config/ik/
├── pinocchio_cpp/controller_only.yaml
├── pinocchio_qp/controller_only.yaml
└── tianji_official/controller_only.yaml
```

`config/controller_only_ik.yaml` 只保留纯手柄输入、回零和公共
安全参数。启动器根据 `ik_backend` 只追加加载对应后端的
`controller_only.yaml`。

## 启用 Pinocchio QP

纯手柄 profile 位于：

```text
config/ik/pinocchio_qp/controller_only.yaml
```

它按 90 Hz 设置期望笛卡尔速度、位置/姿态相对权重、每关节速度上限、
连续性、Home 自然姿态、关节限位减速带和奇异恢复调度。直接运行：

```bash
pixi run sim_controller_only_qp
```

QP 使用工程内的 7 变量 active-set box 求解器，不增加共享库依赖。任务残差
是软代价，因此不可达目标仍能返回连续安全解；关节位置和速度上下界始终是
硬约束。`left/right_position_velocity_residual_m_s`、姿态速度残差、求解迭代
数和激活约束数会发布到 `/pico_body_sim/status`，供 replay 调参使用。

## 启用天机官方 IK

部署任务会把本机 SDK 中的 `libKine.so` 和配置复制到项目私有
`runtime/tianji_official`。先确认设备确实是 Marvin M6 CCS V4.0，并使用与它匹配的
`ccs_m6_40.MvKDCfg`。错误机型配置可能返回数值正常但物理错误的结果。

纯手柄配置位于
`config/ik/tianji_official/controller_only.yaml`。启动时设置：

```yaml
tianji_kinematic_sim:
  ros__parameters:
    ik_backend: tianji_official
    official_ik_library: ""
    official_ik_config: ""
```

正式 runtime 启动器会自动传入内置 SDK 路径；只有需要测试其他 SDK
版本时才在 YAML 中显式覆盖绝对路径。选择默认 `pinocchio_cpp` 时不会
启动 worker，也不会加载厂商库。

## 构建与部署

首次准备锁定环境，然后编译、部署：

```bash
pixi install --locked -e ik-build
pixi run -e ik-build build-ik
pixi run -e ik-build deploy-ik
```

`build-ik` 会校验 Ubuntu 22.04、GCC 11、ROS Humble 16.0.19，且 Pixi
中的 Pinocchio 4.0.0 二进制必须与 `runtime/pin` 逐字节一致。`deploy-ik`
会先将旧文件备份到 `staging/runtime-backup`，再更新正式二进制、worker、
probe，并将整个源码 `config/` 递归同步到 runtime，最后更新
`RUNTIME_TREE_SHA256`。

官方适配器完成以下边界转换：

- 关节角：弧度 ↔ 度；
- TCP 平移：米 ↔ 毫米；
- 左右臂：`ArmSide::kLeft/kRight` ↔ 厂商 serial `0/1`；
- 臂角方向：前三项写入 `m_Input_IK_ZSPPara`，其余三项为零；
- 厂商超工作空间、奇异或关节超限标志转换为拒绝采用的 `IkResult`。

## 离线验证

Pinocchio QP probe 会测试左右臂保持、三轴位移、旋转、组合目标、不可达
目标及恢复，并报告求解耗时：

```bash
staging/ik/lib/pico_body_tianji/pinocchio_qp_ik_probe \
  src/pico_body_tianji/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf
```

随项目安装的 `tianji_official_ik_probe` 会分别对左右臂执行 FK→IK
闭环、`zsp_para`、公共步长限制和不可达目标拒绝检查。它不连接控制器，
也不会驱动真机：

```bash
runtime/pico_body_tianji/lib/pico_body_tianji/tianji_official_ik_probe
```

无参数时使用 `runtime/tianji_official` 内置文件。

首次在真机使用官方后端前，还应运行现有仿真，小范围移动手柄并比较目标
TCP、求解 TCP 和 14 关节输出，再按项目安全检查表连接实体机械臂。
