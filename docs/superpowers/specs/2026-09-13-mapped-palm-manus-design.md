# Mapped corrected-palm Velocity QP＋Manus 移植设计

日期：2026-09-13。状态：设计已由用户确认，软件实现及离线/仿真验证见
[移植记录](../../mapped-palm-porting.md)。已进行真实 PICO＋VR 手柄的仅机械臂仿真测试；
新后端＋真实 Manus 的联合验收仍待执行。转腕精度限制见
[对照报告](../../mapped-palm-rotation-comparison.md)，提交状态以 Git 为准。

## 目标与固定基线

在当前 PICO＋双 VR 手柄＋Manus 的 MuJoCo 流程中新增可选择的机械臂后端，
保持现有 SPARK 默认值、PICO2 裸手流程及 Manus Hand2 行为。

- 参考仓库：`TJ_arm_control`，分支 `feature/pico-manus-teleop-experiments`。
- 用户确认算法：`pico_ee_mapped_corrected_palm_velocity_qp`。
- 用户确认配置：`config/qp_ik_pico_ee_bandwidth_velocity_qp.yaml`。
- tag：`pico-mapped-wuji-hand2-bandwidth-smoothness-baseline-20260906`。
- tag 对应 commit：`2bcfe09e2c78a48c7ba63943deef83d04a139ff8`。
- 当前参考 HEAD：`d623c1b8df0edfba18e1d8b78f93a9f30ef612eb`。
- 核对结果：tag 到 HEAD 的 `src/`、`include/`、`config/`、`models/` 没有差异；
  Viewer 默认入口和构建测试采用 HEAD 行为，算法/配置闭包按 tag 记录原始摘要。

参考仓库的 `run_pico_mapped_wuji_hand2_teleop.sh` 仍指定旧的
`qp_ik_pico_ee_mapped_corrected_palm_velocity_qp.yaml`，不作为本次参数来源。
与正式 bandwidth 配置相比，旧配置的 LF 平移/旋转前馈增益分别为 0.28/0.30，
正式配置为 1.00/0.80；正式配置另包含 normalized_split 平滑项和 decoupled
零空间权重。禁止混用两份配置后宣称与用户验证版本一致。

## 算法与输入合同

```text
内嵌 PICO driver → M0 corrected skeleton → TJVR v4
  → mapped corrected palm＋同帧肩/肘/腕臂角
  → LF/HF Cartesian feedforward → Headroom
  → EE Velocity QP＋臂角/Active vector nullspace → qdot/model_reference
  → 当前协调器 → MuJoCo 天机双臂

Manus rawviz → 现有语义骨架转换 → 官方 Hand2 retarget
  → 当前手部授权与输出 → 同一 MuJoCo 舞肌二代手
```

“不用 SPARK 骨架”是指不执行 SPARK 骨长缩放和 SPARK IK；仍需 TJVR corrected
上肢骨架的位置及旋转，不能用包内普通左右末端 pose 代替 mapped 掌心，也不能丢掉
肩/肘/腕平面臂角。掌心旋转必须保留 `selectMappedCorrectedPalm` 的左右基变换。

保持 200 Hz、`model_reference`、原版双臂更新顺序、源时间与接收时间语义、QP 约束、
前馈与 Headroom 状态、stationary/settled hold、失效及恢复处理。
使用 `hand_tcp_frame_L/R` 做 FK、Jacobian 和误差计算，模型缺少任一坐标系时拒绝启动。
不要静默回退到法兰 TCP，也不要默认替换为 Pinocchio 实验运动学实现。

不启用原版默认关闭的 C1、端点预测、Generic Cartesian OTG、DLS posture guidance、
参数 sweep 或 motion-gain 控制。默认关闭的实验模块若属于编译依赖可随闭包保留，
但不得改变运行路径。保留原版内部参考处理与最终 QP 平滑项，不以“直出”为由删减。

## 接入选择

采用独立原生后端，不替换现有 `spark_headroom` 源码或 PICO2 的 v131 库。
直接扩展 SPARK 核心虽可少复制公共代码，但会影响旧控制器和配置解析；另起独立 Viewer
则无法复用当前协调、Manus、HDF5 与退出管理。因此采用独立 worker＋现有会话壳。

计划的文件边界：

- `src/tianji_teleop/src/ik/mapped_palm/`：固定来源的 C++ 核心、双臂周期适配、worker、
  CMake 和 source manifest；使用独立命名空间及构建目标。
- `src/tianji_teleop/src/ik/mapped_palm/include/tianji_mapped_palm/`：对应闭包头文件，
  与源码一起隔离，不加入旧后端的 include 搜索路径。
- `tools/mapped_palm_native/`：固定依赖环境；不改现有 SPARK 或 v131 的依赖版本。
- `src/tianji_teleop/assets/mapped_palm/`：原版模型及必要资产，记录 TCP/关节/网格摘要。
- `src/tianji_teleop/src/ik/mapped_palm/config/bandwidth.yaml`：原版 bandwidth 参数，
  只允许必要的仓库相对资源路径修改，所有修改进入 manifest。
- `hand_tracking/session_config.py` 和输入后端校验：新增精确后端名称与输入限制。
- `producers/spark/live_simulation.py`、`live_runner.py`：以小范围工厂接缝选择机械臂
  worker、模型、配置及身份；保留 SPARK 默认参数。新算法不构造 SPARK guidance。
- 录制：复用 C++ HDF5 和现有 Manus 数据集，记录真实后端、来源摘要、配置、TCP、
  输入接纳/原生调用及重置审计；不能沿用错误的 `ik_spark_headroom` 身份标签。

对用户暴露的目标入口是现有 `pico_vr_manus_sim` 加：

```text
--ik-backend pico_ee_mapped_corrected_palm_velocity_qp
```

启动器仍管理 driver/M0/TJVR、rawviz、录制和清理；支持现有 `--disable-hands`。
`s/h/r/q`、guard 和仿真能力授权继续复用。首先只开放 `reference_direct`：
额外目标整形和关节轨迹为 passthrough，命令步长裁剪关闭，关节硬限位来自一致模型。
PICO2 或 XR pose-only 输入选用本后端时应明确报错，不隐式转换输入合同。
原来的 SPARK profile 和无覆盖参数的启动指令保持原行为。

## 实施与验证顺序

1. 固定源码、配置、模型、构建依赖及逐文件摘要，增加来源一致性测试。
   正式运行时不读取参考仓库绝对路径；参考仓库只作为移植和测试 oracle。
2. 从原版 Viewer 抽取 mapped 路径双臂周期，保留输入接纳、epoch、按钮、stale、
   前馈更新、Headroom 反馈和模型状态推进顺序，构建独立 worker。
3. 构建独立原版 oracle，以相同 TJVR 包、接收时钟、控制 tick、初始状态逐周期比较
   mapped pose、臂角、q/qdot/qddot、Headroom、任务缩放、hold 和 reset 决策。
   包括源帧重复、乱序、无新帧、追踪失效/恢复、epoch 变化、退化臂平面和限位附近。
   离线放宽求解墙钟预算只能进入显式测试模式，不能改变实时默认值。
4. 核对内嵌 bridge 输出的 corrected skeleton 与参考输入语义；校验姿态基、单位、
   骨架索引、坐标系、映射比例和个人标定。不能只依据 TJVR 版本号认定链路一致。
5. 接入后端选择和同一 MuJoCo/Manus/HDF5 会话。验证缺模型/worker 时启动前报错，
   禁用手时不启动 rawviz；双手、录制、Home/rearm、Ctrl-C 与异常清理均需覆盖。
6. 跑旧 SPARK、PICO2 的配置/合成输入闭环与完整软件回归，确认默认配置未改变。
7. 更新 README 新后端命令，先由用户用 PICO＋控制器测试双臂，再加入 Manus。
   分别记录原版与移植版的输入、跟踪误差、延迟、抖动、QP 失败及退出完整性。

验收分开记录：源码/配置一致性、离线周期一致性、当前框架仿真闭环、真实输入设备验收。
编译通过、算法名称相同或沿用同一求解器均不能代替完整一致性验证。

## 当前边界

本次只扩展仿真算法选项，不启用机器人真机。新功能没有完成前不得在 README 标记为可用。
按既有协作要求顺序执行、不使用子智能体；新需求的代码提交和 push 另行处理。
