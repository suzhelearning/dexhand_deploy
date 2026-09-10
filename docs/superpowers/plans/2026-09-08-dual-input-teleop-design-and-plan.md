# PICO2 / VR + Manus 双输入遥操设计及实施方案

> 实施方式：使用 executing-plans，按阶段顺序执行；不使用子智能体。
> 本文件保留设计与分阶段实施要求；部分项目现已实现。当前可运行命令以README为准，逐项证据及未完成边界见`docs/dual-input-integration-progress.md`，不能将本文件的拟议命令一概当作可用入口。

**Goal:** 启动时选择 PICO2 裸手或 VR + Manus，分别控制当前工程的 MuJoCo 仿真及天机机械臂、舞肌二代手真机。

**Architecture:** 人端采集分为手骨架、机械臂位姿、操作事件三个通道。输入适配后共用目标、IK、retarget、会话授权和执行框架；输入模式、映射、算法、执行环境分别配置，通过能力校验约束组合。

**Tech Stack:** 现有 Python/C++、Pixi、Zenoh、MuJoCo、原生 IK、Wuji retarget/SDK；参考工程 ROS2 采集初期保留在独立进程环境。

## 1. 基线、范围与兼容约束

检查基线：分支 `PICO_Hand_Tracking`，本地提交 `63e1e0f`，标题 `feat: add PICO2 and Manus teleop pipeline`。编写本文件前工作区干净；上次 push 被 GitHub HTTP 403 拒绝，不能声称远端已有该提交。

现有能力与缺口：

| 内容 | 当前基线 | 本计划交付 |
|---|---|---|
| PICO2 TCP、头显/双腕/26点 | 已有 | 保持兼容，明确状态及版本契约 |
| Manus 25→21、TJVR v1–v4 | 已有 | 核对参考数据、完善冗余方向和操作事件 |
| PICO2 仿真双臂 | 初版已有，用户有实测反馈 | 保留既有命令和映射，增加回归验收 |
| 完整手势操作 | 当前输入/目标模块未见明确手势链 | 骨架识别基线、可选原生手势适配 |
| VR 手柄/Tracker 控制 | 参考工程具备，当前非完整迁移 | 接入采集、参考建立、按键、离合 |
| 二代手 retarget | Python dry 路径为简化几何算法 | 显式后端，共用仿真/真机算法输出 |
| 手跟踪真机 profile | 当前明确 simulation-only | 新增经验证的 real profile |
| 旧 mocap/H5/Regrind | 既有功能 | 保持原有入口、默认值和授权行为 |

全局约束：

- 仅支持停止会话后切换模式；运行中热切换不在本期范围。
- 同一会话只激活一种输入模式，不自动回退到另一种输入。
- 不改写现有 `hand_tracking_sim`、`hand_tracking_sim_manus` 默认配置；新行为使用新 profile。
- 保留 `--disable-hands`、现有映射、IK、目标整形、Ruckig、命令裁剪、限位来源和 overlay 语义。
- 不让旧 ROS2 驱动与当前执行器同时向同一机器人发布命令。
- 不将 C++ 一代手 retarget 作为二代手算法，不将 dry 几何近似宣称为官方 retarget。
- 真机不通过删除 `simulation_only` 校验解锁；新增能力校验和独立部署配置。
- v131 当前仿真速度参数及模型状态推进方式不是已验收的真机参数。
- 用户已授权复查后顺序实施；本轮不得commit或push。

受保护的已有启动命令：

```bash
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh \
  --profile hand_tracking_sim --viewer --disable-hands \
  --ik-backend pico_ee_dexhand_qp \
  --arm-target-processor passthrough \
  --joint-trajectory passthrough \
  --command-step-clipping false --joint-limit-source urdf \
  --pico-overlay --ik-target-overlay
```

## 2. 迁移策略

### 2.1 硬性要求：原版算法与数据链路语义一致

`vr_manus` 的原版复现路径是五工程联合路线：PICO/手柄经PICO_tracker产生原版修正骨架，TJ_arm_control指定分支运行完整SPARK算法；Manus按原版语义转换骨架，wuji_teleop对应输入/配置与官方wuji-retargeting驱动二代手。直接XR位姿增量控制是独立可选路径，不计作这条路线的等价实现。

一致性是交付条件，不是“尽量接近”。未解决的算法、输入语义、时序或状态差异均意味着原版迁移未完成；只记录差异不能通过验收。替换启动器、ROS2/UDP/Zenoh传输和执行接入时也必须证明接口语义等价。

必须保持：

1. 采集字段、左右手/设备身份、骨架节点顺序、单位、有效位、时间戳、序号、epoch及缺失值处理。
2. 原版坐标转换、标定产物、骨长重定向、掌心/TCP定义、旋转顺序及转换执行次数。
3. SPARK两阶段求解、前馈估计、Headroom、Velocity QP、内部滤波/连续性/hold、参数、运动学模型和控制状态源。
4. Manus25→21转换及官方二代手retarget版本、配置、资产、初态和关节命名/排列。
5. 输入消费策略、队列/丢帧规则、源采样与控制周期关系、dt计算、参考建立、按键触发、追踪失效、恢复及reset顺序。

允许消息重新封装、显式发布者身份和可逆坐标表示转换；不允许隐式插值、重新采样、丢弃参考会消费的中间帧、重复处理同一采样、静默滤波/裁剪、重复TCP补偿或骨长缩放。各边界恢复到同一规范表示后比较。新增元数据不要求原始封包字节相同，但原始包必须保留用于审计。

`reference_direct` 是五工程复现配置：关闭新增外层整形、关节Ruckig和逐步裁剪，保留原版内部处理。不得以删除原版内部OTG/连续性/hold来实现所谓直出。`processed_guarded`、直接XR映射及新操作绑定均单独标识，不能把其结果用于证明原版一致。

**行为优先级：** 原版已有的输入失效、按键、hold/reset、单侧失败处理以固定参考提交的实际调用链和trace为准。下文通用保持策略、双侧拒绝策略不能未经对照覆盖这些行为；凡与参考不一致，只能用于独立处理模式。现有工程授权和硬安全保护继续有效；一旦介入，标记为集成中断并暂停原版等价性比较，不得移除保护，也不得把中断后的不同轨迹统计为复现通过。真机保护作为单独部署契约验收。

### 2.2 分阶段接入策略

采用“适配现有采集输出，再逐步内置必要模块”。

整套 ROS2 工作区搬入会重复引入启动、控制和依赖管理；立即重写全部驱动难以证明与原版一致。优先复用稳定的 PICO TCP、Manus Zenoh、TJVR UDP 接口。XRoboToolkit 通过独立采集进程或 ROS2 边界桥接，主工程不隐式依赖参考目录中的 install/setup.bash。

| 来源 | 迁移/复用范围 | 边界 |
|---|---|---|
| `pico-manus-teleop/manus` | NODE 语义、raw采集、坐标约定、25→21 | 不同时消费 raw 和 /hand_input 形成双重转换 |
| `pico-manus-teleop/PICO_tracker` | 驱动、TCP/pivot/臂长标定、修正骨架、TJVR | 初期外部采集，读取明确指定的标定产物 |
| `pico-manus-teleop/wuji_teleop/.../pico_input` | XRoboToolkit 数据、腕/上臂 Tracker、按键、增量逻辑 | 不启动参考机器人执行器 |
| `pico-manus-teleop/wuji-retargeting` | 二代手 Retargeter/YAML/模型 | 固定版本、许可证及资产来源 |
| `pico-manus-teleop/wuji_teleop` | 二代手 SDK 集成与设备就绪检查经验 | 保持本工程唯一命令发布权 |
| `pico-manus-teleop/TJ_arm_control` 的 `feature/pico-manus-teleop` | SPARK Headroom Feedforward Velocity QP 完整机械臂算法链、参数、状态推进和运动学契约 | 新增独立IK后端；不替换PICO2 v131，不移植参考Viewer执行器 |

参考源码和依赖版本、标定/模型/配置摘要记录在迁移清单中；生产运行不依赖任何外部工程绝对路径。已有旧集成保持原样，新输入进程单独管理生命周期，禁止移植扫描并杀死其他目录历史进程的启动逻辑。

## 3. 数据流与配置

```text
pico2_hands                         vr_manus
  PICO2 TCP                          Manus + TJVR 或 XRoboToolkit
       |                                         |
       +------ 手骨架 / 位姿 / 操作事件 ----------+
                          |
          +---------------+-------------------+
          |               |                   |
       统一21点         位姿映射           操作请求
          |               |                   |
    二代手retarget       目标整形          会话状态机
          |               |
    20关节命令           IK → 关节轨迹 → coordinator
          |                                   |
      手执行器                           机械臂执行器
          +-------------- sim / real ----------+
```

新建四个 profile：`pico2_hands_sim`、`vr_manus_sim`、`pico2_hands_real`、`vr_manus_real`。底层配置区分 input_mode 与 execution，但对外沿用 `--profile`，不引入两套互相覆盖的启动入口。

拟议的输入组合：

```yaml
input_mode: pico2_hands
hand_input: pico2
arm_input: pico2_head_wrist
operator_input: gesture
```

```yaml
input_mode: vr_manus
hand_input: manus
arm_input: tjvr_corrected_palm   # 或 xr_controller / xr_tracker
operator_input: controller
```

以上字段是待实现的输入配置；execution 由 session capability 确定。键盘保持为可配置的人工操作入口。模式注册表只允许显式列出的组合：非法来源/mapper 组合在启动时拒绝，不用默认值猜测。

每个 session 固化：输入模式、左右设备绑定、mapper、标定 ID、retarget 后端及配置摘要、IK 后端、轨迹策略、模型、执行能力、有效侧和发布者身份。

## 4. 三个通道的契约

### 4.1 手骨架

沿用现有手观察模型和统一21点顺序。字段必须明确 source、side、frame、单位、序号、源时间戳、接收单调时间、有效性、连接代次。MediaPipe 顺序只统一关节语义，不意味着两种设备的坐标方向和尺度天然一致。

- Manus 按 NODE/chainType/fingerJointType 提取，省略四个非拇指 metacarpal；Y 转换只做一次。
- PICO 保留原始26点，使用现有映射产生21点；保留原始掌心与腕部位置用于调试。
- 缺失或重复语义拒绝该侧骨架；不补零伪造有效帧。
- 手部局部几何与机械臂全局位姿分离，不能用腕相对21点反推出机器人目标位置。

### 4.2 机械臂观察

扩展现有 `ArmInputObservation` 的兼容版本，保留 tracked_frame、reference_frame、pose、可选上肢骨架/肘部方向、各自有效位及 calibration_id、tracking_epoch。

- PICO2：同一采样内计算 `T_head_wrist = inverse(T_world_head) * T_world_wrist`。
- TJVR：分别保留 packet target 和 corrected skeleton palm，不把两者命名为同一个 target；保留冗余方向与有效位。
- XR：区分 controller 与 wrist_tracker，并按设备 ID 绑定左右手；controller→palm/TCP 外参单独标定。
- 跨设备数据用各自接收时间判断新鲜度；源时钟未对齐时不直接相减判定同步。

### 4.3 操作观察与事件

新增 `OperatorObservation`（设备状态）和 `OperatorEvent`（去抖后的动作），字段包括 source、side、sequence、epoch、receive_time、valid、action、edge、confidence/available。

统一动作：`start_request`、`pause_request`、`home_request`、`calibrate_request`、`clutch_press`、`clutch_release`。必须映射到现有状态机允许的转换；现有键盘 s 若包含回 Home 语义，不能把同一处理函数直接当作“保持暂停”。

- TJVR v4 bit8 是按键状态/事件标志，须与参考发送逻辑核对，再进行上升沿和序号去重；不能声称它包含全部扳机和摇杆。
- 完整按键/离合需由 XR 原始输入或独立伴随事件流提供；缺失的能力在 profile 中禁用并显示原因。
- PICO 当前 v1 骨架协议没有显式手势 ID。第一版基于21点增加独立识别器；后续原生手势通过新版本/伴随消息适配，保留 v1。
- 先提供 open/pinch/fist 观察与记录。操作绑定默认关闭，通过明确配置启用；普通抓握不能默认触发停止或开始。
- 可配置基线为 armed 状态双手稳定张开0.8秒触发一次 start，必须先释放再触发；阈值和绑定作为新模式配置，不改变旧入口。
- 暂停/回位/标定可先使用键盘；不可靠的手势识别不能替代物理急停。

## 5. 位姿、标定及 IK

保留已有 mapper 工厂。新增 `xr_incremental` 和 `calibrated_palm_direct`；PICO2 沿用已有选项。

| mapper | 输入契约 | 标定/参考 |
|---|---|---|
| 原有 relative_home/head_direct/head_palm_direct | PICO2 头相对腕/掌心 | 保留原行为，包括仅Z高度标定 |
| xr_incremental | 明确的 XR tracking frame | 按开始建立参考；离合保持并在恢复时重建参考 |
| calibrated_palm_direct | TJVR corrected palm 或明确选择的 packet target | 使用独立 tracking→base 和 palm→TCP 外参 |

直接映射遵循 `T_base_tcp = T_base_reference * T_reference_tracked * T_tracked_tcp`。每条路径必须声明唯一骨长重定向责任层 `retarget_owner: upstream | mapper | ik | none`：SPARK修正骨架路径固定为 `ik`，外层只执行规定的坐标/外参转换；消费已经重定向的packet target时为 `upstream`，不得再重定向。PICO2原有位姿增益与此骨长重定向是不同配置，保持原语义。重复开启、输入语义与责任层不符均拒绝启动。

增量旋转乘法顺序、左右手基变换、世界/局部增量必须与参考代码的回放结果逐帧对齐，不能仅照 README 公式重写。先在 observation/sim 中验证前后左右上下、旋转、双手合拢和同一人体整体平移/转向。

标定产物包含设备/操作员 ID、侧、参考帧、TCP定义、外参、单位、版本及摘要。设备重绑定或 epoch 变化使关联参考失效；不扫描目录自动挑选历史标定。

IK继续可插拔；目标整形、关节轨迹和逐步裁剪保持独立。动态肘部方向仅传给声明支持的 IK；不支持时显式诊断，不修改 v131 以伪装支持。

新 real profile 显式绑定算法及设备限制配置：PICO2目标路线为v131，VR+Manus的TJVR修正骨架目标路线为SPARK。两者均在独立真机验收通过前列为不支持组合，不自动回退到其他IK。既有真机IK可用于单独标识的执行链测试，但不计作指定算法的真机迁移完成。

### 5.1 VR + Manus 的参考 IK：必须独立迁移

参考仓库：`pico-manus-teleop/TJ_arm_control`（实际目录由`PICO_MANUS_TELEOP_ROOT`在运行时注入）。
已核实当前分支为 `feature/pico-manus-teleop`，工作区干净，提交为
`c022b17789e9b81141915662b3bb802f4c20a396`。参考README仍有main措辞，迁移以该分支提交及源码为准。

主算法及拟议新增的 `--ik-backend` 名称均为：

```text
spark_upper_qpoases_headroom_feedforward_velocity_qp
```

必须迁移完整链路：

```text
TJVR v4 corrected 肩/肘/腕/掌心骨架及有效状态
  → SPARK机器人骨长重定向与两阶段qpOASES IK
  → 关节参考速度和掌心笛卡尔前馈
  → Headroom位置/速度/加速度/jerk余量调节
  → Velocity QP及原版动态/上臂外侧/任务约束
  → 输出连续性、stationary-reference hold、settled-hold
  → model_reference状态推进
  → 当前工程producer/coordinator/执行器
```

参考默认配置是 `config/qp_ik_pico_teleop.yaml`，模型是 `models/marvin_m6_wuji2.xml`，velocity控制周期200Hz，状态源为model_reference。这些是参考仿真契约，不代表真机已验证。

输入接口不能只留下TCP pose或单一elbow direction。新增版本化的上肢目标扩展，保留左右肩、肘、腕、掌心位置/旋转、有效位、源序号、epoch和标定信息，整体传到producer；末端目标与同一帧骨架必须可关联。旧IK仍消费旧目标，新后端声明 `requires_upper_limb_skeleton` 能力。

`vr_manus` 的 `tjvr_corrected_palm` 子模式默认选该新后端。仅有手柄6D位姿的XR子模式不满足输入契约，启动时拒绝该组合；需要完整上肢重建才能使用，不能补零或用固定肘部方向伪造原版输入。XR位姿子模式仍可选择支持pose-only的既有IK。

SPARK算法内的人体骨架到机器人骨长重定向与外层mapper须明确边界：此后端接收规定坐标系中的修正人体骨架，由算法完成原版构型重定向；外层只做规定的坐标/外参转换，不能提前重复骨长缩放。packet target保留作诊断，是否参与原版前馈必须按参考调用链迁移。

源文件审计至少覆盖：`spark_upper_retarget.cpp`、`spark_upper_qpoases_ik.cpp`、`spark_guidance.cpp`、`spark_feedforward_reference.cpp`、`spark_palm_twist_estimator.cpp`、`spark_constraint_headroom.cpp`、`spark_posture_reference.cpp`、`hierarchical_qp_ik.cpp`、`hierarchical_qp.cpp`、qpOASES封装及其头文件。还必须检查 `apps/run_qp_ik_viewer.cpp` 的反馈、reset和循环顺序；不能只复制solver类而遗漏Viewer中实际参与控制的逻辑。

新模块采用独立命名空间/编译目标，拟置于 `include/tianji_spark/` 和 `src/ik/spark_headroom/`，通过现有 `arm_ik_factory.cpp` 注册。原版config、模型、TCP定义、依赖和状态重置规则记入 `src/ik/spark_headroom/PORTING.md`。若为运动学复现保留参考模型，将其隔离为算法资产，不覆盖当前可视化模型。

外层目标整形/关节Ruckig/逐步裁剪继续可选；“外层passthrough”不关闭SPARK内部前馈调节、连续性和hold。producer诊断须明确算法名、状态源、骨架帧序号、两阶段状态、Headroom、前馈、QP结果和hold原因。

该参考工程明确只驱动MuJoCo，因此新后端先标simulation-only。真机迁移另验model_reference与实测反馈偏差、约束、延迟和恢复机制；不能借由换成既有真机IK就声称参考SPARK算法已经完成真机验收。

### 5.2 SPARK双臂控制周期及接口

现有单侧 `ArmIkSolver::solve/solve_timed` 不能直接表达完整SPARK控制循环。新增 `include/tianji_teleop/ik/bilateral_ik_backend.hpp` 和双臂工厂入口，原单臂接口与默认调用方式保留。producer根据后端能力选择整帧双臂调用，禁止通过两次单侧调用重复推进同一个SPARK guidance实例。

拟议接口类型与责任：

- `BilateralUpperLimbTarget`：同一TJVR帧的左右骨架、掌心、有效位、坐标定义、epoch、序号和源/接收时间；不拼接不同帧的左右目标。
- `BilateralReferenceState`：同一控制周期的双臂q/qdot/qddot与内部参考时间。
- `BilateralTickContext`：唯一tick_id、dt、输入frame_id、当前session/epoch及运行模式。
- `BilateralStepResult`：双臂结果及有效位、共享tick_id、原版控制器诊断和内部反馈；accepted与converged分别保留。
- `step(target, context)`：每tick调用一次；目标序号未更新时按原版目标保持/新鲜度规则处理，不再次作为新采样输入前馈估计器。
- `reset(state, reason)`：明确区分输入epoch更新、原版控制重置和部署保护重置。输入epoch改变按参考Viewer保留model_reference/OTG连续性并执行guidance/接管逻辑，不统一从实测状态重置全部控制器；仅对应原版或部署保护的原因执行完整reset。旧epoch异步回执不得改变新状态。
- `observe_execution(report)`：接收按run/epoch/tick关联的下游处置，作用由5.3节模式明确；不把异步执行反馈替换为原版同周期QP反馈。

SPARK后端拥有双臂内部参考状态，由一个200Hz调度器推进。每tick按参考Viewer调用顺序完成输入接纳、guidance/前馈、控制器step、结果诊断和 `updateHeadroomFeedback(left, right, dt)`；Headroom反馈每tick最多一次，由该次控制器结果构造，在控制器step之后更新。reset、丢失及异常分支也按参考trace核实顺序。

双臂结果可以沿用当前分侧proposal传输，但必须携带相同tick_id；新模式下coordinator只接纳完整配对，不把左右不同tick当作一个控制步。一侧拒绝/缺失时双侧按该模式的保持策略处理。记录“成对接纳”与硬件实际发送/反馈的区别，不声称两台设备网络写入具备事务原子性。

首版SPARK要求完整双侧输入。单臂真机验收采用双侧输入、双侧内部求解、单侧设备输出的显式诊断配置，另一侧保持仿真/影子状态；不能伪造无效侧骨架。未来仅单侧输入支持须有独立参考行为和测试后才能声明。

### 5.3 直通复现与额外处理模式的状态规则

新增配置 `reference_execution_mode: reference_direct | processed_guarded`，仅用于新SPARK入口，不改变现有v131路径。

| 规则 | reference_direct | processed_guarded |
|---|---|---|
| 用途 | 原版算法一致性复现基线 | 用户显式选择额外平滑/裁剪 |
| 外层目标整形/关节轨迹 | passthrough | 可配置 |
| 逐步命令裁剪 | 关闭；硬拒绝仍保留 | 可配置 |
| 内部状态推进 | 保留原版model_reference | 仍为model_reference，不按每帧下游命令覆盖 |
| 原版Headroom反馈 | 本周期QP结果 | 本周期QP结果，与执行误差监督分离 |
| 正常异步回执 | 仅确认关联命令处置 | 记录raw/processed/accepted/actual差异 |
| 下游拒绝/回执超时 | 暂停双臂并冻结后续推进 | 暂停双臂并冻结后续推进 |
| 正常处理造成差异 | 不允许静默改写命令 | 在显式误差/时间阈值内允许；超阈值暂停 |

两种模式均不在实时求解线程阻塞等待网络回执。维护有界在途tick队列，以配置的最大回执年龄和执行偏差监督；接收迟到/乱序/旧epoch回执不回滚状态。暂停时丢弃未执行目标，恢复前从可信的sim状态或真机实测反馈联合reset，并重新授权。单纯“已接纳命令”不等于实测位置；反馈不可信时继续保持暂停。

`processed_guarded` 必须显式配置每侧参考到命令、参考到实测位置偏差上限、最大回执年龄和恢复条件；缺失时拒绝启用。它是增加监督的集成模式，不声明与原版逐帧等价，不通过每tick reset来伪装同步或破坏前馈历史。

原版一致性测试在 `reference_direct`、同初态/时间步/目标流下进行，包括原版已有的输入丢失/恢复、按键、epoch和hold；仅新增下游硬保护中断单独作为集成契约测试。处理模式单独验证平滑输出、偏差门槛、配对拒绝和恢复。两种模式均记录原始参考、处理后命令、接受命令、实际反馈和reset原因。

## 6. 统一二代手 retarget 与执行权

新建 retarget 后端接口 `retarget(observation) -> HandJointTarget`，输出必须包含侧、20个关节名称、rad单位、时间、有效性与来源。官方 YAML/模型版本固定，不能按数组位置猜测左右关节。

`HandJointTarget` 是计划新增于 `retargeting/base.py` 的内部结果类型，不是现有协议消息。producer 将它封装为当前 `HandJointCommand`，填入当前 session 的 envelope、发布者身份和时序字段；算法本身不获得 Zenoh 发布权。无效观察返回无效结果且不推进算法状态，producer按会话策略保持/暂停。

新 profile 首选 `wuji_official_yaml`，复用参考 `Retargeter.from_yaml`。`wuji_sdk` 作为可选后端，需要单独对齐；`legacy_dry_geometry` 仅用于旧入口/明确测试，不自动降级。

原版复现还必须保留 `tj_wuji2_hand_bridge.py` 包装层：按URDF限位裁剪、按实际关节名重排、各侧序号间断时reset滤波。用户补充远端实际联调使用 `adaptive_analytical_manus_wuji_hand_2_{left,right}.yaml`，已从192.168.110.210取回并固定SHA256；此实际配置优先于桥代码默认的wuji_glove命名YAML。启动必须显式指定左右配置，不允许静默回退默认值，也不能把参考内部限位裁剪误当成新增外层裁剪删除。

新增手 producer，将官方算法与硬件连接解耦；sim 和 real 消费同一个 producer 输出。与现有 launcher 的 `producer_hand`/`executor_hand` 身份绑定一致，同侧只有一个关节命令发布者。旧复合 executor 的 retarget 模式保留，新 profile 使用独立 producer + direct executor，禁止重复 retarget。

MuJoCo 与真实手仅在执行、反馈和执行限制上不同；验证的“同输入同命令”指执行保护之前的目标，不能要求不同动力学下实测关节完全相同。

## 7. 状态、追踪丢失与真机

本节表格描述通用/部署保护策略。`reference_direct` 的输入epoch、丢失、按键和恢复严格采用2.1节及参考契约，不套用通用“epoch改变即全面reset/重授权”。新增硬保护介入须单独标记；旧PICO2状态行为保持不变。

| 事件 | 新模式行为 |
|---|---|
| 数据未就绪 | 保持 armed/未授权，不发运动请求 |
| 单侧骨架失效 | 该侧手保持；手控制为 required 时超时进入会话暂停 |
| 腕部短暂识别丢失 | 保持最后已接受的关节命令，不继续积分追赶旧目标 |
| 同epoch恢复，sim | 保留原映射参考，追向恢复目标；匹配用户既定行为 |
| real恢复 | 经过有效性、反馈偏差和执行限制检查；超过恢复阈值保持并要求重新授权 |
| epoch改变/重定位/重新连接 | 清除操作边沿、参考和待处理目标，重新就绪及授权 |
| IK失败/反馈超时/硬件故障 | 使用会话和执行器故障路径，不归类为普通视觉丢失 |
| 离合 | 保持输出；释放后按当前参考重建规则恢复，避免跳变 |

上述新增 real 策略不覆盖旧 PICO sim 的丢失恢复行为。最大保持时间、输入新鲜度、反馈偏差和恢复限速属于各执行 profile 的必填参数，不直接沿用未验收的仿真数值。

真机复用现有 `--confirm-real`、HostReadiness、preflight、coordinator 和设备 watchdog。新增 profile 必须检查机器人/手型号、序列号、固件、活动侧、关节反馈、限位以及唯一控制权。没有实际验收记录不能标记 real-ready。

## 8. 记录、回放与可视化

保留已有 `raw/pico_hand_tracking` 完整数据，不更改其历史含义。扩展版本化 session schema，新增或复用已有等价组：Manus raw、TJVR raw、XR raw、operator observations/events、calibration、resolved configuration。

每个原始流记录源序号、源时钟、接收时间、epoch和有效位；记录操作接受/拒绝原因、原始目标、映射目标、IK诊断和执行反馈。录制原始流之间的关联，不假设频率相同或强制补成同步帧。

旧 H5 可继续读取；新 reader 对缺失可选组有明确行为。观察回放中的 start/按键默认不获得实时机器人控制权；必须走当前会话授权。

保留 PICO/IK overlay，增加可选 XR controller/Tracker、TJVR corrected palm、packet target、肘部参考和坐标轴。显示 input_mode、mapper、calibration_id、tracking状态与数据年龄，帮助识别重复坐标变换。

## 9. 文件布局与接口责任

以下新增文件为计划路径，已有文件采用增量修改：

| 模块 | 新增文件（相对 `src/tianji_teleop/tianji_teleop/`） | 修改边界 |
|---|---|---|
| 输入组合 | `hand_tracking/input_modes.py` | config_loader、run_session.sh、run_source.sh |
| 操作事件 | `hand_tracking/operator_input.py`、`hand_tracking/gesture_recognition.py` | protocol/messages.py、topics.py、target_node.py |
| XR适配 | `hand_tracking/xr_input.py` | observation_node.py、runtime.py |
| 标定 | `hand_tracking/calibration_store.py` | sources/common/pose_mapping.py |
| 官方手后端 | `retargeting/base.py`、`retargeting/factory.py`、`retargeting/wuji_official.py` | 新 `producers/hand_retarget.py` |
| 真机接入 | 新session/source/executor配置 | config_loader、target_node、launcher、现有执行器 |
| 记录 | 复用已有recording目录 | recorder.py、session_h5.py、session_recorder.py |

协议扩展采用版本/可选字段策略，同步核对 Python 和 C++ 消费端；不能给 strict parser 直接添加无法识别的必填字段。若 target_node 持续增长，仅抽出本任务涉及的操作和配置逻辑，不做全工程重构。

当前 `hand_tracking/models.py` 和 `protocol/messages.py` 各有 `ArmInputObservation`：前者是内部模型，后者是线上消息。扩展时同时更新序列化/反序列化适配并测试往返，避免只改内部类型导致动态肘部方向在发布时丢失。

## 10. 顺序实施计划

每项先补有意义的失败用例，再实现最小闭环并回归；阶段结束形成可独立审查的差异。未获得提交指令时保留工作区，不自动 commit。

### 当前执行记录

- 9月9日设备验收准备：补齐Manus SDK子进程专用库路径及文件摘要，支持`--manus-library-dir`；新增PICO/TJVR/Manus仅接收活性探测器与`docs/real-input-simulation-acceptance.md`操作单。802项全量通过、无跳过（305.514秒），包括此前reset超大整数修正；原生CTest 9/9，官方手1339帧最大误差0。按用户安排暂缓实物测试，未启动真实SDK采集或机器人；旧PICO默认行为不变，未暂存/提交/push。当前可进入外部PICO_tracker上游的真实输入→仿真测试准备阶段，不表示完整M6/M8、XR直接采集或真机阶段完成。
- 9月10日路径与XR收尾：清除源码、配置、脚本、文档及录制资产摘要中的主机 checkout 绝对路径；外部采集仓库、Manus rawviz/SDK、PICO/参考目录统一改为环境变量或 CLI 注入，新 HDF5 使用相对资产键并兼容历史绝对键。`vr_manus_xr_sim` 的 XR 采集、Tracker/控制器绑定、start/Home/clutch、Manus callback 及 `raw/xr_input`/`raw/manus_callbacks` 已接入；不改变已验证的`pico2_hands_sim`订阅和处理链路。真实 XR SDK/PC-Service 与设备验收仍待现场执行，未提交/push。
- 9月10日XR诊断增量：新增独立`--xr-overlay`，仅显示`raw/xr_input`中的HMD、左右控制器和四类Tracker，并在接收边界校验router/receiver/序号/新鲜度；不写qpos、不生成目标、不参与IK或手retarget，也不改变`--pico-overlay`及PICO2默认入口。XR SDK Python路径仍只注入采集子进程。参考XR闭包误改的两处可移植性修正已恢复为参考原文，保留来源哈希合同；修正后868项 Python 测试通过、51项按环境条件跳过，真实XR SDK/PC-Service与设备验收未执行。
- 9月10日XR重连安全增量：XR source 为每次成功连接分配连续`connection_generation`，序号在代次内从0开始；raw HDF5显式记录代次并兼容旧1.2帧，操作事件仅接受下一代次，运行中代次切换会让目标桥接拒绝旧参考并要求重新`start`，启动前切换可安全替换参考。PICO2输入、官方手和旧入口未改变。
- 9月10日XR SDK预检与释放增量：新增 `check_xr_sdk.py`，在 XR 会话获取 router/guard 前只检查 `xrobotoolkit_sdk` 必需可调用接口，不调用 `init`、不连接 PC-Service；缺模块或缺符号直接失败，并只向 XR 采集子进程注入 SDK 路径。XR 客户端关闭时调用可用的 SDK close hook，失败仍清理本地连接状态。14项 XR 输入/预检定向测试及脚本语法检查通过，PICO2 路线未触发该预检。
- 9月10日XR录制核验增量：新增 `recording/xr_check.py` 及 `check_dual_recording.py --mode xr`，逐帧核对 `raw/xr_input` 展平列与完整 `XrFrame` JSON、HMD/控制器/Tracker、连接代次和序号；异常闭环失败，不连接 SDK/router，不重放操作、IK 或 retarget。新增 XR 录制核验测试通过。
- 9月10日XR输入兼容与审计增量：采集边界兼容 Pybind 返回的 NumPy/可迭代 Tracker 容器，避免数组真值判断导致运行期异常；XR 录制核验增加 `source_time_ns/source_time_valid` 和 schema 1.2 完整 JSON 连接代次的逐列检查。定向回归及Shell语法检查通过，PICO2链路未触发这些逻辑，未提交/push。
- 9月10日XR操作审计与订阅修正：`vr_manus_xr_sim` recorder 保存完整 XR controller observation envelope 到 `meta/dual_audit/operator_observation`，离线 XR 核验检查路由/publisher、动作集合、代次/序号和 raw 代次关联；同时移除 XR 路径对 `raw/manus_callback` 的重复订阅，并修正 publisher 使用受管会话实际观察实例 ID，避免目标侧误拒绝合法控制器观察。定向录制回归及身份接线回归通过；最终873项 Python 测试通过、51项按环境条件跳过，原生CTest 9/9，PICO2录制布局和默认链路未改，未提交/push。
- 9月10日XR设备探针增量：新增 `probe_teleop_input.py --mode xr`，复用当前 XR binding 和采集边界，先检查 Pybind 必需 API，再只读连接 PC-Service 采样 HMD、双控制器及四个配置 Tracker；输出各信号新帧数/200ms新鲜度，绝不连接 router、启动 Manus、执行操作事件、IK 或机器人命令。补充非法 SDK 路径、真实 API 形状及 controller-only 绑定的测试；最终876项 Python 测试通过、51项按环境条件跳过，原生CTest 9/9，PICO2 流程及默认参数未改，未提交/push。该探针通过仍不等于标定、映射或真机验收通过。
- 9月10日XR+Manus目标层联调增量：离线联调发现参考 `HandInputAssembler.take_latest()` 产生的实际 callback 是扁平126-float，而旧边界只接受`(42,3)`；先加入真实扁平形状回归测试，再让 `manus_callback_observations()` 和 callback envelope 同时接受严格的 flat/row-shaped 21或42个XYZ点，保留非法形状拒绝。新增 `xr_manus_sim_smoke.py`，用合成XR帧和Manus callback经过canonical observation、显式controller start边沿、`xr_incremental`与目标桥，分别验证`xr_tracker`/`xr_controller`双臂双手目标输出及未start不出目标；该工具不启动SDK、rawviz、router、IK或机器人命令。最终882项Python测试通过、51项按环境条件跳过（49.878秒），原生CTest 9/9，静态/路径检查通过；PICO2链路未改，未提交/push，真实设备与真机验收仍未执行。
- 9月9日M6 reset审计增量：新增显式`--mode tjvr --check-native-resets`，在原包/门控/消费核验后被动检查成功rearm回执、14关节有限数值及零导数、执行代次和后续调用顺序。拒绝的rearm、其他run及缺失回执不能支撑代次推进；不执行reset，不验证真实Home/静止反馈或内部新鲜输入截止时刻。793项全量通过、无跳过（327.651秒）；后续超大整数范围修正经20项（含实际UDP跨Home/rearm）复测通过，21.729秒，未再跑全量；原生CTest 9/9。运行控制代码不变、未暂存/提交/push，完整IK状态回放仍未完成。
- 9月9日M6消费边界增量：仅扩展离线检查，利用已有`native_attempt.sample`核验SPARK实际记录消费的原包/接收者/时钟/重同步代数与门控对应，检查消费前门控审计、单次消费、tick/执行代次顺序。保留null输入tick，旧审计缺字段明确标未核验；不推断最新帧调度，不执行IK/reset/授权。786项全量通过、无跳过（320.998秒），原生CTest 9/9，官方手1339帧最大误差0；包含实际UDP跨Home/rearm及单左手/双手联合录制。M6/M8完整状态及算法回放仍未完成；未暂存/提交/push。
- 9月9日M6最新增量：新VR录制增加TJVR逐包门控审计及实际阈值/reset初态合同；`check_dual_recording.py --mode tjvr`按原接收顺序精确重建接受/拒绝、epoch变化和重同步代数。原门控算法/阈值、最新帧消费和旧PICO入口不变。781项全量通过、无跳过（291.049秒），原生CTest 9/9；实际UDP录制重建新增断言另随8项通过（16.739秒）。完整控制消费/映射/IK状态回放仍未实现，不将M6/M8整体标完成；未暂存/提交/push。
- 本轮最新（9月9日）：771项全量测试通过、无跳过，299.287秒；原生CTest 9/9通过，官方Hand2原版1339帧数值对照仍为0。覆盖双输入手命令数值重建、VR单左手修正及空手套绑定预检。新VR拒绝空绑定以保持启动与录制重建合同一致，不改变原版有效ID绑定规则。下方为递进历史快照；未暂存/提交/push，整方案仍未完成，不据此勾选需要真实设备或完整状态重放的阶段。
- 9月9日顺序自查：767项固定快照全量通过，9项原生CTest通过。另修复新VR单左手配置仍使用右手worker解释63维数据的问题，同步记录实际启用侧别；真实单左手/双手worker、SPARK仿真及录制重建两项通过，未改PICO旧配置或双手默认。最新全量结果另记。
- 9月9日PICO手重建增量：新增可选已处理输入hook，仅新PICO受管手组件启用被动消费审计，recorder保存消费起点/序号/关联和官方手/PICO适配资产摘要。离线重建只推进实际消费帧；实际583帧输入、625条手命令数值差值0，已接入新PICO自动smoke。此前759项固定快照全量通过，新增内容的最新全量结果见进度文档；整体M6/M8仍有完整映射/状态/授权回放缺口。
- 9月9日M6后续：新增显式Manus手命令离线重建，要求rawviz/回调及已记录资产一致，保留全部idle回调推进滤波，不执行授权/设备。实际官方worker与SPARK联合录制重建通过。PICO手命令仍需实际消费序号记录，不能将订阅前raw当作已处理输入。此前747项固定快照全量通过；最新增量结果单列于进度文档。
- 9月9日后续：新1.2录制补PICO raw原始接收时钟（旧1.1不变，旧1.2可读）；离线核验扩展被动手势重建。新增`compare_dual_input_reference.py`汇总既有SPARK/官方手轨迹比较，只有已提供阶段通过才返回成功，不声称同步或完整验收。740项快照的1项手势smoke失败定位为测试握拳脉冲未送达，改为接收观测确认；生产逻辑不变，同router重验通过。详见进度文档。
- 9月9日M6增量：新增PICO原包→21点/头相对腕姿及Manus rawviz→原版逐POSE回调的纯离线一致性核验入口。新VR录制保存解析绑定；缺少配置不猜测，事件不授权。13项定向测试及实际官方手/SPARK联合录制重建测试通过；PICO合成仿真录制586帧/2344条观测差值0。尚未覆盖完整映射/retarget/IK状态重建，不代表M6/M8完成，详见进度文档。
- 9月9日后续：新PICO显式双手张开START及结果/组件状态审计已接通；修复新PICO同步录制争锁漏记事件，改为有界FIFO和仅新入口启用的有界行批写。726项固定快照全量通过；VR→PICO→VR同router清理通过，VR段未授权运动/未带手套，不替代完整设备切换验收。VR预检补flat MJCF实际网格文件摘要。其后只读解析器明确将未实现的VR完整controller操作绑定标为runtime_available=false。详见进度文档；XR实际采集、processed_guarded/可信恢复、完整录制回放及真机阶段仍不算完成。
- 9月9日续：两种新sim受管入口已接通，PICO官方二代手与原v131双臂、VR原SPARK与官方Manus手均有本机合成输入联调。完整回归686项通过（后续手势观察新增前），同router停止PICO→启动VR双臂的权限清理验证通过。新PICO只读几何手势观察及schema1.2审计已接通，动作绑定仍关闭；这些是后续增量进展，不表示下面整阶段复选项全部完成。

- Review：已修正epoch全量reset与原版model_reference连续性冲突；明确最新帧消费，补齐官方手桥的限位/重排/filter reset。
- M0进行中：已生成 `docs/dual_input_migration_manifest.md` 和 `docs/reference_equivalence_contract.md`；Manus目录无Git元数据，使用文件摘要。已从远端只读取回正式标定、Manus回放、实际retarget配置与官方手URDF；另从旧main克隆取回两份TJVR（2444/27054帧），容器及逐包CRC校验通过。TJVR与Manus不是已同步的联合录制，完整依赖闭包与算法等价性基准尚未验收。
- M1进行中：已实现纯输入组合解析 `hand_tracking/input_modes.py`，12项测试通过；与现有30项配置/启动器测试合计42项通过。此模块只验证输入契约，不授权执行；新profile、采集进程排他启动及resolved runtime接线尚未完成。
- 基线验证：443项发现集（1项跳过）通过，既有IK仿真构建/probe通过；不等于SPARK迁移测试通过。
- 本轮回归：普通发现集481项，4项跳过，OK；其中3项可选C++对照另以参考源码和真实录制显式启用后全部通过。未改动既有受版本控制的运行代码/配置，未提交。
- M2部分实现：新增隔离 `reference_tjvr.py` 和 `reference_tjvr_stream.py`，保留旧TJVR入口及schema。两份真实录制29498帧与原始C++协议/stream gate逐帧对照通过，并补齐协议异常、按键状态、epoch/乱序/三帧恢复合成测试。已新增raw保真封装和receiver持久化重同步代数，未接入现有运行时。
- M2b部分实现：SPARK原生依赖闭包、独立锁定环境、双臂周期接口、离线worker与有界进程客户端已新增。直接编译原版源码、保留原版Viewer控制块的独立oracle，对两份真实录制共65726个周期比较，所输出字段最大差值0；另通过按键/epoch/丢失恢复合成对照。9组CTest通过。详见 `docs/spark-porting.md`；尚缺工厂/producer/coordinator和profile接线、实时及真机验收，不能标记M2b整体完成。
- M5部分实现：官方retarget及bridge、实际Manus二代手配置和URDF已按摘要迁移，独立Pinocchio4/NLopt2.10.1环境已安装。1339帧Manus21点回放与远端原版40关节结果误差0。新增无设备输出的手部worker及保留float32/非wrist-relative语义的原版Manus输入边界。详见 `docs/wuji-hand-porting.md`；尚未接入手部producer及执行授权，不标记M5整体完成。
- 后续接入进展：新增SPARK双臂producer/工厂、配对协议、coordinator成对处置回执及reference_direct授权/超时锁存；新增分侧URDF限位配置与MuJoCo注入，原版短录制经完整coordinator→MuJoCo路径逐项保持原生q。新增官方手部持久进程客户端及会话授权producer核心，不重复retarget。全量发现集552项，546通过、6跳过；没有提交。
- M2及以后尚未完成。producer核心已有，但新profile/实时采集接线、processed_guarded、XR、手势、完整session HDF5及真机验收仍未完成；不以离线链路通过替代完整方案完成。
- 后续离线联调：新增`vr_manus_sim_smoke.py`，完整短TJVR经原生SPARK→协调器→MuJoCo输出逐项一致，独立Manus1339回调产生2678条手命令，MuJoCo手qpos逐项一致。新增UDP接收层的本机测试、HDF5 1.2的TJVR原包/Manus回调记录、被动recorder新raw入口，旧1.0/1.1回归通过。最后显式启用全部可选环境的569项发现集全通过、无跳过，9组CTest通过；旧受保护PICO2命令smoke通过。详见`docs/dual-input-integration-progress.md`。新live profile仍未接通。
- 2026-09-09续作：上一轮完整580项测试通过。新增原子最终双臂命令已接离线CLI；两种新profile支持`--resolve-only`只读严格解析。新增共享`SparkCoordinatorCycle`、实时双臂核心、rawviz独立接收、官方手独立处理线程与显式授权MuJoCo派生类；SPARK/手worker可选启动握手不推进算法状态。定向用例已通过，当前全量回归运行中。完整live launcher、双手联合实时接线、reset/processed_guarded、XR/手势及真机验收仍未完成；没有提交代码。

### M0：冻结参考与回归基线

续作记录（2026-09-09，仍未提交）：VR+Manus受管实时仿真已接通，原生与官方手worker隔离运行；617项完整发现集、原生9组CTest及旧PICO2受保护smoke通过。新增健康Home联合重置和`h→r→新输入→s`，不清除硬故障、不以旧缓存重新授权。其后新增live `--record`、有界磁盘队列、HDF5可选`dual_audit`、原始rawviz/实际回调/命令反馈记录及独立`--spark-overlay`；定向测试通过，新增变更需继续全量回归。新PICO2 profile、任意位置恢复/processed_guarded、XR/手势及真机验收仍未完成。详细边界和命令见`docs/dual-input-integration-progress.md`。

- [ ] 记录五个参考目录（含TJ_arm_control指定分支）的 commit、dirty状态、算法配置、模型摘要和许可证；区分 wuji-retargeting 顶层与 wuji_teleop 内嵌副本。
- [ ] 新增 `docs/dual_input_migration_manifest.md`，明确每个模块选择哪个来源。
- [ ] 用现有测试、IK构建和受保护参数smoke记录当前结果；保留 PICO2、Manus、TJVR、XR 确定性测试帧。
- [ ] 参考数据未包含真实设备样本时明确标 synthetic；不以此替代硬件验收。
- [ ] 新建 `docs/reference_equivalence_contract.md`，固化五工程实际启动参数、配置/标定/模型摘要、输入消费规则、控制周期、操作状态转换及逐阶段比较点；未核实的契约不进入后续对应模块实现。

交付：可追溯基线与参考输入，不改变运行行为。

### M1：输入模式与启动排他性

- [ ] 新增 `input_modes.py` 及 `tests/test_input_modes.py`：覆盖合法组合、双采集误启动、非法mapper、required侧、旧profile兼容。
- [ ] 新建两种sim profile；修改 `config_loader.py`、`scripts/run_session.sh`、`scripts/run_source.sh` 解析组合和发布者身份。
- [ ] 增加 resolved configuration 输出，包含执行能力和实际加载参数；旧profile解析结果保持一致。
- [ ] 运行对应输入配置测试和 `tests.test_task8_config_launcher`。

交付：能明确选择模式且仅启动对应采集器。

### M2：TJVR + Manus 仿真对齐

- [ ] 扩展 `legacy_pico.py`、`models.py` 和协议：读取方向、按键及其有效位，分别保留目标和修正掌心。
- [ ] 在 `tests/test_legacy_pico_palm.py` 增加v4目标/骨架不同、CRC、方向有效位、乱序/epoch变化用例。
- [ ] 在 `pose_mapping.py` 增加 `calibrated_palm_direct` 和明确的目标选择；新增 `tests/test_calibrated_palm_mapping.py` 检查坐标/TCP只转换一次。
- [ ] 对参考Manus输入逐帧比较21点；保留不完整语义拒绝行为。
- [ ] 新增 `scripts/vr_manus_sim_smoke.py`，验证两路异步采样、失效侧、双臂运动和记录。

交付：参考 corrected-palm 路线在当前仿真中可复现；仅接收bit8不宣称完整手柄支持。

### M2b：SPARK Headroom完整算法迁移（M2后、M3前执行）

- [ ] 固定5.1节参考提交，追踪Viewer到SPARK guidance、QP、状态推进的调用顺序，记录配置所有实际生效参数及依赖；不修改参考仓库。
- [ ] 扩展内部/线上目标及C++解析，新增 `tests/test_upper_limb_target_protocol.py`：整帧关联、方向/旋转有效位、epoch、缺失骨架拒绝、旧目标兼容。
- [ ] 新增 `src/tianji_teleop/include/tianji_spark/`、`src/tianji_teleop/src/ik/spark_headroom/`，迁移算法依赖闭包；修改CMake及工厂注册，复用依赖前核实版本和符号隔离。
- [ ] 新增5.2节双臂后端接口、工厂入口与producer分派；新建 `tests/test_spark_bilateral_contract.py` 和C++周期probe，验证同帧配对、每tick仅一次guidance/Headroom更新、联合reset、旧epoch拒绝及旧单臂接口回归。
- [ ] 扩展 `arm_command_coordinator.py` 的新模式配对接纳及有界tick回执，保留旧profile行为；一侧失败时验证双侧保持，不把网络发送当作设备实际反馈。
- [ ] 新建 `config/producers/ik_spark_headroom.yaml`，绑定 `vr_manus_sim` 的TJVR子模式；新后端不改变旧profile默认值，不映射到现有pico_ee_dexhand_qp名称。
- [ ] 默认绑定 `reference_direct`，实现5.3节可选 `processed_guarded`；新增 `tests/test_spark_execution_state.py`，覆盖Ruckig/裁剪、拒绝、延迟、乱序、实测偏差、联合暂停/reset和重新授权。没有明确阈值的处理模式必须拒绝启动。
- [ ] 在映射测试加入 `retarget_owner` 校验：SPARK使用原始修正骨架，packet target不得再次缩放；位姿/TCP变换与骨长重定向分别验证。
- [ ] 新增C++ `spark_headroom_probe` 和参考trace程序；同骨架序列、初态、模型、参数、时间步比较两阶段结果、前馈、Headroom、QP、q/qdot及hold/reset；测试静止、六轴运动、限位、不可达、丢失、恢复和epoch变化。
- [ ] 新建 `scripts/compare_spark_reference.py` 和 `tests/test_spark_backend_selection.py`；确定性无预算超时场景以q差1e-5rad为初始迁移门槛，状态必须一致。真实计时预算单独测量，不能用放宽数值容差掩盖调用链差异。
- [ ] 扩展VR+Manus smoke：确认producer报告完整算法名、使用上肢骨架，双臂响应并输出相关诊断；再次运行受保护PICO2 v131 smoke。
- [ ] 完成PORTING及仿真一致性报告，明确真机尚未验收。

交付：独立可选的SPARK完整后端与参考仿真一致性证据，而不是仅实现两阶段位置IK。

### M3：XR 手柄/Tracker 与增量映射

- [ ] 阅读并锁定参考 `xrobotoolkit_client.py`、`live_data_source.py`、`incremental_controller.py` 的实际协议和算法。
- [ ] 新增 `xr_input.py`、采集启动入口和显式左右设备绑定配置；原生服务在隔离进程中运行。
- [ ] 新增 `xr_incremental`，移植参考建立、旋转顺序、离合和上臂方向语义；滤波只在配置选定层执行。
- [ ] 新增 `tests/test_xr_input.py`、`tests/test_xr_incremental_mapping.py`，覆盖左右交换、六轴变化、离合恢复、重定位。
- [ ] 对录制XR输入比较参考与迁移目标，先以位置1e-6m、旋转1e-6rad为纯数学转换容差；若参考时间滤波不可确定，固定时间步后比较，不随意放宽阈值。

交付：`vr_manus` 可显式选控制器或Tracker路径，区别于TJVR路径。

### M4：操作事件与 PICO2 手势

- [ ] 新增操作观察/事件协议及 `operator_input.py`；将动作请求接到现有会话权限校验。
- [ ] 新增 `gesture_recognition.py`：实现归一化指间距离/弯曲判据、稳定时间、迟滞、释放重触发；原生手势仅作为独立可选来源。
- [ ] 新增 `tests/test_operator_events.py`、`tests/test_gesture_recognition.py`，覆盖持续按住一次触发、丢包、乱序、重连按住不触发、抓握不误触发、未授权拒绝。
- [ ] 记录手势类型/质量和事件接受原因；新模式显式开启手势启动绑定，旧模式保持键盘。

交付：裸手手势和手柄按键可产生相同会话动作，但不能绕过状态机。

### M5：官方二代手 retarget producer

- [ ] 新建 `retargeting/` 后端接口及 `producers/hand_retarget.py`，锁定官方Python依赖、YAML与左右模型。
- [ ] 在 `pixi.toml`/锁文件中隔离有冲突的依赖，核对现有推理环境不变。
- [ ] 修改新profile启动绑定，独立producer输出 `HandJointCommand`，executor采用direct路径；旧retarget模式继续保留。
- [ ] 新增 `tests/test_hand_retarget_backend.py`、`tests/test_hand_producer_authority.py`，验证20关节名称/单位、左右手、不重复retarget、错误后端不静默降级。
- [ ] 对open/fist/pinch和运动序列，与参考Retargeter使用相同配置/初态/时间步，目标差异以1e-5rad为初始算法迁移门槛；任何超差必须定位来源。
- [ ] 双输入分别在MuJoCo驱动二代手，禁用手部时不启动该producer。

交付：官方retarget独立运行，sim/real可消费同一关节目标。

### M6：录制回放和诊断闭环

- [ ] 扩展 `recording/session_h5.py`、`recorder.py`、协议版本及原始流字段，不改变既有raw组语义。
- [ ] 新增 `tests/test_dual_input_recording.py`：完整raw、独立有效位、不同频率、epoch、标定摘要、旧H5读取、事件回放不授权。
- [ ] 增加XR/TJVR可视化和新模式状态，保留原overlay接口。
- [ ] 从录制会话重建骨架、映射目标、手关节命令；时序相关算法使用记录时间或固定回放时钟。

交付：所有新路径可追踪、可回放、可解释差异。

### M7：真机配置与接入

- [ ] 新增 `pico2_hands_real.yaml`、`vr_manus_real.yaml`；在来源加载中用显式execution capability替代新入口的硬编码simulation限制，旧配置仍保留simulation-only。
- [ ] 增加 `tests/test_hand_tracking_real_preflight.py`：无confirm、身份不匹配、反馈未就绪、硬件异常、仿真专用IK组合均拒绝。
- [ ] 将新hand producer接到现有二代手direct执行路径；核对固件/序列号/关节反馈与命令顺序。
- [ ] 实现真实反馈超差、输入失效和重新授权策略，增加注入故障测试。
- [ ] 分别完成单手、单臂、双侧验收；v131 real单独做模型/反馈/周期验证，未通过则保留禁止使用。
- [ ] 为SPARK新增独立真机配置和验收记录：核对参考模型与真实关节/TCP、位置/速度/加速度/jerk边界、上臂外侧约束、200Hz周期/延迟，以及model_reference对实测偏差的监督；未通过前保持simulation-only，不回退其他算法。
- [ ] 新增 `tests/test_spark_real_preflight.py` 及故障注入：无可靠双臂状态、单侧输出失败、骨架失效、Headroom异常、回执超时、关节偏差超限、旧epoch和重授权均有确定处理。
- [ ] 按5.2节单侧输出诊断方式完成SPARK单臂验收，再验证双臂成对输出与Manus二代手联动；分别验收reference_direct与实际部署选择的processed_guarded，记录全部参数和算法名。

交付：经设备验收的真机profile；不能只靠mock测试标记完成。

### M8：四组合验收和文档

- [ ] 新建 `scripts/compare_dual_input_reference.py`，以同一原始数据和操作序列分别驱动固定版本原版及迁移版，逐阶段比较采集转换、坐标/标定、映射、SPARK参考、前馈、Headroom、QP、手retarget和输出；生成首个分歧帧及原因报告。
- [ ] 使用固定时钟对照重放及实际进程传输两类测试，覆盖静止、六轴运动、双手合拢、抓握、按键、丢帧/乱序、重定位、失效恢复；不得以最终轨迹接近代替中间阶段一致。
- [ ] 两种输入各自验证sim/real，报告手输入、位姿、操作、保持恢复、record/replay结果。
- [ ] 验证停止会话切换模式后没有旧进程、旧发布者、旧参考/事件残留。
- [ ] 验证 mocap/H5/Regrind 和受保护 PICO2 命令；无新增默认行为漂移。
- [ ] 更新 README、配置示例、启动前置条件和已支持组合表，仅把验收通过的命令标为可用。

## 11. 验证命令与完成判据

### 11.1 原版联合路线等价性门槛

比较前固定原版与迁移版的原始输入、初态、时间步、参数和模型版本；数值容差在运行迁移对照前登记，不可为使结果通过而事后放宽。

| 比较层 | 比较内容 | 判定 |
|---|---|---|
| 原始流/转换 | 序号、epoch、有效位、左右/节点语义、消费帧顺序 | 离散字段及消费序列严格一致 |
| 骨架/位姿 | 同单位同坐标的21点、肩肘腕掌心、映射TCP | 初始位置容差1e-6m，旋转测地角1e-6rad；不直接比较四元数符号 |
| SPARK及QP | 两阶段参考、前馈、Headroom、q/qdot/qddot、接受/hold/reset状态 | q初始容差1e-5rad；其余浮点字段逐项登记量纲和容差，状态及更新顺序一致 |
| 手retarget | 20关节名称/排列、有效性与角度 | 名称/有效位一致，角度初始容差1e-5rad |
| 输出/状态机 | 同周期输出、开始/暂停/恢复、失效处理 | 无额外处理的正常输出等价，原版状态转换逐事件一致 |

浮点差异须有量纲、来源和最大误差报告，不能要求跨平台逐字节一致。发现差异时从首个分歧阶段定位，修复并重跑相关序列；无法解释或未解决则该项失败。

固定时间测试用于算法等价，实时链路另测输入到输出延迟、抖动、丢帧及队列行为；在M0对参考环境测量并冻结实时预算。超预算会影响实际跟手效果，不能靠固定时间回放通过就宣称联合效果验收通过。

所有输入序列必须报告全程覆盖情况。硬保护或集成异常中断的区间单列并验证处置，不能静默删除；若正常工作序列因迁移层频繁中断，应判联合验收失败。真实设备精度和受保护执行差异单独验收，不与原版仿真数值等价混为一谈。

### 11.2 自动化与设备验收

已有自动化基线：

```bash
PYTHONPATH=src/tianji_teleop:vendor/python \
  pixi run python -m unittest discover -s tests -p 'test_*.py'
pixi run -e ik-build build-ik-sim
pixi run python scripts/pico_sim_smoke.py \
  --disable-hands --ik-backend pico_ee_dexhand_qp \
  --arm-target-processor passthrough --joint-trajectory passthrough \
  --command-step-clipping false --joint-limit-source urdf \
  --pico-overlay --ik-target-overlay
git diff --check
```

新测试逐项通过同一 `PYTHONPATH` 运行 `python -m unittest tests.<对应模块> -v`，再运行发现集。上次验证记录为443项、1项跳过；这是历史证据，不代表本计划已执行。标准 `pixi run test` 还要求zenohd与ACL，必须分别记录完整任务和替代发现集的结果。

硬件验收记录每侧TCP误差、手关节误差、输入到命令延迟、IK失败比例、恢复最大命令跳变、反馈偏差及故障处置。具体工程验收阈值由设备配置和操作者使用范围确定，在真实运动测试前固化，不能依据synthetic结果宣称精度达标。

完成矩阵：

| 组合 | 骨架→手 | 位姿→臂 | 操作授权 | 丢失/恢复 | raw记录 | 实际设备验收 |
|---|---|---|---|---|---|---|
| pico2_hands_sim | 必须 | 必须 | 必须 | 必须 | 必须 | 真实PICO2输入 |
| vr_manus_sim | 必须 | 必须 | 必须 | 必须 | 必须 | 真实VR+Manus输入 |
| pico2_hands_real | 必须 | 必须 | 必须 | 必须 | 必须 | 指定机器人/二代手 |
| vr_manus_real | 必须 | 必须 | 必须 | 必须 | 必须 | 指定机器人/二代手 |

阶段可独立交付，但只有四行全部完成，才能称“两种模式均支持仿真及真机”。XR控制器/Tracker与TJVR子模式分别列支持状态，不用其中一项通过代替全部。

矩阵每行必须附带 `arm_input + ik_backend + reference_execution_mode`。本计划指定算法交付中，PICO2两行须有v131证据，TJVR+Manus两行须有SPARK证据；既有IK完成的设备冒烟测试单独报告，不替代这些验收。SPARK原版一致性、处理模式正确性、真机部署验收是三个独立结论。

## 12. 实施后的拟议启动方式

以下是新profile实施后的目标用法，当前不要执行：

```bash
# 外部router及所选采集依赖先就绪
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
  pixi run bash scripts/run_session.sh --profile pico2_hands_sim --viewer
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
  pixi run bash scripts/run_session.sh --profile vr_manus_sim --viewer

# 仅在对应真机profile通过设备验收后使用
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
  pixi run bash scripts/run_session.sh --profile pico2_hands_real --confirm-real
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
  pixi run bash scripts/run_session.sh --profile vr_manus_real --confirm-real
```

VR子模式通过配置选择；切换前停止会话。不自动切换模式，也不自动继承另一种输入的标定、参考位姿或未释放的按键。
