# Manus / PICO 手部骨架与机械臂位姿双通道接入方案

日期：2026-09-07
目标分支：`PICO_Hand_Tracking`
状态：阶段 A（接收/发布/记录）与阶段 B/C 的仿真接口代码已实现，代码未提交。合成 PICO→原生 IK→仿真双臂和舞肌二代手的完整进程联调已通过；真实头显输入仍待验收，实体机器人控制与 Regrind 输入适配仍不在本次范围内。

当前验证：实际 MuJoCo 模型上的合成 PICO→Python dry-retarget→双手关节更新，以及双臂 IK 目标位移测试通过。PICO session 已启用手部 overlay；修正 Python retarget 左手关节名称和 target 节点持续授权检查。原生 IK 已通过本机仿真构建生成，Zenoh 依赖已补齐，双臂 QP 求解探针通过。额外 Regrind 轨迹探针的移动目标误差断言未通过；真实头显到完整仿真会话的联调仍待完成。

## 1. 目标与本阶段范围

完整进程验收补充：`pixi run python scripts/pico_sim_smoke.py` 通过。
使用独立 loopback Zenoh 路由器、合成 TCP 输入和实际 session 启动器；
双臂关节反馈最大变化分别约 0.213 / 0.214 rad，双手分别约 0.464 rad。
断流后停止发布目标并回到 idle。HDF5 保留 454 帧 PICO raw，左右
`joint_poses` 均为 `(454, 26, 7)`，所有 raw 数据集首维一致。
结果：`/tmp/pico-sim-smoke-9jyor5wp/result.json`（临时验证产物，不入 Git）。
本轮同时修复 simulation vendor 检查、observation 配置覆盖路径和记录器
关闭时与订阅回调并发写入的竞争；78 项相关回归测试通过。
上述证据不代表真实头显、Manus 完整来源或 Regrind 轨迹回归已通过。

目标是让当前工程支持 Manus 和 PICO 两套输入组合，启动时通过配置选择一种。每套组合包含手指骨架输入和机械臂位姿输入，只创建该组合所需的接收器。

Manus组合的机械臂输入沿用 `TJ_arm_control` 的 `feature/pico-manus-teleop-experiments` 分支中的人体掌心链路；PICO组合使用 `PICO_2` 双腕相对每帧当前头显参考系的位姿。位姿适配细节见第11节。

| input_profile | 手指通道 | 机械臂位姿通道 |
|---|---|---|
| manus | Manus25点→21点 | 旧PICO修正人体掌心链路 |
| pico | PICO_2的26点→21点 | PICO_2双腕→当前头显参考系 |

观察入口实现数据接收、统一骨架发布和 session HDF5 记录，不驱动机器人；新增仿真入口只连接现有 MuJoCo/Wuji dry-run 与 IK 链路，不打开真实设备。保留当前 Regrind 模型推理链路，新增采集功能不改变其输入契约。

提供两种来源入口和一个集成会话入口：

- `pico_capture`：只接收和记录 PICO 数据，并强制 `input_profile: pico`。
- `manus_capture`：接收 Manus 骨架和旧 TJVR 掌心数据，并强制 `input_profile: manus`。
- `hand_tracking_observation`：通过配置选择 Manus 或 PICO，复用相同接收、转换和记录模块。

启动选择仅约束当前工程的接收器，不负责停止外部已运行的采集程序。暂不支持同时启用两套输入组合、运行时热切换或自动回退。Manus组合仍需要旧PICO人体掌心来源；互斥约束不应错误地禁用这一来源。

## 2. 数据流与模块边界

```text
input_profile = manus
  Manus JSON Zenoh ──→ 25→21转换 ───────────→ 手指骨架观察
  旧PICO人体UDP流 ───→ 修正掌心位姿适配 ──→ 机械臂输入观察

input_profile = pico（与上面的组合互斥）
  PICO_2 TCP帧 ──┬──→ 26→21转换 ──────────→ 手指骨架观察
                └──→ inverse(头) × 双腕 ──→ 机械臂输入观察

两类观察 → 发布 / 显示 / HDF5派生记录
原始数据 → HDF5原始记录

仿真控制链路：手指骨架→现有 Wuji Hand 2 retarget；机械臂输入位姿→位姿映射工厂→目标处理工厂（默认透传）→现有 IK 工厂。真实设备执行仍由既有 real profile 单独控制。
```

建议在 Python 包内新增 `hand_tracking/` 模块：

```text
hand_tracking/
    models.py       # 原始帧、统一21点、独立机械臂输入位姿
    manus.py        # Manus JSON Zenoh接收、语义转换
    pico.py         # PICO TCP解析、26→21转换
    legacy_pico.py  # 旧PICO人体UDP流解析及掌心适配
    runtime.py      # 来源选择、有效性、超时、生命周期
```

转换函数保持纯函数，传输和发布独立处理。两个入口共享运行模块，避免复制协议解析、映射和记录逻辑。

当前 `mocap/aligned/hands` 输入已经是21点，保留其兼容路径，不再进行25→21转换。新设备输入不伪装成原有 Motive 对齐帧。

第 1—13 节定义的阶段 A 模块、主题、配置、HDF5 路径、映射/目标处理工厂和仿真目标桥已在当前工作区落地。`hand_tracking_observation` 是接收-only 入口；`hand_tracking_sim` 与 `hand_tracking_sim_manus` 才会在 coordinator 授权后发布既有 target 主题，并继续复用当前 IK、MuJoCo 和 Wuji Hand 2 downstream。

## 3. 统一21点接口

采用标准 MediaPipe 21点顺序，单位为米：

| 索引 | 语义 |
|---|---|
| 0 | wrist |
| 1–4 | thumb：CMC、MCP、IP、TIP |
| 5–8 | index：MCP、PIP、DIP、TIP |
| 9–12 | middle：MCP、PIP、DIP、TIP |
| 13–16 | ring：MCP、PIP、DIP、TIP |
| 17–20 | pinky：MCP、PIP、DIP、TIP |

左右手分别发布，建议消息字段：

| 字段 | 含义 |
|---|---|
| source | manus / pico |
| source_instance_id | 接收运行实例或来源生命周期标识 |
| sequence | 单调递增序列 |
| source_timestamp_ns | 来源时间，保留其时钟域说明 |
| received_timestamp_ns | PC接收时间，明确使用的时钟域 |
| side | left / right |
| mapping_version | 转换规则版本 |
| coordinate_frame | 骨架轴向和参考系标识 |
| keypoints_m | `(21, 3)`，减去第0点后的骨架 |
| joint_valid | `(21,)`，逐点有效性 |
| valid | 所需21点是否全部有效 |
| wrist_pose | 可选腕部位姿，含独立有效性和参考系 |

协议封装沿用当前工程的版本、发布实例、router身份和序列校验约定；同时保留关联原始帧所需的来源序号。

减去腕部位置只消除平移，不消除旋转。不能把 Manus 局部坐标、PICO 初始化参考系和 Motive 世界坐标视为同一世界坐标。轴向修正由设备转换器显式定义；未完成对齐时保留不同的 `coordinate_frame`，下游不得无条件混用。

本阶段统一的是关节拓扑、顺序、单位和消息结构，不宣称两种来源已完成同一腕部朝向坐标归一化。后续接retarget必须明确其输入轴向、左右手约定和内部掌面旋转处理，并用相同手势验证；不能仅因数组形状是 `(21,3)` 就判定输入兼容，也不能重复执行retarget内部已有的方向归一化。机械臂的头相对变换不自动应用于手指骨架。

PICO原始腕部位姿保留在其初始化参考系中，机械臂观察通道另外输出 `pico_head_current` 下的双腕；Manus原始局部骨架不能凭空生成世界腕部位姿，机械臂通道使用旧PICO掌心来源。后续机械臂控制需要另外定义标定和人体到机器人位姿映射。

## 4. Manus 接收与25→21转换

参考代码：

- `/home/zj/current_robotics/pico-manus-teleop/manus`
- `/home/zj/current_robotics/pico-manus-teleop/wuji_teleop`

订阅现有 JSON Zenoh 主题：

```text
manus/raw_skeleton/left_hand
manus/raw_skeleton/right_hand
```

利用 `node_semantics` 中的关节语义及 `array_index` 查找节点，保留 wrist、thumb四点，以及其他四指各四点，去掉四个非拇指 metacarpal。

现有参考转换器的语义选择为：

- thumb：mcp、pip、dip、tip。
- index/middle/ring/pinky：pip、ip、dip、tip。

这些名称是源端语义别名；非拇指 proximal 节点原点对应标准21点中的 MCP，不能根据别名把输出误解为 PIP。参考实现执行 `Y` 取反，应作为 Manus 专属转换规则验证并版本化。

初期要求 JSON 带完整元数据；不接受缺少语义信息的 MNS1 二进制作为可转换输入，不依赖未经验证的固定25点下标。

接入验证注意：

- 语义缺失、重复、越界时标记转换失败，不猜测节点。
- 参考 SDK 注释与现有转换器对拇指 distal 的要求存在差异，需要通过实际 NODE 元数据确认。
- 上游 rawviz 的拓扑数组与姿态数组顺序需要核实；元数据存在不等于两者已经正确对应。
- 上游未提供可靠来源实例标识时，接收端实例不应被宣称为硬件实例；序号回退应触发失效和重新同步。

## 5. PICO 接收与26→21转换

参考代码：`/home/zj/current_robotics/PICO_Hand_Tracking/PICO_2`。

复用其 ADB 转发和 TCP 数据接收协议：设备端口10002，头部格式 `<BBqI`，magic为 `0xAB`，消息类型为 `0x40`；已知版本1包含26个手部关节，标准负载为1968字节。

解析应处理 TCP 分包、粘包、断连、超时和重连，并校验版本、长度、关节数量及有效性字段。未知协议不能静默按当前布局解释。

PICO到标准21点的映射：

```python
PICO_TO_MEDIAPIPE = [
    1,                  # wrist
    2, 3, 4, 5,         # thumb
    7, 8, 9, 10,        # index
    12, 13, 14, 15,     # middle
    17, 18, 19, 20,     # ring
    22, 23, 24, 25,     # pinky
]
```

去掉 palm（0）以及四个非拇指 metacarpal（6、11、16、21）。参考 APK 已输出 FLU（X前、Y左、Z上）和 xyzw 四元数，不能再次套用 Manus 的 Y取反。

PICO参考系由首次有效头显位姿确定原点和水平朝向，不等于机器人或 Motive 世界参考系。保留头显、手、腕部及逐关节有效性；不得把无效关节的零值当作有效观测。

### 5.1 帧标识与有效性

当前已检查的PICO_2协议没有设备帧序号或tracking epoch。接收端定义 `receiver_instance_id`、`connection_generation` 和 `receiver_frame_sequence`：进程启动生成新实例标识，每次TCP连接成功增加连接代次，每个完整帧分配接收序号。原始记录与派生观察携带相同关联键；发布消息自身的序号与接收帧序号分别保存。

这些字段只标识接收端可见的数据，不能证明设备端没有丢帧，也不能把毫秒时间戳当作唯一帧ID。重复时间戳不单独构成重复帧证据。断连时清空不完整包并使观测失效；时间戳回退标记来源不连续，清除派生缓存和未来有状态算法的历史，重新建立时间基线。没有来源epoch时不猜测头显重定位事件。

参考接收器中的 `wrist.valid` 只反映手部块标志，而 `hands[side].valid` 还结合全局flags。新适配器的腕部有效性必须同时满足全局侧别标志、手部块标志、有限位置及合法非零四元数。骨架点还需对应joint有效标志；头相对腕部还需同帧头显有效。四元数数值检查与归一化策略明确版本化，原始值不覆盖。

## 6. 配置、发布与观察运行

阶段 A 配置：

```yaml
input_profile: pico         # manus | pico，选择完整输入组合
observation_only: true
```

`input_profile` 取代早期草案的 `hand_tracking_source`。本阶段观察入口必须要求 `observation_only: true`；设为false时明确报错，不能隐式启动尚未实现的控制模式。PICO组合的机械臂参考系固定为已确认的 `pico_head_current`，不增加未经需求确认的自动回退或参考系切换。

启动时验证来源，只创建选定组合所需适配器。Manus模式不初始化新的PICO_2手部连接，但保留旧人体掌心来源；PICO模式不订阅Manus及旧人体掌心来源。两个入口复用实例约束，避免重复接收同一来源。

阶段 A 观察主题：

```text
tianji/observation/hand/left
tianji/observation/hand/right
tianji/observation/arm_input/left
tianji/observation/arm_input/right
tianji/raw/pico_hand_tracking
tianji/raw/manus_hand_tracking
tianji/raw/legacy_pico_palm
```

观察启动路径仅运行所需接收、发布、诊断和记录组件，不创建机器人控制目标发布器，不启动机械臂/手部执行器，不发送控制启动请求。

当前 `hand_mode: disabled` 不能代表观察模式，因为现有会话仍可能启动机械臂链路。需要在启动脚本中增加明确的观察路径，并处理当前 source 对 coordinator 环境的依赖。

两个通道分别报告有效性、接收时间、数据年龄和错误原因。Manus骨架与旧掌心流独立到达，不能伪造同步帧；后续联合控制需要另行约定时间偏差和失效策略。观察界面显示左右骨架、掌心/腕部坐标轴、来源与参考系、帧率和有效状态，并明确标注未启用机器人控制。

## 7. Session HDF5 扩展

新增完整 `raw/pico_hand_tracking`，保存：

- 原始协议版本、flags、关节数量、设备时间和PC接收时间。
- 来源实例、序列及原始帧关联信息。
- 头显位置、四元数和有效性。
- 左右手腕部位姿及有效性。
- 左右26点的位置、四元数、半径和逐点有效性。
- 原始包字节，以保留保留位、无效样本原值等解码字段之外的信息。

统一骨架和机械臂输入分别记录：

```text
raw/pico_hand_tracking/
raw/manus_hand_tracking/
raw/legacy_pico_palm/
observation/hand_tracking/left/
observation/hand_tracking/right/
observation/arm_input/left/
observation/arm_input/right/
```

统一记录包含21点、逐点有效性、来源、时间、参考系、映射版本和原始帧关联信息。观察数据不写入控制用的 `target/hand`。

机械臂观察记录包括输入来源、跟踪原点（palm/wrist）、参考系、位置、四元数、独立有效性、时间及原始帧关联信息。meta保存解析后的实际配置、映射版本、参考代码提交号和所用变换参数，便于解释与复现派生结果。

Manus左右手可能异步到达，独立记录，不强行拼成同一时刻的双手帧。Manus组合必须同时记录 `raw/manus_hand_tracking`（原始JSON、节点位置/旋转、语义元数据、来源时间与序号）和 `raw/legacy_pico_palm`（原始UDP包、协议版本、来源时间、序号、epoch及掌心/上肢字段）。上述路径是schema布局，不表示同时启用两套输入；只有选定组合写入样本。

Session schema 已显式升级为 1.1；新读取器兼容已有1.0文件，现有读取器严格检查结构，不能声称旧读取器能直接读取扩展文件。schema 1.1 可由接收-only observation 入口或仿真控制会话的被动 recorder 使用，保留当前完整性标记和追加写入机制。

观察入口在独立观察会话中由单一 `ObservationRuntime` 同步写入选定来源的 raw/derived HDF5 流；它不订阅同 router 下无关机器人的控制数据。仿真控制会话则只创建一个 `SessionRecorderNode`，被动订阅选定的 raw/observation 流以及现有 target/joint/session 流，统一写入 schema 1.1；观察进程本身不再打开第二个 HDF5 writer。

### 7.1 写入与完整性规则

区分“完整保留字段”和“保存了全部接收帧”。阶段 A 观察入口和仿真控制会话都使用单一 `SessionH5Writer`：前者由 `ObservationRuntime` 直接写入，后者由被动 `SessionRecorderNode` 汇总写入；同一会话不会创建第二个 HDF5 writer。后续高频可视化/控制接通前，需要按本节约束补充有界队列、单写线程和显式丢帧计数。

阶段 A 已保留接收端帧序号、连接代次、来源/接收时间和 raw/derived 关联键；跨主题到达顺序不作为帧对应关系。队列溢出、记录器入队等指标属于后续异步记录器增强项，当前不能据此声称无损记录。

当前同步 writer 的磁盘异常会终止观察入口并保留 `complete=false` 文件；正常关闭时停止接收、flush 后关闭文件。阶段 A 不提供异步队列排空和独立记录状态计数，因此不把同步写入结果表述为端到端无损。

仿真控制会话使用 `recording/session_hand_tracking.yaml`，并以
`hand_tracking_sim` 或 `hand_tracking_sim_manus` 作为 schema 1.1 的
`source_type`。该 recorder 只订阅并保存数据，不发布目标、不请求会话状态，
因此观察 receiver、target bridge、IK、MuJoCo 和 Wuji Hand 2 之间只存在一个
HDF5 写入者。PICO profile 记录 PICO raw 与 PICO/21 点/腕部观察；Manus profile
记录 Manus raw、历史 TJVR palm raw、Manus/21 点观察以及对应 arm-input。两种
profile 都同时保留现有 `target/*`、`joint/*` 和 `meta/session_events` 流。

现有文件完整性标记表示写入生命周期是否正常结束；新增记录损失指标单独表达是否有已知漏帧。正常关闭不能清除会话中曾发生的损失。无损验收仅覆盖PC接收端可见的帧，协议没有设备序号时不能推断设备至PC之前的完整性。

原始包保留无效数值。标准观察使用有效性掩码和明确的空值编码：JSON 无效位姿使用 `null`，点坐标保留固定形状并附 `joint_valid`；HDF5 固定数组的无效位姿槽使用 NaN 并附独立有效性掩码，不能被消费者当作有效观测。schema 1.1 已固定版本、字段、类型、形状和空值编码。

## 8. 当前 Regrind 模型推理兼容边界

当前 `tianji_teleop/regrind_policy.py` 的模型使用123维observation，输出26维action，不直接消费人的21点骨架。

| 模型输入 | 来源与适配要求 |
|---|---|
| 当前及上一帧20维机器人手部关节角 | 实机模式使用机器人反馈；人的骨架需要retarget才能产生关节目标 |
| 当前及上一帧腕部位姿 | 保留既有推理语义；人的PICO腕部不能未经标定和语义适配直接替换 |
| 物体/锤子位姿 | 当前依赖Motive，手部骨架不能提供 |
| 上一次action、phase | 由推理运行时维护 |
| 参考腕部位姿、参考20维关节角 | 当前从参考轨迹文件加载 |

后续扩展关系：

```text
Manus / PICO → 统一21点 → Retarget → 机器人关节目标 / 参考轨迹

机器人状态 + 物体位姿 + 参考轨迹 + 推理历史
                         ↓
                  123维observation
                         ↓
                   Regrind模型
```

Retarget输出代表关节目标，不能冒充机器人实际反馈。现有实机推理检查Wuji手部反馈及其时效，应继续保留。

新增原始HDF5不能直接作为当前模型参考文件。后续需要生成当前加载器要求的字段：

```text
regrind_retargeting_root_pos
regrind_retargeting_root_quat
regrind_retargeting_joints
object_pos
object_quat
```

该转换还需要retarget、物体跟踪、参考系对齐和时间对齐。此阶段不实现新的模型输入适配，也不承诺仅凭21点映射即可获得与训练数据一致的推理效果。

## 9. 有效性与验证

逐手处理无效帧。任何必要映射点无效时，该手的完整21点观测无效；原始样本仍可记录用于排查。输入超时后失效，不重复发布旧数据为新鲜有效帧。

来源时钟和PC时钟分别保留，不直接相减计算延迟。序列重复、乱序、回退和重连需要明确处理，避免跨重启继承旧有效状态。

实施验收重点：

- Manus语义映射在数组顺序变化时仍正确，缺失及重复语义可检测。
- PICO索引、单位、轴向、逐点有效性和原始字段保存正确。
- TCP分包、断连重连及来源超时可恢复且不产生虚假有效帧。
- HDF5原始和统一数据可回读、可关联，新读取器兼容旧session文件。
- 启动时只激活选定来源，来源入口和集成入口行为一致。
- 观察运行不启动机器人执行器，不产生target、command或控制启动消息。
- 原有mocap输入和Regrind推理契约不受影响。

## 10. 分阶段实施与验收

### 阶段A：采集、观察与记录（已完成）

定义双通道消息、实现25/26→21转换与PICO当前头参考系双腕转换，移植PICO_2接收，接入 Manus JSON 及旧 PICO 掌心流，实现互斥 profile、两个来源入口和一个集成观察入口。扩展 HDF5 原始/派生记录、元数据及旧文件兼容读取。

验收覆盖映射、头手整体移动不变性、同帧有效性、重连、独立通道状态、原始与派生回读关联，以及观察入口不启动机器人控制组件。旧参考系尚未确认的数据只能标为未标定观察，不能标成Base→TCP。完整 Manus 组合的真实来源验证仍需第14节的实际来源证据。

### 阶段B：算法组件化与无控制预览（已完成）

已提取位姿映射工厂和目标处理工厂，提供 `direct_pose`、`relative_home`、`passthrough` 及 `conditioned` 后端，并用纯逻辑测试验证变换顺序、零点初始化、复位和无效输入行为。工厂本身不启动执行器；观察 profile 也不会因配置存在而发布控制目标。

### 阶段C：仿真接通（接口已实现，完整运行验收待完成）

已连接“选定输入→统一观察→手部适配/retarget目标、机械臂位姿映射→目标处理→当前 IK→MuJoCo/Wuji Hand 2 dry-run”。目标桥要求显式 `s`、coordinator 授权和新鲜有效输入后才发布 target；PICO 与 Manus 通过两个互斥 simulation profile 选择。真实设备仍不由这些 profile 启动。

实机控制和 Regrind 参考数据适配不属于本次实现的自动延伸，后续需分别明确需求与验收；现有 `mocap_live`、H5/replay 和 Regrind contracts 保持不变。

## 11. 机械臂位姿适配细节

PICO 仿真控制点采用手腕中心，底层 IK 仍解 `TCP_Link_L/R`。
`relative_home` 的 `ik_tcp_to_control_pose` 表示原 IK TCP→控制点的固定
变换，两侧均为局部 +Z 36.5 mm、朝向不变。`home_pose` 保持原 IK TCP
语义；内部先组合工具变换得到控制点 Home，应用人的相对运动后，再右乘
工具变换的逆得到 IK 目标。该换算属于 `relative_home_v2`，缺省工具变换
为单位变换。PICO 默认使用 passthrough；本次未改变 Manus 的 direct_pose。
此处手腕中心坐标轴沿用原 TCP 朝向，不等同于手模型 `l_wrist/r_wrist`
连杆自身的坐标轴。改变偏移参数时应重新启动会话并采集参考。

手部21点只描述手指形状，机械臂需要另外保留带参考系的6DoF掌心或腕部位姿。不能用已经减去腕部位置的21点恢复手在空间中的位置。

| 启动组合 | 手指骨架来源 | 机械臂位姿来源 |
|---|---|---|
| Manus | Manus原始25点→统一21点 | `TJ_arm_control` 的 `feature/pico-manus-teleop-experiments` 分支中的PICO人体掌心链路 |
| PICO | `PICO_2` 原始26点→统一21点 | `PICO_2` 的左右腕部位姿和头显位姿，形成头参考系下的双腕位姿 |

参考分支的Manus集成文档明确保留既有PICO机械臂UDP/QP路径，Manus负责手指。该分支同时包含多种机械臂模式，后续移植需要核实实际启动配置所选的掌心映射模式，不能把某个候选算法当作唯一现行路径。

本次核对基线为该分支提交 `2bcfe09e2c78a48c7ba63943deef83d04a139ff8`。参考工作区存在未提交实验修改；本文件中的代码结论基于提交内容，不代表这些本地实验已经纳入方案。

```text
Manus组合
  Manus25点 ─────────────→ 统一21点 ───────→ 手部观察/记录
  旧PICO人体掌心链路 ────→ 掌心6DoF适配 ─┐
                                        │
PICO组合（与Manus组合互斥）               ├→ 统一机械臂输入位姿 → 观察/记录
  PICO_2双腕+头显 ───────→ 头参考系双腕 ─┘
  PICO_2手部26点 ────────→ 统一21点 ───────→ 手部观察/记录

仿真控制：统一机械臂输入位姿 → 位姿映射工厂 → 目标处理工厂（可透传） → 当前工程 IK
```

### 11.1 PICO原始参考系与当前头参考系

`PICO_2/pc_scripts/hand_tracking/使用方法.md` 描述：首帧有效头显位置成为固定FLU原点，首帧头显水平朝向成为X前向，去除俯仰和横滚以保持Z向上。接收器分别解析头显和双腕位姿，没有进行当前头参考系变换。

已确认的设计选择：PICO机械臂输入使用每帧当前头显坐标系下的双腕完整6DoF位姿，参考系标识为 `pico_head_current`。这样人在整体平移或改变朝向时，只要手相对头的位姿不变，机械臂输入就不变。原始包的“初始头显锚定坐标系”仅用于保留原始数据，不作为本方案PICO机械臂输入的参考系。

每帧按以下方式派生当前头参考系位姿，使用完整头显旋转，而非仅减去头部位置或仅补偿yaw：

```text
T_head_wrist(t) = inverse(T_tracking_head(t)) * T_tracking_wrist(t)

p_head_wrist = R_tracking_head.T * (p_tracking_wrist - p_tracking_head)
R_head_wrist = R_tracking_head.T * R_tracking_wrist
```

头与腕取同一原始帧并分别验证有效性。头部无效时，两侧派生机械臂输入均无效；单侧腕部无效时，仅该侧派生输入无效。不能使用上一帧头部位姿拼接当前腕部，不能在失效时自动退回固定参考系。手指骨架独立判定有效性。

完整当前头参考系会随头显平移、转动；头单独转动而手保持世界静止时，相对腕部位姿会变化，这是选定参考系的预期行为。当前 simulation target bridge 消费该派生位姿，不再次叠加头部世界运动。

HDF5同时保存原始跟踪参考系中的头显和双腕位姿，以及 `pico_head_current` 下的派生双腕位姿、有效性和原始帧关联信息。

增加坐标变换验收：对头和双腕同时左乘任意相同刚体变换G，计算结果应在数值容差内不变，即 `inverse(G * T_head) * (G * T_wrist) = inverse(T_head) * T_wrist`。覆盖整体平移、整体转向及组合变换，并验证头单独转动时结果按公式变化。

### 11.2 统一机械臂输入及仿真 IK 适配

独立定义机械臂观察位姿，至少包含 `side`、`tracked_frame`（palm/wrist）、`reference_frame`、位置、xyzw四元数、有效性、时间戳和原始帧关联信息。当前已采用 `tianji/observation/arm_input/{side}` 主题和独立 HDF5 观察分组。

掌心和腕部是不同跟踪原点，进入控制时必须显式定义各自到机器人TCP的映射。统一的是输入消息结构和下游目标契约，不应直接混用原始位姿或对已完成机器人坐标映射的掌心重复映射。

当前 `mocap_live` 从 `mocap/aligned/hands` 的 `wrist_pose` 生成相对启动参考的TCP目标；新增 `hand_tracking_target` 已按 profile 消费上述两套组合，并通过既有 `tianji/target/arm/{side}` 接入当前 IK producer。PICO 使用 `relative_home`，Manus 历史掌心使用 `direct_pose`；本次不移植参考工程的整套 QP、前馈或人体肘部冗余策略。

仅有双腕和头显不能恢复完整人体肩肘几何，不能直接复用依赖这些数据的臂角任务。后续应沿用当前工程的默认肘部方向，或另行增加明确的冗余策略。

观察 profile 仍只观察和记录；simulation profile 才会在显式生命周期授权后将派生位姿送入现有 IK。两者都不启动真实机器人。现有 Regrind 模型推理兼容边界不变。

### 11.3 旧掌心链路已核实的接入边界

参考提交中的相关文件：

- `include/tianji_qp_ik/pico_teleop_protocol.hpp`：旧UDP协议、来源序号、tracking_epoch、时间戳和8点上肢骨架定义。
- `src/pico_udp_receiver.cpp`：解码原始包、选择掌心来源、检查流状态并发布最新帧。
- `src/pico_mapped_corrected_palm.cpp`：`selectMappedCorrectedPalm()` 提取修正骨架的左右hand节点并转换末端朝向基。

该掌心选择器读取8点骨架中的左hand（索引3）和右hand（索引7），位置保持不变，朝向为 `R_hand * B_side`。左右基矩阵列向量分别为 `[-Y, -Z, X]` 和 `[Y, Z, X]`。此处处理的是旧人体上肢协议，不是Manus25点，也不是PICO_2的26点手部协议。

该函数不依赖机器人几何，不执行IK或滤波；因此它可以作为来源适配的明确边界，但不能据此断言其输入坐标已经等于当前工程的Base系。旧UDP包未携带完整的坐标系字符串，具体参考系和上游修正含义还需结合实际发送端配置核实，并作为配置元数据记录。

推荐接收原始旧UDP帧，保存原始包和源位姿，再调用等价纯映射生成标明朝向约定的观察位姿。不要直接复制最终关节命令，不把旧控制器的前馈、Headroom、QP或滤波链纳入采集模块。后续TCP映射不能再次应用同一朝向基。

旧接收器当前的掌心选择器会校验完整双臂骨架，失败时整帧拒绝；移植时应保留并显式记录该来源有效性限制，不能宣称上游已支持逐侧独立有效性。新的观察消息仍可表达独立通道状态。

尚待实机/发送端证据确认的事项集中为：旧链路实际采用的profile及参考系、Manus元数据与姿态数组的对应关系及拇指语义。它们不影响先实施纯转换器、PICO头相对观察和记录，但在宣称完整Manus组合已验证前必须完成核实。

## 12. 可插拔位姿映射工厂

需求确认：位姿映射算法应与IK算法分别可选。旧人体掌心直接提供末端几何目标，不应被强制转换成相对启动姿态的运动增量。采用统一映射接口和显式注册工厂，启动时选择后端；初期无需动态库加载、运行中热切换或插件自动发现。

当前工程的 `sources/common/target_mapper.py` 仍保留既有 mocap 目标映射；新增 `sources/common/pose_mapping.py` 提供独立的 `direct_pose` / `relative_home` 工厂。后者只处理已经在 Base 系中的 TCP 目标，不能直接承担任意设备坐标到 Base 系的转换。新增 hand-tracking target bridge 已使用这套工厂，既有 `mocap_live` 行为不做破坏性重构。

### 12.1 区分两种“相对”

- 头相对坐标：`inverse(T_tracking_head(t)) * T_tracking_wrist(t)`，表达当前手相对于当前头的位置和朝向，是已确认的PICO输入定义。
- 启动相对控制：比较当前输入与启动时输入，将变化叠加到机器人Home目标，是一种可选控制映射算法。

前者不要求使用后者。PICO头相对腕部位姿既可以进行直接几何映射，也可以选择启动相对映射；设备适配层不应决定或隐式启用启动相对控制。

### 12.2 接口与后端

已实现的最小接口：

```python
mapper = create_arm_pose_mapper(backend, config)
mapper.reset()                         # 清除旧来源/会话状态
mapper.initialize(reference_or_none)    # 仅相对映射需要启动参考
result = mapper.map(arm_input)          # 输出Base→TCP目标与诊断
```

`arm_input` 携带侧别、跟踪原点、参考系、位姿、有效性和来源帧关联。结果携带侧别、Base→TCP位姿、有效性、后端/映射版本及原始帧关联；无效或未完成标定时不得伪造可用目标。直接映射不要求记录人体启动零点；相对映射在缺少参考时显式报告未初始化。每侧状态隔离。

| 后端名称 | 行为 | 启动人体零点 |
|---|---|---|
| direct_pose | 固定参考系与跟踪原点变换，将掌心/腕部几何位姿映射到TCP | 不需要 |
| relative_home | 输入相对启动参考的变化映射到机器人Home，兼容现有遥操作行为 | 需要 |

最初只实现这两个实际需要的后端。已处于Base→TCP的输入可使用 `direct_pose` 的显式恒等变换配置，不必额外引入同义后端。

直接刚体映射的基本形式为：

```text
T_Base_TCP = T_Base_InputReference * T_InputReference_Tracked * T_Tracked_TCP
```

用于人体到机器人几何对应时，这些是经过确认的映射标定参数，不等于头显与机器人之间未经测量的真实外参。需要人体尺度缩放时应显式扩展位置映射规则，不能把非刚性缩放塞入旋转矩阵。

对PICO，`InputReference` 为 `pico_head_current`，`Tracked` 为wrist；该固定映射不再次引入头在跟踪世界中的位姿，因而保持整体移动不变性。对旧掌心，明确输入是否已经乘过第11.3节的末端朝向基，并保证该操作只做一次。经核实输入与当前Base/TCP完全一致时才可使用恒等映射。

### 12.3 几何映射、目标整形与IK各自负责什么

```text
设备接收/语义适配
        ↓
机械臂输入位姿（标明坐标系）
        ↓
PoseMapper工厂：direct_pose / relative_home / 后续新算法
        ↓
未整形的Base→TCP几何目标
        ↓
TargetProcessor工厂：passthrough / conditioned
        ↓
ArmIkSolver工厂：pinocchio_qp / pinocchio_cpp / tianji_official
```

现有 `EndEffectorTargetMapper` 仍将旧 mocap 的映射和 TargetConditioner 封装在一起；新增 hand-tracking 路径已把坐标映射与目标处理拆开，坐标、零点和映射变换归位姿映射层，约束与时序归目标处理层。新增路径保持 `relative_home` 的运算顺序，不在迁移前后重复施加增益；直接模式不隐式围绕 Home 缩放绝对几何目标。

映射器不订阅传感器、不驱动机器人、不调用IK。目标处理为独立可选算法，`passthrough` 保持几何目标不变，`conditioned` 仅执行一次整形。区分映射输出和处理后输出，使对比记录能够解释目标为何变化。关闭IK前目标整形不关闭IK及执行链的关节约束。

### 12.4 配置与兼容策略

当前仿真配置将输入、映射、目标处理和 IK 分别选择，放在各自组件配置中：

```yaml
# 输入组合配置
input_profile: manus

# 位姿映射配置
arm_pose_mapper: direct_pose

# IK前目标处理配置
arm_target_processor: passthrough

# IK producer配置
ik_backend: pinocchio_qp
```

Manus旧掌心链路选择直接映射，所需参考系和TCP参数核实后显式填写。PICO的头参考系输入已经确认；直接/启动相对的具体控制模式与标定参数不由采集器擅自决定。现有mocap遥操作保持relative_home兼容默认。

工厂应拒绝未知算法名、缺失标定以及不匹配的输入参考系/跟踪原点，不进行静默后端回退。Source运行循环使用统一接口，设备类型与映射选择不再通过散落的条件分支耦合。

观察 profile 只采集和记录双通道数据；仿真 profile 通过独立的 `hand_tracking_target` 在 coordinator 授权后运行映射器并发布候选/实际 target。配置映射后端本身不授权启动控制，且真实 profile 不会被这些配置自动放行。

### 12.5 扩展验收

- direct_pose对相同输入不依赖人体启动姿态；非恒等旋转和平移的组合验证变换顺序，左右朝向基不重复应用。
- relative_home复现旧路径的初始目标、平移和旋转响应，重置后不能继承旧参考。
- PICO头与手共同整体变换时，头相对输入及固定直接映射输出保持不变。
- 工厂选择与错误配置有明确行为；更换映射后端不修改设备接收器或IK调用代码。
- 映射前后及目标整形后数据可关联，旧控制路径的增益和整形不被重复施加。

本节定义并已实现 hand-tracking 路径的映射结构；映射工厂有纯逻辑测试，并已由 simulation-only target bridge 接入现有 IK target 接口。既有 `mocap_live` 路径保持原实现。

## 13. 可插拔目标处理工厂

目标处理从映射类中独立提取，与映射工厂和IK工厂分别配置。选择 `passthrough` 表示关闭IK前笛卡尔目标整形，而不是通过极大限值模拟关闭。

| 后端 | 行为 |
|---|---|
| passthrough | 校验有效性、Base/TCP契约和有限数值，保持有效目标位置与朝向原样输出，无滤波、缩放、限速或Home叠加 |
| conditioned | 复用现有工作空间、笛卡尔速度及加速度处理；映射增益已迁到映射层 |

已实现接口：

```python
processor = create_arm_target_processor(backend, config)
processor.reset(initial_target_or_none)
result = processor.process(mapped_target, dt_s=control_period_s)
```

结果保留原始帧关联、侧别、参考系及处理器版本，提供处理后目标与约束诊断。无效、超时的目标由统一校验拒绝，透传不把无效输入变成有效输出。`passthrough` 不隐式归一化或改写输入；姿态合法性和必要的数值归一化在上游明确完成。

`conditioned` 每侧维护独立历史，显式初始化；来源不连续、重新启动或恢复时重置，不能跨会话沿用速度状态。阶段B保留当前固定控制周期行为；重复的来源帧可以在处理tick间持续追踪，但不得更新原始接收时间而伪装为新鲜输入。若以后支持可变dt，需独立验证算法和参数行为。

当前默认组合：旧 mocap 继续采用既有 `relative_home + conditioned` 行为；Manus 直接掌心路径采用 `direct_pose + passthrough`；PICO 采用当前头参考系输入和 `relative_home + passthrough`。每个 hand-tracking 仿真 profile 都在配置中显式声明组合。

`TJ_arm_control` 某些路径关闭通用Cartesian OTG，但仍包含速度前馈、Headroom或控制器内部约束。本方案不声称该工程完全没有动态处理；增加透传后端也不等于已完整复现其响应。

当前工程IK后仍有独立关节轨迹限制，IK内部和执行器也有各自约束。选择透传不会关闭这些机制。算法对比应记录处理前TCP、处理后TCP、IK结果和后续关节目标，避免把所有响应差异归因于IK。

阶段B验收已覆盖：透传对有效输入不改变几何目标；两种处理器可由工厂选择；未知后端被拒绝；复位和来源失效不会复用旧状态。算法插件不自动注册进程；只有 simulation-only target bridge 在 coordinator 授权后发布控制目标。

## 14. 审查后的待确认事项与完成边界

| 待确认事项 | 所需证据 | 影响范围 |
|---|---|---|
| 旧掌心的真实参考系、朝向修正和运行profile | 指定分支发送端代码/实际配置及样本，记录提交与参数 | 候选位姿已接入仿真 IK 目标链路；确认前不能声称已完成真实 Base/TCP 标定或实机验证 |
| Manus节点数组对应关系与拇指语义 | 实际NODE元数据和同帧姿态样本 | 严格语义转换器和仿真链路已实现；确认前不能声称真实设备的25→21语义已用实测样本验证 |
| retarget接受的轴向、左右手与内部归一化 | 选定Retargeter代码、配置及相同手势对照 | 阶段A保留来源坐标；进入手部控制前完成 |
| 新session schema的版本及字段布局 | 已固定为 schema 1.1；后续变更需增加版本 | 阶段A写入器/读取器的必要契约 |

上述未决项不改变已确认的PICO当前头参考系需求，也不阻止先实现PC接收、纯变换和观察。文档中的合成测试通过与真实设备验证分开报告；功能实现、无损记录和实机控制不能互相替代为完成依据。
