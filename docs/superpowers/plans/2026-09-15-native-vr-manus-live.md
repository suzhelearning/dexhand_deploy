# C++ VR + Manus 联合现场接线

用户确认继续补齐并进行真实输入→MuJoCo 验收。顺序实施，不提交、不修改驱动/SDK，
不涉及真机执行器。保留 Python 默认入口；在端到端离线通过前不解除原生 CLI 的 arms-only 限制。

## 实施与验证顺序

- [ ] 驱动 stdout → 原生有界行接收 → 现有 Manus C++ 解析器 → 带身份、接收时间、
  回调序号和源序号的 HandSampleEnvelope → 同一 NativeHandPipeline/SessionRuntime。
  原始行与回调审计单独保留，EOF/超长/输出队列失败显式终止，不能重连后冒用旧 generation。
  - [x] `ManusIngress` 保留原始行与 SDK metadata，字节分片/左右手语义与 Python 对照。
    `ManusReceiver` 独占已转交的 stdout fd，不负责设备进程或 SDK；停止、EOF、截断验证。
  - [x] 两套 MuJoCo 模型上，rawviz 文本夹具经过管道/原生接收/解析/实际 Hand2 子进程，
    接入主会话并通过 Home/rearm/新输入恢复。该测试的机械臂仍为数值夹具。
  - [x] `SessionRuntime::submit_manus` 同时接收原始行与可选控制输入；独立有界原始行队列，
    经 `SessionOutputBridge` 分发到录制器，不因无回调而丢弃文本行。容量不足拒绝并故障；
    两模型的真实 Hand2 子进程离线用例验证逐行排空，另测溢出不标记完整。
  - [x] 冷启动 `NativeManusProcess`：显式启动原 rawviz 命令，stdout 经 `pass_fds`
    转交后关闭父进程读取端，Python 不逐行读取；保留子进程存活检查和有界退出清理。
    离线子进程测试覆盖继承管道、重复转交拒绝、异常退出和启动失败；尚未接现场 CLI。
  - [x] 网关新增可选 `NativeInputHooks`，主会话启动后开始输入接收，退出先停输入再排空。
    部分启动失败仍清理，输入停止异常导致输出不完整，不冒报成功。默认无钩子保持旧路线。
  - [x] 网关实现可选 `hand_runtime` 构造分支：HandPipeline/带手模型/ManusReceiver
    接入同一主会话。`HandLaunchConfig` 校验双方限位、身份、容量、时间窗、worker argv
    和独占只读 pipe；混合 Python 消费者或描述符冲突拒绝。
    本项只完成实现、构建和配置边界验证，完整网关正向联合运行尚未验收。
  - [x] `NativeGatewayProcess` 支持显式继承 Manus fd，并严格要求 `hands=cpp` 能力标记；
    缺少标记的旧二进制拒绝。此参数仍未从公开现场 CLI 启用。
  - [ ] 现场 CLI 组装实际手部参数/authority、转交真实管道和完整联合输入验收。
- [ ] 原生手部命令、反馈、处理结果及原始输入的发布、HDF5、Viewer 完整消费者。
  对照 Python 字段/左右手顺序/关节名/时间轴；队列排空后才允许完整标记。
  - [x] `SessionHandPublication` 命令/反馈协议与 Python 逐字段对照；通过可选
    `hand_authorities` 接到 `SessionPublication`，未配置时机械臂输出不变。
    校验手臂与手部 router 一致；无实际命令时不生成新命令。
  - [x] `SessionCycleRecording` 写入实际手部命令、反馈及 `accepted_hand_commands`；
    两模型、四种阶段与 Python HDF5 比对（数据、属性、类型、审计 JSON）。
    单侧更新不伪造另一侧命令，未知速度保留 NaN/valid=false，机械臂原有记录不变。
  - [x] `ViewerState` → `ViewerWindow` → 独立 `RenderModel` 传递手部反馈；
    支持两套模型的规范名/旧 finger 别名，40 关节完整校验后更新，缺少更新保持显示。
    双模型完整 qpos/FK 对照通过；不共享控制线程的 MuJoCo 数据。
  - [x] `encode_manus_ingress_columns` 与 `SessionRecordingSink` 的原始 Manus 写入接口：
    保留所有完整 rawviz 行、归一化回调及 SDK 序号/时间戳；沿用 schema 1.2，
    126 维数组明确为 right21-left21，不冒充原始 25 点。真实 C++ HDF5 写入与 Python
    文件的全部字段/属性/类型对照通过；非法行拒绝，未配置来源拒绝，失败后禁止完整关闭。
    已接入主会话有界队列及输出分发；真实 rawviz 现场管道尚未接入网关。
  - [x] 主会话捕获每次弹出的 native HandCommand 和 domain 接受/拒绝结果，保存接收时间、
    run/producer 身份、输入/输出序号、epoch、phase/status/valid flags 与完整 40 关节值。
    启用手部录制时写入 `native_cycle.native_hand_results`，与实际执行的
    `accepted_hand_commands` 区分；机械臂单独运行不新增此字段。
  - [ ] 手部状态/接收处理诊断和最终消费者接线。
- [ ] 冷启动绑定 rawviz、校准文件、worker 和限位；统一 arm/hand authority、Home/
  自动及手动 rearm。增加显式手部能力握手，老二进制拒绝，不静默降级。
  - [x] `build_hand_manifest` 复用当前 `wuji_hand2.yaml` 限位、零位与容差，并使用
    仓库原生 scheduler 启动器；构造 Manus/手部 producer/左右执行器身份和显式 glove 绑定。
    不创建新的硬件驱动，也不使用其他工程的绝对路径。已接入现场 runner。
  - [x] 原生主会话允许拥有手部 pipeline 的联合自动 Home/rearm；仍调用同一双 worker
    reset 事务、检查双方 Home 和 epoch，并要求新输入后 s。两模型上使用真实 Hand2
    子进程及夹具机械臂 IK/reset 验证自动重置后仍可手动 r，不把夹具当作真实 IK 联合验收。
  - [x] 现场 runner 组装参数、声明手部 authority、转交管道并监督原 rawviz 子进程。
- [ ] 两模型离线组合、断流、重置/退出、录制对照，再用真实 PICO+VR+Manus 做 MuJoCo
  验收。只有已完成的部分写入 README 可执行命令，不把内部接口当成现场入口。

## 2026-09-15 本轮离线验证

- 录制、手部发布、Viewer 缓存/窗口边界、退出和两条 CLI 回归：58 项，57 通过、
  1 项可选完整进程测试跳过。并未启动真实设备。
- 渲染模型：3 项通过，涵盖两模型有/无手部反馈、无新反馈保持、非法值原子拒绝和线程所有权。
- 同一渲染测试启用 UBSan 后 3 项通过；原生 session gateway 已重新构建成功；
  `git diff --check` 通过。
- 这些结果不代表完整 C++ Manus 现场路线验收通过；上面未勾选的接线仍需完成。

### 原始 Manus 录制接口补齐后的回归

- Manus 编码/录制器、原生接收/解析/Hand2 联合离线流程、周期 HDF5、发布、退出及两路
  CLI：67 项，66 通过、1 项可选测试跳过。
- 原生 session gateway 重新构建成功，`git diff --check` 通过。
- 新增 Manus 录制编码与录制器测试启用 AddressSanitizer/UBSan：2 项通过。
- 未启动设备、未改变驱动、未解除 C++ 现场入口的 hands-disabled 门控、未提交代码。

### worker 审计及主会话 Manus 队列接线后的回归

- 原生 Hand2/主会话、HDF5、输出/网关、退出及两路 CLI：61 项，60 通过、
  1 项可选测试跳过。Hand2 联合用例包含两模型及新增原始队列溢出场景。
- Hand2/主会话用例启用 UBSan：2 项通过。原生网关构建成功，差异格式检查通过。
- 尚未进行真实输入联合测试；下一步为 rawviz fd 冷启动转交、网关手部配置/能力握手、
  自动及手动 Home/rearm 联合接线，之后才能解除现场入口门控。

### 冷启动管道转交与网关输入生命周期

- 原生/旧 Manus 进程管理、网关正常及输入启动/停止失败、适配器、退出及两路 CLI：
  63 项，62 通过、1 项可选测试跳过。网关生命周期夹具重复验证 5 轮。
- 原生网关构建成功，差异格式检查通过；未启动真实设备、未修改驱动、未提交。
- 本轮为可测试的内部接线接口，不是公开联合入口完成。仍需实际手部 manifest、
  worker/receiver 构造与能力握手，以及联合 Home/rearm 验证。

### 网关手部构造分支与能力握手

- 原生网关已加入手部配置解析与实际 worker/receiver 构造分支，构建成功。
- 配置边界、冷启动、发布、握手、网关生命周期、退出及两路 CLI：73 项，72 通过、
  1 项可选测试跳过。拒绝旧握手、写端 pipe、重复 fd 和非原生消费者配置。
- hands-disabled 公共门控仍保留。手部网关暂保留显式 r 重置，未启用联合自动 rearm；
  下一步验证该事务并接入现场启动器，不能把编译及拒绝路径测试当作全链路设备验收。

### 联合自动 rearm 与手部启动参数组装

- 已替代上阶段“手部网关不启用自动 rearm”的临时限制：网关使用共同的自动 rearm 入口。
- 参数/能力握手、机械臂重置、网关、退出和两条 CLI 回归：53 项，52 通过、1 项跳过。
- 联合 Hand2 用例启用 UBSan：2 项通过（两模型，包含自动/手动重置）；网关构建与
  差异格式检查通过。尚未运行真实设备或完整原生 IK+Manus 网关联合验收。
- 公共 CLI 的 hands-disabled 门控仍保留，尚待现场 runner 调用已完成的接口并验收；
  未修改驱动，未提交代码。

### 原生联合入口与完整离线测试（本次续作）

- runner 已接入 Manus 原始 stdout 独占转交、八个 authority 声明、原 rawviz 生命周期监督，
  失败时反序关闭网关和原驱动进程。公开入口要求显式原生手部 scheduler、发布、Viewer、
  录制和双手；原 Python/PICO2 默认保持不变。README 已增加实验命令。
- 全网关测试暴露并修复了两处组件测试未覆盖的问题：阶段切换前的旧手部待处理队列拖延
  接管；联合逐条录制消费不及输入导致有界队列溢出。旧阶段队列清除仍受原阶段屏障保护；
  idle 接管要求处理赶上已接收的双手输入，teleop 新鲜度阈值不变。
- 联合 HDF5 按列合并写入，批次上限 64 项、4 MiB 触发刷新，持续流约 20 ms 刷新一次，
  关闭前排空。保留逐数据集 FIFO、原始行和重复属性一致性，单独机械臂仍使用原即时写入。
  添加退出状态/原因 stderr，以便输出通道自身失败时仍能定位。
- `NATIVE_JOINT_GATEWAY_TEST=1` 图形离线测试使用独立临时 router、合成 TJVR/rawviz，
  实际运行两种原生 IK、Hand2、MuJoCo、发布和录制。包含 s→h→自动 rearm→s→h→
  手动 r→s→q，验证原始 Manus、双手命令及 worker 审计，退出码 0 和 HDF5 complete。
  最近该组合测试通过（47.646 s）；不使用真实设备，也不构成硬件验收或性能等价证明。
- 真实 PICO＋VR＋Manus 输入验收、较长时间稳定性、同一录制 Python/C++ 端到端性能对照
  仍未完成，不能标记整个方案全部完成。
- 最终回归：原生 runner/参数、两路 CLI、HDF5、手部状态域和 pipeline 共 65 项，
  64 通过、1 项可选测试跳过；手部发布、启动握手、Manus 进程管理另 7 项通过。
  完整图形联合测试再次通过（48.480 s，两种 IK）；Shell 语法与差异格式检查通过。
  检查未发现本次合成输入/网关/手部 worker 残留。未启动真实设备，未提交或 push。

### 真实 Manus 高回调速率接管失败修复

- 现场失败录制 `native_joint_20260915_021946_696028042.h5`：7026 个回调，约 234 Hz；
  5999 个结果中 5744 个过期，年龄中位数 2.228 s、最大 4.382 s。录制为 incomplete，
  不作为正常退出证据。原生 IPC 前置 FIFO 一帧一答限制到 200 Hz，抵消了 scheduler
  自身的最新等待帧语义；idle 要求结果追到最新输入的条件也会在持续输入下饥饿。
- 只合并尚未处理的相邻 sample 作业，不影响在途求解、phase/reset 作业及完整原始录制。
  非有限输入在合并前拒绝。idle 采用 min(age/4,50 ms) 的结果年龄预算（最小 1 ns），
  保留 teleop 的原 200 ms 新鲜度门控；替代上述精确追平条件。
- 双手 150 Hz/侧合成输入先复现启动超时；修复后两 IK 完整图形联合测试通过
  （45.129 s），检查计算序号跳帧但原始回调记录数不减少。状态域/worker 5 项通过；
  两路 CLI、runner 退出和录制回归 48 项中 47 通过、1 项跳过。原生网关重建成功。
- 已重新启动真实 PICO＋VR＋Manus 的 SPARK C++ 仿真：run_id
  `d779db59-035e-4a96-8b5d-4f7167f3cdcf`，收到 `start accepted=true`。
  此为现场接管验证，尚不代表长时间稳定性或退出录制验收完成；所有硬件驱动保持不变。
