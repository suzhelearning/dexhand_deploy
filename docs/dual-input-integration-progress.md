# 双输入接入进度（2026-09-13）

当前目标已收敛为两条路线：**PICO2裸手**与**PICO头显＋双VR手柄＋Manus**。两条路线均已有受管实时仿真入口。PICO2 当前分支已完成真实设备人工试运行；原版 PICO＋VR 手柄链路已完成真实设备人工遥操；当前分支内嵌入口已完成启动及机械臂动作冒烟。Tracker 不属于本阶段目标。本轮工作区未重新接入真实 PICO/Manus，因此完整带 Manus 的当前分支 H5 验收、模式切换压力和长期运行仍未完成。

本轮最新验证：**990项 Python 测试中935项执行通过、55项按环境条件跳过**；PICO2官方手受管 smoke（含高度标定失败/重试、显式手势START、追踪丢失保持/恢复、双overlay）和 XR 控制器＋Manus→MuJoCo 全链路 smoke 均返回`passed=true`。内嵌 PICO bundle 的 ROS2 CTest 为 **29/29 通过**。控制器-only夹具明确模拟零 Tracker，仍验证双臂、官方Hand2双手和Home回位。本轮覆盖 XR 连接代次/重连安全、raw HDF5代次记录、控制器事件代次切换、运行中目标参考拒绝、SDK API预检/原生库预加载/释放、controller-only 无 Tracker API 预检和运行时无 Tracker API 采集、raw XR录制核验、XR操作观察审计、publisher身份接线、只读 XR 设备探针、控制器-only无 Tracker 探针、126-float Manus callback 兼容、Manus rawviz 工作目录固定及 XR+Manus 目标层离线 smoke，以及本轮共享设备路由锁、异常孤儿进程恢复、冲突清理隔离、PID 身份保护和多设备 ADB 绑定回归；PICO2 受保护链路未改动。此前的 PICO2 和原版 PICO＋VR 手柄人工测试另作为现场反馈记录，不与本轮软件回归混称。上述软件结果不替代完整设备验收。

本轮新增 XR 运行时部署能力：从用户指定的 Pybind 源码和 `libPXREARobotSDK.so` 编译
`xrobotoolkit_sdk` 到 Git 忽略的 `vendor/xr_sdk`，`vr_manus_xr_sim` 自动发现并只向
XR 观察进程注入 Python/库路径；新增 XR 专用 `adb reverse`（60061/63901），不改动
PICO2 的 10002 forward。当前机器已完成同源 x86_64/Python 3.10 扩展编译和
controller-only API 导入校验，并迁移旧机同版本 PC-Service 到 Git 忽略的
`vendor/xr_pc_service`；新增 `xr-service --check`/前台启动入口。真实设备探针与验收尚未执行。

实物测试按用户安排暂缓。本轮未重新接入设备，仅完成软件回归与验收准备；此前已有人工作动验证，但尚未形成当前分支带 Manus 的完整 H5 验收记录。操作单见[真实输入设备→仿真验收](real-input-simulation-acceptance.md)。

## 当前交付边界（优先于下方历史记录）

| 项目 | 当前状态 | 尚缺内容 |
|---|---|---|
| PICO2裸手→v131双臂+官方Hand2仿真 | 受管入口、合成闭环及当前分支真实 PICO2 人工试运行已完成 | 正式 H5 验收、长期负载和更完整手型记录 |
| 内嵌原版 PICO wholebody→M0→TJVR→SPARK/Hand2 仿真 | 源码 bundle、独立构建入口、分阶段 readiness、受管 supervisor 已实现；已完成启动及机械臂动作人工冒烟 | 当前分支带 Manus 手套的完整实机 H5、两路切换压力和长期运行 |
| 历史 `vr_manus_sim` TJVR/Tracker+Manus→SPARK+官方Hand2 | 保留旧数据/测试兼容 | 不属于当前目标，不安排本轮设备验收 |
| 停止后切换 | 共享设备路由 guard、过期 guard 恢复、旧 PICO2 profile 互斥和内嵌孤儿进程回收已有回归覆盖 | 带真实输入的反复切换压力验证 |
| PICO手势 | 观察/录制和显式双手张开START可用 | 真实手型质量、其余操作绑定 |
| XR控制器输入 | 控制器-only（无 Tracker）受管采集、增量/离合、控制器事件及 raw 录制已接通；本机 SDK/PC-Service 运行时可检查/启动 | 控制器标定、真实设备采样与验收 |
| SPARK外层额外平滑 | reference_direct可用 | processed_guarded监督及可信恢复；当前明确拒绝启用 |
| 两种新真机profile | 保持禁止 | 部署实现、设备/反馈/保护参数与实际验收 |

运行资产摘要是启动预检快照，不是运行中文件防篡改证明。VR新增flat MJCF实际网格引用闭包，连同原有模型/URDF/配置/worker/手套标定摘要进入resolved配置及录制；外部PICO_tracker当前实际加载的标定仍标`external_not_verified`，不能用已取回的历史YAML冒充。

本轮已从参考机器迁移同版本 x86_64 PC-Service 运行时到 Git 忽略目录，并通过前台包装器
完成文件和启动入口检查；尚未启动真实 XR SDK/PC-Service 采样，控制器标定和真实设备验收仍未完成。因此只把 XR 控制器路线的软件边界标为已接通，不把真实设备验收描述成已完成。

## 已打通的链路

### 真实输入验收准备增量

修复参考Manus `rawviz.out` 在未配置SDK搜索路径时预检可过、启动却找不到`libManusSDK_Integrated.so`的问题。自动发现可执行文件旁的`ManusSDK/lib`，或通过`--manus-library-dir`显式指定；只修改rawviz子进程环境，保留原继承路径，不污染协调器、原生IK或官方手worker。预检记录SDK文件摘要；显式路径缺库立即拒绝。当前受管环境下`ldd`已确认依赖可解析，但没有实际启动SDK采集，不代表手套在线或标定正确。

新增`scripts/probe_teleop_input.py --mode pico|tjvr|manus`，仅接收并报告双侧新帧数、新鲜度和最大间隔，不连接router或执行器、不授权遥操。沿用原协议解析和TJVR门控，PICO重复源时间戳、Manus缓存另一侧不刷新计数。探测器临时独占输入，使用前须停止会话，探测后须退出再启动会话；通过只表示输入活性，不验证标定、设备身份或算法映射。

已新增包含ADB、逐路探测、机械臂优先/双手后续、录制与只读核验步骤的操作单。19项定向测试通过（22.760秒），随后802项全量和9项CTest通过；均为本机合成输入或离线数据。旧PICO控制默认值未改动；新real profile仍禁止。未暂存、提交或push。

### XR 原始输入可视化增量（9月10日）

为现场联调增加独立的 `--xr-overlay`。它只订阅 `raw/xr_input`，在 MuJoCo Viewer 的平移诊断区域显示 HMD 和左右控制器；兼容SDK若返回Tracker也只作被动诊断。每帧校验 router/receiver 身份、序号和 500ms 新鲜度，拒绝的帧只进入状态诊断。该 overlay 不写入 qpos、不生成目标、不参与 IK/手 retarget，也不改变已有 `--pico-overlay` 或 `pico2_hands_sim` 的默认参数。`--xr-sdk-pythonpath` 仍只注入 XR 采集子进程，避免把外部 SDK 路径传给旧 PICO2、协调器或执行器。

XR 入口/可视化定向测试已通过；新增连接代次合同：PC-Service 每次重连从新代次/序号开始，运行中的代次变化会让目标桥接要求重新启动，启动前的重连则替换旧参考。新增 `check_xr_sdk.py` 启动前无副作用 API 预检，并让 XR 客户端关闭时释放已加载 SDK（兼容没有 close hook 的旧绑定）。采集边界同时兼容参考 Pybind 返回的 list 与兼容实现返回的 NumPy/可迭代 Tracker 容器，不用数组真值判断。完整回归中的参考闭包哈希曾因误改只读参考资产而失败，已恢复参考副本原文；修正后872项 Python 测试通过、51项跳过。真实 XR SDK、PC-Service、Tracker/控制器标定和设备验收仍未执行。

新增独立只读 `check_dual_recording.py --mode xr`，逐帧核对 `raw/xr_input` 的 HMD、控制器、按键、源时钟有效位/数值、连接代次/序号以及 HDF5 展平列和完整 `XrFrame` JSON；兼容字段中的Tracker只做被动一致性检查，不是当前路线必需输入。同时核验已录制的 XR 操作观察审计（路由、publisher、动作、代次和序号），并要求 schema 1.2 的完整 JSON 显式保存连接代次。不初始化 SDK、不连接 router、不执行操作事件、IK 或 retarget。相关录制检查测试通过，仍不代替真实设备验收。

新增 `scripts/probe_teleop_input.py --mode xr` 只读设备探针。它复用当前 XR
binding 配置和 `XrRoboToolkitSource`，启动前按所选输入模式检查 Pybind 必需 API，随后只读取 HMD、
双控制器以及兼容配置中的Tracker，在 JSON 中报告各信号有效帧数与 200ms 新鲜度；
不启动 router、Manus、IK、retarget 或任何机器人命令。SDK 路径只存在于探针子进程；
缺 SDK、PC-Service 或控制器数据时返回失败；只有显式选择兼容 `xr_tracker` 模式时才要求绑定 Tracker，
便于进入仿真前定位设备侧问题。当前默认 `xr_controller` 模式不把 Tracker 当作必需输入。
该探针不替代标定、映射和真机安全验收，也不改变 `pico2_hands_sim`。

本轮补齐参考 `install_sdk.sh` 将 Python 扩展与`libPXREARobotSDK.so`分开安装的部署
路径：新增 `--xr-sdk-library-dir` / `TIANJI_XR_SDK_LIBRARY_DIR`，在导入 Pybind
扩展前用绝对路径预加载 companion library，并把该目录的`LD_LIBRARY_PATH`只注入 XR
采集子进程；PICO2、IK、coordinator、Manus worker 和 MuJoCo 不接收该变量。缺库或
动态库加载失败会在创建 router/受管进程前失败。该补丁的定向测试和全量回归均已通过，
但当前机器仍没有实际 XR SDK/PC-Service 设备验收条件。

### XR+Manus 目标层离线 smoke 增量（9月10日）

新增 `scripts/xr_manus_sim_smoke.py` 及对应的纯内存测试，默认用 `xr_controller` 绑定
生成合成 XR 帧；显式 `xr_tracker` 仅保留兼容回归。同时按参考 `HandInputAssembler` 的实际扁平
126-float 形状生成 Manus callback。两路数据经过与 `vr_manus_xr_sim` 相同的
canonical observation 发布、显式 controller start 边沿、`xr_incremental` 映射和
目标桥，验证未 start 时目标数为零、start 只触发一次、双臂和双手目标持续输出。
该 smoke 不启动 SDK、PC-Service、rawviz、router、IK 或机器人命令；报告明确保留
`robot_commands_enabled=false` 和 `hardware_acceptance_complete=false`，只补足
输入到目标层的离线接线证据。与此同时修正 `manus_callback_observations()`，兼容
参考 assembler 的 flat callback 与已有 `(42,3)` 测试形状；PICO2 路径不经过该分支。

### XR 操作观察审计增量（9月10日）

`vr_manus_xr_sim` 的 recorder 现在只订阅一份 `raw/manus_callback`，并额外保存控制器发布的 `tianji/observation/operator/xr` 完整 envelope 到 `meta/dual_audit` 的 `operator_observation` 行。录制器会校验 router、publisher 身份和操作观察字段；离线 XR 核验会检查动作集合、代次/序号递增、重复观察和 raw 代次关联，但仍不会执行 start/Home/clutch。运行时 publisher 使用受管会话实际分配的观察实例 ID，避免目标侧把合法控制器观察误判为外部发布者。该修正避免同一 Manus 回调被重复订阅写入，也不改变 PICO2 录制布局或会话控制。

### M6 reset回执与执行代次审计增量

新增显式`check_dual_recording.py --mode tjvr --check-native-resets --input PATH`，先完成既有原包/门控/消费帧核验，再只读检查reset审计。遵循当前受管VR执行代次从1开始的合同；接受的rearm必须有对应14关节有限数值回执，速度/加速度为零，代次连续递增且后续native cycle采用该代次。缺失/重复/错误代次、错误运行身份、非零导数或不完整回执均失败；拒绝的rearm不推进代次。其他run的回执不计入、不用于离线代次推进。

默认检查及运行时控制代码不变。明确不验证ack位置等于真实反馈、不证明Home/静止，也不推断未录制的内部新鲜输入截止时刻；不执行reset、授权或设备动作。缺少native消费边界的旧文件拒绝严格检查，M6完整状态回放仍未完成。

20项定向测试通过（22.034秒），包含真实UDP接收线程、原生SPARK仿真及Home/rearm后重新启动的实际录制核验；离线CLI输入文件SHA保持不变。随后793项全量通过。顺序异常检查复现了超出float64范围的JSON整数导致`math.isfinite`抛异常的问题，补范围检查后返回差异报告；20项定向复测通过（21.729秒），原生CTest 9/9。未改变控制逻辑或默认检查行为。

### M6 SPARK消费帧离线关联增量

在TJVR门控重建通过后，复用已有`native_cycle.native_attempt.sample`核验实际记录的消费边界，运行时采集/producer/IK均不修改。不按最近时间戳对齐：消费帧必须对应已通过门控且在其之前记录的原包，逐字段核验原接收者/序号/时间、二进制包、discontinuity及重同步代数。拒绝消费被门控拒绝的帧、重复/倒序消费、tick缺口/执行代次回退、未来输入或调用晚于审计、运行身份变化和不完整attempt。

没有新输入的tick保留`sample=null`；rearm后的tick可以从1重启，但输入接收序号不能重置。`native_input_check`分别报告`checked`、`no_native_attempts`、`not_recorded`和`unavailable_legacy_audit`，不将旧文件缺消费边界当作已验证。仍不证明实时调度选择了理论最新帧，不重放native dynamics、Home/reset、授权或执行器，M6整体未完成。

实际SPARK/官方手及UDP跨Home/rearm联合录制9项通过（32.966秒）；补齐跨审计边界顺序检查后，12项录制定向测试通过。最终786项全量通过、无跳过（320.998秒），原生CTest 9/9。运行时代码未改动，未暂存/提交/push，未启动真机。

### M6 TJVR门控重建增量

新VR实时入口在开启录制时额外保存每个已解码TJVR原包的门控决策（含被拒绝的重复包和跳变候选），以及原接收序号/时间、epoch变化和重同步代数。门控参数仍为0.15m/0.6rad；resolved配置保存实际使用阈值与reset初态。可选审计hook默认关闭，不修改旧接收路径、门控算法或最新帧消费规则；有界入队失败沿现有UDP错误锁存停止接收。

新增`check_dual_recording.py --mode tjvr --input PATH`，仅用录制原包及显式门控合同重建并逐条精确核验，不连接router、不启动worker或执行事件。缺少合同/审计的旧HDF5不猜测历史；接收序号必须递增但允许坏包导致的空缺，源协议序号重复仍按原版门控处理。报告拒绝缺失、乱序、运行身份/时钟/布尔类型篡改。它不能证明raw与对应审计一起被删除时的完整性，也不重建控制线程实际消费的最新帧、映射或IK状态，不代表M6整体完成。

实际SPARK＋官方手单左手/双手联合录制已加入门控重建，相关24项测试通过（32.558秒）；随后增加旧/错误合同拒绝用例及实际UDP进程录制重建断言，后续验证结果另记。

最终781项全量通过；另8项实测覆盖实际UDP接收线程、SPARK原生仿真、显式start/Home/rearm及新鲜输入后再次start的录制门控重建。该测试确认输入流跨执行epoch不被误reset；没有重放这些操作，也不表示完整授权状态机已能离线复现。未启动真实设备，未提交代码。

### M6离线重建增量（9月9日）

新增只读 `scripts/check_dual_recording.py --mode pico|manus --input PATH`，不连接router、不调用执行器、不执行START/回Home等审计事件。只接受完整HDF5；缺失、重复、额外关联及数值/元数据差异不会静默覆盖或插值对齐。

- PICO：重新解码wire bytes，核验raw解码字段，再重建左右手21点和头相对手腕观测；原始文件SHA不变。`/tmp/pico-sim-smoke-hxo2tojc/session.h5`实测586帧、2344条观测、差值0。旧raw组没有独立原始接收时钟，因此对该文件仅交叉核验关联观测的接收时间；报告明确保留此限制。
- 后续补齐：新schema 1.2 raw增加`received_timestamp_ns`，保存输入进程原始接收时间，原`time_ns`仍是recorder时间轴；schema 1.1布局不变，旧1.2缺字段仍可读。新文件可直接核验raw→观测时钟，且能重建被动手势滞回标签；不执行手势动作。相关28项定向测试通过。
- Manus：原版rawviz parser/assembler按记录的行顺序、接收时间、手套绑定重建每次回调，比较right→left顺序、63/126维、源序号/时间与callback metadata。新VR解析配置增加`manus_input_contract`，随现有resolved配置写入HDF5；不改变采集/控制算法。无绑定配置的旧文件拒绝猜测。
- 定向13项测试通过；另外显式启用官方手worker和SPARK实际时钟的联合录制/重建测试通过（1项，4.584秒）。首次仅启用SPARK的运行跳过了官方手测试，不作为这条链路通过的证据。
- 后续新增Manus手关节数值重建：显式`--mode manus --retarget-hand-commands`先核验rawviz重建及已记录的官方手资产摘要，再启动隔离worker，按原回调/接收时间推进（含idle回调），对照已有手命令，关节容差1e-5 rad。没有授权重放，也不从录制启动执行器。实际rawviz进程→官方手→SPARK仿真联合录制→独立worker重建通过；相关5项6.450秒。后续异常/资产检查补齐，纯单测不依赖可选原生环境。
- 后续补齐PICO手worker消费历史：仅新受管PICO手组件发布`pico_consumed`被动审计，记录处理序号、实际raw关联、正序号转换和有效侧；共用retarget loop的可选hook默认关闭，旧入口不启用。新1.2 recorder保存为`hand_retarget_input`；启用手时额外保存官方手及PICO适配源资产摘要，禁用手不要求这些资产。
- PICO显式手命令重建按消费审计而不是全部raw推进独立左右worker，消费记录起点/顺序、raw时间、producer身份、资产或命令关联缺失均拒绝，不推测原worker初态。`/tmp/pico-sim-smoke-56_liiy7/session.h5`实际583帧raw/消费帧、2332条观测、583条手势、625条手命令重建差值均0；包含单/双侧跟踪丢失和恢复。此样本消费帧数恰与raw相同，不代替订阅前raw的边界测试；独立单测专门验证raw前缀不进入worker。新PICO官方手smoke已自动调用此核验，旧模式不调用。
- 仍没有重建完整目标映射、SPARK全部内部状态或实际执行/授权决策。M6整体和M8仍未标记完成。

### M8分阶段比较入口增量

新增`compare_dual_input_reference.py`，读取已有原版/迁移版SPARK JSONL及官方手JSON轨迹，逐阶段汇总。复用SPARK原比较器；手部保留1e-5 rad固定容差并校验输入/配置/bridge摘要、输出checksum、40关节顺序、回调时间和首个分歧关节。CLI返回0/1/2表示提供阶段一致/差异/输入错误，不启动算法或设备。

仅手或仅SPARK可单独检查，也可一起提供；报告固定保留`complete_plan_acceptance=false`和`synchronized_inputs_verified=false`。不将独立TJVR/Manus录制拼成同步输入；XR raw 录制已覆盖软件采集边界，但不声称实际加载标定、SDK版本和真机通过。5项工具测试通过，覆盖一个阶段失败而另一个成功的报告及数值类型/摘要篡改。

### 本轮全量中的测试驱动修正

740项快照有1项失败：同router切换中的PICO手势授权超时。失败文件`/tmp/pico-sim-smoke-ctqoz6xn/session.h5`有1499帧raw及1499条手势记录，全部为open，没有任何握拳输入；因此不是授权条件被错误拒绝。测试驱动用0.3秒墙钟Event脉冲触发握拳，调度延迟可令TCP feeder错过整个脉冲。改为等接收端确认三帧新鲜、连续握拳后再张开；只改合成smoke，不改生产手势/授权阈值。修正后同router切换2项通过，49.357秒；全量重新执行中。

后续747项快照全部通过、无跳过（276.504秒），包含原版1339帧手对照差值0；新增M8汇总工具及Manus关节重建发生在其后，不能冒充已包含。`/tmp/pico-sim-smoke-0fvz736v/session.h5`离线检查454帧raw/独立接收时钟、1816条观测、454条被动手势全部差值0。M8新手比较器另实际读取历史原版与当前worker1339帧输出，最大关节差值0。受保护旧命令合成smoke重验通过`/tmp/pico-sim-smoke-jchouul2`，未改旧映射/IK/轨迹处理。

再后续759项固定快照全量通过、无跳过（283.491秒），官方1339帧对照仍为0；之后的PICO消费边界和手命令重建另做上述实测，最新全量重新执行中。未暂存、提交或push。

加入PICO消费/手命令重建后的767项固定快照全量通过、无跳过（291.907秒），官方1339帧对照仍为0；自动smoke已包含真实离线worker重建。9项原生CTest再次全部通过。

### VR单侧手配置修正

顺序自查发现：配置校验允许`active_hand_sides: [left]`，但live worker仍使用默认`single_hand_side=right`，录制合同又硬写双手。已让worker显式采用实际侧别，让录制按原版right→left顺序保存实际启用侧别；默认双手126维及双臂SPARK不改变。实际rawviz子进程→单左手官方worker→SPARK/MuJoCo→HDF5→独立手命令重建通过，确认无右手命令；同一测试夹具的双手路径也通过（2项，11.576秒）。本修正只涉及新VR配置；PICO仍按已有合同要求双手或全部禁用。

### VR手套绑定预检与录制合同一致性

新VR入口此前允许显式空字符串绑定，但录制重建合同要求绑定为`None`或非空字符串。现于解析参数后、资产预检前拒绝空字符串及纯空白绑定，提示省略参数以使用原版自动侧别识别；不裁剪、更改有效ID，也不把原版侧别覆盖规则改成设备白名单。覆盖左右侧、启用/禁用手的参数用例；修正前空字符串用例实际错误返回0，修正后相关25项测试（含实际SPARK/官方手录制重建）通过。该修正不涉及旧PICO入口、映射、IK或平滑。

TJVR原包→原版stream gate→最新帧消费→独立原生SPARK→成对proposal→当前coordinator→MuJoCo。

独立Manus21回放→原版官方Hand2 bridge/Retargeter→仅关节名称转换→会话授权→MuJoCo。

- 两条算法的原版逐帧对照证据见`reference_equivalence_contract.md`、`spark-porting.md`、`wuji-hand-porting.md`。
- 双臂协调器回执是命令处置，不是设备反馈。超时、拒绝或直通输出被修改后锁存暂停；不以单侧结果推进双臂。
- 原版URDF左右臂限位不对称。新分侧配置只用于显式双臂仿真；原来的`robot/arm.yaml`及旧默认profile未更换。
- 原版初始关节角、模型、左右限位均用于离线仿真，未把新模型硬套进旧共享限位。
- 接收层有独立UDP线程、raw-before-gate和typed raw记录入口；已测试localhost数据包，未声称完成设备联调。
- 新双臂最终命令采用原子包，显式绑定run/epoch及原生tick。新MuJoCo接收器拒绝单侧命令混入；坏包会撤销尚未执行的成对命令。
- `SparkCoordinatorCycle`成为共同控制周期，离线回放已改用它；原生失败在同周期进入协调器健康检查，协调器每周期只tick一次。
- `SparkLiveSimulation`是共享控制核心：原版预算、显式六角色authority、启动握手不推进参考、键盘intent接口、所需双手就绪与实际执行状态检查。`live_runner.py`现已绑定UDP、独立Manus/手worker、路由器和终端/Viewer，`run_session.sh`通过受管子入口复用运行锁与清理。
- `ReferenceManusProcess`内置原版rawviz解析边界，独立接收线程保存每次左右手异步回调及各自源序号/时间；有界队列溢出明确失败，不静默合并回调。
- `HandRetargetLoop`独立于机械臂时钟处理实际回调；计算期间收到停止事件会在发布前重新应用，避免迟到手命令。SPARK与官方手worker均新增可选启动握手，原来的无握手调用仍兼容。
- 新`AuthorizedHandMujoco`仅供新模式显式使用，绑定手producer与短期会话授权；过期/故障保持实际手位，显式return在运动学仿真中归零。这不是已验证的真机回零轨迹，旧MuJoCo类默认行为未替换。

## 只读启动配置检查

```bash
pixi run bash scripts/run_session.sh --profile vr_manus_sim --disable-hands --resolve-only
pixi run bash scripts/run_session.sh --profile pico2_hands_sim --resolve-only
pixi run bash scripts/run_session.sh --profile pico_vr_manus_sim --disable-hands --resolve-only
```

## 新PICO2受管仿真入口（9月9日接通）

`run_session.sh --profile pico2_hands_sim --viewer --pico-overlay --ik-target-overlay`默认选择v131、relative_home、外层passthrough、URDF限位及官方Hand2双手。沿用旧target source的`s/q`键盘操作，不套用VR的`h/r`。可显式选择现有mapper/处理选项、`--disable-hands`、`--headless`、`--observation-config`及`--record`。旧profile不改变默认值。

独立contract `pico2_hands_sim.yaml`先解析，`pico2_hands_runtime.yaml`仅描述组件组合；不是改名启动旧profile。仅原observation负责TCP/ADB；运行期显式`--receiver-instance-id`绑定raw与官方手组件，不再误把YAML固定receiver ID当作进程实例ID。新target source只负责双臂，不发布/校验旧手target；独立官方producer处理raw。MuJoCo显式选择`AuthorizedHandMujoco`并注册左右手executor token，反馈来自显示中的同一模型；没有额外Wuji dry-run执行器。所有新实例仍由原受管launcher反序清理。

PICO已授权且TCP持续提供新帧时，短暂单侧/双侧视觉丢失保持实际手位，不喂零、不伪造命令、不推进失效侧官方滤波；恢复后原bridge按真实序号间断处理。手producer仅在当前授权、曾见有效双手且原始输入新鲜时允许保持，启动前仍要求有效双手。机械臂继续使用原有视觉丢失hold；断连、权限故障、处理故障仍中断。此为用户既定PICO仿真保持语义，未推广为真机策略。

录制由独立受管recorder进程使用schema 1.2，包含完整raw/pico_hand_tracking、手/臂命令与实际反馈、解析配置、run ID及官方PICO适配版本；没有Manus回调。该入口复用原独立recorder进程，并非VR入口的AsyncDualRecorder调度；不宣称两者磁盘调度相同。真实标定、完整算法资产摘要仍需补齐。

实际多进程合成验收：双臂禁用手通过`/tmp/pico-sim-smoke-ou6adm0c`；启用官方双手通过`/tmp/pico-sim-smoke-mxbf88tz`；加入完整reader/元数据检查后通过`/tmp/pico-sim-smoke-ttgm1xx0`；单手失效、双手失效、同映射恢复及断连通过`/tmp/pico-sim-smoke-tt44wj1o`。最后一项验证失效侧arm命令无漂移、实际hand反馈不变且没有伪造hand命令。旧受保护参数组合再次通过`/tmp/pico-sim-smoke-g93fqami`。

失败记录：`5z38rw0l`因raw receiver身份不匹配未就绪；`h3u1aslw`因新入口仍启用旧手target校验而退出遥操。分别修正显式身份绑定及职责分离后重验通过，没有放宽时钟/超时或替换原版IK。

两种新配置已存在并严格校验输入/IK/处理方式组合，只读解析不启动采集或路由器。两种已实现sim组合均可报告`runtime_available=true`，这只表示已接线，仍须通过资产/人员标定和输入就绪检查，不表示设备或真机已验收。旧profile分支不使用新解析器。

## 内嵌原版 PICO＋VR手柄＋Manus 仿真入口（2026-09-10）

新增 `pico_vr_manus_sim`，把参考 PICO_tracker 的 `pico_bridge`、M0 掌心/骨架修正和
TJVR bridge 以源码形式放入 `vendor/pico_tracker`，通过 `source_manifest.json` 保存
相对路径和 SHA-256。它使用单独的 Python 3.11/ROS2 Pixi manifest，不改变根环境，个人
标定仍从 `~/.config/pico_tracker` 或 `--pico-calibration-dir` 读取；源码和脚本不包含
开发机绝对 checkout 路径。

`scripts/run_embedded_pico_vr_session.sh` 负责 `adb/APK → driver → raw skeleton readiness
→ M0 → corrected skeleton readiness → TJVR readiness → 当前 vr_manus_sim downstream`，
子进程均为独立 process group，退出时反向清理。ADB 保留原版 `tcp:9999`，只删除本次
实际创建的 forward；PICO2 的 `tcp:10002`、official PICO2 producer、手转换和既有入口未
进入该分支。driver 阶段只要求 `/pico/smpl_raw`，M0 阶段再要求 `/pico/palm_left/right`
及 corrected/epoch 全量 topic，避免在 palm publisher 尚未启动时误报超时。

构建和只读解析：

```bash
pixi run build-embedded-pico
pixi run bash scripts/run_session.sh --profile pico_vr_manus_sim --disable-hands --resolve-only
```

先关闭手套验证双臂，再移除 `--disable-hands` 并提供 `--manus-rawviz`、`--manus-user`。
该入口最终仍使用原版 TJVR `robot_arm_segments`/0.95 reach 映射和当前工程已有的
reference-direct Spark、official Wuji2、coordinator、MuJoCo；不启用 XR SDK/PC-Service，
也不再要求 Tracker。具体设备步骤见[真实输入设备→仿真验收](real-input-simulation-acceptance.md)。

## VR+Manus实时仿真入口（设备验收待完成）

前置：路由器已运行、独立SPARK/官方手环境已构建；上游PICO_tracker继续提供原版修正上肢TJVR数据，默认发往本机UDP15000。它不是PICO2裸手TCP输入。跨机可显式配置`--tjvr-bind`/`--tjvr-port`，不默认暴露公网。

```bash
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh --profile vr_manus_sim --disable-hands --viewer

# 启用双手：必须指定原版rawviz可执行文件及人员标定，不自动选择操作者。
TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447 \
pixi run bash scripts/run_session.sh --profile vr_manus_sim --viewer \
  --manus-rawviz "${PICO_MANUS_TELEOP_ROOT:?set PICO_MANUS_TELEOP_ROOT}/manus/rawviz.out" \
  --manus-user gjy
```

rawviz按自身可执行文件目录读取`calibration/gjyLeftMetaglovePro.mcal`及右手文件，前置检查要求两份存在；不会扫描别的用户标定。PICO正式标定仍由外部PICO_tracker读取，当前入口还未自动采集该标定摘要。

- 终端及Viewer均支持`s`启动、`h`显式回Home、`r`在健康idle且实际双臂精确Home/双手归零后联合重置、`q`回Home后退出。重置后还需新接收帧及再次`s`；重建期间积压帧不能授权。旧epoch回执不能影响新状态。故障锁存或任意位置恢复仍须重开会话；这不是完整故障恢复策略。
- 可用`--headless`和有界`--duration-s`做无窗口检查；时间到会结束仿真，不是物理回零命令。
- 只有此新profile使用新执行类和SPARK；旧PICO2命令未改成SPARK。
- 当前live入口支持`--record /已有目录/新文件.h5`；明确拒绝覆盖已有文件，前置检查不创建文件。`--spark-overlay`独立显示TJVR修正骨架/掌心、包内目标及SPARK实际IK目标；原始过期骨架隐藏、历史IK目标标stale。使用参考世界坐标，不额外叠加Base_L/R。旧PICO/IK overlay和真机参数仍拒绝。
- 启动前资产/模式校验、运行锁和live domain检查；运行期新增冲突控制发布者/自身token丢失会锁存中断，允许被动recorder。
- 主环境Python/动态库覆盖不会传入两个固定依赖worker，不修改主推理环境变量。

## 可运行的离线命令

从工程根目录运行。需要已构建SPARK独立环境/worker及官方手部独立环境；这些命令不连接路由器、PICO、Manus或机器人。

```bash
pixi run python scripts/vr_manus_sim_smoke.py \
  --tjvr vendor/reference_inputs/remote-192.168.110.210-20260908/output_continuity_retest.tjvr \
  --disable-hands

pixi run python scripts/vr_manus_sim_smoke.py \
  --tjvr vendor/reference_inputs/remote-192.168.110.210-20260908/output_continuity_retest.tjvr \
  --manus-recording vendor/reference_inputs/remote-192.168.110.210-20260908/manus_gjy_dual_20260827.pkl
```

追加`--record /绝对路径/新的文件.h5`可记录，路径必须未存在。录制失败不会打印成功报告，文件不能标为完整会话。原始输入数据在git忽略的私有目录，其他机器需要自行提供，不会随提交传播人员标定/录制。

Manus与TJVR按各自时间线零点组合到固定5ms控制时钟，只是合成的异步联调安排，**不是同步硬件录制**。确定性测试放宽两个求解墙钟预算，不等于实时200Hz验收。

## HDF5 1.2

- 保留1.0/1.1原组及读写语义，不升级旧profile的默认版本。
- `raw/tjvr_upper_limb`保存完整二进制包、接收单调时间、receiver身份和原接收序号，包含gate拒绝的已解码包。原始独立有效位、方向、target和corrected palm都能由包重建。
- `raw/manus_callbacks`保存实际retarget输入回调；明确不是SDK raw25。原始SDK节点仍使用既有`raw/manus_hand_tracking`，没有数据就保持空，不伪造。
- 命令和实际MuJoCo关节状态仍写入现有`joint/command`、`joint/state`。元数据明确独立录制、模型名称与输入摘要；配置/模型/标定摘要的完整自动注入仍待接线。
- 新被动recorder在VR+Manus模式订阅`tianji/raw/tjvr_upper_limb`；PICO2模式拒绝该输入。记录不提供控制权限。
- VR受管live的录制使用有界队列、独立分发线程及spawn启动的HDF5写盘进程，不在控制线程等待磁盘，也不与采集线程共用HDF5写入的Python执行时间；溢出/写盘失败中断会话并标记不完整，不能静默丢帧。仍需进行长期吞吐量及200Hz周期验收。
- 可选`meta/dual_audit`保存原生SPARK完整输出字段、执行epoch、协调器回执/原子命令、各周期接纳的手命令、操作接受/拒绝、生命周期及rawviz原始文本行。`raw/manus_callbacks`记录每次实际解析回调，源左右序号/时钟另存callback metadata；没有把这些回调伪装成raw25。
- 记录的操作结果仅供审计，不会作为回放授权。旧1.2文件没有`dual_audit`仍可读取。resolved配置和本地已知算法资产摘要已随live记录；外部PICO_tracker实际生效标定仍标记`external_not_verified`，不能用本地候选文件代替。
- `native_cycle.native_attempt`记录该次原生求解实际消费的输入（无新帧为null），失败尝试也保留，不能仅凭接收时间推断控制线程消费顺序。Home重置的操作记录包含完整`reset_ack`。
- 手producer/执行器状态发布到现有状态topic，并写入`component_status`审计；包含处理回调数、输入时间、最大回调年龄和最大retarget耗时。读取诊断快照不额外推进状态序号，也不刷新原始输入时间。

## XR数学迁移边界

已固定并隔离移植参考`wuji_teleop`的增量控制、One Euro滤波、坐标转换及配置闭包，来源提交为`59028470b8be4abfdd052ff35a06ca73413d9590`。开启`XR_REFERENCE_TEST=1`可对独立原版Python进程做确定性对照：160个合成周期、1120项输出严格相等。

这首先证明数学路径与参考实现一致；在此基础上，当前工程已补齐 XR SDK 采集边界、显式设备绑定、离合、控制器 start/Home 事件、Manus callback 汇聚及 `vr_manus_xr_sim` runtime profile。仍未在本机安装/启动实际 XR SDK/PC-Service，因此不把软件接线通过等同于真实设备验收；不会加载原版机器人输出节点。

## PICO2官方手后端接入边界

新增`hand_tracking/official_pico.py`，只用于固定Manus Hand2 YAML的手骨架输入：PICO原始FLU右手系26点→原21点选择→显式Y反射。YAML原有Z反射与之组成正旋转，由官方腕部坐标估计消除；没有额外头部变换或腕位置归零，旧PICO转换器及机械臂映射未改。针对最近两份PICO录制暴露的PICO手部几何短于舞肌二代模型问题，官方worker边界新增绕腕点的PICO专用1.10径向尺度适配（`pico26_to_official_hand2_yflip_scale_v2`）；原始观测、overlay、HDF5和Manus路线不缩放。节点无效、手无效或腕/食指MCP/中指MCP平面数值退化时该侧无效；头显失效本身不使手部几何失效。保留PICO来源、关联ID、接收序号和源/接收时钟，不把PICO输入写成Manus回调。

新增`producers/pico_official_hand.py`，由调用方提供明确绑定左右侧的独立官方worker。有效侧调用原版63点单侧接口；无效侧不调用、不补零推进滤波，恢复时原版bridge可看到接收帧序号间断并reset。接收器/连接代次变化必须显式新建算法状态，不能静默继承；部分worker失败后不重试或输出部分结果。worker生命周期和会话授权仍归调用方，本模块没有执行权。

9项输入/后端定向测试通过，包括左右选择、元数据、失效、数值退化、官方坐标预处理的刚体不变性，以及“双手→左侧丢失→恢复”在独立侧worker和现有官方双手worker之间逐项一致。均为合成PICO几何，尚无本轮真实PICO手骨架/手型校准验收。

随后已补齐PICO手producer的公共会话授权、一次性命令和共享手线程接线。新增有界raw队列，消费现有ObservationRuntime的原始PICO topic，不创建第二个TCP/ADB接收器；核对原始包与解码字段一致，拒绝外来身份，连接代次变化和队列溢出锁定故障。`PicoHandService`绑定现有raw/session topic，发布标准手命令及producer状态；仅允许simulation，worker生命周期、域互斥与liveliness仍须由受管入口承担。

先前已提供独立本地Zenoh router→官方左右worker→授权MuJoCo手执行器的合成输入测试，该组件测试本身不代表机械臂/完整profile或硬件验收。异步录制器新增显式`pico2_hands_sim`来源与必填模型身份，独立写盘进程支持完整`append_raw_pico`，重读确认原始包、接收序号、26点几何和来源一致、不伪装成Manus回调；既有VR录制默认值不变。随后PICO runtime的受管启动与录制订阅已接通，见上面的多进程验收记录。

本轮29项定向验证首次失败：PICO手回调sequence=3进入处理时age=273092463ns，超过200000000ns门限；这是输入处理前拒绝，不是此前MuJoCo加载后命令交接过期。仅增加失败时完整producer状态诊断后，同一29项重跑通过（3.763秒），未修改门限、时钟、算法或队列策略。延迟来源尚未确认，不能以重跑通过宣称实时问题解决。

## 实时录制待查问题

### 9月9日后续：手组件生命周期与首次回调延迟

新增`PicoHandComponent`，接收受管入口提供的已校验router session、显式身份、连接代次和完整权限清单。启动时先检查控制域冲突，再加载左右官方worker，注册唯一手producer token并接入现有PICO raw/session订阅。退出先停止手命令发布和处理线程，再撤销token、关闭worker；部分启动失败也清理已拥有的资源。权限冲突或已绑定token丢失锁定故障，不自动重启worker。该组件不启动TCP/ADB、router或执行器，不能代替尚未完成的完整profile入口。

该轮53项相关回归失败：PICO第一次retarget事务226012377ns，下一输入年龄207595657ns；SPARK+Manus手命令年龄201123996ns。没有放宽门限。独立10帧实验复现PICO首帧235.388ms，后续约4.6–4.8ms。检查固定源码发现`Retargeter._apply_rotation`在首次回调内导入`scipy.spatial.transform.Rotation`，而worker已提前报告ready。

修复只在本工程worker的`--startup-handshake`路径提前加载该依赖；不改third_party原版文件、不喂假骨架、不调用retarget或重置滤波。新增确定性测试拦截ready并验证模块已加载，同时禁止任何启动期retarget调用；先失败后通过。修复后57项定向测试通过（12.439秒），包含官方worker首帧/序号间断对照、PICO真实本地Zenoh组件交接及SPARK+Manus联合录制。首次回调的延迟缺口已定位，仍不能因此宣称所有201ms命令交接超时或长期实时问题均已消失。

联合测试曾出现一次手回调新鲜度超时，重复运行未稳定复现，原因尚未确认。新增状态诊断用于定位，不放宽200ms输入阈值。

2026-09-09真实PICO2仿真录制`58e819c7-e587-441f-9c4e-ad2bfaabbbab`确认了该超时根因：官方左右worker原先串行IPC，处理线程逐渐落后于输入，处理到接收序号1368时回调年龄为208.787741ms，超过200ms门限；同一录制的最大retarget事务约41.7ms。新增两个独立官方worker的并行事务执行器，保留每侧独立滤波、序号间断语义、200ms新鲜度门限和失败锁存；组件退出时显式回收执行器。新增并发barrier回归先失败后通过；本地双worker 120帧基准中位延迟约2.73ms、P95约2.99ms、最大24.66ms，官方PICO双手完整smoke及805项发现集（50项环境跳过）通过。该结果仍需重新连接真实PICO2做现场时延和手型验收。

新增诊断的联合测试还出现过一次录制队列溢出。核对原版`rawviz.cpp::EnsureTopology`后，修正了测试每帧重复输出HAND/NODE拓扑的差异：拓扑仅首次输出，之后输出POSE；独立队列溢出测试仍保留。修正后19项定向测试通过，但不能据此宣称长期吞吐问题已解决。逐行HDF5写入的本机采样中，1000条审计约0.409秒（含性能分析开销），数据集访问/扩容/写入占主要时间。

随后增加有界连续审计批写：只取队列中已存在的最多64条连续审计，不等待凑批、不跨越其他流、不改变行顺序/时钟，也不改变旧profile的调度。整个批次通过验证后才建立时间线并写盘；磁盘失败仍中断并标记不完整。1000条审计同机对比逐行0.392秒、64行批写0.013秒，重读内容严格相等；这是局部性能测试，不是整条链路200Hz验收。23项接线/录制测试及持续至少2秒的官方手/原生双臂联合录制测试通过；长期设备负载仍待验证。

将联合录制延长到至少2秒后，一次638项全量回归在手执行器出现`authorized hand command rejected`，该轮失败（218.234秒），不得算通过。新增执行器拒绝诊断区分命令年龄、会话年龄、授权阶段、健康状态及内部校验错误；11项相关测试和连续5次联合录制重跑通过，但拒绝根因仍未稳定复现，不能用重跑通过覆盖历史失败。PICO官方输入边界的接线暂未继续，优先完成此问题的证据收集；没有放宽新鲜度或自动重启。

加入拒绝诊断后的639项回归再次复现（204.948秒）：手命令年龄201126477ns超过200000000ns，而会话年龄4315502ns、teleop授权和健康状态均正常。确认是命令新鲜度拒绝，而非IK/关节限位/会话授权失败。为验证写盘线程竞争的影响，将新live录制的HDF5所有权移入独立spawn进程，保留原有有界队列及连续批写；子进程故障不自动重启，失败后禁止完整关闭，只能abort。26项定向测试通过，仍须完整回归和长期负载验证，不将隔离进程本身等同于实时验收通过。

## 未完成项（不能据此标记总体完成）

1. 三种sim受管入口已接通；同一router/运行目录下PICO双臂双手退出→VR双臂启动/退出测试已通过，含残留权限探针。双向反复切换、VR实时手套切换、长时间错误退出压力与设备验收仍需覆盖。
2. PICO2骨架到官方手后端、手producer授权及完整多进程仿真已通过合成测试；实时稳定性及真实PICO/Manus输入验证未完成。
3. 共享控制周期、原子命令、VR live绑定、Home联合重置/显式重新授权及实际UDP进程测试已实现；任意位置可信恢复、故障恢复及`processed_guarded`尚未完成。
4. XR直接子模式、增量映射、控制器事件授权接线已实现；实际 XR SDK/PC-Service 现场联调、控制器标定和设备验收未完成。Tracker不属于当前目标。PICO2几何手势观察及显式双手张开 start 已接通，其余手势操作及真实手型验收未完成。
5. 完整算法分阶段诊断、可信恢复、XR/操作录制回放和长期可视化验收。
6. 真机profile/preflight、真实反馈偏差监督及设备验收。现有SPARK工厂仍拒绝real，不应删除这一保护来伪装完成。

下一阶段继续联合恢复/执行监督、完整模式切换验收、XR现场接入及长期吞吐量/实时周期验收；实时录制已有基础闭环，仍需完整上游标定契约。不能让新的原生worker冒用旧单臂IK接口或复用旧PICO profile的默认算法。

## 最近验证

- 上述最后只读解析修复后，受保护旧PICO2完整参数组合复测`passed: true`：`/tmp/pico-sim-smoke-tf6dqq6e`。新PICO官方双手+显式手势START+单侧/双侧视觉丢失保持/恢复+断连+v131外层passthrough/URDF/双overlay/HDF5复测`passed: true`：`/tmp/pico-sim-smoke-hxo2tojc`。原生CTest本轮9/9通过；`git diff --check`、两入口`bash -n`通过，索引为空，HEAD仍为`63e1e0f`。这是仿真软件验证，不解锁真机，也不抹去`processed_guarded`、XR现场和真机验收缺口。
- FIFO及有界批写修复后固定快照全量：726项通过、0跳过（270.561秒），官方Hand2 1339帧最大误差0，SHA256仍为`8b965b1788a23813b3b2537cd4f8eec7657ae5d1193a95cdb23e0d99fef51b75`。全量中的同router新PICO手势/双臂双手/HDF5及双向切换通过，日志`/tmp/pico-sim-smoke-bvrqn45i`。之后仅修正只读解析器对VR `operator_input=controller`误报runtime_available=true的问题，新增失败用例确认后修复，8项启动器回归通过（0.482秒）；该小改动不混称已经包含在726项快照中。
- 随后718项回归失败1项（257.711秒）：总线实际收到一次PICO手势START结果，HDF5却无`operator_result`。录制回调出现最长约3.04秒积压。新PICO此前由多个订阅回调竞争同步HDF5 writer锁，单次事件未能写入；不能将该文件当作完整操作审计。
- 修复限定新PICO recorder：4096条有界接收FIFO、复制载荷/接收时间、单dispatcher按序处理，退出停止接收并排空已接纳条目。首次仅增加FIFO后明确暴露queue overflow，因此没有放大队列掩盖吞吐不足；再增加每数据集最多256行批写，保留原字段、行顺序、时间线及周期flush。旧`SessionH5Writer`默认仍调用原来的立即写入函数，新PICO才选择缓冲writer。故障、溢出和清理失败均报告不完整，30秒无法排空报告完成性未经验证，不并发关闭HDF5。
- 缓冲写盘与原写盘逐数据集比较通过，包含完整PICO原包、审计和手命令；补齐600条审计的有界/flush顺序、写盘失败不标complete、FIFO不阻塞网络回调、溢出/订阅撤销失败清理测试。46项录制发现集通过（4.870秒）；新PICO双臂/官方双手/手势START/HDF5及VR→PICO→VR切换通过`/tmp/pico-sim-smoke-vecd66d4`（2项，49.826秒）。全量重验另记，不把这些定向结果称为总体方案完成。
- 正常尺度联合测试调整后的完整固定快照：715项通过、0跳过（248.968秒），官方二代手1339帧最大误差0，输出SHA256仍为`8b965b1788a23813b3b2537cd4f8eec7657ae5d1193a95cdb23e0d99fef51b75`。此后补充200ms/200ms+1ns边界与VR→PICO→VR切换测试，需单列后续结果。715项通过不抹去下述历史超时，也不等于长期实时/设备验收。
- 后续714项全量失败1项（260.048秒）：VR+Manus联合测试手命令年龄203744172ns，超过未修改的200000000ns门限。增加入队前、命令队列、录制sink及机械臂cycle耗时诊断后再次复现：入队前190579150ns、队列10152684ns、sink58915ns、arm cycle3326787ns。主要积压在手输入处理侧，不能归咎于IK或通过放宽门限处理。
- 查到联合测试使用的是语义重排用`_payload()`：所有节点共线、位置1–23米，不是正常手部几何。隔离官方worker的100回调采样中，该输入均值8.15ms，正常尺度非共线输入均值4.25ms；这只是局部实验，不是周期保证。联合测试现显式提供正常尺度几何，仍每20ms输出左右POSE、每POSE执行原版回调；旧共线语义测试、溢出和过期拒绝测试保留。新测试验证经过原版assembler后的节点顺序、float32数值、Y只反射一次与非退化性。6项输入测试及连续5次真实时钟联合测试通过；全量回归另记，不以此宣称真实设备长期实时验收完成。

- 在显式手势start接线之前，最新固定快照705项全量通过、0跳过（235.704秒）；官方手1339帧最大误差0、摘要不变。该结果不代替随后新增手势start代码的验证。

### 后续：显式双手张开START绑定

仅新`pico2_hands_sim`增加`--operator-input gesture`，默认配置仍keyboard。动作绑定固定本次router、观测进程/接收器身份和首个成功连接代次。armed下先收到非张开的有效姿态，再双手有效张开持续0.8秒，生成一次start_request，直接调用既有`request_start`，而不是复用可能回Home的`s`切换处理。请求仍检查来源就绪、标定及现有会话授权；forwarded只表示送到会话门控，不表示coordinator已授权。拒绝不排队重试，必须再次释放；非armed、失效、过期、序号断层、外来身份和重连代次不得触发。

新观察topic的结果另写入`meta/dual_audit`的`operator_result`，保留关联接收序号、代次、时间和转交结果；回放只读取审计，不创建绑定。键盘`s/c/q`仍保留；没有手势暂停/回Home/标定，不能替代物理急停。几何available只代表可计算性，不伪装成学习模型置信度。

- 全进程合成骨架测试无键盘`s`：启动时持续张开不启动，释放→稳定张开后实际coordinator进入teleop，双臂/官方双手运动，HDF5只记录一次请求。`/tmp/pico-sim-smoke-owaac47m`返回`passed: true`。这是仿真软件验证，不是实物手型准确率或真机验收。
- 同一手势启动入口叠加单手/双手视觉丢失、保持/恢复及断连回位测试：`/tmp/pico-sim-smoke-86sb0duz`返回`passed: true`。随后保护的旧PICO v131+passthrough+URDF+双overlay命令再次验证通过：`/tmp/pico-sim-smoke-skhkpitg`。
- 新PICO schema1.2 recorder还记录source/producer/coordinator/arm executor和左右hand executor状态为`component_status`审计，payload明确包含`topic`和原始typed `status`；运行期间高度标定状态/均值、算法/处理器诊断和身份可追溯。既有schema1.1订阅不变，VR live仍沿其已有LiveCapture格式，不伪造统一数据布局。10项手势结果/状态录制定向测试通过（0.127秒）。完整模型资产闭包及上游标定文件摘要仍待完善。

- 随后705项回归失败1项（248.386秒）：新增写盘中断测试在Pipe EOF后立即断言进程已退出，未履行interrupt接口规定的调用方回收步骤。改为最多3秒join后仍必须断言进程终止；没有放松进程清理要求，也未修改运行代码来掩盖失败。20项写盘/手线程定向回归通过（2.710秒），全量再次验证另记。这一轮VR+Manus联合测试未失败，但有限重跑不等于长期实时验收。

- 包含手势观察与同router切换的701项全量回归失败1项（242.218秒）：VR+Manus联合测试进入`producer_hand stale or unhealthy`。官方手1339帧最大误差仍为0；该轮不得计为全量通过。
- 为此增加故障时手producer/executor快照和本地时钟诊断，单项重跑通过（3.739秒），不能据此抹去失败。另以确定性测试复现跨线程截止时间竞态：手线程已处理时间为1000000010的输入，而机械臂检查时刻为1000000005，覆盖掉仍新鲜的1000000000输入，导致ready误判。现在保留最多256条小型已处理输入快照，选择截止时刻已接收的最新一条；不重写原始时间戳，不改变回调队列或算法状态，不放宽新鲜度。过期历史和只有未来帧仍明确not-ready。24项联动测试及7项手线程测试通过；该竞态与701项失败是否同因尚未由当时诊断证实。
- 录制关闭新增受控故障测试：暂停测试自己创建的写盘子进程，使用大消息复现Pipe.send阻塞；原关闭超时后dispatcher仍存活。新增owned-worker interrupt只终止该子进程，不从控制线程关闭HDF5/Connection，释放阻塞后由dispatcher清理。原30秒排空预算不变，超时后最多再等5秒并报告完成性未经验证，不以强制退出宣称文件完整或可读。

- 新PICO受管双臂双手、视觉丢失保持和录制接线后的全量回归：686项通过、0跳过（216.424秒），官方手1339帧最大误差0。这是后续模式切换测试和手势观察新增之前的快照，不混作新增后的全量结果。
- 同router/运行目录停止切换测试：2项通过（29.579秒），PICO完整进程日志`/tmp/pico-sim-smoke-3jkqjym7`。测试自己拥有独立localhost router，故意注册残留控制token验证检查能失败，随后清除探针，验证PICO退出清空控制权限、保留router、VR双臂受管入口可启动并退出。VR段未发输入和授权，不计作完整双模式运动对照。

### 后续：只读手势与操作请求边界

新增纯`OperatorObservation`/`OperatorEvent`和显式绑定的边沿过滤器。源/侧/动作/连接epoch固定，按本地接收时间稳定判定；启动或丢失后必须先释放，持续按住只产生一次请求。重复/外来输入不能推进状态，序号间断、失效、低质量及过期输入解除候选，回放不产生动作。该模块不持有执行权限，尚未开启自动动作绑定。

新增21点几何识别基线：腕到中指MCP长度归一化的捏合距离、四指MCP到指尖弦长/骨链长度判断张开和握拳，并带显式迟滞。阈值不是已验收的人体手型模型或概率置信度。无效/退化输入为unknown且不可用。

仅`pico2_hands_sim`显式选择`PicoGestureRuntime`：原始包和原有四条骨架/机械臂观察照常发布，额外发布`tianji/observation/operator/pico_gestures`，包含接收器、代次、序号、接收时间、关联ID及左右标签/几何指标。schema1.2的PICO recorder将其保存为`meta/dual_audit`中的`operator_observation`，完整raw26点仍保留。没有新增命令或请求topic，禁用手执行时仍能观察手势，旧profile默认runtime不变。

- 手势/操作事件/录制/入口定向22项通过（0.333秒）；完整进程验证另行记录，不将纯几何通过当作真实手势验收。

- PICO手组件生命周期与首次回调依赖加载修复后，固定源码完整回归675项全部通过、0跳过（211.072秒）；官方手1339帧最大误差0，输出摘要仍为`8b965b1788a23813b3b2537cd4f8eec7657ae5d1193a95cdb23e0d99fef51b75`。同一10帧独立实验修复后首帧18.035ms，后续3.94–9.12ms；这是单机有限采样，不是最坏周期上界或硬实时保证。
- 本轮PICO组件及录制来源扩展后的完整回归：667项通过、0跳过，229.093秒；官方手1339帧最大误差0。该轮启动后追加了录制模式互斥校验，因此不能把667项结果当作最后一处改动的完整快照验证；模式互斥及录制worker随后单独验证11项通过（2.712秒）。混入另一模式的原始流在入队前拒绝，不影响既有VR默认录制参数。
- PICO组件定向29项曾因273ms输入年龄失败，增加诊断后的重跑通过；完整回归通过也不抹去该历史失败。同机观察到另一工程编译负载，仅记录为环境因素，尚未证明因果，未干预该进程。新PICO受管入口仍保持禁用。
- 写盘进程隔离后最新完整回归：642项Python测试通过、0跳过，207.592秒；官方手1339帧最大误差0。这一轮包含加长联合录制及子进程故障测试，但不据此抹去前述偶发超时，长期实时验收仍未完成。
- 完整回归（手部状态诊断接线后、审计批写前）：636项Python测试全部通过、0跳过，198.682秒，包含XR独立原版数学对照；官方手1339帧最大误差仍为0。审计批写后的全量回归另行记录，不混用此前结论。

- 9月9日全部可选依赖显式启用：617项Python测试通过、0跳过（208.618秒）；包含Home重置与实际UDP下的`h→r→s`。这是实时录制接线前的完整回归，不能替代后续变更的全量验证。
- 后续实时录制定向测试27项全部通过：实际UDP+8秒运行/重置/录制重读、受管shell录制、官方手worker+合成rawviz联合录制、有界队列/关闭异常、审计数据校验及旧1.2兼容。随后增加的关闭失败清理已通过7项录制测试；原生9组CTest再次通过。
- SPARK原生CTest：9组通过。
- 受保护旧PICO2 v131参数组合9月9日再次执行：`pico_sim_smoke.py`返回`passed: true`，日志`/tmp/pico-sim-smoke-snez847o`。
- 写盘进程隔离后再次运行同一受保护组合：`passed: true`，日志`/tmp/pico-sim-smoke-v90rhycj`。
- 离线CLI（单独执行）：3项通过，含双臂、双手、完整HDF5关闭后重读（93.207秒）；已使用原子最终双臂命令。
- 新显式启动门控：有输入不自动授权，过期输入下拒绝的请求不会排队到恢复后执行；启动后的输入丢失仍走原版保持。
- 9月9日定向测试：共享周期/交互启动/完整短trace联合回放6项；Manus接收/官方解析8项；手线程/权限/接收10项；SPARK握手/worker/源码13项；手握手/worker/源码7项；实时双臂核心3项；新增手执行权限/双臂原子接收7项，均通过。这些用例有重叠，不应相加充当独立测试总数。
- Git HEAD保持`63e1e0f`，未暂存、提交或推送。变更包括新增模块、7个既有运行文件的显式兼容扩展及README支持边界说明。
