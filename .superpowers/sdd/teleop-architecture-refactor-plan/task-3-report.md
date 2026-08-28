# Task 3 报告

## 状态

DONE_WITH_CONCERNS：canonical mapper、SessionClient/TargetPublisher、PICO source、aligned mocap live、H5 replay 的新目录和 typed 发布链路已完成；旧 diagnostics 资产按父任务要求恢复，尚待后续 Task 8 迁入 `diagnostics/`。

## RED / GREEN

按 TDD 先新增 `tests/test_canonical_sources.py`，首次运行：

```text
ModuleNotFoundError: No module named 'pico_body_tianji.sources.common.session_client'
```

这是预期的 RED（新 canonical API 尚不存在）。随后实现 source/common、PICO 和 mocap canonical 模块，并迁移测试。最终 focused GREEN：

```text
............................等待 IK Home、有效 tianji_wrist marker 后按 s；节点推导 wuji2 r_mount/r_wrist Home，并把 H5 wrist frame0 转换到 r_wrist。随后 Enter 保压接近，按 r 装载后续轨迹；活动阶段按 s 回 Home，按 q 回 Home 后退出。
键盘 s：已读取 tianji_wrist marker 并推导 r_mount/r_wrist Home；持续按住 Enter，使 r_wrist 接近 H5 wrist frame0，松开保持。
.
----------------------------------------------------------------------
Ran 29 tests in 0.084s

OK
```

另有 canonical 模块语法编译：

```text
pixi run python -m py_compile .../sources/common/*.py .../sources/pico_controller/*.py .../sources/mocap/*.py .../scripts/mocap_live .../scripts/mocap_h5_replay .../scripts/pico_controller_source
# 无输出，退出码 0
```

PICO 构造 smoke：输出 `armed`，退出码 0。

## 文件与行为

- 新增 `sources/common/target_mapper.py`：`ControllerOnlyTargets` → `ArmTargetBatch`，`ControllerOnlyTeleopMapper` → `EndEffectorTargetMapper`，方法改为 `map_relative_controller_frame()` / `map_absolute_tcp_poses()`；不保留旧 mapper API。
- 新增 `sources/common/target_conditioner.py`、`freshness.py`、`replay_clock.py`、`keyboard.py`。
- 新增 `sources/common/session_client.py`：subscriber→query snapshot 顺序，typed `SessionState`/`LatchedBool` 解析，router/coordinator identity 校验，实例+sequence 去重，start/return/shutdown intent 和 1 秒超时 fail-closed。
- 新增 `sources/common/target_publisher.py`：统一 monotonic timestamp/sequence，typed arm/hand target、raw PICO/live/H5、hand joint command、Frame0 skeleton、source status；hand keypoints 提供 wrist-relative 归一化且 root 精确为零。
- 新增 `sources/pico_controller/{controller_frame.py,source.py,node.py}`：PICO A 上升沿只进入 `start_pending`，收到匹配权威 teleop 后才初始化 mapper/发布 target；断流和映射错误请求 return；不读取 Body API。
- 新增 `sources/mocap/live_node.py`：仅订阅 `mocap/aligned/hands`，接收主机 monotonic watchdog 0.5 秒，外部 acquisition envelope 重新封装 raw；stream instance 变化清 reference 并回 armed/return；`frame_valid=false` 不会误杀另一侧有效手；不订阅 robot marker。
- 新增 `sources/mocap/motive.py`：共享 H5/diagnostic Motive rigid-body parser，不产生 live target。
- 迁移 `sources/mocap/h5.py` 和 `h5_replay_node.py`：H5 的 `s` 先冻结 reference、发 intent、进入 `start_pending`；匹配 teleop 后才 approach/replay；arm target 使用 canonical `Base_R`，hand 走 typed relative target 或 typed 20-joint direct command；solved pose 使用 `ArmSolvedPose.target_sequence` 关联检查；Frame0 skeleton 走正式 diagnostics topic；speed/yaw 构造后固定，status 仅声明 simulation capability。
- 更新 `scripts/mocap_live`、`scripts/mocap_h5_replay`、新增 `scripts/pico_controller_source`，以及 CMake Python 入口清单和 H5 viewer import。
- 测试迁移到 `tests/test_target_mapper.py`、`tests/test_h5_replay.py`，新增 `tests/test_canonical_sources.py`；`scripts/test.sh` 改用新 mapper/H5 测试名。
- 原 `controller_only` 中未迁移的 diagnostics 资产已按父任务要求从 `287da97` 恢复：`mocap_keyboard_step.py`、`mocap_keyboard_step_node.py`、`controller_only_trace.py`、`controller_only_real_diagnostic.py`、`raw_keyboard.py`、`__init__.py`。没有恢复旧 mapper/conditioner/source/H5/live/PICO 兼容模块。

## 未做的跨任务事项

- Task 4 负责 coordinator/IK producer、final command authority、robot config 与 C++ consumer，本文没有实现。
- Task 5 负责 MuJoCo/Marvin/Wuji executor、安全 stop 与 hand executor；本文没有实现。
- Task 6 负责 session HDF5 recorder/replay；本文只提供 typed raw 发布，不实现 recorder。
- Task 8 负责把已恢复 diagnostics 资产迁移到 `diagnostics/`、清理全部旧入口/config/runtime、统一 launcher/doctor/CMake 其余部分；本文保留恢复资产供其迁移。
- acquisition 仓库只读，未修改其 dirty 文件；aligned publisher 仅按外部 envelope 读取 `mocap/aligned/hands`。
- 未运行 formatter、lint、full suite、跨语言构建或实体设备验收。

## 风险与后续

1. H5/live 仍复用了旧几何实现，canonical arm 的数值应在 Task 4 producer/coordinator 进程级测试中验证 Base frame 与 robot config 的一致性。
2. H5 旧 diagnostics status 内容中仍有 legacy 诊断字段；后续 Task 8 应迁移并收敛字段，不得重新作为 authority。
3. 恢复的 diagnostics 文件暂在旧 `controller_only` 目录，且其中部分旧 step node import 已迁出的 mapper；Task 8 迁移前不应把这些诊断入口加入产品 launcher。
4. 本地 fake session focused tests 已覆盖 typed wire/identity/lifecycle；尚未接受真实 Zenoh router 的 query/reconnect/ACL 行为。

## Fix round 1（审查修复）

根据 `task-3-review.md` 完成以下可达修复：

- `live_node.py` 正式入口现在以 `raw_keyboard` 后台线程接收 `s/q`，并在关闭时停止线程；PICO 删除 held-A 的错误 return，A 只用于 start rising edge。
- `zenoh_util.open_session()` 统一读取 `TIANJI_ROUTER_ENDPOINT`（默认 `tcp/127.0.0.1:7447`），新增 `require_single_router()`；三个 source CLI 要求 component/router/coordinator identity 并验证实际 `session.info.routers_zid()` 恰好一个且匹配。
- `SessionClient` 强制 `expected_coordinator_instance_id`，并与 `TargetPublisher` 共享 `SequenceAllocator`，避免 intent/data 同一 publisher instance 序号回退；匹配 start 拒绝快照立即清 pending。
- live 使用 frozen reference 的相对旋转 delta，且从 source 参数构造完整 `TargetConditioningSettings`；real preflight capability 按状态输出并支持运行中 capability loss return。
- H5 增加 H5/hand real preflight capability 条件、MotiveFrameSource 严格解析、solved router/producer identity 检查、return intent 请求和完成后自动 return；Frame0 发布字段改为 typed canonical 字段。
- `mujoco_joint_viewer.py` 使用 `Frame0HandSkeleton.from_dict()` 解析 canonical diagnostics 字段，并统一 motive-world home pose；恢复的 step diagnostic node 改为 canonical mapper/conditioner/controller frame imports，避免资产无法 import。

Fix round 1 focused 实际输出：

```text
............................等待 IK Home、有效 tianji_wrist marker 后按 s；节点推导 wuji2 r_mount/r_wrist Home，并把 H5 wrist frame0 转换到 r_wrist。随后 Enter 保压接近，松开保持；按 r 装载后续轨迹；活动阶段按 s 回 Home，按 q 回 Home 后退出。
键盘 s：已读取 tianji_wrist marker 并推导 r_mount/r_wrist Home；持续按住 Enter，使 r_wrist 接近 H5 wrist frame0，松开保持。
.
----------------------------------------------------------------------
Ran 29 tests in 0.082s

OK
```

未完成/风险：SessionClient 尚未完成三类 query reply 的独立 completion/reconnect 重查；H5/live real preflight 依赖 launcher 注入的 preflight 参数，Task 8/5 需接入真实配置扫描；旧 diagnostics 主体仍在 controller_only 待 Task 8 迁目录；原 1035 行 H5 测试已由 canonical 生命周期测试替代，完整几何/terminal/viewer 回归需从历史提交迁移恢复。

## Fix round 2（重审修复）

- 修复 `SessionClient`/`TargetPublisher` 构造字段回归，确保 `router_zid`、coordinator identity、clock、allocator 均实际保存；intent sequence 与 target/status/raw 共用同一 `SequenceAllocator`。
- `open_session()` 统一使用 `TIANJI_ROUTER_ENDPOINT`，三 source CLI 强制 `TIANJI_COORDINATOR_INSTANCE_ID`，并在 source 创建前执行 `require_single_router(session, expected_zid)`。
- live 增加正式 `raw_keyboard` 入口、typed bool real-mode/preflight 参数校验、configured conditioner、frozen reference relative rotation；PICO held-A 不再触发错误 return。
- H5 增加 typed bool H5/hand preflight 与 real-mode fail-closed 校验、Motive typed parser、solved router/producer identity检查、return completion baseline、完成后自动请求 return；Frame0 viewer 改用 `Frame0HandSkeleton.from_dict()` canonical 字段。
- diagnostics step node 的 mapper/controller imports 改为 canonical symbols，保留算法主体供后续 diagnostics 目录迁移。

Round 2 实际 focused 输出：

```text
PYTHONPATH=src/pico_body_tianji:vendor/python pixi run python -m py_compile ...  # 退出码 0，无输出
............................等待 IK Home、有效 tianji_wrist marker 后按 s；节点推导 wuji2 r_mount/r_wrist Home，并把 H5 wrist frame0 转换到 r_wrist。随后 Enter 保压接近，松开保持；按 r 装载后续轨迹；活动阶段按 s 回 Home，按 q 回 Home 后退出。
键盘 s：已读取 tianji_wrist marker 并推导 r_mount/r_wrist Home；持续按住 Enter，使 r_wrist 接近 H5 wrist frame0，松开保持。
.
----------------------------------------------------------------------
Ran 29 tests in 0.084s

OK

git diff --check
# 无输出，退出码 0
```

仍未覆盖：三类 query completion/reconnect 真实 Zenoh 行为、H5/live 完整设备 preflight 扫描、原 1035 行 H5 全部几何/terminal/viewer 回归测试恢复；这些已明确列入 concerns，不能宣称物理/全链路验收完成。

## Fix round 3（重审 open findings）

- `SessionClient` 增加 `snapshot_complete` 与 `reconnect()`：重连会丢弃 coordinator state/latch、identity、sequence baseline，重新执行 subscriber→query；intent 授权后的 pending deadline 不会清掉已授权 return completion。
- `MotiveFrameSource` 进一步拒绝 bool/float/string coercion 的 frame/id、要求非负 frame、正整数 rigid id、有限且归一化 quaternion；H5 callback 保存并消费 typed Motive frame，rigid names 严格校验。
- live/PICO/H5 的 source config real-mode/preflight bool 均要求真实 YAML boolean，real mode 默认 fail closed；active tick 检查 capability loss。live orientation 使用 frozen reference 的 `R_current * inverse(R_reference)` 并应用于 Home rotation；conditioner 完整读取 source 参数。
- diagnostics calibration 节点迁移到 `diagnostics/mocap_calibration_node.py` 并改用 canonical mapper/controller imports；`test_mocap_keyboard_step.py` 恢复完整 27 项行为覆盖并纳入 `scripts/test.sh`，未恢复任何旧 mapper/conditioner/source/H5/live/PICO 兼容 API。
- viewer Frame0 使用正式 `Frame0HandSkeleton.from_dict()` 字段和 typed validation，H5 传入 Motive wrist home 坐标。

Round 3 实际验证：

```text
PYTHONPATH=src/pico_body_tianji:vendor/python pixi run python -m py_compile \
  src/pico_body_tianji/pico_body_tianji/zenoh_util.py \
  src/pico_body_tianji/pico_body_tianji/sources/common/*.py \
  src/pico_body_tianji/pico_body_tianji/sources/pico_controller/*.py \
  src/pico_body_tianji/pico_body_tianji/sources/mocap/*.py \
  src/pico_body_tianji/pico_body_tianji/diagnostics/mocap_calibration_node.py \
  src/pico_body_tianji/scripts/mujoco_joint_viewer.py
# 退出码 0，无输出

PYTHONPATH=src/pico_body_tianji:vendor/python pixi run python -m unittest \
  tests.test_canonical_sources tests.test_target_mapper tests.test_mocap_h5 \
  tests.test_h5_replay tests.test_mocap_keyboard_step
# Ran 56 tests in 0.186s / OK

git diff --check
# 无输出，退出码 0
```

仍未覆盖：真实 Zenoh router 下三 query completion/multiple reply/reconnect 的进程测试；H5/live 真实 H5/hand/设备 preflight 扫描接口仍由 launcher 提供严格 typed result；原 1035 行 H5 测试中的完整 geometry/terminal/viewer 部分尚未完全恢复（当前保留 canonical lifecycle + 旧 step diagnostic 27 项保护）。

## Fix round 4（第四轮修复）

本轮按 brief 与完整重审 findings 继续收敛 source lifecycle、严格解析、real admission、diagnostics 与入口 wiring：

- 新增 `sources/common/real_admission.py` 的 `RealCapabilityInput`，所有 bool 字段要求真实 YAML/Python boolean；`"false"` 等字符串拒绝解析。live 接受 typed capability/provider，并在 start_pending、teleop 每 tick 重新检查 speed、yaw、deadman 与 preflight；real capability 默认不声明。
- H5 对实际 `wuji2_joints` 全部帧执行 `(N,20)`、finite、Wuji beta1 limits 校验，禁止 real admission 依赖前向填充；speed≤0.25、yaw=0、deadman 与 typed preflight 全部满足才暴露 real capability。H5 solved pose 现在强制匹配正式注入的 producer logical id 与 instance id；CLI 支持 `--expected-producer-*`，运行入口从 `TIANJI_ARM_PRODUCER_*` 注入，缺失即 fail closed。
- live 冻结方向 delta 改为 `R_current * R_reference^-1`，并通过 world→Base 旋转共轭后施加 Home rotation；stream/return 不再使用旧 latch 立即切到 armed，return 只接受匹配 intent 的 returning/idle 及新 sequence 的 `at_home=true`、`return_complete=true`，完成有独立 deadline。
- `SessionClient` 将 state、at_home、return_complete 三次 query 分别计 completion，subscriber event 不满足 snapshot barrier；foreign/multiple reply 使 client fail closed，coordinator sequence 使用 publisher-global baseline，reconnect 清 identity、baseline、invalid 状态并重新 query。`TargetPublisher` 补齐共享 allocator 的初始 sequence。
- `MotiveFrameSource` 强制顶层 names、canonical decimal ids（拒绝 `7`/`07` 混用）、重复 id/name、finite/normalized quaternion；H5 与 diagnostics callback 均只保存 typed `MotiveFrame`。Frame0 viewer 改订阅 canonical arm target/solved topics，并在 `apply_latest()` 应用 producer 消息。
- calibration diagnostic 禁止旧 state/target/final-command publisher，仅保留 canonical SessionState 订阅与可选 SessionClient intent；补充 `scripts/mocap_calibration`、CMake install 和 Pixi simulation-only task。H5 validate-only 改为直接 canonical module，live wrapper 只传新 CLI 参数并要求三个 component/router/coordinator identity。

本轮 RED/GREEN 与 scoped 验证：

```text
PYTHONPATH=src/pico_body_tianji:vendor/python pixi run python -m unittest tests.test_task3_round4
# RED（首轮）：ModuleNotFoundError: pico_body_tianji.sources.common.real_admission
# GREEN：Ran 5 tests ... OK

PYTHONPATH=src/pico_body_tianji:vendor/python pixi run python -m unittest \
  tests.test_task3_round4 tests.test_h5_replay tests.test_canonical_sources \
  tests.test_mocap_keyboard_step tests.test_target_mapper
# Ran 43 tests in 0.166s / OK

PYTHONPATH=src/pico_body_tianji:vendor/python pixi run python -m unittest \
  tests.test_controller_only_trace tests.test_controller_only_real_profile
# Ran 5 tests in 0.010s / OK

PYTHONPATH=src/pico_body_tianji:vendor/python pixi run python -m py_compile \
  src/pico_body_tianji/pico_body_tianji/sources/common/*.py \
  src/pico_body_tianji/pico_body_tianji/sources/pico_controller/*.py \
  src/pico_body_tianji/pico_body_tianji/sources/mocap/*.py \
  src/pico_body_tianji/pico_body_tianji/diagnostics/mocap_calibration_node.py \
  src/pico_body_tianji/scripts/mujoco_joint_viewer.py \
  src/pico_body_tianji/scripts/mocap_calibration
# 退出码 0，无输出

git diff --check
# 退出码 0，无输出
```

`tests/test_pico_link_probe.py` 已按批准 clean-cutover 删除；其旧 `full_body` imports 在本轮前即不可解析，不保留兼容 alias。未完成/风险：未运行 full suite、真实 Zenoh router、多进程 launcher、C++/Task 4-8 consumer 与实体设备验收；现有 wrapper 仍含后续 Task 8 的旧 runtime 启动段，需后续统一 session launcher 做最终 clean cutover。当前状态仍为 DONE_WITH_CONCERNS。
## Fix round 5（最终 source 契约修复）

本轮先按要求新增行为测试并确认 RED：

```text
PYTHONPATH=src/pico_body_tianji:vendor/python pixi run python -m unittest tests.test_task3_round4
Ran 17 tests ... FAILED (failures=6, errors=4)
```

RED 具体暴露 H5 `load_mocap_h5` 未导入、PICO `self.start()` 缺失、完整 Motive envelope 被拒绝、非单位 Home 朝向右乘、deadman 错误仍可声明 real、H5 limits 可被覆盖、YAML capability 可自报、SessionClient 旧 query 误标 invalid 以及 calibration `SessionState` 未导入。

已完成修复：

- 恢复 H5 `load_mocap_h5` import，恢复 PICO 正式 `start()`（SessionClient subscriber/query + 初始 status），补充两条入口 smoke。
- Motive parser 按 acquisition NatNet envelope 的完整顶层/marker/rigid-body schema 严格校验；拒绝错误 schema、bool/string/float rigid id、重复 canonical id/name、非 finite/非单位 quaternion；H5 与 calibration 只使用 typed `MotiveFrame`。
- live orientation 使用 world→Base 共轭后的 Base delta 左乘 Home，增加非单位 Home、非交换轴测试。
- real admission 只接收 typed `RealCapabilityInput`/provider；YAML preflight 与 capability 不再生效；H5 direct 全量 20-joint finite/固定 Wuji beta1 limits 扫描不可被配置覆盖；deadman 任意读取异常锁存错误并 bounded return。
- SessionClient 将 subscriber 领先的旧 query 视为该 channel 已完成而不 invalid；维持 foreign/multiple authority fail-closed，使用全局 `(publisher_instance, sequence)` baseline，reconnect 清理 identity/baseline/invalid 后重新 query。
- calibration callback 导入 `SessionState` 并只消费 typed Motive；Frame0 viewer 默认和 H5 wrapper 均切换 canonical `tianji/diagnostics/h5/frame0_hand_skeleton`；测试入口纳入新增 H5 regression。
- 新增 `tests/test_mocap_h5_replay.py`，保留 geometry/deadman/direct/terminal/viewer 回归；修正 diagnostics typed frame fixture。

Fix round 5 focused GREEN：

```text
PYTHONPATH=src/pico_body_tianji:vendor/python pixi run python -m unittest \
  tests.test_task3_round4 tests.test_canonical_sources tests.test_h5_replay \
  tests.test_mocap_h5_replay tests.test_mocap_h5 tests.test_mocap_keyboard_step
Ran 77 tests in 0.207s
OK

PYTHONPATH=src/pico_body_tianji:vendor/python pixi run python -m py_compile \
  src/pico_body_tianji/pico_body_tianji/sources/common/real_admission.py \
  src/pico_body_tianji/pico_body_tianji/sources/common/session_client.py \
  src/pico_body_tianji/pico_body_tianji/sources/mocap/motive.py \
  src/pico_body_tianji/pico_body_tianji/sources/mocap/live_node.py \
  src/pico_body_tianji/pico_body_tianji/sources/mocap/h5_replay_node.py \
  src/pico_body_tianji/pico_body_tianji/sources/pico_controller/node.py \
  src/pico_body_tianji/pico_body_tianji/diagnostics/mocap_calibration_node.py \
  src/pico_body_tianji/scripts/mujoco_joint_viewer.py \
  tests/test_task3_round4.py tests/test_mocap_h5_replay.py
# 无输出，退出码 0

git diff --check
# 无输出，退出码 0
```

仍未完成/不在本轮范围：真实 Zenoh router 多进程 query/reconnect、Task 4 coordinator/IK producer、Task 5 executor/实体设备、Task 8 统一 launcher/runtime clean-cutover；历史 full H5 1035 行测试未原样恢复，但 geometry/deadman/direct/terminal/viewer 关键回归已迁移至 `tests/test_mocap_h5_replay.py`。本轮未运行 formatter、lint、full suite、跨语言构建或物理验收。
