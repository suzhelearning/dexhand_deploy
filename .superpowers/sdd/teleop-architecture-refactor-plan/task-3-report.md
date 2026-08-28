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
