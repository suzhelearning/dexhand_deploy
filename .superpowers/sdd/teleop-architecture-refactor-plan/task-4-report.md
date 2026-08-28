# Task 4 报告

## 已实现

- 新增 `pico_body_tianji/coordination/arm_command_coordinator.py` 与入口 `src/pico_body_tianji/scripts/arm_command_coordinator`：
  - launcher 传入 `publisher_instance_id`、`router_zid`，拒绝匿名 identity；
  - 唯一发布双臂 `ArmJointCommand`、`SessionState`、`LatchedBool(at_home/return_complete)`，双臂每 tick 共享 sequence/timestamp；
  - subscriber/queryable transport、exactly-one source/producer/executor readiness、freshness、proposal 限位/步长/回滚 fault、stale proposal bounded return、arm Home 与 hand at-zero gate；
  - fault 不置 return_complete，正常 return 在 fresh arm/hand 安全状态满足后一次性完成。
- 新增 `config/robot/arm.yaml` 与 `config/coordinator/arm.yaml`，Home/limit/joint order 与 coordinator 八字段均使用弧度。
- 新增 `connection_readiness.py`，并在 `HostReadinessGate` 暴露 connection/start 两层 gate；connection 不要求 policy observation，fault-return 只接受 fresh returning Home command。
- 新增独立 `src/pico_body_tianji/src/producers/arm_ik_producer_node.cpp` 与 `arm_ik_producer` CMake target/runtime wrapper。producer 仅订阅 canonical target/final command、调用原有 `ArmIkSolver`、发布 proposal/solved/status；严格检查 side/frame/schema/quaternion/elbow/identity/freshness，不发布 rejected proposal 占位。
- 三个 IK backend 源文件与 factory 迁移到 `src/pico_body_tianji/src/ik/{pinocchio_cpp,pinocchio_qp,tianji_official}`，算法实现未删除；旧 monolithic 节点与 CMake target 删除。
- 修复 `SessionClient` query barrier：旧的跨 channel sequence reply 只有在本 channel 已有 subscriber/cache value 时才完成该 channel；缺值 channel 不再被全局旧 baseline 错误放行。
- 新增 focused tests 覆盖 robot/coordinator 配置、idle 双臂刷新、exactly-one start、拒绝不变更、proposal fault/bounded Home、stale return、return latch once、readiness split 与 SessionClient query barrier。

## 实际验证

1. `pixi run bash -lc 'source scripts/common.sh && activate_bundle_runtime && python -m py_compile src/pico_body_tianji/pico_body_tianji/coordination/arm_command_coordinator.py src/pico_body_tianji/pico_body_tianji/connection_readiness.py && python -m unittest tests.test_arm_coordinator tests.test_task3_round4 tests.test_canonical_sources'`
   - 结果：`Ran 32 tests ... OK`。
2. `pixi run -e ik-build build-ik`
   - 结果：CMake 配置、三个 backend library、`arm_ik_producer`、official probes、Wuji bridge 均构建/安装成功；输出中仅有既有 CMake Boost policy warning 与 Wuji C++20 designated-initializer warning。
3. `pixi run -e ik-build bash -lc 'cmake --build build/ik --target arm_ik_producer --parallel 4'`
   - 结果：`Built target arm_ik_producer`。
4. `set +e; ./build/ik/arm_ik_producer ...; test "$code" -eq 1`
   - 结果：缺少 `TIANJI_COMPONENT_INSTANCE_ID`、`TIANJI_COORDINATOR_INSTANCE_ID`、`TIANJI_ROUTER_ZID` 时 exit 1，明确 fail-closed。
5. `git diff --check`
   - 结果：无 whitespace error。

## 跨任务未完成项与风险

- Task 3 ledger 中 trusted real preflight/provider 仍按裁决留给 Task 5/8；本提交默认 simulation capability，未宣称真机安全准入或物理验收。
- Task 8/10 仍需完成统一 `run_session` launcher、实际 router ZID 查询、MuJoCo/Marvin/Wuji executor 全链路接线、旧脚本/配置/diagnostic 入口清理及完整 E2E；本任务只更新了 IK build/deploy/test 的直接入口引用。
- C++ producer 当前使用轻量 canonical JSON parser/Zenoh wiring，未完成跨语言 process-level router smoke；需要后续正式 protocol fixture 与 managed ACL router 验证。
- hand producer/status 的完整 profile exactly-one 与执行器重连闭环仍需 Task 5/8 进程级验证；Python coordinator 已提供 typed hand gate 基础。
- 历史 H5 1035-line parity 仍按 Task 3 裁决由 Task 10 扩展，未在本 focused 范围宣称完整回归。

## Fix round 1

- 修复 coordinator 对真实 `zenoh.Sample.payload` 的 bytes 解包；dispatch 对 intent/status/arm state/proposal 及 hand status/state 均接入，解码失败进入 fault，不再静默丢消息。
- IK producer 增加 canonical target 数组边界解析、schema/source/side/frame/future-time/sequence 校验；按 rate 发布 healthy/ready typed status，并声明 producer liveliness token。
- coordinator 增加 bounded Home 轨迹（`home_minimum_duration_s` 与 `home_max_speed_rad_s`）、fault latch（fault 中 intent 不得解除、fault 不 complete）、executor/state stale fault 与 hand producer return 分流；补充 authority instance 与 sequence 检查。
- 修正 return-complete latch 在普通 idle/teleop tick 的 sequence 刷新，避免合法 channel snapshot 被 sequence=0 永久拒绝；新增 hand dispatch callback。
- 实际命令与输出：
  1. `pixi run bash -lc 'source scripts/common.sh && activate_bundle_runtime && python -m unittest tests.test_arm_coordinator'` → `Ran 10 tests ... OK`。
  2. `pixi run -e ik-build bash -lc 'cmake --build build/ik --target arm_ik_producer --parallel 4'` → `Built target arm_ik_producer`。
- 尚未完成：C++ 仍需后续替换为仓库统一正式 JSON parser/完整 cross-language process fixture；HostReadiness 对 Marvin 的生产调用、非默认 router SessionInfo exactly-one、所有历史 launcher/runtime 调用点和完整 hand/executor process smoke 仍需后续修复。

## Fix round 2

- 恢复 C++ target parser 对 `router_zid` 的实际赋值；保留 source 校验，避免合法 target 被错误判定 router mismatch。
- 恢复 coordinator tick 的双臂 command、SessionState、at_home、return_complete 周期发布；所有 wire 输出继续共用 tick sequence/timestamp。
- 修正每轮新 fault 的 bounded-return 起点和起始时间：进入 fault 时从当前 safe command 重新快照，避免复用上一轮轨迹。
- C++ producer 保留独立 status publisher 与 liveliness token，初始化与每次 control tick 发布 healthy/ready typed status。
- 实际命令与输出：
  1. `pixi run bash -lc 'source scripts/common.sh && activate_bundle_runtime && python -m py_compile src/pico_body_tianji/pico_body_tianji/coordination/arm_command_coordinator.py && python -m unittest tests.test_arm_coordinator'` → `Ran 10 tests ... OK`。
  2. `pixi run -e ik-build bash -lc 'cmake --build build/ik --target arm_ik_producer --parallel 4'` → `Built target arm_ik_producer`。
- 保留风险：完整 formal JSON unknown-field parser、HostReadiness 到 Marvin 实际调用、完整 launcher/runtime clean cutover、跨语言 process fixture 与物理 executor 仍需后续闭合。

- round2 follow-up verification repeated the same focused commands after restoring the periodic authority publish path; both remained green (`10 tests OK`, `Built target arm_ik_producer`).
