# Task10 全链路验证与最终清理报告

## 本次修复

- `PicoControllerSource` 修复 control-loop interval 未定义、启动请求异常原子回退，并通过 typed real-capability provider 动态决定 real capability；deadman/preflight 不满足时不会发送 start/target。
- 诊断校准源统一使用 `diagnostic_mocap_calibration` identity，接入 `TargetPublisher` 发布 canonical arm target 与 typed source status；修复 `start_pending` status 崩溃，保持诊断不发布 SessionState authority。
- Marvin executor 修复 returning/fault reconnect authority race：fault 优先级不可被旧 returning/idle snapshot 覆盖，returning 只消费 coordinator bounded returning command，不再合成越权 Home command。
- C++ `arm_ik_producer` 将 final-command baseline 改为按 side 独立，合法双臂同序 command 均可接受，并补齐标准头文件。
- recorder SIGTERM 关闭顺序改为先 undeclare transport、再提交 HDF5 `complete=true`。
- MuJoCo wrapper 按 executor config 显式透传 `headless`，并在 MuJoCo config 固定有限 tracking threshold。
- validation analyzer 增强 direct-real `source_type` profile contract、same-instance cross-topic same-sequence 折叠、protocol drop/rollback 统计、active executor tracking threshold，以及 HDF5/protocol/replay command/state 的 role/logical/side/instance/router authority 过滤。
- 清理旧入口文字与旧 topic 检查，部署脚本改为 canonical allow-list 清理；旧 Wuji dry-run 测试迁移到 canonical retarget unit regressions。

## 实际执行命令与结果

1. `pixi run test`
   - 结果：失败，`test 需要可执行 zenohd 以启动受管临时 router`。
   - 原因：当前环境无可执行 `zenohd`，未绕过受管 router 要求。
2. `PYTHONPATH=src/pico_body_tianji:vendor/python pixi run python -m unittest tests.test_task3_round4 tests.test_validation_tools tests.test_task5_executor_contract -v`
   - 结果：69 tests，全部通过。
3. `PYTHONPATH=src/pico_body_tianji pixi run python -m unittest tests.test_validation_tools -v`
   - 初始结果：2 项失败，分别是新增 analyzer source_type 导入遗漏及 authority fallback；随后已修复。
4. 定向复测：`PYTHONPATH=src/pico_body_tianji pixi run python -m unittest tests.test_validation_tools.ValidationToolsTest.test_fake_headless_bundle_is_safe_and_analyzable tests.test_validation_tools.ValidationToolsTest.test_target_solved_association_requires_authorized_instances_and_unique_key -v`
   - 结果：2 tests，全部通过。
5. `python3 -m py_compile scripts/validation/analyze_runs.py scripts/validation/run_case.py src/pico_body_tianji/pico_body_tianji/sources/pico_controller/node.py src/pico_body_tianji/pico_body_tianji/diagnostics/mocap_calibration_node.py src/pico_body_tianji/pico_body_tianji/executors/marvin/bridge.py src/pico_body_tianji/pico_body_tianji/recording/session_recorder.py`
   - 结果：通过，无输出。
6. `bash -n scripts/run_executor.sh scripts/run_session.sh scripts/run_source.sh scripts/deploy_ik_runtime.sh scripts/doctor.sh && git diff --check`
   - 结果：通过，无输出。
7. clean-cutover 扫描：已对源码、脚本、测试、配置、文档及 runtime 执行废弃 token 与旧 topic 检查。
   - 结果：无匹配。
8. 旧 topic 扫描：已覆盖 `src scripts tests README.md docs pixi.toml`。
   - 修复前发现 `zenoh_util.py` 历史文档和旧 Wuji dry test；均已迁移/清理。修复后无匹配。

- `PYTHONPATH=src/pico_body_tianji:vendor/python pixi run python -m unittest tests.test_task8_config_launcher -v && python3 -m py_compile ... && git diff --check`
  - 结果：8 个 Task8 配置/launcher 测试通过；语法与差异检查通过。
- 运行时文件检查：canonical runtime 中旧入口文件/旧配置目录均不存在。
## Task10 round1 follow-up

- Parent review identified missing PICO `self._rate`, which is now restored; source entry retains explicit `router_zid` and the one-tick interval path.
- Real child environment now explicitly carries speed, yaw, deadman availability, and preflight predicates with fail-closed defaults; no profile can self-admit real capability from YAML.
- Marvin fault reconnect now requires a fresh bounded returning command even after a fault arrives during the connection wait; normal timeout still uses bounded local Home failsafe, while latched fault does not.
- HDF5 arm-command logical identity accepts the wire-level `coordinator` producer under the `arm` coordinator authority; foreign HDF5 publishers raise authority violation. Managed safety-stop evidence records captured same-tick acknowledgement, executor unhealthy/lockout, and post-stop motion evidence rather than synthesizing success.
- Canonical Wuji dry-run test now includes a process-level managed typed-transport smoke (skips honestly when router/binary is unavailable) plus finite/shape, invalid-input, and translation-invariance regressions.
- Round1 command: `PYTHONPATH=src/pico_body_tianji:vendor/python pixi run python -m unittest tests.test_task5_executor_contract.MarvinExecutorSafetyTest tests.test_validation_tools tests.e2e_wuji_hand2_dry -v`
  - Result: 37 tests executed passed; managed process class skipped because `zenohd`/router was unavailable.
- Follow-up analyzer regression: `PYTHONPATH=src/pico_body_tianji:vendor/python pixi run python -m unittest tests.test_validation_tools -v`
  - Result: 26 tests passed; `py_compile` and `git diff --check` also passed.
## Task10 round2 follow-up

- 修正 `PicoControllerSource` 构造签名，恢复 `router_zid` 并显式传给 `SessionClient`/`TargetPublisher`；`self._rate` 保持，入口 smoke 可执行。
- Wuji managed process smoke 仅在 endpoint/router 或 canonical binary 缺失时 skip；启动崩溃、连接失败、超时或没有 canonical command 均 fail。
- authority contract 增加 source intent 与 coordinator latch/control topics；SafetyStop ack 仅接受 validation supervisor 与预分配 executor instance。
- `SafetyStopResult` 保存 request/ack timestamps；managed stop 不把 coordinator wire command 当 SDK motion，缺少每 executor command-counter evidence 时 no-motion 为 unverified/fail，并按 request/ack/status 时间窗判断 same-tick unhealthy/lockout。
- Marvin fault reconnect 在 deadline 内等待 fresh bounded returning command；fault 不可降级，正常 timeout 保留 local bounded Home failsafe。
- real capability provider 改为受保护 regular-file typed attestation；环境变量仅能选择文件路径，缺失/不安全/格式错误默认 denied。
- C++ Wuji bridge now exposes a monotonic `commands_sent` diagnostic counter for real SDK-send/no-motion evidence; C++ build remains externally blocked.
- Round2 command：定向 `py_compile`、`pixi run unittest`、`bash -n`、`git diff --check`。
  - Result：38 tests passed，1 managed Wuji process class skip（endpoint unavailable）；所有语法、shell 与 diff 检查通过。

## Task10 round3 follow-up

- 恢复 C++ Wuji bridge 的 `status_sequence_` 声明，补充真实 SDK `commands_sent` 证据计数。
- Frame0 diagnostics topic 纳入 source authority contract；source intent、coordinator latch、SafetyStop control topics 均保持严格身份校验。
- managed SafetyStop 在 session executor 未 ready 时不发布且不成功；进入 stop 后等待所有 arm/hand matching ack，并按 request/ack/status monotonic 时间窗计算 same-tick evidence。
- real validation 增加 `--real-preflight-file`，要求 regular-file、run/router/supervisor-bound typed attestation；普通环境变量不能自报通过，默认 denied。
- Round3 command：定向 35 tests（validation 26、PICO entry 1、Marvin 8）全部通过；Wuji 3 canonical unit tests已通过，managed process因endpoint缺失skip；py_compile、bash -n、git diff-check通过。C++构建受zenohd/SDK环境阻塞，未虚报通过。
- C++ smoke command: `cmake --build build/task8-cmake --target wuji_hand2_bridge -j2`
  - Result: blocked before execution because `cmake: command not found`.

## 未能执行与外部阻塞

- acquisition `pixi run test` 未在本次修改中触碰；历史 ledger 记录其 151 tests 中仅有用户 dirty baseline 导致的 2 个 `tests/test_object_offset.py` offset 失败，采集仓库用户修改未覆盖。
- `pixi install --locked -e ik-build`、`build-ik`、`deploy-ik`、CMake build 未执行：本轮环境缺少受管 `zenohd`，且官方 IK SDK/设备依赖未作危险绕过。
- managed ACL router/doctor、live/IK/replay/policy/Wuji/diagnostic 进程级 E2E 未执行：缺少 `zenohd`、Motive/Marvin/Wuji 物理设备及官方 SDK 运行前提。没有物理硬件通过声明。
- validation `run_case`/`validation-analyze`真实 managed bundle 未执行；fake bundle analyzer 已在定向测试中通过，fake/headless 结果仍按设计不可声明 physical pass。

## 安全与风险说明

- 未修改 acquisition 用户 dirty/untracked 文件。
- 未扩大 physical limits、未关闭 freshness、未吞掉 fault，也未自动触发危险输入。
- 真实设备验收仍需按 `docs/validation/` gate 执行，完成后再对完整 ROOT 运行 analyzer；本报告不替代物理验证。
