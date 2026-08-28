# Task 5 报告：统一 MuJoCo、Marvin、Wuji executor

## 实现

- 新增 `pico_body_tianji.executors.mujoco.MujocoExecutor`：按 arm/Wuji 唯一 config 做 qpos 名称映射与限位 containment 校验，双臂同 tick 接受 canonical rad command，发布 typed arm/hand state 与 executor status，headless control loop 及 SafetyStop 锁存/ack。
- 新增 `executors.marvin`：`MarvinExecutor`、`MarvinReadiness`、feedback→canonical rad state 适配。连接 gate 区分 source 的 trusted `real` capability 与 arm producer 的 loaded/healthy；fault 只消费 coordinator 新鲜 bounded `returning` command，不再直接跳 Home；feedback stale、state/servo/error、hard limit、tracking error 分支进入 soft-stop。
- 新增 `config/robot/wuji_hand2.yaml` 与 `WujiHandConfig`，固定 20 joint wire 顺序、rad limits、zero/tolerance；canonical 名称到 legacy URDF `*_finger_*` 名称仅在 adapter 内解析。
- 新增 Python `WujiHandExecutor`，提供互斥 `direct`/`retarget`，publisher/router/side/sequence/watchdog 校验，invalid input 不刷新 watchdog，return/fault 回零，typed hand state/status，real capability fail-closed 与 SafetyStop same-tick ack/lockout。
- 将 C++ `wuji_hand2_bridge` 切换到 canonical JSON target/command/state/status/safety topic；direct 模式不声明 command publisher，retarget 模式独占 command publisher；加入 20-joint order/finite/limit、identity、freshness、sequence 与 SafetyStop ack/lockout。
- 修复旧 Marvin bridge 的 `os` 缺失、`node` 未初始化和 fault-return 直接 Home；旧 readiness connection gate 改为 source 必须 real-capable、producer 只需 loaded/healthy。
- CMake 安装新增 MuJoCo/Wuji executor wrappers；common source exports 改为 lazy，避免 headless executor 在缺 Pico SDK 时被 optional mapper import 阻塞。

## 实际验证

1. `pixi run bash -lc 'PYTHONPATH=src/pico_body_tianji python -m unittest tests.test_task5_executor_contract'`
   - 输出：`Ran 8 tests ... OK`
   - 覆盖双臂同 sequence command、MuJoCo state、Wuji 20-joint config、direct identity/watchdog、retarget 平移不变、Marvin source/producer capability gate、fault-return bounded command、SafetyStop ack/no motion。
2. `pixi run bash -lc 'PYTHONPATH=src/pico_body_tianji python -m py_compile ...'`
   - 输出：无错误；覆盖新增 MuJoCo/Marvin/Wuji Python 模块与 CLI。
3. `pixi run -e ik-build build-ik`
   - 首次重编暴露 canonical C++ bridge namespace/解析块问题，修复后重新构建成功。
   - 最终 `pixi run -e ik-build bash -lc 'cmake --build build/ik --target wuji_hand2_bridge -j2'` 输出：`[100%] Built target wuji_hand2_bridge`。
4. `git diff --check`
   - 输出为空，检查通过。

## Task 4/Task 3 风险闭合

- Task 4 paired-command baseline：Python MuJoCo/Marvin 按 side 维护 `(instance, sequence)`，左右同 tick sequence 均可接受，不再把第二侧误判 rollback。
- Task 4 Marvin 构造崩溃：旧入口补齐 `os` 导入并在 finally 前初始化 `node=None`；canonical executor 的 constructor smoke 由 focused tests 覆盖。
- Task 4 fault-return：canonical/旧 bridge fault 分支只发送 coordinator `mode=returning` 的 bounded command，不合成 direct Home 跳变；缺失/过期 bounded command 时不运动。
- Task 4 source/producer capability mismatch：Marvin readiness 明确只要求 source `real` capability；producer status 仅要求 ready/healthy，避免把 simulation-only producer 错当 real admission。
- Task 3 trusted real preflight：Marvin/Wuji real 构造与运行时均要求 `RealCapabilityInput` 或 typed provider，必须满足 speed/yaw/deadman/preflight 全部 predicate；mapping/string 不能伪造 real capability。

## 未完成与 concerns

- 未连接或宣称通过任何实体 Marvin/Wuji；本次只验证 fake hardware/headless 可达路径。急停、servo disable、实体限位、反馈跟踪和设备网络仍需按 Task 9/10 runbook 验收。
- `run_session` 的 profile authority/token 传递、H5 20-joint real preflight scanner、trusted preflight process provider、full launcher lifecycle 仍由 Task 8/9 接入；本提交的 executor API 已提供 typed input/identity 接口，但没有绕过 launcher 自行生成 authority。
- C++ Wuji bridge 当前限位常量与 YAML 对齐但尚未在 C++ 侧加载 YAML；Task 8 应将 YAML loader 接入 C++/doctor，确保配置单一事实来源而非保留常量副本。
- C++ bridge 的 retarget target publisher identity 仍依赖 launcher 传入的 authorized instance；需要 Task 8 profile wiring 明确 source instance 与 hand producer instance 的区分后再进行真实进程验收。
- 旧 `marvin_hardware_bridge.py`/legacy wrappers 与历史产品入口仍在树中，按计划交给 Task 8/10 clean cutover 删除；本任务没有静默删除跨任务入口。
- 未运行 project-wide test/full suite、router 进程级 MuJoCo E2E 或物理设备测试；这些不属于本次 focused proof。
## Round 1 review fixes

- 修复 legacy `HostReadinessGate._base_connection` 被错误删除的问题，并恢复 legacy Marvin 入口的 `router_zid` 注入；新增 canonical `marvin_executor` product wrapper，CMake 不再安装 legacy Marvin wrapper。
- Marvin `fault_return` 每 tick 先读取并校验最新 feedback，再发送 coordinator bounded returning command；SDK 发送前拒绝 finite 越限和超过 `maximum_output_step_deg` 的跳变，保持 soft-stop 与 fault/timeout 语义分离。
- MuJoCo 持续刷新 generic status，发布双侧 `HandExecutorStatus`，并把 matching SafetyStop ack 发布到 `tianji/safety/ack/{executor_id}`。
- Wuji Python 在 returning/fault 先拒绝新输入，强制 authorized publisher instance；SessionState 要求 coordinator identity。C++ Wuji 使用 measured `latest_states()` 发布 hand state，并校验 SessionState coordinator/router/sequence/freshness。
- round 1 focused 验证：`pixi run bash -lc 'PYTHONPATH=src/pico_body_tianji python -m unittest tests.test_task5_executor_contract'` → `Ran 8 tests ... OK`；`pixi run -e ik-build bash -lc 'cmake --build build/ik --target wuji_hand2_bridge -j2'` → `Built target wuji_hand2_bridge`；新增 Python `py_compile` 与 `git diff --check` 均通过。

仍需明确的 executor-level concern：C++ Wuji 的限位数组与 YAML 数值已对齐，但 C++ 尚未直接加载 YAML；Wuji C++ retarget identity/profile wiring、真实 Zenoh 多进程 headless smoke 和实体设备验收仍未完成，不能宣称物理通过。
## Round 2 review fixes

- 修复 legacy Marvin `_LOG`/`OUTPUT_STEP_REFERENCE_VELOCITY_RATIO` 缺失、canonical bridge `main()` 与 CMake 安装入口；legacy readiness 的 router 注入和 `_base_connection` 保持可达。
- Marvin fault-return 保持 feedback-first，并接通 `HardwareSafetyController.observe_command()`、robot hard limits、maximum step 与受控 slew；fault reconnect 使用 fresh bounded returning command，不调用 startup direct Home。
- MuJoCo 增加 coordinator state/at_home/return_complete query gate；ready 前等待 query replies，control tick 持续刷新 generic status、两侧 `HandExecutorStatus` 与 SafetyStop wire ack。
- Python Wuji 在 returning/fault 不更新 pending/baseline/watchdog/qpos；强制 authorized publisher UUID，并校验 coordinator identity、state freshness/sequence；retarget 以 producer_hand role/token 发布，direct 只保留 executor role。
- C++ Wuji 接入 `TIANJI_WUJI_CONFIG` 数组 loader、SafetyStop supervisor/run identity、measured `latest_states()` freshness，measured stale 时锁存 unhealthy/disable；SessionState identity/sequence/freshness、returning/fault input rejection、retarget producer token 与 logical producer/UUID 分离完成。
- round 2 focused 验证：`pixi run bash -lc 'PYTHONPATH=src/pico_body_tianji python -m py_compile ... && PYTHONPATH=src/pico_body_tianji python -m unittest tests.test_task5_executor_contract && git diff --check'` → `Ran 9 tests ... OK`；`pixi run -e ik-build bash -lc 'cmake --build build/ik --target wuji_hand2_bridge -j2'` → `Built target wuji_hand2_bridge`。

仍未宣称实体设备通过；real launcher/provider、多进程 Zenoh E2E 及物理 feedback/servo 验收仍需后续设备阶段。C++ loader 现要求 launcher 提供 `TIANJI_WUJI_CONFIG`，不会以 fallback 常量伪造配置 authority。
## Round 3 review fixes

- 补齐 Marvin 每 tick feedback-first 的 `HardwareSafetyController.observe_feedback`、state/command/`decide` 链路，首条输出以 measured baseline 初始化；feedback frame serial 不推进即 fail-closed。fault reconnect 仅在 fresh bounded returning command 下连接，安全锁存不可由同进程 connect 绕过。
- MuJoCo 增加 per-key coordinator snapshot gate：SessionState/at_home/return_complete 各自等待 query reply 后才 ready，control tick 持续刷新 arm/hand status 与 wire SafetyStop ack。
- Python Wuji 修正 session/live token 构造与关闭、SessionState coordinator identity/sequence/freshness、idle/returning/fault 输入拒绝，并以 retarget 模式声明 `producer_hand` role/token、direct 模式保持 executor-only。
- C++ Wuji 使用 `TIANJI_WUJI_CONFIG` loader 而非 hardcoded limits，retarget 输出逐维 finite/limit 复验；SafetyStop 读取 supervisor/run identity并锁存；measured feedback 缺失时 unhealthy/disable；hand status 的 at_zero/tracking_allowed 来自同 tick measured/freshness，logical producer 与 UUID instance 分离。
- round 3 focused 验证：Python `py_compile` + `tests.test_task5_executor_contract` → `Ran 9 tests ... OK`；C++ `cmake --build build/ik --target wuji_hand2_bridge -j2` → `Built target wuji_hand2_bridge`；`git diff --check` → 空输出。

仍未执行实体设备或完整多进程 Zenoh session 验收；设备/launcher 阶段必须继续使用最新 config、identity 和 preflight，不得以本地 fake 结果代替物理通过。

## Round 4 修复与验证

### 本轮修复

- Python `WujiHandExecutor` 初始化并关闭 `_subscriptions` 与多个 liveliness token；`retarget` 同时声明 `producer_hand` / `executor_hand` token 和 typed `ComponentStatus`，`direct` 保持 executor-only。两种模式只在新鲜 `SessionState=teleop` 接收输入；coordinator state 在 control tick 过期后清空输入、回零并标记 unhealthy；direct accepted command 不再提前改写 qpos。
- `MarvinExecutor` 在 SafetyStop 锁存后于 readiness/SDK 调用前拒绝同进程 `connect()`；`fault` 与 `returning` 均只接受新鲜 bounded returning command 重连。`HardwareSafetyController` 继续完整 observe feedback/state/command/decide 链路；目标先交给 controller 做 step/slew/limits，再校验最终 SDK 输出。SessionState freshness 使用真实 callback receive time，重复 feedback serial 交由 controller timeout 判定。
- `MujocoExecutor` 将 SessionState、at_home、return_complete 改为逐 key typed snapshot barrier：每个 query 必须 exactly-one、coordinator identity/router/sequence/timestamp 合法；缺失、重复、失败或超时保持 not-ready，超时重新发完整三 key query。持续发布 arm/hand status，SafetyStop 保持冻结与 ack。
- C++ Wuji bridge/device 保存 measured state 的本机 monotonic receive time 与递增 callback serial；dry-run 也填充 measured cache。每 tick 发布基于 fresh measured 的 status，zero 使用 YAML tolerance；serial/cache stale 后锁存 unhealthy/disable。retarget 同时发布 producer/executor authority，direct 仅 executor；输入要求 fresh teleop state，SafetyStop 回调同 tick 发布 unhealthy status 与 matching ack。
- 修复 legacy `marvin_hardware_bridge` 使用未定义 `router_zid` 的入口，改为 `require_single_router(session, TIANJI_ROUTER_ZID)` 并在 node 构造失败时关闭 session。

### 回归 RED（实现前）

执行：

```text
pixi run bash -lc 'PYTHONPATH=src/pico_body_tianji python -m unittest tests.test_task5_executor_contract'
```

结果：`Ran 17 tests ... FAILED (failures=7, errors=1)`。失败覆盖预期的 Wuji `_subscriptions` 缺失、MuJoCo snapshot 误解锁/无 retry、Marvin duplicate serial 立即 stale、fault reconnect 触发 SDK、原始 step 误停、旧 SessionState 被 feedback 续鲜，以及 Wuji stale teleop 未置 unhealthy。

### Round 4 实际验证

1. focused Python executor regression：

   ```text
   pixi run bash -lc 'PYTHONPATH=src/pico_body_tianji python -m unittest tests.test_task5_executor_contract'
   ```

   输出：`Ran 20 tests ... OK`。

2. Python 语法：

   ```text
   pixi run bash -lc 'PYTHONPATH=src/pico_body_tianji python -m py_compile src/pico_body_tianji/pico_body_tianji/executors/wuji_hand2/node.py src/pico_body_tianji/pico_body_tianji/executors/mujoco/node.py src/pico_body_tianji/pico_body_tianji/executors/marvin/bridge.py src/pico_body_tianji/pico_body_tianji/executors/marvin/readiness.py src/pico_body_tianji/pico_body_tianji/marvin_hardware_bridge.py tests/test_task5_executor_contract.py'
   ```

   输出：无错误。

3. C++ Wuji build：

   ```text
   pixi run -e ik-build bash -lc 'cmake --build build/ik --target wuji_hand2_bridge -j2'
   ```

   输出：`[100%] Built target wuji_hand2_bridge`。

4. real Zenoh session constructor smoke（peer session，无需 managed router）：

   ```text
   Wuji: 3 2
   MuJoCo: False waiting_snapshot
   Marvin: waiting_for_connection False
   ```

   Wuji 实际 Zenoh session 能完成 3 个 subscriber、2 个 token 的构造与关闭；MuJoCo 在未收到三份 snapshot 时保持 not-ready；Marvin 构造保持等待连接。

5. 空白检查：

   ```text
   git diff --check
   ```

   输出：空。

### 未完成 / 不可宣称项

- 本轮没有启动 managed router、完整 `run_session`/Task 8 profile token wiring，也没有完成多进程 Zenoh query/liveliness E2E。
- C++ Wuji 只完成目标构建与代码路径核查；没有真实 Wuji SDK feedback/servo/disable 网络验收。Marvin 同样没有物理设备连接、跟踪、限位或急停实体验收。
- 以上 focused fake/headless 与 peer-session smoke 不能替代 Task 9/10 设备 gate；不得据此宣称实体执行器通过。

## Round 5 最终修复与验证

### 本轮修复

- Marvin `_admission_ok()` 在合法 typed `RealCapabilityInput` 路径明确返回 `True`；保留 SafetyStop 锁存、fault/returning bounded reconnect、feedback-first、`HardwareSafetyController` 限位/step/slew。
- Marvin reconnect 根据权威 SessionState 区分 `returning` 与 `fault`：正常 returning 重连保持 returning，并可在收到 idle 后回到 `armed_idle`；fault 重连保持锁存 `fault_return`。SafetyStop 后普通 connect 仍在任何 SDK 调用前拒绝。
- MuJoCo snapshot 使用 eclipse-zenoh 1.10 的 `reply.ok` → `reply.result` → `Sample.payload` 解包；三 key 仍要求 typed、exactly-one、coordinator/router identity 与 timestamp freshness。subscriber 或前一次尝试已收到较新 sequence 时，延迟/重试旧 snapshot 只计为该 key 满足，不覆盖当前值；重试保留 sequence baseline，缺失/重复/失败仍保持 not-ready 并重试。
- Python Wuji retarget 的 `producer_hand`/`executor_hand` ComponentStatus `component_id` 分别与 producer/executor liveliness logical id 完全一致；direct 继续 executor-only。
- C++ Wuji 在读取 `latest_states()` cache 后重新采样 monotonic 时间计算 measured age，避免 tick 开始时间把正常异步反馈误判为 future；重复 serial/断流继续 stale lockout，每 tick 发布 measured-derived status，SafetyStop callback 同轮发布 unhealthy status 与 matching ack。

### Round 5 回归 RED

先加入真实 Zenoh Reply-shaped fake（仅 `reply.ok`、`reply.result.payload`，不提供直接 `payload/get_payload()`）、subscriber 新状态覆盖旧 snapshot、retry baseline、普通 Marvin admission/connect、returning/fault reconnect、Wuji role-id 一致性回归，执行：

```text
pixi run bash -lc 'PYTHONPATH=src/pico_body_tianji python -m unittest tests.test_task5_executor_contract'
```

输出：`Ran 24 tests ... FAILED (failures=5)`；失败分别暴露 admission 无明确 True、returning 重连错误进入 fault_return、Reply 解包失败、snapshot sequence rollback、Wuji role/status logical-id 不一致。

retry baseline 回归在保留旧 snapshot 清理逻辑时单独执行：

```text
pixi run bash -lc 'PYTHONPATH=src/pico_body_tianji python -m unittest tests.test_task5_executor_contract.Task5ExecutorContractTest.test_mujoco_retry_keeps_newer_subscriber_baseline'
```

输出：`Ran 1 test ... FAILED`，实际得到 sequence `1` 而非预期 `2`；移除重试时对 `_snapshot_values` 的清理后转绿。

### Round 5 实际验证

1. focused executor regression：

   ```text
   pixi run bash -lc 'PYTHONPATH=src/pico_body_tianji python -m unittest tests.test_task5_executor_contract'
   ```

   输出：`Ran 25 tests in 0.112s`，`OK`。

2. Python 语法：

   ```text
   pixi run bash -lc 'PYTHONPATH=src/pico_body_tianji python -m py_compile src/pico_body_tianji/pico_body_tianji/executors/wuji_hand2/node.py src/pico_body_tianji/pico_body_tianji/executors/mujoco/node.py src/pico_body_tianji/pico_body_tianji/executors/marvin/bridge.py src/pico_body_tianji/pico_body_tianji/executors/marvin/readiness.py src/pico_body_tianji/pico_body_tianji/marvin_hardware_bridge.py tests/test_task5_executor_contract.py'
   ```

   输出：无错误。

3. C++ Wuji bridge：

   ```text
   pixi run -e ik-build bash -lc 'cmake --build build/ik --target wuji_hand2_bridge -j2'
   ```

   输出：`[100%] Built target wuji_hand2_bridge`。

4. 真实 Zenoh peer session constructor smoke：

   ```text
   PYTHONPATH=src/pico_body_tianji pixi run python -c 'import zenoh; from tests.test_task5_executor_contract import _FakeModel,_FakeData; from pico_body_tianji.executors.mujoco.node import MujocoExecutor; c=zenoh.Config.from_json5("{\"mode\":\"peer\"}"); s=zenoh.open(c); x=MujocoExecutor(session=s, model=_FakeModel(), data=_FakeData(), publisher_instance_id="mujoco", router_zid="peer", coordinator_instance_id="coord"); print(type(s).__name__, x.status.phase, x.status.ready); x.close(); s.close()'
   ```

   输出：`Session waiting_snapshot False`；构造/订阅/查询注册可达，未收到三份 snapshot 时保持 not-ready。

5. 空白检查：

   ```text
   git diff --check
   ```

   输出：空。

### 未完成 / 不可宣称项

- 未启动 managed router、完整 `run_session`/Task 8 profile authority wiring 或多进程 query/liveliness E2E；peer session smoke 不替代这些验收。
- 未连接真实 Marvin/Wuji 设备，未完成实体 feedback、servo-disable、限位、断网或急停 gate；C++ build 与 fake/headless 测试不构成物理通过。
