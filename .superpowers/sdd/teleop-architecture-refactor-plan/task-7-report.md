# Task 7 自审报告

## 交付内容

- 新增 `pico_body_tianji.producers.policy.contracts`：
  - `PolicyObservation`、`PolicyAction`、`PolicyRunner`。
  - `ObservationBuilder`：executor state freshness、future/stale 拒绝；velocity 缺失时使用相邻 position/timestamp 有限差分；首帧或差分间隔越界时返回 not-ready。
  - `ActionAdapter`：14 维 `left Joint1..7 + right Joint1..7` 顺序，支持 `absolute_position_rad`、`delta_position_rad`、`velocity_rad_s`；finite、shape、robot limits、maximum step 检查；输出 canonical 左右 `ArmJointProposal`，不创建 final command。
  - `HoldPolicyRunner`：输出最新 observation position 作为 absolute hold action。
- 新增 `PolicyProducerNode`：只注册 `hold` runner，订阅 typed arm state/session state/arm targets，发布 typed producer status 与 arm proposal；没有 `tianji/command/arm/*` publisher；模型/状态/action 异常时 ready/healthy 正确降级且不发 malformed placeholder。
- policy producer 使用 launcher 传入 `TIANJI_COMPONENT_INSTANCE_ID`、`TIANJI_ROUTER_ZID`，注册 `tj/live/producer/arm/<logical_id>/<instance_id>`，不生成匿名运行时身份。
- 新增 `config/producers/policy_hold.yaml`、`scripts/policy_hold_producer`，并将 wrapper 纳入 CMake Python 程序安装清单。
- 新增 focused 行为测试，包含三种 action mode、shape/nonfinite/limit/step、velocity 估计、stale/gap、hold 输出、typed proposal/no final command、policy proposal→coordinator final command path。

## RED 证据

命令：

```text
PYTHONPATH=src/pico_body_tianji python3 -m unittest tests.test_policy_producer -q
```

实际输出：

```text
ImportError: ... ModuleNotFoundError: No module named 'pico_body_tianji.producers'
FAILED (errors=1)
```

这是新 policy package 尚不存在时的预期 RED。

## GREEN 证据

命令：

```text
PYTHONPATH=src/pico_body_tianji pixi run python -m unittest tests.test_policy_producer -q
```

实际输出：

```text
Ran 9 tests in 0.015s
OK
```

命令：

```text
PYTHONPATH=src/pico_body_tianji pixi run python -m unittest tests.test_policy_producer tests.test_arm_coordinator tests.test_task5_executor_contract -q
```

实际输出：

```text
Ran 44 tests in 0.130s
OK
```

命令：

```text
PYTHONPATH=src/pico_body_tianji pixi run python -m py_compile src/pico_body_tianji/pico_body_tianji/producers/policy/contracts.py src/pico_body_tianji/pico_body_tianji/producers/policy/node.py
git diff --check
```

实际输出：无输出，命令成功。

## 未完成项 / 跨任务边界

- 未修改 `ArmIkSolver`、coordinator 状态机或 MuJoCo executor；本任务通过 canonical proposal 接入已有实现。完整 router/ACL、launcher process lifecycle、session HDF5、replay、Wuji/Marvin 和 validation 工具属于批准计划的其它 Task。
- 未执行 project-wide build/lint/full suite；按任务要求只运行 policy/coordinator/headless focused tests、`py_compile` 与 `git diff --check`。
- 未进行真实 Marvin/Wuji/PICO/Motive 物理验收；需由操作者按 validation runbook 采集设备证据。

## Round 1 修复与证据

- 修复 CMake `install(PROGRAMS ...)` 的 `DESTINATION lib/${PROJECT_NAME}`。
- 修复 source checkout 与 installed bundle 的 policy/coordinator config 路径。
- `ObservationBuilder` 以 `(executor publisher_instance_id, sequence)` 识别重复 frame；fresh duplicate 复用最近 observation，不再因 `delta_ns=0` 错误降级。
- 修复 arm target 身份读取为 `ArmTargetCommand.envelope.*`，并保留 target sequence。
- `PolicyProducerNode` 对 coordinator instance、SessionState identity/sequence/malformed 做 fail-closed；非法快照清掉 teleop state，直到接受新权威快照或重启。

命令：

```text
PYTHONPATH=src/pico_body_tianji pixi run python -m unittest tests.test_policy_producer tests.test_arm_coordinator tests.test_task5_executor_contract -q
```

实际输出：

```text
Ran 47 tests in 0.130s
OK
```

配置路径 smoke：

```text
PYTHONPATH=src/pico_body_tianji pixi run python -c 'from pico_body_tianji.producers.policy.contracts import ActionAdapter; print(ActionAdapter().maximum_step_rad)'
0.0132645022
PYTHONPATH=src/pico_body_tianji pixi run python -c 'from pico_body_tianji.producers.policy.node import _load_policy_config; print(_load_policy_config())'
{'policy': 'hold', 'rate_hz': 90.0, 'stale_timeout_s': 0.2, 'maximum_step_rad': 0.0132645022, 'capabilities': ['simulation']}
```

另行运行 `py_compile` 与 `git diff --check` 均无输出且成功。
