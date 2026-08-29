# Task 9 自审报告

## 交付内容

- `src/pico_body_tianji/config/validation/test_matrix.yaml`：18 个固定 case、profile、required devices/capability、active sides、hand mode、velocity/acceleration ratio、duration、prerequisite、dangerous/controlled stop criteria。
- `scripts/validation/run_case.py`：`--list`、matrix schema/preflight、real `--confirm-real` 与 prerequisite gate、唯一 validation supervisor instance、manifest、status/operator events、日志、Session HDF5 bundle、checksums、显式且锁存的 SafetyStopSupervisor。`--fake --headless` 只生成可分析的 schema smoke，operator outcome 固定 `aborted`，不伪造设备通过，不自动按键或运动。
- `scripts/validation/analyze_runs.py`：固定 bundle/checksum、manifest/schema、当前 canonical config/runtime/ACL hash、status/ack/run id、operator result/event、session-v1 HDF5 严格校验，并输出 rate/drop、joint step/velocity、saturation/rejection、tracking/Home/hand-zero/fault/soft-stop/operator-event 汇总到 `analysis.json`/`analysis.md`。
- `docs/validation/README.md`、`01_preflight.md`–`06_result_handoff.md`：G0–G4 阶段门、设备矩阵、完整命令、步骤、topic/status/实体动作、立即停止、记录和通过判据；明确无物理设备只交付 aborted smoke。
- `pixi.toml`：新增 `validation-run` 与 `validation-analyze` tasks。
- `tests/test_validation_tools.py`：matrix/list、fake/headless bundle、分析器 checksum/schema/status/hash fail-closed、real confirm gate、缺 ack lockout focused tests。

## TDD 与验证证据

先写 RED 测试并运行：

```text
python3 -m unittest tests.test_validation_tools
FFFEFE ... FileNotFoundError/ModuleNotFoundError（matrix/run_case 尚未实现）
```

实现后运行：

```text
pixi run python -m unittest tests.test_validation_tools
......
Ran 6 tests in 2.407s
OK

pixi run validation-run -- --list
acquisition_live ... fault_recovery_real

rm -rf /tmp/tianji-validation-task9 && pixi run validation-run -- --case pico_sim --output /tmp/tianji-validation-task9 --fake --headless
/tmp/tianji-validation-task9/20260829T021932Z_pico_sim_2106c801

pixi run validation-analyze -- /tmp/tianji-validation-task9
{"bundles": 1, "run_ids": ["20260829T021932Z_pico_sim_2106c801"]}

rm -rf /tmp/tianji-validation-danger && pixi run validation-run -- --case pico_sim --output /tmp/tianji-validation-danger --fake --headless --danger-stop collision_risk && pixi run validation-analyze -- /tmp/tianji-validation-danger
{"bundles": 1, "run_ids": ["20260829T021952Z_pico_sim_519caa5e"]}

pixi run python -m py_compile scripts/validation/run_case.py scripts/validation/analyze_runs.py
（无输出，exit 0）

bash -n scripts/run_session.sh scripts/common.sh scripts/run_source.sh scripts/run_producer.sh scripts/run_executor.sh scripts/test.sh
（无输出，exit 0）

git diff --check
（无输出，exit 0）
```

还验证了：

- `marvin_pico_real_10pct` 未提供 `--confirm-real` 时 exit 2 并打印 `real validation requires explicit --confirm-real`；未尝试连接 router/设备。
- fake danger stop 的 status 记录 `expected_executor_ids == acked_executor_ids`、`ack_complete=true`、`new_motion_commands_after_stop=false`、`lockout=true`，分析器成功；缺 ack 的 `SafetyStopSupervisor` 保持 `locked=true` 并返回 ack failure。
- analyzer 对 checksum 行、manifest schema、status schema、manifest config hash 的篡改均返回非零；不生成误导性分析结果。
最新 focused 证据（包含显式 identity handoff 与 capture 文件）：

```text
PYTHONPATH=src/pico_body_tianji pixi run python -m unittest tests.test_validation_tools
........
Ran 8 tests in 2.862s
OK

rm -rf /tmp/tianji-validation-task9 && pixi run validation-run -- --case pico_sim --output /tmp/tianji-validation-task9 --fake --headless && pixi run validation-analyze -- /tmp/tianji-validation-task9
.../20260829T024004Z_pico_sim_479abf8d
{"bundles": 1, "run_ids": ["20260829T024004Z_pico_sim_479abf8d"]}

PYTHONPATH=src/pico_body_tianji pixi run python - <<'PY' ... instance-handoff-ok
```


## 物理验收状态

本次没有 Marvin、Wuji、PICO、Motive 或可执行设备结果，未进行物理动作，未声明任何 real case 通过。fake/headless 产物仅证明 bundle/schema/safety 记录链路，`operator_result.outcome=aborted`。真正验收必须按 runbook 由操作者在设备现场完成并重新运行 `validation-analyze`。

## Review round1 修复

- prerequisite gate 改为扫描 manifest `case_id`，并调用 analyzer 完整验证 checksums/schema/current config/runtime/ACL hash 与 operator outcome；目录名或手工 outcome 不再绕过。
- run_case 增加受控 `--operator-outcome pass|fail|aborted`，要求显式 operator event，fake 始终 aborted；managed children 继承准确 instance handoff 与 IK backend；child 非零退出向上传播。
- replay profile 不再强制 `--record`，也不在缺失 recording 时制造 complete 空 HDF5；acquisition observation 同样不写空 session。
- analyzer 的 tracking 使用 command/state 最近时间配对，hand zero 使用 Wuji zero/tolerance，proposal rejection/fault/soft-stop 来自 status；无记录时标为 `unavailable` 而非伪造零值。
- 增加 managed liveliness/protocol capture 文件和每个 child log 收集基础；危险 stop 保持 locked、要求 matching ack；manifest fake 模式只记录实际 supervisor。

Round1 验证：

```text
PYTHONPATH=src/pico_body_tianji pixi run python -m unittest tests.test_validation_tools
........
Ran 8 tests ... OK

pixi run validation-run -- --list
# 18 fixed IDs

pixi run validation-run -- --case pico_sim --output /tmp/tianji-validation-round1 --fake --headless
pixi run validation-analyze -- /tmp/tianji-validation-round1
{"bundles": 1, ...}

pixi run validation-run -- --case pico_sim --output /tmp/tianji-validation-round1-danger --fake --headless --danger-stop collision_risk
pixi run validation-analyze -- /tmp/tianji-validation-round1-danger
{"bundles": 1, ...}

pixi run validation-run -- --case target_replay_sim --output /tmp/tianji-validation-replay --fake --headless
pixi run validation-analyze -- /tmp/tianji-validation-replay
{"bundles": 1, ...}; session.h5 不存在

pixi run python -m py_compile scripts/validation/run_case.py scripts/validation/analyze_runs.py
bash -n scripts/run_session.sh
git diff --check
```
## 跨任务风险与最终阻塞项

1. `run_session.sh` 已接入 validation 的显式 handoff：run_id、source、arm producer/executor、coordinator、recorder，以及每侧 hand producer/executor 均由环境变量传入；run_case managed manifest 使用同一批 UUID，fake bundle 只记录实际 validation supervisor，不记录未启动 child 的随机 ID。仍需最终 managed router smoke 证明 children 实际 status/liveliness 与 manifest 一致。
2. managed router/Zenoh、SessionRecorder 的 recorder teardown、headless MuJoCo config、diagnostic calibration instance 和 overlay 第二 authority 需要在最终进程 smoke 中实测；本次 fake smoke 未绕过安全边界，也没有把未测项标 pass。
3. Marvin reconnect race 需要在 fault/returning 实机或 fake SDK 进程中验证：`fault_return` 只能消费 bounded Home，Home 后仍 fault，重启前不得 teleop。当前 runbook 已把它列为立即停止/失败判据，但本次没有设备证据。
4. analyzer 依据当前工作树 canonical config/runtime/ACL hash；生成 bundle 后修改这些文件会按设计失败，必须用同一代码/config hash 重跑 case。
5. managed danger-stop transport 在未捕获真实 executor/SDK 证据时将 `new_motion_commands_after_stop` 标为未验证；analyzer 会拒绝该结果，而不是写入合成的 false。fake/headless 仅使用确定性的 fake executor 证据，并保持 `aborted`。
