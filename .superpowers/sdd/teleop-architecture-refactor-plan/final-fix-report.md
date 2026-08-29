# Final Review 综合修复报告

## 修复范围

本波次针对 `final-review.md` 的 Critical/Important findings 完成代码修复，未恢复旧入口、旧 topic、SMPL/full-body 或 mocap step alias。

- `recording/replay_cli.py`：headless 默认先走 coordinator `request_start()`，按 `rate_hz` 调用 `tick()`，支持显式 pause/resume/return 控制，并在 return/fault 完成后退出。
- `recording/replay.py`：session HDF5 的历史 `router_zid` 仅保留 provenance，不再阻止当前 router 上的合法 replay。
- coordinator：删除重复 `_check_teleop_health()`，恢复 typed hand status、fresh hand state、instance matching、`tracking_allowed` 门控；按 active side 强制 `required_capability`。
- IK producer：C++ 读取完整 canonical `producers/ik.yaml` 的关键安全/worker 参数、control rate/freshness、source logical/instance identity；real profile status 声明 `simulation+real`；target 严格绑定 launcher source identity；source sequence baseline 按 instance/sequence 管理。
- policy producer：ArmJointState 与 arm target 绑定 launcher executor/source authority；status 与该 control tick 的左右 proposals 共享一个 sequence allocator 点。
- Marvin：readiness 接收并校验 profile authority 的 source/producer logical id、instance、router；fault reconnect 的 bounded-return 等待遵守统一 deadline。
- Wuji：real wrapper/main 只允许 native `wuji_hand2_bridge` + SDK device 路径，缺 native binary 时 fail closed，不再运行 Python memory-only no-op；native hand executor authority/live/component id 与 canonical `wuji_{side}` 对齐。
- launcher：`run_session.sh` 监控全部 child 的运行期退出并触发既有反序 cleanup；h5_real + overlay 启动 passive H5 Frame0 viewer；H5 auto hand mode 通过正式 loader 按 active side 检查 canonical 20-joint 数据、valid frame、finite limits。
- validation：danger-stop 等待 launcher completion，按 config-derived bounded window 检查每个 executor 最新 sequence 的 fresh ready/healthy status；analyzer 分块 SHA256 大文件，仅保留小型 real attestation；全局 protocol drops 进入 pass gate；sequence folding 允许同 instance 跨 topic 同 tick pair，拒绝同 topic duplicate/rollback；authority 过滤覆盖 role/logical/side/instance/router。
- tooling：`scripts/test.sh` 使用完整 unittest discovery 并显式包含 `e2e_wuji_hand2_dry`；doctor 增加 runtime canonical entry allowlist；新增 `validation-scan-real`/`scan_real_preflight.py`，只接受外部设备感知 scanner 输出并要求 root-owned 0600 artifact。
- tests：新增 `tests/test_final_review_regressions.py`，覆盖 replay lifecycle、hand tracking gate、global protocol drops、cross-topic sequence folding 和 replay router provenance。

## 验证证据

- `PYTHONPATH=src/pico_body_tianji pixi run python -m unittest tests.test_final_review_regressions tests.test_validation_tools tests.test_task5_executor_contract tests.test_policy_producer tests.test_session_replay tests.test_task8_config_launcher tests.e2e_wuji_hand2_dry`
  - 结果：`Ran 90 tests ... OK (skipped=1)`。
  - 唯一 skip：未提供 managed router 或已构建 Wuji bridge 时跳过 Wuji process smoke；若 router 已连通但进程启动/transport 失败，测试会 `AssertionError`，不会转 skip。
- `pixi run -e ik-build build-ik`
  - 结果：IK build environment check 通过，CMake/GCC/Pinocchio/Zenoh 配置通过，`arm_ik_producer`、`wuji_hand2_bridge`、formal fixture 等目标构建并安装到 staging 成功。
- `PYTHONPATH=src/pico_body_tianji pixi run python -m py_compile ...`
  - 结果：本波次修改的 Python 入口、coordinator、executor、policy、replay、validation、scanner 全部通过。
- `bash -n scripts/run_session.sh scripts/run_executor.sh scripts/run_producer.sh scripts/test.sh scripts/doctor.sh`
  - 结果：通过。
- `git diff --check`
  - 结果：通过。
- clean-cutover 搜索：`smpl/full_body/pico_controller_input/mocap_keyboard_step/host_input_mode/tianji_kinematic_sim` 无结果。
- `pixi run python scripts/validation/scan_real_preflight.py --help`
  - 结果：CLI 可发现并显示设备感知 scanner 要求。

## 未执行/外部阻塞

- 本报告未宣称物理 Marvin/Wuji、PICO、Motive 或 acquisition 设备验收通过；真实设备动作、servo feedback、急停实体行为仍需按 validation runbook 由操作者执行。
- acquisition 独立工作树保留用户已有 dirty/untracked 标定/设备现场，不执行 reset、clean 或覆盖；其已知 `tests/test_object_offset.py` offset fixture 不一致事实保持不变。
- managed external router、acquisition `pixi run test`、完整 teleop `pixi run test`、三 backend 进程 E2E、replay/policy/diagnostic 真实 router E2E、validation-run/analyze tamper/safety bundle 需在具备对应 router/runtime/device 条件的环境执行。

## 提交

本文件与本波次代码由主分支负责人统一提交一次；提交 hash 以最终 commit 为准。
