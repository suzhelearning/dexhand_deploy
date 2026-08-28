# Task 6 报告：单会话三层 HDF5 与 replay 生命周期

## 实现

- 新增 `pico_body_tianji.recording.session_h5`：建立 session v1 根 attrs、appendable/chunked 三层 `/raw`、`/target`、`/joint` 与 `/meta/session_events` 布局。
- recorder-relative `time_ns` 使用 writer 注入的本机 monotonic receive clock；外部来源时间用 `int64 source_time_ns + bool source_time_valid` 成对保存，纳秒大于 2^53 可无损 round-trip，valid=false 时 payload 不参与语义，读取还原为 `None`。
- 每条记录写入 UTF-8 `publisher_instance_id`；joint stream 写入 names/order、logical id attrs；target stream 写入 frame/source attrs；invalid mocap/H5 hand arrays 以 NaN 存储并由 valid mask 隔离。
- `SessionH5Writer` 创建时固定 `complete=false`，正常 `close()` 设置 true，每秒按 flush interval 刷新；`abort()`/异常上下文保留 incomplete 文件。`SessionH5Reader` 默认拒绝 incomplete，并递归拒绝 HDF5 soft/external link；`allow_incomplete=True` 仅供诊断恢复。
- 新增被动 `SessionRecorderNode`：按 profile `source_type` 仅订阅对应 typed raw，其余 canonical target/arm+hand command/state/SessionState 通过 Task 1 parser 解析；未知 key、raw type、topic side、router ZID 或 malformed payload 立即失败并保留 incomplete。
- 新增 `TargetReplaySource`：source-role liveliness `tj/live/source/target_replay`，通过 intent 等待权威 teleop state，按 recorded time 输出 fresh arm/hand target；pause 只冻结 recorded time/frame，wire sequence/timestamp 仍递增，结束发 return 并等待 idle + at_home + return_complete latch。
- 新增 `JointReplayNode`：同进程独立 source、producer arm/hand liveliness/token，按 recorded arm command 输出 `ArmJointProposal`，按 recorded hand command 输出 direct `HandJointCommand`，不启动 IK producer；同样支持 pause/resume、intent 与 return 生命周期。
- `recording.__init__` 导出 writer/loader、recorder 与两种 replay 节点；保留 acquisition v4 H5 loader，不向旧 H5 写 session 数据。

## TDD / 实际验证

1. RED：
   - `PYTHONPATH=src/pico_body_tianji pixi run python -m unittest tests.test_session_h5`
   - 首次因 `pico_body_tianji.recording` 尚不存在而导入失败，随后实现最小 HDF5 API。
   - `PYTHONPATH=src/pico_body_tianji pixi run python -m unittest tests.test_session_recorder`
   - 首次因 `recording.recorder` 尚不存在而导入失败，随后实现严格 recorder。
2. GREEN focused：
   - `PYTHONPATH=src/pico_body_tianji pixi run python -m unittest tests.test_session_h5 tests.test_session_recorder tests.test_session_replay`
   - 输出：`Ran 8 tests in 0.069s`，`OK`。
   - 覆盖三层布局、chunk/maxshape、根 attrs、receive timeline、nullable source time、complete/incomplete recovery、external/soft link rejection、raw profile filtering、strict typed recording、overwrite rejection、joint/target replay、pause wire freshness、recorded target sequence、liveliness token。
3. Python 语法：
   - `PYTHONPATH=src/pico_body_tianji pixi run python -m py_compile src/pico_body_tianji/pico_body_tianji/recording/__init__.py src/pico_body_tianji/pico_body_tianji/recording/session_h5.py src/pico_body_tianji/pico_body_tianji/recording/recorder.py src/pico_body_tianji/pico_body_tianji/recording/replay.py`
   - 输出为空，无语法错误。
4. 空白检查：
   - `git diff --cached --check`
   - 输出为空，通过。

## 未完成 / concerns

- `run_session.sh`、`target_replay_sim`/`joint_replay_sim` CLI 的统一 launcher、`--record` exit 2、预分配 instance/token、profile preflight 与反序清理属于 Task 8，尚未在本任务重复实现。
- recorder/replay 已提供稳定 Python API，但尚未接入完整多进程 managed-router launcher；未运行 full suite、router process E2E 或真实设备。
- 物理 Marvin/Wuji、急停、servo disable、实体 feedback/hand zero 验收必须按 Task 9/10 runbook 执行，本任务没有宣称通过。
- 运行旧 `tests.test_h5_replay` 时，当前并行 Task 3 的 `sources.mocap` 可选 `pico_input` 依赖在本地环境缺失，导致导入失败；Task 6 没有修改 acquisition v4 loader 或旧 H5 replay 代码。
## Round 1 修复与验证

- 修复 `/raw/mocap_live` 写入路径：该布局严格不含 generic `sequence`，仅保存 `stream_instance_id/stream_sequence/frame_index`；`append_raw_mocap` 不再访问不存在列。
- `source_time_ns` 全部改为 HDF5 `int64`，并用 `source_time_valid` 表达 nullable；writer 使用注入 clock 生成 receive-relative timeline。
- reader 打开时递归拒绝 soft/external link，并验证固定路径、dataset dtype/shape/chunk/maxshape、每组行数、根 attrs、frame id、optional `wuji2_joints` layout 与合法 source type。
- replay 使用 strict Task 1 parser；Target/Joint 接入可选 `SessionClient` 的 subscriber/query authority 同步，持续 heartbeat；Target hand path 为 retarget，Joint hand path 为 direct。liveliness key 现在携带 instance id；Joint 为 source、producer arm、producer hand 分别声明 token/status。
- coordinator state 为 `returning` 或 `fault` 时立即停止 replay 输出；fault 锁存且拒绝新的 start。pause 期间只冻结 recorded clock/frame，tick 仍发送 fresh status/wire；return 需 idle、at_home、return_complete 三者闭环后再 armed，支持下一次 session。

RED/GREEN：

```text
PYTHONPATH=src/pico_body_tianji pixi run python -m unittest tests.test_session_h5.SessionH5Test.test_mocap_valid_and_invalid_rows_keep_big_source_time_exact
→ 首次失败：append_raw_mocap 访问不存在 raw/mocap_live/sequence。
→ 修复后：Ran 1 test ... OK

PYTHONPATH=src/pico_body_tianji pixi run python -m unittest tests.test_session_h5 tests.test_session_recorder tests.test_session_replay
→ Ran 9 tests ... OK

PYTHONPATH=src/pico_body_tianji pixi run python -m py_compile src/pico_body_tianji/pico_body_tianji/recording/__init__.py src/pico_body_tianji/pico_body_tianji/recording/session_h5.py src/pico_body_tianji/pico_body_tianji/recording/recorder.py src/pico_body_tianji/pico_body_tianji/recording/replay.py
→ 输出为空，无语法错误。

git diff --cached --check
→ 输出为空。
```

本轮仍未实现 Task 8 launcher/CLI 的 `--record` 拒绝与 managed-router 多进程 wiring；未执行完整 suite、实体设备、急停、servo/feedback/hand-zero 物理验收。
## Round 2 修复与验证

- 修复 TargetReplaySource 的 concrete `publish_status`，每个 `tick` 在 `armed/start_pending/replaying/returning/fault` 发布 typed source heartbeat；JointReplayNode 每 tick 发布 source、producer_arm，启用 hand 时另发 producer_hand status。
- replay start/return 现在要求 `SessionClient.startup_ready` 三 query barrier；同步 `SessionClient.poll()`、`start_authorized`、`return_completion_fresh`，超时/拒绝可观察地回到 armed，coordinator 主动 returning 停止输出并等待 idle，fault 锁存且拒绝新 start；每次 return 清除旧 at_home/return_complete，避免复用上一轮 latch。
- liveliness key 追加对应 publisher instance；JointReplay 仅在 active hand domain 存在时声明 hand producer token，并使 token logical id 与 status 一致。
- reader 严格验证 raw H5 parent 与两侧 hands 行数、optional `wuji2_joints` 形状、side/frame/source/joint_names/logical_id attrs、固定 dataset set、dtype/shape/chunk/maxshape/root attrs；session events 不含未批准的 `sequence` 列。
- 新增真实 `RawMocapLiveSample` valid/null row 测试，验证 `/raw/mocap_live` 不访问不存在的 generic sequence、invalid NaN mask，以及 `2**60+17` source timestamp int64 无损 round-trip；增加 hand command/state attrs 与 velocity-valid assertions、target status heartbeat assertions。

验证命令：

```text
PYTHONPATH=src/pico_body_tianji pixi run python -m unittest tests.test_session_h5 tests.test_session_recorder tests.test_session_replay
→ Ran 9 tests ... OK

PYTHONPATH=src/pico_body_tianji pixi run python -m py_compile src/pico_body_tianji/pico_body_tianji/recording/__init__.py src/pico_body_tianji/pico_body_tianji/recording/session_h5.py src/pico_body_tianji/pico_body_tianji/recording/recorder.py src/pico_body_tianji/pico_body_tianji/recording/replay.py tests/test_session_h5.py tests/test_session_recorder.py tests/test_session_replay.py
→ 输出为空，无语法错误。

git diff --check
→ 输出为空。
```

仍未完成 Task 8 launcher/CLI `--record` exit 2 与 managed-router 多进程接线，亦未执行 full suite、实体设备、急停、servo/feedback/hand-zero 物理验收。
## Round 3 修复与验证

- replay active `SessionClient` authority：SessionClient 清掉 start intent（拒绝/timeout）时 `start_pending` 原子回 `armed`；任何 phase 收到权威 fault 都进入 fault-lock 并拒绝后续 start；外部 returning 停止输出、idle 后重新 armed。
- replay constructors 强制 arm 与 hand active/inactive 集合分别恰好覆盖 left/right；无 active hand 时不声明 hand producer token。
- SessionH5Reader 收紧 root attrs/group set、complete 必须 bool、合法 source_type/router/robot attr、严格 UTF-8 vlen string dtype；固定 dataset set、H5 parent/hands 行数、optional joints、target source/frame/side 与 canonical joint_names/logical_id/order 均校验。
- 增补 RawH5 optional hand joints/parent rows 测试与 source heartbeat/producer heartbeat 观察断言。

验证：

```text
PYTHONPATH=src/pico_body_tianji pixi run python -m py_compile src/pico_body_tianji/pico_body_tianji/recording/session_h5.py src/pico_body_tianji/pico_body_tianji/recording/replay.py tests/test_session_h5.py tests/test_session_replay.py
→ 输出为空。

PYTHONPATH=src/pico_body_tianji pixi run python -m unittest tests.test_session_h5 tests.test_session_recorder tests.test_session_replay
→ Ran 10 tests in 0.140s，OK。

git diff --check
→ 输出为空。
```

仍未完成 Task 8 launcher/CLI `--record` exit 2、managed-router 多进程 wiring，以及 Task 9/10 实体设备和物理安全验收。
