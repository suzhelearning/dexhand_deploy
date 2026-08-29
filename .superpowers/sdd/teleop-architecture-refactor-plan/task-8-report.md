# Task 8 report

## 已实现

- 建立 `src/pico_body_tianji/config/` canonical tree：robot、sources、producers、coordinator、executors、recording、replay、diagnostics、9 个 session profiles；session 不保存 router endpoint/IK backend，robot config 统一 names/Home/limits/zero。
- 新增 `pico_body_tianji.config_loader`：唯一 `TIANJI_ROUTER_ENDPOINT`（默认 `tcp/127.0.0.1:7447`）、配置根解析、strict YAML fields；`open_session`/router helper exactly-one router 且关闭 multicast scouting。
- `common.sh` 使用 canonical `tj/live/**` token 查询、router ZID、logical/instance 冲突检测、UUID instance、guard；real 禁止 override/skip。
- 新增 `run_source.sh`、`run_producer.sh`、`run_executor.sh`、`run_session.sh`，预分配 component/coordinator UUID，按 recorder→coordinator→executor→producer→source 启动，启动失败有界检测并反序清理。real 必须用户显式 `--confirm-real`；replay/diagnostic recording 在 router/spawn 前 exit 2。
- replay/recorder CLI、H5 wrist、trace metrics、real readiness、joint watcher、mocap calibration diagnostics 均只读权威 state/status 或发送 intent，不发布第二 state/final command。
- CMake install、runtime deploy 清单收敛为 canonical source/producer/executor/recorder/replay/policy/diagnostics；同步 staging/runtime Python/config 并清除旧 entry；删除旧产品源码、入口、配置、测试和 wrappers。
- Pixi tasks、README、ARCHITECTURE、IK/H5/future/data-flow 已同步。

## RED → GREEN 与 Round 1 focused 证据

初始 Task8 测试在缺少 `config_loader`/新 launcher 时 ImportError/FileNotFound；实现后：

```text
PYTHONPATH=src/pico_body_tianji pixi run python -m unittest tests.test_task8_config_launcher
Ran 5 tests in 0.006s
OK
```

Round 1 修复后：

```text
bash -n scripts/*.sh
PYTHONPATH=src/pico_body_tianji:vendor/python pixi run python -m py_compile <Task8 affected modules>
PYTHONPATH=src/pico_body_tianji:vendor/python pixi run python -m unittest \
  tests.test_task8_config_launcher tests.test_session_recorder \
  tests.test_canonical_sources tests.test_arm_coordinator
Ran 23 tests in 0.064s
OK
git diff --check
```

CLI 安全拒绝：

```text
run_session --profile target_replay_sim --record ... -> exit 2: replay profile cannot be recorded
run_session --profile diagnostic_mocap_calibration_sim --record ... -> exit 2: diagnostic profile cannot be recorded: no session raw schema
```

CMake：`cmake -S ... -B build/task8-cmake` configure 通过（仅 Pinocchio Boost CMP0167 开发警告）；`cmake --build ... --parallel 2` 成功构建 `arm_ik_producer`、IK probe/worker、Wuji bridge；`cmake --install ...` 成功安装 canonical wrappers/config。

## Round 1 review fixes

已处理 review 中 P1/P2：arm executor UUID；sim Wuji `--dry-run`/real typed provider；replay active hand sides 与 canonical rate；spawn 前 domain conflict 与有界进程检测；deploy runtime/staging Python 定义、source wrapper 检查与 staging 清理；producer/executor config 透传；H5 direct `h5_direct`/joint replay instance 布线；source extra args；coordinator hand mode/sides；MuJoCo 仅显式 headless；run_session 激活 bundle；real mocap speed/yaw 0.25/0；recorder instance/token/status；diagnostic record preflight；arm executor tokens；joint watcher；real diagnostic typed readiness；Pixi real tasks 无硬编码确认；test router log mkdir；doctor canonical entry/hash/旧 entry 检查。

## parked findings 状态与限制

- Task3 trusted real preflight、PICO interval/start、viewer default、diagnostic canonical target：已纳入 canonical configs/launcher/diagnostics；真实设备 preflight 仍需 operator typed input，缺失 fail-closed。
- Task4 clean executable/runtime、profile authority、HostReadiness：已切换 canonical runtime/config。
- Task5 Marvin returning/fault fault-dominant bounded、managed router：launcher 注入可信 provider，executor 仍保持 fault 安全语义。
- Task6 replay CLI/`--record`：已实现拒绝录制和 source/producer 独立 identity。
- 未启动真实 acquisition ACL/router、未跑 full suite/formatter/lint、未做 Marvin/Wuji 物理/急停验收；H5 overlay 已提供 `--viewer` passive MuJoCo 模式，最终坐标目检仍需现场 DISPLAY。未修改外部 acquisition 仓库。

## Round 2 review fixes

继续修复审查指出的入口断链：恢复 `run_session` 实际 router ZID 赋值、arm executor UUID 和 real producer/source→Marvin 顺序；source/live/H5/replay/Wuji 的 typed provider、direct `h5_direct` identity、active/inactive hand sides、canonical replay config、executor/producers config 透传均保持在实际 child invocation；MuJoCo 仅显式 `--headless` 进入无窗口路径；run_session 在 spawn 前进行 domain conflict gate，组件启动后进行有界存活检测并由 guard 反序清理；`open_session` 明确关闭 multicast scouting。

Marvin 每 tick 刷新 typed status；MuJoCo/Marvin 声明 arm liveliness；H5 direct 声明 `producer/hand/h5_direct` token/status；Wuji real 使用 trusted typed preflight；recorder 保存 instance、recorder token/status；trace metrics、real diagnostic、joint watcher 和 H5 passive overlay 均提供实际只读运行入口。deploy 定义 runtime/staging Python 根、清理两处旧产物并只校验 source wrapper；doctor/test/Pixi 入口保持 canonical。

Round 2 实际验证：

```text
bash -n scripts/*.sh
PYTHONPATH=src/pico_body_tianji:vendor/python pixi run python -m py_compile <affected modules>
Ran 23 tests in 0.065s
OK
git diff --check
pixi run -e ik-build cmake -S ... -B build/task8-cmake ...  # configure OK (Boost CMP0167 dev warnings)
pixi run -e ik-build cmake --build build/task8-cmake --parallel 2  # OK
pixi run -e ik-build cmake --install build/task8-cmake  # OK
```

已验证 replay 与 diagnostic recording 在 router/spawn 前均 exit 2。未执行真实 acquisition ACL/router、full suite、物理 Marvin/Wuji/急停；H5 passive viewer 最终目检需现场 DISPLAY。外部 acquisition 仓库未修改。

## Round 3 review fixes

恢复 `run_session` 的唯一 coordinator/UUID/router/config/hand profile 与可选 recorder child；H5 重新导入 `MotiveFrame/MotiveFrameSource`，direct hand 发布 `h5_direct` producer token/status，并在 approaching/replaying 保持 ready，同时按可信 H5/hand/deadman/speed/yaw preflight 声明 `simulation`/`real` capabilities。上一轮 router、real 顺序/provider、config 透传、replay active args、arm/recorder/diagnostic token 与 viewer/headless 语义保持。

Round 3 实际命令：

```text
bash -n scripts/*.sh
PYTHONPATH=src/pico_body_tianji:vendor/python pixi run python -m py_compile <affected modules>
PYTHONPATH=src/pico_body_tianji:vendor/python pixi run python -m unittest tests.test_task8_config_launcher tests.test_session_recorder tests.test_canonical_sources tests.test_arm_coordinator
Ran 23 tests in 0.065s
OK
git diff --check
```

未运行真实 H5/Zenoh router 因当前环境无 operator acquisition session；validate-only 与 physical/display overlay 仍需实际 H5/设备验收。
