# H5 交互输入与 Frame0 MuJoCo Overlay 实施报告

## 交付范围

- `scripts/run_session.sh`
  - 只有受管 `source` 在启动器自身 stdin 为 TTY 时显式打开并继承 `/dev/tty`。
  - `source` stdout/stderr 经 `tee` 同时写入 `${TELEOP_RUNTIME_DIR}/${run_id}-source.log` 并镜像到启动终端。
  - 其他组件以及无 TTY 场景显式使用 `/dev/null` 作为 stdin，避免抢占操作按键，也避免自动化阻塞。
- `src/pico_body_tianji/pico_body_tianji/executors/mujoco/node.py`
  - 订阅 `topics.FRAME0_HAND_SKELETON`，使用 `Frame0HandSkeleton.from_dict()` 严格解析。
  - 仅接受 launcher 注入的 `TIANJI_SOURCE_INSTANCE_ID`、当前 router ZID、`frame_id=motive_world`、`side=right` 和严格递增 sequence。
  - 在 executor Home 初始化后、transport subscriber 声明前固定解析 `r_wrist_axis_{0,1,2}`，不会使用后续运动中的腕位姿。
  - 非 headless viewer 在 `viewer.lock()` 内向 `viewer.user_scn` 写入 21 个 sphere 与 payload 的 20 条 capsule edge，并受 `maxgeom` 上限约束。
  - overlay callback 只更新进程内 diagnostics cache；拒绝消息也不改变 executor health/readiness，不发布 `SessionIntent`、state/status 或 final command。
- `tests/test_h5_interaction_overlay.py`
  - PTY 下的真实受管启动边界测试；同时证明非 source 组件不能读取终端字节、source log 保留且终端可见。
  - typed payload、authority、frame、side、sequence、固定 Home 坐标变换、viewer lock、`ngeom` 上限测试。
  - 使用实际 `tianji_wuji2.urdf`、实际 MuJoCo `MjvScene` 的无窗口 geometry API smoke。

## TDD 红灯证据

命令：

```bash
pixi run bash -lc 'source scripts/common.sh && activate_bundle_runtime && python -m unittest -v tests.test_h5_interaction_overlay'
```

旧 MuJoCo executor 的 overlay 红灯（上述完整模块命令中的 3 个 ERROR）：

```text
test_subscribes_and_strictly_rejects_wrong_diagnostic_authority ... ERROR
test_valid_message_builds_fixed_home_aligned_points_and_edges ... ERROR
test_viewer_draws_clear_bounded_geometry_under_lock ... ERROR

TypeError: MujocoExecutor.__init__() got an unexpected keyword argument 'source_instance_id'
```

PTY fixture 本身修正后，单独确认 launcher 红灯：

```bash
pixi run bash -lc 'source scripts/common.sh && activate_bundle_runtime && python -m unittest -v tests.test_h5_interaction_overlay.ManagedSourceTerminalTest'
```

```text
test_interactive_managed_source_receives_byte_and_mirrors_log ... FAIL

AssertionError: False is not true : 错误：组件 source 在启动阶段退出。

Ran 1 test in 1.673s
FAILED (failures=1)
```

两个有效红灯与已定位根因一一对应：旧 MuJoCo executor 没有 Frame0 typed diagnostics 输入；旧 launcher 的后台 source 没有显式 stdin 重定向，在非 job-control shell 中得到 `/dev/null`。

## 绿灯证据

最终 focused 命令：

```bash
pixi run bash -lc 'source scripts/common.sh && activate_bundle_runtime && python -m unittest -v tests.test_h5_interaction_overlay tests.test_task5_executor_contract tests.test_task8_config_launcher.Task8LauncherTest'
```

完整通过摘要：

```text
Ran 38 tests in 6.405s
OK
```

模块拆分：新增交互/overlay 5 tests，既有 executor contract 28 tests，既有 launcher contract 5 tests。新增 actual-MuJoCo smoke 使用真实 URDF 和 `MjvScene(maxgeom=64)`，确认写入 `ngeom=41`。

实际 URDF Home wrist frame 额外探针：

```text
origin= [0.000248, -0.944, 1.123994] det= 1.0
```

`git diff --check` 在提交前执行并通过（无输出，exit 0）。

## 坐标公式

记：

- $M$：Motive world；
- $S$：MuJoCo world；
- $W_h$：Home 时的 `r_wrist` frame；
- payload 的 `robot_wrist_home_pose` 为 $T^M_{W_h}$；
- executor 在 Home 关节角 `mj_forward` 后，从 `r_wrist_axis_{0,1,2}` 得到 $T^S_{W_h}$。

固定世界变换为：

$$
T^S_M = T^S_{W_h}\left(T^M_{W_h}\right)^{-1}.
$$

实现中的旋转和平移展开为：

$$
R^S_M = R^S_{W_h}\left(R^M_{W_h}\right)^T,
$$

$$
t^S_M = p^S_{W_h} - R^S_M p^M_{W_h},
$$

$$
p_S = R^S_M p_M + t^S_M.
$$

模拟 wrist frame 的轴由三个 URDF debug geoms 恢复：`geom_xmat[:,2]` 给出 cylinder 轴；$x$ 取 geom 0，$z$ 取 geom 2，$y=z\times x$，geom 1 用于一致性校验。原点使用三个 cylinder center 减去固定半长 $0.045\,m$ 后取均值，并校验三个估计原点在 $10^{-4}\,m$ 内一致、旋转正交且 `det=+1`。

该 frame 在 `_initialize_home()` 后立即捕获；Frame0 消息到达时只使用缓存的 $T^S_{W_h}$。因此后续 arm command 改变当前 wrist FK 时，Frame0 target skeleton 不会跟随机械臂移动。

## Authority 与输入边界

- `s/r/q` 仍只由 source 内原有键盘状态机解释；launcher 只提供正确的终端 FD，不解释按键。
- MuJoCo executor 没有新增 publisher；Frame0 subscriber 不发送 intent、proposal、command、state 或 status。
- malformed、unknown field、错误 router、错误 source instance、错误 frame、左侧 payload、sequence rollback 均只拒绝 diagnostics sample；不会把 executor 标记 unhealthy。
- headless `tick()` 路径未加入 viewer 或 overlay 绘制调用；仅非 headless loop 调用 `update_frame0_viewer_overlay()`。

## 实际风险与限制

1. Frame0 overlay 当前只支持右手，因为唯一被要求和验证的模拟安装 frame 是 `r_wrist_axis_{0,1,2}`；左侧消息 fail closed。
2. URDF debug axis 的 cylinder 半长必须保持 `0.045 m`。若资产删除/改名、轴不再正交或原点不一致，overlay 会禁用并拒绝 sample，但不会影响 executor 控制、health 或 readiness。
3. viewer 的真实图形窗口未在本次无显示验证中人工目检；已用实际 MuJoCo `MjvScene`/真实 URDF 验证 21 sphere + 20 capsule 的 API、有限坐标与 geometry 数量。现场仍应确认颜色、点半径和视角是否满足目检距离。
4. Frame0 是固定 diagnostics target，不设 freshness timeout；最后一个合法高 sequence skeleton 会一直显示到 viewer 关闭或进程重启。这避免固定 frame0 因 replay 阶段推进而消失，但也意味着 source 异常退出不会自动清除图形。
5. 交互式日志镜像依赖系统 `tee`。`tee` 是短生命周期的 process-substitution companion，source 退出或受管进程组终止后随 pipe EOF 退出；实际被登记和反序停止的 authority process group 仍是 source 本身。
6. 无 controlling TTY 或 launcher stdin 非 TTY 时有意回退到 `/dev/null`，不会阻塞；这类自动化必须通过协议注入或测试 fixture 驱动状态，不能期待键盘 `s`。

## 评审修复轮 1：终端模式恢复

评审指出：source 将继承的 `/dev/tty` 切换为 raw/no-echo 后，若 launcher 因其他组件异常而以 SIGTERM 反序清理，source 的 Python `finally` 不保证执行，可能把启动 shell 永久留在 raw 模式。

修复采用 exact-state 恢复，而不是会覆盖用户自定义的 `stty sane`：

1. 仅在交互式 source 已成功打开 `/dev/tty` 后、启动 source 前，通过该同一 FD 执行 `stty -g`，保存完整编码状态；无法保存时拒绝启动交互 source。
2. `run_session_cleanup_and_release` 继续调用既有 `teleop_cleanup_and_release`，保持 common guard、受管进程组反序 TERM/KILL、记录保留和 guard release 语义。
3. 只有既有 cleanup 已停止所有受管进程后，才执行 `stty \"${saved_state}\" </dev/tty`；EXIT、INT、TERM、source 登记失败和启动阶段退出都经过同一路径。
4. 无 TTY 时状态为空，不调用 `stty`，仍显式给组件 `/dev/null`，不引入自动化阻塞。

按用户最新指示，本修复轮未继续运行测试、formatter 或 linter；代码和报告直接提交。剩余实际风险是 launcher 自身遭遇不可捕获的 SIGKILL 或宿主崩溃时无法执行任何 EXIT trap，此时只能由用户在 shell 中人工恢复终端；可捕获的正常退出、INT 和 TERM 已覆盖。

## 评审修复轮 2：仅在 source 已停止后恢复终端

正式复审指出：`teleop_cleanup_and_release` 返回非零时，`TELEOP_CHILDREN_FILE` 会保留未能停止的受管进程记录；上一轮无条件恢复终端，可能与仍存活、仍在设置 raw mode 的 source 竞争。

本轮在不重新解释 PID/start-ticks、也不改变 common 身份验证和释放规则的前提下，只读取 common cleanup 留下的失败记录：

- 若失败记录仍含精确 label `source` 且存在 saved tty state，则明确报错，保留 saved state、guard 和 children 失败记录，并拒绝执行 `stty`。
- 若 cleanup 失败仅涉及其他 label，而 `source` 已不在失败记录中，则安全恢复 exact tty state。
- cleanup 成功时 children file 已由既有 guard release 删除，正常恢复 exact state。

按用户指示，本轮未运行任何测试、formatter、linter 或其他验证命令。

## 评审修复轮 3：隔离 arm 与 hand executor authority 变量

根因是 `run_session.sh` 的 embedded Python 先把 shell 传入的 arm executor UUID 解包到 `executor_instance`，随后 hand authority rows 循环又把每行 hand executor UUID 解包到同名变量。Python 循环变量在循环结束后仍保留，因此启用 hand 时 `authorities["executor_arm"].publisher_instance_id` 被最后一侧 hand executor UUID 覆盖，coordinator 从启动起报告 `component authority mismatch for executor_arm/mujoco`。

修复将 arm 参数明确命名为 `arm_executor_config`、`arm_executor_instance` 和 `arm_executor_logical_id`，每行 hand UUID 只写入 `hand_executor_instance`。`executor_arm` 现在直接且始终引用 shell 传入的 `arm_executor_instance`；hand rows 只更新对应 side 的 `executor_hand`。logical id、router、disabled mapping 和 JSON schema 均保持不变，也不再依赖循环结束后的局部变量。

按用户要求，本轮未运行任何测试、formatter、linter 或运行场景；仅人工审查 embedded Python 参数解包、hand rows 循环和最终 JSON 构造的数据流。

## 评审修复轮 4：协调器状态序列与并发串行化

线上 `h5_sim` 的 Wuji `invalid session state: session state sequence rollback` 根因位于 coordinator：control `tick()` 与 Zenoh callback 可并发读写 `_sequence`、`_state` 及输入缓存。旧实现允许 tick 先分配 sequence，intent callback 再递增并发布 SessionState，随后 tick 使用 callback 改过的当前 sequence 发布另一批消息；start readiness rejection 还会直接复用已发布 sequence。

本轮只修改 arm coordinator：

- 增加唯一 `threading.RLock`；完整 `tick()` 与全部 Zenoh input callback 在该锁内处理，status、proposal、arm/hand state 及 coordinator state/sequence 不再交错更新。control loop 的 `sleep` 仍在 `tick()` 返回后执行，不持锁等待。
- queryable 在同一锁内取得并序列化 SessionState、`at_home`、`return_complete` 快照，再执行 reply，避免读取 tick 中途的混合状态。
- 用单一 `_next_state()` 分配 standalone SessionState sequence。authority mismatch、fault-latched response、start non-idle、start readiness rejection，以及 accepted start/return 和内部 returning/fault transition 都先递增 sequence；拒绝快照同时保存为当前 `_state`，因此 subscriber 与后续 query snapshot 对该 intent 的拒绝结果一致可见。
- 周期 tick 仍只分配一次 sequence；左右 final command、SessionState、coordinator status、`at_home` 与 `return_complete` 继续共享该 tick 的 sequence。accepted start 的 state/home/complete 和 accepted return 的 state/complete 也保持原有共享 sequence；未放宽任何 Wuji duplicate/rollback、authority、fault latch 或 readiness 检查。

人工交错推演：假设上一批已发布 sequence 为 `N-1`。tick 先获得锁，分配 `N`，在锁内完整构造并发布左右 command、state、status 和 latch 后释放；期间 intent callback 只能等待。callback 随后获得锁，start 接受或拒绝均通过 `_next_state()` 分配并发布 `N+1`，再释放；下一次 tick 才能获得锁并分配、发布整批 `N+2`。反向调度时 callback 先发布 `N`，当前 tick 再发布 `N+1`，下个 tick 发布 `N+2`。两种调度均无 duplicate、rollback 或半更新快照。

按用户要求未运行测试、formatter、linter或运行场景。未验证风险：尚未在真实 Zenoh callback 线程调度及 Wuji executor 上复测；publisher `put()` 现在位于原子发布临界区，若底层同步阻塞会延后输入 callback，但不会改变 sequence 顺序，且 control loop 不在锁内 sleep。

## 评审修复轮 5：跨通道一致 sequence 快照

复审进一步指出，只有 state 递增仍不足以满足 SessionClient 的 coordinator 全局 sequence baseline：若 state 已发布 `N+1`，但 `at_home`/`return_complete` 仍停留在 `N`，后到的 latch subscriber 或 query reply 会作为跨通道 rollback 被拒绝。另一个窗口是 queryable 只在锁内构造 payload、释放锁后才调用 `query.reply()`，因此旧 latch reply 的提交可能落在新 state publication 之后。

最小修复如下：

- `_reply_snapshot()` 现在从读取对象、JSON 序列化直到 `query.reply()` 返回都持有同一个 coordinator `RLock`；tick 或 intent publication 无法插入 query payload 与 reply 之间。
- `_next_state()` 每次递增 state sequence 时，立即以同一 sequence 和 `SessionState.timestamp_ns` 刷新 `at_home` 与 `return_complete`，布尔值默认原样保留；accepted start 显式把两者置 false，return/returning/fault 显式把 completion 置 false，原有 latch 语义不变。
- authority mismatch、fault-latched、start non-idle、readiness rejection、accepted start/return 的直接 response 都统一发布 state/home/complete 三件套；内部 returning/fault 即使在下一 tick 前被 query，也只会暴露同 sequence/timestamp 的一致快照。

人工交错推演：上一组为 `N-1` 时，若 query 先持锁，它会完整 reply `N-1` 后才允许 intent/tick；intent 随后一次生成并发布 state/home/complete `N`，tick 再发布 command/state/home/complete `N+1`。若 intent 先持锁，则三件套 `N` 完整发布后 query 才能 reply 同一 `N` 快照，tick 最后发布 `N+1`；若 tick 先执行，则整批 `N`、intent 三件套 `N+1`、下一 tick 整批 `N+2`。不存在 state `N+1` 搭配 latch `N`，也不存在旧 latch reply 在本地新 state publication 中途插入。

按用户要求仍未运行测试、formatter、linter或运行场景。剩余风险是 Zenoh 传输层在 `query.reply()` 返回后的网络投递仍可能受网络调度影响；本地 coordinator 已保证调用顺序和每个跨通道快照的 sequence/timestamp 一致，消费者现有 `(instance, sequence)` 合并规则仍负责丢弃真实网络旧包，未削弱 duplicate/rollback fault。
