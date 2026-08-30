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
