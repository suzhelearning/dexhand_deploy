# H5 默认 Viewer 改动报告

## 改动

- `scripts/run_executor.sh` 现在消费互斥的 `--viewer` / `--headless`：
  - `--viewer` 覆盖 `executors/mujoco.yaml` 的 `headless: true`，不会把 wrapper-only flag 或 `--headless` 转给 Python entry；
  - `--headless` 显式且仅一次转给 Python entry；
  - 两者同时出现以 exit 2 拒绝；显示参数用于非 MuJoCo executor 时同样 fail closed。
- `scripts/run_session.sh` 仅将 `h5_sim` 的默认显示模式改为 viewer；显式 `--headless` 覆盖该默认。其他 profile 继续由现有 executor config 决定（当前为 headless）。
- session launcher 只把解析后的显示参数传给 arm executor；H5 source 不接收 `--viewer` / `--headless`。显式 viewer 仅允许 simulation + MuJoCo executor。
- 更新两个 wrapper 的 `--help` 和 `README.md`：`pixi run h5_sim -- --h5 TAKE.h5` 默认打开 viewer；自动化命令追加 `--headless`。
- 未修改 Python MuJoCo executor、router、coordinator、IK、hand 或 safety authority/control 行为，也未新增 profile/config 分叉。

## TDD 证据

实现前运行已有 viewer override 红灯测试：

```text
PYTHONPATH="$PWD/src/pico_body_tianji:$PWD/vendor/python" pixi run python -m unittest tests.test_task8_config_launcher.Task8LauncherTest.test_mujoco_viewer_consumes_override_without_forwarding_headless
```

实际关键输出：

```text
FAIL: test_mujoco_viewer_consumes_override_without_forwarding_headless
AssertionError: '--viewer' unexpectedly found in [..., '--viewer', '--config', ...]
Ran 1 test in 0.011s
FAILED (failures=1)
```

新增 session 可执行边界测试后、实现前运行：

```text
PYTHONPATH="$PWD/src/pico_body_tianji:$PWD/vendor/python" pixi run python -m unittest tests.test_task8_config_launcher.Task8LauncherTest.test_h5_session_selects_default_viewer_and_explicit_headless
```

实际关键输出：

```text
FAIL (display_args=()): AssertionError: '--viewer' not found in arm executor launch arguments
FAIL (display_args=('--headless',)): AssertionError: '--headless' not found in arm executor launch arguments
Ran 1 test in 3.285s
FAILED (failures=2)
```

该行为测试运行真实 `run_session.sh` 解析/组装路径，只在进程启动边界替换 `setsid` 与外部进程；它同时断言显示参数不会进入 H5 source。

## 最终验证

指定 acceptance 命令：

```text
PYTHONPATH="$PWD/src/pico_body_tianji:$PWD/vendor/python" pixi run python -m unittest tests.test_task8_config_launcher.Task8LauncherTest.test_mujoco_viewer_consumes_override_without_forwarding_headless
```

实际输出：

```text
.
----------------------------------------------------------------------
Ran 1 test in 0.008s

OK

Wall time: 0.12 seconds
```

相关 launcher 测试模块：

```text
PYTHONPATH="$PWD/src/pico_body_tianji:$PWD/vendor/python" pixi run python -m unittest tests.test_task8_config_launcher
```

实际输出：

```text
...........
----------------------------------------------------------------------
Ran 11 tests in 3.392s

OK

Wall time: 3.52 seconds
```

互斥参数 smoke：

```text
$ scripts/run_session.sh --profile h5_sim --viewer --headless
错误：--viewer 与 --headless 互斥。
(exit 2)
```

不兼容 profile smoke：

```text
$ scripts/run_session.sh --profile h5_real --viewer --confirm-real
错误：--viewer/--headless 仅适用于 MuJoCo executor。
(exit 2)
```

补充自检：`git diff --check` 无输出，exit 0。按约束未运行 formatter、linter 或项目级测试。

## 风险

- 自动测试覆盖 wrapper 的真实参数解析和 session 的真实启动参数组装，但为隔离 router/设备依赖，在 `setsid` 进程启动边界使用受控替身；本轮未启动真实 Zenoh session 或实际 MuJoCo 图形窗口。
- Viewer 需要可用的图形显示/OpenGL 环境；headless 自动化必须继续显式追加 `--headless`。实际 GUI 渲染能力仍取决于运行主机。

## 修复轮 1：validation `--headless` 透传

评审发现 `scripts/validation/run_case.py:_run_session()` 已把
`args.headless` 写入 manifest，但未把对应 flag 传给 `run_session.sh`。因此
`validation-run --case h5_sim --headless` 仍会触发 H5 的默认 viewer。

新增最小行为测试，在受控 `subprocess.Popen` 边界读取 validation 实际组装的
session 命令，并同时覆盖 headless 与非 headless 两种调用。实现前红灯：

```text
PYTHONPATH="$PWD/src/pico_body_tianji:$PWD/vendor/python" pixi run python -m unittest tests.test_validation_tools.ValidationToolsTest.test_managed_h5_session_forwards_headless_only_when_requested
```

```text
FAIL (headless=True)
AssertionError: 0 != 1
----------------------------------------------------------------------
Ran 1 test in 0.015s

FAILED (failures=1)
```

修复后 `_run_session()` 仅在 `args.headless` 为真时向 `run_session.sh` 命令追加
一次 `--headless`；非 headless 命令保持不变。最终实际输出：

```text
.
----------------------------------------------------------------------
Ran 1 test in 0.015s

OK

Wall time: 0.18 seconds
```

本轮仍未运行 formatter、linter 或项目级测试。
