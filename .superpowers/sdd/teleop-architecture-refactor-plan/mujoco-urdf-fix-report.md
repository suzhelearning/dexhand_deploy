# MuJoCo URDF 路径修复报告

## 改动

- 修改 `src/pico_body_tianji/pico_body_tianji/executors/mujoco/node.py`。
- 新增私有辅助函数 `_resolve_configured_urdf()`：当 `--config` 中的 `urdf` 为相对路径时，以该配置文件对应 package root（配置路径的 `parents[2]`）为基准解析。
- 源码配置 `src/pico_body_tianji/config/executors/mujoco.yaml` 因此解析到 `src/pico_body_tianji/assets/tianji_wuji2/tianji_wuji2.urdf`；runtime 配置 `share/pico_body_tianji/config/...` 同样以其 package root 解析。
- 显式传入 `--urdf` 的路径语义未改变；绝对路径配置值也保持原样，不添加 fallback。

## 测试

执行命令：

```text
PYTHONPATH="$PWD/src/pico_body_tianji:$PWD/vendor/python" pixi run python -m unittest tests.test_task5_executor_contract.Task5ExecutorContractTest.test_mujoco_configured_urdf_resolves_from_package_root
```

实际输出：

```text
.
----------------------------------------------------------------------
Ran 1 test in 0.044s

OK


Wall time: 0.24 seconds
```

## 风险

- 路径解析依赖 canonical package config 布局（`<package-root>/config/...`），与源码和 runtime/share 安装布局一致；不兼容任意自定义嵌套配置目录。
- 未运行项目级测试、formatter、linter 或真实 MuJoCo 进程；本次仅执行指定的单个回归 unittest。
