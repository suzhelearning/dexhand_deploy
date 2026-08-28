# Task 1 实施报告：版本化 Zenoh topics/messages contract

## 实现文件

- `src/pico_body_tianji/pico_body_tianji/protocol/topics.py`
  - 集中定义 session/source、target、producer、coordinator、executor、safety、raw/diagnostics 以及 acquisition 固定 key。
  - 对 side、executor id 提供参数化 key helper，业务代码无需拼接 key。
- `src/pico_body_tianji/pico_body_tianji/protocol/messages.py`
  - 实现 `ProtocolEnvelope` 和全部 brief 要求的 typed message：arm/hand target、proposal、solved pose、command/state、session、latch、status、safety、三类 raw、frame-0 skeleton。
  - `to_dict()/from_dict()` 使用平铺顶层 wire envelope；解析严格拒绝缺字段、unknown field、unknown schema、非法 side/frame、错误 shape、错误 joint order、非 finite 数值。
  - arm frame 固定 `left -> Base_L`、`right -> Base_R`；arm quaternion 在构造时归一化，wire parser 要求 norm `[0.999, 1.001]`；elbow reference 要求 norm `>=1e-8` 并归一化；hand wire wrist keypoint 强制为零；invalid mocap/H5 hand 的 pose/keypoints/joints 强制为 null。
  - 统一 arm `Joint1_L..Joint7_L` / `Joint1_R..Joint7_R` 和 hand 20-joint `l_|r_` wire 顺序。
- `src/pico_body_tianji/pico_body_tianji/protocol/__init__.py`
  - 导出协议消息与 topics。
- `tests/test_protocol.py`
  - 覆盖所有消息 round-trip、topic 常量/helper、unknown schema、缺字段、unknown field、shape、side/frame、20-joint order、quaternion/elbow geometry、null source timestamp、nonfinite、invalid hand 和 raw hand records。

## TDD 证据

### RED

命令：

```bash
PYTHONPATH=src/pico_body_tianji pixi run python -m unittest discover -s tests -p 'test_protocol.py'
```

实际输出（实现 protocol 包之前）：

```text
E
======================================================================
ERROR: test_protocol (unittest.loader._FailedTest)
...
ModuleNotFoundError: No module named 'pico_body_tianji.protocol'
----------------------------------------------------------------------
Ran 1 test in 0.000s

FAILED (errors=1)
```

该失败来自待实现的协议包缺失，而不是测试拼写或断言错误。

### GREEN

命令：

```bash
PYTHONPATH=src/pico_body_tianji pixi run python -m unittest discover -s tests -p 'test_protocol.py'
```

实际输出：

```text
........
----------------------------------------------------------------------
Ran 8 tests in 0.001s

OK
```

另执行了协议源码语法检查：

```bash
PYTHONPATH=src/pico_body_tianji pixi run python -m py_compile \
  src/pico_body_tianji/pico_body_tianji/protocol/topics.py \
  src/pico_body_tianji/pico_body_tianji/protocol/messages.py \
  tests/test_protocol.py
```

该命令无输出并返回成功。

## 自审与风险

- 本任务未迁移旧 source/node，未删除旧入口，未修改 acquisition。
- parser 对 wire payload 执行 exact-key 校验，不从 topic、liveliness 或 HDF5 attrs 推断 envelope。
- raw 消息使用完整 envelope；acquisition 的 stream instance/sequence 作为 mocap live raw 的独立字段保留，供后续 source ordering 使用。
- 所有可直接构造的 wire 类型均要求显式传入 `publisher_instance_id` 与 `router_zid`，不存在匿名 identity 默认值；`from_dict()` 同样严格要求二者。
- `SafetyStopRequest.validate_authority(expected_supervisor_instance_id, expected_run_id)` 与 `SafetyStopAck.validate_for(expected_executor_id, expected_run_id)` 为 consumer 的授权/run 绑定入口；request 强制 `latch=true`。
- 未运行格式化器、lint、项目级 suite 或 acquisition 测试；聚焦协议测试和语法检查均通过。

## Round 1 审查修复

- `ComponentStatus` 补齐严格 envelope `sequence`，并在 `to_dict()/from_dict()` 统一处理。
- 删除所有可上 wire 的匿名 identity 默认值；Frame0 diagnostics 也要求显式 sequence、publisher instance 与 router ZID。
- 增加 SafetyStop consumer 的 supervisor/run、executor/run/latch 校验入口及负例测试。
- diagnostics 改为递归 JSON finite 校验；topics side helper 统一拒绝非法 side；hand wrist 不再容错改写，必须精确为 `[0, 0, 0]`；messages/topics 使用显式 `__all__`。
- 新增 quaternion `[0.999, 1.001]` 边界/外侧、elbow 阈值、错误 arm order、raw discriminator、session discriminator、nested diagnostics NaN/Inf/非JSON、constructor identity 缺失等测试。

修复后聚焦测试命令及实际输出：

```bash
PYTHONPATH=src/pico_body_tianji pixi run python -m unittest discover -s tests -p 'test_protocol.py'
```

```text
............
----------------------------------------------------------------------
Ran 12 tests in 0.002s

OK
```

修复后语法检查命令及实际输出：

```bash
PYTHONPATH=src/pico_body_tianji pixi run python -m py_compile src/pico_body_tianji/pico_body_tianji/protocol/topics.py src/pico_body_tianji/pico_body_tianji/protocol/messages.py tests/test_protocol.py
```

无输出，返回成功。

## Round 2 审查修复

- `SafetyStopRequest.validate_authority()` 取消 `expected_run_id=None` 默认值，active run 必须显式传入并始终比较，避免陈旧 run 被授权 supervisor 绕过；保留 ack executor/run/latch 绑定。
- 扩充协议测试：断言 brief 全部 topic key、消息 unknown field、explicit elbow zero、mocap/H5 source discriminator、invalid hand non-null、nested Inf/非 JSON diagnostics、除 proposal 外所有 direct-wire constructor 缺 identity。

Round 2 聚焦测试命令及实际输出：

```bash
PYTHONPATH=src/pico_body_tianji pixi run python -m unittest discover -s tests -p 'test_protocol.py'
```

```text
.............
----------------------------------------------------------------------
Ran 13 tests in 0.001s

OK
```

Round 2 语法检查命令及实际输出：

```bash
PYTHONPATH=src/pico_body_tianji pixi run python -m py_compile src/pico_body_tianji/pico_body_tianji/protocol/topics.py src/pico_body_tianji/pico_body_tianji/protocol/messages.py tests/test_protocol.py
```

无输出，返回成功。
## Round 3 审查修复

- 为所有消息类型逐一调用其 `from_dict()` 并断言 unknown field 被拒绝，覆盖 envelope、arm/hand、session/status、safety、raw 与 diagnostics。
- diagnostics 测试同时覆盖嵌套 `math.nan`、`math.inf` 和非 JSON `object()`。

Round 3 聚焦测试命令及实际输出：

```bash
PYTHONPATH=src/pico_body_tianji pixi run python -m unittest discover -s tests -p 'test_protocol.py'
```

```text
...............
----------------------------------------------------------------------
Ran 15 tests in 0.003s

OK
```

Round 3 语法检查命令及实际输出：

```bash
PYTHONPATH=src/pico_body_tianji pixi run python -m py_compile src/pico_body_tianji/pico_body_tianji/protocol/topics.py src/pico_body_tianji/pico_body_tianji/protocol/messages.py tests/test_protocol.py
```

无输出，返回成功。
