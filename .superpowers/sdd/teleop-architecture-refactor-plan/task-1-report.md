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
- 当前 constructor 为便于后续 producer/executor 单元构造，为部分直接 wire 类型提供显式匿名 identity 默认值；`from_dict()` 仍要求 wire 上必须有 `publisher_instance_id` 与 `router_zid`，因此外部 JSON 不存在兼容性缺口。后续生产节点必须传入 launcher 分配的真实 identity，不能依赖匿名默认值。
- 组件 capability 要求至少包含 `simulation` 或 `real`；safety request 强制 `latch=true`。Safety request 的 launcher identity 授权与 request/ack 跨消息 run-id 关联由后续 coordinator/validation 层执行，本消息 parser 只验证本消息结构和值域。
- 未运行格式化器、lint、项目级 suite 或 acquisition 测试；聚焦协议测试和语法检查均通过。
