# SPARK Headroom 移植记录与离线一致性证据

日期：2026-09-08。状态：**原生算法闭包、离线对照及coordinator/MuJoCo离线接入已实现；实时profile尚未接通，不是双输入遥操整体完成报告。** 未提交或推送代码。

## 来源及隔离边界

- 参考：`pico-manus-teleop/TJ_arm_control`，提交 `c022b17789e9b81141915662b3bb802f4c20a396`。
- 算法：`spark_upper_qpoases_headroom_feedforward_velocity_qp`，双臂同周期、200 Hz、`model_reference`。
- `src/tianji_teleop/src/ik/spark_headroom/source_manifest.json` 固定32个核心实现、34个传递头文件及8个原版测试文件的原始/迁移 SHA256。机械迁移仅替换命名空间及 include 前缀 `tianji_qp_ik` → `tianji_spark`；测试反向替换并检查原始摘要。
- 新增 `bilateral_cycle.cpp` 按原版 Viewer 顺序接入接纳、接管、guidance、控制器和 Headroom。原始数据接收门控另在 Python 中逐帧对照，控制周期只消费最新帧，不改为 FIFO。
- 单独 `tools/spark_native/pixi.toml` 和原版锁文件，Pinocchio 3.9.0、MuJoCo 3.10.0、qpOASES 3.2.2、libosqp 1.0.0、yaml-cpp 0.8.0；Ruckig 0.19.4。不修改主工程依赖，不与现有 Pinocchio 后端混链。
- libosqp 包的 CMake version 文件标成 0.0.0，故 CMake 使用 CONFIG 查找，不用虚假的 EXACT 1.0.0 检查；实际包版本由原版锁文件固定。
- `assets/spark` 复制原版 XML/URDF；XML 仅调整 meshdir，复用经比对一致的现有网格。原模型、原后端及旧会话默认值不改。
- 尚需完成五工程许可证/分发闭包审核；本记录不替代该审核。

## 原版 oracle 的独立性

`tests/cpp/spark/reference_viewer_cycle.hpp` 保留原版 Viewer 控制循环中328行源码原文，外部适配只提供确定性时钟、最新帧槽和非图形诊断。可选 `spark_reference_worker` 编译参考目录的原始类和命名空间，不调用迁移版 `NativeSparkCycle`。测试检查该原文及参考提交。

两端使用同一记录接收时钟、5 ms tick、到达时间不大于 tick 的帧先接收再取最新、末尾250 ms无新输入。`--deterministic-test` 仅将求解墙钟预算放宽为3600秒，迭代数和算法参数不改。**实时预算、线程调度及硬件端到端延迟没有因此通过验收。**

JSONL 包含输入/配置/模型/URDF/worker摘要、模型语义及全部mesh摘要，每周期结果以及带结果摘要的完成尾记录。缺尾记录、摘要不符或离散状态不同均拒绝。容差固定为 q/速度1e-5、加速度1e-4、TCP位置/姿态1e-6、任务/Headroom缩放1e-9；本次所比较字段实际误差均为0。

## 已执行结果

| 输入 | 原始帧数 | 控制周期 | 门控拒绝 | 对比结果 |
| --- | ---: | ---: | ---: | --- |
| `output_continuity_retest.tjvr` | 2444 | 5478 | 2 | 所输出字段一致，最大差值0 |
| `output_20260827_204546.tjvr` | 27054 | 60248 | 8 | 所输出字段一致，最大差值0 |

两个回放的控制失败和预算耗尽计数均为0（仅限上述确定性模式）。短录制有432个接管周期，左右臂最大单关节运动范围分别约3.057/2.368 rad，故并非全程静止比较。

结果行 SHA256：

- 短：`0d49f90409edd830527def4a28a6a2c1161acbdea975e82849a81b1944f0117d`
- 长：`b6c21403fe2d5085e5bcad2efc6df3c59770377c2d4d6cbbbace01a91f3d4dd2`

当前机器 JSONL 位于 `/tmp/spark-replay-Xp8FIT/`：`short-reference.jsonl` 对 `short-port-verified.jsonl`，`long-reference.jsonl` 对 `long-port.jsonl`。临时产物非仓库交付资产，可用下述命令重建。

另有基于录制姿态构造、明确标为 synthetic 的按键变化、epoch切换、跳变门控、丢失恢复序列，两端对比通过。原版 bit8 在**状态变化**时切换暂停/恢复，不应擅改成仅上升沿；新流首次按键状态仅初始化。真实录制本身不覆盖这些事件。

9组 CTest 通过。其中原始 session 测试中的两个 Viewer 源文本/UI测试明确排除；当前没有迁移该 Viewer UI，不将排除项计为通过。

## 构建及复现

```bash
pixi install --manifest-path tools/spark_native/pixi.toml --locked
pixi run --manifest-path tools/spark_native/pixi.toml configure
pixi run --manifest-path tools/spark_native/pixi.toml build
pixi run --manifest-path tools/spark_native/pixi.toml test

# 可选：需本机保留固定提交的参考目录
pixi run --manifest-path tools/spark_native/pixi.toml cmake \
  -S src/tianji_teleop/src/ik/spark_headroom -B build/spark-native \
  -DSPARK_REFERENCE_ROOT="${PICO_MANUS_TELEOP_ROOT:?set PICO_MANUS_TELEOP_ROOT}/TJ_arm_control"
pixi run --manifest-path tools/spark_native/pixi.toml cmake \
  --build build/spark-native --target spark_reference_worker --parallel 2

SPARK_NATIVE_TEST=1 SPARK_REFERENCE_TEST=1 \
TJVR_REFERENCE_ROOT="${PICO_MANUS_TELEOP_ROOT:?set PICO_MANUS_TELEOP_ROOT}/TJ_arm_control" \
TJVR_TRACE_DIRECTORY="$PWD/vendor/reference_inputs/remote-192.168.110.210-20260908" \
PYTHONPATH=src/tianji_teleop:vendor/python \
  pixi run python -m unittest tests.test_spark_native_worker \
    tests.test_spark_worker_client tests.test_spark_replay_cli \
    tests.test_spark_reference_oracle tests.test_spark_full_equivalence -v
```

对实际录制分别运行 `scripts/spark_trace_replay.py --input TRACE --output NEW.jsonl --deterministic-test`。参考端另传 `--worker build/spark-native/spark_reference_worker` 及原版 `--config/--model/--urdf`。输出路径必须未存在。最后运行 `scripts/compare_spark_reference.py REFERENCE.jsonl PORT.jsonl`。

## 接入层与剩余工作

- `ReferenceTjvrReceiver` 已实现 gate前raw记录边界、最新帧只消费一次、持久化 resynchronization_generation 和完整raw封装。尚未替换旧运行时接收器。
- `SparkWorkerClient` 提供单进程双臂调用、周期/结果关联、有限数值校验、超时和退出报错；无自动重启、无网络、无设备输出。仅接受 simulation 能力声明，且声明本身不等于获得会话授权。
- 新增双臂producer、独立后端工厂、显式会话授权及coordinator配对回执。`reference_direct`监视回执关联、超时和命令是否被修改，异常锁存暂停，不隐式重启。回执只是coordinator命令处置，不是设备反馈；`processed_guarded`尚未启用。
- 原版URDF左右臂限位不对称，新增仅供双臂仿真使用的`BilateralArmRobotConfig`，coordinator与MuJoCo均按侧校验，并使用原版左右初始关节角。旧配置格式、默认限位及profile不变。
- `tests/test_spark_coordinated_sim.py`以完整短TJVR录制验证原生SPARK→coordinator→MuJoCo：5,000多周期的双侧命令与原生q逐项相等，MuJoCo实际qpos逐项相等，最大关节运动跨度大于1rad。测试使用固定5ms时钟与deterministic预算；不是实时资格验证。
- 尚缺：实际反馈偏差监督与processed_guarded、长期实时周期、XR SDK/PC-Service 现场联调及真机验收；新 sim profile、互斥启动和 session HDF5 已接通。
- 新增接收专用`ReferenceTjvrUdp`，后台独立接收打时间戳、控制侧最新帧消费；本机UDP测试通过，真实PICO_tracker未在本机联调。新增HDF5 1.2及被动recorder支持TJVR完整raw；不改变旧1.0/1.1含义。
- 本阶段通过说明已比较的机械臂算法路径一致，不说明两套设备联合遥操或真机整体一致。Manus `.pkl` 与 TJVR 不同步，不能拼成联合硬件验收数据。

## 本轮收尾复核

顺序自查（未使用子智能体）补齐了IPC不完整末行、未来接收时间、重复JSON字段、非有限诊断值及空对比trace的拒绝测试。原版oracle的传输层同步加相同输入校验，原始Viewer控制块不变。

最新全量Python发现集开启SPARK/原版协议/官方手部可选环境后：521项，520通过、1跳过；该跳过项为旧IK ELF依赖隔离检查，随后显式指定 `build/ik-legacy-review/arm_ik_producer` 单独运行所属5项测试，全部通过。9组原生CTest再次通过。测试日志中的人为注入flush失败、追踪丢失及soft stop属于异常路径测试，并非真实设备运行。

上述521项为前一阶段结果。本阶段新增接入后，全量Python发现集开启SPARK_NATIVE_TEST和WUJI_REFERENCE_TEST：552项，546通过、6跳过；跳过的可选检查不算通过。

最后一轮显式开启SPARK原版oracle、TJVR原版C++对照、官方手部环境及旧IK ELF检查后，全量569项全部通过、无跳过（172.071秒）；包含原子不覆盖创建、Manus回调记录、显式启动门控及两种离线CLI录制检查。9组原生CTest再次全部通过；受保护PICO2 v131 smoke通过。

Git HEAD仍为 `63e1e0f`；本阶段已修改coordinator、MuJoCo执行器及录制层，新增双臂逻辑由显式配置启用，默认路径保持兼容。没有提交、推送、启动真实设备或改动旧启动入口。`git diff --check`只涵盖已跟踪差异；机械迁移文件另由源码摘要验证，不能把该命令当作所有未跟踪源码的格式审查。
