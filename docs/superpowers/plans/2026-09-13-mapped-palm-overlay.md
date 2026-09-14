# Mapped-palm 显示隔离修复

用户已确认修复显示差异，要求保留 PICO2 与旧 SPARK＋Manus 功能。
按 [项目约定](../../../AGENTS.md) 顺序执行，不提交、不启动设备。

## 设计与范围

新增独立 `executors/mujoco/mapped_palm_overlay.py`，复用只读几何绘制与目标校验。
仅 `live_runner.py` 的 mapped 后端且启用 overlay 时选择它；旧 `SparkOverlay` 不修改。
不修改 C++ IK、映射参数、模型资产、输入采集、命令、录制格式和会话默认配置。

- 从 `native_attempt.sample` 解码已消费帧；只有其源序号、tracking epoch 与 native
  `applied_sequence/applied_epoch` 相符才更新骨架。原始接收回调不改变显示骨架。
- 执行 epoch 改变清理旧骨架；仅 teleop、输入 live 且 50ms 内显示已接纳骨架。
- 隐藏未使用的 packet 目标和未做末端基变换的骨架姿态轴，保留实际 IK 目标轴。
- 在独立 Viewer MjData 更新模型自带 target_L/R；无活动目标时放在渲染 TCP 位置。
  不修改 executor MjData、模型几何或控制状态。

## 实施与验证

- [x] 测试先复现缺少独立显示能力；增加拒帧、idle、超时、非法诊断及渲染隔离测试。
- [x] 增加执行 epoch 切换但源序号不变的回归，观察失败后修复旧骨架残留。
- [x] 实现独立 overlay 和三个 runner 接入点（选择、周期快照、渲染）。
- [x] 原生 SPARK/mapped 定向回归及完整软件回归。
- [ ] 用户重新启动后确认视觉效果；不以软件回归冒充现场验收。

定向命令：

```bash
WUJI_REFERENCE_TEST=1 SPARK_NATIVE_TEST=1 MAPPED_PALM_NATIVE_TEST=1 PYTHONPATH=src/tianji_teleop:. \
pixi run python -m unittest tests.test_spark_live_simulation tests.test_mapped_palm_native \
  tests.test_mapped_palm_overlay tests.test_spark_overlay tests.test_pico_official_smoke -q
```

本轮结果：上述定向测试 25/25 通过（26.676s），包含旧 SPARK 与新 mapped 的官方手
仿真回归；普通完整发现集 1021 项，961 项执行通过、60 项按环境条件跳过（57.750s）。
另外读取用户 `mapped_palm_arms_20260913_191755.h5` 的前 500 个带 sample 的原生周期，
验证 500 次已采用骨架显示且不出现 packet 目标。编译检查与差异空白检查通过。
未启动现场设备，未改动旧 SPARK 显示类、PICO2/VR 会话配置、IK 源码或参数，未提交。
