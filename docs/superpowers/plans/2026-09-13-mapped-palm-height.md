# mapped-palm 可选 Z 标定实施计划

用户已确认：保留原版入口，新增显式 `--mapped-palm-height-calibration`；仅 Z 标定，
不改 X/Y、姿态、臂角、比例或其他路线。遵循 AGENTS.md，不提交、不启动设备。

## 已确认设计

- 仅内嵌/历史 TJVR mapped-palm 仿真后端可启用；PICO2、SPARK、XR 拒绝该选项。
- 默认路径和配置不变。新路径使用独立 simulation/control-loop 子类及标定模块。
- `c` 仅在健康 idle/Home 开始双侧 2 秒稳定采样；采集有效不同帧，缺帧/移动/epoch
  变化使本次失败，保留上次成功值。未成功或正在采集中禁止 start。
- 参考高度来自当前 mapped MuJoCo 模型 q=0 的 hand_tcp_frame_L/R，只计算 FK，不命令伸臂。
- 校准成功在 Home 做显式模型状态重置，再配置 C++ 适配层 Z 偏移；映射为
  z_target = z_mapped + z_horizontal_tcp - mean(z_mapped_calibration)。
- 不改原始 TJVR 字节、接收门控和 vendored IK 核心；偏移应用于掌心参考生成之前。
- IPC 仅在原生 tick=0 时接受标定配置；每周期输出偏移诊断，操作审计记录采样均值、
  模型参考、偏移、执行代次。保留原始 raw 数据，校准路线不声明原版数值等价。
- 标定仅本次 session 有效；Home/rearm 保持已成功偏移，重新启动需重新 c。

## 验证步骤

- [x] 先测试 CLI 隔离及原版默认不变。
- [x] 先测试新鲜度、稳定采样、左右独立 Z、失败保留、未标定 start 拒绝。
- [x] 测试原生默认与非零偏移对照：只改变目标 Z、tick>0 拒绝配置、reset 保留偏移。
- [x] 测试新会话 c/s、Home 校准 barrier、原始字节和 HDF5 标定/reset 诊断。
- [x] 更新操作说明并跑原版/PICO2/SPARK 回归；现场验收留给用户。

## 软件验证结果

- 专项：`WUJI_REFERENCE_TEST=1 SPARK_NATIVE_TEST=1 MAPPED_PALM_NATIVE_TEST=1`
  下包含高度标定、原生 mapped、SPARK 仿真/官方手、PICO2手势、配置与录制的 60 项通过（51.979s）。
- 全量发现集 1029 项：967 项执行通过、62 项按环境条件跳过（60.351s）。
- mapped 原生构建成功，CTest 1/1 通过。Shell 语法、Python 编译及 diff 检查通过。
- 当前模型 q=0 的水平 TCP Z 为左 1.1210001003 m、右 1.1209936842 m。
- 初次 idle 未执行 native tick 时 guard 尚未 paused；测试复现 c 被错误拒绝后，改为先验证
  健康 Home，完成采样时才在同一控制线程的重置 barrier 内 pause/reset/configure。
- 标定完成事件在新 execution epoch 的首个 snapshot 之前入队，包含原生 reset ack。

本轮未运行现场设备；未改 PICO2 标定模块、SPARK 原生核心、会话默认配置或原始 TJVR 数据。
标定结果仅会话内有效，普通 native reset 会保留它；未标定的原版协议和输出字段保持不变。
