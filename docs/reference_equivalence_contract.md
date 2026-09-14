# 五工程联合路线参考契约（源码核对记录）

版本见 `dual_input_migration_manifest.md`。本记录保存阶段性的源码核对和验证证据，不能作为完整等价性通过报告。
下文“待完成”“接线未完成”指记录当时的状态；当前操作与交付范围见
[README](../README.md) 和 [真实输入仿真验收单](real-input-simulation-acceptance.md)。
后续进展不改变本记录中已限定的对照范围，也不自动补足未测试分支。

## 已核实行为

- SPARK基准：`spark_upper_qpoases_headroom_feedforward_velocity_qp`，`config/qp_ik_pico_teleop.yaml`、`models/marvin_m6_wuji2.xml`、velocity 200Hz、model_reference。不得把README另一段fast模型回放示例混为同一模型基准。
- `apps/run_qp_ik_viewer.cpp` 用 `pico_frames->tryReadLatest` 获取控制周期内最新帧。接收线程、原始录制与控制消费分别记录；不能强迫逐帧FIFO消费改变控制时序。
- 同文件先计算控制器step，随后将左右qdot、上一加速度、任务缩放和accepted构造为Headroom反馈，执行一次 `updateHeadroomFeedback`。
- epoch变化更新输入流、SPARK guidance及接管逻辑；model_state_only时不调用同步实测参考。参考注释明确保持OTG连续，防止模型与反馈不一致时参考跳变。
- 参考输入新鲜度来自接收单调时间。`pico_teleop_session.cpp` 定义输入age及live，不用跨设备未校准时钟直接相减。
- 手部基准桥代码默认使用wuji_glove命名YAML，但用户提供的远端实际联调基准为 `adaptive_analytical_manus_wuji_hand_2_{left,right}.yaml`。已取回实文件及摘要；实际基准必须显式覆盖桥的左右配置，不能用代码默认值替代用户实际部署配置。
- 手部输入63项为指定单侧、126项为right+left；每侧执行序号间断reset → 官方retarget → URDF限位裁剪 → 关节名称重排 → TJH2输出。
- ROS callback递增的桥侧序号与Manus原始序号不是同一个字段。迁移不得用原始设备丢帧直接替换桥侧序号reset条件。

## 待完成的等价性证据

两份真实TJVR已取回并通过容器及CRC检查（详见来源清单），但只包含epoch137和flags0xff，不覆盖epoch切换或bit8按键触发。针对性合成状态分支及两份实际录制的SPARK逐周期对比已通过，范围和限制见 [SPARK移植报告](spark-porting.md)。独立Manus `.pkl` 不作为与其同步的数据；手部迁移版已与远端原版1339帧40关节基准比较，误差0，见 [官方手部移植报告](wuji-hand-porting.md)。这仍不替代实时联合采集和执行验收。

接收门控、按键变化、guidance接管、epoch与丢失恢复以及QP依赖已经建立源码和确定性对照。仍需覆盖未在现有测试中触发的失败/reset分支、实际标定接入及实时调度；后续对应模块实施前逐项补齐，不用推测填入。

比较点：raw接受/消费帧 → 21点及上肢骨架 → 坐标转换 → SPARK参考 → 前馈/Headroom → QP及q/qdot/qddot → 手retarget/限位/重排 → 状态及输出。确定性输入和实时传输分别验收，新增硬保护介入单独标注。

## 已实现的隔离协议/流接纳对照（2026-09-08）

- `hand_tracking/reference_tjvr.py`：新增显式参考解码入口，保留完整raw/旧frame、双侧归一化方向及独立有效位、按键状态。四元数原始模长误差大于1e-3时拒绝；未声明方向或非有限/过短方向只使该方向无效，不拒绝整帧。旧 `legacy_pico.py` 的行为与消息schema不变。
- `hand_tracking/reference_tjvr_stream.py`：独立的接纳状态机，保留原版epoch单调性、乱序拒绝、双侧packet target跳变检测和三帧稳定重接纳。被跳变拒绝的帧仍推进观测序号；回到已接纳目标清除候选。此处不做IK reset、操作授权或最新帧合并。
- 原版 `pico_udp_receiver.cpp` 在解码成功后、stream gate之前录制；所以TJVR文件不等于最终被控制器消费的帧序列。参考benchmark经UDP实时发送，受线程调度影响，不能仅凭其名称将运行视为逐周期确定性基准。

验证直接链接参考目录的原始 `pico_teleop_protocol.cpp` 和 `so3.cpp`，不复制重写oracle、不启动网络/设备。测试适配器为 `tests/cpp/reference_tjvr_probe.cpp`。两份录制共29498帧，以及协议异常/流状态合成序列，对照通过：接受/拒绝、metadata、有效位、按键状态、gate决策严格一致；双侧位姿矩阵、方向、骨架位置/旋转按绝对误差1e-12比较通过。浮点门槛仅用于这一协议层，不替代IK验收门槛。

可复现命令（原始仓库及参考录制均为可选本地资产）：

```bash
TJVR_REFERENCE_ROOT="${PICO_MANUS_TELEOP_ROOT:?set PICO_MANUS_TELEOP_ROOT}/TJ_arm_control" \
TJVR_EIGEN_INCLUDE="$PWD/.pixi/envs/ik-build/include/eigen3" \
TJVR_TRACE_DIRECTORY="$PWD/vendor/reference_inputs/remote-192.168.110.210-20260908" \
PYTHONPATH=src/tianji_teleop:vendor/python \
  pixi run python -m unittest tests.test_reference_tjvr_cpp -v
```

该可选对照3项通过；普通发现集在没有环境变量时跳过，不把缺失资产当作对照通过。后续已实现receiver持久化resynchronization_generation、每控制周期最新帧消费及独立SPARK guidance/Headroom/QP状态推进，两份录制共65726个控制周期与原版输出一致，详见移植报告。运行会话接线和端到端验收仍未完成；不能把本节协议结果称为完整机械臂算法等价性验收。
