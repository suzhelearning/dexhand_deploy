# Mapped-palm bandwidth baseline 移植与验证

## 本次提交前审查（2026-09-13）

- 审查当前新增后端、共享接线、标定/显示、诊断与录制退出边界；不调整算法参数。
- 修正按q回Home期间再被SIGINT/窗口关闭中断时的退出原因，避免误报操作完成。
- Python全量 `unittest discover -s tests -q`：1040项，978通过、62条件跳过。
- 显式启用 `MAPPED_PALM_NATIVE_TEST=1 SPARK_NATIVE_TEST=1 WUJI_REFERENCE_TEST=1`：
  mapped/SPARK真实原生worker、官方Hand2合成输入、标定、显示及调度专项57/57通过。
- mapped原生模型启动CTest 1/1；Shell语法、Markdown相对链接及diff空白检查通过。
- [转腕独立原版对照](mapped-palm-rotation-comparison.md)两组均17001周期通过。
  转腕位置误差保留为已知算法限制，不声明精度或实时验收完成。
- 本轮没有连接设备或执行真机测试；新后端与真实Manus联合现场验收仍待完成。

以下较早测试计数和临时产物路径保留为历史证据，不代替上述本次检查。

## 2026-09-13 现场日志与调度诊断

审查 `mapped_palm_height_20260913_202718.h5`：123,368 个原生周期双臂均
accepted，11 帧 TJVR 跳变拒收，7 次原生周期输入过期，HDF5 complete=true。
同 execution epoch 内原生间隔中位数 4.93 ms、P99 9.76 ms、最大 24.63 ms。
这些是既有录制的观察结果，不代表下面改动后的设备验收。

- 启动终端改为摘要，完整 resolved/资产哈希继续存于录制 metadata。
- 保留最后 reset 周期计数，另加全程 `native_ticks_total`；增加退出触发原因和
  `home_return_completed`，不将信号直接退出解释成已回 Home。
- 控制线程新增固定空间的分阶段 mean/max/over-period 耗时统计；维持原有绝对
  截止时间追赶、固定 dt 和 QP 计算语义。
- VR 显示刷新上限 60 Hz，控制仍为 200 Hz；mapped 显示复用已更新的 FK。
  本机 1,000 次离线渲染准备对照：重复 FK 路径约 0.382 ms/次，复用后约
  0.254 ms/次；不包含窗口同步/GPU，不据此宣称现场实时达标。

本轮相关 Python 回归 50 项：49 通过、1 项环境跳过，包含显式原生高度测试。
现场设备尚未重跑；下一次退出报告的 timing 可用于进一步定位控制 step 与输入、
快照开销。未改 PICO2 输入或 QP 源码，未提交。

2026-09-13。新后端 `pico_ee_mapped_corrected_palm_velocity_qp` 已接入
`pico_vr_manus_sim`，默认 SPARK 和 PICO2 的后端与配置保持原样。代码尚未提交。

## 来源与边界

来源 commit `2bcfe09e2c78a48c7ba63943deef83d04a139ff8`（用户指定 baseline tag），
正式配置为原版 `qp_ik_pico_ee_bandwidth_velocity_qp.yaml`。
算法源码完整保存在 `src/tianji_teleop/src/ik/mapped_palm/src/` 和 `include/`，
机械变换仅命名空间；模型仅修改相对 meshdir。`source_manifest.json` 保存每个文件
原始与移植后 SHA256。部署不读取参考 checkout，不替换现有 SPARK 或 v131 原生库。

新增 `bilateral_cycle.cpp` 适配原版 Viewer 控制周期，`native_worker.cpp` 提供有界 IPC。
前馈从 mapped 掌心和源时间产生，接收端跳变检测也使用 mapped 掌心；原始 TJVR 字节
仍原样录制。无效 corrected 骨架在进入 gate 前拒绝，H5 校验按记录的 target_source 重建。
Manus 输入、官方 Hand2、会话授权、退出清理和 C++ HDF5 继续复用现有模块。

原版源码闭包包含默认关闭的其他算法实现及实验配置结构；新后端运行时不构造 SPARK
guidance，不启用 C1、Generic Cartesian OTG 或 motion gain。运行中的数学参数来自
`src/tianji_teleop/src/ik/mapped_palm/config/bandwidth.yaml`，不是旧 mapped 配置。

## 已完成的验证

- 独立 Pixi/CMake 构建成功，命名空间 `tianji_mapped_palm`，独立 worker 和动态依赖。
- 原始/移植文件摘要、bandwidth 参数、输入组合与默认 SPARK 不变检查。
- 新 worker 身份、模型参考状态、双臂 reset、有界消息协议复用及新会话仿真命令链。
- mapped 掌心 gate：普通包 pose 跳变不改变新算法 gate；corrected 掌心跳变仍拒绝。
- 合成 rawviz → 官方双手 Hand2 → 同一个 MuJoCo → C++ HDF5；Manus callbacks、
  手关节命令重建与 TJVR 接纳审计通过。这里不是实体手套测试。
- 已执行旧 SPARK 原生会话/IPC、VR CLI 和双输入配置专项回归。

本次修复后验证：Python 全量 **1016 项，956 项通过、60 项按环境条件跳过**；
显式启用新原生后端、旧 SPARK 和官方 Hand2，连同 overlay/校验回归的合并专项 **59/59 通过**。
原生模型启动 CTest **1/1 通过**。
Python 编译检查、Shell/README 命令语法及 `git diff --check` 通过。

原版 Viewer 的短轨迹独立对照使用 `output_continuity_retest.tjvr`，2444 帧、27.145 秒
源时间、5430 个控制周期。原版单独产生 CSV，移植版从同一原始 trace 按原版
`PicoTraceReplay` 的源时间重基准与 latest-only 顺序回放，没有使用移植版作为 oracle。

| 比较项 | 最大绝对差 |
| --- | --- |
| 双臂 q | 5.00e-12 rad |
| 双臂 qdot | 5.00e-12 rad/s |
| 双臂 qddot（仅比较原版标记加速度有效的周期） | 3.01e-10 rad/s² |
| Headroom scale、位置/姿态任务缩放 | 5.01e-13 |
| input_live、applied epoch、applied sequence | 逐周期完全一致 |

原始 CSV 写出精度有限；上述误差量级与其舍入精度一致。对照使用显式离线模式放宽
QP 墙钟预算，不能证明实时线程调度和端到端延迟相同。源时间回放也不替代现场接收时序。
本机此次 CSV 位于 `/tmp/mapped-palm-reference-review/`，是可重建临时结果，不随代码交付。

## Eggbeat 正式基准复核与修复（2026-09-13）

使用用户提供的 `pico_eggbeat_bandwidth_20260904_session01.tjvr`，SHA256：
`85e06eb32a5b0a8783bb74e7d663ad4f1de75a9f4db053bedcccd853ed55b9a3`。
共 10,682 帧、119.793 秒源数据；原版回放到 120 秒，共 24,001 个 200 Hz 控制周期。
原版 Viewer 来自 `d623c1b8df0edfba18e1d8b78f93a9f30ef612eb`，正式配置/算法如上。
接收结果为 10,596 帧接受、86 帧跳变拒绝、2 次 epoch 初始化/切换、36 次重同步；
原版控制失败为 0。它比此前短轨迹覆盖更多输入恢复情况。

初次对比失败：全程最大关节位置差 0.406869 rad。定位与修复如下：

1. **控制循环适配遗漏 FK 刷新。** 原版 `currentTargets()` 先 `robot.forward()`，
   再读取 TCP；移植适配器遗漏了刷新。控制器写入 qpos/qvel 后，MuJoCo 的
   site_xpos/site_xmat 尚未更新，epoch/reset 的目标起点因此来自上一周期缓存。
   已恢复原版每次接纳输入时读取当前 FK 的时序，保持控制器关节历史连续。
   新增测试用独立 MuJoCo FK 核对 reset 起点：修复前约 0.71 mm 偏差，修复后通过。
   隔离实验使用原版静态算法库仍复现旧适配器分歧，排除了命名空间移植/算法库构建主因。
2. **比较器口径错误。** 原版 CSV 加速度来自绘图差分器，在 epoch/reset 或无效输出时
   清零并标记无效；worker 报告控制器内部加速度。现在按原版
   `*_reference_acceleration_valid` 比较，仅忽略无定义的绘图加速度，位置/速度仍逐周期检查。
   不通过放宽阈值或清零实际控制加速度来消除差异。
3. 离线测试模式的 QP、前馈预览、任务预览墙钟预算均对齐原版 1 秒；该差异经实验
   **不是本次关节分歧主因**。正式运行的预算和全部数学参数不变。

修复后同一完整基准对比通过，原有 `1e-7` 对比阈值未放宽：

| 比较项 | 最大绝对差 |
| --- | --- |
| 双臂 q | 5.00e-12 rad |
| 双臂 qdot | 5.00e-12 rad/s |
| 双臂有效 qddot | 5.01e-11 rad/s² |
| Headroom scale、位置/姿态任务缩放 | 5.01e-13 |
| input_live、applied epoch、applied sequence | 24,001 周期完全一致 |

本轮原版 CSV 在 `/tmp/mapped-eggbeat-oracle.RkEudK/{reference,joints}.csv`，
是临时验证产物，不作为部署依赖。基准数据也不复制到工程或写成运行时必需路径。
此结果证明该基准下的确定性数值复现，不代表现场调度、设备延迟或长时间运行已验收。
该 TJVR 不含 Manus 手指数据，不能单独作为手套/Hand2 路线的真实设备验收。

## 重建对照

先在参考工程用正式配置、模型和算法执行 headless
`--deterministic-pico-replay TRACE --telemetry REF.csv --joint-telemetry JOINTS.csv`。
显式指定足够长的 `--duration`（上述 Eggbeat 对照使用 `--duration 120`），确认导出的
控制周期覆盖全部源数据。比较器拒绝空/截短 CSV、缺失周期及 NaN/Inf，结果包含
`input_frames`、`consumed_frames`、`full_trace_consumed`，不把局部回放当作完整验收。
保留启动输出确认 `control_state_source=model_reference`、`hand_tcp_frame_L/R` 和算法名称。
然后在本工程执行：

```bash
pixi run python scripts/check_mapped_palm_oracle.py \
  --trace /实际路径/input.tjvr \
  --reference-joints /实际路径/JOINTS.csv \
  --reference-telemetry /实际路径/REF.csv

MAPPED_PALM_NATIVE_TEST=1 WUJI_REFERENCE_TEST=1 \
PYTHONPATH=src/tianji_teleop:. pixi run python -m unittest \
  tests.test_mapped_palm_native tests.test_mapped_palm_input \
  tests.test_mapped_palm_port tests.test_mapped_palm_session \
  tests.test_mapped_palm_oracle -v
```

## 后续 review 修复

- 共享 overlay 按会话选定的算法校验结果，新后端显示 `Mapped-palm IK target`；
  不会将 SPARK 结果当作 mapped-palm 接受。默认 SPARK 标签和校验行为不变。
- oracle 在启动 worker 前校验参考数据非空、完整覆盖、连续周期及有限数值；
  reset 绘图加速度仍按有效位决定是否比较，不放宽误差阈值。
- HDF5 校验先检查所有 raw 帧的接收器身份、序号和时间，再过滤无效 mapped 骨架。
  `raw_frames` 为全部记录数，`gate_frames` 为进入 gate 的帧数，
  `invalid_selected_targets` 单独记录；无效骨架不能绕过原始数据来源与顺序检查。
- 新增回归覆盖新后端标签/身份、空/截短 CSV、NaN/Inf、无效骨架帧的异来源、
  重复序号、时间回退；HDF5 测试通过真实临时文件读写验证，不改用户录制。

## 尚需现场验证

用 README 新后端命令先验证双臂，再加入 Manus；检查目标 TCP、姿态/位置跟随、
臂角、短暂丢失恢复、手指稳定性、Home/rearm、Ctrl-C 和 HDF5 完整性。
新后端仅开放仿真，未连接或验收机器人真机。
# 控制线程耗时诊断（2026-09-13）

结束事件 `dual_live_complete.timing` 增加以下固定空间累计统计（毫秒）：

- `schedule_lag`：处理操作命令后、读取输入前相对计划周期起点的延迟；
  包含调度延迟、之前周期积压和操作处理，不等同于纯 OS 唤醒时间。
- `input_and_feedback`：输入处理与控制前反馈更新。
- `coordinator_cycle`：协调器周期，包含调用 native worker。
- `command_and_simulation`：命令执行与控制后反馈更新。
- `request_encode`、`ipc_roundtrip_decode`、`result_validate`：成功 native 调用的
  请求编码、IPC 往返及 JSON 解码、结果校验。

后面三项嵌套于 `coordinator_cycle`，不能与父项相加；成功调用样本数不必等于
总控制周期数。失败调用仍沿用原故障处理，不记为成功耗时样本。
IPC 包含 C++ 计算、序列化、进程调度、管道等待与 Python 解码，不能直接解释成
纯 QP 求解耗时。统计不会修改控制时钟、算法 dt、调度规则或 native 协议。
报告同时保存在录制的 lifecycle 审计中；Ctrl-C 正常清理才会生成最终汇总，
强制终止不能保证有汇总。离线合成输入只能验证计时链路，不能替代现场实时验收。

### 长间隔调用栈采样

VR live runner 的后台探针每 20 ms 检查一次控制阶段，超过 50 ms 时记录
Python 调用栈（不读取局部变量、不打印、不从磁盘加载源码）。每个阶段实例只记一次，
最多保存 32 条；`control_stalls_dropped` 记录容量耗尽后未保存的阶段数。
最终事件/HDF5 lifecycle 中的 `control_stalls` 包含阶段、进入阶段时的会话状态、
已完成周期计数、采样时间及栈位置；`control_stall_probe_error` 表示探针本身的异常。
`timing.actions` 单独统计操作处理，包括标定完成时的同步 Home 重建。

探针没有运动、故障或恢复权限，不修改绝对周期计划。它是尽力而为的 Python 采样：
全进程挂起或长时间占用 GIL 也可能令探针无法及时运行；阶段标签与调用栈之间存在
采样竞争，需结合时间线解读。它不提供 C++ 内部调用栈，也不覆盖 5～20 ms 短抖动。
`waiting` 的耗时从进入等待开始，并非精确的 OS 唤醒延迟。

现场 `mapped_palm_timing_20260913_223515.h5` 的标定间隔约 288 ms；
离线三次 Home 重建为 207/210/210 ms，主要等待 native reset 返回。
启动前约 847 ms 的间隔未复现，不能据此认定为同一个原因或声称已修复。

### 标定重建后的空闲调度修复

启用 `--mapped-palm-height-calibration` 时，成功标定或显式 Home rearm 后，
若仍为 idle，下一周期按“屏障完成时刻 + 控制周期”安排，不补跑重建期间的空闲周期。
若同批操作已进入 teleop，则保留原计划；普通遥操超时仍保持原有绝对周期追赶。
原 SPARK、未启用高度标定的 mapped-palm 及 PICO2 路线不改变调度行为。

此项不会缩短 native reset 的重建时间，`timing.actions` 与长停顿调用栈仍如实记录。
也不代表启动前 847 ms 的偶发间隔或普通控制短抖动已解决。

### 高度标定显示分层

移除旧版对整侧肩/肘/腕/掌显示点及连线的 Z 平移。与原版绘制已应用骨架的方式一致，
青色 `Applied corrected` 始终保留收到并被 native 应用的原始点位。
启用标定后额外以紫色 `Z calibrated palm position` 显示掌心位置加 Z 偏移，
不画原始掌心姿态轴；黄色 native IK target 仍表示目标处理后的实际目标。
关闭标定时不增加紫色标记，原显示路径保持不变；显示不写入控制或采集数据。
肩点属于共同 world 坐标系，不保证与机器人第一关节逐点重合（人体肩宽可能不同）。
此修复只纠正显示语义，不改变目标偏移、求解器或现场跟踪性能；完整原版数值等价
仍需要同一输入、配置和时序的独立对照，不能仅凭可视化一致宣称通过。
