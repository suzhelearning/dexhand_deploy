# mapped-palm 前伸 X/Z 标定

## 已确认范围

新增 `--mapped-palm-xz-calibration`，仅用于全 C++ mapped-palm 仿真路线。
左右臂参考关节均为 `[0,-90,0,0,0,0,0]°`。按 c 时人在 Home 等待阶段双手水平向前伸直，
稳定采样两秒；分别将掌心 X/Z 对齐模型参考 TCP 的 X/Z。Y 和姿态不变。
参考只用于 FK，不发送参考关节命令；Home 仍读取 arm.yaml。
不改 PICO、Manus、机械臂及灵巧手驱动，不改 SPARK、PICO2 裸手和 Python 入口默认行为。

## 实现边界

- MuJoCo 使用独立 mjData 计算 hand_tcp_frame_L/R 的参考 X/Z，不污染运行状态。
- 原 HeightCalibration 保留默认 Z-only，显式提供 X 参考才计算 X 偏移。
- 保留采样数量、时长、稳定性、新鲜度、epoch 和最大偏移保护。
- 候选偏移只有在下一 execution epoch 的 worker reset 和 TJMX1 回执全部成功后提交。
- C++ worker 只在 mapped target 位置叠加常量，不改源骨架、姿态或算法内部约束。
- 原 TJBR kind=2 保持 595 字节；新 X/Z 模式 kind=3 为 611 字节，追加两个 X 偏移。
  C++ decoder、Viewer 和 HDF5 JSON adapter 同步支持；Python 原协议不变。
- 新入口要求 scheduler/publication/viewer/recording 均 cpp，且显式 viewer 和新录制文件。
- 原 `--mapped-palm-height-calibration` 仍为 Z-only，两选项互斥。

## 验证项

`tests/test_native_xz_calibration.py`：稳定采样、失败重试保留结果、CLI 隔离。
`tests/test_native_height_calibration.py`：原 Z-only 对照、真实 native worker X/Z 回执、
FK 只读性、只改变 X/Z、不改变姿态、reset 保留偏移。
`tests/test_worker_result_json.py`：旧协议与 Python 结果一致，新 kind=3 的审计编码。
`tests/test_native_joint_gateway.py`（NATIVE_JOINT_GATEWAY_TEST=1）：合成 TJVR/Manus 输入，
真实 C++ worker、发布、Viewer、HDF5；覆盖 SPARK、原 mapped 和新 X/Z，
包括 c、s、h、自动重置、手动 r 和 q，检查正常退出及录制完整。

这些为软件/离线验证；真实设备下的前伸动作、姿态约束与奇异点附近体验仍需用户验收。
本轮不提交或 push。

## 标定反馈修复

标定初始回执仅表示采样开始；终端动作关联保留到最终提交或失败回执。
采样输入中断、稳定性检查失败、操作者取消和会话结束均应发出一次失败回执，
不改变已成功标定的偏移保留规则。回执继续进入原生 operator_result 录制路径。
相关回归见 `tests/test_native_live_shutdown.py` 与 `tests/test_native_scheduled_ik.py`。

终端发送命令统一先分配请求 ID、登记动作，再发送，避免立即返回的 C++ 回执早于登记。
发送失败或中断时清理动作及标定等待关联；不持有动作锁进行 socket I/O。
覆盖普通按键、窗口关闭、时长到期及退出清理。C++ Viewer 自有按键协议不变。

## MuJoCo 窗口提示

左上角 HUD 从 SessionSnapshot 读取标定状态，从既有输出消费线程读取最近按键回执。
显示采样数量、等待 ACK、成功及偏移、失败原因、旧标定保留情况和按键提示。
使用 MuJoCo 原生 mjr_overlay 英文字体，不增加 Python 渲染或修改运动授权。
`tests/test_native_viewer_hud.py` 覆盖各标定状态；原 Viewer 按键和显示状态回归保留。
此改动不修正追踪源 epoch 变化，也不放宽相应保护；现场验证需要重启会话。

## 现场启动记录（2026-09-15）

- run_id：`0731de0d-34d4-4cee-b482-f3b803821721`。
- 真实 PICO raw/M0/bridge 就绪，左右 Manus 已加载 gjy 标定；
  启动摘要确认 `hands=True; calibration=XZ; scheduler=cpp; publication=cpp;
  recording_adapter=cpp; viewer=cpp`。
- 本次上游世界 X 总偏移为 0.20 m。
- 录制路径：`recordings/device_acceptance/native_joint_mapped_xz_20260915_041623_720478106.h5`。
- 这里只确认启动，尚未将该录制标记为正常完成或把用户实际跟踪效果认定为验收通过。
- 完整命令及操作见 [README](../../../README.md#mapped-palm双臂前伸-xz-标定c)。
