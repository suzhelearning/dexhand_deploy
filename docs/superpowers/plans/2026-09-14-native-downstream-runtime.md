# TJVR 下游在线链路 C++ 迁移实施方案

用户已确认本方案的架构方向及实施授权。顺序实施，不使用子智能体，不提交或 push。

## 目标与边界

保留 PICO driver、M0、TJVR 发送及全部设备 SDK；下游发布、显示/操作和录制接线逐项
接入现有 C++ session gateway。保留 Python 默认入口和现有 `scheduler=cpp` 入口作对照。
每个新增入口必须可独立启用、可追踪，并保持命令身份、模型/TCP、状态语义和 HDF5 兼容。

## 实施顺序

- [ ] 原生消息发布：新增 `native/control/session_publication.hpp`，直接将 C++ 周期快照
  编码成现有主题消息；独立有界发布队列连接 Zenoh。CLI 显式选择发布后端，关闭对应
  Python 高频发布，避免重复发布者。逐字段与现有 Python 消息对照。
  - [x] 编码器、字段对照、可选 `--publication-backend cpp`、原生 Zenoh 输出、
    禁止重复 Python 发布、旧二进制能力握手及发布失败退出回归。
  - [x] 独立消费者解耦：`output_fanout.hpp` 为原生发布与 Python 通道各设有界 FIFO；
    慢消费者、溢出、取消及最后一帧输出失败通过离线回归，完成回执等待所有消费者排空。
  - [ ] 真实输入发布验收尚未完成，不能将父项标为完成。
- [ ] 原生录制接线：复用 HDF5 block/stream 组件，原始输入、cycle、命令、反馈及操作
  事件直接进入 C++ 录制消费者。复用既有 schema，确认独占创建、故障与完整关闭语义。
  - [x] 修复接收适配器接受成功包时丢失 `decoded/ingress_sequence` 的缺陷；
    成功包和被门控拒绝的可解析包均可进入 raw 捕获，真实 worker rearm 回归核对字节和序号。
    这是驱动之后的适配修复，不修改任何 SDK/驱动或 TJVR 发送端。
  - [x] `worker_result_json.hpp` 完整 worker 审计编码：SPARK 引导状态、mapped-palm
    可选高度偏移、左右臂全部诊断字段与既有 Python 解码契约逐字段对照。
    64 组固定随机样本覆盖大整数与可选字段，并拒绝错后端、截断、非法布尔及非有限值。
    该模块尚未接入在线录制消费者，不代表录制热路径已完全迁移。
  - [x] `raw_tjvr_hdf5.hpp` 原始包及 ingress audit 数据块、
    `session_cycle_hdf5.hpp` 完整机械臂 cycle 数据块，通过实际 C++ HDF5 写入器与
    Python schema/字段对照；保留重复包、未采纳 IK、原始输入 base64 及完整 worker 诊断。
    支持跨流排空顺序引起的负相对时间，拒绝非法包、越界序号及非法标识；
    写入失败后的文件保持 incomplete。
  - [x] `SessionRecordingSink` 消费原始包、周期及带处理时间的操作回复，写入 opening/
    close 生命周期审计；取消、编码失败和遗弃均不提交完整标记。
    `NativeSessionGateway` 提供独立录制队列 hooks，等待所有队列排空再执行录制关闭。
    正常、发布失败、末帧失败、录制失败和录制关闭失败分别重复验证。
    空输入控制周期仍保留 IK 结果，`native_attempt` 为空，不误报坏包。
  - [x] 可选 `--recording-adapter cpp` 接通 schema 冷启动、统一时钟原点、fd 移交、
    `recording=cpp` 能力握手和 Python 逐帧录制去重；默认仍为 python。
    实际 SPARK/mapped-palm 网关子进程接受合成 TJVR、产生周期、Home 退出及完整落盘
    通过离线测试；无设备/SDK 操作、无 Zenoh router 启动。
  - [ ] 真实输入长时录制、异常断开验收及同数据端到端性能对照；父项暂不标记完成。
- [ ] 原生显示和操作：独立 MuJoCo model/data 与 GLFW，消费最新显示快照；骨架、目标
  TCP 的坐标系与现有 overlay 对照。操作事件通过原有状态机，渲染不持有控制锁。
  - [x] `RenderModel` 独立 model/data 与场景生成：两种机器人模型的 FK、目标 mocap
    位姿、200 ms 过期/未来时间/非激活时 TCP 回退与 Python 对照；拒绝跨线程访问，
    非法双臂/目标更新不改变上一帧。该步骤没有接入窗口或改动控制执行器。
  - [x] 可选 `--viewer-backend cpp` 接通 GLFW 窗口、独立显示缓存和骨架叠加；
    `s/h/r/q/c` 与关闭窗口提交原状态机事件，终端按键保留；旧二进制需能力握手。
    两种 overlay 的新鲜度、执行 epoch、重复周期和非法目标诊断与 Python 对照。
    无显示环境明确失败；不创建静默 Python 回退。
  - [ ] 实际 GLFW 画面、鼠标/窗口关闭和真实输入组合验收，不能用无显示测试替代。
- [ ] 同数据 A/B：录制文件选择和 SHA256 固定，分别验证命令/状态语义与消费者输出；
  性能测试预热、交替重复，输出 p50/p95/p99、吞吐、CPU 与丢弃/故障计数。
  - [x] `benchmark_native_publication.py`：同 HDF5 区间的消息字段与编码/序列化耗时对照。
  - [x] `benchmark_native_consumer.py`：同 HDF5 区间，原生消费者组合跳过 Python
    完整快照构造；状态/计数逐帧相等，预热后交替测量。只测已解码 Python 消费开销。
  - [x] 原生消费者齐备时自动协商 `diagnostics=summary`，Python 不再接收逐包 TJVR、
    完整 IK 周期或逐周期 receipt。状态摘要约 10 Hz，状态/epoch 变化和最终计数不遗漏；
    原生发布与录制仍消费所有快照，旧完整协议保持默认。已验证队列保全与协议隔离，
    尚非端到端性能或完整手部调度完成。
  - [ ] 完整调度、录制/渲染/网络端到端对照和 CPU/队列计数；当前只是局部微基准。
- [ ] 组合入口回归和文档：逐项与组合验证，保持 PICO2、Python VR、原生 VR 原有入口。

## 验证规则

先验证消费边界再接在线入口；离线测试不连接现场设备、不启动 router、不发送机器人命令。
允许在测试内部使用受控的 Unix socket、原生子进程及临时 HDF5。性能结论必须写明测量范围，
消费者编码提速不等于整链路延迟下降。未完成的消费者或现场验收不得标记完成。
