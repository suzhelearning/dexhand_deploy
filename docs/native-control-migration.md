# C++ 高频控制迁移（2026-09-14）

目标是让 C++ 拥有高频数据与控制执行，Python 负责启动、配置、交互和离线分析。
迁移可以包括协调器的逐周期检查，不能因此取消命令所有权、身份、输入新鲜度、
限位、Home/rearm、epoch 和故障语义。PICO2 裸手与 PICO＋VR＋Manus 的入口和默认行为保留。

进度说明：以下按阶段保留当时范围。最新离线 C++ 组合已接入 IK、MuJoCo、Home rearm、
mapped-palm 高度标定及完整双臂回执编码，并减少仿真适配层临时分配（阶段 14–18）。
阶段 19 补充独立数据报接收及有界输出线程，已组成离线接入夹具；现场默认控制编排仍未切换。
阶段 20 将输出消费及 I/O 失败通道接入原生运行时，具体 HDF5/Zenoh sink 仍待接入。
尚未完成完整网络/手部输入、输出发布者接线、HDF5/viewer 消费与现场验收。

## 当前已实施：可选 C++ 二进制周期结果

两个 VR/TJVR native worker 增加 `--binary-results`，会话入口为
`--native-result-format binary`，默认 `json`。只支持 `vr_manus_sim` / `pico_vr_manus_sim`。
结果由 C++ 直接写入固定大小缓冲区，避免关节/位姿/诊断数值转换成十进制 JSON；
Python 用 `struct` 解包后交付原有协调器与录制接口。C++ 在该选项下关闭 stdio 同步与
cin/cout 自动绑定，响应仍显式 flush。原始移植算法、模型、参数均未修改。

这一步只迁移**结果序列化路径**。输入请求仍为已有有界 ASCII/hex 协议；启动握手、
reset、height ack 仍为 JSON。200 Hz 调度、协调器和 MuJoCo 执行编排目前仍在 Python。
不能把这一步描述为整个控制层已经 C++ 化。

二进制 ABI v1：8 字节 `<4sBBH` 头，magic=`TJBR`、version=1、backend=1(Spark)/2(mapped)、
固定 payload 大小；其余字段 little-endian，IEEE754 float64，显式整数宽度，无 C++ struct padding。
完整字段顺序由 `src/tianji_teleop/src/ik/native_binary_results.hpp` 写出，
`hand_tracking/native_binary_results.py` 定义解码布局。所有原有周期诊断字段保留。
末尾附带 C++ 求解与结果编码耗时；这两项仅进入 owner timing，不改变 native_result/HDF5 的
原有数值语义。编码耗时不包含最终写入/flush。

每周期依然只有一个请求在途；无重试、自动回退、重新启动或额外命令发布者。
错误版本/后端/长度、截断、超时、非有限结果或不匹配的 tick/time 均使客户端失效。
JSON/binary 格式写入 resolved configuration，随录制保存。

构建与选择（在工程根目录）：

```bash
pixi run --manifest-path tools/mapped_palm_native/pixi.toml build
pixi run --manifest-path tools/spark_native/pixi.toml build
```

在已验证的 VR/TJVR 启动命令末尾增加 `--native-result-format binary` 即可。
删除该参数恢复默认 JSON。更新过的 native worker 才支持此选项，旧二进制会明确启动失败。
PICO2 裸手入口不接收此选项。

## 测量方法与首轮结果

上一轮 `ipc_roundtrip_decode` 包含 C++ 求解、进程调度、协议和 Python 解码，
不能把该值整体当作 Python/JSON 开销。本次增加 `native_step` 与 `native_result_encode`。

离线基准工具 `scripts/benchmark_native_results.py` 不连接 router/设备、不发布机器人命令。
同批输入轮换调用顺序，使用两个独立 worker；每步比较完整结果。
`--deterministic-test` 放宽原有 wall-clock 求解预算，只用于一致性，不能证明实时性能。

```bash
pixi run python scripts/benchmark_native_results.py \
  --backend pico_ee_mapped_corrected_palm_velocity_qp \
  --trace /path/to/reference.tjvr --cycles 2000 --deterministic-test
# 正常预算计时：移除 --deterministic-test；壁钟预算导致的结果差异须另行分析。
```

本机 `pico_eggbeat_bandwidth_20260904_session01.tjvr` 前 2000 个调度周期，
mapped-palm deterministic 对比：完整结果差异 **0**。单次观测：

| 指标 | JSON | 二进制 |
|---|---:|---:|
| 完整 client.step 平均 | 1.164 ms | 1.149 ms |
| P99 | 2.145 ms | 2.190 ms |
| C++ 求解平均 | 未单独计量 | 0.957 ms |
| C++ 结果编码平均 | 未单独计量 | 0.000754 ms |

平均改善很小，P99 未改善；不能据此宣称现场跟踪更快或 200 Hz 已通过实时验收。
该实现提供可对比的 native 结果接口与归因数据，主要收益仍需后续完整高频循环迁移。

Spark 对同一录制前 2000 周期的 deterministic 对照同样差异为 0；
JSON/binary 平均 1.254/1.206 ms，P99 2.530/2.675 ms。也是平均小幅下降而
P99 没有改善。两个后端的数字均为该次离线单机观测，未连接真实输入、viewer 或 recorder。

第一阶段验证：`PYTHONPATH=src/tianji_teleop:. pixi run python -m unittest discover -s tests -p 'test_*.py' -q`
运行 1047 项，跳过 62 项，其余通过；修复了旧计时测试绕过构造函数而缺少新字段的夹具。
两个 native Release build 成功，mapped CTest 1/1、Spark CTest 9/9 通过。
显式启用 native 的定向测试运行 45 项、跳过 3 项，其余通过。
新测试覆盖两个实际 worker 的完整字段对照、断流/epoch/高度/reset、协调器与 MuJoCo
执行一致性、短读/截断/非法输出闭锁，以及入口参数传递和 PICO2 隔离。

## 第二阶段：C++ 执行回执检查

新增 `--execution-guard cpp`（默认 `python`），仅接入两个 VR/TJVR 仿真入口，
可与 JSON 或 binary 结果格式独立组合。它替换 producer 的 ExecutionGuard，
不替换 coordinator 的最终命令发布，也不将回执冒充机器人测量反馈。

`native/control/execution_guard.hpp` 是不依赖 Python 的 C++17 状态核心：
连续 tick、单调时钟检查、回执超时、在途上限、双臂命令精确比较、首次故障锁存。
`python_execution_guard.cpp` 在 CPython 边界用 C++ 检查身份、schema、类型与有限数值，
使用当前 Python ABI 构建为原生扩展。高频调用不启动进程、不做 JSON/pipe 往返、不阻塞，
保持 GIL；Python 适配器只保留启动参数验证和方法转发。配置在构造后只读，
显式 Home/rearm 创建下一 epoch 的同类 C++ guard，不自动退回 Python。

```bash
pixi run build-native-control
# 在已有 VR/TJVR 仿真启动命令中追加：
# --execution-guard cpp
# 如需同时使用上一阶段的二进制结果：再追加 --native-result-format binary
```

扩展路径随当前 CPython 的 EXT_SUFFIX 解析，构建产物在 `build/control-native`；
缺失或 ABI 不兼容明确启动失败，不运行时自动编译。选项与扩展 SHA256 写入 resolved
configuration 和录制元数据。PICO2 裸手入口拒绝该选项，原有默认路线不新增依赖。

微基准：`pixi run python scripts/benchmark_execution_guard.py`，交替顺序、20 批 × 2000 次
`check/register/observe`，Python/C++ 平均约 5.99/1.45 μs（该局部约 4.1 倍）。

增加整段核心离线工具：

```bash
pixi run python scripts/benchmark_live_control.py \
  --trace /path/to/reference.tjvr --cycles 600
```

同一 eggbeat 录制前 600 周期，mapped-palm 两侧各执行 599 次原生调用，结果差异 0。
Python/C++ guard 的整段周期平均 2.535/2.528 ms，P99 3.798/3.876 ms。
此处使用正常求解壁钟预算，但无实时调度、viewer、网络输出、手套或录制负载。
微基准倍数不能解释为整条遥操提速倍数，P99 尚无改善，仍不具备实时验收结论。

定向测试复用原有回执测试，并增加 3000 步差分事件序列、类型/边界/Unicode reason、
短生命周期实例、显式缺库失败、PICO2 隔离及两个真实 native worker＋MuJoCo 的 Home/rearm。
C++ 状态核心也单独通过 AddressSanitizer/UndefinedBehaviorSanitizer 测试。

第二阶段全量回归运行 1062 项，跳过 62 项，其余通过；显式 native 定向回归运行
64 项、跳过 3 项，其余通过。新扩展使用 `-O3 -DNDEBUG -Wall -Wextra -Werror`
构建成功。未连接真实设备，未提交代码；现场带 viewer/Manus/HDF5 的负载验收仍待进行。

当前 200 Hz 调度、完整 coordinator、输入协议对象与 MuJoCo 执行编排仍在 Python；
这是逐段迁移的第二步，不是“完整 C++ 控制器已完成”。

## 反馈路径减负（2026-09-14）

VR 内存控制核心没有为 MuJoCo 执行器绑定 transport publisher，原实现却在每次
`tick` 构造 arm/hand state 和 status 协议字典，随后 `_put(None, ...)` 丢弃。
现改为仅为实际绑定的 publisher 构造消息；内部 status、反馈序号、命令过期检查、
MuJoCo 更新、手部授权与返回 Home 全部保留。有发布器的 PICO2/Zenoh 路径继续发送
原协议，不改变入口、默认后端或录制格式。该改动是减少 Python 无效工作，**不是新的
C++ 控制器迁移**；MuJoCo 计算本身仍使用原生库。

隔离测量：与 HEAD 原 `_publish_states` / `_publish_status` 方法对照，同一执行器、
固定时钟、无 publisher，20 批 × 1000 次交替顺序调用；仅测反馈构造，不测 IK。
仅机械臂平均 11.73 → 4.71 μs，启用双手平均 41.24 → 4.94 μs。
每控制周期有两次反馈更新，节省量仍是微秒级，不能代表整链路倍数提升。

同一 eggbeat 录制 1000 周期、正常求解预算、binary 结果：修改前/后 C++ guard
整周期平均 2.656/2.531 ms，P99 3.977/3.525 ms。这是分次观测，原生求解平均也从
1.015 变为 0.981 ms，因此不能将整段下降全部归因于本次代码。
修改后的 Python/C++ guard 对照各执行 999 次 native 调用、完整结果差异为 0；
这验证的是两个 guard 的对照，不是修改前后结果的逐帧比较。

新增测试覆盖无发布端口不序列化、完整端口反馈/心跳保持，以及仅手部状态端口的
授权与过期状态。没有省掉任一次 `mj_forward`：手部回零、公开模型状态和派生量
还需要单独定义所有权/失效规则，不能只检查 arm qpos 就盲目缓存。

本轮全量回归运行 1065 项、跳过 62 项，其余通过；显式启用原生后端的
MuJoCo/两条输入路线/二进制结果/C++ guard 定向回归运行 42 项、跳过 2 项，其余通过。
`git diff --check` 通过。没有启动现场设备或会话，没有提交或 push。

## 第三阶段：C++ MuJoCo 批量执行与反馈

入口 `--simulation-backend cpp`，默认 `python`，只在 VR/TJVR 会话接入。
与 guard、native result format 独立选择，选项和扩展文件 SHA256 随 resolved/HDF5 保存。

```bash
pixi run build-native-mujoco
# 已有 VR/TJVR 命令追加 --simulation-backend cpp
# 同时选择之前两个可选实现时：
# --simulation-backend cpp --execution-guard cpp --native-result-format binary
```

`native/control/python_mujoco.cpp` 直接链接当前 pixi 环境的 MuJoCo 原生库，
不是通过 Python 回调间接执行 `mj_forward`。启动时绑定真实的 `MjModel/MjData`，
检查对象类型、model/data 归属及头文件/运行库版本；持有强引用防止指针悬空。
使用官方对象的 `_address`，故这是绑定当前 MuJoCo ABI 的私有适配，升级后必须重建、回归。
本机版本为 3.10.0；构建路径来自当前 `sys.prefix`，无其他工程的固定路径依赖。

C++ 缓存最多四组关节地址，先验证整个输入批次的长度/地址/有限数值，再批量写 qpos
并调用 `mj_forward`。反馈读取由 C++ 复制为独立数值列表，不暴露可写视图。
未提供的组保持当前 qpos；不会自行生成命令、映射或控制意图。
现有执行器仍负责身份、时序、限位、手部授权和 Home；native Python 异常锁住执行器，
不静默回退。默认实现及 PICO2 入口不加载新扩展。

当前保留 GIL，要求同一控制线程独占模型/执行数据；Viewer 继续使用独立 render data。
这一步**不是**新建完整 C++ 控制线程、C++ Viewer 或动力学伺服器：仍是原先 qpos 直写
加 forward 的运动学仿真，没有改为 `mj_step`，也没有省略原来的两次更新。
协议对象、手部返回零的编排、启动 Home 初始化和完整调度器仍在 Python。

对照命令：

```bash
pixi run python scripts/benchmark_live_control.py \
  --trace /path/to/reference.tjvr --cycles 1000 --comparison simulation
```

使用 eggbeat 参考录制、正常求解预算、两边 binary 结果和 C++ guard，轮换调用顺序：

| 后端 | 周期/native 调用 | IK 与仿真反馈差异 | Python/C++ 总周期均值 | Python/C++ P99 |
|---|---:|---:|---:|---:|
| mapped-palm | 1000/999 | 0/0 | 4.081/4.175 ms | 9.027/8.464 ms |
| Spark | 600/599 | 0/0 | 3.212/3.225 ms | 5.792/6.175 ms |

本轮没有证实整链路性能改善；收益不能仅按语言推断，因此保持可选。测试不包含真实
输入、viewer、Manus 和录制负载，不构成 200 Hz 实时验收。

基础与执行器测试覆盖逐帧 qpos/xpos/xmat/qM 对照、坏批次不部分写入、重复/越界地址、
对象归属/生命周期、反馈副本、超时/手部授权/Home/故障及缺库显式失败。
全量回归运行 1079 项、跳过 62 项，其余通过；随后补充生命周期/缺库/故障测试，
定向回归运行 45 项、跳过 2 项，其余通过。构建、shell 语法与 `git diff --check` 通过。
未提交或 push，未操作现场设备。

## 第四阶段：协调器命令数值模块

新增 `--coordinator-math cpp`，默认 `python`，仅接入 VR/TJVR 双臂仿真。
`pixi run build-native-control` 同时生成回执 guard 和 `_tianji_command_math` 扩展；
选项与扩展 SHA256 进入 resolved/HDF5。缺库明确报错，没有自动回退。

`native/control/command_math.hpp` 不依赖 Python：包含七关节硬限位检查、源时间窗/
步长上限、tracking hold 不绕过硬限位、可选裁剪及精确 Home 插值。
CPython 适配仅转换经过协调器身份/接收新鲜度检查的数值和配置；读取当前配置，
不缓存可能过期的数值副本。原 coordinator 仍执行 start/return/rearm/fault、
双臂配对/收据、命令发布和输入授权。双臂故障保持仍由原有分支处理，未迁移成自动回零。

编译使用 `-ffp-contract=off`，避免 Home 插值被 FMA 合并而改变舍入。
还补齐了 Python 整数与浮点超时阈值比较的语义：超过 2^53 的纳秒计数不能先转 double，
否则某些阈值后 1 ns 会被舍入为未超时。包含该边界及 int64 两端的回归。

```bash
pixi run build-native-control
# 已有 VR/TJVR 命令追加 --coordinator-math cpp
# 离线数值对照；只在该离线脚本支持放宽原生求解预算：
pixi run python scripts/benchmark_live_control.py \
  --trace /path/to/reference.tjvr --cycles 1000 \
  --comparison coordinator --deterministic-test
# 性能观测移除 --deterministic-test；不能把放宽预算后的耗时作为实时验收。
```

eggbeat 录制对照：mapped-palm 1000 周期/999 次 native 调用，Spark 600 周期/599 次
native 调用；确定性预算下两边 IK 输出和仿真反馈差异均为 0。
这里两侧都使用 binary 结果和 C++ guard，MuJoCo 编排选项保持 Python，只改变命令数学实现。

正常壁钟预算的 mapped-palm **首轮有 80 个差异周期**；当时 Python 侧原生求解最大
耗时达 7.013 ms，没有记录首次分歧的具体字段，不能仅凭这一点断言全部由求解预算造成。
随后工具增加首个分歧记录与确定性对照开关。确定性对照通过，再次正常预算运行
1000 周期差异为 0，尚未复现首轮差异。保留这项限制，不以重跑通过覆盖首轮结果。
第二轮正常预算 Python/C++ 总周期平均 2.613/2.582 ms，P99 4.953/5.224 ms；
平均改善小、P99 未改善，不能宣称整链路性能问题解决。

测试包含 500 组裁剪/Home 随机数值对照、2000 个时间窗/步长随机事件、错误输入、
双臂故障原因/命令/回执对照，以及两个实际 IK worker 同时启用所有 C++ 选项时的
Home/rearm、新输入和显式 start 门控。纯 C++ 模块通过 ASan/UBSan 检查。
真实输入、Viewer、Manus 与 HDF5 同时负载的验收仍未执行。

第四阶段全量回归运行 1098 项、跳过 62 项，其余通过；显式启用原生后端的组合
定向回归运行 60 项、跳过 2 项，其余通过。构建、shell 语法、`git diff --check`
及纯 C++ ASan/UBSan 测试通过。未提交或 push。

## 第五阶段：C++ 会话状态机与调度器内核（离线，尚未接入现场）

实现见 `native/control/session_machine.hpp` 和 `session_runtime.hpp`，详细待办见
[实施计划](superpowers/plans/2026-09-14-native-session-runtime.md)。

- C++ 持有 idle/teleop/returning/fault、Home/退出完成锁存、epoch 与 rearm 后新输入屏障。
  保留显式 start、故障不可 rearm 清除、精确 Home 和 reset ack 门控。
- `std::thread` 持有绝对周期调度；事件与回复分别有界，溢出锁存故障；最新健康快照额外
  有 TTL。停止可唤醒定时等待，不把中断停止标为已返回 Home。没有逐周期 Python callback。
- 对照测试将当前 Python coordinator 的已校验健康事实喂给 C++，比较 208 个事件的
  接受结果、状态和原因；另测 Home、手部宽限、时钟回退、ack/epoch、新输入、队列和线程。
  这不是原始输入协议、命令值、回执或 HDF5 的端到端等价测试。

离线测试（不连接设备，不发布运动命令）：

```bash
PYTHONPATH=src/tianji_teleop:. pixi run python -m unittest tests.test_native_session_runtime -q
```

当前该模块仅模拟双臂仿真状态语义，健康事实必须来自可信入口校验。
它尚未持有真实身份/sequence 校验、IK worker 的 reset 事务、标定、关节命令与 MuJoCo，
也没有接管手部授权、录制或 PICO2 的完整状态。调度测试不证明硬实时性能或性能提升。
**没有新增现场入口开关，原有两条遥操路线继续使用原状态机和调度器。**

本轮验证：专用测试 4 项通过；全量回归 1102 项、跳过 62 项，其余通过；
显式启用原生后端的组合回归 64 项、跳过 2 项，其余通过。
独立线程测试通过 ASan/UBSan，`git diff --check` 通过。
未进行现场设备测试，未提交或 push。

## 第六阶段：C++ 双臂命令周期接入调度器（仍为离线）

`native/control/session_commands.hpp` 将已有数值内核与会话状态机连接起来。
`SessionRuntime` 可在构造时显式传入命令配置，由同一 C++ 线程处理操作事件、双臂
proposal、命令生成及回执快照；未传配置时保留第五阶段的纯状态机模式。

- 命令 owner 持有两臂上一帧最终命令、源时间锚点、Home 插值起点和待处理 tick。
  启动/Home/rearm 使用 owner 的真实命令值判断 `command_home`，不相信调用方提供的布尔值。
- 配对 proposal 检查固定生产者/实例/router、run、epoch、tick 和源时间。
  未处理配对不能被静默覆盖；一侧硬限位或步长失败时两侧都保持之前的最终命令。
- Home、直出/裁剪、tracking hold 和故障保持使用现有 C++ 数值内核。
  时钟检查贯穿操作、输入和控制周期，避免 start 与 proposal 间的回退被漏检。
- 每周期最新命令快照与命令处理回执分离。回执有独立有界队列，慢消费者不会因
  快照更新而丢失回执；回执队列满时锁存故障。它表示 coordinator 处理结果，**不是执行器反馈**。

离线对照：

```bash
PYTHONPATH=src/tianji_teleop:. pixi run python -m unittest \
  tests.test_native_session_runtime tests.test_native_session_commands -q
```

两种配置各 493 个事件，比较与 Python coordinator 的接受结果、状态/原因、回执接受值及
14 个关节值（精确相等），覆盖连续目标、Home、再次启动及非法目标后的双臂保持。
额外 C++ 测试覆盖旧 tick、待处理覆盖、时钟回退，以及线程生成命令和延迟读取回执。
健康状态在对照中由测试提供，不能替代真实身份/新鲜度输入验收。

尚缺完整 wire schema 与 domain health/反馈校验、完整协议回执、真实 IK/MuJoCo 执行、
reset/标定事务和手部/录制接线。没有现场启动开关，也没有改变已验证路线的默认实现；
不能以本阶段通过宣称已完成全链路 C++ 化或证明性能提升。

本轮验证：专用测试 6 项通过；全量 1104 项、跳过 62 项，其余通过；原生后端组合
66 项、跳过 2 项，其余通过。命令周期与调度线程的独立 ASan/UBSan 检查通过，
`git diff --check` 通过。未操作现场设备，未提交或 push。

## 第七阶段：固定授权机械臂健康/反馈校验（离线）

新增 `native/control/session_health.hpp`，并通过 `SessionRuntime` 的可选健康配置接入
同一个 C++ 调度线程。此模式要求机械臂命令配置与 Home/生产者身份一致、手部关闭。

- 固定 source、producer_arm、executor_arm 的 logical ID、实例和 router；校验各自
  单调序号、ready/healthy/simulation 能力，观察用 source 不取得控制权。
- 校验反馈的 executor 身份、14 个关节名称顺序与有限数值；分别计算反馈新鲜度、
  Home 容差及精确 Home。状态与反馈分别计时，状态心跳不会刷新旧反馈。
- 校验在 C++ 线程消费有界事件时执行，新鲜度从本机入队时刻计量，不从处理时刻计量。
  调用方提供的 `received_ns` 被覆盖。该模式拒绝 `update(SessionFacts)` 布尔注入。
- 错误锁存并进入会话 fault；默认旧离线模式仍保留，现有现场入口不切换。

测试比较了 120 个快照的三个域 readiness、反馈 freshness、容差 Home 和精确 Home；
C++ 单独覆盖序号重放、身份/关节顺序错误、观察源、旧反馈与新心跳、排队过期及配置冲突。

```bash
PYTHONPATH=src/tianji_teleop:. pixi run python -m unittest \
  tests.test_native_session_health tests.test_native_session_runtime tests.test_native_session_commands -q
```

边界：这里只消费所需字段的 typed 消息，不是完整 wire schema 解码器；尚未迁移手部域、
动态授权替换、输入骨架或实际执行反馈通道。为防止把心跳序号冒充原始帧进展，该健康模式
暂时明确拒绝 rearm；必须接入真实 reset ack 和新原始输入事务后再启用。
完整 IK/MuJoCo、录制与现场入口接线仍未完成，不能用离线通过宣称现场性能已经提升。

本轮验证：专用状态/命令/健康测试 8 项通过；全量回归 1106 项、跳过 62 项，其余通过；
显式原生后端组合 68 项、跳过 2 项，其余通过。健康校验与线程接线的 ASan/UBSan、
`git diff --check` 通过。未启动现场设备，未提交或 push。

## 第八阶段：C++ 原生 worker reset 传输（已对接真实 worker，非设备）

`native/control/worker_reset_client.hpp` 使用 Linux `posix_spawn` 和本机 socketpair
直接启动已有原生 worker，以 stdin/stdout 原协议收发。C++ 自己处理 startup handshake、
`TJSR1` reset 与状态应答，不经过 Python IPC client。

- 检查 worker 算法/类型，reset 应答必须与请求的 14 个位置完全相等、速度和加速度全零。
  检查字段集合及数值类型，拒绝重复 JSON key、超长/过深数据及额外输出。
- 一个绝对超时覆盖请求写入和应答读取。失败关闭实例、回收直接子进程，不重试、不重启；
  epoch 只在应答通过后更新。只管理自身子进程，不按名称或进程组清理。
- 子进程环境与原 Python client 相同，隔离 PYTHONPATH/PYTHONHOME/LD_LIBRARY_PATH/LD_PRELOAD；
  不修改父进程环境。宿主需保证此 client 独占自己子进程的 wait/reap，不使用全局 reaper 抢收。
- 当前编译依赖 C++17、Linux POSIX 接口和 `nlohmann/json.hpp` 开发头文件。
  测试会明确报告缺少编译依赖或 worker，不能把跳过当作真实 worker 通过。

```bash
PYTHONPATH=src/tianji_teleop:. pixi run python -m unittest tests.test_native_worker_reset -q
# 可选：同一组传输及真实 worker reset 测试开启 ASan/UBSan
PYTHONPATH=src/tianji_teleop:. NATIVE_RESET_SANITIZERS=1 \
  pixi run python -m unittest tests.test_native_worker_reset -q
```

测试分别启动仓库构建的 SPARK、mapped-palm C++ worker，执行 epoch 2、3 的 reset。
故障注入覆盖错误位置、非零速度、布尔冒充关节值、重复字段、超长应答、超时和 EOF，
并检查失败不推进 epoch、连接不可复用、直接子进程已被回收。

第八阶段尚未将该 client 接入 SessionRuntime 的 rearm 事务（第九阶段已接入，见下）。它只复位 worker 模型/历史，
不证明执行器实测 Home，不确认新原始帧，不发布命令。健康模式的 rearm 仍明确拒绝，
待复位前后健康/Home 检查与新输入屏障接线后再启用；完整现场 C++ 替换仍未完成。

本轮验证：reset 专用 3 项（含两个实际 worker）及 ASan/UBSan 通过；全量回归
1108 项、跳过 62 项，其余通过（该轮开始后补充的启动边界测试已另行定向验证）；
最终原生后端组合 71 项、跳过 2 项，其余通过。`git diff --check` 通过。
未连接现场设备，未提交或 push。

## 第九阶段：C++ 异步 rearm 事务接入真实 worker（离线）

`SessionRuntime` 可选持有 `ResetEndpoint`，实际实现由 `WorkerResetClient` 提供。
只有固定授权机械臂健康模式、后端 epoch 与会话一致时才能配置该入口。未配置时仍拒绝 rearm。

- 复位前检查 idle、各域健康/新鲜、实测精确 Home、最终命令 Home，以及 next epoch。
  使用 owner 的 14 个 Home 命令发送请求，不采用事件中的 `reset_ack` 布尔值。
- C++ reset 工作线程执行 IPC；调度线程继续消费健康/反馈、维护 Home 命令和检测故障。
  reset pending 时不接受新的命令及操作转换；外部 `stop()` 可取消任务并回收 worker。
- 完成时以当前单调时钟重新检查反馈新鲜度、精确 Home 和返回 epoch，才提交会话 epoch，
  清空旧 proposal/history 并返回一次最终结果。收到 ACK 不等于无条件提交。
- 复位期间反馈过期/偏离 Home、后端失败/epoch 不符则锁存 fault，不推进会话 epoch。
  worker 可能已经改变内部状态，因此失败后必须重建会话，不自动补偿或重试。
- IPC 等待支持原子取消请求，按短轮询片段检查取消，不由另一个线程关闭正在使用的 fd。
  中断 stop 不报告成功 rearm 或成功 Home 退出。

```bash
PYTHONPATH=src/tianji_teleop:. pixi run python -m unittest tests.test_native_session_rearm -q
PYTHONPATH=src/tianji_teleop:. NATIVE_RESET_SANITIZERS=1 \
  pixi run python -m unittest tests.test_native_session_rearm tests.test_native_worker_reset -q
```

测试包括延迟 endpoint 下调度继续执行、过期/偏离 Home、错误 epoch、取消；另分别通过
真实 SPARK 和 mapped-palm worker 的状态 ACK 提交会话 epoch 2。真实 worker 测试的
健康及 Home 反馈由离线夹具提供，不是设备反馈。阻塞 IPC 取消也使用真实子进程验证。

**阶段 9 时新原始帧释放条件尚未接入（后续见阶段 10）**：提交 rearm 后 start 被新输入屏障拒绝，新的状态心跳
也不能释放它。此阶段只验证屏障不会误放行，尚未验证合法新帧能够恢复遥操。
因此没有新增现场开关，不能把本阶段当作可用的完整 native 遥操入口；原有路线不变。
此处的 epoch 提交只覆盖会话命令内核与 worker，尚不代表 producer/执行回执 guard/MuJoCo
等参与者的完整多方复位事务；这些参与者需要在实际控制周期接线时一并验证。
下一步需接入原始帧解码/流代际判定，再连接 IK 周期、MuJoCo、手部与录制。

本轮验证：rearm/reset 专用 6 项及 ASan/UBSan 通过；全量回归 1112 项、跳过 62 项，
其余通过；显式原生后端组合 74 项、跳过 2 项，其余通过。`git diff --check` 通过。
未操作现场设备，未提交或 push。

## 阶段 10：原始 TJVR 验证与 rearm 新帧屏障

新增 `native/control/raw_input.hpp`、`tjvr_input.hpp`，在 C++ session owner 中消费
带固定来源身份的有界原始数据报。复用仓库内原版 C++ TJVR decoder、stream gate 和
corrected-palm selector，未修改移植算法、骨架定义或模型/TCP。

- CRC/格式、序号、epoch、跳变确认沿用参考实现；拒绝帧不更新本地新鲜度或输入修订号。
- rearm 捕获输入 epoch/generation，提交时记录全部已处理输入及本地时间截止点。
  只有提交后入队、仍新鲜、同 epoch/generation 的新合法帧才能允许显式 `start`。
- 输入代际改变后不能释放旧屏障，必须在健康且精确 Home 条件下重新 rearm。
  状态心跳以及带 `start` 字符串标签的数据消息均不能充当操作请求清除屏障。
- 真正 SPARK/mapped-palm worker 的离线测试分别验证首次 rearm、新帧放行、重复帧拒绝、
  epoch 改变拒绝 start、再次 rearm 后新帧放行；健康与 Home 仍由测试夹具提供。
  消息标签绕过测试已验证去掉修复后两后端均失败。

这是**接收侧流接受与复位屏障接线**，不等于完整输入可用于 IK 的验收：没有监听 UDP，
没有把解码骨架交给实际每周期 IK，也未完成 SPARK 骨架可用性、运行中重同步策略及
多参与者 reset 的完整联动。`packet` 接受语义沿用参考接收器，不能替代上层骨架要求。
没有新增现场开关，已有 Python 默认调度及两条设备路线未切换。

验证入口：`tests.test_native_raw_input`；可用 `NATIVE_RAW_SANITIZERS=1` 编译运行
ASan/UBSan。测试需要 C++17、Eigen，实际 worker 测试另需 nlohmann-json 及已构建 worker；
缺失依赖时显式跳过，不视为通过硬件验收。

本轮实测：raw 专项 ASan/UBSan 2 项通过（含两后端及 epoch 变化子场景）；
全量回归 1114 项，跳过 62 项，其余通过；显式启用 SPARK/mapped-palm 的原生组合
76 项，跳过 2 项官方手部 opt-in 测试，其余通过。`git diff --check` 通过。
未做现场或真机验收，未提交、push。

## 阶段 11：C++ 逐周期 IK worker 客户端

`worker_reset_client.hpp` 在原 reset-only 模式之外新增显式 binary-step 模式，
复用已有 Linux 子进程、截止时间、取消及独占清理逻辑。`worker_result.hpp` 读取既有
TJBR v1，提供双臂 q/qdot/qddot、目标、QP/hold 状态、SPARK guidance 和 mapped-palm
高度偏移等完整类型字段，并保留原始结果帧供后续诊断接线。现有 Python 客户端及
两套 IK worker 源码、本轮均未因这一接口而改写。

- 请求仍为既有 TJSC1；要求连续 tick、递增 int64 时钟、合法接收时间及有界数据包。
  客户端不替代上游原始 TJVR 流门控，不进行骨架重映射或执行授权。
- 校验二进制版本、后端、固定长度、所有浮点有限性/布尔值，以及结果 tick、时钟和
  deterministic 模式；没有退回 JSON 或自动重启。诊断中的有限值校验不会只检查关节。
- 请求写入与响应读取共用一个超时；部分读写可继续，错误/超时/取消使实例关闭。
  本地参数错误发生在 I/O 前，不推进客户端时序。reset ACK 后 tick 从 1 重启，时钟不回退。
- 每套真实 worker 对照 80 周期（含新帧、epoch 变化及输入过期）加 1 个 reset 后周期，
  完整结果与 Python binary 客户端逐项一致；测试固定时钟并启用 deterministic-test，
  **不是现场时延或跟踪性能结论**。

这一步去掉了测试链路中逐周期 Python 请求/解码，但仍是 C++→worker 子进程 IPC，
不是同进程 IK owner，也没有接入 `SessionRuntime` 的实际 IK/guard/MuJoCo tick。
客户端的调用必须由单一 owner 串行化（仅 `request_stop()` 可并发），不能和异步 reset
同时 step。仍有字符串/结果缓冲分配，未宣称零分配或硬实时；完整控制链路及 HDF5
接线、生命周期对照、性能基准仍待完成。没有新增或切换现场 CLI 默认值。

验证入口：`tests.test_native_worker_step`；设置 `NATIVE_STEP_SANITIZERS=1` 可运行
ASan/UBSan。实际 worker 对照需要已构建二进制，编译测试需要 C++17/nlohmann-json。

本轮最终验证：step 专项 ASan/UBSan 3 项通过；原生组合 79 项，跳过 2 项官方手部
opt-in 测试，其余通过；全量回归 1117 项，跳过 62 项，其余通过。int64 最大正值
边界另经先失败后修复对齐验证。`git diff --check` 通过。未运行现场会话，未提交或 push。

## 阶段 12：C++ 独占 MuJoCo 与离线执行闭环夹具

新增 `native/control/owned_mujoco.hpp`，直接加载 MJCF 并独占 model/data 的创建和释放。
不使用 Python `_address`、不借用 Python 对象，不修改已存在的 CPython MuJoCo adapter。
按名字预绑定最多四组 scalar joints，可返回关节分组、完整 qpos 和指定 body 的世界系
位置/四元数（wxyz）。模型名字和路径由调用者显式提供，没有外部工程绝对路径。

- 一批关节全部校验后再写入，缺失组保持上次值，NaN/长度错误不会只改一侧。
  继续使用直接 qpos + `mj_forward`，没有切换为动力学 `mj_step` 或 PD。
- 模型与 data 不暴露可变指针，快照为复制值；读写限创建对象的 owner 线程。
  body 的观测坐标系是 MuJoCo 世界系，不是新的 IK 映射或新的 TCP。
- 两套完整模型的关节/FK 对照、四组更新与手部 hold、无效配置、跨线程访问等由
  `tests.test_native_owned_mujoco` 覆盖；body 对照使用 XML 中实际存在的
  `hand_tcp_mount_L/R`，不是未经核对的 `l_wrist/r_wrist`。

`tests/cpp/native_owned_cycle_driver.cpp` 将实际 C++ worker、`SessionCommands`、
`ExecutionGuard` 和新 MuJoCo owner 在同一**离线测试线程**中串联。两后端各 40 个
固定时钟控制帧，最终关节反馈逐项对照原 Python worker 客户端结果，并用实际模型的
反馈确认 Home/shutdown 完成。测试使用 MJCF 关节范围、显式宽步长/快速 Home 参数，
不是现场配置或性能对比；来源健康和操作授权由夹具提供，不证明真实输入门控完成。

尚未把该对象嵌入 `SessionRuntime` 线程的创建、tick、取消和销毁流程；生产输入、完整
receipt/schema、手部权限/retarget、录制与 viewer 快照出口仍待接线。因此不新增现场
入口，不改变两条已验证路线的默认值，也不称为完整实时 native 控制器。快照仍有内存
分配，MuJoCo 内核未获得命令授权或独立限位策略，上游 coordinator/guard 仍不可省略。

可用 `NATIVE_OWNED_MUJOCO_SANITIZERS=undefined` 运行 UBSan，`=1` 请求 ASan+UBSan。
当前 GCC 的 ASan 构建在所装 MuJoCo `mjsan.h` 函数定义后的 attribute 语法处失败，
尚未运行 ASan 检测；没有通过改 vendor 或屏蔽 sanitizer 宏规避。此项限制必须保留，
不能把 UBSan 通过写成 ASan 通过。

本轮验证：独占 MuJoCo 专项 UBSan 4 项通过；原生组合 83 项，跳过 2 项官方手部
opt-in 测试，其余通过；全量回归 1121 项，跳过 62 项，其余通过。`git diff --check`
通过。ASan 受上述头文件兼容问题阻挡。未操作现场设备，未提交或 push。

## 阶段 13：MuJoCo 接入 C++ 调度线程

新增 `simulation_endpoint.hpp` 与 `mujoco_endpoint.hpp`；`SessionRuntime` 可显式接受
纯 C++ simulation factory，在自己的线程中创建、执行、读取及销毁 MuJoCo。
不传 factory 的旧模式保持原行为，没有变更现场 CLI 或两条 Python 默认入口。

- 仅允许配套完整 arms-only health/command 配置。内部 executor 身份来自固定配置，
  实际反馈经过原有 finite/序号/顺序/身份验证；外部 executor status 与 arm feedback
  被拒绝，不能覆盖 owner 的测量。source/producer 健康仍是外部输入，不能自动伪造就绪。
- 每周期只执行协调器产生的最终命令，应用后读取仿真反馈，下一周期以实际反馈判断
  Home。关闭时清除 simulation readiness，并在所属线程释放 model/data；保留末次
  反馈快照不代表退出后的反馈仍新鲜。
- 模型加载在调度线程内、队列互斥锁外执行。加载期间可接收 stop，但底层同步加载
  函数本身不能取消，join 需要等待它返回；返回后若已停止，不执行初始 Home 写入。
- 首次内部反馈必须排在启动前入队的消息之后，不能为了隐藏时间倒序而改写接收时间。
  创建异常/空 endpoint、执行异常或非法反馈进入 fault；不能误报 shutdown/Home 完成。

`tests.test_native_runtime_simulation` 用两套真实模型验证线程生命周期、配对命令与
反馈一致、Home 退出、中断及上述故障。来源/producer 状态和 proposal 仍是测试夹具；
**IK worker 尚未由此调度线程调用**，也没有完成其 execution guard、原始帧采样、
多方 reset/高度标定事务接线，不能把这一阶段叫做完整现场 native 遥操。
没有网络监听、HDF5/viewer 出口或真实设备操作。调用/快照仍有分配，不宣称硬实时。

UBSan 验证开关：`NATIVE_RUNTIME_SIM_UBSAN=1`。ASan 仍受阶段 12 所述第三方头文件
兼容限制，本阶段未通过绕过宏或修改 vendor 来声称 ASan 完成。

本轮验证：线程仿真专项 UBSan 通过（两模型 × 七种生命周期/异常场景）；
原生组合 84 项，跳过 2 项官方手部 opt-in 测试，其余通过；全量回归 1122 项，
跳过 62 项，其余通过。`git diff --check` 通过。未运行现场会话，未提交或 push。

## 阶段 14：原生调度线程内 IK → 协调器 → guard → MuJoCo

新增 `ik_endpoint.hpp`、`worker_ik_endpoint.hpp`。`SessionRuntime` 可显式选择由其
线程创建/销毁的 IK worker 服务，使用已有二进制客户端；原 IK 代码、模型/TCP 未改动。
普通模式不传 factory，原行为不变；没有添加现场 CLI 或自动迁移默认入口。

- 仅支持 arms-only、已配 raw gate/owned simulation 的 reference-direct 命令模式，
  拒绝 clipping 和单独 reset owner 混用。外部 IK proposal/producer status 被拒绝，
  producer 就绪来自内部 worker 与有效骨架输入，不能通过外部消息伪造。
- 每个原始数据包先经原 TJVR 门控，再保存最近一份接受数据；控制周期最多消费一次。
  保留接收时间、断流标记和持续 generation，后续没有新样本时传空样本，不重复发送旧帧。
  初始输入要求骨架有效；这不是 PICO2/Manus 原始数据接线。
- 仅在显式 start 后的 teleop 状态调用 IK；结果经关联检查、协调器身份/数值/时序验证、
  内部 execution guard 精确关节比对后才执行。真实两后端测试验证反馈与 IK 参考一致，
  Home 期间不再执行 IK，Home 完成使用实际仿真反馈。
- IPC 等待释放队列锁，stop 可取消等待。等待期间入队的健康/操作/原始帧在结果采用前
  处理，保持接收顺序；return 或 fault 会丢弃晚到结果。队列溢出在 IPC 返回时立即
  fault，不能多执行一帧。该竞态已由先失败后修复的注入测试覆盖。
- 校验失败、NaN、越限、过期、取消均不能执行坏结果；实例退出后同线程释放 worker。
  仍有同步 IPC、复制与分配，并非同进程零分配 IK，也未获得硬实时资格。
  worker factory 的同步启动期间尚未发布 endpoint，stop 需要等待构造返回或启动超时；
  可取消 IPC 的保证针对 endpoint 已创建后的 step，不覆盖任意 factory 的阻塞代码。

**阶段 14 时的限制（rearm 已在阶段 15 扩展）：**本原生组合只支持一次 start→teleop→Home/退出。
回 Home 后明确拒绝再次 start 和 rearm；不能把上一 worker 模型状态隐式用于新一轮。
之前独立 reset 模式仍可用，但未与此 owned IK、guard、MuJoCo 的多方事务合并。
高度标定、完整 receipt/HDF5、viewer、手部、完整网络输入及现场验收仍待实现。
因此它是离线可运行的 native 组合，不是替代现有现场脚本的新默认方案。

`tests.test_native_scheduled_ik` 包含两真实 worker 的线程仿真及受控故障 endpoint：
IPC 中取消/return、队列溢出、错 tick、非有限数、越限和过期。故障 endpoint 只在测试
driver 内，生产代码不读取测试环境变量。开启 `NATIVE_SCHEDULED_IK_UBSAN=1` 跑 UBSan；
ASan 仍受阶段 12 的 MuJoCo 头文件限制。测试固定配置和 deterministic IK 不代表现场性能。

本轮验证：调度 IK 专项 UBSan 通过（27.313 s）；原生组合 85 项（74.139 s），
跳过 2 项，其余通过；全量回归 1123 项（132.132 s），跳过 62 项，其余通过。
`git diff --check` 通过。未切换现场默认入口、未操作真实设备、未提交或 push。

## 阶段 15：owned IK 与 Home rearm 事务

`IkEndpoint` 增加可选的借用 reset 服务，`WorkerIkEndpoint` 使用原来的同一个
`WorkerResetClient`，不创建第二个模型状态。没有该能力的 endpoint 继续拒绝 rearm。
独立 reset 模式仍保留；禁止同时配置两个 reset owner。

- 仅显式授权、idle、双臂真实反馈和最终命令均精确 Home、输入新鲜且有效、
  worker/session epoch 一致、请求恰为下一 epoch，才启动异步 reset。
- 正常离开 teleop 引起的 guard 暂停不等于 worker 故障，允许进入上述重置事务；
  session fault 仍不可用 rearm 清除。reset 待定期间不执行 IK，也不接受 start。
- 调度器继续产生 Home 命令和实际仿真反馈；持续检查健康、输入时效和源代际。
  真实 worker 重置可能超过一个输入新鲜度窗口，因此等待期间仍需连续有效输入。
- 真实 ACK 校验由原 C++ 客户端完成（精确关节位置、零速度/加速度、请求 epoch）。
  在 ACK 后再次检查条件，再提交 coordinator epoch、重建 guard、清零当前 epoch
  IK tick 和旧参考/待采样输入。仅提交后的新帧、相同源代际和显式 start 可恢复运行。
  提交前新帧和重复帧均不能解除屏障。
- reset 失败、错 epoch、源代际变化、断流或队列溢出均不能提交新 session epoch；
  即使 worker 已重置，也不回滚后继续运行，而是保留 fault 并要求重建会话。
- stop/fault 清理先取消并 join reset，再在 owner 线程销毁 IK；不伪造 Home 完成。
  同一个客户端不会并发执行 step/reset，只有取消接口可跨线程调用。

测试扩展 `tests.test_native_scheduled_ik`：两套真实 worker 均验证
start→Home→ACK→新帧→再次 start→Home 退出；另用受控 reset 服务注入成功、取消、
源代际变化、错误 ACK epoch、异常、断流与溢出，检查调度进度、无 IK 并发、
epoch 不误提交及对象释放顺序。测试服务不进入生产代码。

这仍是 **arms-only 离线原生组合**：高度标定事务、完整网络/手部、录制/可视化出口
和现场验收尚未接入。现有现场入口及其默认行为不变，不宣称硬实时或现场性能提升。

本轮验证：扩展后的调度 IK 专项 UBSan 通过（33.052 s）；原生组合 85 项
（76.789 s），跳过 2 项，其余通过；全量回归 1123 项（131.754 s），跳过 62 项，
其余通过。`git diff --check` 通过。未操作设备、未提交或 push。

## 阶段 16：C++ mapped-palm 高度标定事务

新增常量存储的 `height_calibration.hpp`，按既有 Python 规则采样：2 秒窗口、
双臂各至少 30 个不同接收时间戳、至少 1.5 秒跨度、250 ms 最大间隔、各轴 6 cm
稳定性门槛及绝对值不超过 1 m 的 Z 偏移。源 epoch/generation 变化不能混入一次标定。
调度器只消费每周期最近一份被原门控接受的样本；不改写原始 TJVR 字节。

`MujocoEndpoint` 的参考值来自原模型 `hand_tcp_frame_L/R` 在双臂 q=0 时的 site Z，
使用独立 mjData；不会把水平参考姿态写入执行数据。已与原 `horizontal_tcp_heights`
数值对照。不是更改 TCP、X/Y 平移或姿态映射。

原生会话通过显式构造选项开启此能力；无 CLI/默认行为变化，SPARK worker 明确拒绝。
首次 start 要先标定；仅授权且健康精确 Home 时可开始，采样中拒绝重复请求/start/rearm。
采样失败保留已提交的旧结果，正常 return/shutdown 可取消采样。

采样完成后，在异步任务中顺序执行同一个 worker 的 reset ACK 和 `TJMH1` 高度 ACK。
两次 ACK 均通过、且输入代际/健康/精确 Home 后置条件仍成立，才提交 coordinator epoch、
guard、输入屏障和标定结果。reset 成功但高度 ACK 失败时，session 不提交新 epoch，
进入 fault；不能隐式回滚继续执行。取消先 join 整个任务再销毁 IK。
IK 每次输出还要与已提交高度值精确匹配。普通 Home rearm 保留 worker 中既有偏移。

测试包含：与 Python 逐步采样对照（正常/移动/重复/失效/断流/代际/失败重试/样本不足）、
模型只读参考对照、真实 worker 高度 ACK/跨 reset 保留、错误/布尔/重复键/过长/超时/EOF ACK，
完整调度标定→启动→Home，以及高度 ACK 失败和等待中取消时不误提交状态。
测试注入接口不进入现场配置。高度状态快照尚不是完整 HDF5 标定审计出口。

## 阶段 17：双臂回执元数据与输出侧编码

内部 `CommandReceipt` 现在随生成时刻保存 run/router、execution epoch、tick 和 timestamp，
不再依赖消费时的最新 session epoch。rearm 后 tick 可重新从 1 开始，旧回执仍可区分。
`receipt_wire.hpp` 在消费侧生成既有 `arm_bilateral_receipt` 全字段格式；控制 tick
不调用 JSON 编码。输出者必须显式提供自己的 coordinator instance，编码函数不注册
身份、不发布消息，也不授予新命令权。

已与 Python 协调器逐字段比较正常、裁剪和故障回执，包括双臂最终命令、原因、时间、
身份和 stage；补测新 epoch 的 tick=1 元数据。完整输出发布、录制错误传播和消费者
生命周期仍待接入，不能将格式一致视为已经完成现场链路。

阶段 16–17 验证：调度 IK/高度场景 UBSan 通过（42.830 s）；包含原版高度测试的
原生组合 95 项，跳过 2 项，其余通过；回执字段对照通过；全量回归 1126 项
（147.703 s），跳过 62 项，其余通过。

## 阶段 18：固定数组仿真命令与反馈

`OwnedMujoco` 新增固定大小数组的批量写入与关节组读取；`MujocoEndpoint` 的双臂
路径直接传递 `ArmPair`，不再为 14 个关节构造临时 vector 或先复制完整 qpos/body 快照。
原可选手部分组、null-group 保持和完整快照接口不变。

固定批量写入仍先检查 owner、分组维度、全部关节有限性，再写入任一关节并执行
原有 `mj_forward`；没有改成动力学积分，也没有取消上游协调器/限位检查。
越界、错误分组大小、跨线程访问及后半批 NaN 测试均要求不改变原始执行状态。

两套真实模型的分配计数测试确认固定反馈为 0 次 C++ `operator new`；新增写入测试
先复现 100 次循环产生 400 次适配层分配，再验证固定数组路径降为 0，数值保持一致。
此计数不覆盖 MuJoCo 内部 C 分配器，也不代表整个控制循环零分配或已达到硬实时。
owned MuJoCo 5 项 UBSan 测试通过（25.861 s）。

本批最终全量回归：1127 项（148.964 s），跳过 62 项，其余通过；`git diff --check`
通过。默认现场入口保持不变，未操作设备、未提交或 push。后续仍需接入完整输入、
输出消费者与现场仿真验收，不能据此宣称整个工程已完成 C++ 化。

## 阶段 19：原生数据报接收与有界输出消费者（离线接入层）

新增 `native/control/datagram_receiver.hpp`：

- 独占接管调用方移交的数据报 socket，设置 close-on-exec；不 bind/rebind、
  不启动 SDK、不声明来源身份、不授予运动权限。
- 独立 C++ 接收线程使用 `recvmsg`，接收后立即记录本地单调时间。
  拒绝空包、超过 656 字节及 `MSG_TRUNC` 包，不能将截断前缀当作有效 TJVR。
- 大小合规仅表示可交给解码器；CRC、epoch、序号、跳变、掌心选择仍由原版
  `TjvrInput` 门控。接收计数不会替代有效输入新鲜度。
- 接收端 sink 必须为有界非阻塞提交；拒绝或异常锁存失败。停止通过 eventfd 唤醒，
  join 后关闭自身描述符，不跨线程 close 正在被 poll 的 socket。

新增 `native/control/bounded_output.hpp`：

- C++ 独立消费者处理按值移交的 FIFO 项，容量 1–65536；队列长度有界，
  payload 大小仍由接入方的 typed schema 约束。
- 满队列不静默丢弃/继续成功；写入、关闭和取消失败保留错误。
- `finish()` 仅在生产者停止后调用，排空并成功执行 finalize 才设置队列 complete。
  `abort()`/析构不会执行成功 finalize；可取消另一个线程正在等待的 finish。
- write/finalize 必须提供有界 I/O，cancel 必须支持并发取消。该通用层不能强制
  中断任意阻塞回调；正式 recorder/publisher 适配必须落实超时与取消契约。
- **队列 complete 不等于 session HDF5 complete**，尚未连接磁盘 schema、
  Zenoh 授权发布者或完整会话 supervisor；失败必须由后续 supervisor 接入门控。

`tests/test_native_session_io.py` 包含接收资源/大小/失败/退出测试、
消费者顺序/溢出/写入及 finalize 失败/并发取消测试，以及组合夹具：
AF_UNIX socketpair → 接收线程 → 原版 TJVR gate → 原生状态机/命令 → 完整回执消费。
组合覆盖 packet 与 mapped 两种目标选择，验证重复包/坏包拒绝、接收不自动开始、
重复包不能续命、源过期故障保持、回执身份/epoch/tick 及结束排空。
健康输入和关节 proposal 为离线夹具，不构成完整现场命令链验收。

复现（无网络监听、SDK 或设备）：

```bash
PYTHONPATH=src/tianji_teleop:. pixi run python -m unittest tests.test_native_session_io -q
NATIVE_SESSION_IO_SANITIZERS=1 NATIVE_RAW_SANITIZERS=1 \
  PYTHONPATH=src/tianji_teleop:. pixi run python -m unittest tests.test_native_session_io -q
```

原有两条路线的 CLI/默认值、本体 IK/坐标系/TCP 均未因阶段 19 改动。
这一步移走了新增接入组件中的 Python 线程依赖，**未证明当前现场循环提速或已全 C++ 化**。

阶段 19 全量回归：1130 项（155.532 s），跳过 62 项，其余通过。
随后消毒器复测暴露了组合夹具中“单次关节 proposal 过期”和“raw 过期”均为
200 ms 的竞争断言；仅将夹具 proposal 期限分离，未改生产门控/期限。
修订后的 3 项 ASan/UBSan 复测通过（31.481 s）。未操作真实设备，未提交或 push。

## 阶段 20：原生输出接线与独立 I/O 故障通道

新增 `native/control/session_output_bridge.hpp`，由专用 C++ 线程消费运行时的
操作回执与双臂命令回执，再交给有界输出线程。每个队列各自保留 FIFO 顺序，
分批公平消费；不将两个独立队列伪装成全局时间排序。接入方不能再同时 pop 同一队列。

`SessionRuntime::report_failure()` 是**本地 owner 的停机故障接口**，不是网络协议、
operator intent 或运动授权。保留首个报告，限制原因长度，不占普通事件队列。
控制线程在周期开始、IK 返回后采用结果之前应用故障；空闲 deadline 等待可被唤醒。
本轮没有改动 IK 算法、模型、映射、TCP 或原来的新鲜度/限位配置。

- 输出满队列、write 异常会通过该接口使控制进入 fault，保持上一条命令；
  不因输出消费者堵塞而在控制线程内执行写入。
- I/O 故障发生于 IK 在途时，迟到结果不能更新命令、参考快照或已采用 tick。
  发生于 reset 时请求取消，旧执行代次保留，不将 late ACK 提交为成功 rearm。
- `DatagramReceiver` 增加可选失败通知；在 stats 锁外调用，可连接该故障接口。
  通知自身异常单独记为 `notification_failed`，不覆盖原始接收错误、不终止进程。
- 先停输入生产者，再关闭 bridge。`finish()` 停止 runtime 并排空其输出；
  只有原生状态已经完成显式 shutdown/Home 时才向 sink 传递完成资格。
  直接中断只传递 incomplete；finalize 失败不报告 `session_complete`。
- `abort()` 支持取消阻塞的输出操作，然后 join；重复关闭幂等。
  write/finalize/cancel 仍须满足上一阶段的有界 I/O 与并发取消契约。

完成资格只覆盖这里管理的原生运行时和回执流；**不是完整 HDF5 文件已验收**。
sink 仍需核验其全部 raw/diagnostics/hand 数据已排空并成功关闭，才能设置文件 complete。
目前尚未声明 Zenoh 发布者、创建完整 session schema 或接入现场 CLI；默认两条路线未切换。

测试覆盖源接收失败通知、输出失败/溢出/关闭失败、队列满时故障通道、长 deadline
唤醒、正常 Home 结束与中断的区别；原生 IK/MuJoCo 组合另覆盖在途 IK 与 reset 的 I/O 故障。

阶段 20 验证：I/O 4 项 ASan/UBSan 通过（48.049 s）；调度 IK/MuJoCo 组合
UBSan 通过（43.204 s）。全量回归 1131 项（159.196 s），跳过 62 项，其余通过。
顺序复核及差异空白检查通过；未使用子智能体、未操作真实设备、未提交或 push。

## 阶段 21：C++ HDF5 列编码与有界流传输

新增 `native/control/hdf5_stream_client.hpp`，复用现有 `native/hdf5_recorder`
磁盘进程的协议，不引入第二种录制格式，也不改变现场 recorder 默认入口。

- `Hdf5Block` 在 C++ 中编码 int64、float64、uint8、UTF-8 字符串和变长原始字节列，
  明确使用小端格式。保留缺测浮点 NaN；运动数值检查仍由控制层负责。
- 限制单块 64 MiB、每列 1–4096 行、列和属性各最多 2048 项；拒绝重复列、相对路径、
  NUL 文本及非白名单属性。无效追加不改变已有块。列类型、尾部维度和领域数据合法性
  由 schema/调用者保证，不将这个低层编码器当作控制数据验证器。
- `Hdf5StreamClient` 接收独占移交的已连接 UNIX stream 描述符；不监听、不启动进程、
  不获取发布权限。启动 schema 和磁盘子进程仍由接入方负责。
- I/O 单线程所有，支持另一线程请求取消；非阻塞 send/recv、绝对超时及最长 10 ms
  poll 分段检查取消。发送使用 `MSG_NOSIGNAL`，不修改进程全局 SIGPIPE 行为。
- 严格校验 READY/OK 回执。断连、异常回执、超时或取消后锁存失败、关闭描述符，
  禁止后续补发成功关闭。析构不发送 complete；磁盘进程的回收仍归启动者。
- `close(true)` 必须由上层确认全部生产者停止、数据排空且会话满足完成条件后调用；
  本客户端不会自行判断。若 C 已到磁盘端但其 ACK 丢失，调用方只能报告关闭结果未知，
  不能假定磁盘 complete 一定为 false。此处不改变既有协议的这一边界。

离线测试使用 socketpair 和已有 C++ 磁盘 writer。比较 session schema 1.2 下两条原始
TJVR 帧和审计记录的所有数据、类型及属性，分别验证 complete/incomplete 关闭；另外
检查负整数、矩阵、NaN、空变长行、UTF-8 属性及写端堵塞。没有启动现场输入或网络监听。

这一步完成的是**通用录制编码与传输组件**，还不是原生运行时完整录制接线。
完整周期快照、手部数据、诊断列组装及现场入口仍待接入；不能据此宣称现场循环已提速。
本轮全量回归 1135 项（162.525 s），跳过 62 项，其余通过。之后追加实际磁盘拒绝
不存在列、客户端不能再成功关闭、文件保持 incomplete 的测试；最终专项 5 项通过，
ASan/UBSan 同样 5 项通过。全量计数不包含最后追加的这一项，未将旧计数冒充新运行。

```bash
PYTHONPATH=src/tianji_teleop:. pixi run python -m unittest tests.test_native_recording_transport -q
NATIVE_RECORDING_SANITIZERS=1 PYTHONPATH=src/tianji_teleop:. \
  pixi run python -m unittest tests.test_native_recording_transport -q
```

未操作真实设备、未提交或 push；两条已验证路线的默认启动方式未改。

## 阶段 22：原生完成周期快照与输出线程接线

`SessionRuntime::enable_cycle_capture(capacity)` 在启动前显式启用周期录制出口，
容量为 1–8192；默认关闭。`pop_cycle()` 是独立的单消费者接口，返回值拥有自己的
数据，不引用正在变化的 worker 或 MuJoCo 内存。接入 `SessionOutputBridge` 后，
由它独占消费操作回执、命令回执及周期队列，输出回调仍不在控制线程执行。

每个**完成的调度周期**记录：周期序号/完成时间、当时会话状态、命令与回执、实际反馈、
高度标定状态、最新源进度/接收时间，以及本周期可选的完整 worker 请求与返回结果
（含原始诊断 wire）。`ik_adopted` 只有结果通过采用流程且命令回执接受时才为真。
idle/Home 周期不借用上次 IK 的 request/result；最新源进度与本次 worker 输入分别保存，
避免把 IPC 等待期间新到的数据误当成本次 IK 使用的数据。

- 队列空间在周期执行前检查；满队列锁存 `cycle_capture_failed` 并进入 fault，
  不继续采用 IK，不覆盖已有快照。pending reset 请求取消，后续不能通过 rearm 清掉故障。
- 复制的请求 packet 最多 656 字节、结果 wire 最多 1206 字节，容量按条数和载荷共同约束。
- 正常 shutdown/Home 的最后周期先进入队列，再退出控制线程；bridge 排空三种队列。
  三个队列各自 FIFO，不宣称跨队列全局时间排序。
- 取消中断/抛异常的未完成周期不伪造成完成周期；接入方必须同时读取最终 runtime 状态和
  I/O 失败状态，不能仅凭队列排空或最后一条旧快照判断整个 session 完整。

实际 SPARK、mapped-palm C++ worker/MuJoCo 测试覆盖跨 rearm 的逐周期序号、完整结果
wire、命令与反馈及 Home 终帧；受控迟到结果测试覆盖 return/I/O fault 后记录结果但不采用。
独立测试覆盖默认关闭、配置边界、队列满及复制出的快照不被后续状态改写。

这仍不是现场全量 HDF5 已接齐：快照到既有全部列的组装、原始输入全帧审计、手部域及
现场发布者/入口仍待迁移。原来的两条默认现场路线、IK 算法和映射均未切换。

阶段 22 全量回归：1136 项（160.521 s），跳过 62 项，其余通过。
原生 IK/MuJoCo 组合 UBSan 通过（44.568 s），I/O 组合 4 项 ASan/UBSan 通过
（50.991 s）；之后将周期对象改为仅启用采集时构造，全量回归已覆盖该优化。
优化后的 `test_session_runtime.cpp`、`test_session_output_bridge.cpp` 又分别重新编译运行，
ASan/UBSan 均通过（`-O1 -g -fsanitize=address,undefined -fno-pie -no-pie`）。
未提交、未 push、未使用子智能体或操作真实设备。

## 阶段 23：机械臂快照到既有 HDF5 列的 C++ 编码

新增 `native/control/arm_cycle_hdf5.hpp`，在输出侧将完成周期中的双臂命令、位置反馈、
会话状态编码为原 session HDF5 的 `joint/command/arm/{left,right}`、`joint/state/arm`
和 `meta/session_events` 全部列，交给前一阶段的 C++ stream client/磁盘 writer。
没有新增数据集或更改已有 schema。

`ArmCycleRecordMetadata` 必须由录制/发布 owner 提供真实的发布者、逻辑 ID、消息序号、
proposal/target/intent 关联序号以及共享时间轴下的 `time_ns`。这些字段不从 IK tick 猜测；
消息序号、IK tick 和接收帧序号不能互相替代。编码器只录制，不获取发布或运动权限。

- 固定保持左臂 7 关节、右臂 7 关节的标准顺序，保留 `names/joint_names/logical_id` 属性。
- 可空序号写入 -1 和独立有效位，与 Python writer 一致；零序号不误判为缺失。
- 当前原生快照没有速度反馈，按既有约定写 `velocity_valid=false` 和 14 个 NaN，
  不把缺测速度伪装成零速度。
- 缺命令或反馈、缺身份、非法时间/序号/模式、非有限关节值在任何磁盘操作前拒绝。
  fault 会话的命令 mode 仍需 owner 显式提供合法 idle/teleop/returning 值，不能把
  会话状态枚举直接强塞进不同的命令模式枚举。
- 编码是无状态的，不自行决定时间轴原点或发行序号，不负责防重/重排或完成关闭。

离线测试通过同一 C++ 磁盘 writer 写入两个周期，与 `SessionH5Writer` 逐数据集比较
数值、dtype 和全部属性，并通过 `SessionH5Reader` 校验。专项 6 项通过，
ASan/UBSan 同样 6 项通过（6.667 s），包含异常字段拒绝与之前的传输失败测试。

**剩余范围未勾成完成：**全量 `native_cycle` 审计负载、原始输入全帧、手部域以及
实际发布 owner 元数据接入仍待实施。此编码器并未替换现场录制默认路径，不宣称
整个 `append_live_cycle_snapshot` 已实现等价替代，也没有仅凭这些列就标记 session 完整。

## 阶段 24：官方 Hand2 低通滤波的可选 C++ 接入

新增 `native/hand/lowpass.cpp`、`scripts/build_native_hand.py` 及
`producers/native_hand_filter.py`。原 worker 在首个回调前按显式选项替换两侧滤波对象，
不改动固定版本的第三方 retarget 源码；SDK、设备连接与命令发布者不变。

```bash
pixi run build-native-hand
```

在两条原仿真启动命令前与 router 环境变量同级添加 `TIANJI_HAND_FILTER_BACKEND=cpp`；
默认/`python` 保留原行为。独立 worker 也支持 `--filter-backend cpp`。
不启用手部的会话无需构建或加载该库。启动 C++ 失败不自动切回 Python。

- C++ 状态按实例独立，固定 20 关节；保留首次输入直出和 reset 后重新初始化。
- 原优化器输出 float32：逐步运算维持原单精度舍入，禁止 FMA 合并；需要时按参考行为
  提升为 float64。第一次真实 worker 对照发现约 1e-8 rad 差异，据此修正了精度语义，
  没有用放宽容差掩盖差异。
- reset 仍由原 bridge 的每侧 sequence/gap 逻辑驱动；没有改变 Manus 连续采样适配，
  PICO2 仍由各侧自己的 worker/有效性控制，缺失侧不推进滤波。
- ABI-independent `.so` 可供主环境与固定手部 Python 环境使用；调用者必须串行使用
  next/reset/close。非法数据不修改滤波状态，返回数组不与原生状态共享存储。
- VR/Manus resolved 配置和 PICO2 session 录制元数据在启用时记录 backend、ABI、库/源码/
  适配器 SHA256；原默认配置的元数据保持不变。

验证包含从固定源码提取的 LPFilter 数值对照、混合精度/重置/独立侧状态、无效输入不
污染状态、真实官方 worker 的 Python/CPP 输出，以及现有 PICO2/Manus 客户端在 CPP
环境变量下的调用。34 项相关回归（跳过 5 项）通过；随后启用固定环境的参考测试，
15 项通过；CPP 滤波环境下另外 11 项通过。原生 C ABI 夹具 ASan/UBSan 通过。
阶段 24 全量回归：1141 项（171.930 s），跳过 62 项，其余通过。
`pixi run build-native-hand` 构建通过，差异空白检查通过。

这只是滤波子项接入，**不是完整手部 retarget C++ 化**。优化器/几何预处理、Python
worker/IPC、现场调度仍有后续迁移；单独 ctypes 调用有边界开销，不宣称这一小步已提速。
没有启动现场设备或提交/push。

## 阶段 25：可选 C++ 手骨架几何预处理

`native/hand/geometry.cpp` 使用固定大小 Eigen 矩阵实现原版的输入轴反射、SVD 腕坐标系、
左右手 MANO 基变换、外旋 XYZ 对应旋转矩阵和腕/拇指偏移。旋转矩阵由原 SciPy 仅在启动
时生成；每帧几何运算在 C++ 内完成，不改 pinned 第三方源码。`wuji_hand_worker.py` 在
首个回调前接入 `TIANJI_HAND_GEOMETRY_BACKEND=cpp`，可与 C++ LPFilter 独立组合，
PICO2/Manus 原始驱动、优化器、丢帧 reset、限位、发布权和默认入口不变。

异常边界：拒绝非有限数据、无效配置和腕/食指根/中指根共线的非唯一坐标系，失败不写
输出；这比原版退化输入可能产生任意 SVD 轴更严格，不能宣称所有退化输入接受集合等价。
非退化几何对照 600 帧，绝对容差 2e-13 米；实际 pinned 双手 worker 6 帧含序列 gap，
关节输出绝对容差 1e-7 rad。C++ 几何＋滤波组合的官方手部/PICO producer 回归 11 项通过。
固定单帧、每轮 10000 次、3 轮 `timeit` 微基准（包含 ctypes/数组封装）：原版每次
53.8–55.5 μs，C++ 适配器 3.91–4.11 μs。此测量与回归任务同机运行，不是受控实时负载测试，
不包括优化求解、worker IPC 或控制循环，只说明该几何子段的本机耗时。ASan/UBSan C ABI
夹具通过，覆盖非有限输入、空指针、长度/旋转校验及失败不写输出。
未做现场设备验收或整链路性能对比；完整求解器、输入解析、调度接线及录制集成仍未收尾。

本阶段验证：`pixi run build-native-hand` 通过；`PYTHONPATH=src/tianji_teleop:. pixi run
python -m unittest discover -s tests -p 'test_*.py' -q` 共 1144 项，167.307 s，跳过 62 项，
其余通过。差异空白检查通过；未提交或 push，也未启动任何现场硬件会话。

## 阶段 26：独立可选的 C++ Hand2 优化器

原 `AdaptiveOptimizerAnalytical` Python 文件没有修改，默认选择仍为 Python。
`native/hand/optimizer.cpp` 在隔离 Hand2 worker 中执行目标构造、Pinocchio FK/世界位置
Jacobian、解析目标/梯度、NLopt SLSQP 及 warm-start，迭代中不回调 Python。
构建 `pixi run build-native-hand-optimizer`，显式设置 `TIANJI_HAND_OPTIMIZER_BACKEND=cpp`；
与几何/滤波选项独立，可组合。硬件驱动、SDK、发布门控、Manus 回调顺序及 PICO2 独立
侧输入没有更改。录制记录该库、适配器、源文件和锁定依赖摘要；重库只在 worker 中校验
加载，不注入主进程的 Python/Pinocchio 运行环境。

保留 20 关节顺序/URDF 限位、厘米单位、原损失与归一化 epsilon、捏合混合权重、
thumb_skip_pip、hyperextension/coupling 可选项、50 次评估及 ftol_abs=1e-4。
每次成功结果先转 float32 再保存 double warm state，与原实现一致。普通 NLopt failure
沿用初值回退（stderr 报告），roundoff/forced-stop 等其他失败不静默输出初值。
无效帧/输出缓冲区不提交状态；跨线程使用/显式关闭被拒绝，安装仅允许未运行的优化器。
不迁移原 Python 逐次 timing/画图诊断；新接口供现有 worker 的 solve/reset/cost 路径使用。

本轮离线证据（不是旧录制关节命令的全链路复现或现场验收）：

- 双侧×默认/可选惩罚：80 组 cost/gradient 对照（含捏合权重），80 次连续 solve
  覆盖 reset、显式初值；cost/梯度绝对容差分别 1e-11/1e-10，solve 2e-5 rad。
- 完整双手 worker 40 回调含 gap，对照原版输出；非法 backend 拒绝启动，默认元数据不变。
- Manus 既有 `pico_vr_manus_20260913_012835.h5` 前 1000 回调：左/右最大关节差异
  7.4506e-9/2.9802e-8 rad；本机一次含启动/JSON 通信/滤波的离线处理约 3.69/0.73 秒
  （Python/C++），不是实时负载保证。
- PICO2 既有 `pico_20260910_221810.h5` 前 1000 原始帧，按原适配器和独立侧 worker
  回放；左 1000、右 825 有效调用，关节输出差异均为 0。这是在当前同一配置下比较两种
  优化器，不是重放当时实际 consumption audit 的授权/调度结果。

完整 native 输入解析、现场调度接线、录制热路径整合仍是剩余工作；本阶段不是将整个
worker 改成 C++，也没有默认切换、实物验收、提交或 push。

sanitizer 检查使用锁定环境的 libstdc++，并指定 `EIGEN_MALLOC_ALREADY_ALIGNED=1` 与
预编译 Pinocchio 的 Eigen 分配约定一致（ASan 会改变 Eigen 的自动检测结果）。ASan/UBSan
均保持启用，C ABI 夹具通过；预编译的 Pinocchio/NLopt 本身没有重新做 sanitizer 构建。
覆盖真实模型 solve/cost、无效输入不写输出/不改状态、跨线程拒绝及 reset/destruction。
另有安装失败回滚测试，确认一侧初始化失败时两侧仍保留原 Python 对象。

全量回归首轮暴露旧 UDP 测试等待条件竞争：`datagrams` 在 ingest 入口递增，不能表示
raw sink 已完成。受控离线重现得到入口 datagrams=4/raw=2，回调完成后 accepted=2/raw=3。
只将测试等待条件改为最终 accepted=2，保留 datagrams/raw/重复帧断言；未改 UDP 生产实现。

最终验证：`pixi run build-native-hand-optimizer` 构建通过；显式启用
`WUJI_REFERENCE_TEST=1` 的新优化器检查 3 项通过（子进程包含原版数值/状态/worker 对照）；
几何＋滤波＋优化器三个 C++ 开关组合的既有手部/PICO producer 检查 11 项通过。
全量默认回归最终 1147 项、171.225 s，跳过 63 项，其余通过。差异空白检查通过。

## 阶段 27：可选 C++ Hand2 worker 与固定频率调度链

本阶段把驱动输出之后的 Hand2 worker 和调度内核合并为一个可选的 C++ 子链。实际边界为：

```text
PICO/Manus 已有驱动与 Python 输入适配
    → TJHI（二进制 canonical 21 点）
    → C++ fixed-rate scheduler
      → geometry → AdaptiveOptimizerAnalytical → LPFilter → limit/permutation
    → TJHO（二进制 Hand2 命令）
    → 现有 Python 授权 producer / Zenoh 发布 / MuJoCo hand executor
```

新增 `native/hand/pipeline.hpp` 统一旧 C++ worker 与 scheduler 的几何、优化、滤波、限位和
关节排列实现；`native/hand/scheduler.hpp` 负责固定周期、latest-input、session/epoch、
generation、新鲜度、定长输出队列和故障锁存。`scheduler_wire.hpp` / `scheduler_main.cpp`
提供固定 little-endian `TJHS/TJHI/TJHO/TJHK` 协议，C++ 调度线程不调用 Python callback，
也不连接设备、Zenoh、机器人或 MuJoCo。异常线程、输出写失败、队列溢出和 teleop stale
均 fail-closed；过期 teleop 帧在进入有状态 retarget/filter/optimizer 之前即被丢弃，
不会污染恢复后的首帧状态。scheduler 的 stdin/stdout 使用可中断的有界 POSIX I/O，
读端或写端退出时会唤醒另一侧并回收线程。

Python 适配器提供两个明确模式：原有同步 `retarget()` 保留给 Python worker/A-B 测试；
显式 `--hand-scheduler-backend cpp` 使用异步 `submit_retarget()`/`poll_retarget()`。
提交只做有界写入，独立 reader 持续排空 `TJHO`，应用 loop 即使暂时没有新驱动回调也能
消费完成结果；输出在适配器边界按 latest-result 合并。PICO2 的左右 scheduler 只有在
同一帧序列的两侧都完成后才组成双手结果，session 切换会丢弃旧 phase 的结果，避免回位
前的输出成为下一次 teleop 的首个命令。

2026-09-14 异步边界修复：

- PICO 双侧采用一对在途输入加一对最新等待输入；只合并尚未提交的帧，避免左右
  scheduler 独立跳帧后无法匹配。重复 idle 心跳不清空准备中的结果；phase 切换清空配对。
- client 在 phase/epoch 切换时记录输入序号屏障，隔离已经缓冲及之后到达的旧结果；
  新会话必须使用新输入。双侧结果保留并检查 epoch、phase 和处理状态。
- 过期输入返回无有效关节的完成结果，让等待中的双手配对可以结束并处理下一帧；
  worker 无返回时使用客户端事务超时报告故障。读取线程故障即使没有新输入也会上报。
- 处理审计成功后才发布命令；审计期间允许停止事件入队，发布前重新检查授权、
  输入年龄及停止事件。C++ 求解失败唤醒调度线程，并取消进程的输入等待。

以上是离线可靠性修复；默认 Python 路径和现有硬件驱动保留。真实设备延迟、长时间运行、
输出合并的完整审计计数及 C++ 求解期间锁竞争仍需后续验证，不能据此宣称整体迁移完成。

本轮验证：原生 scheduler 重新构建成功，C++ 调度测试通过严格编译检查及 ASan/UBSan；
完整 Python 回归运行 1198 项（1133 通过、65 跳过）。完整回归启动后补充的协议状态检查
及显式 epoch 配对清理，再由最新相关回归覆盖：101 项（89 通过、12 跳过）。
新增测试覆盖审计失败、审计期间停止、无新输入时读取故障、worker 挂起超时、过期完成、
旧结果跨 rearm、双手突发输入、idle 心跳及双侧 epoch 不一致；本轮没有现场设备验收。

后续现场试用准备：求解与 reset 在 state mutex 外执行，独立 tick mutex 保证有状态
pipeline 串行；phase/epoch 变化后清空排队结果，并用 session revision 丢弃正在计算的
旧结果。阻塞求解器的并发测试确认会话和输入仍能进入。首次会话序号 `0` 按现有
coordinator 合同接收，重复 `0` 仍拒绝；原生协议与 Python 客户端同步修复。
应用状态新增合并、待关联和退出未完成的计数，关联队列溢出明确失败。
这些计数不能替代每次优化器内部消费的完整追踪；暂不据此声明异步录制可逐周期数值回放。
新增离线组合联调实际运行原生 Hand2、SPARK、执行检查、协调器数值模块、MuJoCo 接口
和 HDF5，确认双臂及双手命令、录制和输入重建；现场试用入口见 README。

试用前验证：启用 `SPARK_NATIVE_TEST=1 WUJI_REFERENCE_TEST=1` 的相关回归 74 项全部通过，
包含真实原生进程与合成输入 MuJoCo/HDF5 联调；另有 82 项应用/启动回归（79 通过、3 跳过）。
原生 hand/control/MuJoCo 模块构建成功，并通过 C++ ASan/UBSan 检查。首次扩展回归中旧
Python 单左手测试曾触发 200 ms 输入过期，独立重跑及随后完整相关回归未复现，未调整门限。
可选 PICO 数值对照测试补上生产路径已有的几何缩放，生产映射未改。真实设备性能仍待验收。

2026-09-14 C++ 主调度现场启动故障修复：`scheduler=cpp` 接受 start 后，200 Hz IK 在
没有新 TJVR 包的周期发送默认 `WorkerTick`（空 packet、接收时间/来源元数据为 0）。
Python TJSO 解码器原先错误地要求所有 request 的接收时间为正，导致 side-effect reader
退出。现在区分空输入周期和携带数据包的周期：前者来源字段必须全 0，后者保持正接收时间
且不得晚于周期时间；request id 和周期时间仍必须为正。新增 C++ 编码器到 Python 解码器
的“有包→空周期→有包”回归。本修复仅涉及消息适配，不修改 IK 或输入新鲜度门限。

冷启动 launcher 只在启动期复用锁定 Hand2 Python 环境解析 YAML、Pinocchio frame/joint
顺序和数值参数，生成受权限保护的 manifest 后立即 `exec` C++；运行期不再走 Python
retarget callback。PICO2 保留左右独立 scheduler、真实 connection generation 和缺失侧
语义；VR/Manus 保留 right-then-left 的 126 点 callback。显式 rearm 后的 epoch 在下一次
session heartbeat 提交，`returning → idle` 自动推进一次，避免旧滤波状态跨会话复用。

构建与启用：

```bash
pixi run build-native-hand-scheduler
# 在 pico2_hands_sim、pico_vr_manus_sim 或 vr_manus_sim 命令中追加：
--hand-scheduler-backend cpp
```

默认仍为 `python`；`cpp` 不能与 `--hand-worker-backend cpp` 同时使用。两种实现的
backend、协议 ABI、scheduler/optimizer/library/source SHA256 会进入 resolved/HDF5
provenance；缺少产物时启动前失败，不静默退回 Python。旧 Python worker、PICO/Manus SDK
驱动、原始包解析、`HandRetargetLoop`、授权发布和手部 MuJoCo executor 均保持可用，未被
本阶段替换，因此这不是整个 live session 或完整 HDF5 热路径已 C++ 化的声明。

本阶段新增验证覆盖固定 wire 尺寸、双侧实际 C++ process、PICO2 producer 和
VR/Manus producer 的 canonical 命令、generation/epoch 重置、return-to-idle 单次推进、
stale teleop fail-closed、启动产物检查、PICO2/VR 入口参数传递和默认 Python 隔离；构建通过，
C++ 核心测试通过，相关 Python 回归 88 项（跳过 14 项）通过，全量回归 1182 项（跳过
65 项）通过。未连接现场设备、未启动真实 PICO/Manus、未提交或 push。

## 用户确认的迁移范围

保留全部既有硬件驱动、SDK 及连接接口，包括 PICO、Manus、机械臂和灵巧手。
不为 C++ 化改变 APK、USB 权限、设备绑定或真机驱动；硬件原始输出契约保持不变。
继续迁移驱动输出之后的输入解析、手部 retarget/滤波、现场控制调度接线与录制热路径。
配置/启动管理、依赖检查和离线分析可保留 Python/Shell。

上述四项均以**接入可选现场入口并完成回归**为交付条件，不以原生库编译或单独离线
夹具通过代替。两条原入口保留作对照；真实输入 MuJoCo 验收不等于真机运动授权。
具体剩余任务见 [原生会话实施方案](superpowers/plans/2026-09-14-native-session-runtime.md)
中的 Updated scope，各项尚未完成时保持未勾选。

## 后续迁移顺序及验收边界

1. **C++ 控制引擎**：输入快照、单调时钟调度、目标映射/IK、逐周期检查、执行与反馈
   在同一 C++ owner 内运行。Python 只发送显式操作请求、接收完成回执。
   状态机需要从现有测试提取事件序列，逐一对照 start、Home、rearm、故障和丢帧。
   C++ 命令权必须成为同一 coordinator 的独立实现，不能绕过现有授权发布者。
   事件/调度契约、状态机、固定身份健康输入、原始帧门控、配对命令及 IK/MuJoCo
   已组成支持 Home rearm/高度标定的离线会话；下一步接完整输入与输出消费者，再接现场入口；
   不仅用一个 C++ 定时器反复调用 Python `core.step`，否则 GIL 和协议转换仍在热路径。
   真正 native owner 需要在同一执行域内串联已迁移的 IK、guard、MuJoCo 和有界快照出口。
2. **C++ 仿真执行与快照**：关节设置/FK/反馈在控制引擎内完成，viewer 用独立状态快照。
   保留精确 Home 反馈与双臂原子执行，再消除重复 FK、dict/deepcopy 和往返通信。
3. **输入与手部**：为 TJVR 和 PICO2 分别实现适配，Manus 25 点/统一骨架/Hand2 retarget
   与滤波按原始数据逐帧对照。逐条启用，不能因控制层统一而改变坐标系/TCP或骨架定义。
4. **诊断与录制**：控制线程只写有界数据队列，C++ recorder/独立输出线程消费；
   队列满、写入失败和退出排空维持原有可观察语义，不静默丢弃控制或录制数据。

各阶段都保留原入口作对照，通过同输入一致性、异常状态序列及长时间离线负载测试后，
再用真实输入驱动 MuJoCo 验收。真机控制不在本次授权范围，默认切换也不由离线通过推断。

## 2026-09-14：原生下游发布首批实现

新增 `--publication-backend cpp`，严格要求 `--scheduler-backend cpp`（仍为双臂、关闭手部）。
`session_publication.hpp` 从原生周期快照编码 11 个常规主题及可选 bilateral receipt；
`session_zenoh_publication.hpp` 使用仓库 Zenoh C/C++ 依赖，连接显式 endpoint 并核对 router。
发布在现有 C++ 有界输出桥的消费线程中执行，异常经桥传回状态机进入 fault。
默认仍为 Python；选择 cpp 后 Python 保留录制/显示快照但禁止重复 put。
启动握手明确确认 `publication=cpp`，避免旧可执行文件忽略参数后两个实现都不发布。

首批实现中发布与 Python 快照发送共用原生输出桥；后续已拆成独立有界消费者，见下节。
录制的数据整理、viewer/keyboard 和手部主调度接线尚未消除 Python；完整迁移见
[下游实施方案](superpowers/plans/2026-09-14-native-downstream-runtime.md)。

局部性能复现（不启动 router、SDK 或控制会话）：

```bash
pixi run python scripts/benchmark_native_publication.py \
  --recording recordings/device_acceptance/native_cpp_vr_arms_20260914_200434_144958423.h5 \
  --offset 5000 --cycles 1000
```

录制 SHA256：`5eda22f45bf9e147ca6cca2e3128343fd296a658b03ddc7b186234f4100d6ffc`。
第 5000 起的 1000 个 cycle：逐字段差异 0；首轮消息编码及逐主题 JSON 序列化
均值 Python 142.803 μs、C++ 52.433 μs，约 2.72 倍；p95 分别 149.828 / 54.018 μs。
两端使用相同录制命令、反馈、时间和状态；缺失的 producer 元数据以同一回放标识填入。
这是消费者微基准，排除启动、输入解析、网络、磁盘和渲染，不能宣称整链路提速 2.72 倍。
Python 快照适配仍服务录制/显示，尚未测量在线净 CPU 收益。

本轮验证：严格 C++ 编译通过；gateway/适配器/状态机/发布/VR CLI/内嵌启动器/
PICO2 CLI 共 52 项，51 通过、1 项需启动隔离 router 的完整进程测试未启用；
包含原生输出回调故障、逐周期发布计数、重复发布抑制、旧二进制握手拒绝。
未连接现场设备、未进行新发布后端的实际 Zenoh 传输/真实输入验收，未提交或 push。

迁移时发现的既有语义：Python `_latched_dict` 的 `return_complete` 总为 false。
本次编码器刻意保持其发布值，以隔离迁移与行为变化；原生状态机 Home/退出完成状态及
gateway 完成帧是另外的通道。该发布语义需独立测试和修复，不能据此认定代码没有剩余问题。

## 2026-09-14：独立输出队列与 raw 捕获修复

- `OutputFanout` 将不可变快照分发给原生发布和 Python 通道各自的有界 FIFO，
  不在控制线程或分发线程执行 sink I/O。慢消费者不会直接阻塞另一条消费线程；
  若任一路积压到上限，则故障退出，不静默丢帧或无限积压。
- 正常退出先排空所有消费者，再发送 gateway 完成帧。最后一个输出失败也不得算成功。
  `NativeSessionGateway::output_complete()` 与 Home 完成分开，主程序退出码同时检查二者。
  取消时先通知所有 sink，再等待线程退出。
- 修复 `TjvrInput` 成功接受分支没有保存 `decoded` 与 `ingress_sequence` 的问题。
  该缺陷导致成功包不进入 raw 捕获，IK request 的接收序号为 0；修复只补传元数据，
  不改变坐标系、骨架、门控决策、IK 算法或任何硬件驱动。
- 新增真实解码器的成功/重复/坏包测试；真实 SPARK、mapped-palm worker 的 rearm 测试
  同时核对 raw 包字节、连续接收序号及接受/拒绝状态。ASan/UBSan 下原始输入测试通过，
  输出队列慢消费者/溢出/取消/最后一项失败测试重复 10 次通过。

历史记录注意：此前受影响的 C++ 录制可能只保存少量被拒绝的原始包，即使文件
`complete=true` 也不能证明 raw 流完整。现有文件不覆盖、不补造丢失数据；需要新录制验证
完整原始输入链路。之前同 HDF5 的消息编码 A/B 仍只证明记录下来的命令/反馈编码一致。

剩余：原生录制接线、原生 viewer/键盘、完整手部主调度接线和端到端同数据性能验证。
本节不表示全链路 C++ 完成。PICO、Manus、机械臂、灵巧手驱动及全部 SDK 保持不变。

本轮最终回归命令（2026-09-14）：

```bash
PYTHONPATH=src/tianji_teleop:. pixi run python -m unittest \
  tests.test_output_fanout tests.test_native_raw_input \
  tests.test_native_recording_transport tests.test_native_session_gateway \
  tests.test_native_session_gateway_adapter tests.test_native_session_runtime \
  tests.test_native_session_publication tests.test_vr_manus_live_cli \
  tests.test_embedded_pico_launcher tests.test_pico_hand_live_cli \
  tests.test_native_binary_results -q
```

68 项：67 通过，1 项隔离 router 完整进程测试未启用。网关夹具重复正常、运行中故障、
末帧故障三种退出情形各 5 次；测试等待原始包实际送达后再请求 start，并在读取 EOF 后
关闭接收端，消除两个测试自身的跨线程时序假设。原生 gateway 重新构建通过，
`git diff --check` 和 Shell 语法检查通过；无设备操作、无提交/push。

## 2026-09-14：完整机械臂录制数据块对照

新增 `raw_tjvr_hdf5.hpp` 和 `session_cycle_hdf5.hpp`，供后续原生录制消费者使用：

- 原始 TJVR 保留原字节、接收序号、接收时间与接受/拒绝审计，不以映射骨架替代 raw。
- 周期数据块保留左右命令、反馈、session 状态、完整 IK 结果、原始输入关联、
  coordinator receipt、命令身份和来源状态；命令字段复用原生发布编码契约。
- 测试通过真实 C++ HDF5 写入进程生成临时文件，与 Python `SessionH5Writer`
  对照层级、属性、dtype 和内容；审计 JSON 按字段比较，不要求键的序列化顺序相同。
- 两种 IK 均覆盖 idle/teleop/returning/fault 和未采纳结果；跨队列可能出现负相对时间戳，
  保留该值以兼容现有时间线，不钳位、不重采样。非法包、非法身份和越界序号拒绝写入，
  已写入的前序数据保留，文件不标记完整。

当前边界：这是数据块编码和磁盘写入验证，**尚未接入现场 gateway 录制消费者**。
不增加新的现场启动参数，不改变 Python 默认录制路径，不涉及任何驱动。
仍需补齐操作/生命周期审计、启动时 fd 移交和后端协商、统一时钟原点与最终完成确认。

本轮回归：下列 51 项中 50 通过，1 项 `SPARK_NATIVE_TEST` 可选完整进程测试未启用。

```bash
PYTHONPATH=src/tianji_teleop:. pixi run python -m unittest \
  tests.test_raw_tjvr_hdf5 tests.test_session_cycle_hdf5 \
  tests.test_native_recording_transport tests.test_worker_result_json \
  tests.test_native_session_publication tests.test_native_session_gateway_adapter \
  tests.test_vr_manus_live_cli tests.test_pico_hand_live_cli -q
```

这些验证不代表现场验收或端到端性能收益；完整 C++ 链路仍未收尾。未操作真实设备、未提交/push。

## 2026-09-14：录制消费者与网关独立队列

- `SessionRecordingSink` 将 raw、完整 cycle 和 operator reply 写入现有 C++ HDF5
  磁盘进程，保留统一文件时间原点；opening 与关闭生命周期可追踪。
- `SessionReply` 内部增加动作名称和处理时间，覆盖普通回复与异步 reset 回复。
  既有 gateway wire 格式保持不变；Python 入口不必跟随升级协议。
- `NativeSessionGateway` 新增可选录制 hooks，以独立有界队列消费输出。
  所有队列排空后才关闭录制，录制写入/关闭异常阻止发送成功完成帧。
- 编码失败会使 sink 永久进入失败状态，后续 `finish(true)` 不能伪造成功；
  取消、未关闭即析构、主动 incomplete 退出均保留未完成文件。
- 修复新 cycle 编码器对“没有新输入的 IK tick”误报坏包的问题：
  保留完整 IK 结果，将 `native_attempt` 留空；与 Python 对照通过。

验证：57 项回归中 56 通过、1 项可选完整进程测试未启用；原生 gateway 重建通过。
网关测试正常、发布失败、末帧失败、录制写入失败、录制关闭失败各重复 5 次。

**入口状态仍需区分**：消费者和网关队列已有实现，现场主程序/启动器尚未创建并连接
该 sink，仍缺文件描述符移交、后端能力协商及关闭 Python 重复录制。当前命令不变，
不能据此宣称录制在线热路径已经全 C++。未改驱动、未操作设备、未提交/push。

## 2026-09-14：可选原生录制入口接通（更新上述接线状态）

`--recording-adapter cpp` 现已连接实际网关主程序；必须同时选择
`--scheduler-backend cpp --disable-hands` 并提供新的 `--record` 路径。
默认适配器为 python，两种选择均可使用 C++ 磁盘写入器，但只有新选项把逐帧
raw/cycle/operator 编码与录制消费留在 C++。新选项不改变任何驱动或算法。

`NativeRecordingOwner` 仅做独占 schema 创建、文件描述符移交、子进程回收与最终状态核验。
网关必须返回 `recording=cpp` 能力确认；旧二进制不能静默禁用 Python 录制而未启动原生录制。
父进程移交后关闭自己的 socket 副本，避免引用残留阻止 EOF；完整状态须同时有磁盘成功
退出和文件 complete 标记，外层失败可撤销该标记，不能反向补造成功。

新增实际 SPARK、mapped-palm 网关子进程测试：仅使用 localhost 临时端口和合成 TJVR，
无设备驱动、无 router；核对原始字节、周期、操作审计和 Home 退出后的 HDF5 完整标记。
另覆盖独占文件拒绝覆盖、fd 移交、遗弃退出、外层否决完整状态和旧二进制握手拒绝。
构建原生网关通过。录制消费者/周期/raw/运行时等 17 项回归通过；两种实际网关测试
及独占文件测试共 3 项通过。真实输入长时录制与端到端性能对照仍待验收，不能将
这一步等同于整个 C++ 迁移完成。启动方式见 README 的“原生录制接线”实验选项。

## 2026-09-14：原生显示基础层（尚非现场 viewer）

`native/control/render_model.hpp` 新增独立 MuJoCo 渲染 model/data 和场景生成接口，
不暴露执行器数据、不输出机器人命令。两种资产模型的双臂 FK、mocap 目标位置/
四元数与 Python 对照；目标超过 200 ms、来自未来时间或不处于激活状态时，回退
到渲染模型自己的 TCP。此处 200 ms 与既有 mapped-palm 显示规则一致，不是控制门限。

完整更新先验证再写入：非有限关节、非单位目标四元数不会改写上一帧；所有 MuJoCo
访问限于模型创建线程。场景数据由显示侧持有，生成场景不更改关节状态。
相关渲染/overlay/独立 MuJoCo 回归 17 项通过；无现场窗口或设备操作。

该阶段剩余的 GLFW 接线见下一节；实际画面验收仍未完成。

### 原生窗口接线与 Python 无消费者快照裁剪（2026-09-14）

新增 `--viewer-backend cpp`，仅限原生双臂调度器。GLFW/OpenGL 留在网关主线程，
独立 model/data，输出消费者只更新有锁的最新缓存，渲染在解锁后执行。
键盘和窗口关闭提交状态机意图，不直接写关节；高位操作 ID 避免与 Python 终端操作冲突。
重复/旧周期、非法四元数与高度偏移不能替换有效显示目标。驱动与 Python 默认入口不变。

当 publication/viewer 均为 cpp，且 Python 不承担录制/overlay 时，不再构造完整 Python
LiveCycleSnapshot，只更新状态、epoch 和计数供终端及退出管理使用。协议帧接收/解码仍在
Python，不能称为零 Python 热路径或端到端迁移完成。

同数据复现：

```bash
pixi run python scripts/benchmark_native_consumer.py \
  --recording recordings/device_acceptance/native_cpp_vr_arms_20260914_200434_144958423.h5 \
  --offset 5000 --cycles 1000 --repeats 3
```

SHA256：`5eda22f45bf9e147ca6cca2e3128343fd296a658b03ddc7b186234f4100d6ffc`。
预热一轮、交替三轮，3000 次测量：完整快照 mean/p95/p99 为
73.88/77.31/81.64 µs，摘要为 0.91/1.17/1.37 µs；状态与计数逐帧差异为零。
这是预解码后的 Python 消费开销，不包含网络、IK、HDF5、渲染或整链路调度延迟。
实际窗口及真实输入组合尚未验收；完整手部主调度仍是后续工作，不能解除 arms-only 限制。

本轮验证：原生网关重新编译成功；入口/overlay/渲染/网关/录制与输出队列组合回归
69 项中 68 项通过，1 项 `SPARK_NATIVE_TEST` 现场全进程测试未启用。
额外核对实际网关 GLFW 启动失败时 HDF5 保持 incomplete；启动器测试使用假驱动，
没有操作现场设备。`git diff --check` 和两个会话脚本 `bash -n` 通过。

### 原生诊断摘要协议（2026-09-14，后续收尾）

在 publication/viewer 均为 cpp，且无 Python 录制消费者时，自动使用
`diagnostic_transport=summary`，启动必须返回 `diagnostics=summary` 能力确认。
新增可选 kind=7 定长计数加有界原因文本，不改既有 kind=1..6 的布局。
Python 不再解码原始包和完整 worker 结果，稳态摘要周期 100 ms；状态、epoch、
reset pending 或捕获故障变化立即发。退出前刷新最终周期计数，最终退出状态取完成回执。
所有命令权限和故障门控仍在原状态机中；操作回复不节流。

这个裁剪只作用于 Python 输出分支。C++ 发布、显示缓存和 HDF5 分支继续收到原始
完整快照；测试核对原生录制/发布周期数量相同、四个原始包均到达录制消费者，
Python 摘要不含 raw/receipt/cycle。摘要与完整协议交叉发送会明确失败，不能静默回退。
故障发生在最后一个周期之后时，Python 退出报告采用完成回执的最终状态，避免显示旧的
`teleop` 状态。补充回归覆盖此边界、畸形摘要、末帧刷新、原生队列完整性与能力握手。
`--scheduler-backend cpp` 仍为 arms-only；Manus 完整主调度尚未接入。上述同数据
73.88→0.91 µs 结果只属于前一节的 Python 消费裁剪，不用于量化新摘要通道的端到端收益。

该阶段组合回归 74 项中 73 项通过，1 项现场全进程测试未启用；没有操作设备或启动 router。

### 可选 Manus C++ 解析接线（2026-09-14）

`--manus-parser-backend cpp` 将 rawviz 行解析、语义 21 点转换和双手回调组装接到
`ReferenceManusProcess` 的原接收队列；默认 python。原驱动进程、SDK、设备接口、
主机接收时钟、原始行及回调录制入口不变；退出先停止接收线程再释放原生解析状态。
启动前加载检查，HDF5 来源包含后端和解析库/源码哈希。PICO2 入口不接收该选项。

测试核对错序节点、右前左后、单侧、固定手套绑定、重复序号、缺点失效与恢复、
非有限坐标、溢出、数字拼写和 64 位 SDK 发布时间。资源上限为 65536 字节/行、
64 个手套流、每流 512 条节点元数据；超界明确失败。此阶段保留 Python 的收包线程、
回调对象与应用状态/发布接线，不声称完整主调度器已接入手部。

离线前缀对照：

```bash
pixi run python scripts/benchmark_native_manus_parser.py \
  --recording recordings/device_acceptance/pico_vr_manus_20260913_015227.h5 \
  --lines 2000 --allow-incomplete-prefix
```

文件 SHA256：`fdeb844ff85e3b2c4bd08befed62b052251e8e522160c4e104e4f5054bc3788f`。
该文件未完整关闭，明确只取连续原始行前缀、双侧自动识别做两种解析器对照，不作为
完整会话/硬件验收。2000 行产生 1774 个回调，骨架值、左右源序号及源时间零差异。
预热一轮、逐行交替三轮的 6000 次测量，Python mean/p95/p99 为
34.06/37.98/40.59 µs，C++（含 ctypes 和回调组装）为 21.05/22.74/23.76 µs。
最大值分别为 222.74/3680.97 µs，未证明最坏延迟改善；这些结果不包含 retarget、
线程调度全链路、发布、仿真或录制性能，不能作为实时性资格。

软件验证：原生解析库构建通过；与实际 C++ Hand2 调度进程组合产生双侧规范 20 关节
结果，未连接设备。解析、原生手部调度、VR/PICO2 入口、网关协议组合回归 96 项中
95 项通过，1 项现场全进程测试未启用；脚本语法和 diff 空白检查通过。
没有新增 `--viewer-backend` 选项；现有 Python viewer 和所有驱动未修改。

### 原生手部仿真执行端点（2026-09-14）

`MujocoEndpoint` 增加内部显式启用的手部模式，仍默认仅机械臂；没有新增现场 CLI，
`--scheduler-backend cpp` 仍要求 `--disable-hands`。这是主会话手部接线的执行接口，
不是完整手部 C++ 会话已完成的标记。

- 同一个模型/数据拥有者线程以固定数组更新双臂和可选左右手；20 关节顺序保持协议定义。
- 所有关节数和有限值检查在任何写入前完成，一批仅运行一次 `mj_forward`，不引入动力学步进。
- 缺少某侧手命令时保持该侧；原有 `apply(arms)` 调用也保持手部，不归零。
- 模型绑定沿用 Python 的标准名称优先、缺失时使用 `_finger_` 别名规则。当前模型中的
  食指、中指、无名指使用别名，小指和拇指使用标准名称；不改变对外协议名称。
- 原生命令门控仍需在调用之前完成：身份、epoch、源新鲜度、会话阶段、关节限位。
  这个仿真存储/FK 接口不提供运动授权，也不替代手部命令发布者。

离线验证覆盖两套 SPARK/mapped-palm 模型、整批拒绝、缺侧保持、跨线程拒绝、
高度参考不扰动当前姿态、混合固定数组与动态批次的全 qpos/FK 一致性，以及别名
优先级和重复/非法名称拒绝。分配测试在预热后运行 100 次机械臂＋双手 apply/feedback，
计数对象是 C++ `operator new`，不是 MuJoCo 内部 malloc、整体内存或现场实时性保证。

驱动、SDK、Python 默认入口和现有现场命令不变。完整手部主会话的输入接线、
门控、发布、显示及 HDF5 手部数据仍需继续集成，不能用执行端点通过代替设备验收。

本轮验证命令与结果：

```bash
pixi run build-native-session-gateway
PYTHONPATH=src/tianji_teleop:. pixi run python -m unittest \
  tests.test_native_hand_mujoco_endpoint tests.test_native_owned_mujoco \
  tests.test_native_runtime_simulation tests.test_native_session_gateway_adapter \
  tests.test_vr_manus_live_cli tests.test_pico_hand_live_cli -q
PYTHONPATH=src/tianji_teleop:. NATIVE_OWNED_MUJOCO_SANITIZERS=undefined \
  pixi run python -m unittest tests.test_native_owned_mujoco -q
```

网关构建通过；组合回归 52 项中 51 项通过、1 项可选全进程测试未启用；UBSan
7 项全部通过。未连接设备、启动真实会话或操作驱动，未提交或 push。

### 原生主会话手部门控与执行接线（2026-09-14）

新增 `SessionHandDomain`，并通过显式内部配置接到 `SessionRuntime` 的拥有者线程，
不改变 Python 默认路径或网关 CLI。当前接入的是单一双手输入流/retarget 发布者边界，
并非已经完成 PICO2 独立左右手输入的原生现场接线。

- 固定源身份、生产者身份、router 和 run；输入代次不能静默改变。
- 原始本地输入时间、序号与 retarget 结果严格关联；关联队列有界，溢出显式故障。
- 输入和结果不靠主循环轮询刷新年龄；没有双手新鲜处理结果时不能启动，遥操中过期故障保持。
- 结果需满足有限值、有效侧标志及配置关节限位，整批通过后才可执行。
- 开始/返回/重置边界清理待执行结果；旧输入即使在新阶段算完，也只丢弃，不执行、不误判为
  新生产者故障。重置后必须有新的输入及当前 epoch 的结果，旧缓存不能恢复 ready。
- 手部反馈来自同一 C++ MuJoCo 拥有者，不接受外部布尔值伪造手部 readiness。
- 按现有 `AuthorizedHandMujoco` 语义，故障保持实际手姿态，显式仿真返回时置配置零位，
  双臂/双手在同一个模型上执行。这里的瞬时归零不是物理设备轨迹。

离线测试包含 C++ 手部调度线程→原生主会话→两套 MuJoCo 模型的组合测试，以及
无手部数据拒绝启动、正常跟踪、Home、坏结果保持、超时保持。组合测试使用明确的
数值求解夹具，不宣称该测试验证了官方优化器数值一致性、设备表现或端到端性能。

尚未完成：现场驱动输出到此输入接口及 worker epoch/reset 握手的接线、手部完整发布/显示/HDF5、完整现场
验收。`--scheduler-backend cpp` 仍要求 `--disable-hands`，不能移除该保护来试完整手部。

验证：`pixi run build-native-session-gateway` 构建通过；以下组合回归 91 项中
90 项通过、1 项可选全进程测试未启用。未操作驱动/设备，未提交或 push。

```bash
PYTHONPATH=src/tianji_teleop:. pixi run python -m unittest \
  tests.test_native_session_hands tests.test_native_session_runtime \
  tests.test_native_session_commands tests.test_native_session_rearm \
  tests.test_native_session_health tests.test_native_runtime_simulation \
  tests.test_native_session_gateway_adapter tests.test_native_hand_mujoco_endpoint \
  tests.test_native_hand_scheduler tests.test_vr_manus_live_cli tests.test_pico_hand_live_cli \
  tests.test_gateway_summary tests.test_native_session_publication tests.test_session_cycle_hdf5 -q
```

额外以 `NATIVE_HAND_DOMAIN_UBSAN=1` 运行 `tests.test_native_session_hands`，
2 项通过；编译启用 `-fno-sanitize-recover=undefined`，检查失败不能被退出码 0 掩盖。

### 原生手部子进程与重置完成确认（2026-09-14）

新增 `native/hand/worker_client.hpp`，由 C++ 直接启动并通过有界二进制 IPC
调用已有原生 Hand2 调度器。冷启动仍可经过原有 Python launcher，不涉及硬件驱动。

- 新请求 `TJHR` 仅允许 idle 下的下一 epoch；双侧求解器 reset 都完成且会话未改变后，
  才生成关联请求序号、时间和 epoch 的 `TJHA`。客户端收到正确确认后才提交本地 epoch。
- 错误确认、截断、超时与取消不推进客户端 epoch；只清理该客户端自己启动的子进程。
  关闭子进程不等于 Home、发布排空或 HDF5 完整关闭确认。
- 原 `TJHS`/`TJHO` 布局不变，旧 Python 客户端不会收到新增确认帧。
- 修复调度采样时间与输入接收之间的竞态：晚于当前 tick 时间的输入留待下一周期，
  不输出倒置时间、不提前推进优化器/滤波器状态。idle/teleop 均有先失败再通过的测试。

真实原生 Hand2 子进程离线测试验证 reset 后相同输入的首帧关节结果完全一致；
故障子进程覆盖错误 epoch/序号/请求时间、未来或倒退完成时间、保留位、截断、超时和取消。
这些不等于真实设备验收，也未测出新的端到端性能收益。

本轮构建 `pixi run build-native-hand-scheduler` 成功（保留依赖 CMake CMP0167 开发警告）。
以下回归 95 项，92 项通过、3 项跳过：

```bash
PYTHONPATH=src/tianji_teleop:. pixi run python -m unittest \
  tests.test_native_hand_reset tests.test_native_hand_scheduler tests.test_native_hand_worker \
  tests.test_native_session_hands tests.test_native_session_runtime tests.test_native_session_rearm \
  tests.test_vr_manus_live_cli tests.test_pico_hand_live_cli tests.test_hand_retarget_loop \
  tests.test_pico_hand_component -q
```

跳过项为两项显式启用的 Hand2 参考对照和一项可选完整现场进程测试。
随后补跑 `WUJI_REFERENCE_TEST=1` 的 `tests.test_native_hand_worker -v`，5 项全部通过，
包含上述两项 Python/原生求解对照；另重跑 `tests.test_native_session_rearm -v`，2 项通过。
可选完整现场进程测试未启用。

该阶段尚缺该客户端与主会话联合 reset 事务、现场手部输入及完整发布/显示/HDF5 的接线；
其中联合 reset 的后续实现见下一节。
主调度器 `--scheduler-backend cpp` 仍只开放无手套机械臂测试；原 Python 默认路线不变。
本轮未操作驱动、设备或真实会话，未提交或 push。

### 主会话联合手部重置接线（2026-09-14，后续进展）

`SessionRuntime` 新增可选 `HandResetEndpoint`，原生实现使用已有 Hand2 子进程客户端。
该接口是内部接线，不改变现场 CLI 或默认 Python 路线。

- 有手部域时，缺少手部重置端点会拒绝 rearm，不允许仅靠机械臂确认推进会话。
- 重置前后保留健康、精确 Home、手部零位检查；在异步事务中依次验证机械臂与手部确认，
  两者均达到请求 epoch 才提交主会话。机械臂成功、手部失败时不尝试伪造回滚，而是锁存故障、
  保留旧会话 epoch，必须重启恢复。
- 停止、故障和丢失重置条件同时取消两侧事务。等待手部期间主调度器继续运行，不能开始遥操。
- 重置确认不充当输入 readiness；完成后仍需要新输入及当前 epoch 的结果。
- 机械臂专用路径没有手部端点，保持既有行为。所有驱动及 SDK 均未修改。

离线测试包含真实原生 Hand2 子进程分别与真实 SPARK、mapped-palm worker 的联合重置；
另用明确故障夹具测试单边失败、缺失端点、无响应取消及重置期间源健康丢失。
它们不是设备驱动，也不验证完整遥操数值轨迹。

`pixi run build-native-session-gateway` 构建成功。以下回归 78 项中 77 项通过、
1 项可选现场进程测试未启用（Hand2 Python/原生参考对照已启用）：

```bash
PYTHONPATH=src/tianji_teleop:. WUJI_REFERENCE_TEST=1 pixi run python -m unittest \
  tests.test_native_session_hands tests.test_native_hand_reset tests.test_native_hand_scheduler \
  tests.test_native_hand_worker tests.test_native_session_runtime tests.test_native_session_rearm \
  tests.test_native_height_calibration tests.test_native_scheduled_ik \
  tests.test_vr_manus_live_cli tests.test_pico_hand_live_cli -q
```

剩余主线：同一手部 worker 的现场输入/结果及阶段同步接线、完整手部发布/显示/HDF5、
端到端同录制对照和真实输入仿真验收。主调度器 C++ 路线仍要求 `--disable-hands`。
未连接设备、未启动现场会话，未提交或 push。

额外检查：`NATIVE_HAND_DOMAIN_UBSAN=1` 下运行 `tests.test_native_session_hands`，
3 项通过；补入源健康丢失场景后，再运行联合重置测试 1 项通过。
检查使用 `-fno-sanitize-recover=undefined`，不忽略未定义行为错误。`git diff --check` 通过。

### 同一原生手部 worker 的输入、结果与阶段接线（2026-09-15）

新增 `HandPipelineEndpoint` / `NativeHandPipeline`，通过已有可选手部端点配置接入
`SessionRuntime`。入口是**驱动解析后的规范化双手骨架** `HandSampleEnvelope`，
不是新的 SDK 或现场网络协议；目前仍是内部接口，没有开放新的现场 CLI 组合。

- 独立 C++ IPC 线程串行处理同一 Hand2 子进程的骨架输入、阶段切换、结果和 reset。
  主会话仅操作有界队列，不等待手部求解或 IPC 回复。
- 输入队列为有界 FIFO；这一步不宣称与参考现场 latest-input 采样时序完全一致。
  输入及结果队列溢出显式失败，不能靠无限积压隐藏负载问题。
- 主会话校验输入身份、时间、序号、代次，关联原生结果，并使用原有新鲜度、阶段、epoch、
  限位门控后写入其拥有的 MuJoCo 模型。选择此端点时，拒绝外部注入手部结果或仅输入回执，
  避免绕过本次会话的原生生产者。
- Home、重置、再次启动由同一 worker 完成；重置期间拒收样本并清理旧任务/结果，
  不用状态心跳冒充重置后的新输入。手部重置与求解不会同时操作同一个 IPC 客户端。
- 停止/故障取消阻塞 I/O；IPC 线程退出时立即关闭并回收自己拥有的子进程，
  不等端点对象析构，不把资源关闭当成 Home 或录制排空成功。

离线集成测试在 SPARK、mapped-palm 两套 MuJoCo 模型上使用真实原生 Hand2 worker，
覆盖规范化输入→求解→主会话→手部反馈→Home→rearm→新输入再次启动。
机械臂输入与 reset 在这组测试中是明确的夹具，不是机械臂 IK 数值一致性测试。
另覆盖代次变化、非有限输入、无响应、队列溢出、阻塞中重置取消和子进程及时回收。

修复子进程回收后，最终组合回归 80 项中 79 项通过、1 项可选现场进程测试未启用：

```bash
PYTHONPATH=src/tianji_teleop:. WUJI_REFERENCE_TEST=1 pixi run python -m unittest \
  tests.test_native_hand_pipeline tests.test_native_session_hands tests.test_native_hand_reset \
  tests.test_native_hand_scheduler tests.test_native_hand_worker tests.test_native_session_runtime \
  tests.test_native_session_rearm tests.test_native_height_calibration tests.test_native_scheduled_ik \
  tests.test_vr_manus_live_cli tests.test_pico_hand_live_cli -q
```

主网关重新构建通过。`NATIVE_HAND_DOMAIN_UBSAN=1` 下的手部管线及主会话手部测试
5 项通过；生命周期回收测试另以 `-fsanitize=undefined -fno-sanitize-recover=undefined`
单独编译运行通过。未发现这些测试覆盖范围内的未定义行为。
额外运行 `tests.test_native_session_gateway_adapter`、`tests.test_session_cycle_hdf5` 和
`tests.test_native_session_publication`，15 项全部通过，覆盖原有网关、机械臂周期录制和发布编码。

**仍待完成**：现场驱动输出接收、PICO2 独立左右手接线、完整手部发布/显示/HDF5 消费者、
冷启动 CLI 接线、同录制端到端数值及性能对照。没有改变原 Python 默认路线，
`--scheduler-backend cpp` 仍要求 `--disable-hands`。未操作驱动或真实设备，未提交或 push。

### 2026-09-15：原生现场适配器退出与监督修复

本轮仅修改 `native_live_runner.py` 的监督和收尾逻辑，不改变驱动、IK、协议或
Python 默认遥操入口：

- 运行循环检查 consumer、gateway、live authority 和发布线程错误；异常进入清理，
  关闭网关控制连接，触发原生侧既有控制通道断开故障门控，不继续接受操作。
- 正常退出先消费到完成帧，再排空异步发布，等待原生子进程退出，最后决定 HDF5
  是否完整。消费线程超时、发布尾部失败或非零退出不能算成功。
- 回 Home 使用原生成功完成帧及 idle 状态确认；外层发布或 authority 错误不再
  覆盖这项已收到的确认，但仍使整个会话失败、录制保持 incomplete。
- 关闭 Python Viewer 后等待完成期间保留循环等待，避免忙循环抢占 CPU。

新增 `tests.test_native_live_shutdown`，覆盖排空竞态、运行时监督故障及主循环清理、
尾部发布失败、原生完成确认和非零退出。相关离线组合回归共 66 项：65 项通过，
1 项跳过；未重新启动真实 PICO/Manus 会话，不代表现场退出验收已通过。

```bash
PYTHONPATH=src/tianji_teleop:. pixi run python -m unittest \
  tests.test_native_live_shutdown tests.test_native_session_gateway_adapter \
  tests.test_native_session_publication tests.test_gateway_summary \
  tests.test_vr_manus_live_cli tests.test_pico_hand_live_cli \
  tests.test_native_session_gateway tests.test_session_cycle_hdf5 -q
```

### 2026-09-15：回 Home 后自动 rearm，保留手动 r

原生网关显式启用 arms-only 自动 Home 重置；其他 SessionRuntime 调用者默认不启用，
Python 遥操入口不变。已接受的 `h/return` 在精确 Home、健康和新鲜度条件满足后调用
既有异步 reset 事务，等待真实 worker ACK、推进 epoch，并保留重置后新输入屏障。
内部完成报告为 `auto_rearm`（保留 reply id 0）；不发送 start。
手动 `r` 仍调用同一事务；重复重置在 pending 阶段拒绝，`q/shutdown` 不触发自动重置。

SPARK 和 mapped-palm 实际原生 worker + MuJoCo 离线测试覆盖连续两次
Home→自动重置→新输入→start，以及自动重置后的手动 r 和重新启动。
本轮组合测试 48 项中 47 项通过，1 项可选现场进程测试未启用：

```bash
PYTHONPATH=src/tianji_teleop:. pixi run python -m unittest \
  tests.test_native_scheduled_ik tests.test_native_session_rearm \
  tests.test_native_live_shutdown tests.test_vr_manus_live_cli tests.test_pico_hand_live_cli -q
```

`pixi run build-native-session-gateway` 成功，`git diff --check` 通过。
未启动设备会话，未提交代码；真实输入下的 h→s 和手动 r 仍待现场复测。
