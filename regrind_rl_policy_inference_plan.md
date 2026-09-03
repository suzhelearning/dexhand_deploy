# Regrind RL Policy 推理与真机闭环方案（z-up）

## 方案摘要

采用“远端只推理、本机掌握控制权”的架构：

```text
Motive/Wuji 反馈
  -> 本机 policy_gateway
  -> 远端 model_5750
  -> 本机安全适配
  -> 右腕 IK + Wuji 右手
```

- 远端固定使用 `model_5750`（SHA256 `c2373a…e211`）、take001、50 Hz。
- 本机控制 `frame_index`、标定、同步、动作校验、Zenoh authority、watchdog、return/fault。
- 右腕 6D 目标复用现有 IK 转为 Tianji 右臂 7 关节；右手输出 20 关节；左臂保持 Home。
- 首个真机测试即右腕与右手完整闭环，但限制速度和加速度为 25%。
- 远端服务独立人工启动；SSH 密码不写入配置、环境变量或日志。

## 接口与实现

### 远端推理服务

- 在 IsaacLab Python 3.11 环境安装并锁定 `eclipse-zenoh>=1.10,<2`。
- 使用私有可靠 Zenoh key：
  - `tianji/inference/regrind/request`
  - `tianji/inference/regrind/result`
- Request 严格包含：
  - `schema_version/run_id/request_seq/frame_index`
  - Motive 右腕 pose、hammer pose
  - Wuji 实测 20 关节位置
  - calibration/config 版本
- Result 严格包含：
  - 对应的 `run_id/request_seq/frame_index`
  - `obs[123]`
  - actor mean `raw_action[26]`
  - checkpoint/config hash、推理耗时和服务序列
- 远端不得发布 `tianji/target/**`、`proposal/**`、`command/**` 或 session intent。
- 修正训练契约偏差：
  - history 顺序为 `[次新, 最新]`
  - 使用训练一致的 normalization
  - 腕旋转采用 `delta_quat ⊗ base_quat`
  - `phi=i/T`
  - 执行参考索引 `0..T-2`
  - stale/坏输入时不伪造状态、不推理、不发布结果
  - 不在真机推理中人工注入训练噪声或随机延迟

### 本机 policy_gateway

- 同时承担 `source` 与 `producer_hand`，但不替代 arm producer：
  - 发布右侧 canonical `ArmTargetCommand(Base_R→TCP)`
  - 发布右侧 canonical `HandJointCommand[20]`
  - 现有 IK producer 继续生成右臂 proposal，coordinator 保持唯一 arm final-command authority
- 本机以 50 Hz 发送明确 `frame_index`；一次超时冻结该 frame 并保持上一安全目标，连续 3 次或 60 ms 失败则停发并进入 bounded return。
- 使用本地接收时间判断 freshness，禁止跨机器比较 `monotonic_ns()`：
  - 输入年龄不超过 40 ms
  - 腕、hammer、手反馈互差不超过 20 ms
- 本机独立复算并校验动作：
  - `raw_action` 必须严格为 26 维、finite、范围 `[-1,1]`
  - 腕位置：`reference_pos + 0.02 * action[0:3]`
  - 腕旋转：`Exp(0.064 * action[3:6]) ⊗ reference_quat`
  - 手指：`reference_joint + 0.064 * action[6:26]`
  - 再执行 workspace、速度、关节硬限和 step 限制
- 腕与手使用独立 topic，不增加事务协议；但同一 frame 必须先整体校验，任何一侧失败均不发布。
- 所有 canonical wire timestamp 由本机重新生成；远端时间仅作诊断。
- 非 `teleop` 状态不得调用 actor 或发布 target/hand command。

### 标定、启动与安全

- `tianji_wrist` 必须固定在真实右腕；机器人 Home 时冻结完整 Motive pose，结合既有 rigid→mount→wrist→TCP 外参建立 Motive→`Base_R` 的 SE(3) 变换。
- 用多个机器人验证姿态检查外参，位置残差不超过 5 mm、旋转残差不超过 2°。
- Motive 物体名必须唯一且精确为 `hammer`，不依赖静态 ID 9。
- 每次运行前 hammer 与 take001 frame0 误差必须不超过 10 mm/5°。
- 双阶段人工授权：
  1. 确认模型/hash、标定、输入 freshness、机器人 Home、手 zero、急停与 executor ACK。
  2. 操作者单独执行 arm 和 start；任何重连、return 或 fault 均需重新授权。
- 普通断流/超时在反馈健康时 bounded return；schema、身份、错序、NaN、越界、tracking、硬限位或反馈异常触发锁存 fault/soft-stop。
- Wuji 补充保护：
  - 必须收到完整 20 关节、finite 且限内的反馈
  - 任一关节误差超过 0.25 rad 持续 100 ms，或瞬时超过 0.5 rad，锁存 soft-stop
  - SafetyStop 必须获得 Marvin 和 Wuji 两个 executor ACK
- 真机 profile 显式启动本地 safety supervisor；现场必须有操作者和独立安全员掌握物理急停。
- 当前宽松 ACL 仅允许离线、shadow 和 MuJoCo；真机前必须收窄防火墙与 Zenoh ACL，使远端只能读 request、写 result，不能写控制空间。

## 记录与验证

- 每次测试强制记录：
  - Motive 原始腕/hammer、Wuji 实测关节
  - 同步后的 policy 输入、`obs[123]`、`raw_action[26]`
  - reference、右腕 target、IK solved/proposal、arm/hand command 与反馈
  - source/producer/executor status、liveliness、SafetyStop/ACK
  - request/result 序号、关联关系、时延、发布时差
  - checkpoint、配置、代码、ACL、标定 hash
- 扩展 recorder 和 validation analyzer，增加 `policy_gateway` source、26D action 证据、手部 tracking、跨流关联及新的 sim/real validation case。
- 晋级顺序：
  1. 对全部 12 条轨迹做官方逐帧离线对拍：obs 最大绝对误差不超过 `1e-6`，actor action 不超过 `1e-5`。
  2. take001 完成连续 3 次 live shadow，不发布控制 target。
  3. take001 在 MuJoCo 完成 10 次，并通过断链、延迟、坏包、错序、NaN、越界、重复 authority、急停和恢复测试。
  4. take001 通过 IK 可达性、碰撞和限位预检后，才允许真机 25% 速度测试；预检失败时保持阻断，不自动换轨迹。
  5. 真机人工复位后运行 10 次。
- 性能门槛：request→result p99 小于 15 ms，单帧 deadline 为 20 ms。
- 最终验收：
  - 至少 6/10 次同时满足：hammer 抬升超过 50 mm，最终位置误差小于 70 mm。
  - 全部试验零越界、零未授权运动、零未处理安全异常。
  - 每次结束必须完成双臂 bounded Home、右手 zero 和 fresh feedback 确认。

## 已确认假设

- 训练、远端 URDF 和本仓库 Wuji 右手 20 关节顺序已逐项一致，单位为 rad。
- take001 是唯一真机验收轨迹，禁止 `--traj all` 和模糊名称回退。
- 部署 actor 使用确定性的均值动作，无 RNN 隐藏状态；每个 run 重新初始化 history 和 last_action。
- 当前远端部署目录不是 Git 仓库，因此以代码、模型、配置和资产 SHA256 清单作为可复现版本边界。
