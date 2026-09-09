# DexhandVelocityQpIk7 直出接入计划

目标：新增 pico_ee_dexhand_qp 后端；目标整形、IK 后 Ruckig、协调器增量裁剪独立可选。
按用户确认顺序执行，不使用子智能体、不提交代码、不操作正在运行的真实 PICO 会话或实体机器人。

## 接入边界

- 复制参考工程 DexhandVelocityQpIk7、速度 Ruckig 与 SO(3) 实现，保留来源说明。
- 移除对参考工程 MuJoCo 控制器的类型依赖；由本工程 Pinocchio 模型提供 Base→TCP FK 和 Jacobian。
- 注册 ArmIkSolver 工厂新后端，不能以旧后端别名代替移植。
- 新后端默认直出；现有后端缺省行为不变。
- 目标整形仍使用 passthrough/conditioned；关节处理使用 passthrough/ruckig；协调器裁剪独立布尔配置。
- 关闭平滑或裁剪不关闭有限值、硬限位、超时、身份与异常步长拒绝；回位轨迹不因遥操直出而取消。

## 执行与验收

- [x] 移植核心及最小类型，记录来源文件与哈希。
- [x] 新增原生后端探针：工厂注册、双臂 FK、位移和旋转收敛、限位与无效输入拒绝。
- [x] 实现本工程模型适配和左右臂独立求解状态。
- [x] 先新增裁剪关闭测试，再实现协调器开关，保留安全拒绝测试。
- [x] 新增 IK 后处理选择，直通分支不调用 Ruckig；旧默认仍使用 Ruckig。
- [x] 配置和启动入口贯通，非法选项启动失败，并公开当前实际模式。
- [x] 编译原生 IK，运行后端探针与 Python 回归。
- [x] 独立路由器合成 PICO 双臂会话验证直出和可选平滑，记录未验证范围。
- [x] 更新 README 的可复制启动命令。

## 验证结果

- 原生 build-ik-sim 成功；新核心探针、新后端工厂双臂探针、原有 QP 探针通过。
- 65 项相关 Python 回归通过；shell 语法与 git diff --check 通过。
- 新后端直出：`/tmp/pico-sim-smoke-asruqkhn/result.json`；976 条匹配的 final command 与 proposal 完全一致，454 帧 raw，断流回 idle。
- 新后端 + Ruckig + 裁剪：`/tmp/pico-sim-smoke-qbs01mlf/result.json`；456 帧 raw，断流回 idle。
- 旧 pinocchio_qp + conditioned + 后处理直通 + 关闭裁剪：`/tmp/pico-sim-smoke-gpem46p3/result.json` 通过。
- 实际模式通过 producer/coordinator 状态核验；source 状态也新增目标整形模式。
- 以上为本机合成输入仿真，不代表真实头显、实体机器人或原有 Regrind 移动轨迹探针已通过。
- 没有提交代码，没有重启或关闭用户自己的 PICO/仿真会话。
