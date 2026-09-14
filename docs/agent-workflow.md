# 按任务读取的工作参考

授权和长期约束集中在 [AGENTS.md](../AGENTS.md)。本页只做导航，启动指令不另抄一份。

## 操作与状态

- 日常操作先读 [README](../README.md) 中对应路线：PICO2 使用手追踪 APK/TCP 10002；
  VR 手柄使用 wholebody APK、内嵌 driver/M0/TJVR/TCP 9999。两者不能混用。
- 验收范围及证据缺口见 [真实输入仿真验收单](real-input-simulation-acceptance.md)。
  用户的人工试运行反馈与完整录制、长时间稳定性验收分别记录。
- 生效参数以本次 CLI 覆盖和会话配置解析结果为准；README 的推荐覆盖参数不一定等于
  YAML 默认值。例如 PICO2 推荐 `head_palm_direct`，preset 默认仍为 `relative_home`。

## 修改或核对算法

- SPARK：[移植报告](spark-porting.md) 与 [参考契约](reference_equivalence_contract.md)。
- mapped-palm：[移植报告](mapped-palm-porting.md) 与
  [设计](superpowers/specs/2026-09-13-mapped-palm-manus-design.md)。保持独立 C++ 核心及周期适配；
  Python 会话接线不等于 Python 重写算法。该选项的离线对照不能证明现场实时效果一致。
- 手部：[官方手部移植报告](wuji-hand-porting.md)。区分源骨架、语义转换、retarget 和命令滤波。
- 共享控制契约：[架构](ARCHITECTURE.md)；录制回放：[格式说明](mocap_h5_v40_format.md)
  涉及 acquisition v4，不能直接当作 session HDF5 的 schema。

## 历史方案与技能

`docs/superpowers/plans/`、`specs/` 及进度记录保存相应日期的设计、执行步骤与证据。
只读取当前任务相关部分；早期“待实现”“未提交”、临时执行要求和测试计数不自动代表当前状态。
恢复旧方案前核对实际代码、Git 状态及后续记录；遇到冲突保留证据并注明适用版本，
不删除仍有效的算法、协议或授权约束。

本次审计前项目没有 `SKILL.md` 或技能 `references/`。项目约定放在根入口，
不再复制为同义技能。外部安装的技能不属于项目维护范围；应用时遵循根入口的授权约定，
不因其模板要求自动提交、启用子智能体或更改全局配置。
