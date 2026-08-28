# Future plan

当前产品面向可审计的 canonical target/session contract。后续优化必须以
`validation-run` 产出的 session HDF5、status、liveliness 和 operator events 为
证据，不通过放宽 limits、freshness 或 fault 来“通过”。

## 可扩展点

- 增加 source：实现 typed raw/target、source status 和 intent 生命周期，声明
  唯一 source liveliness token；不发布 SessionState。
- 增加 producer：实现 ArmJointProposal/HandJointCommand contract，拒绝
  nonfinite/shape 输入；不发布 final command。
- 增加 executor：复用 robot/hand config 的 names、Home、limits、zero 和安全停机，
  接受 launcher 授权 identity 后再置 ready。
- 增加 diagnostics：仅订阅权威 state/status 或发送 intent，结果写在 session 外部
  的分析文件，不创建第二 authority。

外部 acquisition v4 H5 继续只读；进入 teleop 后，live 输入来自
`mocap/aligned/hands`。robot marker 只用于 H5 frame0/安装诊断。

所有真实设备试验必须按 10% 速度/加速度开始，急停、tracking error、feedback
stale、router ZID 变化、重复 authority 和 physical limit 都是立即停止条件。
