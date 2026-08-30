# Tianji Teleop

这是一个由 PICO controller、`mocap/aligned/hands` live 人手或 acquisition v4 H5
回放产生 canonical target 的遥操作系统。IK producer、唯一 command coordinator、
MuJoCo/Marvin/Wuji executor 均通过 versioned Zenoh protocol 通信。

## 唯一入口

先由操作者启动外部 router，再共享同一个 endpoint：

```bash
export TIANJI_ROUTER_ENDPOINT=tcp/127.0.0.1:7447
pixi run doctor
pixi run pico_sim
pixi run mocap_live_sim
pixi run h5_sim -- --h5 TAKE.h5
pixi run target_replay_sim -- session.h5
pixi run joint_replay_sim -- session.h5
```

`h5_sim` 默认打开 MuJoCo viewer；自动化或无显示环境必须显式追加
`--headless`：

```bash
pixi run h5_sim -- --h5 TAKE.h5 --headless
```

真实设备必须明确确认，且由 profile 的 real capability、HostReadiness 和设备
preflight 共同放行：

```bash
pixi run pico_real -- --confirm-real
pixi run mocap_live_real -- --confirm-real
pixi run h5_real -- --confirm-real --h5 TAKE.h5
```

`run_session.sh` 在 spawn 前为每个 component 分配 UUID，并注入
`TIANJI_COMPONENT_INSTANCE_ID`、`TIANJI_COORDINATOR_INSTANCE_ID` 和实际
`TIANJI_ROUTER_ZID`。组件不会自行生成身份；同 logical id 的多 instance 会被
拒绝。session profile 不保存 router endpoint 或 IK backend。

## 配置

唯一配置树位于 `src/pico_body_tianji/config/`：

- `robot/`：双臂和 Wuji Hand 2 的 names、Home、rad limits、zero；
- `sources/`、`producers/`、`executors/`：组件参数；
- `coordinator/arm.yaml`：状态机和回 Home 参数；
- `recording/`、`replay/`、`diagnostics/`：session v1、回放、标定；
- `sessions/`：只组合 component config、capability、active/inactive sides 和 hand mode。

关节 wire 单位固定为 rad；Marvin 只在 SDK 边界转换为 degree。外部
acquisition H5 v4 与 session HDF5 v1 是两个不同 schema。

## 安全边界

MuJoCo、Marvin 和 Wuji 都只消费 coordinator final command 或受授权 hand
command publisher。diagnostics 仅订阅权威 state/status 或发送 intent，不发布
第二份 state/final command。普通 return 是 bounded Home；feedback、router、
identity、hard limit 或 safety stop 异常进入 fault，不能由 return 清除。

IK 构建与 runtime 部署：

```bash
pixi run -e ik-build build-ik
pixi run -e ik-build deploy-ik
```

安装产物只包含 `arm_ik_producer`、canonical source/producer/executor、recorder、
replay、policy 和 diagnostics 入口；过时入口不会保留 alias。
