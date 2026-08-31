# Arm IK backends

`arm_ik_producer` 是唯一的 arm IK producer，消费 canonical
`tianji/target/arm/{left,right}`，发布 accepted proposal 与 solved pose；
coordinator 才能把 proposal 变成 final command。三个 backend 为
`pinocchio_cpp`、`pinocchio_qp`、`tianji_official`，选择只写入
`src/tianji_teleop/config/producers/ik.yaml` 的 `ik_backend`。

所有 backend 共享 `ArmIkSolver::solve()`、`IkSettings` 和 `IkResult`，使用
`config/robot/arm.yaml` 中的 names、Home 和 rad limits。target 的 side/frame、
sequence、freshness 或几何非法时 producer 不调用 solver，也不发送 accepted=false
占位 proposal。solver result 必须 finite 且符合 `maximum_joint_step_rad`。

```bash
pixi run -e ik-build build-ik
TIANJI_IK_BACKEND=pinocchio_cpp pixi run mocap_live_sim
```

官方 SDK 只在独立 worker 与 producer 进程加载；Marvin executor 不加载官方 IK
库。运行时入口为 `arm_ik_producer`，不保留旧 executable 或配置别名。
