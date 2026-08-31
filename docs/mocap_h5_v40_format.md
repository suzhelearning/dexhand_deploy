# acquisition v4 H5 format

本仓库将外部 acquisition v4 H5 作为只读输入，不在文件内追加 session 数据。
`tianji_teleop.sources.mocap.h5.load_mocap_h5()` 校验 root schema、时间轴、
单侧 valid mask、wrist quaternion、keypoints 和可选 `wuji2_joints`。

H5 source 在 session preflight 固定 hand path：存在 `wuji2_joints` 使用
`direct`，否则使用 `retarget`。invalid side 的字段保持 null，不能前向填充。
frame0 marker 仅用于 H5 安装/坐标诊断；live target 只订阅
`mocap/aligned/hands`。

```bash
pixi run h5_sim -- --h5 TAKE.h5
```

该输入格式与 session HDF5 v1 不同；session recorder 只记录 typed raw、target、
proposal、command、state 和事件流。
