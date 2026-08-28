# External acquisition format notes

acquisition v4 H5 是外部输入格式，teleop session v1 是本地记录格式；两者不得
混写。转换时使用 `sources.mocap.h5.load_mocap_h5()`，保留 source timestamp，
进入 teleop wire 后由接收主机生成 monotonic timestamp。

检查重点：

- 单侧 `valid` 决定 wrist/keypoints 是否可用；无效侧不得使用旧值；
- quaternion 必须 finite，keypoints 发布前减去 0 号 wrist；
- 可选 `wuji2_joints` 仅在 H5 profile preflight 选择 direct path；
- robot marker 不构成 live target，仅用于 H5 frame0/diagnostic overlay。

```bash
pixi run h5_sim -- --h5 TAKE.h5 --validate-only
```
