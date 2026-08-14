# IK profiles

`controller_only_ik.yaml` 保留纯手柄输入、回零和公共安全参数；
本目录下每个后端的 `controller_only.yaml` 只保留后端选择与算法专用参数。
启动器根据 `ik_backend` 只加载对应的一个 profile。

```text
ik/
├── pinocchio_cpp/controller_only.yaml
├── pinocchio_qp/controller_only.yaml
└── tianji_official/controller_only.yaml
```
