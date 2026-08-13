# 厂商运行时说明

打包脚本从当前本机提取下列真机运行时：

- XRoboToolkit Python 3.10 / Linux x86_64 扩展；
- `libPXREARobotSDK.so`；
- `pico_input` 的增量映射和 One-Euro Filter；
- `tianji_world_output` 的坐标变换与当前 `tianji_robot.yaml`；
- 已在真机通过 `DEMO_PYTHON/showcase_position.py` 验证的
  `SDK_PYTHON/fx_robot.py`、`DCSS` 和同目录 `libMarvinSDK.so`。

项目内 `vendor/marvin_sdk` 是从
`/home/boen/tianji/TJ_FX_ROBOT_CONTRL_SDK-master/SDK_PYTHON` 原样固定
的已验证控制 SDK，同时供 ROS 安装与 Pixi 包使用。也可在打包时通过
`PICO_BODY_TIANJI_MARVIN_SDK_DIR` 显式指定另一套完整 SDK。打包后
`VENDOR_SHA256SUMS` 固化其内容，目标机不会回退到其他 Marvin SDK。

包内不含 APK、相机、灵巧手、HTC、官方控制节点或 `libKine.so`。
源码提供可选的 `tianji_official` IK 适配器，但只有显式配置外部
`libKine.so` 与匹配的 `*.MvKDCfg` 后才会加载；默认 Pinocchio 后端不
依赖厂商运动学库。
RViz、MuJoCo、robot_state_publisher 和 Marvin URDF/mesh 仅用于本项目
纯运动学可视化，不替代控制器安全功能。厂商二进制保持原样，具体使用
与再分发权限以设备及 SDK 授权条款为准。

为运行 PICO 输入和坐标转换，打包脚本仅从现有工作区复制以下白名单
文件：`incremental_controller.py`、`one_euro_filter.py`、
`config_loader.py`、`transform_utils.py` 和 `tianji_robot.yaml`。不会
复制完整官方仓库，也不会修改这些来源文件。
