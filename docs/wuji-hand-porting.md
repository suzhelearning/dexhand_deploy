# 官方舞肌二代手迁移进度

2026-09-08：算法和输入边界已隔离迁移并完成下述离线测试，**未接入新会话producer，未做设备验收**。

## 固定来源

- `third_party/wuji_hand_retargeting` 来源于顶层 `wuji-retargeting` 提交 `5875d0b9d1557a4eca118387cf79e99f3c14cb1d`，不是 `wuji_teleop` 内嵌副本。14个源码/配置/模型/许可证文件逐字节检查；清单见该目录 `source_manifest.json`。保留原MIT许可证。
- 二代手URDF来自用户指定远端资产，记录的wuji-description提交为 `8271644a78d69ed9a4adcf9165d882c64ad33dfa`；源文件摘要独立记录。只复制本算法所需URDF，未复制该子模块的全部可视化资产，模型分发许可证仍须完成审核。
- 实际配置显式选择 `adaptive_analytical_manus_wuji_hand_2_left.yaml` / `right.yaml`，不使用原bridge默认的wuji_glove配置。
- `tools/wuji_hand_native` 是独立环境。105个conda依赖包从原 `wuji_teleop/pixi.lock` 的linux-64闭包提取，保持原URL、构建号和摘要；不含ROS、设备SDK或GUI。源锁文件SHA256：`6034ad83a5e26fa1255fc5a64508ffa9178f71732568e290c578d13a683bcbd4`。
- 实测导入版本：NumPy2.4.4、SciPy1.17.1、NLopt2.10.1、Pinocchio4.0.0、PyYAML6.0.3。主工程和SPARK3.9环境不改。隔离锁为v6，当前pixi会提示格式升级；未因此重新求解或替换依赖。

## 已验证的算法一致性

本机迁移代码运行 `manus_gjy_dual_20260827.pkl`，与此前在远端原始环境生成的 `original_hand_trace.json` 比较：

- 1339帧，每帧left20+right20，关节名称、时间、配置/输入/bridge摘要一致。
- 最大关节误差0 rad，输出float64字节SHA256一致：`8b965b1788a23813b3b2537cd4f8eec7657ae5d1193a95cdb23e0d99fef51b75`。
- 原版顺序保留：桥回调序号间断reset → Retargeter → 官方URDF限位裁剪 → 官方关节名称重排。
- `scripts/wuji_hand_worker.py` 是有界stdin/stdout入口，保留上述bridge，不创建ROS节点、socket或设备。异常退出，不静默重启或替换算法。已测试首帧与基准一致、回调序号跳跃重置后结果一致、非法输入不输出关节结果。

这里的回放输入已是MediaPipe21，不是原始Manus25。它与PICO的TJVR不同步，不构成联合硬件遥操验收。

## Manus输入边界

新增 `hand_tracking/reference_manus/`，从没有Git元数据的原 `manus` 目录固定两个文件摘要，唯一代码变换为包内相对import。导入时不启动ROS。

- 按语义选点，float32，只翻转一次Y，**不减腕点**。
- 双侧回调内容顺序为right63+left63，任意一侧更新可与另一侧最新帧组成回调；两侧未齐不输出。
- 保留源序号乱序忽略、无效侧清除，以及原来源序号与桥回调序号的区别。
- 原有 `hand_tracking/manus.py` 的wrist-relative统一骨架定义不改，新参考路径不得误用旧观察结果作为未经验证的替代输入。
- 当前选点/异步合并测试使用标识明确的构造样本；尚需真实rawviz25点样本对照完整采集过程。原版没有新的side年龄保护，本模块未擅加改变参考序列的阈值；部署保护由后续会话层显式定义。

## 复现

```bash
pixi install --manifest-path tools/wuji_hand_native/pixi.toml --locked
WUJI_REFERENCE_TEST=1 PYTHONPATH=src/tianji_teleop:vendor/python \
  pixi run python -m unittest tests.test_wuji_native_environment \
    tests.test_wuji_port_sources tests.test_wuji_reference_equivalence \
    tests.test_wuji_native_worker tests.test_reference_manus -v
```

离线worker入口：

```bash
pixi run --manifest-path tools/wuji_hand_native/pixi.toml \
  python scripts/wuji_hand_worker.py
```

每行请求schema1、kind=`wuji_hand_input`、正整数 `callback_sequence` / `timestamp_ns`、`points` 为63或126个数。单侧63默认right，可显式 `--single-hand-side left`。输出kind=`wuji_hand_result`，每侧包含有效位、原版关节名称和20关节rad结果。**这不是会话启动指令，输出不拥有执行权限。**

新增`producers/hand_retarget.py`：隔离进程持久客户端、严格双侧结果关联与官方名称→canonical名称适配；不重复retarget或裁剪。`HandRetargetProducer`保留每回调推进、序号跳变原版reset，未授权/失效侧不输出命令，失败不静默重启。回调序号与Manus设备序号始终分开。

`scripts/vr_manus_sim_smoke.py`已验证完整短TJVR与独立1339帧Manus回放：2678条手命令应用到原版MuJoCo模型，手关节qpos与官方输出最大差值0；机械臂同时通过native/coordinator/sim直出检查。两份输入不是同步硬件录制。HDF5 1.2单独保存retarget前回调，不把21点文件伪装成raw25。

剩余：rawviz进程及实时采集接线、与旧统一21点观察接口的明确分流、live profile/实际SDK direct执行接线、PICO2官方手指路线、完整录制诊断及硬件联调。
