# 转腕输入的原版/移植版对照（2026-09-13）

## 结论与边界

本段输入下没有发现移植算法的确定性数值分歧，但原版同样存在大幅转腕时的位置
跟踪误差。此版本提供仿真测试入口，不声明位置锁定、实时性能达标或真机验收通过。
算法权重、约束和可达性优化另行处理，不能凭 `accepted=true` 判断跟踪精度。

## 数据和参考版本

- 输入：`recordings/device_acceptance/mapped_palm_rotation_20260913_231901.h5`，
  complete=True，run_id `90d98938-076b-49e3-94bb-8024f6ff12ac`。
- 输入 SHA256：`36c4491a4863369c41442a250acbb28b9345d068488a8f60278cbf109abcf82f`。
- 7521 个原始 TJVR 包，源时间跨度84.481秒。
- 原版分支 `feature/pico-manus-teleop-experiments`，HEAD
  `d623c1b8df0edfba18e1d8b78f93a9f30ef612eb`，从干净源码独立构建。
- 正式配置 `qp_ik_pico_ee_bandwidth_velocity_qp.yaml`；模型 `marvin_m6_wuji2.xml`；
  启动确认 `hand_tcp_frame_L/R`、model_reference、velocity 和指定 mapped-palm 算法。

## 对照方法

原版 Viewer 使用 headless、model-state-only、deterministic-pico-replay、duration=85，
产生 telemetry 和 joint telemetry；当前版本使用 `scripts/check_mapped_palm_oracle.py`
按同一200Hz源时间、初始状态、latest-only消费顺序逐周期比较。

第一组：双方直接使用未修改的原始TJVR输入，不启用高度标定。

第二组：原版输入为明确标注的派生trace，仅给掌心点3/7的Z分别加
`-0.2656741232184414` / `-0.3455597183693726` m并重算CRC；
当前worker仍读取未改的原始包，通过`configure_height`设置这两个偏移。
不改肩/肘/腕、姿态或原始HDF5。

| 比较 | 周期 | 最大q差 rad | 最大qdot差 rad/s | 最大有效qddot差 rad/s² |
|---|---:|---:|---:|---:|
| 无高度偏移 | 17001 | 5.00e-12 | 5.00e-12 | 5.00e-11 |
| 等效高度偏移 | 17001 | 5.00e-12 | 5.00e-12 | 5.00e-11 |

两组全部7521帧消耗完毕，既有1e-7阈值未放宽；headroom/任务缩放最大差约5e-13，
input_live、applied epoch、applied sequence逐周期一致。

## 精度不是通过项

以现场首次native输入对应的源时间23.294秒为起点，统计其后5～58秒原版CSV的
`position_error_m`。这是源时间近似对应，不是逐个现场控制周期配对。

| 原版输入 | 左臂P95 cm | 右臂P95 cm | 左臂最大 cm | 右臂最大 cm |
|---|---:|---:|---:|---:|
| 无偏移 | 15.968 | 11.636 | 28.320 | 18.021 |
| 本次Z偏移 | 16.096 | 13.540 | 20.262 | 25.689 |

现场录制中下发关节角与对应仿真关节反馈逐周期相同；由这些角度重建掌心TCP后，
也观察到转腕时的显著位置误差。不是仅由腕部/法兰绕掌心运动造成的视觉变化。
这些结果不能单独确定是任务权重、关节约束、Jacobian还是可达性导致。

确定性回放从第一帧连续运行，不复现现场idle、按c/按s时机及线程抖动；
离线模式放宽求解墙钟预算，不证明实时或上游设备映射等价。
独立构建、派生trace及CSV留在临时目录 `/tmp/mapped-rotation-compare.wCh7rT/`，
不是项目部署依赖，不随代码提交。原始录制亦不纳入Git。
