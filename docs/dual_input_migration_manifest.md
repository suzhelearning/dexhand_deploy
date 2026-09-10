# 双输入迁移来源基线

当前工程基线：`63e1e0f`，分支 `PICO_Hand_Tracking`。顺序实施，不提交。

来源根目录为 `pico-manus-teleop`；审计时的实际目录由`PICO_MANUS_TELEOP_ROOT`注入，以下路径不是部署时硬编码依赖。

| 来源 | 固定版本 | 检查结果 |
|---|---|---|
| PICO_tracker | 4aeb6f595e56ec9a273656ff5e122ee7a456c59b | 工作区检查干净 |
| manus | 无Git元数据，使用下列文件SHA256 | 不能声称已固定SDK与全部采集资产 |
| wuji_teleop | 59028470b8be4abfdd052ff35a06ca73413d9590 | 本机无git-lfs；禁用filter只读检查显示两项Manus SDK .so差异，尚未判定为内容修改还是LFS表示差异 |
| wuji-retargeting（顶层） | 5875d0b9d1557a4eca118387cf79e99f3c14cb1d | 主工作区检查干净，模型子模块未初始化 |
| TJ_arm_control | c022b17789e9b81141915662b3bb802f4c20a396 | feature/pico-manus-teleop，工作区检查干净 |

选定顶层wuji-retargeting的 `example/tj_wuji2_hand_bridge.py` 作为联合手链基准；内嵌retarget和SDK路径不作为数值等价替代品。

## 文件SHA256

```text
manus/rawviz.cpp
fe151c7ecfe8cbf8afa19bdaa0a72649094513e956353f1f2018a842602cd15c
manus/wuji2_hand_input.py
23ffe81fe04451a43876a79b849cbafb200461b4957ce29d561e81913cef3d96
manus/manus_hand_input.py
3036934217296295ba4d87df393fe717d9e701e13142caaf1ea0b43585b1887c
TJ_arm_control/config/qp_ik_pico_teleop.yaml
ae05543aeb8bdd52f2e4f909163ae9efe47d1253075df17468af57ce5a5af1ef
TJ_arm_control/models/marvin_m6_wuji2.xml
dcb3040c7cb5b5d1d897c56f9e5834df0c682d179919baa6a1fef60ce782b4ae
wuji-retargeting/example/tj_wuji2_hand_bridge.py
ef7ad3b1f9c389a5fc3a0f970e11f7fffeaa9f0af217b3fd45430fb6228e76cb
TJ_arm_control/src/pico_teleop_protocol.cpp
63f65f990efd5e63aa6e11add0fa037c0f775a55b87bdf26820c1580a5540568
TJ_arm_control/src/so3.cpp
cfc7d4779bf5b1ef6b3741704897fe43d9447d40588495bd6c878b042e001285
```

## 首次本地检查时的缺口（后续取回状态见下节）

- `wuji_retargeting/wuji-description` 指向 `8271644a78d69ed9a4adcf9165d882c64ad33dfa`，尚未初始化；默认二代手YAML引用的URDF缺失。
- `example/utils/mujoco-sim` 子模块指向 `46ba87ed9913da1b9d95e670c2b6e87135a23245`，尚未初始化。
- `example/data/manus_gjy_dual_20260827.pkl` 与 `avp1.pkl` 为LFS指针，不能当作真实回放样本加载。
- 首次在feature目录搜索未找到 `.tjvr`；后续已从旧main克隆取回两份，见下节。README中的9442帧描述不作为这两份样本的帧数。
- 操作员标定、全部模型依赖闭包及许可证尚需逐项清点。没有真实输入样本时只能报告synthetic验证。

这些缺口不阻碍纯配置/协议开发，但在获取和固定完整资产前不能标记官方retarget或联合等价性验收完成。参考仓库未被改写。

## 远端基准取回记录

用户提供源：`zj@192.168.110.210`。远端只读核实，未改配置、启动进程或执行器。身份凭据不写入文档或脚本。

本地副本：`vendor/reference_inputs/remote-192.168.110.210-20260908/`，位于既有Git忽略目录，人员标定与录制不加入提交。该目录是参考资产，不自动应用为当前机器运行配置。复制目录中的tracking_epoch/lock只是采集时附带文件，不作为迁移运行状态恢复来源。

已取回：

- `pico_tracker/`：用户列出的七份正式YAML，源自远端用户配置目录（运行时由操作者提供，不写死主机路径）。
- `recordings/`：PICO TCP/pivot/臂长候选采样与报告。
- `calibration/`：GJY左右Manus `.mcal`。
- `config/`：远端retarget配置，实际基准选manus命名的左右二代手配置。
- `urdf/`：远端官方子模块中的左右手URDF（包含ROS变体）；未复制全部245MB资产，完整可视化依赖闭包仍需核对。
- `manus_gjy_dual_20260827.pkl`：772839字节，受限NumPy反序列化检查得到1339帧，首末时间0.00361567921936512/14.998271892312914秒，首末双侧各21×3。不是原始25点录制，不能直接验证原始Manus25→21转换。

远端TJ_arm_control、PICO_tracker、wuji-retargeting HEAD与上表一致。远端手模型子模块为 `8271644a78d69ed9a4adcf9165d882c64ad33dfa`，已初始化。远端feature目录没有TJVR录制；实际录制来自旧main克隆，不据此更换上表固定的算法版本。

### PICO TJVR录制

已从旧版TJ_arm_control克隆的`benchmark_results/pico_live/traces/`取回到上述本地参考目录，传输摘要一致。

| 文件 | 字节 | 帧数 | 接收时间跨度 |
|---|---:|---:|---:|
| output_continuity_retest.tjvr | 1622832 | 2444 | 27.139897830 s |
| output_20260827_204546.tjvr | 17963872 | 27054 | 300.985973953 s |

```text
output_continuity_retest.tjvr
4253dc699a14d1a79712b3fcf7e95a88954dfda55bcd5f8ee51311a928354ce5
output_20260827_204546.tjvr
2e1cb545a7c6729a70b0f59c6f32810701ba4879cc19ad764ceea60c0be4481c
```

使用 `hand_tracking/tjvr_trace.py` 完整遍历：两份均为TJVT v1容器，内部全部为656字节TJVR v4包，CRC全部有效，接收时间单调不减。两份epoch均仅137、flags均0xff，不覆盖按键bit8触发和epoch切换。这只证明容器/包完整性，不代表位姿语义或SPARK算法等价性通过。

`.tjvr`只含PICO数据，不含Manus手指；上述 `.pkl` 是独立手部样本，没有证据表明与两份TJVR同步。分别用于机械臂和手链基准，不拼成同步联合验收数据。读取器保留原始包与相对接收时间，不重采样；控制层仍需复现原版每周期读取最新帧的策略。

新录制在feature工程内使用 `benchmark_results/pico_live/traces/output_<时间戳>.tjvr`，Viewer追加 `--pico-record`；提前创建父目录，每次使用不存在的文件名，禁止覆盖既有录制。

传输前后核对一致的SHA256：

```text
manus_gjy_dual_20260827.pkl
7cfa10e4cfaee104a847940820e034b12f3250279a3487aabc001be9a190d4ea
config/adaptive_analytical_manus_wuji_hand_2_left.yaml
9bbcd8bbe898a6b942aa972f6aaa28e9ec69ebdf7ed6ecb89fe48553fbb05770
config/adaptive_analytical_manus_wuji_hand_2_right.yaml
4f45c09d8f9847e3e2f44c30a9c91ad7532452e4911d6870ddb7b5c7a303c94b
```

模型本地摘要：left.urdf `cec0a7eb6a34fd82e200def7b75c1d477fad790b2de903aec58e59991994c471`，right.urdf `1ae70be3f5e64532203e599eaa98d2af368d0214be9c949a358b7abaa8b6265a`。

取回实际配置包含非默认 `input_axis_sign`、`mediapipe_rotation` 和 `segment_scaling`。不能把这些标定参数替换成桥默认配置，也不能省略，因为它们直接影响实际手指输出。

## 原版手部离线基线验证

使用远端现有 `wuji_teleop/.pixi/envs/default/bin/python`，直接调用原版 `OfficialWujiHand2Bridge`，显式指定实际Manus左右YAML。仅算法计算，不启动ROS节点、UDP发送或设备驱动。

依赖版本：Python3.12、numpy2.4.4、scipy1.17.1、nlopt2.10.1、pin4.0.0、pyyaml6.0.3。本地独立venv按这些版本安装时pip索引没有nlopt2.10.1，安装失败；没有降级算法依赖，也没有修改现有Pixi环境。

可复现脚本：`scripts/reference_manus_trace.py`。脚本经SSH stdin运行，完整JSON输出保存在本地参考目录的 `original_hand_trace.json`。输出顺序left20+right20，1339×40，所有值有限；两次独立运行输出SHA256一致：

```text
8b965b1788a23813b3b2537cd4f8eec7657ae5d1193a95cdb23e0d99fef51b75
```

生成规则：每条录制21点数据模拟一次桥callback，序号从1递增；时间取录制t。该回放不包含原始采集丢帧和ROS到达时序，不能证明实时输入消费完全等价，也不能用于25→21的源数据验证。生成基线时的结论仅为“指定实际配置下原版手算法可重现运行”；后续迁移对比结果如下。

## 本地隔离迁移和对照更新（2026-09-08）

- SPARK：固定原版依赖锁并构建独立原生后端；两个真实录制共65726个控制周期，与直接编译原版类和Viewer控制块的oracle比较，所输出字段最大差值0。详见 [SPARK移植报告](spark-porting.md)。不是实时或设备验收结论。
- 手部：上述pip缺少NLopt版本的问题已通过原版conda锁的105包闭包解决，未降级。原始retarget、bridge、实际Manus配置和URDF固定到隔离目录。1339×40结果与远端原版基准完全一致，输出SHA256不变。详见 [手部移植报告](wuji-hand-porting.md)。
- 原始Manus Python输入模块已按本页摘要迁移，新增独立float32/非wrist-relative路径；原有工程转换定义和所有旧运行profile不变。
- 当前新代码未提交、未接入新session运行入口；联合实时输入、授权、执行、HDF5及真机验收仍是独立未完成项。
