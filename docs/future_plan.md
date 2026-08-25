# 未来计划（供后续 Agent 阅读）

> 本文档记录 mocap H5 轨迹回放链路的当前状态、阻塞项与后续任务，
> 供后续接手工作的 Agent 快速建立上下文。最后更新：2026-08-20
> （commit `47daec3`，分支 `zenoh`）。

## 1. 链路背景（先读这个）

H5 右腕轨迹回放是"录制 → 回放"两段式链路：

```
录制（Windows/Motive 侧）              回放（本项目，Linux）
Motive 刚体 + Manus 手套骨架    →     HDF5（Motive 系绝对位姿）   ← 数据源
        │                              │
        └─ acquisition 项目    →       mocap_h5_replay_node（动捕系绝对目标）
                                      ↓
                              mocap_to_robot（同向轴映射）
                                      ↓
                              world→chest → One-Euro → 1:1 整形
                                      ↓
                              tianji_kinematic_sim（IK）→ 关节命令
                                      ↓
                              （真机）marvin_hardware_bridge 安全桥
```

**坐标系（关键，此前踩过坑）**：

| 坐标系 | +X | +Y | +Z |
|---|---|---|---|
| Motive（动捕，z-up 右手系） | 前 | 左 | 上 |
| PICO（手柄，y-up） | 右 | 上 | 后（朝用户） |
| 机器人 world（REP 103） | 前 | 左 | 上 |
| right_chest（IK 目标系） | 前 | 上 | 右 |

Windows Motive、H5 与机器人世界系已统一为 +X 前、+Y 左、+Z 上。
链路使用单位矩阵（见
`vendor/python/tianji_world_output/config/tianji_robot.yaml`）：

```
mocap_to_robot = [[1,0,0],[0,1,0],[0,0,1]]
```

H5 `hands/<side>/wrist_*` 是 Manus 人手 wrist 位姿；机器人端点必须是
wuji hand2 beta1 厂商 URDF 的 `r_wrist`。Motive `tianji_wrist`
rigid 经 GL/GO 定位 marker，再通过安装链定位 `r_mount` 和
`r_wrist`；`r_mount` 不能代替 Manus wrist 端点。对齐后的
`r_wrist` 经 `inverse(T_tcp_wrist)` 转成 Tianji TCP IK 目标。

## 2. 当前状态（2026-08-20）

**已实现并验证**（`sim_mocap_h5` / `real_mocap_h5`）：

- 绝对 wrist→r_wrist frame0 接近：s 读取实时 raw rigid，经 GL/GO、
  marker→r_mount→r_wrist 推导 Home；Enter 保压接近，稳定后 r 装载
  后续轨迹；
- raw `tianji_wrist` rigid→URDF `marker_mocap` 使用 Motive Visuals：
  GL `[1,-4,2]mm`、GO Pitch/Yaw/Roll `[-1,10,0]deg`；
- marker_mocap→r_mount：
  `[0.004,0,0]m` / `[0,-0.70710678,0,0.70710678]`；
- 厂商 beta1 `r_mount→r_wrist`：
  `[0.003,0.00025016,-0.0285]m` /
  `[0,0,0.0000081995,0.99999999997]`；
- TCP→r_mount：`[0,0,0.008]m` /
  `[0.70710678,0.70710678,0,0]`；
- H5 Manus wrist→wuji2 r_wrist：`[0,0,0]m`，
  旋转 `[[0,0,-1],[0,-1,0],[-1,0,0]]`（det=+1），四元数
  `[0.70710678,0,-0.70710678,0]`；
- `mocap_to_robot` 为单位世界轴映射；
- 组合 URDF 物理链为 TCP→marker(tianji/center/wuji2 三 frame)
  →wuji2 `r_mount`→`r_wrist`→fingers；
- `real_mocap_h5` 严格 readiness，marker 名称日志去重。

**验证证据**：

- pose compose/inverse/frame0 alignment/wrist→TCP round-trip 单测；
- H5 frame0 映射后虚拟 TCP 目标恰为 IK Home；
- marker 与 wrist 诊断分别保留，误差比较使用同端点 wrist；
- e2e 键盘步进方向与组合 URDF 原点/坐标轴验证通过。

## 3. 当前剩余工作（最高优先级）

**完成真实 take 的 wrist 相对轨迹仿真与真机验收。**

旧现象（绝对首帧模式）为 0.46m 跳变并触发 workspace saturation。
这不是单纯 wrist offset 造成，而是把不同 Motive 绝对位置的 marker
中心与人手 wrist 首帧直接相减。新语义已删除该 approach：frame0
对齐到 robot wrist Home，因此不应再出现首帧大跳变。

仍需标定 Manus wrist offset，提高 H5 wrist 绝对姿态/相对旋转的精度；
但 offset 不再用于要求机器人追人的绝对世界位置。

## 4. 任务清单（按优先级）

### P0：运行 wrist 对齐后的完整仿真验收

```bash
pixi run sim_mocap_h5 -- /path/to/take.h5 --wuji2 \
  --speed 0.1 --yaw-deg 0 --right-rigid-id <实际ID>
```

操作：s → Enter 保压到绝对 frame0 → 松开 → r → Enter 回放 → s 回 Home。
确认端点为 wrist、无非预期跳变，并记录 saturation/工作空间限制。

### P1：标定 Manus wrist offset 并重录

```bash
cd /home/current/syz/mocap/acquisition
bash scripts/calibrate_wrist_offset.sh right --user shd
bash scripts/calibrate_wrist_offset.sh left --user shd
```

结果写入 `offset/<user>.yaml`；重录后核对 H5
`effective_config_yaml`，重点验证相对旋转和手腕局部轨迹。

### P2：真机验收（有实体机器人时）

```bash
# 终端 1
pixi run sim_mocap_h5 -- /path/to/new_take.h5 \
  --topics-only --speed 0.1 --yaw-deg 0 --right-rigid-id <实际ID>
# 终端 2（确认 48V、急停、清场、刚体 ID 后）
pixi run real_mocap_h5 -- --confirm-real
```

等待桥输出“真机链路已就绪”（phase=armed_idle）后：s 读取 marker；
Enter 短按/保压接近绝对 frame0；稳定后松开，按 r，再分段推进轨迹。
桥默认 10% 速度/加速度，首次验收**不要**调高 ratio。

### P3：工程性收尾

- [ ] Windows 端 `natnet-zenoh` 的名称映射与 ID 不一致问题：当前
  `mocap/rigid_body_names` 报 `{10: 'right_arm'}`，但实时帧实际 ID 是
  `3`（配置注释里 ID 与名字还有历史错位：`left_wrist`/`right_wrist`
  名字与物理位置相反）。根治在发布端，本项目用 `--right-rigid-id 3`
  临时规避。
- [ ] 评估是否把 `--yaw-deg` 标定流程文档化（录制时人朝向 vs 机器人
  正前方）。
- [ ] 双臂 H5 回放（当前只发布右臂，左臂保持 Home）——明确需求后再做。
- [ ] 软工作区参数是否要按真机安全距离重新标定（当前 [0.42, 0.38, 0.38]
  是手柄链路沿用值）。

## 5. 给后续 Agent 的避坑清单

1. **不要**把 Motive 数据当 PICO 系用 `pico_to_robot`；必须用
   `mocap_to_robot`（动捕同向映射）。
2. 修改 `vendor/python/` 下任何文件后，**必须**更新根目录
   `VENDOR_SHA256SUMS`（doctor 完整性校验会拦截），并重新
   `pixi run -e ik-build deploy-ik` 部署 runtime。
3. 改 Python 节点后部署：`pixi run -e ik-build cmake --install build/ik
   --prefix staging/ik && pixi run -e ik-build deploy-ik`。
4. 运行锁：同一时刻只能有一套受管遥操作任务（`/tmp/pico-tianji-teleop-1000/guards`）。
   冲突检查看 `tj/live`（zenoh liveliness）。残留孤儿进程用
   `pgrep -af 'tianji_kinematic_sim|pico_controller_input|mocap'` 排查，
   确认无主后 kill（曾有 1 天前的孤儿 `pico_controller_input` 阻塞启动）。
5. Enter 保压依赖 X11 物理键状态（X11KeyState 读 Return/KP_Enter）；
   无 DISPLAY 时节点拒绝自动运动。自动化测试用 `xdotool keydown/up Return`
   （DISPLAY=:0）。
6. 方向验证速查：动捕 +z（操作者前）→ 机器人 +X（前）；按 `↑` 应看到
   机器人前进，按 `←` 应看到机器人左移。H5 首帧位移若"看到前移但推导
   后移"，说明又用了错误的矩阵。
7. 刚体 ID 确认：订阅 `mocap/hands/frame` 看 `rigid_bodies[].id`，
   与 `mocap/rigid_body_names` 对齐；不一致时用 `--right-rigid-id` 显式指定。
8. 测试：`pixi run test`（doctor + 单测 + portable IK 烟测）。单测
   直接跑 `pixi run bash -lc 'source scripts/common.sh && activate_bundle_runtime
   && python -m unittest tests.test_mocap_h5 tests.test_mocap_h5_replay ...'`。

## 6. 相关文件索引

| 内容 | 路径 |
|---|---|
| H5 加载/轨迹/偏航 | `src/pico_body_tianji/pico_body_tianji/controller_only/mocap_h5.py` |
| H5 回放节点（状态机） | `.../controller_only/mocap_h5_replay_node.py` |
| 动捕映射器 | `.../controller_only/controller_only_mapper.py` |
| 输入→机器人矩阵 | `vendor/python/tianji_world_output/config/tianji_robot.yaml` |
| 配置加载/查询 | `vendor/python/tianji_world_output/config_loader.py`、`transform_utils.py` |
| 增量控制器（滤波+映射） | `vendor/python/pico_input/incremental_controller.py` |
| 真机桥就绪契约 | `src/pico_body_tianji/pico_body_tianji/host_readiness.py` |
| 真机桥 | `.../marvin_hardware_bridge.py` |
| 运行入口 | `scripts/run_mocap_h5_replay.sh`（sim）、`scripts/run_mocap_h5_real.sh`（real） |
| 采集/标定（外部仓库） | `/home/current/syz/mocap/acquisition/`（`scripts/calibrate_wrist_offset.sh`） |
| 测试 | `tests/test_mocap_h5*.py`、`tests/test_mocap_keyboard_step.py`、`tests/e2e_mocap_keyboard_step.py` |
