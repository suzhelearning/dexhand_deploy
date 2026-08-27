# mocap-acquisition HDF5 格式标准(v4.0)

本项目回放链路(`pixi run sim_mocap_h5`)唯一支持的动捕 H5 数据格式。
本文件是完整格式规范,供采集端、数据提供方与消费端共同遵循。

- 格式身份:`schema_name = "mocap-acquisition"`,`h5_version = "4.0"`,
  布局 `schema_layout = "compact-aligned-60hz-v1"`
- 加载器实现:`src/pico_body_tianji/pico_body_tianji/controller_only/mocap_h5.py`
  (`load_mocap_h5()`),以下校验规则以该实现为准

## 1. 目录结构

```text
/                                 根属性见 §2
├─ time_ns          (N,)  int64   公共时间轴,linux-clock-monotonic,60 Hz
├─ valid            (N,)  uint8   整帧有效标记(含物体刚体)
├─ events/                        (可选)录制事件
│  ├─ frame_index   (M,)  int64   事件所在帧下标
│  └─ type          (M,)  uint8   0=start, 1=stop(当前仅有这两个枚举)
├─ hands/
│  ├─ left/                       必须存在组;可完全无效(见 §4.3)
│  │  ├─ keypoints_world        (N,21,3) float32 腕部相对,MediaPipe 点序
│  │  ├─ wrist_position         (N,3)   float32 Motive 系,米
│  │  ├─ wrist_quaternion_xyzw  (N,4)   float32 W 系,xyzw 序
│  │  └─ valid                  (N,)    uint8
│  └─ right/                      同 left
└─ objects/                       (可选)动捕物体刚体,每个物体一组
   └─ <object_name>/              如 hammer
      ├─ object_position        (N,3) float32 Motive 系,米
      ├─ object_quaternion_xyzw (N,4) float32 xyzw 序
      └─ valid                  (N,)  uint8
```

## 2. 根属性

| 属性 | 必须 | 值/说明 |
| --- | --- | --- |
| `h5_version` | **是** | `"4.0"`,加载器强制校验,否则拒绝 |
| `schema_layout` | 若存在 | 必须为 `"compact-aligned-60hz-v1"`,否则拒绝 |
| `output_hz` | 否 | 采样率,默认 60.0;回放按此推进 |
| `take_id` | 否 | 录制编号(int) |
| `schema_name` | 否 | `"mocap-acquisition"` |
| `time_domain` | 否 | `"linux-clock-monotonic"` |
| `start_wall_ns` / `end_wall_ns` | 否 | 墙钟起止(int64) |
| `effective_config_yaml` | 否 | 采集配置(采集端写入) |

## 3. 数据集语义

### 3.1 time_ns

每帧真实时间戳(单调时钟),间隔严格为正(60 Hz 时 16 666 667 ns)。
**加载器强制校验:一维、≥ 2 帧、严格单调递增**,否则拒绝。

### 3.2 valid 标记

- 根 `valid`:整帧(含物体刚体)是否有效;纯动捕会话中物体未跟踪时
  常为 False,**不能用作手腕回放的门控**;
- `hands/<side>/valid`:单侧手部标记,手腕位姿缺失的帧标 False;
- `objects/<name>/valid`:物体刚体有效标记。

### 3.3 hands/<side>/(左/右手)

| 数据集 | 形状/类型 | 语义 |
| --- | --- | --- |
| `keypoints_world` | (N,21,3) float32 | 手部 21 键点,**腕部相对**坐标(0 号点=腕中心≈0),MediaPipe 点序 |
| `wrist_position` | (N,3) float32 | 手腕中心绝对位置(Motive 系,米) |
| `wrist_quaternion_xyzw` | (N,4) float32 | 手腕姿态,**xyzw 元素序**,W 局部系 |
| `valid` | (N,) uint8 | 该侧逐帧有效标记 |

键点 0 号(腕)必须与 `wrist_position` 重合(加载器按 ≤ 1e-5 m 清洗),
即 `keypoints_world` 天然满足"腕部相对"定义。

**MediaPipe 21 点序**:0=腕,1-4=拇指,5-8=食指,9-12=中指,
13-16=无名指,17-20=小指。骨段连接(20 条边):

```text
0-1, 1-2, 2-3, 3-4                    拇指
0-5, 5-6, 6-7, 7-8                    食指
0-9, 9-10, 10-11, 11-12              中指
0-13, 13-14, 14-15, 15-16            无名指
0-17, 17-18, 18-19, 19-20            小指
```

### 3.4 objects/<name>/

动捕物体刚体(如锤子)的位姿轨迹。当前回放节点不消费该组,保留给
viewer/后续功能;写入时仍应遵循同样形状与 xyzw 四元数约定。

## 4. 坐标系与语义约定

### 4.1 世界系

Motive 系:**x-forward / z-up** 右手系,米制
(+X 操作者前,+Y 操作者左,+Z 上),与机器人 world 系轴完全同向,
故 `mocap_to_robot` 为单位阵;`apply_yaw_world` 绕竖直轴(+Z)旋转
整个轨迹,用于标定录制时人的朝向与机器人正前方的夹角。

### 4.2 wrist 局部系 W

采集端标定、经 21 点几何验证:

```text
W +x = 指尖方向   W +y = 手背法向   W +z = 小指方向
```

回放链路经固定 `W→wuji2` 变换(Ry(-90°))映射到机器人 `r_wrist` 系。

### 4.3 无效侧的处理

单侧可完全无跟踪(如左手全 NaN):加载器将该侧全部标记无效,
回放节点按"保持 Home"处理——**导出时必须存在 `hands/left` 与
`hands/right` 两个组**(数据结构完整),数值可全 NaN + `valid=0`。

## 5. 加载器校验规则(硬性 + 防御性清洗)

加载时依次执行(`load_mocap_h5`):

1. **外部链接拒绝**:含外部/软链接的 HDF5 直接拒绝(防恶意文件);
2. **版本**:`h5_version == "4.0"`;`schema_layout` 若存在必须匹配;
3. **时间轴**:`time_ns` 存在、一维、≥ 2 帧、严格单调递增;
4. **布局**:`hands/left` 与 `hands/right` 都必须存在,且各自含
   `keypoints_world / wrist_position / wrist_quaternion_xyzw / valid`
   四个数据集,形状分别为 (N,21,3)/(N,3)/(N,4)/(N);
5. **防御性清洗**(不满足的帧标记无效,不拒绝文件):
   - 所有数值有限(NaN/Inf 帧无效);
   - 键点 0 号与 `wrist_position` 差 ≤ 1e-5 m;
   - `wrist_quaternion_xyzw` 范数 ∈ [0.95, 1.05]。

## 6. 最小合规示例(Python)

```python
import h5py, numpy as np

N, HZ = 120, 60.0
out = "/tmp/demo_v40.h5"

def side_group(f, name):
    g = f.create_group(f"hands/{name}")
    g.create_dataset("keypoints_world", (N, 21, 3), dtype="f4")        # 腕部相对
    g.create_dataset("wrist_position", (N, 3), dtype="f4")             # Motive 系,米
    g.create_dataset("wrist_quaternion_xyzw", (N, 4), dtype="f4")      # xyzw,W 系
    g.create_dataset("valid", (N,), dtype="u1")
    return g

with h5py.File(out, "w") as f:
    f.attrs["h5_version"] = "4.0"
    f.attrs["schema_layout"] = "compact-aligned-60hz-v1"
    f.attrs["output_hz"] = HZ
    f.create_dataset("time_ns", data=np.arange(N) * int(1e9 / HZ))     # 严格递增
    f.create_dataset("valid", data=np.ones(N, dtype="u1"))
    for side in ("left", "right"):
        g = side_group(f, side)
        g["wrist_position"][:] = ...          # 绝对位置
        g["wrist_quaternion_xyzw"][:] = ...   # 归一化四元数(xyzw)
        g["keypoints_world"][:, 0] = 0.0      # 0 号点=腕
        g["keypoints_world"][:, 1:] = ...     # 腕部相对键点(MediaPipe 序)
        g["valid"][:] = 1
```

## 7. 回放链路消费方式(简)

- `reference_index()`:最早有任一侧有效数据的帧,作为回放参考帧;
- 键点 0 号必须与 wrist 重合,加载器依赖该性质做数值清洗;
- 完全无效侧:回放节点以合成参考位姿"保持 Home";
- `output_hz` 决定回放推进速率;`--speed` 为倍速;
- viewer 显示 frame0 骨架时按 MediaPipe 点序与 20 条骨段绘制。

## 8. 相关文件

| 文件 | 作用 |
| --- | --- |
| `src/pico_body_tianji/pico_body_tianji/controller_only/mocap_h5.py` | 加载器(`load_mocap_h5`/`MocapRecording`),校验与清洗 |
| `src/pico_body_tianji/pico_body_tianji/controller_only/mocap_h5_replay_node.py` | 回放节点(frame0 定位、轨迹推进、键盘门控) |
| `src/pico_body_tianji/scripts/mujoco_joint_viewer.py` | MuJoCo viewer(frame0 骨架、wrist 双轴显示) |
| `docs/kp800_to_v40_format_diff.md` | kp800 外部数据包与本格式的差异分析与适配要求 |
