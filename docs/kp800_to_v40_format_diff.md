# kp800 数据包 ↔ 天机 v4.0 回放格式差异分析与适配要求

## 1. 背景与结论

kp800 数据包(`pkg_kp800_20260827/1_原始动捕/*.h5`)与天机项目回放链路
(`pixi run sim_mocap_h5 -- <take.h5>`)支持的 mocap-acquisition **v4.0
格式不同**,本地回放链路目前无法直接读取。

本文通过 **take001 双版本交叉验证**(同一次采集的两版导出:
`/home/current/data/20260826/20260826_163712_837567_take001.h5` 为 v4.0,
`pkg_kp800_20260827/1_原始动捕/hammer__20260826_163712_837567_take001.h5`
为 kp800 导出),实证锁定全部差异点,并给出 kp800 侧所需的适配修改要求。

**结论**:数值上两版数据的动捕内容一致(物体轨迹几乎逐帧吻合),差异集中在
**导出布局、时间轴、四元数元素序与 wrist 局部系、键点语义**四个方面。
适配只需按第 4 节逐条修改导出端,本地链路零改动。

## 2. 差异清单(全部经数值验证)

| # | 差异点 | v4.0(本项目) | kp800(对方) | 证据 |
|---|---|---|---|---|
| 1 | 顶层布局 | `hands/{left,right}/` 组 + `objects/hammer/` + `time_ns` + `valid` + `events` | 裸名 7 个数据集:`robot_keypoints / robot_pos / robot_quat / robot_joints / mano_joint_coords / object_pos / object_quat` | 结构对比 |
| 2 | 时间轴 | `time_ns`(int64,60 Hz 单调递增真实时间戳) | **无时间轴**,仅按帧序(50 Hz) | 结构对比 |
| 3 | valid 标记 | 根 `valid` + 每侧 `hands/<side>/valid` + `objects/hammer/valid` | **无** | 结构对比 |
| 4 | 元数据 | `h5_version="4.0"`、`schema_layout="compact-aligned-60hz-v1"`(加载器强制校验)、`output_hz`、`take_id` | regrind 流水线属性(`retarget_pipeline`、`robot_keypoint_mano_ids` 等) | 根属性对比 |
| 5 | wrist 位置 | `hands/right/wrist_position`(Motive 系,x-forward / z-up,米) | `robot_pos`——**数值一致**(逐帧差约几 mm,属采样/对齐噪声) | 范围对比一致 |
| 6 | wrist 姿态·元素序 | `wrist_quaternion_xyzw`(**xyzw 序**) | `robot_quat`(**wxyz 序**) | `object_quat` 重排 xyzw 后与 v4.0 角度差 **p50=0.30°,p95=2.94°**,实锤 |
| 7 | wrist 姿态·局部系 | W 系:+x=指尖,+y=手背,+z=小指(采集端标定) | 局部系与 W 系差**固定旋转**(约 90°),需乘 `R` 转换 | 手静止帧段验证 `R_rel` 恒定(见下) |
| 8 | 键点·点序 | MediaPipe 21 点序(0=腕,1-4 拇指,5-8 食指,9-12 中指,13-16 无名,17-20 小指) | `robot_keypoint_mano_ids = 0,13,14,15,16,1,2,3,17,4,5,6,18,10,11,12,19,7,8,9,20` → 与 MediaPipe **同构**,无需重排 | 属性声明 |
| 9 | 键点·坐标 | `keypoints_world` 为**腕部相对**(0 号点≈0) | `robot_keypoints` 为**全局绝对**(0 号点==`robot_pos`,逐帧精确相等) | 数值验证 |
| 10 | **键点·语义** | Manus 手套**人手** 21 键点 | `robot_keypoints` **不是同一组键点**(对齐帧数值差 70~90 mm,重排与否都不吻合) | 数值对比 |
| 11 | 左右手 | `hands/left` + `hands/right` 都必须存在(一侧可全无效,按"保持 Home"处理) | 仅右侧 `robot_*` | 结构对比 |
| 12 | 物体 | `objects/hammer/{object_position, object_quaternion_xyzw, valid}` | 裸名 `object_pos / object_quat`(quat 为 wxyz 序) | 结构 + 数值对比 |
| 13 | 帧率 | 60 Hz(加载器按 `output_hz` 属性支持任意值) | 50 Hz(声明) | manifest |

### 7 的补充:W 系 ← kp800 系的固定旋转

对帧对齐后的 wrist 姿态做 `R_rel = R_W · R_kp800⁻¹`(kp800 侧已按 wxyz
重排),在手静止帧段验证为恒定旋转:

```text
R ≈ [[ 0.035,  0.998, -0.03 ],
     [ 0.95,  -0.02,   0.32 ],
     [ 0.32,  -0.04,  -0.95 ]]
```

即 kp800 局部系 x→W 的 y、y→W 的 x、z→W 的 −z(约 90° 的轴置换旋转,
具体由 regrind 流水线的手掌参考系决定)。对方导出前需把 wrist 四元数
乘该旋转(或从源头对齐手掌参考系定义)。

## 3. v4.0 格式规范(加载器 `load_mocap_h5` 的实际校验规则)

对方适配时的**硬性要求**(不满足即拒绝加载):

### 3.1 根属性

| 属性 | 要求 |
| --- | --- |
| `h5_version` | 必须为 `"4.0"` |
| `schema_layout` | 若存在必须为 `"compact-aligned-60hz-v1"`(建议直接写入) |
| `output_hz` | 可选,回放按此帧率推进(可填 50.0) |
| `take_id` | 可选 |

### 3.2 数据集布局

```text
/                     根属性: h5_version, schema_layout, output_hz, take_id
├─ time_ns             (N,) int64   真实/合成时间戳,必须严格单调递增, N≥2
├─ valid               (N,) uint8   整帧有效标记(可全 1)
├─ events/             (可选) 录制事件
├─ hands/
│  ├─ left/            必须存在组(可全无效)
│  │  ├─ keypoints_world        (N,21,3) float  腕部相对, MediaPipe 点序
│  │  ├─ wrist_position         (N,3)   float   Motive 系(x-forward/z-up,米)
│  │  ├─ wrist_quaternion_xyzw  (N,4)   float   xyzw 序, W 局部系
│  │  └─ valid                  (N,)    uint8
│  └─ right/            同 left
└─ objects/
   └─ hammer/
      ├─ object_position        (N,3) float
      ├─ object_quaternion_xyzw (N,4) float   xyzw 序
      └─ valid                  (N,)  uint8
```

### 3.3 加载器防御性清洗(不满足的帧被标记无效,不报错)

- 键点 0 号(腕)与 `wrist_position` 每帧差 ≤ 1e-5 m(即"腕部相对"的
  直接含义,写入前确保 `keypoints_world[:,0] == 0`);
- `wrist_quaternion_xyzw` 每帧范数 ∈ [0.95, 1.05],数值有限;
- 所有数组数值有限;`time_ns` 严格单调递增。

### 3.4 坐标系约定

- 世界系:Motive 系 x-forward / z-up(+X 操作者前,+Y 操作者左,+Z 上),
  与机器人 world 系同向;
- wrist 局部系 W:`+x=指尖, +y=手背, +z=小指`(键点几何验证过的定义);
- 键点序:MediaPipe(与 kp800 的 `robot_keypoint_mano_ids` 同构)。

## 4. 对方(kp800 导出端)逐条修改点

对应第 2 节差异表序号:

1. **布局**:把裸名数据集组织进 v4.0 的 `hands/left|right/` 组与
   `objects/hammer/` 组,补齐 `time_ns`、`valid`(根 + 每侧 + objects)。
2. **时间轴**:写 `time_ns = np.arange(N) * int(1e9/output_hz)`,严格递增。
3. **valid**:全 1 即可(本会话中右手有效帧约 99%)。
4. **元数据**:写 `h5_version="4.0"`、`schema_layout`、`output_hz=50.0`。
5. `robot_pos` → `hands/right/wrist_position`(数值已一致,直接搬)。
6. **四元数元素序**:`robot_quat`(wxyz)→ 重排为 xyzw 写入
   `wrist_quaternion_xyzw`。
7. **wrist 局部系**:按第 2.7 节的固定旋转 `R` 转换后再写入;或从
   regrind 源头对齐手掌参考系(推荐,一劳永逸)。
8. 点序已同构,无需重排(建议保留 `robot_keypoint_mano_ids` 属性以便复查)。
9. **键点坐标**:`hands/right/keypoints_world = robot_keypoints − robot_pos`
   (腕部相对);0 号点必须为 0。
10. **键点语义(最重要)**:`robot_keypoints` 与 v4.0 的 `keypoints_world`
   不是同一组键点(数值差 70~90 mm),请从**原始 Manus 手套键点**导出
   21 点 MediaPipe 序键点写入 `hands/right/keypoints_world`;若无法提供,
   需与项目方确认 `robot_keypoints` 的确切来源与坐标系后再定。
11. **左手**:写 `hands/left/` 组(可全 NaN + `valid=0`,链路按保持 Home
   处理);若后续需要左手回放再补真数据。
12. **物体**:`object_pos/object_quat` → `objects/hammer/object_position`
   与 `object_quaternion_xyzw`(quat 重排为 xyzw)+ `valid`。
13. 帧率:保持 50 Hz 即可(`output_hz=50.0`)。

## 5. 验证方式

对方按以上修改导出任一 take 后,可用本项目快速校验:

```bash
# 加载校验(结构/属性/清洗规则全走一遍,不启动节点)
pixi run python -m pico_body_tianji.controller_only.mocap_h5_replay_node \
  <new_take.h5> --validate-only

# 或与 v4.0 版 take001 做数值比对(应逐帧吻合)
```

## 6. 附:本次实证所用的双版本 take

| 版本 | 路径 |
| --- | --- |
| v4.0 | `/home/current/data/20260826/20260826_163712_837567_take001.h5`(411 帧@60 Hz) |
| kp800 | `/home/current/Documents/pkg_kp800_20260827/1_原始动捕/hammer__20260826_163712_837567_take001.h5`(821 帧@50 Hz) |

两版为同一次采集(v4.0 为其中 6.8 s 子段,经物体轨迹最近邻对齐)。
