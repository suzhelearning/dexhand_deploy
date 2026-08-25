#!/usr/bin/env python3
"""mocap-acquisition HDF5 文件（v4.0 紧凑 60Hz 布局）的只读加载器。

文件来源：/home/current/syz/mocap/acquisition 采集程序（schema 名
``mocap-acquisition``、布局 ``compact-aligned-60hz-v1``）。本模块只做
纯数据读取与校验，不依赖 ROS 2，可独立单元测试。

布局约定（v4.0）：

- 根属性 ``h5_version == "4.0"``，公共时间轴 ``time_ns``（int64，
  linux-clock-monotonic，固定 60 Hz）；
- 每帧 ``valid`` 标记**整帧**（含物体刚体）是否有效；由于物体刚体在
  纯动捕会话中通常从未跟踪，根级 ``valid`` 几乎恒为 False，**不能**
  用作手腕回放的门控；
- ``hands/<side>/valid`` 是单侧手部标记，手腕位姿为 Motive 系
  （x-forward / z-up 右手系、米制）：
  - ``hands/<side>/wrist_position``        (N,3) float32
  - ``hands/<side>/wrist_quaternion_xyzw``  (N,4) float32（xyzw 序）
- 单侧可能完全没有跟踪（如 take003 左手全 NaN）：本模块将其视为
  该侧全部无效，由回放节点按“保持 Home”处理。

安全防护：拒绝包含外部/软链接的 HDF5（与采集端 replay_hdf5.py
一致），防止恶意文件经 HDF5 外部链接读取任意本地文件。

坐标约定：Motive 系为 x-forward / z-up（+X 操作者前，+Y 操作者左，
+Z 上），与机器人 world 系（+X 前、+Y 左、+Z 上）轴完全同向，故
``mocap_to_robot`` 为单位阵。
``apply_yaw_world`` 绕 Motive +Z（竖直轴）旋转整个轨迹，等价于绕
机器人世界 Z（竖直轴）旋转，用于标定录制时人的朝向与机器人正前方
的夹角。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import h5py
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

SUPPORTED_H5_VERSION = "4.0"
SUPPORTED_SCHEMA_LAYOUT = "compact-aligned-60hz-v1"
SIDES = ("left", "right")

HAND_KEYPOINT_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20),
)

_QUAT_NORM_TOLERANCE = 0.05
_SYNTHETIC_REFERENCE_POSE = np.array(
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float64
)


@dataclass(frozen=True)
class HandRecording:
    """单侧手部轨迹（Motive 系，x-forward / z-up，米制）。"""

    valid: np.ndarray  # (N,) bool；False 表示该帧手腕缺失
    wrist: np.ndarray  # (N,7) float64 [x,y,z,qx,qy,qz,qw]
    keypoints_world: np.ndarray  # (N,21,3) float64，MediaPipe 顺序


@dataclass(frozen=True)
class MocapRecording:
    """一次 mocap-acquisition 录制的可用内容。"""

    path: Path
    time_ns: np.ndarray  # (N,) int64 公共时间轴
    hands: dict[str, HandRecording]
    output_hz: float
    take_id: int | None

    @property
    def frame_count(self) -> int:
        return int(self.time_ns.size)

    @property
    def duration_s(self) -> float:
        if self.frame_count < 2:
            return 0.0
        return float(self.time_ns[-1] - self.time_ns[0]) / 1.0e9

    def first_valid_index(self, side: str) -> int | None:
        """该侧第一个有效帧下标；完全无效返回 None。"""
        indices = np.flatnonzero(self.hands[side].valid)
        return int(indices[0]) if indices.size else None

    def reference_index(self) -> int:
        """回放参考帧（等效于按 A 的时刻）：最早有任一侧有效数据的帧。

        完全无效的侧由回放节点使用合成参考位姿保持 Home。
        """
        candidates = [
            self.first_valid_index(side)
            for side in SIDES
            if self.first_valid_index(side) is not None
        ]
        return min(candidates) if candidates else 0

    def summary(self) -> dict[str, object]:
        per_side = {}
        for side in SIDES:
            valid = self.hands[side].valid
            per_side[side] = {
                "valid_frames": int(valid.sum()),
                "valid_ratio": float(valid.mean()),
            }
        return {
            "path": str(self.path),
            "frame_count": self.frame_count,
            "duration_s": round(self.duration_s, 3),
            "output_hz": self.output_hz,
            "take_id": self.take_id,
            "hands": per_side,
        }


def reject_external_links(f: h5py.File, prefix: str = "") -> None:
    """递归检查并拒绝含外部/软链接的 HDF5（不解析链接，仅查类型）。"""
    for name in f:
        link = f.get(name, getlink=True)
        if not isinstance(link, h5py.HardLink):
            raise ValueError(
                f"拒绝含 {type(link).__name__} 的 HDF5: {prefix}{name}"
                + (
                    f" -> {link.filename}"
                    if isinstance(link, h5py.ExternalLink)
                    else ""
                )
            )
        obj = f[name]
        if isinstance(obj, h5py.Group):
            reject_external_links(obj, f"{prefix}{name}/")


def load_mocap_h5(path: str | Path) -> MocapRecording:
    """加载 v4.0 mocap-acquisition HDF5，返回纯数据回放结构。

    Raises:
        FileNotFoundError: 文件不存在。
        ValueError: 版本/布局不受支持，或必要数据集缺失/形状不符。
    """
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"mocap h5 文件不存在：{path}")

    with h5py.File(path, "r") as f:
        reject_external_links(f)

        version = str(f.attrs.get("h5_version", ""))
        if version != SUPPORTED_H5_VERSION:
            raise ValueError(
                f"不支持的 h5_version={version!r}，仅支持 "
                f"{SUPPORTED_H5_VERSION!r}（compact-aligned-60hz-v1）"
            )
        schema_layout = str(f.attrs.get("schema_layout", ""))
        if schema_layout and schema_layout != SUPPORTED_SCHEMA_LAYOUT:
            raise ValueError(
                f"不支持的 schema_layout={schema_layout!r}，仅支持 "
                f"{SUPPORTED_SCHEMA_LAYOUT!r}"
            )

        if "time_ns" not in f:
            raise ValueError("缺少公共时间轴数据集 time_ns")
        time_ns = np.asarray(f["time_ns"][:], dtype=np.int64)
        if time_ns.ndim != 1 or time_ns.size < 2:
            raise ValueError(
                f"time_ns 须为一维且至少 2 帧，实际 {time_ns.shape}"
            )
        if np.any(np.diff(time_ns) <= 0):
            raise ValueError("time_ns 必须严格单调递增")

        output_hz = float(f.attrs.get("output_hz", 60.0))
        take_id_attr = f.attrs.get("take_id")
        take_id = int(take_id_attr) if take_id_attr is not None else None

        hands: dict[str, HandRecording] = {}
        for side in SIDES:
            group_name = f"hands/{side}"
            if group_name not in f:
                raise ValueError(f"缺少手部组 {group_name}")
            group = f[group_name]
            required = (
                "keypoints_world",
                "wrist_position",
                "wrist_quaternion_xyzw",
                "valid",
            )
            for dataset in required:
                if dataset not in group:
                    raise ValueError(
                        f"{group_name} 缺少数据集 {dataset}"
                    )
            keypoints = np.asarray(
                group["keypoints_world"][:], dtype=np.float64
            )
            position = np.asarray(
                group["wrist_position"][:], dtype=np.float64
            )
            quaternion = np.asarray(
                group["wrist_quaternion_xyzw"][:], dtype=np.float64
            )
            flagged = np.asarray(group["valid"][:], dtype=bool)
            expected = (time_ns.size,)
            if keypoints.shape != (time_ns.size, 21, 3):
                raise ValueError(
                    f"{group_name}/keypoints_world 形状 "
                    f"{keypoints.shape} 与时间轴 {expected} 不符"
                )
            if position.shape != (time_ns.size, 3):
                raise ValueError(
                    f"{group_name}/wrist_position 形状 "
                    f"{position.shape} 与时间轴 {expected} 不符"
                )
            if quaternion.shape != (time_ns.size, 4):
                raise ValueError(
                    f"{group_name}/wrist_quaternion_xyzw 形状 "
                    f"{quaternion.shape} 与时间轴 {expected} 不符"
                )
            if flagged.shape != expected:
                raise ValueError(
                    f"{group_name}/valid 形状 {flagged.shape} "
                    f"与时间轴 {expected} 不符"
                )
            # 防御性清洗：数值非有限或四元数未归一化的帧按无效处理。
            quaternion_norm = np.linalg.norm(quaternion, axis=1)
            root_error = np.linalg.norm(
                keypoints[:, 0] - position, axis=1
            )
            numerically_ok = (
                np.isfinite(keypoints).all(axis=(1, 2))
                & np.isfinite(position).all(axis=1)
                & np.isfinite(quaternion).all(axis=1)
                & (root_error <= 1.0e-5)
                & (quaternion_norm > 1.0 - _QUAT_NORM_TOLERANCE)
                & (quaternion_norm < 1.0 + _QUAT_NORM_TOLERANCE)
            )
            valid = flagged & numerically_ok
            wrist = np.concatenate((position, quaternion), axis=1)
            hands[side] = HandRecording(
                valid=valid,
                wrist=wrist,
                keypoints_world=keypoints,
            )

    return MocapRecording(
        path=path,
        time_ns=time_ns,
        hands=hands,
        output_hz=output_hz,
        take_id=take_id,
    )


def apply_yaw_world(
    wrist: np.ndarray, yaw_deg: float
) -> np.ndarray:
    """把整条手腕轨迹绕 Motive 竖直轴（+Z）旋转 yaw_deg 度。

    Motive 系为 x-forward / z-up：竖直轴为 +Z，故绕 +Z 旋转。
    位置与姿态一起旋转：p' = Rz(θ)·p，q' = q_yaw ⊗ q（xyzw 序左乘）。
    经回放节点的 ``mocap_to_robot``（单位阵）映射后，等价于绕机器人
    世界 Z 轴旋转，用于对齐录制时人的朝向与机器人正前方。θ=0 时原样。
    """
    if not np.isfinite(yaw_deg):
        raise ValueError("yaw_deg 必须为有限数值")
    angle = float(np.deg2rad(yaw_deg))
    if abs(angle) < 1.0e-12:
        return np.asarray(wrist, dtype=np.float64)
    cos_a, sin_a = np.cos(angle), np.sin(angle)
    rotation = np.array(
        [
            [cos_a, -sin_a, 0.0],
            [sin_a, cos_a, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    half = 0.5 * angle
    yaw_quat = np.array(
        [0.0, 0.0, np.sin(half), np.cos(half)], dtype=np.float64
    )

    poses = np.asarray(wrist, dtype=np.float64)
    result = poses.copy()
    result[:, :3] = poses[:, :3] @ rotation.T
    # xyzw 序四元数左乘 q_yaw * q。
    x, y, z, w = yaw_quat
    qx, qy, qz, qw = poses[:, 3], poses[:, 4], poses[:, 5], poses[:, 6]
    result[:, 3] = w * qx + x * qw + y * qz - z * qy
    result[:, 4] = w * qy - x * qz + y * qw + z * qx
    result[:, 5] = w * qz + x * qy - y * qx + z * qw
    result[:, 6] = w * qw - x * qx - y * qy - z * qz
    return result

def _finite_pose(pose: np.ndarray, label: str) -> np.ndarray:
    values = np.asarray(pose, dtype=np.float64)
    if values.shape != (7,) or not np.isfinite(values).all():
        raise ValueError(f"{label} 必须是 7 个有限数值的 xyzw 位姿")
    quaternion_norm = float(np.linalg.norm(values[3:7]))
    if quaternion_norm < 1.0e-8:
        raise ValueError(f"{label} 四元数不能为零")
    result = values.copy()
    result[3:7] /= quaternion_norm
    return result


def compose_pose(
    parent_from_middle: np.ndarray,
    middle_from_child: np.ndarray,
) -> np.ndarray:
    """复合两个 ``[p, q_xyzw]`` 刚体位姿，返回 parent_from_child。"""
    first = _finite_pose(parent_from_middle, "parent_from_middle")
    second = _finite_pose(middle_from_child, "middle_from_child")
    first_rotation = Rotation.from_quat(first[3:7])
    position = first[:3] + first_rotation.apply(second[:3])
    orientation = (first_rotation * Rotation.from_quat(second[3:7])).as_quat()
    return np.concatenate((position, orientation))


def invert_pose(parent_from_child: np.ndarray) -> np.ndarray:
    """返回刚体位姿的逆 ``child_from_parent``。"""
    pose = _finite_pose(parent_from_child, "parent_from_child")
    inverse_rotation = Rotation.from_quat(pose[3:7]).inv()
    return np.concatenate(
        (inverse_rotation.apply(-pose[:3]), inverse_rotation.as_quat())
    )


def align_pose_to_reference(
    pose: np.ndarray,
    source_reference: np.ndarray,
    target_reference: np.ndarray,
) -> np.ndarray:
    """把 source_reference 刚性对齐到 target_reference 后变换 pose。

    ``T_aligned(t) = T_target_ref · inverse(T_source_ref) · T_source(t)``。
    H5 回放只允许 wrist→wrist 对齐：H5 第 0 帧 wrist 对齐机器人
    ``r_wrist`` Home，不允许 r_mount/marker 中心作为目标参考点。
    """
    alignment = compose_pose(
        target_reference,
        invert_pose(source_reference),
    )
    return compose_pose(alignment, pose)




@dataclass(frozen=True)
class HandTrajectorySample:
    """按录制源时间插值后的单侧手腕位姿。"""

    pose: np.ndarray
    elapsed_s: float
    source_frame_index: int
    complete: bool


class HandPoseTrajectory:
    """以有效手腕帧为关键帧，跨短暂丢帧连续插值的位置和姿态轨迹。"""

    def __init__(
        self,
        recording: MocapRecording,
        *,
        side: str = "right",
        yaw_deg: float = 0.0,
    ) -> None:
        if side not in SIDES:
            raise ValueError(f"side 必须是 {SIDES} 之一，实际 {side!r}")
        valid_indices = np.flatnonzero(recording.hands[side].valid)
        if valid_indices.size == 0:
            raise ValueError(f"录制中 {side} 手腕没有有效位姿")

        self.recording = recording
        self.side = side
        self.yaw_deg = float(yaw_deg)
        self.valid_indices = valid_indices.astype(np.int64, copy=False)
        self.start_frame_index = int(valid_indices[0])
        self.end_frame_index = int(valid_indices[-1])
        self._start_ns = int(recording.time_ns[self.start_frame_index])
        self._frame_times_s = (
            recording.time_ns.astype(np.float64) - self._start_ns
        ) / 1.0e9
        self._valid_times_s = self._frame_times_s[valid_indices]
        self._poses = apply_yaw_world(
            recording.hands[side].wrist[valid_indices],
            self.yaw_deg,
        )
        self._slerp = (
            None
            if valid_indices.size == 1
            else Slerp(
                self._valid_times_s,
                Rotation.from_quat(self._poses[:, 3:7]),
            )
        )

    @property
    def duration_s(self) -> float:
        return float(self._valid_times_s[-1])

    @property
    def interpolated_frame_count(self) -> int:
        span = self.recording.hands[self.side].valid[
            self.start_frame_index : self.end_frame_index + 1
        ]
        return int(np.count_nonzero(~span))

    def pose_at_frame(self, frame_index: int) -> np.ndarray:
        """返回原始帧时刻的连续位姿；无效帧由相邻有效帧插值。"""
        if not 0 <= frame_index < self.recording.frame_count:
            raise ValueError(
                f"frame_index={frame_index} 超出 "
                f"[0, {self.recording.frame_count})"
            )
        return self.sample(float(self._frame_times_s[frame_index])).pose

    def sample(self, elapsed_s: float) -> HandTrajectorySample:
        """按轨迹相对时间采样；区间外钳制到首末有效位姿。"""
        elapsed = float(elapsed_s)
        if not np.isfinite(elapsed):
            raise ValueError("elapsed_s 必须是有限数值")
        elapsed = float(np.clip(elapsed, 0.0, self.duration_s))
        position = np.array(
            [
                np.interp(
                    elapsed,
                    self._valid_times_s,
                    self._poses[:, axis],
                )
                for axis in range(3)
            ],
            dtype=np.float64,
        )
        quaternion = (
            self._poses[0, 3:7].copy()
            if self._slerp is None
            else self._slerp(elapsed).as_quat()
        )
        raw_index = int(
            np.searchsorted(self._frame_times_s, elapsed, side="right") - 1
        )
        raw_index = int(
            np.clip(
                raw_index,
                self.start_frame_index,
                self.end_frame_index,
            )
        )
        return HandTrajectorySample(
            pose=np.concatenate((position, quaternion)),
            elapsed_s=elapsed,
            source_frame_index=raw_index,
            complete=elapsed >= self.duration_s - 1.0e-9,
        )


def synthetic_reference_pose() -> np.ndarray:
    """完全无效侧的合成参考位姿（单位四元数、零增量）。

    回放时该侧每帧都使用参考位姿本身，增量恒为零，映射结果恒为
    机器人 Home，使缺失侧机械臂保持在安全初始位。
    """
    return _SYNTHETIC_REFERENCE_POSE.copy()
