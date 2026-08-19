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
  （y-up 右手系、米制）：
  - ``hands/<side>/wrist_position``        (N,3) float32
  - ``hands/<side>/wrist_quaternion_xyzw``  (N,4) float32（xyzw 序）
- 单侧可能完全没有跟踪（如 take003 左手全 NaN）：本模块将其视为
  该侧全部无效，由回放节点按“保持 Home”处理。

安全防护：拒绝包含外部/软链接的 HDF5（与采集端 replay_hdf5.py
一致），防止恶意文件经 HDF5 外部链接读取任意本地文件。

坐标约定：Motive 系为 y-up，与 PICO 手柄的 y-up 世界系同族；
``apply_yaw_world`` 绕 Motive +Y（竖直轴）旋转整个轨迹，等价于在
``pico_to_robot`` 映射后绕机器人世界 Z（竖直轴）旋转，用于标定
录制时人的朝向与机器人正前方的夹角。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import h5py
import numpy as np

SUPPORTED_H5_VERSION = "4.0"
SUPPORTED_SCHEMA_LAYOUT = "compact-aligned-60hz-v1"
SIDES = ("left", "right")

_QUAT_NORM_TOLERANCE = 0.05
_SYNTHETIC_REFERENCE_POSE = np.array(
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float64
)


@dataclass(frozen=True)
class HandRecording:
    """单侧手腕轨迹（Motive 系，y-up，米制）。"""

    valid: np.ndarray  # (N,) bool；False 表示该帧手腕缺失
    wrist: np.ndarray  # (N,7) float64 [x,y,z,qx,qy,qz,qw]


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
                "wrist_position",
                "wrist_quaternion_xyzw",
                "valid",
            )
            for dataset in required:
                if dataset not in group:
                    raise ValueError(
                        f"{group_name} 缺少数据集 {dataset}"
                    )
            position = np.asarray(
                group["wrist_position"][:], dtype=np.float64
            )
            quaternion = np.asarray(
                group["wrist_quaternion_xyzw"][:], dtype=np.float64
            )
            flagged = np.asarray(group["valid"][:], dtype=bool)
            expected = (time_ns.size,)
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
            numerically_ok = (
                np.isfinite(position).all(axis=1)
                & np.isfinite(quaternion).all(axis=1)
                & (quaternion_norm > 1.0 - _QUAT_NORM_TOLERANCE)
                & (quaternion_norm < 1.0 + _QUAT_NORM_TOLERANCE)
            )
            valid = flagged & numerically_ok
            wrist = np.concatenate((position, quaternion), axis=1)
            hands[side] = HandRecording(valid=valid, wrist=wrist)

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
    """把整条手腕轨迹绕 Motive 竖直轴（+Y）旋转 yaw_deg 度。

    位置与姿态一起旋转：p' = Ry(θ)·p，q' = q_yaw ⊗ q（xyzw 序左乘）。
    经回放节点的 ``pico_to_robot`` 映射后，等价于绕机器人世界 Z 轴
    旋转，用于对齐录制时人的朝向与机器人正前方。θ=0 时原样返回。
    """
    if not np.isfinite(yaw_deg):
        raise ValueError("yaw_deg 必须为有限数值")
    angle = float(np.deg2rad(yaw_deg))
    if abs(angle) < 1.0e-12:
        return np.asarray(wrist, dtype=np.float64)
    cos_a, sin_a = np.cos(angle), np.sin(angle)
    rotation = np.array(
        [
            [cos_a, 0.0, sin_a],
            [0.0, 1.0, 0.0],
            [-sin_a, 0.0, cos_a],
        ],
        dtype=np.float64,
    )
    half = 0.5 * angle
    yaw_quat = np.array(
        [0.0, np.sin(half), 0.0, np.cos(half)], dtype=np.float64
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


def synthetic_reference_pose() -> np.ndarray:
    """完全无效侧的合成参考位姿（单位四元数、零增量）。

    回放时该侧每帧都使用参考位姿本身，增量恒为零，映射结果恒为
    机器人 Home，使缺失侧机械臂保持在安全初始位。
    """
    return _SYNTHETIC_REFERENCE_POSE.copy()
