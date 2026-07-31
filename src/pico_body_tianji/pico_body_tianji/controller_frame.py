from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ControllerFrame:
    """一帧 PICO 左右手柄位姿，不包含任何 SMPL/Body 数据。"""

    left_pose: np.ndarray
    right_pose: np.ndarray

    @classmethod
    def from_poses(cls, left_pose, right_pose) -> "ControllerFrame":
        return cls(
            left_pose=cls._validated_pose(left_pose, "left"),
            right_pose=cls._validated_pose(right_pose, "right"),
        )

    def virtual_trackers(self) -> dict[str, np.ndarray]:
        """复用官方增量控制器的 wrist 角色命名。"""
        return {
            "pico_left_wrist": self.left_pose.copy(),
            "pico_right_wrist": self.right_pose.copy(),
        }

    def signature(self) -> bytes:
        return self.left_pose.tobytes() + self.right_pose.tobytes()

    @staticmethod
    def _validated_pose(pose, side: str) -> np.ndarray:
        values = np.asarray(pose, dtype=np.float64)
        if values.shape != (7,) or not np.isfinite(values).all():
            raise ValueError(
                f"{side} controller pose must be a finite 7-vector"
            )
        quaternion_norm = float(np.linalg.norm(values[3:7]))
        if not 0.95 <= quaternion_norm <= 1.05:
            raise ValueError(
                f"{side} controller quaternion is unavailable or invalid"
            )
        normalized = values.copy()
        normalized[3:7] /= quaternion_norm
        return normalized
