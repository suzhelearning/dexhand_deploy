from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class WristPoseFrame:
    """A validated pair of left/right wrist poses used by target mappers."""

    left_pose: np.ndarray
    right_pose: np.ndarray

    @classmethod
    def from_poses(cls, left_pose, right_pose) -> "WristPoseFrame":
        return cls(
            left_pose=cls._validated_pose(left_pose, "left"),
            right_pose=cls._validated_pose(right_pose, "right"),
        )

    @staticmethod
    def _validated_pose(pose, side: str) -> np.ndarray:
        values = np.asarray(pose, dtype=np.float64)
        if values.shape != (7,) or not np.isfinite(values).all():
            raise ValueError(f"{side} wrist pose must be a finite 7-vector")
        quaternion_norm = float(np.linalg.norm(values[3:7]))
        if not 0.95 <= quaternion_norm <= 1.05:
            raise ValueError(f"{side} wrist quaternion is unavailable or invalid")
        normalized = values.copy()
        normalized[3:7] /= quaternion_norm
        return normalized
