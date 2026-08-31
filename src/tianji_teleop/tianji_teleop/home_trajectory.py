from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TrajectorySample:
    joints: np.ndarray
    complete: bool


class HomeTrajectory:
    """零端速 smoothstep 关节回位轨迹，单位为度和秒。"""

    def __init__(
        self,
        *,
        start_joints: np.ndarray,
        home_joints: np.ndarray,
        start_time: float,
        minimum_duration: float,
        max_speed_deg_s: float,
    ):
        self.start_joints = np.asarray(start_joints, dtype=np.float64)
        self.home_joints = np.asarray(home_joints, dtype=np.float64)
        if self.start_joints.shape != self.home_joints.shape:
            raise ValueError("start_joints and home_joints must have equal shape")
        if max_speed_deg_s <= 0.0 or minimum_duration <= 0.0:
            raise ValueError("trajectory duration and speed must be positive")

        self.start_time = float(start_time)
        max_delta = float(
            np.max(np.abs(self.home_joints - self.start_joints), initial=0.0)
        )
        # smoothstep 的 ds/dt 峰值为 1.5 / duration。
        speed_limited_duration = 1.5 * max_delta / float(max_speed_deg_s)
        self.duration = max(float(minimum_duration), speed_limited_duration)

    def sample(self, now: float) -> TrajectorySample:
        progress = (float(now) - self.start_time) / self.duration
        progress = float(np.clip(progress, 0.0, 1.0))
        blend = progress * progress * (3.0 - 2.0 * progress)
        joints = (
            self.start_joints
            + blend * (self.home_joints - self.start_joints)
        )
        return TrajectorySample(joints=joints, complete=progress >= 1.0)
