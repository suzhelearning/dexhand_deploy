from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from pico_input.incremental_controller import IncrementalController

from ..controller_frame import ControllerFrame
from .target_conditioner import (
    ControllerTargetConditioner,
    TargetConditioningDiagnostics,
    TargetConditioningSettings,
)


@dataclass(frozen=True)
class ControllerOnlyTargets:
    """双手柄独立模式产生的左右机械臂 IK 输入。"""

    left_pose: np.ndarray
    right_pose: np.ndarray
    left_default_elbow_direction: np.ndarray
    right_default_elbow_direction: np.ndarray
    left_conditioning: TargetConditioningDiagnostics
    right_conditioning: TargetConditioningDiagnostics


class ControllerOnlyTeleopMapper:
    """不依赖 Body/Tracker 的双手柄相对末端映射。"""

    def __init__(
        self,
        config,
        *,
        rate: float = 90.0,
        min_cutoff: float = 1.0,
        beta: float = 0.7,
        conditioning_settings: TargetConditioningSettings | None = None,
        default_zsp_directions: dict[str, object] | None = None,
        input_to_robot: object | None = None,
    ):
        self._controller = IncrementalController(
            config,
            rate=rate,
            min_cutoff=min_cutoff,
            beta=beta,
            input_to_robot=input_to_robot,
        )
        self._default_elbow_directions = {
            side: self._unit_direction(
                (
                    default_zsp_directions[side]
                    if default_zsp_directions is not None
                    else config.get_default_zsp_direction(side)
                ),
                side,
            )
            for side in ("left", "right")
        }
        if conditioning_settings is None:
            conditioning_settings = TargetConditioningSettings(
                rate_hz=rate,
                translation_gain=np.ones(3),
                rotation_gain=1.0,
                workspace_relative_radii_m=np.full(3, 10.0),
                workspace_soft_zone_ratio=0.99,
                maximum_linear_speed_m_s=100.0,
                maximum_angular_speed_rad_s=100.0,
                maximum_linear_acceleration_m_s2=10000.0,
                maximum_angular_acceleration_rad_s2=10000.0,
            )
        self._conditioners = {
            side: ControllerTargetConditioner(
                config.init_pos[side],
                config.init_quat[side],
                conditioning_settings,
            )
            for side in ("left", "right")
        }

    def initialize(self, frame: ControllerFrame) -> set[str]:
        """记录按下 A 时的左右手柄位姿作为相对运动零点。"""
        for conditioner in self._conditioners.values():
            conditioner.reset()
        return self._controller.initialize(frame.virtual_trackers())

    def map_absolute_poses(
        self, left_pose: np.ndarray, right_pose: np.ndarray,
    ) -> ControllerOnlyTargets:
        """直接整形 chest 系绝对 TCP 位姿，不经过手柄 Home 相对映射。"""
        poses = {}
        conditioning = {}
        for side, pose in (("left", left_pose), ("right", right_pose)):
            values = np.asarray(pose, dtype=np.float64)
            if values.shape != (7,) or not np.isfinite(values).all():
                raise ValueError(f"{side} absolute pose 必须是有限 7 向量")
            position, quaternion, diagnostics = self._conditioners[
                side
            ].condition(values[:3], values[3:])
            poses[side] = np.concatenate((position, quaternion))
            conditioning[side] = diagnostics
        return ControllerOnlyTargets(
            left_pose=poses["left"],
            right_pose=poses["right"],
            left_default_elbow_direction=(
                self._default_elbow_directions["left"].copy()
            ),
            right_default_elbow_direction=(
                self._default_elbow_directions["right"].copy()
            ),
            left_conditioning=conditioning["left"],
            right_conditioning=conditioning["right"],
        )

    def map_frame(self, frame: ControllerFrame) -> ControllerOnlyTargets:
        virtual_trackers = frame.virtual_trackers()
        poses = {}
        conditioning = {}
        for side, role in (
            ("left", "pico_left_wrist"),
            ("right", "pico_right_wrist"),
        ):
            position, quaternion = self._controller.compute_target_pose(
                virtual_trackers[role],
                role,
            )
            if position is None or quaternion is None:
                raise RuntimeError(
                    f"{side} controller-only mapper is not initialized"
                )
            position, quaternion, diagnostics = self._conditioners[
                side
            ].condition(position, quaternion)
            poses[side] = np.concatenate((position, quaternion))
            conditioning[side] = diagnostics

        return ControllerOnlyTargets(
            left_pose=poses["left"],
            right_pose=poses["right"],
            left_default_elbow_direction=(
                self._default_elbow_directions["left"].copy()
            ),
            right_default_elbow_direction=(
                self._default_elbow_directions["right"].copy()
            ),
            left_conditioning=conditioning["left"],
            right_conditioning=conditioning["right"],
        )

    @staticmethod
    def _unit_direction(values, side: str) -> np.ndarray:
        direction = np.asarray(values, dtype=np.float64)
        if direction.shape != (3,) or not np.isfinite(direction).all():
            raise ValueError(
                f"{side} default elbow direction must be a finite 3-vector"
            )
        norm = float(np.linalg.norm(direction))
        if norm < 1.0e-8:
            raise ValueError(
                f"{side} default elbow direction must be nonzero"
            )
        return direction / norm
