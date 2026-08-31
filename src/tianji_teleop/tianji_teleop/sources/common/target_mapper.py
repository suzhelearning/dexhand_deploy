from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from tianji_world_output.transform_utils import (
    apply_world_rotation_to_chest_pose,
    transform_world_to_chest,
)

from .wrist_pose_frame import WristPoseFrame
from .target_conditioner import (
    TargetConditioningDiagnostics,
    TargetConditioningSettings,
    TargetConditioner,
)


@dataclass(frozen=True)
class ArmTargetBatch:
    """左右腕位姿产生的 canonical 机械臂目标。"""

    left_pose: np.ndarray
    right_pose: np.ndarray
    left_default_elbow_direction: np.ndarray
    right_default_elbow_direction: np.ndarray
    left_conditioning: TargetConditioningDiagnostics
    right_conditioning: TargetConditioningDiagnostics


class EndEffectorTargetMapper:
    """将双腕位姿映射为相对末端目标。"""

    def __init__(
        self,
        config,
        *,
        rate: float = 90.0,
        conditioning_settings: TargetConditioningSettings | None = None,
        default_zsp_directions: dict[str, object] | None = None,
        input_to_robot: object | None = None,
    ):
        matrix = np.asarray(
            config.mocap_to_robot if input_to_robot is None else input_to_robot,
            dtype=np.float64,
        )
        if (
            matrix.shape != (3, 3)
            or not np.isfinite(matrix).all()
            or not np.allclose(matrix @ matrix.T, np.eye(3), atol=1.0e-6)
            or not np.isclose(np.linalg.det(matrix), 1.0)
        ):
            raise ValueError("input_to_robot must be a finite proper rotation matrix")
        self._input_to_robot = matrix
        self._initial_wrist_poses: dict[str, np.ndarray] | None = None
        self._home_positions = {
            side: np.asarray(config.init_pos[side], dtype=np.float64)
            for side in ("left", "right")
        }
        self._home_rotations = {
            side: Rotation.from_quat(config.init_quat[side])
            for side in ("left", "right")
        }
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
            side: TargetConditioner(
                config.init_pos[side],
                config.init_quat[side],
                conditioning_settings,
            )
            for side in ("left", "right")
        }

    def initialize(self, frame: WristPoseFrame) -> set[str]:
        """记录左右腕位姿作为相对运动零点。"""
        for conditioner in self._conditioners.values():
            conditioner.reset()
        self._initial_wrist_poses = {
            "left": frame.left_pose.copy(),
            "right": frame.right_pose.copy(),
        }
        return {"left_wrist", "right_wrist"}

    def map_absolute_tcp_poses(
        self, left_pose: np.ndarray, right_pose: np.ndarray,
    ) -> ArmTargetBatch:
        """直接整形 Base 系绝对 TCP 位姿，不经过腕部 Home 相对映射。"""
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
        return ArmTargetBatch(
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

    def map_relative_wrist_frame(self, frame: WristPoseFrame) -> ArmTargetBatch:
        if self._initial_wrist_poses is None:
            raise RuntimeError("relative wrist mapper is not initialized")
        poses = {}
        conditioning = {}
        for side, current_pose in (
            ("left", frame.left_pose),
            ("right", frame.right_pose),
        ):
            initial_pose = self._initial_wrist_poses[side]
            position = self._home_positions[side] + transform_world_to_chest(
                self._input_to_robot @ (current_pose[:3] - initial_pose[:3]),
                side,
            )
            delta_input = (
                Rotation.from_quat(current_pose[3:])
                * Rotation.from_quat(initial_pose[3:]).inv()
            )
            delta_world = Rotation.from_matrix(
                self._input_to_robot
                @ delta_input.as_matrix()
                @ self._input_to_robot.T
            )
            quaternion = Rotation.from_matrix(
                apply_world_rotation_to_chest_pose(
                    self._home_rotations[side].as_matrix(),
                    delta_world,
                    side,
                )
            ).as_quat()
            position, quaternion, diagnostics = self._conditioners[
                side
            ].condition(position, quaternion)
            poses[side] = np.concatenate((position, quaternion))
            conditioning[side] = diagnostics

        return ArmTargetBatch(
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
