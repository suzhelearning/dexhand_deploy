from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from pico_input.incremental_controller import IncrementalController

from .body_frame import BodyFrame
from ..controller_frame import ControllerFrame
from .elbow_retargeting import (
    ArmAngleConstraintResult,
    SMPLArmAngleConstraint,
)
from .robot_geometry import (
    MARVIN_SHOULDER_CENTER_IN_STAND_M,
    MARVIN_SHOULDER_ORIGIN_M,
)


@dataclass(frozen=True)
class ControllerTargets:
    left_pose: np.ndarray
    right_pose: np.ndarray
    left_elbow_direction: np.ndarray
    right_elbow_direction: np.ndarray
    left_body_keypoints: np.ndarray
    right_body_keypoints: np.ndarray
    smpl_skeleton_keypoints: np.ndarray
    controller_positions: np.ndarray
    left_arm_angle: ArmAngleConstraintResult
    right_arm_angle: ArmAngleConstraintResult
    elbow_constraint_source: str


class ControllerTeleopMapper:
    """PICO 双手柄相对位姿到 Marvin 安全初始末端的映射。"""

    def __init__(
        self,
        config,
        *,
        rate: float = 90.0,
        min_cutoff: float = 1.0,
        beta: float = 0.7,
        elbow_min_cutoff: float = 0.3,
    ):
        self._controller = IncrementalController(
            config,
            rate=rate,
            min_cutoff=min_cutoff,
            beta=beta,
            elbow_min_cutoff=elbow_min_cutoff,
        )
        self._pico_to_robot = np.asarray(
            config.pico_to_robot, dtype=np.float64
        )
        self._body_to_robot_chest = {
            side: np.asarray(
                config.get_world_to_chest_rotation(side),
                dtype=np.float64,
            )
            @ self._pico_to_robot
            for side in ("left", "right")
        }
        self._arm_angle_constraint = SMPLArmAngleConstraint(
            {
                side: np.asarray(
                    config.get_default_zsp_direction(side),
                    dtype=np.float64,
                )
                for side in ("left", "right")
            },
            maximum_deviation_deg=179.0,
            maximum_step_deg=180.0,
        )

    def initialize(
        self,
        frame: ControllerFrame,
        body_frame: BodyFrame,
    ) -> set[str]:
        self._arm_angle_constraint.reset()
        torso_frame = self._controller_frame_in_torso(frame, body_frame)
        return self._controller.initialize(torso_frame.virtual_trackers())

    def map_frame(
        self,
        frame: ControllerFrame,
        body_frame: BodyFrame | None,
    ) -> ControllerTargets:
        if body_frame is None:
            raise ValueError(
                "live SMPL torso frame is required for controller mapping"
            )
        torso_frame = self._controller_frame_in_torso(frame, body_frame)
        virtual_trackers = torso_frame.virtual_trackers()
        poses = {}
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
                    f"{side} controller mapper is not initialized"
                )
            poses[side] = np.concatenate((position, quaternion))

        body_points = {
            side: self._keypoints_in_chest(body_frame, side)
            for side in ("left", "right")
        }
        arm_angles = {}
        for side in ("left", "right"):
            arm_angles[side] = self._arm_angle_constraint.constrain(
                side,
                body_points[side],
                target_wrist_position=poses[side][:3],
            )

        return ControllerTargets(
            left_pose=poses["left"],
            right_pose=poses["right"],
            left_elbow_direction=arm_angles["left"].ik_direction,
            right_elbow_direction=arm_angles["right"].ik_direction,
            left_body_keypoints=body_points["left"],
            right_body_keypoints=body_points["right"],
            smpl_skeleton_keypoints=self.map_skeleton(body_frame),
            controller_positions=self.map_controller_positions(
                frame,
                body_frame,
            ),
            left_arm_angle=arm_angles["left"],
            right_arm_angle=arm_angles["right"],
            elbow_constraint_source="smpl_arm_angle_target_axis",
        )

    def map_skeleton(self, body_frame: BodyFrame) -> np.ndarray:
        """把实时胸廓下的 SMPL 骨架放到机器人 Stand 坐标系。"""
        points = body_frame.skeleton_keypoints()
        return (
            self._pico_to_robot @ np.asarray(points, dtype=np.float64).T
        ).T + MARVIN_SHOULDER_CENTER_IN_STAND_M

    def map_controller_positions(
        self,
        frame: ControllerFrame,
        body_frame: BodyFrame,
    ) -> np.ndarray:
        """把左右手柄原点叠加到与 SMPL 骨架相同的 Stand 坐标系。"""
        torso_frame = self._controller_frame_in_torso(frame, body_frame)
        points = np.stack(
            (torso_frame.left_pose[:3], torso_frame.right_pose[:3])
        )
        return (
            self._pico_to_robot @ points.T
        ).T + MARVIN_SHOULDER_CENTER_IN_STAND_M

    def _controller_frame_in_torso(
        self,
        frame: ControllerFrame,
        body_frame: BodyFrame,
    ) -> ControllerFrame:
        return ControllerFrame.from_poses(
            body_frame.pose_in_torso_frame(frame.left_pose),
            body_frame.pose_in_torso_frame(frame.right_pose),
        )

    def _keypoints_in_chest(
        self,
        body_frame: BodyFrame,
        side: str,
    ) -> np.ndarray:
        points = body_frame.arm_keypoints(side)
        return (
            self._body_to_robot_chest[side]
            @ np.asarray(points, dtype=np.float64).T
        ).T + MARVIN_SHOULDER_ORIGIN_M
