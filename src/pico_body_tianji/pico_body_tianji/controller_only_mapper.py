from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from pico_input.incremental_controller import IncrementalController

from .controller_frame import ControllerFrame


@dataclass(frozen=True)
class ControllerOnlyTargets:
    """双手柄独立模式产生的左右机械臂 IK 输入。"""

    left_pose: np.ndarray
    right_pose: np.ndarray
    left_default_elbow_direction: np.ndarray
    right_default_elbow_direction: np.ndarray


class ControllerOnlyTeleopMapper:
    """不依赖 Body/Tracker 的双手柄相对末端映射。"""

    def __init__(
        self,
        config,
        *,
        rate: float = 90.0,
        min_cutoff: float = 1.0,
        beta: float = 0.7,
    ):
        self._controller = IncrementalController(
            config,
            rate=rate,
            min_cutoff=min_cutoff,
            beta=beta,
        )
        self._default_elbow_directions = {
            side: self._unit_direction(
                config.get_default_zsp_direction(side),
                side,
            )
            for side in ("left", "right")
        }

    def initialize(self, frame: ControllerFrame) -> set[str]:
        """记录按下 A 时的左右手柄位姿作为相对运动零点。"""
        return self._controller.initialize(frame.virtual_trackers())

    def map_frame(self, frame: ControllerFrame) -> ControllerOnlyTargets:
        virtual_trackers = frame.virtual_trackers()
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
                    f"{side} controller-only mapper is not initialized"
                )
            poses[side] = np.concatenate((position, quaternion))

        return ControllerOnlyTargets(
            left_pose=poses["left"],
            right_pose=poses["right"],
            left_default_elbow_direction=(
                self._default_elbow_directions["left"].copy()
            ),
            right_default_elbow_direction=(
                self._default_elbow_directions["right"].copy()
            ),
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
