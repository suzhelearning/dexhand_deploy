from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np


def _unit(vector: np.ndarray) -> np.ndarray:
    values = np.asarray(vector, dtype=np.float64)
    if values.shape != (3,) or not np.isfinite(values).all():
        raise ValueError("elbow direction must be a finite 3-vector")
    norm = float(np.linalg.norm(values))
    if norm < 1e-9:
        raise ValueError("elbow direction must be non-zero")
    return values / norm


def _orthogonal_axis(vector: np.ndarray) -> np.ndarray:
    basis = (
        np.array([1.0, 0.0, 0.0])
        if abs(float(vector[0])) < 0.8
        else np.array([0.0, 1.0, 0.0])
    )
    axis = np.cross(vector, basis)
    return axis / np.linalg.norm(axis)


def _slerp(
    start: np.ndarray, target: np.ndarray, fraction: float
) -> np.ndarray:
    start = _unit(start)
    target = _unit(target)
    cosine = float(np.clip(np.dot(start, target), -1.0, 1.0))
    angle = float(np.arccos(cosine))
    if angle < 1e-9:
        return start.copy()
    if np.pi - angle < 1e-9:
        axis = _orthogonal_axis(start)
        partial = np.pi * float(fraction)
        return _unit(
            start * np.cos(partial)
            + np.cross(axis, start) * np.sin(partial)
        )
    scale = np.sin(angle)
    result = (
        np.sin((1.0 - fraction) * angle) / scale * start
        + np.sin(fraction * angle) / scale * target
    )
    return _unit(result)


def _limit_angle(
    start: np.ndarray,
    target: np.ndarray,
    maximum_angle_deg: float,
) -> np.ndarray:
    start = _unit(start)
    target = _unit(target)
    cosine = float(np.clip(np.dot(start, target), -1.0, 1.0))
    angle = float(np.arccos(cosine))
    maximum_angle = float(np.deg2rad(maximum_angle_deg))
    if angle <= maximum_angle:
        return target.copy()
    return _slerp(start, target, maximum_angle / angle)


class AbsoluteElbowDirectionLimiter:
    """限制绝对人体肘平面，但不把启动姿态重新标定为零变化。"""

    def __init__(
        self,
        default_directions: dict[str, np.ndarray],
        *,
        maximum_deviation_deg: float,
        maximum_step_deg: float,
    ):
        if maximum_deviation_deg <= 0.0:
            raise ValueError("maximum_deviation_deg must be positive")
        if maximum_step_deg <= 0.0:
            raise ValueError("maximum_step_deg must be positive")
        self._defaults = {
            side: _unit(direction)
            for side, direction in default_directions.items()
        }
        self._maximum_deviation_deg = float(maximum_deviation_deg)
        self._maximum_step_deg = float(maximum_step_deg)
        self._previous: dict[str, np.ndarray] = {}

    def reset(self) -> None:
        self._previous.clear()

    def limit(self, side: str, measured_direction: np.ndarray) -> np.ndarray:
        if side not in self._defaults:
            raise ValueError(f"missing default elbow direction for {side}")
        default = self._defaults[side]
        desired = _limit_angle(
            default,
            measured_direction,
            self._maximum_deviation_deg,
        )
        previous = self._previous.get(side, default)
        output = _limit_angle(
            previous,
            desired,
            self._maximum_step_deg,
        )
        self._previous[side] = output
        return output.copy()


@dataclass(frozen=True)
class ArmAngleConstraintResult:
    """一帧 SMPL 肩—肘—腕几何对应的连续臂角约束结果。"""

    ik_direction: np.ndarray
    physical_direction: np.ndarray
    projection_point: np.ndarray
    shoulder_wrist_axis: np.ndarray
    measured_angle_deg: float | None
    constrained_angle_deg: float
    source: str


class SMPLArmAngleConstraint:
    """在肩—腕轴的正交平面内约束 SMPL 臂角。

    libKine 的 ``zsp_para`` 使用与物理肘偏移相反的参考平面方向。
    因此本类先由 SMPL 肩/肘/腕计算物理肘平面，再取反得到 IK
    方向。绝对范围和逐帧变化都约束在同一个一维有符号臂角上，
    输出始终与肩—腕轴正交。
    """

    _WRIST_AXIS_EPSILON_M = 1e-6
    _ELBOW_OFFSET_EPSILON_M = 0.015
    _DIRECTION_EPSILON = 1e-9

    def __init__(
        self,
        default_directions: dict[str, np.ndarray],
        *,
        maximum_deviation_deg: float,
        maximum_step_deg: float,
        angle_smoother: Callable[[str, float], float] | None = None,
    ):
        if maximum_deviation_deg <= 0.0:
            raise ValueError("maximum_deviation_deg must be positive")
        if maximum_deviation_deg >= 180.0:
            raise ValueError("maximum_deviation_deg must be below 180")
        if maximum_step_deg <= 0.0:
            raise ValueError("maximum_step_deg must be positive")
        self._defaults = {
            side: _unit(direction)
            for side, direction in default_directions.items()
        }
        self._maximum_deviation_deg = float(maximum_deviation_deg)
        self._maximum_step_deg = float(maximum_step_deg)
        self._angle_smoother = angle_smoother
        self._previous_angles: dict[str, float] = {}
        self._previous_measured_angles: dict[str, float] = {}
        self._previous_axes: dict[str, np.ndarray] = {}

    def reset(self) -> None:
        self._previous_angles.clear()
        self._previous_measured_angles.clear()
        self._previous_axes.clear()

    def constrain(
        self,
        side: str,
        keypoints: np.ndarray,
        *,
        target_wrist_position: np.ndarray | None = None,
    ) -> ArmAngleConstraintResult:
        if side not in self._defaults:
            raise ValueError(f"missing default elbow direction for {side}")
        points = np.asarray(keypoints, dtype=np.float64)
        if points.shape != (3, 3) or not np.isfinite(points).all():
            raise ValueError(
                "SMPL arm keypoints must be a finite 3x3 array"
            )

        shoulder, elbow, wrist = points
        shoulder_to_wrist = wrist - shoulder
        wrist_distance = float(np.linalg.norm(shoulder_to_wrist))
        if wrist_distance < self._WRIST_AXIS_EPSILON_M:
            if target_wrist_position is None:
                output_axis = self._previous_axes.get(side)
                if output_axis is None:
                    output_axis = _orthogonal_axis(self._defaults[side])
                output_projection = shoulder.copy()
            else:
                output_axis, output_projection = self._output_geometry(
                    shoulder,
                    wrist,
                    target_wrist_position,
                    projection_fraction=0.5,
                )
            return self._held_result(
                side,
                output_axis,
                output_projection,
                source="smpl_wrist_singularity_hold",
            )

        axis = shoulder_to_wrist / wrist_distance
        self._previous_axes[side] = axis.copy()
        shoulder_to_elbow = elbow - shoulder
        projection_distance = float(np.dot(shoulder_to_elbow, axis))
        projection_point = shoulder + projection_distance * axis
        output_axis, output_projection = self._output_geometry(
            shoulder,
            wrist,
            target_wrist_position,
            projection_fraction=projection_distance / wrist_distance,
        )
        elbow_offset = elbow - projection_point
        elbow_offset_length = float(np.linalg.norm(elbow_offset))
        if elbow_offset_length < self._ELBOW_OFFSET_EPSILON_M:
            return self._held_result(
                side,
                output_axis,
                output_projection,
                source="smpl_singularity_hold",
            )

        measured_ik = -elbow_offset / elbow_offset_length
        reference = self._reference_direction(side, axis)
        measured_angle_deg = self._signed_angle_deg(
            reference,
            measured_ik,
            axis,
        )
        measured_angle_deg = self._unwrap_measured_angle(
            side, measured_angle_deg
        )
        smoothed_angle_deg = measured_angle_deg
        if self._angle_smoother is not None:
            smoothed_angle_deg = float(
                self._angle_smoother(side, measured_angle_deg)
            )
            if not np.isfinite(smoothed_angle_deg):
                raise ValueError("smoothed arm angle must be finite")
        desired_angle_deg = float(
            np.clip(
                smoothed_angle_deg,
                -self._maximum_deviation_deg,
                self._maximum_deviation_deg,
            )
        )
        previous_angle_deg = self._previous_angles.get(side, 0.0)
        angle_step_deg = desired_angle_deg - previous_angle_deg
        constrained_step_deg = float(
            np.clip(
                angle_step_deg,
                -self._maximum_step_deg,
                self._maximum_step_deg,
            )
        )
        constrained_angle_deg = float(
            np.clip(
                previous_angle_deg + constrained_step_deg,
                -self._maximum_deviation_deg,
                self._maximum_deviation_deg,
            )
        )
        limited = (
            not np.isclose(desired_angle_deg, smoothed_angle_deg)
            or not np.isclose(constrained_step_deg, angle_step_deg)
        )
        if limited:
            source = "smpl_arm_angle_limited"
        elif not np.isclose(smoothed_angle_deg, measured_angle_deg):
            source = "smpl_arm_angle_smoothed"
        else:
            source = "smpl_arm_angle"
        return self._result(
            side,
            self._reference_direction(side, output_axis),
            output_axis,
            output_projection,
            measured_angle_deg=measured_angle_deg,
            constrained_angle_deg=constrained_angle_deg,
            source=source,
        )

    def _held_result(
        self,
        side: str,
        axis: np.ndarray,
        projection_point: np.ndarray,
        *,
        source: str,
    ) -> ArmAngleConstraintResult:
        reference = self._reference_direction(side, axis)
        return self._result(
            side,
            reference,
            axis,
            projection_point,
            measured_angle_deg=None,
            constrained_angle_deg=self._previous_angles.get(side, 0.0),
            source=source,
        )

    def _result(
        self,
        side: str,
        reference: np.ndarray,
        axis: np.ndarray,
        projection_point: np.ndarray,
        *,
        measured_angle_deg: float | None,
        constrained_angle_deg: float,
        source: str,
    ) -> ArmAngleConstraintResult:
        ik_direction = self._rotate_about_axis(
            reference,
            axis,
            np.deg2rad(constrained_angle_deg),
        )
        ik_direction = _unit(
            ik_direction - float(np.dot(ik_direction, axis)) * axis
        )
        self._previous_angles[side] = float(constrained_angle_deg)
        return ArmAngleConstraintResult(
            ik_direction=ik_direction.copy(),
            physical_direction=-ik_direction.copy(),
            projection_point=np.asarray(
                projection_point, dtype=np.float64
            ).copy(),
            shoulder_wrist_axis=np.asarray(
                axis, dtype=np.float64
            ).copy(),
            measured_angle_deg=measured_angle_deg,
            constrained_angle_deg=float(constrained_angle_deg),
            source=source,
        )

    def _reference_direction(
        self,
        side: str,
        axis: np.ndarray,
    ) -> np.ndarray:
        default = self._defaults[side]
        projected = default - float(np.dot(default, axis)) * axis
        if float(np.linalg.norm(projected)) >= self._DIRECTION_EPSILON:
            return _unit(projected)

        return _orthogonal_axis(axis)

    def _output_geometry(
        self,
        shoulder: np.ndarray,
        source_wrist: np.ndarray,
        target_wrist_position: np.ndarray | None,
        *,
        projection_fraction: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        target_wrist = (
            np.asarray(source_wrist, dtype=np.float64)
            if target_wrist_position is None
            else np.asarray(target_wrist_position, dtype=np.float64)
        )
        if (
            target_wrist.shape != (3,)
            or not np.isfinite(target_wrist).all()
        ):
            raise ValueError(
                "target wrist position must be a finite 3-vector"
            )
        shoulder_to_target = target_wrist - shoulder
        target_distance = float(np.linalg.norm(shoulder_to_target))
        if target_distance < self._WRIST_AXIS_EPSILON_M:
            raise ValueError("target wrist axis must be non-zero")
        output_axis = shoulder_to_target / target_distance
        fraction = float(np.clip(projection_fraction, 0.0, 1.0))
        output_projection = shoulder + fraction * shoulder_to_target
        return output_axis, output_projection

    def _unwrap_measured_angle(
        self,
        side: str,
        measured_angle_deg: float,
    ) -> float:
        previous = self._previous_measured_angles.get(side)
        if previous is None:
            unwrapped = float(measured_angle_deg)
        else:
            delta = (
                float(measured_angle_deg) - previous + 180.0
            ) % 360.0 - 180.0
            unwrapped = previous + delta
        self._previous_measured_angles[side] = unwrapped
        return unwrapped

    @staticmethod
    def _signed_angle_deg(
        reference: np.ndarray,
        direction: np.ndarray,
        axis: np.ndarray,
    ) -> float:
        sine = float(np.dot(axis, np.cross(reference, direction)))
        cosine = float(
            np.clip(np.dot(reference, direction), -1.0, 1.0)
        )
        return float(np.degrees(np.arctan2(sine, cosine)))

    @staticmethod
    def _rotate_about_axis(
        vector: np.ndarray,
        axis: np.ndarray,
        angle_rad: float,
    ) -> np.ndarray:
        cosine = float(np.cos(angle_rad))
        sine = float(np.sin(angle_rad))
        return (
            vector * cosine
            + np.cross(axis, vector) * sine
            + axis * float(np.dot(axis, vector)) * (1.0 - cosine)
        )
