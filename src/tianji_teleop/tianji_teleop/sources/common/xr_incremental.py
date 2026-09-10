"""Reference-faithful incremental pose mapping for the XR/Manus route."""
from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from ...hand_tracking.models import ArmInputObservation
from ...hand_tracking.reference_xr.config_loader import TianjiConfig
from ...hand_tracking.reference_xr.incremental_controller import IncrementalController
from .pose_mapping import MappedArmPose


SIDES = ("left", "right")
_IDENTITY_POSE = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float64)


def _value(config: Any, name: str, default: Any = None) -> Any:
    if isinstance(config, Mapping):
        return config.get(name, default)
    return getattr(config, name, default)


def _side_value(config: Any, name: str, side: str, default: Any) -> Any:
    value = _value(config, name, None)
    if value is None:
        return default
    if isinstance(value, Mapping):
        return value.get(side, default)
    return value


def _pose(value: Any, field: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (7,) or not np.isfinite(result).all():
        raise ValueError(f"{field} must be a finite 7-vector")
    norm = float(np.linalg.norm(result[3:]))
    if norm < 1.0e-12:
        raise ValueError(f"{field} quaternion must be non-zero")
    result[3:] /= norm
    return result


def _compose(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    first_rotation = Rotation.from_quat(first[3:])
    second_rotation = Rotation.from_quat(second[3:])
    position = first[:3] + first_rotation.apply(second[:3])
    return np.concatenate((position, (first_rotation * second_rotation).as_quat()))


class XrIncrementalMapper:
    """Map XR poses with the migrated ``IncrementalController`` semantics.

    ``ArmInputObservation.pose`` remains the raw controller or wrist-tracker
    pose in the XR tracking frame.  At start, each side is anchored to the
    current observation.  Position and orientation deltas then follow the
    reference PICO->robot and world->chest transforms.  Clutch release rebases
    the raw input onto the last output so it cannot create a target jump.
    """

    def __init__(self, config: Any):
        default_config = Path(__file__).parents[2] / "hand_tracking" / "reference_xr" / "config" / "tianji_robot.yaml"
        config_path = Path(_value(config, "reference_config_path", default_config)).expanduser()
        self._reference_config = TianjiConfig.load(str(config_path), use_ros=False)
        self._rate = float(_value(config, "rate_hz", 90.0))
        self._min_cutoff = float(_value(config, "min_cutoff", 1.0))
        self._beta = float(_value(config, "beta", 0.7))
        self._elbow_min_cutoff = float(_value(config, "elbow_min_cutoff", 0.3))
        self._dynamic_elbow = _value(config, "dynamic_elbow_direction", False)
        self._require_elbow = _value(config, "require_elbow_tracking", False)
        if not isinstance(self._dynamic_elbow, (bool, np.bool_)):
            raise ValueError("dynamic_elbow_direction must be boolean")
        if not isinstance(self._require_elbow, (bool, np.bool_)):
            raise ValueError("require_elbow_tracking must be boolean")
        if self._require_elbow and not self._dynamic_elbow:
            raise ValueError("require_elbow_tracking requires dynamic_elbow_direction")
        for name, value in (("rate_hz", self._rate), ("min_cutoff", self._min_cutoff),
                            ("beta", self._beta), ("elbow_min_cutoff", self._elbow_min_cutoff)):
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        self._expected_reference = {
            side: _side_value(config, "expected_reference_frame", side, "xr_tracking")
            for side in SIDES
        }
        self._expected_tracked = {
            side: _side_value(config, "expected_tracked_frame", side, "wrist_tracker")
            for side in SIDES
        }
        for side in SIDES:
            for field, value in (("expected_reference_frame", self._expected_reference[side]),
                                 ("expected_tracked_frame", self._expected_tracked[side])):
                if not isinstance(value, str) or not value:
                    raise ValueError(f"{side}.{field} must be a non-empty string")
        self._tracked_to_wrist = {
            side: _pose(_side_value(config, "tracked_to_wrist_pose", side, _IDENTITY_POSE),
                        f"{side}.tracked_to_wrist_pose")
            for side in SIDES
        }
        self._controller = IncrementalController(
            self._reference_config,
            rate=self._rate,
            min_cutoff=self._min_cutoff,
            beta=self._beta,
            elbow_min_cutoff=self._elbow_min_cutoff,
        )
        self._initialized = False
        self._clutched: set[str] = set()
        self._last_raw: dict[str, np.ndarray] = {}
        self._last_raw_elbow: dict[str, np.ndarray] = {}
        self._last_output: dict[str, np.ndarray] = {}
        self._last_elbow_output: dict[str, np.ndarray] = {}
        self._last_elbow_direction: dict[str, tuple[float, float, float]] = {}

    @property
    def reference_config_path(self) -> str:
        return str(self._reference_config.config_path)

    @property
    def initialized(self) -> bool:
        return self._initialized

    def _check_observation(self, observation: ArmInputObservation) -> None:
        if not isinstance(observation, ArmInputObservation):
            raise TypeError("arm_input must be ArmInputObservation")
        if observation.side not in SIDES:
            raise ValueError("arm input side must be left or right")
        if observation.reference_frame != self._expected_reference[observation.side]:
            raise ValueError(
                f"{observation.side} input reference frame must be "
                f"{self._expected_reference[observation.side]!r}"
            )
        if observation.tracked_frame != self._expected_tracked[observation.side]:
            raise ValueError(
                f"{observation.side} input tracked frame must be "
                f"{self._expected_tracked[observation.side]!r}"
            )

    def _tracked_pose(self, observation: ArmInputObservation) -> np.ndarray:
        if not observation.valid or observation.pose is None:
            raise ValueError("valid XR arm observation requires a pose")
        return _compose(_pose(observation.pose, "arm_input.pose"), self._tracked_to_wrist[observation.side])

    @staticmethod
    def _tracked_elbow_pose(observation: ArmInputObservation) -> np.ndarray | None:
        if observation.elbow_pose is None:
            return None
        return _pose(observation.elbow_pose, "arm_input.elbow_pose")

    @staticmethod
    def _role(side: str) -> str:
        return f"pico_{side}_wrist"

    @staticmethod
    def _elbow_role(side: str) -> str:
        return f"pico_{side}_arm"

    @property
    def dynamic_elbow_direction(self) -> bool:
        return bool(self._dynamic_elbow)

    def _mapping_version(self) -> str:
        return "xr_incremental_v2" if self._dynamic_elbow else "xr_incremental_v1"

    def initialize(self, reference: Any = None) -> None:
        if not isinstance(reference, Mapping) or not reference:
            raise ValueError("xr_incremental requires a side-to-observation mapping")
        tracker_poses: dict[str, np.ndarray] = {}
        raw_values: dict[str, np.ndarray] = {}
        elbow_values: dict[str, np.ndarray] = {}
        for side, observation in reference.items():
            if side not in SIDES:
                raise ValueError("XR initialization has an invalid side")
            self._check_observation(observation)
            raw = self._tracked_pose(observation)
            tracker_poses[self._role(side)] = raw
            raw_values[side] = raw
            if self._dynamic_elbow:
                elbow_raw = self._tracked_elbow_pose(observation)
                if elbow_raw is None:
                    if self._require_elbow:
                        raise ValueError(f"fresh forearm tracker pose is required for {side}")
                else:
                    tracker_poses[self._elbow_role(side)] = elbow_raw
                    elbow_values[side] = elbow_raw
        self._controller.reset()
        self._controller.initialize(tracker_poses)
        self._initialized = True
        self._clutched.clear()
        self._last_raw = raw_values
        self._last_raw_elbow = elbow_values
        self._last_output.clear()
        self._last_elbow_output.clear()
        self._last_elbow_direction.clear()

    def reset(self) -> None:
        self._controller.reset()
        self._initialized = False
        self._clutched.clear()
        self._last_raw.clear()
        self._last_raw_elbow.clear()
        self._last_output.clear()
        self._last_elbow_output.clear()
        self._last_elbow_direction.clear()

    def set_clutch(self, side: str, pressed: bool) -> None:
        if side not in SIDES:
            raise ValueError("clutch side must be left or right")
        if not isinstance(pressed, (bool, np.bool_)):
            raise ValueError("clutch state must be boolean")
        if pressed:
            self._clutched.add(side)
            return
        if side not in self._clutched:
            return
        self._clutched.remove(side)
        raw = self._last_raw.get(side)
        output = self._last_output.get(side)
        if self._initialized and raw is not None and output is not None:
            role = self._role(side)
            self._controller.init_tracker_poses[role] = self._controller._pose_to_matrix(raw)
            self._controller.robot_init_positions[role] = output[:3].copy()
            self._controller.robot_init_rotations[role] = Rotation.from_quat(output[3:])
            self._controller._pos_filters.pop(role, None)
            self._controller._rot_filters.pop(role, None)
        raw_elbow = self._last_raw_elbow.get(side)
        output_elbow = self._last_elbow_output.get(side)
        if self._initialized and raw_elbow is not None and output_elbow is not None:
            role = self._elbow_role(side)
            self._controller.init_tracker_poses[role] = self._controller._pose_to_matrix(raw_elbow)
            self._controller.robot_init_positions[role] = output_elbow[:3].copy()
            self._controller.robot_init_rotations[role] = Rotation.from_quat(output_elbow[3:])
            self._controller._pos_filters.pop(role, None)
            self._controller._rot_filters.pop(role, None)

    def _invalid(self, arm_input: ArmInputObservation) -> MappedArmPose:
        return MappedArmPose(
            arm_input.side, None, False, "xr_incremental",
            self._mapping_version(),
            arm_input.frame_association_id,
        )

    def map(self, arm_input: ArmInputObservation) -> MappedArmPose:
        self._check_observation(arm_input)
        if not self._initialized:
            raise RuntimeError("xr_incremental is not initialized")
        if not arm_input.valid or arm_input.pose is None:
            return self._invalid(arm_input)
        raw = self._tracked_pose(arm_input)
        self._last_raw[arm_input.side] = raw
        if arm_input.side in self._clutched:
            output = self._last_output.get(arm_input.side)
            if output is None:
                return self._invalid(arm_input)
            return MappedArmPose(
                arm_input.side, output.copy(), True, "xr_incremental", self._mapping_version(),
                arm_input.frame_association_id,
                self._last_elbow_direction.get(arm_input.side),
            )
        position, quaternion = self._controller.compute_target_pose(raw, self._role(arm_input.side))
        if position is None or quaternion is None:
            return self._invalid(arm_input)
        output = np.concatenate((np.asarray(position, dtype=np.float64), np.asarray(quaternion, dtype=np.float64)))
        output = _pose(output, "mapped XR pose")
        self._last_output[arm_input.side] = output.copy()
        elbow_direction = self._last_elbow_direction.get(arm_input.side)
        if self._dynamic_elbow:
            raw_elbow = self._tracked_elbow_pose(arm_input)
            if raw_elbow is not None:
                self._last_raw_elbow[arm_input.side] = raw_elbow
                elbow_position, elbow_quaternion = self._controller.compute_target_pose(
                    raw_elbow, self._elbow_role(arm_input.side)
                )
                if elbow_position is not None and elbow_quaternion is not None:
                    elbow_output = _pose(
                        np.concatenate((np.asarray(elbow_position, dtype=np.float64),
                                        np.asarray(elbow_quaternion, dtype=np.float64))),
                        "mapped XR elbow pose",
                    )
                    self._last_elbow_output[arm_input.side] = elbow_output.copy()
                    physical, _ = self._controller.compute_elbow_direction(
                        np.zeros(3, dtype=np.float64), output[:3], elbow_output[:3], arm_input.side
                    )
                    physical = np.asarray(physical, dtype=np.float64)
                    norm = float(np.linalg.norm(physical))
                    if norm > 1.0e-12 and np.isfinite(physical).all():
                        # The reference IK consumes anti-gravity direction;
                        # compute_elbow_direction returns the physical elbow
                        # offset direction, so the published hint is negated.
                        elbow_direction = tuple(float(item) for item in (-physical / norm))
                        self._last_elbow_direction[arm_input.side] = elbow_direction
        return MappedArmPose(
            arm_input.side, output, True, "xr_incremental",
            self._mapping_version(),
            arm_input.frame_association_id, elbow_direction,
        )


__all__ = ["XrIncrementalMapper"]
