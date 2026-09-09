"""Source-aware adaptation from canonical hand observations to hand targets.

The observation stream intentionally preserves each device's coordinate-frame
label.  This module is the small boundary where a control profile is allowed
to choose the retarget input frame.  It does not know about Zenoh, sessions,
retarget SDKs, or robot executors.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Protocol

import numpy as np

from ..protocol.messages import HandSkeletonObservation


SIDES = ("left", "right")


class HandObservationRejected(ValueError):
    """Raised when an observation cannot safely become a hand target."""


def _config_value(config: Any, name: str, default: Any = None) -> Any:
    if isinstance(config, Mapping):
        return config.get(name, default)
    return getattr(config, name, default)


def _side_value(config: Any, name: str, side: str, default: Any) -> Any:
    value = _config_value(config, name, default)
    if isinstance(value, Mapping):
        return value.get(side, default)
    return value


def _rotation(value: Any, field: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3, 3) or not np.isfinite(result).all():
        raise ValueError(f"{field} must be a finite 3x3 matrix")
    if not np.allclose(result @ result.T, np.eye(3), atol=1.0e-6):
        raise ValueError(f"{field} must be orthonormal")
    if not np.isclose(float(np.linalg.det(result)), 1.0, atol=1.0e-6):
        raise ValueError(f"{field} must be a proper rotation")
    return result.copy()


def _positive_seconds(value: Any, field: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be a positive finite number") from exc
    if not np.isfinite(result) or result <= 0.0:
        raise ValueError(f"{field} must be a positive finite number")
    return result


@dataclass(frozen=True)
class PreparedHandTarget:
    """One validated, wrist-relative target ready for ``HandTargetCommand``."""

    side: str
    keypoints_m: np.ndarray
    source_timestamp_ns: int | None
    observation_sequence: int
    observation_publisher_instance_id: str
    frame_association_id: str
    adapter_version: str

    def __post_init__(self) -> None:
        if self.side not in SIDES:
            raise ValueError("side must be left or right")
        points = np.asarray(self.keypoints_m, dtype=np.float64)
        if points.shape != (21, 3) or not np.isfinite(points).all():
            raise ValueError("prepared hand keypoints must be finite with shape (21,3)")
        if not np.array_equal(points[0], np.zeros(3, dtype=np.float64)):
            raise ValueError("prepared hand keypoints wrist must be exactly zero")
        if isinstance(self.observation_sequence, bool) or self.observation_sequence < 0:
            raise ValueError("observation_sequence must be non-negative")
        if not isinstance(self.observation_publisher_instance_id, str) or not self.observation_publisher_instance_id:
            raise ValueError("observation publisher identity is required")
        if not isinstance(self.frame_association_id, str) or not self.frame_association_id:
            raise ValueError("frame_association_id is required")
        if not isinstance(self.adapter_version, str) or not self.adapter_version:
            raise ValueError("adapter_version is required")
        object.__setattr__(self, "keypoints_m", points.copy())


class HandTargetAdapter(Protocol):
    def reset(self) -> None: ...

    def adapt(
        self,
        observation: HandSkeletonObservation,
        *,
        now_ns: int,
    ) -> PreparedHandTarget: ...


class FixedRotationHandTargetAdapter:
    """Apply one configured proper rotation to one source/profile."""

    def __init__(self, config: Any):
        source = _config_value(config, "source")
        coordinate_frame = _config_value(config, "coordinate_frame")
        if not isinstance(source, str) or not source:
            raise ValueError("hand target adapter source is required")
        if not isinstance(coordinate_frame, str) or not coordinate_frame:
            raise ValueError("hand target adapter coordinate_frame is required")
        self.source = source
        self.coordinate_frame = coordinate_frame
        self.max_age_ns = int(_positive_seconds(_config_value(config, "max_age_s", 0.5), "max_age_s") * 1.0e9)
        self._rotations = {
            side: _rotation(
                _side_value(config, "rotation", side, np.eye(3)),
                f"rotation.{side}",
            )
            for side in SIDES
        }
        self._baselines: dict[str, tuple[str, int]] = {}

    @property
    def version(self) -> str:
        return "fixed_rotation_v1"

    def reset(self) -> None:
        self._baselines.clear()

    def adapt(
        self,
        observation: HandSkeletonObservation,
        *,
        now_ns: int,
    ) -> PreparedHandTarget:
        if not isinstance(observation, HandSkeletonObservation):
            raise TypeError("observation must be HandSkeletonObservation")
        if isinstance(now_ns, bool) or not isinstance(now_ns, (int, np.integer)):
            raise ValueError("now_ns must be an integer")
        now_ns = int(now_ns)
        if observation.source != self.source:
            raise HandObservationRejected("hand observation source is not authorized")
        if observation.coordinate_frame != self.coordinate_frame:
            raise HandObservationRejected("hand observation coordinate frame is not authorized")
        if observation.timestamp_ns > now_ns:
            raise HandObservationRejected("hand observation timestamp is in the future")
        if now_ns - observation.timestamp_ns > self.max_age_ns:
            raise HandObservationRejected("hand observation is stale")
        if not observation.valid or not all(observation.joint_valid):
            raise HandObservationRejected("hand observation is invalid")
        baseline = self._baselines.get(observation.side)
        current = (observation.publisher_instance_id, observation.sequence)
        if baseline is not None:
            old_instance, old_sequence = baseline
            if current[0] != old_instance or current[1] <= old_sequence:
                raise HandObservationRejected("hand observation identity or sequence rollback")

        points = np.asarray(observation.keypoints_m, dtype=np.float64)
        # Canonical observations are already wrist-relative.  Re-relative here
        # as a defensive idempotent boundary before applying the one profile
        # rotation; this also guarantees the exact protocol wrist invariant.
        points = points - points[0]
        points = points @ self._rotations[observation.side].T
        points[0] = 0.0
        result = PreparedHandTarget(
            side=observation.side,
            keypoints_m=points,
            source_timestamp_ns=observation.source_timestamp_ns,
            observation_sequence=observation.sequence,
            observation_publisher_instance_id=observation.publisher_instance_id,
            frame_association_id=observation.frame_association_id,
            adapter_version=self.version,
        )
        self._baselines[observation.side] = current
        return result


def create_hand_target_adapter(name: str, config: Any) -> HandTargetAdapter:
    if name == "fixed_rotation":
        return FixedRotationHandTargetAdapter(config)
    raise ValueError("unknown hand target adapter: " + str(name))


__all__ = [
    "FixedRotationHandTargetAdapter",
    "HandObservationRejected",
    "HandTargetAdapter",
    "PreparedHandTarget",
    "create_hand_target_adapter",
]
