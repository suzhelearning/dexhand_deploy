"""Pure observation-to-target bridge for the simulation control profile.

The bridge is intentionally independent of Zenoh and lifecycle management.
It validates canonical observation messages, applies the selected hand-frame
adapter plus arm mapping/conditioning factories, and returns typed target
inputs for a node to publish.  A caller must explicitly start teleoperation;
no target is produced while the bridge is armed.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

from ..protocol.messages import (
    ArmInputObservation as ArmInputObservationWire,
    HandSkeletonObservation,
    strict_loads,
)
from ..sources.common.pose_mapping import ArmPoseMapper
from ..sources.common.target_processing import ArmTargetProcessor
from .hand_target_adapter import HandTargetAdapter, PreparedHandTarget
from .models import ArmInputObservation


SIDES = ("left", "right")


class TargetBridgeInputRejected(ValueError):
    """Raised when an observation cannot safely feed a target path."""


@dataclass(frozen=True)
class PreparedArmTarget:
    side: str
    pose: np.ndarray
    elbow_reference_direction: tuple[float, float, float]
    source_timestamp_ns: int | None
    observation_sequence: int
    observation_publisher_instance_id: str
    frame_association_id: str
    mapping_backend: str
    processor_backend: str
    tracking_valid: bool = True

    def __post_init__(self) -> None:
        if self.side not in SIDES:
            raise ValueError("side must be left or right")
        pose = np.asarray(self.pose, dtype=np.float64)
        if pose.shape != (7,) or not np.isfinite(pose).all() or np.linalg.norm(pose[3:]) < 1.0e-12:
            raise ValueError("prepared arm target pose must be a finite non-zero-quaternion 7-vector")
        elbow = np.asarray(self.elbow_reference_direction, dtype=np.float64)
        if elbow.shape != (3,) or not np.isfinite(elbow).all() or np.linalg.norm(elbow) < 1.0e-12:
            raise ValueError("elbow_reference_direction must be a finite non-zero 3-vector")
        if isinstance(self.observation_sequence, bool) or self.observation_sequence < 0:
            raise ValueError("observation_sequence must be non-negative")
        if not isinstance(self.observation_publisher_instance_id, str) or not self.observation_publisher_instance_id:
            raise ValueError("observation publisher identity is required")
        if not isinstance(self.frame_association_id, str) or not self.frame_association_id:
            raise ValueError("frame_association_id is required")
        if not isinstance(self.mapping_backend, str) or not self.mapping_backend:
            raise ValueError("mapping_backend is required")
        if not isinstance(self.processor_backend, str) or not self.processor_backend:
            raise ValueError("processor_backend is required")
        object.__setattr__(self, "pose", pose.copy())
        object.__setattr__(self, "elbow_reference_direction", tuple(float(value) for value in elbow))


@dataclass(frozen=True)
class BridgeTargets:
    hand: tuple[PreparedHandTarget, ...] = ()
    arm: tuple[PreparedArmTarget, ...] = ()


def _payload(value: Any) -> Any:
    if isinstance(value, Mapping):
        return value
    if isinstance(value, (bytes, bytearray, memoryview)):
        return strict_loads(bytes(value))
    payload = getattr(value, "payload", None)
    if payload is not None:
        return strict_loads(bytes(payload))
    return value


def _positive_seconds(value: Any, field: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be a positive finite number") from exc
    if not np.isfinite(result) or result <= 0.0:
        raise ValueError(f"{field} must be a positive finite number")
    return result


def _elbow(value: Any, field: str) -> tuple[float, float, float]:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3,) or not np.isfinite(result).all() or np.linalg.norm(result) < 1.0e-12:
        raise ValueError(f"{field} must be a finite non-zero 3-vector")
    return tuple(float(item) for item in result)


def _arm_model(value: ArmInputObservationWire) -> ArmInputObservation:
    return ArmInputObservation(
        source=value.source,
        side=value.side,
        tracked_frame=value.tracked_frame,
        reference_frame=value.reference_frame,
        pose=None if value.pose is None else np.asarray(value.pose, dtype=np.float64),
        valid=value.valid,
        source_timestamp_ns=value.source_timestamp_ns,
        received_timestamp_ns=value.received_timestamp_ns,
        receiver_instance_id=value.receiver_instance_id,
        receiver_frame_sequence=value.receiver_frame_sequence,
        mapping_version=value.mapping_version,
        frame_association_id=value.frame_association_id,
        source_sequence=value.source_sequence,
        source_instance_id=value.source_instance_id,
    )


class ObservationTargetBridge:
    """Convert canonical observations into targets only after explicit start."""

    def __init__(
        self,
        *,
        input_profile: str,
        router_zid: str,
        observation_publisher_instance_id: str,
        hand_adapter: HandTargetAdapter,
        arm_input_source: str,
        pose_mapper: ArmPoseMapper,
        target_processor: ArmTargetProcessor,
        active_sides: tuple[str, ...] = SIDES,
        active_hand_sides: tuple[str, ...] = SIDES,
        elbow_reference_direction: Mapping[str, Any] | None = None,
        arm_input_max_age_s: float = 0.5,
        period_s: float = 1.0 / 60.0,
        clock: Any | None = None,
        hold_on_tracking_loss: bool = False,
    ) -> None:
        if input_profile not in {"pico", "manus"}:
            raise ValueError("input_profile must be pico or manus")
        if not router_zid or not observation_publisher_instance_id or not arm_input_source:
            raise ValueError("router, observation publisher, and arm source identities are required")
        if any(side not in SIDES for side in active_sides) or len(set(active_sides)) != len(active_sides):
            raise ValueError("active_sides must contain unique left/right values")
        if any(side not in SIDES for side in active_hand_sides) or len(set(active_hand_sides)) != len(active_hand_sides):
            raise ValueError("active_hand_sides must contain unique left/right values")
        if not active_sides:
            raise ValueError("at least one active arm side is required")
        if not hasattr(hand_adapter, "adapt") or not hasattr(hand_adapter, "reset"):
            raise TypeError("hand_adapter must implement adapt/reset")
        if not hasattr(pose_mapper, "map") or not hasattr(pose_mapper, "initialize"):
            raise TypeError("pose_mapper must implement map/initialize")
        if not hasattr(target_processor, "process") or not hasattr(target_processor, "reset"):
            raise TypeError("target_processor must implement process/reset")
        directions = elbow_reference_direction or {}
        self.input_profile = input_profile
        self.router_zid = router_zid
        self.observation_publisher_instance_id = observation_publisher_instance_id
        self.hand_adapter = hand_adapter
        self.arm_input_source = arm_input_source
        self.pose_mapper = pose_mapper
        self.target_processor = target_processor
        self.active_sides = tuple(active_sides)
        self.active_hand_sides = tuple(active_hand_sides)
        self._elbows = {
            side: _elbow(directions.get(side), f"elbow_reference_direction.{side}")
            for side in SIDES
        }
        self.arm_input_max_age_ns = int(_positive_seconds(arm_input_max_age_s, "arm_input_max_age_s") * 1.0e9)
        self.period_s = _positive_seconds(period_s, "period_s")
        self._clock = clock
        self._hand_latest: dict[str, HandSkeletonObservation] = {}
        self._arm_latest: dict[str, ArmInputObservation] = {}
        self._arm_latest_sequence: dict[str, int] = {}
        self._hand_baselines: dict[str, tuple[str, int]] = {}
        self._arm_baselines: dict[str, tuple[str, int]] = {}
        self._last_hand_emitted: dict[str, int] = {}
        self._last_arm_emitted: dict[str, int] = {}
        self._last_arm_timestamp_ns: dict[str, int] = {}
        self._started = False
        self.hold_on_tracking_loss = hold_on_tracking_loss
        self._hold_poses: dict[str, np.ndarray] = {}

    @property
    def tracking_hold_sides(self) -> tuple[str, ...]:
        return tuple(side for side, value in self._arm_latest.items() if not value.valid)

    @property
    def started(self) -> bool:
        return self._started

    def _now(self) -> int:
        if self._clock is None:
            import time
            return time.monotonic_ns()
        return int(self._clock())

    def _fresh(self, timestamp_ns: int, now_ns: int, maximum_age_ns: int) -> bool:
        return timestamp_ns <= now_ns and now_ns - timestamp_ns <= maximum_age_ns

    def _require_identity(self, publisher_instance_id: str, side: str, sequence: int, baselines: dict[str, tuple[str, int]]) -> None:
        if publisher_instance_id != self.observation_publisher_instance_id:
            raise TargetBridgeInputRejected("observation publisher identity is not authorized")
        old = baselines.get(side)
        if old is not None and (publisher_instance_id != old[0] or sequence <= old[1]):
            raise TargetBridgeInputRejected("observation identity or sequence rollback")
        baselines[side] = (publisher_instance_id, sequence)

    def ingest_hand_observation(self, value: Any, *, now_ns: int | None = None) -> HandSkeletonObservation:
        try:
            observation = value if isinstance(value, HandSkeletonObservation) else HandSkeletonObservation.from_dict(_payload(value))
        except Exception as exc:
            raise TargetBridgeInputRejected(f"invalid hand observation: {exc}") from exc
        if observation.router_zid != self.router_zid or observation.side not in self.active_hand_sides:
            raise TargetBridgeInputRejected("hand observation router or side is not authorized")
        adapter_source = getattr(self.hand_adapter, "source", None)
        adapter_frame = getattr(self.hand_adapter, "coordinate_frame", None)
        if adapter_source is not None and observation.source != adapter_source:
            raise TargetBridgeInputRejected("hand observation source is not authorized")
        if adapter_frame is not None and observation.coordinate_frame != adapter_frame:
            raise TargetBridgeInputRejected("hand observation coordinate frame is not authorized")
        if not observation.valid or not all(observation.joint_valid):
            raise TargetBridgeInputRejected("hand observation is invalid")
        self._require_identity(
            observation.publisher_instance_id,
            observation.side,
            observation.sequence,
            self._hand_baselines,
        )
        if now_ns is not None:
            maximum_age_ns = int(getattr(self.hand_adapter, "max_age_ns", self.arm_input_max_age_ns))
            if not self._fresh(observation.timestamp_ns, int(now_ns), maximum_age_ns):
                self._hand_baselines.pop(observation.side, None)
                raise TargetBridgeInputRejected("hand observation is stale")
        self._hand_latest[observation.side] = observation
        return observation

    def ingest_arm_observation(self, value: Any) -> ArmInputObservation:
        try:
            wire = value if isinstance(value, ArmInputObservationWire) else ArmInputObservationWire.from_dict(_payload(value))
        except Exception as exc:
            raise TargetBridgeInputRejected(f"invalid arm observation: {exc}") from exc
        if wire.router_zid != self.router_zid or wire.side not in self.active_sides:
            raise TargetBridgeInputRejected("arm observation router or side is not authorized")
        if wire.source != self.arm_input_source:
            raise TargetBridgeInputRejected("arm observation source is not authorized")
        if (not wire.valid or wire.pose is None) and not self.hold_on_tracking_loss:
            raise TargetBridgeInputRejected("arm observation is invalid")
        previous = self._arm_latest.get(wire.side)
        if previous is not None and (wire.reference_frame != previous.reference_frame or wire.tracked_frame != previous.tracked_frame):
            raise TargetBridgeInputRejected("arm observation coordinate frame changed")
        self._require_identity(
            wire.publisher_instance_id,
            wire.side,
            wire.sequence,
            self._arm_baselines,
        )
        observation = _arm_model(wire)
        self._arm_latest[observation.side] = observation
        self._arm_latest_sequence[observation.side] = wire.sequence
        return observation

    def _require_start_inputs(self, now_ns: int, *, allow_tracking_loss: bool = False) -> None:
        for side in self.active_sides:
            observation = self._arm_latest.get(side)
            if observation is None or (not allow_tracking_loss and (not observation.valid or observation.pose is None)) or not self._fresh(observation.received_timestamp_ns, now_ns, self.arm_input_max_age_ns):
                raise TargetBridgeInputRejected(f"fresh valid arm observation is required for {side}")
        for side in self.active_hand_sides:
            observation = self._hand_latest.get(side)
            if observation is None or not observation.valid or not self._fresh(observation.timestamp_ns, now_ns, self.arm_input_max_age_ns):
                raise TargetBridgeInputRejected(f"fresh valid hand observation is required for {side}")

    def start(self, *, now_ns: int | None = None) -> None:
        if self._started:
            raise RuntimeError("target bridge is already started")
        now_ns = self._now() if now_ns is None else int(now_ns)
        self._require_start_inputs(now_ns)
        references = {side: self._arm_latest[side] for side in self.active_sides}
        try:
            self.pose_mapper.initialize(references)
            self.target_processor.reset()
            if self.hold_on_tracking_loss:
                self._hold_poses = {
                    side: self.pose_mapper.map(value).pose.copy()
                    for side, value in references.items()
                }
        except Exception as exc:
            raise TargetBridgeInputRejected(f"target bridge initialization failed: {exc}") from exc
        self._last_hand_emitted.clear()
        self._last_arm_emitted.clear()
        self._last_arm_timestamp_ns.clear()
        self._started = True

    def reset(self) -> None:
        self._started = False
        self._hold_poses.clear()
        self._hand_latest.clear()
        self._arm_latest.clear()
        self._arm_latest_sequence.clear()
        self._hand_baselines.clear()
        self._arm_baselines.clear()
        self._last_hand_emitted.clear()
        self._last_arm_emitted.clear()
        self._last_arm_timestamp_ns.clear()
        self.hand_adapter.reset()
        self.pose_mapper.reset()
        self.target_processor.reset()

    def tick(self, *, now_ns: int | None = None) -> BridgeTargets:
        if not self._started:
            return BridgeTargets()
        now_ns = self._now() if now_ns is None else int(now_ns)
        self._require_start_inputs(now_ns, allow_tracking_loss=self.hold_on_tracking_loss)

        hands: list[PreparedHandTarget] = []
        for side in self.active_hand_sides:
            observation = self._hand_latest[side]
            if observation.sequence <= self._last_hand_emitted.get(side, -1):
                continue
            try:
                hands.append(self.hand_adapter.adapt(observation, now_ns=now_ns))
            except Exception as exc:
                raise TargetBridgeInputRejected(f"hand target adaptation failed for {side}: {exc}") from exc

        arms: list[PreparedArmTarget] = []
        for side in self.active_sides:
            observation = self._arm_latest[side]
            if observation.source != self.arm_input_source:
                raise TargetBridgeInputRejected(f"arm observation source changed for {side}")
            if observation.source_sequence is not None and observation.source_sequence < 0:
                raise TargetBridgeInputRejected(f"arm source sequence is invalid for {side}")
            if observation.receiver_frame_sequence < 0:
                raise TargetBridgeInputRejected(f"arm receiver sequence is invalid for {side}")
            observation_sequence = self._arm_latest_sequence[side]
            if observation_sequence <= self._last_arm_emitted.get(side, -1):
                continue
            if not observation.valid:
                arms.append(PreparedArmTarget(side, self._hold_poses[side], self._elbows[side],
                    observation.source_timestamp_ns, observation_sequence,
                    self.observation_publisher_instance_id, observation.frame_association_id,
                    'tracking_hold', 'tracking_hold', tracking_valid=False))
                continue
            mapped = self.pose_mapper.map(observation)
            dt_s = self.period_s
            previous_timestamp = self._last_arm_timestamp_ns.get(side)
            if previous_timestamp is not None:
                dt_s = max((observation.received_timestamp_ns - previous_timestamp) / 1.0e9, self.period_s * 0.25)
            processed = self.target_processor.process(mapped, dt_s=dt_s)
            if not processed.valid or processed.pose is None:
                raise TargetBridgeInputRejected(f"arm target processing returned invalid output for {side}")
            self._hold_poses[side] = processed.pose.copy()
            arms.append(
                PreparedArmTarget(
                    side=side,
                    pose=processed.pose,
                    elbow_reference_direction=self._elbows[side],
                    source_timestamp_ns=observation.source_timestamp_ns,
                    observation_sequence=observation_sequence,
                    observation_publisher_instance_id=self.observation_publisher_instance_id,
                    frame_association_id=processed.frame_association_id,
                    mapping_backend=processed.mapping_backend,
                    processor_backend=processed.backend,
                )
            )

        for item in hands:
            self._last_hand_emitted[item.side] = item.observation_sequence
        for item in arms:
            self._last_arm_emitted[item.side] = item.observation_sequence
            self._last_arm_timestamp_ns[item.side] = self._arm_latest[item.side].received_timestamp_ns
        return BridgeTargets(tuple(hands), tuple(arms))


__all__ = [
    "BridgeTargets",
    "ObservationTargetBridge",
    "PreparedArmTarget",
    "TargetBridgeInputRejected",
]
