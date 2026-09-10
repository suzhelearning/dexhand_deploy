"""Simulation-only target source for Manus/PICO hand tracking.

This node is the control-side boundary of the hand-tracking pipeline.  It
consumes canonical observations, waits for an explicit coordinator-authorized
teleop state, and publishes the existing ``tianji/target/arm`` and
``tianji/target/hand`` messages.  It never solves IK, sends joint commands, or
opens a robot device.
"""
from __future__ import annotations

import argparse
from collections.abc import Mapping
import logging
import os
from pathlib import Path
import signal
import threading
import time
from typing import Any, Callable

import numpy as np

from ..config_loader import load_component_config, require_finite_positive
from ..protocol import topics
from ..sources.common.session_client import SessionClient
from ..sources.common.target_publisher import SequenceAllocator, TargetPublisher
from ..sources.common.keyboard import raw_keyboard
from ..zenoh_util import ZenohJsonSub, open_session, require_single_router
from .hand_target_adapter import create_hand_target_adapter
from .target_bridge import (
    BridgeTargets,
    ObservationTargetBridge,
    TargetBridgeInputRejected,
)
from ..sources.common.pose_mapping import create_arm_pose_mapper, HeadPalmDirectMapper
from .height_calibration import HeightCalibration, horizontal_reference
from ..sources.common.target_processing import create_arm_target_processor
from .xr_operator import XrControllerOperatorConfig
from .xr_input import XR_ARM_INPUTS


LOG = logging.getLogger("hand_tracking_target")
TARGET_SOURCE_ID = "hand_tracking_target"
_SIDES = ("left", "right")
_SOURCE_BY_PROFILE = {"pico": "pico", "manus": "legacy_pico_palm"}
_HAND_FRAME_BY_PROFILE = {
    "pico": "pico_tracking_initial_wrist_relative",
    "manus": "manus_local_vuh_y_flipped_wrist_relative",
}
_CONFIG_KEYS = frozenset(
    {
        "input_profile",
        "simulation_only",
        "rate_hz",
        "period_s",
        "arm_input_max_age_s",
        "active_sides",
        "active_hand_sides",
        "elbow_reference_direction",
        "arm_input_source",
        "hand_adapter",
        "hand_adapter_config",
        "arm_pose_mapper",
        "arm_pose_mapper_config",
        "head_direct_mapper_config",
        "head_palm_direct_mapper_config",
        "arm_target_processor",
        "arm_target_processor_config",
        "operator_config",
    }
)


def _mapping(value: Any, field: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be a mapping")
    return dict(value)


def _sides(value: Any, field: str) -> tuple[str, ...]:
    if not isinstance(value, (list, tuple)) or (not value and field != "active_hand_sides"):
        raise ValueError(f"{field} must be a non-empty list")
    result = tuple(value)
    if any(side not in _SIDES for side in result) or len(set(result)) != len(result):
        raise ValueError(f"{field} must contain unique left/right values")
    return result


def _ensure_fields(value: Mapping[str, Any], allowed: set[str], field: str) -> None:
    extra = set(value) - allowed
    if extra:
        raise ValueError(f"unknown fields in {field}: {sorted(extra)}")


def _validate_rotation_config(value: Mapping[str, Any]) -> None:
    rotation = value.get("rotation")
    if rotation is None:
        return
    if isinstance(rotation, Mapping):
        if set(rotation) - set(_SIDES):
            raise ValueError("hand_adapter_config.rotation must contain only left/right")
        for side, matrix in rotation.items():
            array = np.asarray(matrix, dtype=np.float64)
            if array.shape != (3, 3) or not np.isfinite(array).all():
                raise ValueError(f"hand_adapter_config.rotation.{side} must be a finite 3x3 matrix")
    else:
        array = np.asarray(rotation, dtype=np.float64)
        if array.shape != (3, 3) or not np.isfinite(array).all():
            raise ValueError("hand_adapter_config.rotation must be a finite 3x3 matrix or side mapping")


def _load_config(path: str | os.PathLike[str]) -> dict[str, Any]:
    """Load and normalize one strict simulation target profile."""
    config = load_component_config(
        path,
        allowed_keys=_CONFIG_KEYS,
        required_keys={
            "input_profile",
            "simulation_only",
            "active_sides",
            "active_hand_sides",
            "elbow_reference_direction",
            "arm_input_source",
            "hand_adapter",
            "hand_adapter_config",
            "arm_pose_mapper",
            "arm_pose_mapper_config",
            "arm_target_processor",
            "arm_target_processor_config",
        },
    )
    if config["simulation_only"] is not True:
        raise ValueError("simulation_only must be exactly true")
    profile = config["input_profile"]
    if profile not in _SOURCE_BY_PROFILE:
        raise ValueError("input_profile must be exactly 'pico' or 'manus'")
    active_sides = _sides(config["active_sides"], "active_sides")
    active_hand_sides = _sides(config["active_hand_sides"], "active_hand_sides")
    directions = _mapping(config["elbow_reference_direction"], "elbow_reference_direction")
    if set(directions) != set(_SIDES):
        raise ValueError("elbow_reference_direction must contain exactly left and right")
    for side, direction in directions.items():
        vector = np.asarray(direction, dtype=np.float64)
        if vector.shape != (3,) or not np.isfinite(vector).all() or np.linalg.norm(vector) < 1.0e-12:
            raise ValueError(f"elbow_reference_direction.{side} must be a finite non-zero 3-vector")

    expected_arm_source = _SOURCE_BY_PROFILE[profile]
    allowed_arm_sources = {expected_arm_source}
    if profile == "manus":
        allowed_arm_sources.add("xr")
    if config["arm_input_source"] not in allowed_arm_sources:
        raise ValueError(
            f"input_profile={profile!r} requires arm_input_source in {sorted(allowed_arm_sources)!r}"
        )
    hand_adapter_config = _mapping(config["hand_adapter_config"], "hand_adapter_config")
    _ensure_fields(
        hand_adapter_config,
        {"source", "coordinate_frame", "max_age_s", "rotation"},
        "hand_adapter_config",
    )
    if config["hand_adapter"] != "fixed_rotation":
        raise ValueError("hand_adapter must be fixed_rotation")
    if hand_adapter_config.get("source") != profile:
        raise ValueError(f"hand_adapter_config.source must be {profile!r}")
    if hand_adapter_config.get("coordinate_frame") != _HAND_FRAME_BY_PROFILE[profile]:
        raise ValueError(
            "hand_adapter_config.coordinate_frame must be "
            f"{_HAND_FRAME_BY_PROFILE[profile]!r}"
        )
    _validate_rotation_config(hand_adapter_config)

    if "operator_config" in config:
        if profile != "manus" or config["arm_input_source"] != "xr":
            raise ValueError("operator_config is only valid for the XR/Manus target source")
        operator_config = _mapping(config["operator_config"], "operator_config")
        XrControllerOperatorConfig.from_mapping(operator_config)
        config["operator_config"] = operator_config

    mapper_name = config["arm_pose_mapper"]
    mapper_config = _mapping(config["arm_pose_mapper_config"], "arm_pose_mapper_config")
    mapper_fields = {
        "home_pose",
        "input_to_base_rotation",
        "input_origin_in_base_m",
        "tracked_to_tcp_pose",
        "expected_reference_frame",
        "expected_tracked_frame",
        "dynamic_elbow_direction",
        "require_elbow_tracking",
    }
    if mapper_name not in {"relative_home", "direct_pose", "head_direct", "head_palm_direct", "xr_incremental"}:
        raise ValueError("arm_pose_mapper must be relative_home, direct_pose, head_direct, head_palm_direct or xr_incremental")
    if mapper_name in {"head_direct", "head_palm_direct"} and profile != "pico":
        raise ValueError("head mapping requires PICO input")
    if mapper_name == "head_palm_direct":
        mapper_fields.update({"head_height_offset_m", "head_forward_offset_m", "tcp_local_z_correction_deg"})
    if mapper_name == "relative_home":
        mapper_fields.add("ik_tcp_to_control_pose")
    if mapper_name == "xr_incremental":
        if profile != "manus" or config["arm_input_source"] != "xr":
            raise ValueError("xr_incremental requires input_profile='manus' and arm_input_source='xr'")
        mapper_fields.update({"reference_config_path", "tracked_to_wrist_pose", "rate_hz",
                              "min_cutoff", "beta", "elbow_min_cutoff",
                              "dynamic_elbow_direction", "require_elbow_tracking",
                              "controller_to_wrist_pose"})
    _ensure_fields(mapper_config, mapper_fields, "arm_pose_mapper_config")
    if mapper_name == "xr_incremental":
        create_arm_pose_mapper("xr_incremental", mapper_config)
    if "head_direct_mapper_config" in config:
        fixed = _mapping(config["head_direct_mapper_config"], "head_direct_mapper_config")
        _ensure_fields(fixed, {"input_to_base_rotation", "input_origin_in_base_m",
            "tracked_to_tcp_pose", "expected_reference_frame", "expected_tracked_frame"},
            "head_direct_mapper_config")
        create_arm_pose_mapper("head_direct", fixed)
        config["head_direct_mapper_config"] = fixed
    if "head_palm_direct_mapper_config" in config:
        fixed = _mapping(config["head_palm_direct_mapper_config"], "head_palm_direct_mapper_config")
        _ensure_fields(fixed, {"input_to_base_rotation", "input_origin_in_base_m",
            "tracked_to_tcp_pose", "expected_reference_frame", "expected_tracked_frame",
            "head_height_offset_m", "head_forward_offset_m", "tcp_local_z_correction_deg"}, "head_palm_direct_mapper_config")
        create_arm_pose_mapper("head_palm_direct", fixed)
        config["head_palm_direct_mapper_config"] = fixed

    processor_name = config["arm_target_processor"]
    processor_config = _mapping(
        config["arm_target_processor_config"], "arm_target_processor_config"
    )
    if processor_name == "passthrough":
        _ensure_fields(processor_config, set(), "arm_target_processor_config")
    elif processor_name == "conditioned":
        _ensure_fields(
            processor_config,
            {
                "rate_hz",
                "translation_gain",
                "rotation_gain",
                "workspace_relative_radii_m",
                "workspace_soft_zone_ratio",
                "maximum_linear_speed_m_s",
                "maximum_angular_speed_rad_s",
                "maximum_linear_acceleration_m_s2",
                "maximum_angular_acceleration_rad_s2",
                "initial_position",
                "initial_quaternion",
            },
            "arm_target_processor_config",
        )
    else:
        raise ValueError("arm_target_processor must be passthrough or conditioned")

    rate_hz = require_finite_positive(config.get("rate_hz", 60.0), "rate_hz")
    period_s = require_finite_positive(config.get("period_s", 1.0 / rate_hz), "period_s")
    arm_input_max_age_s = require_finite_positive(
        config.get("arm_input_max_age_s", 0.5), "arm_input_max_age_s"
    )
    config["input_profile"] = profile
    config["active_sides"] = active_sides
    config["active_hand_sides"] = active_hand_sides
    config["elbow_reference_direction"] = directions
    config["hand_adapter_config"] = hand_adapter_config
    config["arm_pose_mapper_config"] = mapper_config
    config["arm_target_processor_config"] = processor_config
    config["rate_hz"] = rate_hz
    config["period_s"] = period_s
    config["arm_input_max_age_s"] = arm_input_max_age_s
    return config


def select_arm_pose_mapper(config: dict[str, Any], backend: str | None) -> None:
    """Select a startup mapping without reusing incompatible geometry fields."""
    if backend is None:
        return
    if config['simulation_only'] is not True:
        raise ValueError('--arm-pose-mapper requires simulation')
    if backend == 'xr_incremental':
        if config['input_profile'] != 'manus' or config['arm_input_source'] != 'xr':
            raise ValueError('xr_incremental requires the XR/Manus target source')
        config['arm_pose_mapper'] = backend
        create_arm_pose_mapper(backend, config['arm_pose_mapper_config'])
        return
    if config['input_profile'] != 'pico':
        raise ValueError('--arm-pose-mapper requires PICO simulation or explicit XR/Manus mapper')
    if backend not in ('relative_home', 'head_direct', 'head_palm_direct'):
        raise ValueError('invalid --arm-pose-mapper')
    if backend == config['arm_pose_mapper']:
        return
    if backend in ('head_direct', 'head_palm_direct'):
        field = backend + '_mapper_config'
        fixed = _mapping(config.get(field), field)
        create_arm_pose_mapper(backend, fixed)
        config['arm_pose_mapper_config'] = fixed
        config['arm_pose_mapper'] = backend
    else:
        raise ValueError('relative_home requires a relative_home source config')


def select_xr_arm_input(config: dict[str, Any], arm_input: str | None) -> None:
    """Select the tracked object used by the isolated XR/Manus arm source.

    Tracker and controller poses share the same incremental mapper and
    reference frame.  Only the expected tracked-frame label changes; the
    PICO2 target configs intentionally reject this selector.
    """
    if arm_input is None:
        return
    if arm_input not in XR_ARM_INPUTS:
        raise ValueError("XR/Manus arm input must be xr_tracker or xr_controller")
    if (
        config.get("input_profile") != "manus"
        or config.get("arm_input_source") != "xr"
        or config.get("arm_pose_mapper") != "xr_incremental"
    ):
        raise ValueError("XR/Manus arm input selection requires the XR/Manus target config")
    mapper_config = dict(config["arm_pose_mapper_config"])
    # Keep the Tracker calibration as the canonical fallback.  Selecting the
    # controller must not overwrite it permanently because this function is
    # also used by tests/tools that resolve more than one input variant in the
    # same process.  Older configs without a controller-specific extrinsic
    # retain the historical shared transform.
    tracker_pose = config.get("_xr_tracker_to_wrist_pose")
    if tracker_pose is None:
        tracker_pose = mapper_config.get("tracked_to_wrist_pose")
        config["_xr_tracker_to_wrist_pose"] = tracker_pose
    if arm_input == "xr_controller":
        controller_pose = mapper_config.get("controller_to_wrist_pose", tracker_pose)
        mapper_config["tracked_to_wrist_pose"] = controller_pose
    else:
        mapper_config["tracked_to_wrist_pose"] = tracker_pose
    mapper_config["expected_tracked_frame"] = (
        "controller" if arm_input == "xr_controller" else "wrist_tracker"
    )
    create_arm_pose_mapper("xr_incremental", mapper_config)
    config["arm_pose_mapper_config"] = mapper_config
    config["xr_arm_input"] = arm_input


def create_bridge_from_config(
    config: Mapping[str, Any],
    *,
    router_zid: str,
    observation_publisher_instance_id: str,
    clock: Callable[[], int] | None = None,
) -> ObservationTargetBridge:
    """Build the profile-selected geometry pipeline without transport."""
    hand_adapter = create_hand_target_adapter(
        config["hand_adapter"], config["hand_adapter_config"]
    )
    pose_mapper = create_arm_pose_mapper(
        config["arm_pose_mapper"], config["arm_pose_mapper_config"]
    )
    target_processor = create_arm_target_processor(
        config["arm_target_processor"], config["arm_target_processor_config"]
    )
    return ObservationTargetBridge(
        input_profile=config["input_profile"],
        router_zid=router_zid,
        observation_publisher_instance_id=observation_publisher_instance_id,
        hand_adapter=hand_adapter,
        arm_input_source=config["arm_input_source"],
        pose_mapper=pose_mapper,
        target_processor=target_processor,
        active_sides=tuple(config["active_sides"]),
        active_hand_sides=tuple(config["active_hand_sides"]),
        elbow_reference_direction=config["elbow_reference_direction"],
        arm_input_max_age_s=float(config["arm_input_max_age_s"]),
        period_s=float(config["period_s"]),
        clock=clock,
        hold_on_tracking_loss=(config['input_profile'] == 'pico' and not config['active_hand_sides'])
        or config['arm_input_source'] == 'xr',
    )


class HandTrackingTargetNode:
    """Lifecycle-managed, simulation-only observation target publisher."""

    def __init__(
        self,
        *,
        bridge: ObservationTargetBridge | Any | None = None,
        config: Mapping[str, Any] | None = None,
        session: Any | None = None,
        session_client: Any | None = None,
        target_publisher: Any | None = None,
        router_zid: str | None = None,
        publisher_instance_id: str | None = None,
        observation_publisher_instance_id: str | None = None,
        coordinator_instance_id: str | None = None,
        active_sides: tuple[str, ...] | None = None,
        active_hand_sides: tuple[str, ...] | None = None,
        rate_hz: float | None = None,
        clock: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        if bridge is None:
            if config is None or not router_zid or not observation_publisher_instance_id:
                raise ValueError("config, router_zid and observation publisher identity are required")
            bridge = create_bridge_from_config(
                config,
                router_zid=router_zid,
                observation_publisher_instance_id=observation_publisher_instance_id,
                clock=clock,
            )
        self.bridge = bridge
        self._clock = clock
        self._lock = threading.RLock()
        self._phase = "armed"
        self._last_error: str | None = None
        self._calibration_input_error: str | None = None
        self._closed = False
        self._stop_event = threading.Event()
        self._return_deadline_ns: int | None = None
        self._target_processor_name = str(config['arm_target_processor']) if config is not None else 'injected'
        self._pose_mapper_name = str(config['arm_pose_mapper']) if config is not None else 'injected'
        self._xr_operator_config = config.get('operator_config') if config is not None else None
        if config is not None:
            resolved_active_sides = tuple(config["active_sides"])
            resolved_active_hand_sides = tuple(config["active_hand_sides"])
            resolved_rate_hz = float(config["rate_hz"])
            profile = str(config["input_profile"])
        else:
            resolved_active_sides = active_sides or _SIDES
            resolved_active_hand_sides = _SIDES if active_hand_sides is None else active_hand_sides
            resolved_rate_hz = 60.0 if rate_hz is None else float(rate_hz)
            profile = str(getattr(bridge, "input_profile", "unknown"))
        if set(resolved_active_sides) - set(_SIDES) or not resolved_active_sides:
            raise ValueError("active_sides must contain left/right")
        if set(resolved_active_hand_sides) - set(_SIDES):
            raise ValueError("active_hand_sides must contain left/right")
        if not np.isfinite(resolved_rate_hz) or resolved_rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive and finite")
        self.active_sides = tuple(resolved_active_sides)
        self.active_hand_sides = tuple(resolved_active_hand_sides)
        self.rate_hz = resolved_rate_hz
        self.period_s = 1.0 / resolved_rate_hz
        self.input_profile = profile
        self._height_calibration = (
            HeightCalibration(self.active_sides)
            if isinstance(getattr(bridge, 'pose_mapper', None), HeadPalmDirectMapper) else None
        )
        self._height_reference = None
        self._height_reported_state = None

        allocator = SequenceAllocator()
        if session_client is None:
            if session is None or not router_zid or not publisher_instance_id or not coordinator_instance_id:
                raise ValueError(
                    "session, router, publisher, and coordinator identities are required"
                )
            session_client = SessionClient(
                session,
                source=TARGET_SOURCE_ID,
                publisher_instance_id=publisher_instance_id,
                router_zid=router_zid,
                expected_coordinator_instance_id=coordinator_instance_id,
                allocator=allocator,
            )
            if target_publisher is None:
                target_publisher = TargetPublisher(
                    session,
                    source=TARGET_SOURCE_ID,
                    publisher_instance_id=publisher_instance_id,
                    router_zid=router_zid,
                    allocator=allocator,
                )
            session_client.start()
        elif target_publisher is None and session is not None:
            if not router_zid or not publisher_instance_id:
                raise ValueError("router and publisher identities are required")
            target_publisher = TargetPublisher(
                session,
                source=TARGET_SOURCE_ID,
                publisher_instance_id=publisher_instance_id,
                router_zid=router_zid,
                allocator=allocator,
            )
        if session_client is None or target_publisher is None:
            raise ValueError("session_client and target_publisher are required")
        self._session_client = session_client
        self._publisher = target_publisher
        self._subscriptions: list[Any] = []
        if session is not None:
            for side in self.active_hand_sides:
                self._subscriptions.append(
                    ZenohJsonSub(session, topics.hand_observation(side), self.on_hand_observation)
                )
            for side in self.active_sides:
                self._subscriptions.append(
                    ZenohJsonSub(session, topics.arm_input_observation(side), self.on_arm_observation)
                )
        self._publish_status()

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def last_error(self) -> str | None:
        return self._last_error

    @property
    def stop_event(self) -> threading.Event:
        return self._stop_event

    @property
    def closed(self) -> bool:
        return self._closed

    def _now_ns(self) -> int:
        return int(self._clock())

    def _publish_status(self) -> None:
        try:
            startup_ready = bool(getattr(self._session_client, "startup_ready", False))
            snapshot_complete = getattr(self._session_client, "snapshot_complete", None)
            snapshot_timed_out = getattr(self._session_client, "snapshot_timed_out", None)
            error = self._last_error or self._calibration_input_error
            self._publisher.publish_source_status(
                component_id=TARGET_SOURCE_ID,
                phase=self._phase,
                ready=startup_ready and self._phase != "fault" and error is None,
                healthy=self._phase != "fault" and error is None,
                capabilities=("simulation",),
                error=error,
                diagnostics={
                    "simulation_only": True,
                    "input_profile": self.input_profile,
                    "arm_target_processor": self._target_processor_name,
                    "arm_pose_mapper": self._pose_mapper_name,
                    "height_calibration": self._height_calibration.status() if self._height_calibration else {'state': 'disabled'},
                    "active_sides": list(self.active_sides),
                    "active_hand_sides": list(self.active_hand_sides),
                    "tracking_hold_sides": list(getattr(self.bridge, 'tracking_hold_sides', ())),
                    "session_snapshot_complete": (
                        None if snapshot_complete is None else bool(snapshot_complete)
                    ),
                    "session_snapshot_timed_out": (
                        None if snapshot_timed_out is None else bool(snapshot_timed_out)
                    ),
                    "session_coordinator_instance_id": getattr(
                        self._session_client, "coordinator_instance_id", None
                    ),
                    "target_topics": [topics.arm_target(side) for side in self.active_sides]
                    + [topics.hand_target(side) for side in self.active_hand_sides],
                },
            )
        except Exception:
            LOG.exception("failed to publish hand-tracking target status")

    def _set_error(self, error: Any) -> None:
        self._last_error = str(error) or type(error).__name__

    def _request_return_locked(self, reason: str) -> None:
        if self._phase in {"armed", "returning", "fault"}:
            return
        try:
            self._session_client.request_return(reason)
        except Exception as exc:
            self._set_error(f"return request failed: {exc}")
            self._phase = "fault"
            self._return_deadline_ns = None
            try:
                self.bridge.reset()
            except Exception:
                LOG.exception("failed to reset target bridge after return failure")
            return
        self._phase = "returning"
        self._return_deadline_ns = self._now_ns() + 5_000_000_000

    def _input_rejected_locked(self, reason: str, error: Exception) -> None:
        if (self._phase == 'armed' and reason == 'invalid arm observation'
                and self._height_calibration and (
                    self._height_calibration.state == 'collecting'
                    or (self._height_calibration.state == 'failed' and self._calibration_input_error is not None))):
            # A successful wrist calibration can recover this input error only.
            # Do not overwrite an unrelated coordinator/hand/transport fault.
            self._calibration_input_error = f'{reason}: {error}'
            self._height_calibration.fail('invalid observation during calibration')
            return
        if self._height_calibration and self._height_calibration.state == 'collecting':
            self._height_calibration.fail('invalid observation during calibration')
        self._set_error(f"{reason}: {error}")
        if self._phase in {"start_pending", "teleop"}:
            self._request_return_locked("target_bridge_rejected")

    def on_hand_observation(self, value: Any) -> bool:
        with self._lock:
            try:
                self.bridge.ingest_hand_observation(value, now_ns=self._now_ns())
                return True
            except Exception as exc:
                self._input_rejected_locked("invalid hand observation", exc)
                self._publish_status()
                return False

    def on_arm_observation(self, value: Any) -> bool:
        with self._lock:
            try:
                observation = self.bridge.ingest_arm_observation(value)
                if self._height_calibration:
                    self._height_calibration.add(observation, self._now_ns())
                return True
            except Exception as exc:
                self._input_rejected_locked("invalid arm observation", exc)
                self._publish_status()
                return False

    def request_height_calibration(self) -> bool:
        with self._lock:
            if self._closed or self._phase != 'armed' or self._height_calibration is None:
                LOG.warning('Height calibration requires head_palm_direct in armed state')
                return False
            if self._height_calibration.state == 'collecting':
                return False
            try:
                self._height_reference = horizontal_reference(os.environ.get('TIANJI_ARM_URDF'))
            except Exception as exc:
                self._height_calibration.fail(str(exc))
                LOG.warning('Height calibration model reference rejected: %s', exc)
                self._publish_status()
                return False
            self._height_calibration.begin(self._now_ns())
            self._height_reported_state = 'collecting'
            LOG.warning('Height calibration: hold both arms horizontal and wrists steady for 2 seconds')
            self._publish_status()
            return True

    def _tick_height_calibration(self, now_ns):
        calibration = self._height_calibration
        if calibration is None:
            return
        means = calibration.tick(now_ns)
        if means is not None:
            try:
                self.bridge.pose_mapper.calibrate_height(means, self._height_reference)
                calibration.accept(means)
                self._calibration_input_error = None
            except Exception as exc:
                calibration.fail(str(exc))
        if calibration.state != self._height_reported_state:
            if calibration.state == 'calibrated':
                LOG.warning('Height calibration succeeded: %s; press s to start teleop', calibration.means)
            elif calibration.state == 'failed':
                LOG.warning('Height calibration failed: %s; previous calibration retained', calibration.error)
            self._height_reported_state = calibration.state

    def request_start(self, *, reason: str = 'hand_tracking_target_s') -> bool:
        with self._lock:
            if self._closed or self._phase != "armed":
                return False
            if self._height_calibration and self._height_calibration.state == 'collecting':
                LOG.warning('Wait for height calibration before pressing s')
                return False
            if self._height_calibration and self._height_calibration.state == 'failed' and self._height_calibration.means is None:
                LOG.warning('Height calibration failed without a previous result; press c to retry')
                return False
            if not bool(getattr(self._session_client, "startup_ready", False)):
                return False
            if self._height_calibration and (self._last_error or self._calibration_input_error):
                LOG.warning('Cannot start while an unresolved source error remains')
                return False
            now_ns = self._now_ns()
            try:
                self.bridge.start(now_ns=now_ns)
                if self._height_calibration:
                    self._publish_status()
                self._session_client.request_start(reason)
            except Exception as exc:
                self._set_error(f"target start rejected: {exc}")
                try:
                    self.bridge.reset()
                except Exception:
                    LOG.exception("failed to reset target bridge after start rejection")
                self._publish_status()
                return False
            self._last_error = None
            self._phase = "start_pending"
            self._publish_status()
            return True

    def request_return(self, reason: str = "operator_return") -> bool:
        with self._lock:
            if self._phase not in {"start_pending", "teleop"}:
                return False
            self._request_return_locked(reason)
            self._publish_status()
            return self._phase == "returning"

    def set_clutch(self, side: str, pressed: bool) -> bool:
        """Forward a validated XR clutch edge to the selected pose mapper."""
        with self._lock:
            mapper = getattr(self.bridge, "pose_mapper", None)
            setter = getattr(mapper, "set_clutch", None)
            if not callable(setter) or side not in _SIDES:
                return False
            try:
                setter(side, bool(pressed))
            except Exception as exc:
                self._set_error(f"XR clutch rejected: {exc}")
                self._publish_status()
                return False
            return True

    def on_key(self, value: str) -> None:
        if value == 'c':
            self.request_height_calibration()
        elif value == "s":
            if self._phase == "armed":
                self.request_start()
            else:
                self.request_return("hand_tracking_target_s_return")
        elif value in {"q", "\x03"}:
            if self._phase in {"start_pending", "teleop"}:
                self.request_return("hand_tracking_target_quit")
            else:
                self._stop_event.set()

    def _finish_return_locked(self) -> None:
        try:
            self.bridge.reset()
        except Exception as exc:
            self._set_error(f"target bridge reset failed: {exc}")
            self._phase = "fault"
            return
        self._phase = "armed"
        self._return_deadline_ns = None
        self._last_error = None

    def tick(self, *, now_ns: int | None = None) -> BridgeTargets:
        with self._lock:
            now_ns = self._now_ns() if now_ns is None else int(now_ns)
            self._session_client.poll()
            self._tick_height_calibration(now_ns)
            if self._phase == "returning":
                if bool(getattr(self._session_client, "return_completion_fresh", False)):
                    self._finish_return_locked()
                elif self._return_deadline_ns is not None and now_ns >= self._return_deadline_ns:
                    self._set_error("coordinator return completion timeout")
                    self._phase = "fault"
                    self._return_deadline_ns = None
                    try:
                        self.bridge.reset()
                    except Exception:
                        LOG.exception("failed to reset target bridge after return timeout")
                self._publish_status()
                return BridgeTargets()
            if self._phase == "start_pending":
                if bool(getattr(self._session_client, "start_authorized", False)):
                    self._phase = "teleop"
                elif getattr(self._session_client, "pending_intent_sequence", None) is None:
                    try:
                        self.bridge.reset()
                    except Exception:
                        LOG.exception("failed to reset target bridge after start cancellation")
                    self._phase = "armed"
                self._publish_status()
                return BridgeTargets()
            if self._phase != "teleop":
                self._publish_status()
                return BridgeTargets()
            if not self._session_client.start_authorized:
                self._set_error("coordinator teleop authorization lost")
                self._request_return_locked("coordinator_authorization_lost")
                self._publish_status()
                return BridgeTargets()
            try:
                targets = self.bridge.tick(now_ns=now_ns)
                for item in targets.hand:
                    self._publisher.publish_hand_target(
                        side=item.side,
                        keypoints_m=item.keypoints_m,
                        source_timestamp_ns=item.source_timestamp_ns,
                        source=TARGET_SOURCE_ID,
                    )
                for item in targets.arm:
                    self._publisher.publish_arm_target(
                        side=item.side,
                        position_m=item.pose[:3],
                        orientation_xyzw=item.pose[3:],
                        elbow_reference_direction=item.elbow_reference_direction,
                        source_timestamp_ns=item.source_timestamp_ns,
                        source=TARGET_SOURCE_ID,
                        **({'tracking_valid': False} if not item.tracking_valid else {}),
                    )
            except Exception as exc:
                self._input_rejected_locked("target bridge rejected", exc)
                targets = BridgeTargets()
            self._publish_status()
            return targets if self._phase == "teleop" else BridgeTargets()

    def run(self, stop_event: threading.Event | None = None) -> None:
        stop_event = self._stop_event if stop_event is None else stop_event
        next_tick = time.monotonic()
        while not stop_event.is_set() and not self._closed:
            next_tick += self.period_s
            self.tick()
            stop_event.wait(max(0.0, next_tick - time.monotonic()))

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._stop_event.set()
            try:
                self.bridge.reset()
            except Exception:
                LOG.exception("failed to reset target bridge during close")
            for resource in self._subscriptions:
                try:
                    resource.close()
                except Exception:
                    try:
                        resource.undeclare()
                    except Exception:
                        pass
            self._subscriptions.clear()
            try:
                self._publisher.close()
            except Exception:
                LOG.exception("failed to close target publisher")
            try:
                self._session_client.close()
            except Exception:
                LOG.exception("failed to close session client")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="simulation-only Manus/PICO target publisher")
    parser.add_argument("--config", required=True, help="canonical hand-tracking target YAML")
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="stop after this many seconds (diagnostic/testing)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    config = _load_config(args.config)
    select_arm_pose_mapper(config, os.environ.get('TIANJI_ARM_POSE_MAPPER'))
    select_xr_arm_input(config, os.environ.get("TIANJI_XR_ARM_INPUT"))
    processor_override = os.environ.get('TIANJI_ARM_TARGET_PROCESSOR')
    if processor_override is not None:
        if processor_override not in ('passthrough', 'conditioned'):
            raise ValueError('invalid TIANJI_ARM_TARGET_PROCESSOR')
        config['arm_target_processor'] = processor_override
        if processor_override == 'passthrough':
            config['arm_target_processor_config'] = {}
    if os.environ.get("TIANJI_HAND_MODE") == "disabled":
        config["active_hand_sides"] = ()
    if args.duration is not None and (not np.isfinite(args.duration) or args.duration <= 0.0):
        raise ValueError("duration must be finite and positive")
    required_profile = os.environ.get("TIANJI_REQUIRED_OBSERVATION_PROFILE")
    if required_profile is not None and config["input_profile"] != required_profile:
        raise ValueError(
            f"target entry requires input_profile={required_profile!r}, "
            f"got {config['input_profile']!r}"
        )
    session = open_session()
    node: HandTrackingTargetNode | None = None
    gesture_binding = None
    xr_operator_binding = None
    stop_event = threading.Event()
    old_handlers = {signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)}

    def request_stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    for signum in old_handlers:
        signal.signal(signum, request_stop)
    keyboard_thread: threading.Thread | None = None
    duration_thread: threading.Thread | None = None
    try:
        router_zid = require_single_router(session, os.environ.get("TIANJI_ROUTER_ZID") or None)
        component_instance_id = os.environ.get("TIANJI_COMPONENT_INSTANCE_ID", "hand-tracking-target")
        observation_instance_id = os.environ.get(
            "TIANJI_OBSERVATION_PUBLISHER_INSTANCE_ID", "hand-tracking-observation"
        )
        coordinator_instance_id = os.environ.get("TIANJI_COORDINATOR_INSTANCE_ID", "")
        if not coordinator_instance_id:
            raise ValueError("TIANJI_COORDINATOR_INSTANCE_ID is required")
        node = HandTrackingTargetNode(
            config=config,
            session=session,
            router_zid=router_zid,
            publisher_instance_id=component_instance_id,
            observation_publisher_instance_id=observation_instance_id,
            coordinator_instance_id=coordinator_instance_id,
        )
        from .pico_gesture_start import bind_from_environment
        gesture_binding = bind_from_environment(os.environ, session=session, node=node)
        from .xr_operator import bind_from_environment as bind_xr_operator_from_environment
        xr_operator_binding = bind_xr_operator_from_environment(
            os.environ, session=session, node=node, config=config
        )

        def on_key(value: str) -> None:
            if node is not None:
                node.on_key(value)
                if node.stop_event.is_set():
                    stop_event.set()

        keyboard_thread = threading.Thread(
            target=raw_keyboard,
            args=(on_key, stop_event),
            name="hand-tracking-target-keyboard",
            daemon=True,
        )
        keyboard_thread.start()
        if args.duration is not None:
            def stop_after_duration() -> None:
                if not stop_event.wait(args.duration):
                    stop_event.set()

            duration_thread = threading.Thread(
                target=stop_after_duration,
                name="hand-tracking-target-duration",
                daemon=True,
            )
            duration_thread.start()
        node.run(stop_event)
        return 0
    finally:
        stop_event.set()
        try:
            if gesture_binding is not None:
                gesture_binding.close()
            if xr_operator_binding is not None:
                xr_operator_binding.close()
        finally:
            if node is not None:
                node.close()
        if keyboard_thread is not None:
            keyboard_thread.join(timeout=1.0)
        if duration_thread is not None:
            duration_thread.join(timeout=1.0)
        session.close()
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)


__all__ = [
    "HandTrackingTargetNode",
    "TARGET_SOURCE_ID",
    "_load_config",
    "create_bridge_from_config",
    "select_xr_arm_input",
    "main",
]


if __name__ == "__main__":
    raise SystemExit(main())
