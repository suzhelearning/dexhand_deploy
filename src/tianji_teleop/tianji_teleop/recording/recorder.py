"""Passive Zenoh recorder for session-v1 typed messages."""
from __future__ import annotations

import json
import math
import threading
from pathlib import Path
from typing import Any, Callable, Mapping

from ..protocol import topics
from ..protocol.messages import (
    ArmInputObservation as ArmInputObservationWire,
    ArmJointCommand,
    ArmJointState,
    ArmTargetCommand,
    HandJointCommand,
    HandJointState,
    HandSkeletonObservation,
    HandTargetCommand,
    RawH5ReplaySample,
    RawMocapLiveSample,
    SessionState,
    ComponentStatus,
    strict_loads,
)
from ..hand_tracking.models import (
    ArmInputObservation as ArmInputObservationModel,
    HandObservation,
    LegacyPicoPalmFrame,
    ManusRawFrame,
    PicoRawFrame,
)
from ..zenoh_util import declare_component_liveliness
from .session_h5 import EXTENDED_SCHEMA_VERSION, SCHEMA_NAME, SCHEMA_VERSION, SessionH5Writer

class RecorderProtocolError(ValueError):
    """A malformed or profile-incompatible message reached the recorder."""


_RAW = {
    "mocap_live": (topics.RAW_MOCAP_LIVE, RawMocapLiveSample),
    "h5_replay": (topics.RAW_H5_REPLAY, RawH5ReplaySample),
}

_EXTENDED_RAW = {
    topics.RAW_PICO_HAND_TRACKING: (PicoRawFrame, "append_raw_pico", "pico"),
    topics.RAW_MANUS_HAND_TRACKING: (ManusRawFrame, "append_raw_manus", "manus"),
    topics.RAW_LEGACY_PICO_PALM: (LegacyPicoPalmFrame, "append_raw_legacy_palm", "manus"),
}
_EXTENDED_SOURCE_PROFILE = {
    "hand_tracking_observation": None,
    "hand_tracking_sim": "pico",
    "hand_tracking_sim_manus": "manus",
}


def _payload(value: Any) -> Any:

    if isinstance(value, dict):
        return value
    if isinstance(value, (bytes, bytearray, memoryview)):
        return strict_loads(bytes(value))
    payload = getattr(value, "payload", None)
    if payload is not None:
        return strict_loads(bytes(payload))
    return value


def _wire_mapping(value: Any, *, key: str) -> tuple[dict[str, Any], str]:
    """Decode an extended raw payload and keep its transport router identity."""
    decoded = _payload(value)
    if not isinstance(decoded, Mapping):
        raise RecorderProtocolError(f"raw payload for {key} must be an object")
    payload = dict(decoded)
    router = payload.pop("router_zid", None)
    if not isinstance(router, str) or not router:
        raise RecorderProtocolError(f"raw payload for {key} is missing router_zid")
    return payload, router


def _hand_observation_model(message: HandSkeletonObservation) -> HandObservation:
    return HandObservation(
        source=message.source,
        side=message.side,
        source_instance_id=message.source_instance_id,
        source_sequence=message.source_sequence,
        source_timestamp_ns=message.source_timestamp_ns,
        received_timestamp_ns=message.received_timestamp_ns,
        receiver_instance_id=message.receiver_instance_id,
        receiver_frame_sequence=message.receiver_frame_sequence,
        coordinate_frame=message.coordinate_frame,
        mapping_version=message.mapping_version,
        keypoints_m=message.keypoints_m,
        joint_valid=message.joint_valid,
        valid=message.valid,
        wrist_pose=message.wrist_pose,
        frame_association_id=message.frame_association_id,
    )


def _arm_input_observation_model(message: ArmInputObservationWire) -> ArmInputObservationModel:
    return ArmInputObservationModel(
        source=message.source,
        side=message.side,
        tracked_frame=message.tracked_frame,
        reference_frame=message.reference_frame,
        pose=message.pose,
        valid=message.valid,
        source_timestamp_ns=message.source_timestamp_ns,
        received_timestamp_ns=message.received_timestamp_ns,
        receiver_instance_id=message.receiver_instance_id,
        receiver_frame_sequence=message.receiver_frame_sequence,
        mapping_version=message.mapping_version,
        frame_association_id=message.frame_association_id,
        source_sequence=message.source_sequence,
        source_instance_id=message.source_instance_id,
    )
_RECORDING_CONFIG_KEYS = frozenset({"flush_interval_s", "schema_name", "schema_version"})


def _recording_config(value: Mapping[str, Any] | None, *, flush_interval_s: float) -> dict[str, Any]:
    if value is None:
        value = {
            "flush_interval_s": flush_interval_s,
            "schema_name": SCHEMA_NAME,
            "schema_version": SCHEMA_VERSION,
        }
    if not isinstance(value, Mapping) or set(value) != _RECORDING_CONFIG_KEYS:
        raise ValueError(
            "recording config must contain exactly flush_interval_s, schema_name and schema_version"
        )
    try:
        interval = float(value["flush_interval_s"])
    except (TypeError, ValueError) as exc:
        raise ValueError("recording flush_interval_s must be numeric") from exc
    if not math.isfinite(interval) or interval <= 0:
        raise ValueError("recording flush_interval_s must be finite and positive")
    if value["schema_name"] != SCHEMA_NAME or value["schema_version"] not in {
        SCHEMA_VERSION,
        EXTENDED_SCHEMA_VERSION,
    }:
        raise ValueError("unsupported session recording schema")
    return {
        "flush_interval_s": interval,
        "schema_name": str(value["schema_name"]),
        "schema_version": str(value["schema_version"]),
    }


def _key_map() -> dict[str, tuple[type[Any], str, str | None]]:
    result: dict[str, tuple[type[Any], str, str | None]] = {}
    for side in ("left", "right"):
        result[topics.arm_target(side)] = (ArmTargetCommand, "append_arm_target", side)
        result[topics.hand_target(side)] = (HandTargetCommand, "append_hand_target", side)
        result[topics.hand_observation(side)] = (HandSkeletonObservation, "append_hand_observation", side)
        result[topics.arm_input_observation(side)] = (
            ArmInputObservationWire,
            "append_arm_input_observation",
            side,
        )
        result[topics.arm_command(side)] = (ArmJointCommand, "append_arm_command", side)
        result[topics.hand_command(side)] = (HandJointCommand, "append_hand_command", side)
        result[topics.hand_state(side)] = (HandJointState, "append_hand_state", side)
    result[topics.ARM_STATE] = (ArmJointState, "append_arm_state", None)
    result[topics.SESSION_STATE] = (SessionState, "append_session_state", None)
    return result


class SessionRecorderNode:
    """Record only the typed streams selected by ``source_type``.

    This node never republishes messages and has no authority over session
    state.  ``receive`` is public both for Zenoh callbacks and deterministic
    """

    def __init__(
        self,
        session: Any,
        output_path: str | Path,
        *,
        source_type: str,
        robot_model: str,
        router_zid: str,
        publisher_instance_id: str | None = None,
        flush_interval_s: float = 1.0,
        recording_config: Mapping[str, Any] | None = None,
        input_profile: str | None = None,
        metadata: Mapping[str, Any] | None = None,
        clock: Callable[[], int] | None = None,
    ) -> None:
        publisher_instance_id = publisher_instance_id or __import__("os").environ.get("TIANJI_COMPONENT_INSTANCE_ID") or "recorder"
        if not publisher_instance_id:
            raise ValueError("publisher_instance_id is required")
        self.session = session
        self.publisher_instance_id = publisher_instance_id
        self.router_zid = router_zid
        self.source_type = source_type
        self._failed: RecorderProtocolError | None = None
        self._closed = False
        self._writer_lock = threading.RLock()
        self._status_sequence = 0
        self.recording_config = _recording_config(
            recording_config,
            flush_interval_s=flush_interval_s,
        )
        self.input_profile = input_profile
        if self.recording_config["schema_version"] == EXTENDED_SCHEMA_VERSION:
            expected_profile = _EXTENDED_SOURCE_PROFILE.get(source_type, "invalid")
            if expected_profile == "invalid":
                raise ValueError(f"schema 1.1 does not support source_type: {source_type}")
            if input_profile not in {"pico", "manus"}:
                raise ValueError("schema 1.1 recorder requires input_profile='pico' or 'manus'")
            if expected_profile is not None and input_profile != expected_profile:
                raise ValueError(
                    f"source_type={source_type} requires input_profile={expected_profile!r}"
                )
        metadata_value = dict(metadata or {})
        if self.recording_config["schema_version"] == EXTENDED_SCHEMA_VERSION:
            metadata_value.setdefault(
                "recording_mode",
                "control_and_observation"
                if source_type in {"hand_tracking_sim", "hand_tracking_sim_manus"}
                else "observation_only",
            )
            metadata_value.setdefault("input_profile", input_profile)
            metadata_value.setdefault("source_type", source_type)
        self._liveliness_token = (
            declare_component_liveliness(
                session, role="recorder", logical_id="session_recorder", instance_id=publisher_instance_id
            ) if session is not None else None
        )
        self._status_publisher = (
            session.declare_publisher(topics.RECORDER_STATUS)
            if session is not None and hasattr(session, "declare_publisher")
            else None
        )
        self.writer = SessionH5Writer(
            output_path,
            source_type=source_type,
            robot_model=robot_model,
            router_zid=router_zid,
            flush_interval_s=self.recording_config["flush_interval_s"],
            schema_name=self.recording_config["schema_name"],
            schema_version=self.recording_config["schema_version"],
            clock=clock or __import__("time").monotonic_ns,
            metadata=metadata_value,
        )
        self._resources: list[Any] = []
        selected_raw = _RAW.get(source_type)
        if selected_raw is not None:
            self._declare(selected_raw[0])
        if self.recording_config["schema_version"] == EXTENDED_SCHEMA_VERSION:
            for key in self._selected_extended_raw_keys():
                self._declare(key)
        key_map = _key_map()
        for key in key_map:
            if self.recording_config["schema_version"] != EXTENDED_SCHEMA_VERSION and (
                key.startswith("tianji/observation/")
            ):
                continue
            self._declare(key)
        self._publish_status(ready=True, healthy=True)

    def _selected_extended_raw_keys(self) -> tuple[str, ...]:
        if self.recording_config["schema_version"] != EXTENDED_SCHEMA_VERSION:
            return ()
        return tuple(
            key
            for key, (_message_type, _method_name, profile) in _EXTENDED_RAW.items()
            if profile == self.input_profile
        )

    def _publish_status(self, *, ready: bool, healthy: bool, error: str | None = None) -> None:
        if self._status_publisher is None:
            return
        self._status_sequence += 1
        status = ComponentStatus(
            1, self._status_sequence, __import__("time").monotonic_ns(),
            "recorder", "session_recorder", "ready" if ready else "fault",
            ready, healthy, ["simulation"], error, {},
            self.publisher_instance_id, self.router_zid,
        )
        payload = json.dumps(status.to_dict(), separators=(",", ":")).encode("utf-8")
        try:
            self._status_publisher.put(payload, encoding="application/json")
        except TypeError:
            self._status_publisher.put(payload)

    @property
    def failed(self) -> bool:
        return self._failed is not None

    @property
    def failure(self) -> RecorderProtocolError | None:
        return self._failed

    def _declare(self, key: str) -> None:
        try:
            resource = self.session.declare_subscriber(key, lambda sample, key=key: self._on_sample(key, sample))
        except TypeError:
            resource = self.session.declare_subscriber(key, lambda sample: self._on_sample(key, sample))
        self._resources.append(resource)

    def _on_sample(self, key: str, sample: Any) -> None:
        with self._writer_lock:
            if not self._closed:
                self.receive(key, sample)
    def _validate_router(self, message: Any) -> None:
        router = getattr(message, "router_zid", None)
        if router is None:
            envelope = getattr(message, "envelope", None)
            router = getattr(envelope, "router_zid", None)
        if router != self.router_zid:
            raise RecorderProtocolError("message router_zid does not match recorder router")

    def receive(self, key: str, payload: Any, *, received_time_ns: int | None = None) -> Any:
        """Parse and append one sample; unknown keys/types fail closed."""
        with self._writer_lock:
            return self._receive_locked(key, payload, received_time_ns=received_time_ns)

    def _receive_locked(self, key: str, payload: Any, *, received_time_ns: int | None = None) -> Any:
        if self._closed:
            raise RuntimeError("recorder is closed")
        try:
            extended_raw = _EXTENDED_RAW.get(key)
            if extended_raw is not None:
                if self.recording_config["schema_version"] != EXTENDED_SCHEMA_VERSION:
                    raise RecorderProtocolError(
                        f"extended raw key is unavailable in schema {self.recording_config['schema_version']}: {key}"
                    )
                if key not in self._selected_extended_raw_keys():
                    raise RecorderProtocolError(
                        f"raw key is not allowed by profile {self.source_type}: {key}"
                    )
                message_type, method_name, _profile = extended_raw
                raw_value, router = _wire_mapping(payload, key=key)
                if router != self.router_zid:
                    raise RecorderProtocolError(
                        f"raw payload router_zid does not match recorder router: {key}"
                    )
                message = message_type.from_dict(raw_value)
                getattr(self.writer, method_name)(message, received_time_ns=received_time_ns)
                return message
            if key in {item[0] for item in _RAW.values()}:
                selected = _RAW.get(self.source_type)
                if selected is None or key != selected[0]:
                    raise RecorderProtocolError(f"raw key is not allowed by profile {self.source_type}: {key}")
                message_type = selected[1]
                message = payload if isinstance(payload, message_type) else message_type.from_dict(_payload(payload))
                self._validate_router(message)
                if getattr(message, "source_type", None) != self.source_type:
                    raise RecorderProtocolError(f"raw source_type mismatch: expected {self.source_type}")
                self.writer.append(message, received_time_ns=received_time_ns)
                return message
            specification = _key_map().get(key)
            if specification is None:
                raise RecorderProtocolError(f"unknown recorder key: {key}")
            message_type, method_name, side = specification
            message = payload if isinstance(payload, message_type) else message_type.from_dict(_payload(payload))
            self._validate_router(message)
            if side is not None and message.side != side:
                raise RecorderProtocolError(f"topic side does not match payload: {key}")
            if isinstance(message, HandSkeletonObservation):
                if self.recording_config["schema_version"] != EXTENDED_SCHEMA_VERSION:
                    raise RecorderProtocolError("hand observations require session HDF5 schema 1.1")
                if message.source != self.input_profile:
                    raise RecorderProtocolError(
                        f"hand observation source does not match recorder profile: {message.source}"
                    )
                message_for_writer = _hand_observation_model(message)
            elif isinstance(message, ArmInputObservationWire):
                if self.recording_config["schema_version"] != EXTENDED_SCHEMA_VERSION:
                    raise RecorderProtocolError("arm observations require session HDF5 schema 1.1")
                expected_source = "pico" if self.input_profile == "pico" else "legacy_pico_palm"
                if message.source != expected_source:
                    raise RecorderProtocolError(
                        f"arm observation source does not match recorder profile: {message.source}"
                    )
                message_for_writer = _arm_input_observation_model(message)
            else:
                message_for_writer = message
            getattr(self.writer, method_name)(message_for_writer, received_time_ns=received_time_ns)
            return message
        except RecorderProtocolError as exc:
            self._failed = exc
            raise
        except Exception as exc:
            error = RecorderProtocolError(f"failed to record {key}: {exc}")
            self._failed = error
            raise error from exc

    def flush(self) -> None:
        with self._writer_lock:
            if not self._closed:
                self.writer.flush()

    def close(self) -> None:
        # Serialize whole frame appends, not merely individual HDF5 calls.
        # Release the lock before undeclare: it may wait for a queued callback.
        with self._writer_lock:
            if self._closed:
                return
            self._closed = True
            if self._failed is None:
                self.writer.close()
            else:
                self.writer.abort()
        for resource in self._resources:
            try:
                resource.undeclare()
            except Exception:
                try:
                    resource.close()
                except Exception:
                    pass
        self._resources.clear()
        self._publish_status(ready=False, healthy=self._failed is None, error=str(self._failed) if self._failed else None)
        if self._liveliness_token is not None:
            try:
                self._liveliness_token.undeclare()
            except Exception:
                pass
            self._liveliness_token = None
        if self._status_publisher is not None:
            try:
                self._status_publisher.undeclare()
            except Exception:
                pass
            self._status_publisher = None

    def abort(self) -> None:
        if self._closed:
            return
        self._failed = self._failed or RecorderProtocolError("recorder aborted")
        self.close()

    def __enter__(self) -> "SessionRecorderNode": return self
    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.abort() if exc_type is not None or self._failed is not None else self.close()


__all__ = ["RecorderProtocolError", "SessionRecorderNode"]
