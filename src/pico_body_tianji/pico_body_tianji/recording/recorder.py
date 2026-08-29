"""Passive Zenoh recorder for session-v1 typed messages."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Callable

from ..protocol import topics
from ..protocol.messages import (
    ArmJointCommand,
    ArmJointState,
    ArmTargetCommand,
    HandJointCommand,
    HandJointState,
    HandTargetCommand,
    RawH5ReplaySample,
    RawMocapLiveSample,
    RawPicoControllerSample,
    SessionState,
    ComponentStatus,
    strict_loads,
)
from ..zenoh_util import declare_component_liveliness
from .session_h5 import SessionH5Writer


class RecorderProtocolError(ValueError):
    """A malformed or profile-incompatible message reached the recorder."""


_RAW = {
    "pico_controller": (topics.RAW_PICO_CONTROLLER, RawPicoControllerSample),
    "mocap_live": (topics.RAW_MOCAP_LIVE, RawMocapLiveSample),
    "h5_replay": (topics.RAW_H5_REPLAY, RawH5ReplaySample),
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


def _key_map() -> dict[str, tuple[type[Any], str, str | None]]:
    result: dict[str, tuple[type[Any], str, str | None]] = {}
    for side in ("left", "right"):
        result[topics.arm_target(side)] = (ArmTargetCommand, "append_arm_target", side)
        result[topics.hand_target(side)] = (HandTargetCommand, "append_hand_target", side)
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
        self._status_sequence = 0
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
            flush_interval_s=flush_interval_s,
            clock=clock or __import__("time").monotonic_ns,
        )
        self._resources: list[Any] = []
        selected_raw = _RAW.get(source_type)
        if selected_raw is not None:
            self._declare(selected_raw[0])
        key_map = _key_map()
        for key in key_map:
            self._declare(key)
        self._publish_status(ready=True, healthy=True)

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
        if self._closed:
            raise RuntimeError("recorder is closed")
        try:
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
            getattr(self.writer, method_name)(message, received_time_ns=received_time_ns)
            return message
        except RecorderProtocolError as exc:
            self._failed = exc
            raise
        except Exception as exc:
            error = RecorderProtocolError(f"failed to record {key}: {exc}")
            self._failed = error
            raise error from exc

    def flush(self) -> None:
        self.writer.flush()

    def close(self) -> None:
        if self._closed:
            return
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
        if self._failed is None:
            self.writer.close()
        else:
            self.writer.abort()
        self._closed = True

    def abort(self) -> None:
        if self._closed:
            return
        self._failed = self._failed or RecorderProtocolError("recorder aborted")
        self.close()

    def __enter__(self) -> "SessionRecorderNode": return self
    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.abort() if exc_type is not None or self._failed is not None else self.close()


__all__ = ["RecorderProtocolError", "SessionRecorderNode"]
