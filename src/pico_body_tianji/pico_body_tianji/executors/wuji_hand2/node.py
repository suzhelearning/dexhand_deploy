"""Canonical Wuji Hand 2 executor.

``retarget`` 只消费 wrist-relative keypoints 并发布 HandJointCommand；
``direct`` 只消费授权 publisher 的 HandJointCommand。两种模式在构造时
互斥选择，executor 不重发 direct command，也不发布 SessionState。
"""
from __future__ import annotations

import json
import math
import os
from typing import Any, Mapping, Sequence
import time

import numpy as np

from ...protocol import topics
from ...protocol.messages import (
    ComponentStatus,
    HAND_JOINT_NAMES,
    HandExecutorStatus,
    HandJointCommand,
    HandJointState,
    HandTargetCommand,
    ProtocolError,
    SafetyStopAck,
    SafetyStopRequest,
    SessionState,
    strict_loads,
)
from ...sources.common.real_admission import RealCapabilityInput, parse_real_capability
from .config import WujiHandConfig


HAND_TIMEOUT_S = 0.5


def _put(publisher: Any, payload: Mapping[str, Any]) -> None:
    if publisher is None:
        return
    data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    try:
        publisher.put(data, encoding="application/json")
    except TypeError:
        publisher.put(data)


def _payload(value: Any) -> Mapping[str, Any]:
    value = getattr(value, "payload", value)
    return value if isinstance(value, Mapping) else strict_loads(bytes(value))


def _retarget_keypoints(points: Sequence[Sequence[float]], config: WujiHandConfig) -> list[float]:
    """Simple finite geometry retargeter used by dry/headless execution.

    It consumes relative points, but also normalizes an arbitrary point array by
    its wrist so a caller cannot accidentally make output depend on world
    translation. The vendor retargeter remains behind the C++ SDK boundary for
    real operation.
    """
    points = np.asarray(points, dtype=np.float64)
    if points.shape != (21, 3) or not np.isfinite(points).all():
        raise ValueError("keypoints must be a finite [21,3] array")
    points = points - points[0]
    values: list[float] = []
    # Thumb has four independent joints. The remaining fingers are four-joint
    # chains; flexion follows segment direction and abduction follows x spread.
    for base in (1, 5, 9, 13, 17):
        chain = points[base : base + 4]
        if chain.shape != (4, 3):
            raise ValueError("keypoint chain is incomplete")
        if base == 1:
            bend = []
            for index in range(3):
                vector = chain[index + 1] - chain[index]
                bend.append(float(np.arctan2(np.linalg.norm(vector[:2]), max(abs(float(vector[2])), 1e-6))))
            joint = [bend[0], float(np.arctan2(chain[1, 1], max(abs(float(chain[1, 0])), 1e-6))), bend[1], bend[2]]
        else:
            segments = np.diff(np.vstack((np.zeros((1, 3)), chain)), axis=0)
            bend = [float(np.arctan2(np.linalg.norm(segment[:2]), max(abs(float(segment[2])), 1e-6))) for segment in segments]
            spread = float(np.arctan2(float(chain[0, 0]), max(abs(float(chain[0, 1])), 1e-6)))
            joint = [bend[0], spread, bend[1], bend[2]]
        values.extend(joint)
    return config.validate_positions(values)


class WujiHandExecutor:
    """One-side hand executor with direct/retarget exclusivity and watchdog."""

    def __init__(
        self,
        *,
        mode: str,
        side: str,
        publisher_instance_id: str,
        router_zid: str,
        authorized_producer: str,
        authorized_publisher_instance_id: str | None = None,
        coordinator_instance_id: str | None = None,
        authorized_target_source: str | None = None,
        config: WujiHandConfig | Mapping[str, Any] | str | os.PathLike[str] | None = None,
        session: Any = None,
        device: Any = None,
        dry_run: bool = True,
        command_timeout_s: float = HAND_TIMEOUT_S,
        run_id: str | None = None,
        safety_supervisor_instance_id: str | None = None,
        real_capability: RealCapabilityInput | Any | None = None,
        clock: Any = time.monotonic_ns,
    ) -> None:
        if mode not in {"direct", "retarget"}:
            raise ValueError("mode must be direct or retarget")
        if side not in {"left", "right"}:
            raise ValueError("side must be left or right")
        if not publisher_instance_id or not router_zid or not authorized_producer or not authorized_publisher_instance_id:
            raise ValueError("hand executor identities are required")
        if command_timeout_s <= 0.0:
            raise ValueError("command_timeout_s must be positive")
        self.mode = mode
        self.side = side
        self.publisher_instance_id = publisher_instance_id
        self.router_zid = router_zid
        self.authorized_producer = authorized_producer
        self.authorized_publisher_instance_id = authorized_publisher_instance_id
        self.coordinator_instance_id = coordinator_instance_id
        self.authorized_target_source = authorized_target_source
        self.config = config if isinstance(config, WujiHandConfig) else WujiHandConfig.from_mapping(config) if isinstance(config, Mapping) else WujiHandConfig.load(config)
        self.session = session
        self.device = device
        self.dry_run = dry_run
        self.command_timeout_ns = int(float(command_timeout_s) * 1e9)
        self.run_id = run_id
        self.safety_supervisor_instance_id = safety_supervisor_instance_id
        self.real_capability = real_capability
        if not dry_run:
            if real_capability is None or not (
                isinstance(real_capability, RealCapabilityInput) or callable(real_capability)
            ):
                raise ValueError("real Wuji executor requires typed real capability input")
            try:
                if not parse_real_capability(real_capability).admitted:
                    raise ValueError("real Wuji capability preflight denied")
            except ValueError:
                raise
            except Exception as exc:
                raise ValueError(f"real Wuji capability preflight denied: {exc}") from exc
        self.clock = clock
        self._state = "idle"
        self._session_sequence = -1
        self._session_received_ns: int | None = None
        self._latest_target: HandTargetCommand | None = None
        self._latest_command: HandJointCommand | None = None
        self._input_received_ns: int | None = None
        self._baseline: tuple[str, int] | None = None
        self._target_baseline: tuple[str, int] | None = None
        self._sequence = 0
        self._status_sequence = 0
        self._safety_locked = False
        self._safety_ack: SafetyStopAck | None = None
        self._healthy = True
        self._qpos = list(self.config.zero_position_rad)
        self._last_command: HandJointCommand | None = None
        self._last_error: str | None = None
        self._last_safety_sequence: int | None = None
        self._subscriptions: list[Any] = []
        self._publishers: dict[str, Any] = {}
        self._live_tokens: dict[str, Any] = {}
        self._setup_transport()
        self._publish_status()

    @property
    def tracking_allowed(self) -> bool:
        return self._tracking_allowed(int(self.clock()))

    @property
    def unhealthy(self) -> bool:
        return not self._healthy

    @property
    def at_zero(self) -> bool:
        return self.config.at_zero(self._qpos)

    @property
    def safety_locked(self) -> bool:
        return self._safety_locked

    @property
    def safety_ack(self) -> SafetyStopAck | None:
        return self._safety_ack

    @property
    def position_rad(self) -> list[float]:
        return list(self._qpos)
    def _setup_transport(self) -> None:
        if self.session is None:
            return
        # Safety/session subscriptions always precede ready status.
        self._subscriptions.extend([
            self.session.declare_subscriber(topics.SESSION_STATE, self.on_session_state),
            self.session.declare_subscriber(topics.SAFETY_STOP, self.on_safety_stop),
        ])
        if self.mode == "retarget":
            self._subscriptions.append(self.session.declare_subscriber(topics.hand_target(self.side), self.on_hand_target))
        else:
            self._subscriptions.append(self.session.declare_subscriber(topics.hand_command(self.side), self.on_hand_command))
        self._publishers = {
            "state": self.session.declare_publisher(topics.hand_state(self.side)),
            "status": self.session.declare_publisher(topics.hand_executor_status(self.side)),
            "component": self.session.declare_publisher(topics.EXECUTOR_STATUS),
            "safety_ack": self.session.declare_publisher(topics.safety_ack(self.publisher_instance_id)),
        }
        if self.mode == "retarget":
            self._publishers["command"] = self.session.declare_publisher(topics.hand_command(self.side))
        if hasattr(self.session, "liveliness"):
            executor_logical = f"wuji_{self.side}"
            self._live_tokens["executor"] = self.session.liveliness().declare_token(
                f"tj/live/executor/hand/{executor_logical}/{self.publisher_instance_id}"
            )
            if self.mode == "retarget":
                self._live_tokens["producer"] = self.session.liveliness().declare_token(
                    f"tj/live/producer/hand/{self.authorized_producer}/{self.publisher_instance_id}"
                )

    def on_session_state(self, value: SessionState | Mapping[str, Any] | Any) -> None:
        try:
            state = value if isinstance(value, SessionState) else SessionState.from_dict(_payload(value))
            if state.router_zid != self.router_zid:
                raise ProtocolError("session state router_zid mismatch")
            if self.coordinator_instance_id is None or state.publisher_instance_id != self.coordinator_instance_id:
                raise ProtocolError("session state coordinator identity mismatch")
            now = int(self.clock())
            if state.timestamp_ns > now or now - state.timestamp_ns > self.command_timeout_ns:
                raise ProtocolError("session state is stale")
            if state.sequence <= self._session_sequence:
                raise ProtocolError("session state sequence rollback")
            self._session_sequence = state.sequence
            self._session_received_ns = now
            self._state = state.state
            if self._state in {"returning", "fault"}:
                self._latest_target = None
                self._latest_command = None
                self._input_received_ns = None
        except (ProtocolError, TypeError, ValueError) as exc:
            self._mark_unhealthy(f"invalid session state: {exc}")

    def _mark_unhealthy(self, reason: str) -> None:
        self._healthy = False
        self._last_error = reason
        self._state = "fault"
        self._latest_target = None
        self._latest_command = None
    def _teleop_state_fresh(self, now_ns: int) -> bool:
        return bool(
            self._state == "teleop"
            and self._session_received_ns is not None
            and 0 <= now_ns - self._session_received_ns <= self.command_timeout_ns
        )

    def _expire_to_return(self, reason: str) -> None:
        if self._state == "teleop":
            self._state = "returning"
        self._healthy = False
        self._last_error = reason
        self._latest_target = None
        self._latest_command = None
        self._input_received_ns = None

    def _require_fresh_teleop(self) -> None:
        now = int(self.clock())
        if self._state != "teleop":
            raise ProtocolError("hand input is accepted only in teleop")
        if not self._teleop_state_fresh(now):
            self._expire_to_return("coordinator teleop state is stale")
            raise ProtocolError("coordinator teleop state is stale")


    def on_hand_target(self, value: HandTargetCommand | Mapping[str, Any] | Any) -> bool:
        if self.mode != "retarget" or self._safety_locked:
            return False
        try:
            target = value if isinstance(value, HandTargetCommand) else HandTargetCommand.from_dict(_payload(value))
            self._require_fresh_teleop()
            if target.router_zid != self.router_zid or target.side != self.side:
                raise ProtocolError("hand target identity mismatch")
            if self.authorized_target_source is not None and target.source != self.authorized_target_source:
                raise ProtocolError("hand target source is not authorized")
            if self.authorized_publisher_instance_id is not None and target.publisher_instance_id != self.authorized_publisher_instance_id:
                raise ProtocolError("hand target publisher instance is not authorized")
            if self._target_baseline is not None:
                old_instance, old_sequence = self._target_baseline
                if target.publisher_instance_id != old_instance or target.sequence <= old_sequence:
                    raise ProtocolError("hand target identity or sequence rollback")
            _retarget_keypoints(target.keypoints_m, self.config)
            self._target_baseline = (target.publisher_instance_id, target.sequence)
            self._latest_target = target
            self._input_received_ns = int(self.clock())
            return True
        except (ProtocolError, TypeError, ValueError) as exc:
            self._last_error = f"invalid hand target: {exc}"
            self._latest_target = None
            self._input_received_ns = None
            self._state = "returning"
            return False

    def rebind_authority(self, producer: str, publisher_instance_id: str) -> None:
        """Launcher-authorized replacement, only after safe return."""
        if self._state not in {"returning", "fault"}:
            raise RuntimeError("hand authority may only rebind while returning or fault")
        if not self.at_zero:
            raise RuntimeError("hand authority rebind requires at_zero")
        if not producer or not publisher_instance_id:
            raise ValueError("producer and publisher_instance_id are required")
        self.authorized_producer = producer
        self.authorized_publisher_instance_id = publisher_instance_id
        self._baseline = None
        self._target_baseline = None
        self._healthy = True
    def on_hand_command(self, value: HandJointCommand | Mapping[str, Any] | Any) -> bool:
        if self.mode != "direct" or self._safety_locked:
            return False
        try:
            command = value if isinstance(value, HandJointCommand) else HandJointCommand.from_dict(_payload(value))
            self._require_fresh_teleop()
            if command.router_zid != self.router_zid or command.side != self.side or command.producer != self.authorized_producer:
                raise ProtocolError("hand command producer/side/router is not authorized")
            if self.authorized_publisher_instance_id is not None and command.publisher_instance_id != self.authorized_publisher_instance_id:
                raise ProtocolError("hand command publisher instance is not authorized")
            positions = self.config.validate_positions(command.position_rad)
            if self._baseline is not None:
                old_instance, old_sequence = self._baseline
                if command.publisher_instance_id != old_instance or command.sequence <= old_sequence:
                    raise ProtocolError("hand command identity or sequence rollback")
            # Only accepted commands update baseline and watchdog.
            self._baseline = (command.publisher_instance_id, command.sequence)
            self._latest_command = command
            self._input_received_ns = int(self.clock())
            return True
        except (ProtocolError, TypeError, ValueError) as exc:
            self._mark_unhealthy(f"invalid hand command: {exc}")
            return False

    def on_safety_stop(self, value: SafetyStopRequest | Mapping[str, Any] | Any) -> bool:
        if self._safety_locked:
            return False
        if not self.safety_supervisor_instance_id or not self.run_id:
            self._last_error = "safety stop rejected: authorization is not configured"
            return False
        try:
            request = value if isinstance(value, SafetyStopRequest) else SafetyStopRequest.from_dict(_payload(value))
            request.validate_authority(self.safety_supervisor_instance_id, self.run_id)
            if request.envelope.router_zid != self.router_zid:
                raise ProtocolError("safety stop router_zid mismatch")
            if self._last_safety_sequence is not None and request.envelope.sequence <= self._last_safety_sequence:
                raise ProtocolError("safety stop sequence rollback")
        except (ProtocolError, TypeError, ValueError) as exc:
            self._last_error = f"invalid safety stop: {exc}"
            return False
        self._safety_locked = True
        self._healthy = False
        self._state = "fault"
        self._latest_target = None
        self._last_safety_sequence = request.envelope.sequence
        self._latest_command = None
        self._last_error = request.reason
        if self.device is not None:
            for name in ("soft_stop", "disable", "stop"):
                callback = getattr(self.device, name, None)
                if callable(callback):
                    try:
                        callback()
                    except TypeError:
                        callback(True)
                    break
        self._safety_ack = SafetyStopAck(
            ProtocolEnvelope(1, self.publisher_instance_id, self.router_zid, request.envelope.sequence, int(self.clock())),
            self.publisher_instance_id, self.run_id, True, request.reason,
        )
        self._publish_status()
        _put(self._publishers.get("safety_ack"), self._safety_ack.to_dict())
        return True

    def _tracking_allowed(self, now_ns: int) -> bool:
        return bool(
            not self._safety_locked and self._healthy and self._state == "teleop"
            and self._teleop_state_fresh(now_ns)
            and self._input_received_ns is not None
            and 0 <= now_ns - self._input_received_ns <= self.command_timeout_ns
        )
    def _real_admission_ok(self) -> bool:
        if self.dry_run:
            return True
        try:
            return parse_real_capability(self.real_capability).admitted
        except Exception as exc:
            self._last_error = f"real capability denied: {exc}"
            return False


    def _send(self, values: Sequence[float]) -> None:
        values = self.config.validate_positions(values)
        self._qpos = list(values)
        if self.device is not None and not self._safety_locked:
            send = getattr(self.device, "send", None)
            if callable(send):
                send(list(values))
    def tick(self, *, now_ns: int | None = None) -> HandJointCommand | None:
        now_ns = int(self.clock()) if now_ns is None else int(now_ns)
        self._sequence += 1
        if self._safety_locked:
            self._publish_state(now_ns)
            return None
        if self._state == "teleop" and not self._teleop_state_fresh(now_ns):
            self._expire_to_return("coordinator teleop state expired")
        if not self._real_admission_ok():
            self._mark_unhealthy(self._last_error or "real capability denied")
            self._publish_state(now_ns)
            self._publish_status()
            return None
        command: HandJointCommand | None = None
        if self._tracking_allowed(now_ns):
            if self.mode == "retarget" and self._latest_target is not None:
                values = _retarget_keypoints(self._latest_target.keypoints_m, self.config)
                command = HandJointCommand(
                    1, self._sequence, now_ns, self.authorized_producer, self.side,
                    list(self.config.joint_names), values, self.publisher_instance_id, self.router_zid,
                )
                self._last_command = command
                _put(self._publishers.get("command"), command.to_dict())
                self._send(values)
            elif self.mode == "direct" and self._latest_command is not None:
                values = self.config.validate_positions(self._latest_command.position_rad)
                self._send(values)
                command = self._latest_command
        else:
            self._state = "returning" if self._state not in {"fault", "idle"} else self._state
            current = np.asarray(self._qpos, dtype=np.float64)
            zero = np.asarray(self.config.zero_position_rad, dtype=np.float64)
            step = np.asarray(self.config.zero_tolerance_rad, dtype=np.float64) * 0.2
            values = (current + np.clip(zero - current, -step, step)).tolist()
            self._send(values)
            if self.config.at_zero(values):
                self._state = "fault" if self._state == "fault" else "returning"
        self._publish_state(now_ns)
        self._publish_status()
        return command

    def _publish_state(self, now_ns: int) -> None:
        state = HandJointState(
            1, self._sequence, now_ns, "wuji_hand2", self.side,
            list(HAND_JOINT_NAMES[self.side]), list(self._qpos), None,
            self.publisher_instance_id, self.router_zid,
        )
        _put(self._publishers.get("state"), state.to_dict())
    def _publish_status(self) -> None:
        now = int(self.clock())
        self._status_sequence += 1
        healthy = self._healthy and not self._safety_locked
        tracking = self._tracking_allowed(now)
        status = HandExecutorStatus(
            1, self._status_sequence, now, self.side,
            healthy, healthy, self.at_zero, tracking, self._last_error,
            self.publisher_instance_id, self.router_zid,
        )
        _put(self._publishers.get("status"), status.to_dict())
        roles = (
            (("producer_hand", "executor_hand") if self.mode == "retarget" else ("executor_hand",))
        )
        for role in roles:
            component = ComponentStatus(
                1, self._status_sequence, now, role,
                f"wuji_hand2_{self.side}_{role.removeprefix('producer_').removeprefix('executor_')}",
                self.mode, healthy, healthy,
                ["simulation"] if self.dry_run else ["real"], self._last_error,
                {"side": self.side, "mode": self.mode, "at_zero": self.at_zero, "tracking_allowed": tracking},
                self.publisher_instance_id, self.router_zid,
            )
            _put(self._publishers.get("component"), component.to_dict())

    def run(self, *, rate_hz: float = 100.0) -> None:
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        period = 1.0 / float(rate_hz)
        next_tick = time.monotonic()
        while True:
            next_tick += period
            self.tick()
            time.sleep(max(0.0, next_tick - time.monotonic()))

    def close(self) -> None:
        for resource in (*self._subscriptions, *self._publishers.values()):
            try:
                if resource is not None:
                    resource.undeclare()
            except (AttributeError, RuntimeError):
                pass
        self._subscriptions.clear()
        self._publishers.clear()
        for token in self._live_tokens.values():
            try:
                if token is not None:
                    token.undeclare()
            except (AttributeError, RuntimeError):
                pass
        self._live_tokens.clear()
        if self.device is not None:
            close = getattr(self.device, "close", None)
            if callable(close):
                close()


__all__ = ["WujiHandExecutor", "WujiHandConfig", "_retarget_keypoints"]
