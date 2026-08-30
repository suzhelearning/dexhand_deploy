"""Canonical MuJoCo arm/hand executor.

该模块只消费 coordinator 的 final command，并发布执行器 state/status。它不
发布 ``SessionState`` 或任何 final command；headless 模式与有窗口模式共享同一
control tick，方便在没有显示服务器时验证真实的 command/safety 行为。
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import logging
import os
from pathlib import Path
import time
from typing import Any, Mapping

import numpy as np

from ...coordination.arm_command_coordinator import ArmRobotConfig
from ...protocol import topics
from ...protocol.messages import (
    ALL_ARM_JOINT_NAMES,
    ARM_JOINT_NAMES,
    HAND_JOINT_NAMES,
    ArmJointCommand,
    ArmJointState,
    ComponentStatus,
    HandJointCommand,
    HandExecutorStatus,
    HandJointState,
    LatchedBool,
    ProtocolEnvelope,
    ProtocolError,
    SafetyStopAck,
    SafetyStopRequest,
    SessionState,
    strict_loads,
)
from ...mujoco_urdf import portable_mujoco_urdf
from ..wuji_hand2.config import WujiHandConfig
from ...zenoh_util import key, open_session, require_single_router, declare_component_liveliness

_LOG = logging.getLogger(__name__)
SIDES = ("left", "right")


def _put(publisher: Any, payload: Mapping[str, Any]) -> None:
    if publisher is None:
        return
    data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    try:
        publisher.put(data, encoding="application/json")
    except TypeError:
        publisher.put(data)


def _declare_publisher(session: Any, topic_name: str) -> Any:
    if session is None or not hasattr(session, "declare_publisher"):
        return None
    return session.declare_publisher(topic_name)


def _declare_subscriber(session: Any, topic_name: str, callback: Any) -> Any:
    if session is None or not hasattr(session, "declare_subscriber"):
        return None
    return session.declare_subscriber(topic_name, callback)


def _sample_payload(sample: Any) -> Mapping[str, Any]:
    payload = getattr(sample, "payload", sample)
    if isinstance(payload, Mapping):
        return payload
    return strict_loads(bytes(payload))

def _joint_id(model: Any, name: str) -> int:
    names = getattr(model, "joint_names", None)
    if isinstance(names, Mapping):
        try:
            return int(names[name])
        except KeyError:
            return -1
    ids = getattr(model, "_ids", None)
    if isinstance(ids, Mapping):
        try:
            return int(ids[name])
        except KeyError:
            return -1
    try:
        import mujoco
        return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
    except (ImportError, AttributeError, TypeError):
        return -1


def _validate_model_joints(
    model: Any,
    names: tuple[str, ...],
    lower: tuple[float, ...],
    upper: tuple[float, ...],
    *,
    aliases: Mapping[str, str] | None = None,
) -> dict[str, int]:
    """Resolve canonical names and reject missing, duplicate or too-tight joints."""
    addresses = getattr(model, "jnt_qposadr", None)
    if addresses is None:
        raise ValueError("MuJoCo model lacks jnt_qposadr")
    resolved: dict[str, int] = {}
    used_ids: set[int] = set()
    for index, canonical in enumerate(names):
        candidate = canonical
        joint_id = _joint_id(model, candidate)
        if joint_id < 0 and aliases:
            candidate = aliases.get(canonical, canonical)
            joint_id = _joint_id(model, candidate)
        if joint_id < 0:
            raise ValueError(f"MuJoCo model missing joint: {canonical}")
        if joint_id in used_ids:
            raise ValueError(f"MuJoCo model resolves duplicate joint: {canonical}")
        used_ids.add(joint_id)
        address = int(addresses[joint_id])
        resolved[canonical] = address
        ranges = getattr(model, "jnt_range", None)
        limited = getattr(model, "jnt_limited", None)
        # The robot config remains the narrower authority. A model whose physical
        # range cannot contain that authority is unsafe and must fail closed.
        if ranges is not None and (limited is None or bool(limited[joint_id])):
            model_lower, model_upper = float(ranges[joint_id][0]), float(ranges[joint_id][1])
            if model_lower > lower[index] + 1e-6 or model_upper < upper[index] - 1e-6:
                raise ValueError(
                    f"MuJoCo joint limit mismatch for {canonical}: "
                    f"model=({model_lower},{model_upper}) config=({lower[index]},{upper[index]})"
                )
    if len(resolved) != len(names) or len(set(resolved.values())) != len(names):
        raise ValueError("MuJoCo joint mapping contains duplicate qpos addresses")
    return resolved


@dataclass(frozen=True)
class _Pending:
    command: ArmJointCommand | HandJointCommand
    received_ns: int


class MujocoExecutor:
    """Apply canonical radian commands to a MuJoCo model."""

    def __init__(
        self,
        *,
        session: Any = None,
        model: Any,
        data: Any,
        publisher_instance_id: str,
        router_zid: str,
        coordinator_instance_id: str,
        hand_config: WujiHandConfig | Mapping[str, Any] | str | os.PathLike[str] | None = None,
        hand_sides: tuple[str, ...] = SIDES,
        hand_overlay: bool = False,
        run_id: str | None = None,
        safety_supervisor_instance_id: str | None = None,
        command_timeout_s: float = 0.2,
        clock: Any = time.monotonic_ns,
    ) -> None:
        if not publisher_instance_id or not router_zid or not coordinator_instance_id:
            raise ValueError("executor, router, and coordinator identities are required")
        if command_timeout_s <= 0.0:
            raise ValueError("command_timeout_s must be positive")
        self.session = session
        self.model = model
        self.data = data
        self.publisher_instance_id = publisher_instance_id
        self.router_zid = router_zid
        self.coordinator_instance_id = coordinator_instance_id
        self.run_id = run_id
        self.safety_supervisor_instance_id = safety_supervisor_instance_id
        self.command_timeout_ns = int(float(command_timeout_s) * 1e9)
        self.clock = clock
        self.robot = ArmRobotConfig.load()
        self.hand_config = (
            hand_config if isinstance(hand_config, WujiHandConfig)
            else WujiHandConfig.from_mapping(hand_config)
            if isinstance(hand_config, Mapping)
            else WujiHandConfig.load(hand_config)
        )
        self.hand_sides = tuple(hand_sides)
        self.hand_overlay = bool(hand_overlay)
        if any(side not in SIDES for side in self.hand_sides):
            raise ValueError("hand_sides must contain left/right")
        self._arm_addresses = {
            side: _validate_model_joints(
                model,
                tuple(getattr(self.robot, f"{side}_joint_names")),
                self.robot.lower_limits_rad,
                self.robot.upper_limits_rad,
            )
            for side in SIDES
        }
        # Legacy URDF uses *_finger_* for non-thumb joints. This mapping is an
        # adapter detail; canonical protocol names never carry the alias.
        hand_addresses: dict[str, dict[str, int]] = {}
        hand_lower = self.hand_config.lower_limits_rad
        hand_upper = self.hand_config.upper_limits_rad
        for side in self.hand_sides:
            canonical = tuple(
                f"{'l' if side == 'left' else 'r'}_{name[2:]}"
                for name in self.hand_config.joint_names
            )
            aliases = {
                name: name.replace("_mcp_", "_finger_mcp_").replace("_pip", "_finger_pip").replace("_dip", "_finger_dip")
                for name in canonical if "thumb_" not in name
            }
            hand_addresses[side] = _validate_model_joints(
                model, canonical, hand_lower, hand_upper, aliases=aliases
            )
        self._hand_addresses = hand_addresses
        qpos = np.asarray(getattr(data, "qpos"), dtype=np.float64)
        if qpos.ndim != 1:
            raise ValueError("MuJoCo data.qpos must be one-dimensional")
        self._qpos = qpos
        self._pending_arm: dict[str, _Pending] = {}
        self._pending_hand: dict[str, _Pending] = {}
        self._arm_baseline: dict[str, tuple[str, int]] = {}
        self._hand_baseline: dict[str, tuple[str, int]] = {}
        self._session_state: SessionState | None = None
        self._at_home: LatchedBool | None = None
        self._return_complete: LatchedBool | None = None
        self._session_received_ns: int | None = None
        self._latch_received_ns: dict[str, int] = {}
        self._latest_received_ns: dict[tuple[str, str], int] = {}
        self._state_sequence = 0
        self._command_count = 0
        self._status_sequence = 0
        self._safety_locked = False
        self._safety_ack: SafetyStopAck | None = None
        self._healthy = True
        self._last_error: str | None = None
        self._subscriptions: list[Any] = []
        self._publishers: dict[str, Any] = {}
        self._liveliness_token = declare_component_liveliness(
            session, role="executor/arm", logical_id="mujoco", instance_id=publisher_instance_id
        ) if session is not None else None
        self._snapshot_timeout_ns = max(self.command_timeout_ns, 1_000_000_000)
        self._snapshot_attempt = 0
        self._snapshot_query_started_ns: int | None = None
        self._snapshot_seen: set[str] = set()
        self._snapshot_values: dict[str, Any] = {}
        self._snapshot_failed = False
        self._snapshot_ready = session is None or not hasattr(session, "get")
        self._setup_transport()
        self._initialize_home()
        self._status = self._make_status(
            ready=self._snapshot_ready, healthy=self._snapshot_ready, phase="ready"
        )
        self._publish_status()

    @property
    def safety_locked(self) -> bool:
        return self._safety_locked

    @property
    def safety_ack(self) -> SafetyStopAck | None:
        return self._safety_ack

    @property
    def arm_state(self) -> ArmJointState:
        values = [float(self._qpos[address]) for side in SIDES for name in getattr(self.robot, f"{side}_joint_names") for address in (self._arm_addresses[side][name],)]
        return ArmJointState(
            1, self._state_sequence, int(self.clock()), "mujoco",
            list(ALL_ARM_JOINT_NAMES), values, None,
            self.publisher_instance_id, self.router_zid,
        )

    @property
    def status(self) -> ComponentStatus:
        return self._status

    def hand_state(self, side: str) -> HandJointState:
        if side not in self._hand_addresses:
            raise ValueError(f"hand side is not enabled: {side}")
        values = [float(self._qpos[address]) for name, address in self._hand_addresses[side].items()]
        return HandJointState(
            1, self._state_sequence, int(self.clock()), "mujoco", side,
            list(HAND_JOINT_NAMES[side]), values, None,
            self.publisher_instance_id, self.router_zid,
        )

    def _setup_transport(self) -> None:
        if self.session is None:
            return
        # State/safety subscribers are deliberately declared before ready status.
        self._subscriptions.extend([
            _declare_subscriber(self.session, topics.SESSION_STATE, self.on_session_state),
            _declare_subscriber(self.session, topics.AT_HOME, self.on_at_home),
            _declare_subscriber(self.session, topics.RETURN_COMPLETE, self.on_return_complete),
            _declare_subscriber(self.session, topics.SAFETY_STOP, self.on_safety_stop),
        ])
        self._subscriptions.extend([
            _declare_subscriber(self.session, topics.arm_command("left"), self.on_arm_command),
            _declare_subscriber(self.session, topics.arm_command("right"), self.on_arm_command),
        ])
        for side in self.hand_sides:
            self._subscriptions.append(_declare_subscriber(self.session, topics.hand_command(side), self.on_hand_command))
        self._publishers = {
            "arm_state": _declare_publisher(self.session, topics.ARM_STATE),
            "status": _declare_publisher(self.session, topics.EXECUTOR_STATUS),
            "safety_ack": _declare_publisher(self.session, topics.safety_ack(self.publisher_instance_id)),
        }
        if not self.hand_overlay:
            for side in self.hand_sides:
                self._publishers[f"hand_state_{side}"] = _declare_publisher(self.session, topics.hand_state(side))
                self._publishers[f"hand_status_{side}"] = _declare_publisher(self.session, topics.hand_executor_status(side))
        if hasattr(self.session, "get"):
            self._query_coordinator_snapshot()

    def _reply_payload(self, reply: Any) -> Mapping[str, Any]:
        if not getattr(reply, "ok", False):
            raise ProtocolError("coordinator snapshot reply is not successful")
        result = getattr(reply, "result", None)
        if result is None:
            raise ProtocolError("successful coordinator snapshot reply has no result")
        return _sample_payload(result)

    def _on_snapshot_reply(self, key_name: str, reply: Any, attempt: int) -> None:
        if attempt != self._snapshot_attempt:
            return
        if key_name in self._snapshot_seen:
            self._snapshot_failed = True
            self._snapshot_ready = False
            self._last_error = f"duplicate coordinator snapshot reply: {key_name}"
            return
        self._snapshot_seen.add(key_name)
        try:
            payload = self._reply_payload(reply)
            if key_name == "state":
                parsed: SessionState | LatchedBool = SessionState.from_dict(payload)
                if parsed.publisher_instance_id != self.coordinator_instance_id:
                    raise ProtocolError("snapshot coordinator instance mismatch")
            elif key_name in {"at_home", "return_complete"}:
                parsed = LatchedBool.from_dict(payload)
                if parsed.publisher_instance_id != self.coordinator_instance_id:
                    raise ProtocolError("snapshot coordinator instance mismatch")
            else:
                raise ProtocolError(f"unknown coordinator snapshot key: {key_name}")
            now_ns = int(self.clock())
            if parsed.router_zid != self.router_zid:
                raise ProtocolError("snapshot router_zid mismatch")
            if parsed.timestamp_ns > now_ns or now_ns - parsed.timestamp_ns > self._snapshot_timeout_ns:
                raise ProtocolError(f"stale coordinator snapshot: {key_name}")
            current = self._snapshot_values.get(key_name)
            if current is not None and parsed.sequence <= current.sequence:
                # A subscriber callback may have delivered newer authority while
                # the query was in flight. The snapshot still satisfies this
                # key, but must never roll that value back.
                if key_name == "state":
                    self._session_state = current
                elif key_name == "at_home":
                    self._at_home = current
                else:
                    self._return_complete = current
                self._snapshot_seen.add(key_name)
                if len(self._snapshot_seen) == 3 and not self._snapshot_failed:
                    self._snapshot_ready = True
                return
            if key_name == "state":
                self._session_state = parsed
                self._session_received_ns = now_ns
            elif key_name == "at_home":
                self._at_home = parsed
                self._latch_received_ns[key_name] = now_ns
            else:
                self._return_complete = parsed
                self._latch_received_ns[key_name] = now_ns
            self._snapshot_values[key_name] = parsed
            if len(self._snapshot_seen) == 3 and not self._snapshot_failed:
                self._snapshot_ready = True
        except (ProtocolError, TypeError, ValueError) as exc:
            self._snapshot_failed = True
            self._snapshot_ready = False
            self._last_error = f"invalid coordinator snapshot {key_name}: {exc}"

    def _query_coordinator_snapshot(self) -> None:
        self._snapshot_attempt += 1
        attempt = self._snapshot_attempt
        self._snapshot_query_started_ns = int(self.clock())
        self._snapshot_seen.clear()
        self._snapshot_failed = False
        self._snapshot_ready = False
        keys = (
            ("state", topics.SESSION_STATE),
            ("at_home", topics.AT_HOME),
            ("return_complete", topics.RETURN_COMPLETE),
        )
        for key_name, snapshot_key in keys:
            try:
                self.session.get(
                    snapshot_key,
                    lambda reply, key_name=key_name, attempt=attempt: self._on_snapshot_reply(
                        key_name, reply, attempt
                    ),
                    timeout=1.0,
                )
            except (AttributeError, RuntimeError, TypeError) as exc:
                self._snapshot_failed = True
                self._last_error = f"coordinator snapshot query failed: {exc}"

    def _initialize_home(self) -> None:
        for side in SIDES:
            for name, value in zip(getattr(self.robot, f"{side}_joint_names"), getattr(self.robot, f"{side}_home_rad")):
                self._qpos[self._arm_addresses[side][name]] = value
        for side, addresses in self._hand_addresses.items():
            for index, address in enumerate(addresses.values()):
                self._qpos[address] = self.hand_config.zero_position_rad[index]
        try:
            import mujoco
            mujoco.mj_forward(self.model, self.data)
        except (ImportError, AttributeError, TypeError):
            pass

    def on_session_state(self, value: SessionState | Mapping[str, Any] | Any) -> None:
        try:
            state = value if isinstance(value, SessionState) else SessionState.from_dict(_sample_payload(value))
            if state.router_zid != self.router_zid or state.publisher_instance_id != self.coordinator_instance_id:
                raise ProtocolError("session state coordinator identity mismatch")
            now_ns = int(self.clock())
            if state.timestamp_ns > now_ns or now_ns - state.timestamp_ns > self._snapshot_timeout_ns:
                raise ProtocolError("session state is stale")
            if self._session_state is not None:
                if state.sequence < self._session_state.sequence:
                    raise ProtocolError("session state sequence rollback")
                if state.sequence == self._session_state.sequence:
                    return
            self._session_state = state
            self._session_received_ns = now_ns
            self._snapshot_values["state"] = state
        except (ProtocolError, TypeError, ValueError) as exc:
            self._healthy = False
            self._last_error = f"invalid session state: {exc}"

    def _observe_latch(self, name: str, value: LatchedBool | Mapping[str, Any] | Any) -> None:
        latch = value if isinstance(value, LatchedBool) else LatchedBool.from_dict(_sample_payload(value))
        if latch.router_zid != self.router_zid or latch.publisher_instance_id != self.coordinator_instance_id:
            raise ProtocolError(f"{name} coordinator identity mismatch")
        previous = self._snapshot_values.get(name)
        if isinstance(previous, LatchedBool) and latch.sequence < previous.sequence:
            raise ProtocolError(f"{name} sequence rollback")
        now_ns = int(self.clock())
        if latch.timestamp_ns > now_ns or now_ns - latch.timestamp_ns > self._snapshot_timeout_ns:
            raise ProtocolError(f"{name} is stale")
        self._snapshot_values[name] = latch
        self._latch_received_ns[name] = now_ns

    def on_at_home(self, value: LatchedBool | Mapping[str, Any] | Any) -> None:
        try:
            self._observe_latch("at_home", value)
            self._at_home = self._snapshot_values["at_home"]
        except (ProtocolError, TypeError, ValueError) as exc:
            self._healthy = False
            self._last_error = f"invalid at_home latch: {exc}"

    def on_return_complete(self, value: LatchedBool | Mapping[str, Any] | Any) -> None:
        try:
            self._observe_latch("return_complete", value)
            self._return_complete = self._snapshot_values["return_complete"]
        except (ProtocolError, TypeError, ValueError) as exc:
            self._healthy = False
            self._last_error = f"invalid return_complete latch: {exc}"

    def _accept_sequence(self, kind: str, side: str, instance: str, sequence: int) -> None:
        baseline = self._arm_baseline if kind == "arm" else self._hand_baseline
        previous = baseline.get(side)
        if previous is not None:
            old_instance, old_sequence = previous
            if instance != old_instance:
                raise ProtocolError(f"{kind} executor input instance changed")
            if sequence <= old_sequence:
                raise ProtocolError(f"{kind} command sequence rollback")
        baseline[side] = (instance, sequence)

    def on_arm_command(self, value: ArmJointCommand | Mapping[str, Any] | Any) -> bool:
        try:
            command = value if isinstance(value, ArmJointCommand) else ArmJointCommand.from_dict(_sample_payload(value))
            if command.router_zid != self.router_zid or command.producer != "coordinator" or command.publisher_instance_id != self.coordinator_instance_id:
                raise ProtocolError("arm command coordinator identity mismatch")
            lower = getattr(self.robot, "lower_limits_rad")
            upper = getattr(self.robot, "upper_limits_rad")
            if any(value < lo or value > hi for value, lo, hi in zip(command.position_rad, lower, upper)):
                raise ProtocolError("arm command exceeds robot hard limits")
            self._accept_sequence("arm", command.side, command.publisher_instance_id, command.sequence)
            received = int(self.clock())
            self._pending_arm[command.side] = _Pending(command, received)
            self._latest_received_ns[("arm", command.side)] = received
            return True
        except (ProtocolError, TypeError, ValueError) as exc:
            self._healthy = False
            self._last_error = f"invalid arm command: {exc}"
            self._publish_status()
            return False

    def on_hand_command(self, value: HandJointCommand | Mapping[str, Any] | Any) -> bool:
        try:
            command = value if isinstance(value, HandJointCommand) else HandJointCommand.from_dict(_sample_payload(value))
            if command.router_zid != self.router_zid:
                raise ProtocolError("hand command router_zid mismatch")
            positions = self.hand_config.validate_positions(command.position_rad)
            self._accept_sequence("hand", command.side, command.publisher_instance_id, command.sequence)
            received = int(self.clock())
            self._pending_hand[command.side] = _Pending(command, received)
            self._latest_received_ns[("hand", command.side)] = received
            return True
        except (ProtocolError, TypeError, ValueError) as exc:
            self._healthy = False
            self._last_error = f"invalid hand command: {exc}"
            self._publish_status()
            return False

    def on_safety_stop(self, value: SafetyStopRequest | Mapping[str, Any] | Any) -> bool:
        if self.safety_supervisor_instance_id is None or self.run_id is None:
            self._last_error = "safety stop rejected: authorization is not configured"
            return False
        try:
            request = value if isinstance(value, SafetyStopRequest) else SafetyStopRequest.from_dict(_sample_payload(value))
            request.validate_authority(self.safety_supervisor_instance_id, self.run_id)
            if request.envelope.router_zid != self.router_zid:
                raise ProtocolError("safety stop router_zid mismatch")
            if self._safety_ack is not None and request.envelope.sequence <= self._safety_ack.envelope.sequence:
                raise ProtocolError("safety stop sequence rollback")
        except (ProtocolError, TypeError, ValueError) as exc:
            self._last_error = f"invalid safety stop: {exc}"
            return False
        self._safety_locked = True
        self._healthy = False
        self._last_error = request.reason
        self._safety_ack = SafetyStopAck(
            ProtocolEnvelope(1, self.publisher_instance_id, self.router_zid, request.envelope.sequence, int(self.clock())),
            self.publisher_instance_id, self.run_id, True, request.reason,
        )
        self._publish_status()
        self._publish("safety_ack", self._safety_ack.to_dict())
        return True

    def _publish(self, name: str, payload: Mapping[str, Any]) -> None:
        _put(self._publishers.get(name), payload)

    def _make_status(self, *, ready: bool, healthy: bool, phase: str) -> ComponentStatus:
        self._status_sequence += 1
        return ComponentStatus(
            1, self._status_sequence, int(self.clock()), "executor_arm", "mujoco",
            phase, ready and self._healthy, healthy and self._healthy, ["simulation"], self._last_error,
            {"headless": True, "safety_locked": self._safety_locked, "hand_overlay": self.hand_overlay, "commands_sent": self._command_count},
            self.publisher_instance_id, self.router_zid,
        )

    def _publish_status(self) -> None:
        status = self._make_status(ready=self._snapshot_ready and not self._safety_locked, healthy=self._snapshot_ready and not self._safety_locked, phase="soft_stopped" if self._safety_locked else ("waiting_snapshot" if not self._snapshot_ready else "ready"))
        self._status = status
        self._publish("status", status.to_dict())

    def _publish_states(self) -> None:
        self._publish("arm_state", self.arm_state.to_dict())
        self._publish_status()
        now = int(self.clock())
        session_fresh = bool(
            self._session_received_ns is not None
            and 0 <= now - self._session_received_ns <= self._snapshot_timeout_ns
        )
        if self.hand_overlay:
            return
        for side in self.hand_sides:
            state = self.hand_state(side)
            self._publish(f"hand_state_{side}", state.to_dict())
            ready = self._snapshot_ready and not self._safety_locked
            healthy = ready and self._healthy
            tracking = bool(
                healthy
                and session_fresh
                and self._session_state is not None
                and self._session_state.state == "teleop"
            )
            self._publish(
                f"hand_status_{side}",
                HandExecutorStatus(
                    1, self._state_sequence, now, side, ready, healthy,
                    self.hand_config.at_zero(state.position_rad), tracking,
                    self._last_error, self.publisher_instance_id, self.router_zid,
                ).to_dict(),
            )

    def tick(self, *, now_ns: int | None = None) -> dict[str, Any]:
        now_ns = int(self.clock()) if now_ns is None else int(now_ns)
        self._state_sequence += 1
        if not self._snapshot_ready:
            started = self._snapshot_query_started_ns
            if started is None or now_ns - started >= self._snapshot_timeout_ns:
                self._query_coordinator_snapshot()
            self._publish_states()
            return {"arm": {}, "hand": {}}
        if self._safety_locked:
            self._publish_states()
            return {"arm": {}, "hand": {}}
        applied: dict[str, Any] = {"arm": {}, "hand": {}}
        for side, pending in tuple(self._pending_arm.items()):
            command = pending.command
            if now_ns - command.timestamp_ns > self.command_timeout_ns:
                self._last_error = f"arm command stale: {side}"
                continue
            values = np.asarray(command.position_rad, dtype=np.float64)
            if not np.isfinite(values).all():
                self._last_error = f"arm command nonfinite: {side}"
                continue
            for name, value in zip(getattr(self.robot, f"{side}_joint_names"), values):
                self._qpos[self._arm_addresses[side][name]] = float(value)
            applied["arm"][side] = command
        for side, pending in tuple(self._pending_hand.items()):
            command = pending.command
            if now_ns - command.timestamp_ns > self.command_timeout_ns:
                self._last_error = f"hand command stale: {side}"
                continue
            values = self.hand_config.validate_positions(command.position_rad)
            for address, value in zip(self._hand_addresses[side].values(), values):
                self._qpos[address] = value
            applied["hand"][side] = command
        self._command_count += len(applied["arm"]) + len(applied["hand"])
        try:
            import mujoco
            mujoco.mj_forward(self.model, self.data)
        except (ImportError, AttributeError, TypeError):
            pass
        self._publish_states()
        return applied

    def run(self, *, headless: bool = True, rate_hz: float = 60.0) -> None:
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        period = 1.0 / float(rate_hz)
        if headless:
            next_tick = time.monotonic()
            while True:
                next_tick += period
                self.tick()
                time.sleep(max(0.0, next_tick - time.monotonic()))
            return
        try:
            import mujoco.viewer
        except ImportError as exc:
            raise RuntimeError("mujoco.viewer is required for the non-headless overlay") from exc
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                started = time.monotonic()
                self.tick()
                viewer.sync()
                time.sleep(max(0.0, period - (time.monotonic() - started)))
    def close(self) -> None:
        if self._liveliness_token is not None:
            try:
                self._liveliness_token.undeclare()
            except Exception:
                pass
            self._liveliness_token = None
        for resource in (*self._subscriptions, *self._publishers.values()):
            if resource is not None:
                try:
                    resource.undeclare()
                except (AttributeError, RuntimeError):
                    pass
        self._subscriptions.clear()
        self._publishers.clear()
def _resolve_configured_urdf(config_path: Path, value: str | os.PathLike[str]) -> Path:
    urdf = Path(value)
    if urdf.is_absolute():
        return urdf
    package_root = config_path.expanduser().resolve().parents[2]
    return package_root / urdf


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="canonical MuJoCo executor")
    parser.add_argument("--headless", action="store_true", help="run without mujoco.viewer")
    parser.add_argument("--hand-overlay", action="store_true", help="consume hand commands without publishing hand authority")
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--urdf", type=Path, default=None)
    parser.add_argument("--rate", type=float, default=60.0)
    parser.add_argument("--run-id", default="")
    parser.add_argument("--hand-sides", default=None, help="comma-separated hand sides; empty disables MuJoCo hand overlay")
    parser.add_argument("--publisher-instance-id", default=os.environ.get("TIANJI_COMPONENT_INSTANCE_ID", "mujoco"))
    parser.add_argument("--coordinator-instance-id", default=os.environ.get("TIANJI_COORDINATOR_INSTANCE_ID", "coordinator"))
    return parser.parse_args(argv)

def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    configured: Mapping[str, Any] = {}
    if args.config is not None:
        import yaml
        configured = yaml.safe_load(args.config.read_text(encoding="utf-8")) or {}
        if not isinstance(configured, Mapping):
            raise SystemExit("MuJoCo config must be a mapping")
        if args.urdf is None and configured.get("urdf"):
            args.urdf = _resolve_configured_urdf(args.config, configured["urdf"])
        if args.rate == 60.0 and configured.get("rate_hz"):
            args.rate = float(configured["rate_hz"])
    if args.hand_sides is None:
        configured_sides = configured.get("hand_sides", ())
        if isinstance(configured_sides, str):
            args.hand_sides = configured_sides
        else:
            args.hand_sides = ",".join(str(side) for side in configured_sides)
    if not args.hand_overlay and configured.get("hand_overlay") == "mujoco":
        args.hand_overlay = True
    hand_sides = tuple(side.strip() for side in str(args.hand_sides).split(",") if side.strip())
    if any(side not in SIDES for side in hand_sides):
        raise SystemExit("--hand-sides must contain only left/right")
    try:
        import mujoco
    except ImportError as exc:
        raise SystemExit(f"mujoco is required for headless executor: {exc}")
    root = Path(__file__).resolve().parents[5]
    urdf = args.urdf or root / "src" / "pico_body_tianji" / "assets" / "tianji_wuji2" / "tianji_wuji2.urdf"
    xml, assets = portable_mujoco_urdf(urdf)
    model = mujoco.MjModel.from_xml_string(xml, assets)
    data = mujoco.MjData(model)
    session = open_session()
    try:
        router = require_single_router(session, os.environ.get("TIANJI_ROUTER_ZID"))
        executor = MujocoExecutor(
            session=session, model=model, data=data,
            publisher_instance_id=args.publisher_instance_id,
            router_zid=router,
            coordinator_instance_id=args.coordinator_instance_id,
            hand_sides=hand_sides,
            hand_overlay=args.hand_overlay,
            run_id=args.run_id or None,
            safety_supervisor_instance_id=os.environ.get("TIANJI_SAFETY_SUPERVISOR_INSTANCE_ID"),
        )
        executor.run(headless=args.headless, rate_hz=args.rate)
    except KeyboardInterrupt:
        return 0
    finally:
        if "executor" in locals():
            executor.close()
        session.close()
    return 0


__all__ = ["MujocoExecutor", "_validate_model_joints", "main"]

if __name__ == "__main__":
    raise SystemExit(main())
