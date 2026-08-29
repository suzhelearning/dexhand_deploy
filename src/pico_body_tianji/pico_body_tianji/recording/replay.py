"""Session-v1 target and joint replay lifecycle nodes.

Replay is a source role, not a second coordinator.  It emits fresh wire
sequence/timestamps while retaining the recorded source timestamp and frame.
The two replay modes are deliberately separate: target replay feeds the IK /
hand-retarget graph, while joint replay feeds final-joint consumers directly.
"""
from __future__ import annotations

from collections.abc import Iterable
import json
import math
import time
from pathlib import Path
from typing import Any, Callable

import yaml

from ..config_loader import canonical_config_root
from ..protocol import topics
from ..protocol.messages import (
    ARM_JOINT_NAMES,
    HAND_JOINT_NAMES,
    ArmJointProposal,
    ComponentStatus,
    HandJointCommand,
    LatchedBool,
    SessionIntent,
    SessionState,
    strict_loads,
)
from ..sources.common.session_client import SessionClient
from ..sources.common.target_publisher import SequenceAllocator, TargetPublisher
from ..zenoh_util import ZenohPub
from .session_h5 import SessionH5Reader


def validate_direct_real_recording(
    recording: str | Path,
    *,
    active_sides: Iterable[str] = ("left", "right"),
    active_hand_sides: Iterable[str] = ("left", "right"),
) -> None:
    """Fail closed before a direct replay can connect a real executor.

    A direct replay is a trusted-file boundary: it must be a complete,
    canonical session-v1 file and every command frame must match the current
    robot/hand names, finite values, and immutable hard limits.  Runtime
    command validation remains necessary, but it must not be the first place
    an invalid file is discovered.
    """
    path = Path(recording).expanduser()
    if path.is_symlink() or not path.is_file():
        raise ValueError("direct real replay requires a trusted regular HDF5 file")
    sides = tuple(active_sides)
    hand_sides = tuple(active_hand_sides)
    allowed = {"left", "right"}
    if (
        not sides
        or any(side not in allowed for side in sides)
        or len(set(sides)) != len(sides)
        or any(side not in allowed for side in hand_sides)
        or len(set(hand_sides)) != len(hand_sides)
    ):
        raise ValueError("direct real replay sides must be unique left/right values")
    reader: SessionH5Reader | None = None
    try:
        reader = SessionH5Reader(path)
        attrs = reader.attrs
        if attrs.get("source_type") != "joint_replay":
            raise ValueError("direct real replay requires source_type=joint_replay")
        root = canonical_config_root()
        arm_config = yaml.safe_load((root / "robot" / "arm.yaml").read_text(encoding="utf-8")) or {}
        hand_config = yaml.safe_load((root / "robot" / "wuji_hand2.yaml").read_text(encoding="utf-8")) or {}
        arm_lower = tuple(float(value) for value in arm_config["lower_limits_rad"])
        arm_upper = tuple(float(value) for value in arm_config["upper_limits_rad"])
        if len(arm_lower) != 7 or len(arm_upper) != 7 or any(
            not math.isfinite(value) or lower >= upper
            for value, lower, upper in zip(arm_lower, arm_lower, arm_upper)
        ):
            raise ValueError("robot arm hard limits are invalid")
        hand_lower = tuple(float(value) for value in hand_config["lower_limits_rad"])
        hand_upper = tuple(float(value) for value in hand_config["upper_limits_rad"])
        if len(hand_lower) != 20 or len(hand_upper) != 20 or any(
            not math.isfinite(value) or lower >= upper
            for value, lower, upper in zip(hand_lower, hand_lower, hand_upper)
        ):
            raise ValueError("Wuji hand hard limits are invalid")

        for side in sides:
            expected_names = tuple(arm_config[f"{side}_joint_names"])
            if expected_names != ARM_JOINT_NAMES[side]:
                raise ValueError(f"robot arm config has non-canonical {side} names")
            rows = reader.read_arm_command(side)
            if not rows:
                raise ValueError(f"direct real replay has no active arm command stream: {side}")
            for index, row in enumerate(rows):
                if tuple(row.get("names", ())) != expected_names:
                    raise ValueError(f"arm command {side}[{index}] has invalid joint order")
                values = row.get("position_rad")
                if (
                    not isinstance(values, (list, tuple))
                    or len(values) != 7
                    or any(
                        not isinstance(value, (int, float))
                        or isinstance(value, bool)
                        or not math.isfinite(float(value))
                        or float(value) < lower
                        or float(value) > upper
                        for value, lower, upper in zip(values, arm_lower, arm_upper)
                    )
                ):
                    raise ValueError(f"arm command {side}[{index}] is non-finite or outside hard limits")
                if row.get("mode") not in {"idle", "teleop", "returning"}:
                    raise ValueError(f"arm command {side}[{index}] has invalid mode")
        for side in hand_sides:
            expected_names = HAND_JOINT_NAMES[side]
            rows = reader.read_hand_command(side)
            if not rows:
                raise ValueError(f"direct real replay has no active hand command stream: {side}")
            for index, row in enumerate(rows):
                if tuple(row.get("names", ())) != expected_names:
                    raise ValueError(f"hand command {side}[{index}] has invalid joint order")
                values = row.get("position_rad")
                if (
                    not isinstance(values, (list, tuple))
                    or len(values) != 20
                    or any(
                        not isinstance(value, (int, float))
                        or isinstance(value, bool)
                        or not math.isfinite(float(value))
                        or float(value) < lower
                        or float(value) > upper
                        for value, lower, upper in zip(values, hand_lower, hand_upper)
                    )
                ):
                    raise ValueError(f"hand command {side}[{index}] is non-finite or outside hard limits")
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        if isinstance(exc, ValueError) and str(exc).startswith("direct real replay"):
            raise
        raise ValueError(f"direct real replay preflight failed: {exc}") from exc
    finally:
        if reader is not None:
            reader.close()


_TARGET_LIVELINESS = "tj/live/source/target_replay"
_JOINT_SOURCE_LIVELINESS = "tj/live/source/joint_replay"
_JOINT_PRODUCER_LIVELINESS = "tj/live/producer/arm/joint_replay"
_JOINT_HAND_PRODUCER_LIVELINESS = "tj/live/producer/hand/joint_replay"


def _payload(value: Any) -> Any:
    if isinstance(value, dict): return value
    if isinstance(value, (bytes, bytearray, memoryview)): return strict_loads(bytes(value))
    raw = getattr(value, "payload", None)
    return strict_loads(bytes(raw)) if raw is not None else value


class _ReplayLifecycle:
    def __init__(
        self,
        *,
        session: Any,
        source: str,
        source_publisher_instance_id: str,
        router_zid: str,
        expected_coordinator_instance_id: str | None,
        rate_hz: float,
        clock: Callable[[], int],
        session_client: SessionClient | None,
    ) -> None:
        if not source or not source_publisher_instance_id or not router_zid: raise ValueError("source, instance and router_zid are required")
        if rate_hz <= 0.0: raise ValueError("rate_hz must be positive")
        self.session = session; self.source = source; self.router_zid = router_zid
        self.source_publisher_instance_id = source_publisher_instance_id; self.rate_hz = float(rate_hz); self._clock = clock
        self._source_allocator = SequenceAllocator()
        self._intent_publisher = ZenohPub(session, topics.SESSION_INTENT) if session is not None else None
        self._session_client = session_client
        if self._session_client is None and expected_coordinator_instance_id and session is not None:
            self._session_client = SessionClient(session, source=source, publisher_instance_id=source_publisher_instance_id, router_zid=router_zid, expected_coordinator_instance_id=expected_coordinator_instance_id, allocator=self._source_allocator, clock=clock)
        self._live_token: Any = None
        self._started = False
        self._closed = False
        self._state: SessionState | None = None
        self._pending_action: str | None = None
        self._pending_intent_sequence: int | None = None
        self._phase = "armed"
        self._paused = False
        self._recorded_time_ns = 0
        self._wall_time_ns: int | None = None
        self._return_requested = False
        self._at_home = False
        self._return_complete = False
    @property
    def phase(self) -> str: return self._phase
    @property
    def paused(self) -> bool: return self._paused
    @property
    def recorded_time_ns(self) -> int: return self._recorded_time_ns
    @property
    def state(self) -> SessionState | None: return self._state
    @property
    def pending_intent_sequence(self) -> int | None: return self._pending_intent_sequence
    @property
    def starts_ik_producer(self) -> bool: return False

    def _register_liveliness(self, key: str) -> None:
        if self.session is None: return
        try:
            self._live_token = self.session.liveliness().declare_token(key)
        except (AttributeError, TypeError):
            self._live_token = None

    def start(self) -> None:
        if self._started: return
        self._started = True
        if self._session_client is not None:
            self._session_client.start()
        if self._phase == "fault":
            self.publish_status(phase="fault", ready=False, healthy=False, error="replay is fault-locked")
        else:
            self.publish_status(phase="armed", ready=True, healthy=True)

    def _send_intent(self, action: str, reason: str) -> int:
        if not self._started: self.start()
        sequence = self._source_allocator.next(); timestamp = int(self._clock())
        if self._session_client is not None:
            if action == "start": sequence = self._session_client.request_start(reason)
            elif action == "return": sequence = self._session_client.request_return(reason)
            else: sequence = self._session_client.request_shutdown(reason)
        elif self._intent_publisher is not None:
            intent = SessionIntent(1, sequence, timestamp, self.source, action, reason, self.source_publisher_instance_id, self.router_zid)
            self._intent_publisher.put_json(intent.to_dict())
        self._pending_action = action; self._pending_intent_sequence = sequence
        if action == "start":
            self._at_home = False; self._return_complete = False; self._phase = "start_pending"
        elif self._phase == "fault":
            # A fault is a terminal local lock.  The coordinator may still
            # need a shutdown/return intent, but that intent must never turn
            # this instance into a returning phase or complete/unlock it.
            self._at_home = False; self._return_complete = False; self._return_requested = True
        else:
            self._at_home = False; self._return_complete = False; self._phase = "returning"; self._return_requested = True
        return sequence
    def request_start(self, reason: str = "replay_start") -> int:
        if self._phase == "fault": raise RuntimeError("replay is fault-locked")
        if self._session_client is not None and not self._session_client.startup_ready:
            self._phase = "armed"; self.publish_status(phase="armed", ready=False, healthy=True, error="coordinator snapshot not ready")
            raise TimeoutError("coordinator snapshot barrier is not ready")
        return self._send_intent("start", reason)
    def request_return(self, reason: str = "replay_complete") -> int:
        if self._session_client is not None and not self._session_client.startup_ready: raise TimeoutError("coordinator snapshot barrier is not ready")
        return self._send_intent("return", reason)
    def request_shutdown(self, reason: str = "replay_shutdown") -> int: return self._send_intent("shutdown", reason)
    def on_session_state(self, state: SessionState | dict[str, Any] | bytes) -> None:
        if not isinstance(state, SessionState): state = SessionState.from_dict(_payload(state))
        if state.router_zid != self.router_zid: return
        if self._session_client is not None and state.publisher_instance_id != self._session_client.expected_coordinator_instance_id: return
        self._state = state
        if state.state == "fault":
            self._phase = "fault"; self._paused = False; self._pending_action = None; return
        if state.state == "returning" and self._phase == "replaying":
            self._phase = "returning"; self._paused = False; return
        if state.state == "idle" and self._phase == "replaying":
            self._phase = "armed"; self._paused = False; return
        if state.state == "idle" and self._phase == "returning" and self._pending_action is None:
            self._phase = "armed"; self._paused = False; self._return_requested = False; return
        pending = self._pending_intent_sequence
        if pending is None or state.intent_sequence != pending: return
        if self._pending_action == "start":
            if self._session_client is not None and not self._session_client.start_authorized and state.state == "teleop": return
            if state.state == "teleop":
                self._phase = "replaying"; self._paused = False; self._recorded_time_ns = 0; self._wall_time_ns = None; self._pending_action = None
            elif state.state in {"idle", "fault"}:
                self._phase = "armed" if state.state == "idle" else "fault"; self._pending_action = None; self._pending_intent_sequence = None
        elif self._pending_action in {"return", "shutdown"} and state.state == "idle":
            self._maybe_finish_return()
    def _sync_authority(self) -> None:
        if self._session_client is None: return
        self._session_client.poll()
        if self._session_client.snapshot_timed_out and self._phase == "start_pending":
            self._pending_action = None; self._pending_intent_sequence = None; self._phase = "armed"
        pending = self._pending_intent_sequence
        if pending is not None and self._session_client.pending_intent_sequence != pending:
            self._pending_action = None; self._pending_intent_sequence = None
            if self._phase in {"start_pending", "returning"}: self._phase = "armed"; self._return_requested = False
        state = self._session_client.state
        if state is not None and (self._state is None or state.sequence != self._state.sequence):
            self.on_session_state(state)
        if self._session_client.at_home is not None: self._at_home = bool(self._session_client.at_home)
        if self._session_client.return_complete is not None: self._return_complete = bool(self._session_client.return_complete)
        self._maybe_finish_return()

    def on_latched(self, value: LatchedBool | dict[str, Any] | bytes, *, kind: str) -> None:
        if not isinstance(value, LatchedBool): value = LatchedBool.from_dict(_payload(value))
        if value.router_zid != self.router_zid: return
        if kind == "at_home": self._at_home = value.value
        elif kind == "return_complete": self._return_complete = value.value
        else: raise ValueError("kind must be at_home or return_complete")
        self._maybe_finish_return()

    def _maybe_finish_return(self) -> None:
        if self._phase == "fault": return
        if self._pending_action in {"return", "shutdown"} and self._session_client is not None and not self._session_client.return_completion_fresh: return
        if self._pending_action in {"return", "shutdown"} and self._state is not None and self._state.state == "idle" and self._at_home and self._return_complete:
            self._phase = "armed"; self._pending_action = None; self._pending_intent_sequence = None; self._paused = False; self._return_requested = False; self.on_return_complete()

    def pause(self) -> None:
        if self._phase != "replaying": raise RuntimeError("replay is not active")
        self._paused = True

    def resume(self) -> None:
        if self._phase != "replaying": raise RuntimeError("replay is not active")
        self._paused = False; self._wall_time_ns = None

    def publish_status(self, *, phase: str, ready: bool, healthy: bool, error: str | None = None) -> None:
        # Implemented by concrete node.
        del phase, ready, healthy, error
    def on_return_complete(self) -> None: pass

    def close(self) -> None:
        if self._closed: return
        self._closed = True
        if self._live_token is not None:
            try: self._live_token.undeclare()
            except Exception: pass
        if self._session_client is not None:
            try: self._session_client.close()
            except Exception: pass
        if self._intent_publisher is not None: self._intent_publisher.close()


class TargetReplaySource(_ReplayLifecycle):
    """Replay target rows as one canonical source-role component."""

    def __init__(
        self,
        recording: str | Path | SessionH5Reader,
        *,
        session: Any = None,
        publisher_instance_id: str,
        router_zid: str,
        active_sides: Iterable[str] = ("left", "right"),
        inactive_sides: Iterable[str] = (),
        active_hand_sides: Iterable[str] = (),
        inactive_hand_sides: Iterable[str] = (),
        rate_hz: float = 60.0,
        speed: float = 1.0,
        clock: Callable[[], int] = time.monotonic_ns,
        expected_coordinator_instance_id: str | None = None,
        session_client: SessionClient | None = None,
    ) -> None:
        if speed <= 0.0: raise ValueError("speed must be positive")
        self.reader = recording if isinstance(recording, SessionH5Reader) else SessionH5Reader(recording)
        self.active_sides = self._sides(active_sides); self.inactive_sides = self._sides(inactive_sides); self.active_hand_sides = self._sides(active_hand_sides); self.inactive_hand_sides = self._sides(inactive_hand_sides)
        self._require_declared_sides(self.active_sides, self.inactive_sides, "arm")
        self._require_declared_sides(self.active_hand_sides, self.inactive_hand_sides, "hand")
        if set(self.active_sides) & set(self.inactive_sides) or set(self.active_hand_sides) & set(self.inactive_hand_sides): raise ValueError("a side cannot be active and inactive")
        self.speed = float(speed)
        self._arms = {side: self.reader.read_arm_target(side) for side in self.active_sides}
        self._hands = {side: self.reader.read_hand_target(side) for side in self.active_hand_sides}
        for side in self.active_sides:
            if not self._arms[side]: raise ValueError(f"recording has no active arm stream: {side}")
        for side in self.active_hand_sides:
            if not self._hands[side]: raise ValueError(f"recording has no active hand stream: {side}")
        self._check_recorded_times()
        if self.reader.attrs.get("router_zid") != router_zid:
            raise ValueError("recording router_zid does not match replay router")
        super().__init__(session=session, source="target_replay", source_publisher_instance_id=publisher_instance_id, router_zid=router_zid, expected_coordinator_instance_id=expected_coordinator_instance_id, rate_hz=rate_hz, clock=clock, session_client=session_client)
        self._publisher = TargetPublisher(session, source="target_replay", publisher_instance_id=publisher_instance_id, router_zid=router_zid, clock=clock, allocator=self._source_allocator) if session is not None else None
        self._register_liveliness(f"{_TARGET_LIVELINESS}/{publisher_instance_id}")
        self._last_indexes = {side: -1 for side in self.active_sides}; self._last_hand_indexes = {side: -1 for side in self.active_hand_sides}

    @staticmethod
    def _sides(sides: Iterable[str]) -> tuple[str, ...]:
        result = tuple(sides)
        if any(side not in ("left", "right") for side in result) or len(set(result)) != len(result): raise ValueError("sides must be unique left/right values")
        return result

    @staticmethod
    def _require_declared_sides(active: tuple[str, ...], inactive: tuple[str, ...], domain: str) -> None:
        if set(active) | set(inactive) != {"left", "right"}:
            raise ValueError(f"profile must explicitly declare active or inactive {domain} sides")

    def _check_recorded_times(self) -> None:
        for rows in (*self._arms.values(), *self._hands.values()):
            if any(int(row["time_ns"]) < 0 for row in rows) or any(int(b["time_ns"]) < int(a["time_ns"]) for a, b in zip(rows, rows[1:])):
                raise ValueError("recorded stream time is not monotonic")
    def publish_status(self, *, phase: str, ready: bool, healthy: bool, error: str | None = None) -> None:
        if self._publisher is not None:
            self._publisher.publish_source_status(component_id="target_replay", phase=phase, ready=ready, healthy=healthy, capabilities=["simulation"], error=error)

    @staticmethod
    def _row_at(rows: list[dict[str, Any]], recorded_ns: int) -> tuple[int, dict[str, Any]] | None:
        index = -1
        for pos, row in enumerate(rows):
            if int(row["time_ns"]) <= recorded_ns: index = pos
            else: break
        return None if index < 0 else (index, rows[index])

    def _emit(self, recorded_ns: int) -> bool:
        emitted = False
        if self._publisher is None: return False
        for side, rows in self._arms.items():
            selected = self._row_at(rows, recorded_ns)
            if selected is not None:
                index, row = selected; self._last_indexes[side] = index
                self._publisher.publish_arm_target(side=side, position_m=row["position_m"], orientation_xyzw=row["orientation_xyzw"], elbow_reference_direction=row["elbow_reference_direction"], source_timestamp_ns=row.get("source_timestamp_ns"), source=row.get("source") or "target_replay", frame_id=row.get("frame_id")); emitted = True
        for side, rows in self._hands.items():
            selected = self._row_at(rows, recorded_ns)
            if selected is not None:
                index, row = selected; self._last_hand_indexes[side] = index
                self._publisher.publish_hand_target(side=side, keypoints_m=row["keypoints_m"], source_timestamp_ns=row.get("source_timestamp_ns"), source=row.get("source") or "target_replay"); emitted = True
        return emitted

    def tick(self, now_ns: int | None = None) -> None:
        self._sync_authority()
        self.publish_status(phase=self._phase, ready=self._phase in {"armed", "start_pending", "replaying"}, healthy=self._phase != "fault")
        if self._phase != "replaying": return
        now = int(self._clock() if now_ns is None else now_ns)
        if self._wall_time_ns is None: self._wall_time_ns = now
        elif not self._paused:
            self._recorded_time_ns += max(0, int((now - self._wall_time_ns) * self.speed)); self._wall_time_ns = now
        self._emit(self._recorded_time_ns)
        last = max((int(rows[-1]["time_ns"]) for rows in (*self._arms.values(), *self._hands.values()) if rows), default=0)
        if not self._paused and self._recorded_time_ns >= last and not self._return_requested:
            self.request_return()

    def close(self) -> None:
        if self._publisher is not None: self._publisher.close()
        self.reader.close(); super().close()


class JointReplayNode(_ReplayLifecycle):
    """Replay arm commands as proposals and hand commands directly."""

    def __init__(
        self,
        recording: str | Path | SessionH5Reader,
        *,
        session: Any = None,
        source_publisher_instance_id: str | None = None,
        producer_publisher_instance_id: str | None = None,
        publisher_instance_id: str | None = None,
        router_zid: str,
        active_sides: Iterable[str] = ("left", "right"),
        inactive_sides: Iterable[str] = (),
        active_hand_sides: Iterable[str] = (),
        inactive_hand_sides: Iterable[str] = (),
        rate_hz: float = 60.0,
        capabilities: Iterable[str] = ("simulation",),
        clock: Callable[[], int] = time.monotonic_ns,
        expected_coordinator_instance_id: str | None = None,
        session_client: SessionClient | None = None,
    ) -> None:
        self.capabilities = tuple(dict.fromkeys(str(value) for value in capabilities))
        if not self.capabilities or not ({"simulation", "real"} & set(self.capabilities)):
            raise ValueError("capabilities must include simulation or real")
        source_publisher_instance_id = source_publisher_instance_id or publisher_instance_id
        producer_publisher_instance_id = producer_publisher_instance_id or publisher_instance_id
        if not source_publisher_instance_id or not producer_publisher_instance_id: raise ValueError("source and producer instance ids are required")
        self.reader = recording if isinstance(recording, SessionH5Reader) else SessionH5Reader(recording)
        self.active_sides = self._sides(active_sides); self.inactive_sides = self._sides(inactive_sides); self.active_hand_sides = self._sides(active_hand_sides); self.inactive_hand_sides = self._sides(inactive_hand_sides)
        self._require_declared_sides(self.active_sides, self.inactive_sides, "arm")
        self._require_declared_sides(self.active_hand_sides, self.inactive_hand_sides, "hand")
        if set(self.active_sides) & set(self.inactive_sides) or set(self.active_hand_sides) & set(self.inactive_hand_sides): raise ValueError("a side cannot be active and inactive")
        self._arms = {side: self.reader.read_arm_command(side) for side in self.active_sides}; self._hands = {side: self.reader.read_hand_command(side) for side in self.active_hand_sides}
        for side in self.active_sides:
            if not self._arms[side]: raise ValueError(f"recording has no active arm command stream: {side}")
        for side in self.active_hand_sides:
            if not self._hands[side]: raise ValueError(f"recording has no active hand command stream: {side}")
        if self.reader.attrs.get("router_zid") != router_zid:
            raise ValueError("recording router_zid does not match replay router")
        super().__init__(session=session, source="joint_replay", source_publisher_instance_id=source_publisher_instance_id, router_zid=router_zid, expected_coordinator_instance_id=expected_coordinator_instance_id, rate_hz=rate_hz, clock=clock, session_client=session_client)
        self.producer_publisher_instance_id = producer_publisher_instance_id; self._producer_allocator = SequenceAllocator(); self._producer_publishers: dict[str, ZenohPub] = {}
        self._source_liveliness = f"{_JOINT_SOURCE_LIVELINESS}/{source_publisher_instance_id}"; self._producer_liveliness = f"{_JOINT_PRODUCER_LIVELINESS}/{producer_publisher_instance_id}"
        self._register_liveliness(self._source_liveliness)
        self._producer_live_token: Any = None
        self._hand_producer_live_token: Any = None
        if session is not None:
            try:
                live = session.liveliness()
                self._producer_live_token = live.declare_token(self._producer_liveliness)
                if self.active_hand_sides:
                    self._hand_producer_live_token = live.declare_token(f"{_JOINT_HAND_PRODUCER_LIVELINESS}/{producer_publisher_instance_id}")
            except (AttributeError, TypeError): pass
        self._last_indexes = {side: -1 for side in self.active_sides}; self._last_hand_indexes = {side: -1 for side in self.active_hand_sides}

    @staticmethod
    def _sides(sides: Iterable[str]) -> tuple[str, ...]:
        result = tuple(sides)
        if any(side not in ("left", "right") for side in result) or len(set(result)) != len(result): raise ValueError("sides must be unique left/right values")
        return result

    @staticmethod
    def _require_declared_sides(active: tuple[str, ...], inactive: tuple[str, ...], domain: str) -> None:
        if set(active) | set(inactive) != {"left", "right"}:
            raise ValueError(f"profile must explicitly declare active or inactive {domain} sides")

    def _producer_pub(self, key: str) -> ZenohPub:
        publisher = self._producer_publishers.get(key)
        if publisher is None:
            publisher = ZenohPub(self.session, key); self._producer_publishers[key] = publisher
        return publisher

    def publish_status(self, *, phase: str, ready: bool, healthy: bool, error: str | None = None) -> None:
        if self.session is None: return
        sequence = self._source_allocator.next()
        status = ComponentStatus(1, sequence, int(self._clock()), "source", "joint_replay", phase, ready, healthy, list(self.capabilities), error, {}, self.source_publisher_instance_id, self.router_zid)
        self._producer_pub(topics.SOURCE_STATUS).put_json(status.to_dict())
        producer = ComponentStatus(1, self._producer_allocator.next(), int(self._clock()), "producer_arm", "joint_replay", phase, ready, healthy, list(self.capabilities), error, {"no_ik_producer": True}, self.producer_publisher_instance_id, self.router_zid)
        self._producer_pub(topics.PRODUCER_STATUS).put_json(producer.to_dict())
        if self.active_hand_sides:
            hand_producer = ComponentStatus(1, self._producer_allocator.next(), int(self._clock()), "producer_hand", "joint_replay", phase, ready, healthy, list(self.capabilities), error, {"direct": True}, self.producer_publisher_instance_id, self.router_zid)
            self._producer_pub(topics.PRODUCER_STATUS).put_json(hand_producer.to_dict())

    @staticmethod
    def _row_at(rows: list[dict[str, Any]], recorded_ns: int) -> tuple[int, dict[str, Any]] | None:
        index = -1
        for pos, row in enumerate(rows):
            if int(row["time_ns"]) <= recorded_ns: index = pos
            else: break
        return None if index < 0 else (index, rows[index])

    def _emit(self, recorded_ns: int) -> None:
        if self.session is None: return
        for side, rows in self._arms.items():
            selected = self._row_at(rows, recorded_ns)
            if selected is None: continue
            index, row = selected; self._last_indexes[side] = index
            proposal = ArmJointProposal(1, self._producer_allocator.next(), int(self._clock()), "joint_replay", side, row.get("target_sequence"), row["names"], row["position_rad"], {"recorded_time_ns": int(row["time_ns"]), "recorded_producer": row.get("producer", "")}, self.producer_publisher_instance_id, self.router_zid)
            self._producer_pub(topics.arm_proposal(side)).put_json(proposal.to_dict())
        for side, rows in self._hands.items():
            selected = self._row_at(rows, recorded_ns)
            if selected is None: continue
            index, row = selected; self._last_hand_indexes[side] = index
            command = HandJointCommand(1, self._producer_allocator.next(), int(self._clock()), "joint_replay", side, row["names"], row["position_rad"], self.producer_publisher_instance_id, self.router_zid)
            self._producer_pub(topics.hand_command(side)).put_json(command.to_dict())

    def tick(self, now_ns: int | None = None) -> None:
        self._sync_authority()
        self.publish_status(phase=self._phase, ready=self._phase in {"armed", "start_pending", "replaying"}, healthy=self._phase != "fault")
        if self._phase != "replaying": return
        now = int(self._clock() if now_ns is None else now_ns)
        if self._wall_time_ns is None: self._wall_time_ns = now
        elif not self._paused:
            self._recorded_time_ns += max(0, now - self._wall_time_ns); self._wall_time_ns = now
        self._emit(self._recorded_time_ns)
        last = max((int(rows[-1]["time_ns"]) for rows in (*self._arms.values(), *self._hands.values()) if rows), default=0)
        if not self._paused and self._recorded_time_ns >= last and not self._return_requested: self.request_return()

    def close(self) -> None:
        for publisher in self._producer_publishers.values(): publisher.close()
        for token in (self._producer_live_token, self._hand_producer_live_token):
            if token is not None:
                try: token.undeclare()
                except Exception: pass
        self.reader.close(); super().close()


__all__ = ["TargetReplaySource", "JointReplayNode", "validate_direct_real_recording"]
