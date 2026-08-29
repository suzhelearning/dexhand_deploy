"""唯一的双臂 command/session authority。

该模块只负责 canonical arm proposal 的仲裁、回 Home 和生命周期状态；IK
producer 与执行器均通过严格 protocol messages 交换数据，不在此模块中重复
解析 JSON 或重新生成身份。
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
import time
from pathlib import Path
from typing import Any, Callable, Mapping

from ..protocol import topics
from ..protocol.messages import (
    ALL_ARM_JOINT_NAMES,
    ARM_JOINT_NAMES,
    ARM_MODES,
    ArmJointCommand,
    ArmJointProposal,
    ArmJointState,
    ComponentStatus,
    LatchedBool,
    ProtocolError,
    SessionState,
    HandExecutorStatus,
    HandJointState,
    strict_loads,
)

from ..zenoh_util import declare_component_liveliness

@dataclass(frozen=True)
class ArmRobotConfig:
    left_joint_names: tuple[str, ...]
    right_joint_names: tuple[str, ...]
    left_home_rad: tuple[float, ...]
    right_home_rad: tuple[float, ...]
    lower_limits_rad: tuple[float, ...]
    upper_limits_rad: tuple[float, ...]

    @property
    def home_all(self) -> tuple[float, ...]:
        return self.left_home_rad + self.right_home_rad

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "ArmRobotConfig":
        required = {"left_joint_names", "right_joint_names", "left_home_rad", "right_home_rad", "lower_limits_rad", "upper_limits_rad"}
        extra = set(value) - required
        missing = required - set(value)
        if missing or extra:
            raise ValueError(f"arm config fields mismatch; missing={sorted(missing)}, extra={sorted(extra)}")
        def names(raw: Any, side: str) -> tuple[str, ...]:
            result = tuple(raw) if isinstance(raw, (list, tuple)) else ()
            expected = ARM_JOINT_NAMES[side]
            if result != expected or len(set(result)) != 7:
                raise ValueError(f"{side}_joint_names must exactly match canonical order")
            return result
        def vector(raw: Any, field: str) -> tuple[float, ...]:
            if not isinstance(raw, (list, tuple)) or len(raw) != 7:
                raise ValueError(f"{field} must contain seven values")
            result = tuple(float(x) for x in raw)
            if not all(math.isfinite(x) for x in result):
                raise ValueError(f"{field} must be finite")
            return result
        lower, upper = vector(value["lower_limits_rad"], "lower_limits_rad"), vector(value["upper_limits_rad"], "upper_limits_rad")
        if any(lo >= hi for lo, hi in zip(lower, upper)):
            raise ValueError("lower_limits_rad must be below upper_limits_rad")
        left_home, right_home = vector(value["left_home_rad"], "left_home_rad"), vector(value["right_home_rad"], "right_home_rad")
        if any(not lo <= x <= hi for x, lo, hi in zip(left_home, lower, upper)) or any(not lo <= x <= hi for x, lo, hi in zip(right_home, lower, upper)):
            raise ValueError("home positions must be within joint limits")
        return cls(names(value["left_joint_names"], "left"), names(value["right_joint_names"], "right"), left_home, right_home, lower, upper)

    @classmethod
    def load(cls, path: str | os.PathLike[str] | None = None) -> "ArmRobotConfig":
        path = Path(path) if path else Path(__file__).resolve().parents[4] / "src" / "pico_body_tianji" / "config" / "robot" / "arm.yaml"
        try:
            import yaml
            value = yaml.safe_load(path.read_text(encoding="utf-8"))
        except Exception as exc:
            raise ValueError(f"unable to load arm config {path}: {exc}") from exc
        if not isinstance(value, Mapping):
            raise ValueError("arm config must be an object")
        return cls.from_mapping(value)


@dataclass(frozen=True)
class IntentResult:
    accepted: bool
    state: SessionState
    reason: str


@dataclass(frozen=True)
class _Timed:
    value: Any
    received_ns: int


class ArmCommandCoordinator:
    """仲裁唯一的双臂 final command，并驱动 session state machine。

    ``publisher_instance_id``、``router_zid`` 必须由 launcher 传入；该类绝不
    生成匿名或进程内 fallback identity。
    """

    def __init__(
        self,
        session: Any,
        *,
        publisher_instance_id: str,
        router_zid: str,
        profile: Mapping[str, Any] | None = None,
        robot_config: ArmRobotConfig | Mapping[str, Any] | str | os.PathLike[str] | None = None,
        coordinator_config: Mapping[str, Any] | str | os.PathLike[str] | None = None,
        clock: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        if not publisher_instance_id or not router_zid:
            raise ValueError("publisher_instance_id and router_zid are required")
        self.session = session
        self.publisher_instance_id = publisher_instance_id
        self.router_zid = router_zid
        self.clock = clock
        self.profile = dict(profile or {})
        if "authorities" in self.profile:
            self.authorities = self._validate_authorities(self.profile["authorities"])
        else:
            # Direct unit users may construct a coordinator without launcher
            # wiring; the production entry point always supplies this map.
            self.authorities = None
        self.robot = robot_config if isinstance(robot_config, ArmRobotConfig) else (ArmRobotConfig.from_mapping(robot_config) if isinstance(robot_config, Mapping) else ArmRobotConfig.load(robot_config))
        self.config = self._coordinator_config(coordinator_config)
        self._sequence = 0
        self._state = self._make_state("idle", "startup", None)
        self._at_home = LatchedBool(1, 0, 0, True, publisher_instance_id, router_zid)
        self._return_complete = LatchedBool(1, 0, 0, False, publisher_instance_id, router_zid)
        self._statuses: dict[tuple[str, str], _Timed] = {}
        self._role_instances: dict[tuple[str, str], str] = {}
        self._arm_state: _Timed | None = None
        self._arm_state_baseline: tuple[str, int] | None = None
        self._proposals: dict[str, _Timed] = {}
        self._hand_status: dict[str, _Timed] = {}
        self._hand_status_baseline: dict[str, tuple[str, int]] = {}
        self._hand_state: dict[str, _Timed] = {}
        self._hand_state_baseline: dict[str, tuple[str, int]] = {}
        self._return_started_ns: int | None = None
        self._return_start_command: dict[str, list[float]] | None = None
        self._safe_command = {"left": list(self.robot.left_home_rad), "right": list(self.robot.right_home_rad)}
        self._publishers: dict[str, Any] = {}
        self._queryables: list[Any] = []
        self._liveliness_token = (
            declare_component_liveliness(
                session,
                role="coordinator/arm",
                logical_id="arm",
                instance_id=publisher_instance_id,
            )
            if session is not None
            else None
        )
        if session is not None:
            self._setup_transport(session)
    @staticmethod
    def _validate_authorities(value: Any) -> dict[str, Any]:
        roles = ("source", "producer_arm", "producer_hand", "coordinator_arm", "executor_arm", "executor_hand")
        if not isinstance(value, Mapping) or set(value) != set(roles):
            raise ValueError("authorities must contain exactly the six canonical roles")

        def identity(raw: Any, role: str) -> dict[str, Any]:
            if not isinstance(raw, Mapping):
                raise ValueError(f"authority {role} must be an identity mapping")
            required = {"logical_id", "publisher_instance_id", "router_zid"}
            if set(raw) - required - {"enabled"} or not required <= set(raw):
                raise ValueError(f"authority {role} identity fields are incomplete")
            result = {
                "logical_id": str(raw["logical_id"]),
                "publisher_instance_id": str(raw["publisher_instance_id"]),
                "router_zid": str(raw["router_zid"]),
            }
            if not all(result.values()):
                raise ValueError(f"authority {role} identity fields must be non-empty")
            if "enabled" in raw:
                result["enabled"] = bool(raw["enabled"])
            return result

        result: dict[str, Any] = {}
        for role in roles:
            raw = value[role]
            if role in {"producer_hand", "executor_hand"} and isinstance(raw, Mapping) and "logical_id" not in raw:
                if set(raw) != {"left", "right"}:
                    raise ValueError(f"authority {role} side mapping must contain left/right")
                result[role] = {side: identity(raw[side], f"{role}/{side}") for side in ("left", "right")}
            else:
                result[role] = identity(raw, role)
        return result

    def _expected_authority(self, role: str, side: str | None = None) -> Mapping[str, Any] | None:
        if self.authorities is None:
            return None
        value = self.authorities.get(role)
        if side is not None and isinstance(value, Mapping) and "logical_id" not in value:
            value = value.get(side)
        return value if isinstance(value, Mapping) else None

    def _matches_authority(self, role: str, logical_id: str, instance: str, router: str, *, side: str | None = None) -> bool:
        expected = self._expected_authority(role, side)
        if expected is None and self.authorities is not None:
            value = self.authorities.get(role)
            if isinstance(value, Mapping) and "logical_id" not in value and side is None:
                return any(
                    self._matches_authority(role, logical_id, instance, router, side=candidate)
                    for candidate in ("left", "right")
                )
        if expected is None:
            return self.authorities is None
        return (
            bool(expected.get("enabled", True))
            and logical_id == expected.get("logical_id")
            and instance == expected.get("publisher_instance_id")
            and router == expected.get("router_zid")
        )


    @staticmethod
    def _coordinator_config(raw: Mapping[str, Any] | str | os.PathLike[str] | None) -> dict[str, float]:
        required = ("rate_hz", "proposal_timeout_s", "maximum_command_step_rad", "home_minimum_duration_s", "home_max_speed_rad_s", "home_tolerance_rad", "state_timeout_s", "hand_return_timeout_s")
        if raw is None:
            raw = Path(__file__).resolve().parents[4] / "src" / "pico_body_tianji" / "config" / "coordinator" / "arm.yaml"
        if isinstance(raw, Mapping):
            value = dict(raw)
        else:
            try:
                import yaml
                value = yaml.safe_load(Path(raw).read_text(encoding="utf-8"))
            except Exception as exc:
                raise ValueError(f"unable to load coordinator config {raw}: {exc}") from exc
        if not isinstance(value, Mapping) or set(value) != set(required):
            raise ValueError("coordinator config must contain only the eight canonical fields")
        result = {key: float(value[key]) for key in required}
        if any(not math.isfinite(x) or x <= 0.0 for x in result.values()):
            raise ValueError("coordinator config values must be finite and positive")
        return result

    @property
    def state(self) -> SessionState:
        return self._state

    @property
    def at_home(self) -> LatchedBool:
        return self._at_home

    @property
    def return_complete(self) -> LatchedBool:
        return self._return_complete

    def _setup_transport(self, session: Any) -> None:
        # Deliberately use only canonical topic functions. Queryables reply with
        # the same typed payloads that subscribers receive.
        for key, getter in ((topics.SESSION_STATE, lambda: self._state), (topics.AT_HOME, lambda: self._at_home), (topics.RETURN_COMPLETE, lambda: self._return_complete)):
            if hasattr(session, "declare_queryable"):
                self._queryables.append(session.declare_queryable(key, lambda query, get=getter, key=key: query.reply(key, json.dumps(get().to_dict(), separators=(",", ":")).encode())))
        for side in ("left", "right"):
            if hasattr(session, "declare_publisher"):
                self._publishers[side] = session.declare_publisher(topics.arm_command(side))
        if hasattr(session, "declare_publisher"):
            self._publishers["state"] = session.declare_publisher(topics.SESSION_STATE)
            self._publishers["home"] = session.declare_publisher(topics.AT_HOME)
            self._publishers["complete"] = session.declare_publisher(topics.RETURN_COMPLETE)

    def _publish(self, name: str, payload: Mapping[str, Any]) -> None:
        publisher = self._publishers.get(name)
        if publisher is not None:
            publisher.put(json.dumps(payload, separators=(",", ":")).encode("utf-8"), encoding="application/json")

    def _make_state(self, state: str, reason: str, intent_sequence: int | None) -> SessionState:
        return SessionState(1, self._sequence, self.clock(), state, reason, "coordinator", intent_sequence, self.publisher_instance_id, self.router_zid)

    def _fresh(self, timed: _Timed | None, now_ns: int) -> bool:
        return timed is not None and 0 <= now_ns - timed.received_ns <= int(self.config["state_timeout_s"] * 1e9)

    def _domain_ready(self, role: str, now_ns: int) -> bool:
        side_map = self.authorities is not None and role in {"producer_hand", "executor_hand"} and isinstance(self.authorities.get(role), Mapping) and "logical_id" not in self.authorities[role]
        if side_map:
            for side in tuple(self.profile.get("hand_sides", ("left", "right"))):
                expected = self._expected_authority(role, side)
                matches = [
                    timed for (entry_role, entry_id), timed in self._statuses.items()
                    if entry_role == role
                    and self._fresh(timed, now_ns)
                    and expected is not None
                    and self._matches_authority(role, entry_id, timed.value.publisher_instance_id, timed.value.router_zid, side=side)
                ]
                if len(matches) != 1 or not matches[0].value.ready or not matches[0].value.healthy:
                    return False
            return True
        entries = [timed for (entry_role, _), timed in self._statuses.items() if entry_role == role and self._fresh(timed, now_ns)]
        if len(entries) != 1:
            return False
        status = entries[0].value
        if not self._matches_authority(role, status.component_id, status.publisher_instance_id, status.router_zid):
            return False
        return status.ready and status.healthy and self.profile.get("required_capability", "simulation") in status.capabilities
    def update_component(self, status: ComponentStatus | Mapping[str, Any], *, received_ns: int | None = None) -> None:
        try:
            parsed = status if isinstance(status, ComponentStatus) else ComponentStatus.from_dict(status)
        except (ProtocolError, TypeError, ValueError) as exc:
            self._enter_fault(f"malformed component status: {exc}")
            return
        if parsed.router_zid != self.router_zid:
            self._enter_fault("component router_zid mismatch")
            return
        diagnostic_side = parsed.diagnostics.get("side") if isinstance(parsed.diagnostics, Mapping) else None
        if not self._matches_authority(parsed.component_role, parsed.component_id, parsed.publisher_instance_id, parsed.router_zid, side=diagnostic_side):
            self._enter_fault(f"component authority mismatch for {parsed.component_role}/{parsed.component_id}")
            return
        key = (parsed.component_role, parsed.component_id)
        previous = self._statuses.get(key)
        previous_instance = self._role_instances.get(key)
        if previous_instance is not None and previous_instance != parsed.publisher_instance_id:
            if self._state.state == "teleop":
                self._enter_fault(f"duplicate authority for {parsed.component_role}/{parsed.component_id}")
                return
            # Launcher-authorized replacement is only safe outside teleop.  A
            # new instance starts its own (instance, sequence) baseline.
            previous = None
        if previous is not None and parsed.sequence <= previous.value.sequence:
            self._enter_fault(f"component sequence rollback for {parsed.component_role}/{parsed.component_id}")
            return
        self._role_instances[key] = parsed.publisher_instance_id
        self._statuses[key] = _Timed(parsed, self.clock() if received_ns is None else received_ns)

    def update_arm_state(self, state: ArmJointState | Mapping[str, Any], *, received_ns: int | None = None) -> None:
        try:
            parsed = state if isinstance(state, ArmJointState) else ArmJointState.from_dict(state)
        except (ProtocolError, TypeError, ValueError) as exc:
            self._enter_fault(f"malformed arm state: {exc}")
            return
        if parsed.router_zid != self.router_zid or tuple(parsed.names) != ALL_ARM_JOINT_NAMES:
            self._enter_fault("arm state identity/order mismatch")
            return
        previous = self._arm_state
        if previous is not None:
            previous_instance, previous_sequence = self._arm_state_baseline or (
                previous.value.publisher_instance_id, previous.value.sequence
            )
            if parsed.publisher_instance_id != previous_instance:
                if self._state.state == "teleop":
                    self._enter_fault("arm executor instance changed during teleop")
                    return
            elif parsed.sequence <= previous_sequence:
                self._enter_fault("arm state sequence rollback")
                return
        self._arm_state_baseline = (parsed.publisher_instance_id, parsed.sequence)
        self._arm_state = _Timed(parsed, self.clock() if received_ns is None else received_ns)

    def update_hand_executor_status(self, status: HandExecutorStatus | Mapping[str, Any], *, received_ns: int | None = None) -> None:
        try:
            parsed = status if isinstance(status, HandExecutorStatus) else HandExecutorStatus.from_dict(status)
        except (ProtocolError, TypeError, ValueError) as exc:
            self._enter_fault(f"malformed hand executor status: {exc}")
            return
        if parsed.router_zid != self.router_zid:
            self._enter_fault("hand executor router_zid mismatch")
            return
        if not self._matches_authority(
            "executor_hand",
            f"wuji_{parsed.side}",
            parsed.publisher_instance_id,
            parsed.router_zid,
            side=parsed.side,
        ):
            self._enter_fault(f"hand executor authority mismatch for {parsed.side}")
            return
        previous = self._hand_status.get(parsed.side)
        baseline = self._hand_status_baseline.get(parsed.side)
        if previous is not None and baseline is not None:
            previous_instance, previous_sequence = baseline
            if parsed.publisher_instance_id != previous_instance:
                if self._state.state == "teleop":
                    self._enter_fault("hand executor instance changed during teleop")
                    return
            elif parsed.sequence <= previous_sequence:
                self._enter_fault("hand executor status sequence rollback")
                return
        self._hand_status_baseline[parsed.side] = (parsed.publisher_instance_id, parsed.sequence)
        self._hand_status[parsed.side] = _Timed(parsed, self.clock() if received_ns is None else received_ns)

    def update_hand_state(self, state: HandJointState | Mapping[str, Any], *, received_ns: int | None = None) -> None:
        try:
            parsed = state if isinstance(state, HandJointState) else HandJointState.from_dict(state)
        except (ProtocolError, TypeError, ValueError) as exc:
            self._enter_fault(f"malformed hand state: {exc}")
            return
        if parsed.router_zid != self.router_zid:
            self._enter_fault("hand state router_zid mismatch")
            return
        if not self._matches_authority(
            "executor_hand",
            f"wuji_{parsed.side}",
            parsed.publisher_instance_id,
            parsed.router_zid,
            side=parsed.side,
        ):
            self._enter_fault(f"hand state executor authority mismatch for {parsed.side}")
            return
        status = self._hand_status.get(parsed.side)
        if status is not None and status.value.publisher_instance_id != parsed.publisher_instance_id:
            self._enter_fault("hand state executor identity mismatch")
            return
        previous = self._hand_state.get(parsed.side)
        baseline = self._hand_state_baseline.get(parsed.side)
        if previous is not None and baseline is not None:
            previous_instance, previous_sequence = baseline
            if parsed.publisher_instance_id != previous_instance:
                if self._state.state == "teleop":
                    self._enter_fault("hand state instance changed during teleop")
                    return
            elif parsed.sequence <= previous_sequence:
                self._enter_fault("hand state sequence rollback")
                return
        self._hand_state_baseline[parsed.side] = (parsed.publisher_instance_id, parsed.sequence)
        self._hand_state[parsed.side] = _Timed(parsed, self.clock() if received_ns is None else received_ns)

    def update_proposal(self, proposal: ArmJointProposal | Mapping[str, Any], *, received_ns: int | None = None) -> bool:
        try:
            parsed = proposal if isinstance(proposal, ArmJointProposal) else ArmJointProposal.from_dict(proposal)
        except (ProtocolError, TypeError, ValueError) as exc:
            self._enter_fault(f"malformed arm proposal: {exc}")
            return False
        if parsed.router_zid != self.router_zid or parsed.side not in ("left", "right"):
            self._enter_fault("arm proposal identity mismatch")
            return False
        role = ("producer_arm", parsed.producer)
        status_timed = self._statuses.get(role)
        if status_timed is None or status_timed.value.publisher_instance_id != parsed.publisher_instance_id or status_timed.value.router_zid != parsed.router_zid:
            self._enter_fault("proposal producer authority mismatch")
            return False
        observed_ns = self.clock() if received_ns is None else int(received_ns)
        if parsed.timestamp_ns > observed_ns or observed_ns - parsed.timestamp_ns > int(self.config["state_timeout_s"] * 1e9):
            self._enter_fault("arm proposal timestamp stale")
            return False
        old = self._proposals.get(parsed.side)
        if old is not None:
            old_instance = old.value.publisher_instance_id
            if parsed.publisher_instance_id != old_instance:
                if self._state.state == "teleop":
                    self._enter_fault("arm producer instance changed during teleop")
                    return False
                old = None
            elif parsed.sequence <= old.value.sequence:
                self._enter_fault("arm proposal sequence rollback")
                return False
        self._proposals[parsed.side] = _Timed(parsed, observed_ns)
        return True

    def handle_proposal_dict(self, value: Mapping[str, Any]) -> bool:
        return self.update_proposal(value)

    def _arm_at_home(self, now_ns: int) -> bool:
        if not self._fresh(self._arm_state, now_ns):
            return False
        return all(abs(x - y) <= self.config["home_tolerance_rad"] for x, y in zip(self._arm_state.value.position_rad, self.robot.home_all))

    def _hand_enabled(self) -> bool:
        return bool(self.profile.get("hand_enabled") or self.profile.get("hand_sides"))

    def _hand_at_zero_ready(self, now_ns: int) -> bool:
        sides = tuple(self.profile.get("hand_sides", ("left", "right")))
        tolerance = float(self.profile.get("zero_tolerance_rad", 0.05))
        for side in sides:
            status = self._hand_status.get(side)
            state = self._hand_state.get(side)
            if not self._fresh(status, now_ns) or not self._fresh(state, now_ns):
                return False
            if not (status.value.ready and status.value.healthy and status.value.at_zero and not status.value.tracking_allowed):
                return False
            if any(abs(x) > tolerance for x in state.value.position_rad):
                return False
        return True

    def _hand_tracking_fresh(self, now_ns: int) -> bool:
        """Require matching, fresh hand status and state while teleoperating."""
        for side in tuple(self.profile.get("hand_sides", ("left", "right"))):
            status = self._hand_status.get(side)
            state = self._hand_state.get(side)
            if not self._fresh(status, now_ns) or not self._fresh(state, now_ns):
                return False
            if status.value.publisher_instance_id != state.value.publisher_instance_id:
                return False
            if not (status.value.ready and status.value.healthy and status.value.tracking_allowed):
                return False
        return True

    def _return_ready(self, now_ns: int) -> bool:
        return self._arm_at_home(now_ns) and (not self._hand_enabled() or self._hand_at_zero_ready(now_ns))

    def _start_ready(self, now_ns: int) -> tuple[bool, str]:
        for role in ("source", "producer_arm", "executor_arm"):
            if not self._domain_ready(role, now_ns):
                return False, f"{role} not exactly-one fresh healthy ready"
        if self._hand_enabled():
            if not self._domain_ready("producer_hand", now_ns):
                return False, "producer_hand not exactly-one fresh healthy ready"
            if not all(self._hand_at_zero_ready(now_ns) for _ in (0,)):
                return False, "hand executor/state not fresh at zero"
        if not self._fresh(self._arm_state, now_ns) or not self._arm_at_home(now_ns):
            return False, "arm state is not fresh at Home"
        if not self._commands_at_home():
            return False, "final command is not at Home"
        return True, "accepted"
    def _commands_at_home(self) -> bool:
        return all(
            self._safe_command[side] == list(getattr(self.robot, f"{side}_home_rad"))
            for side in ("left", "right")
        )

    def handle_intent(self, intent: Any) -> IntentResult:
        action, sequence, reason = getattr(intent, "action", None), getattr(intent, "sequence", None), getattr(intent, "reason", "")
        now_ns = self.clock()
        if self._state.state == "fault":
            rejected = self._make_state("fault", self._fault_reason or "fault latched", sequence)
            self._publish("state", rejected.to_dict())
            return IntentResult(False, rejected, "fault latched; restart required")
        if action == "start" and self._state.state != "idle":
            rejected = self._make_state(self._state.state, "start requires idle", sequence)
            self._publish("state", rejected.to_dict())
            return IntentResult(False, rejected, "start requires idle")
        if action == "start":
            ready, why = self._start_ready(now_ns)
            if not ready:
                rejected = self._make_state(self._state.state, why, sequence)
                self._publish("state", rejected.to_dict())
                return IntentResult(False, rejected, why)
            self._sequence += 1
            self._state = self._make_state("teleop", "accepted", sequence)
            self._at_home = LatchedBool(1, self._sequence, self._state.timestamp_ns, False, self.publisher_instance_id, self.router_zid)
            self._return_complete = LatchedBool(1, self._sequence, self._state.timestamp_ns, False, self.publisher_instance_id, self.router_zid)
            self._publish("state", self._state.to_dict())
            self._publish("home", self._at_home.to_dict())
            self._publish("complete", self._return_complete.to_dict())
            return IntentResult(True, self._state, "accepted")
        if action in ("return", "shutdown"):
            self._sequence += 1
            self._return_started_ns = now_ns
            self._return_start_command = {side: list(values) for side, values in self._safe_command.items()}
            self._state = self._make_state("returning", reason or action, sequence)
            self._return_complete = LatchedBool(1, self._sequence, self._state.timestamp_ns, False, self.publisher_instance_id, self.router_zid)
            self._publish("state", self._state.to_dict())
            self._publish("complete", self._return_complete.to_dict())
            return IntentResult(True, self._state, "accepted")
        return IntentResult(False, self._state, "unsupported intent")

    def _enter_returning(self, reason: str, now_ns: int) -> None:
        if self._state.state in {"returning", "fault"}:
            return
        self._sequence += 1
        self._return_started_ns = now_ns
        self._return_start_command = {side: list(values) for side, values in self._safe_command.items()}
        self._state = self._make_state("returning", reason, self._state.intent_sequence)
        self._return_complete = LatchedBool(1, self._sequence, self._state.timestamp_ns, False, self.publisher_instance_id, self.router_zid)

    def _check_teleop_health(self, now_ns: int) -> None:
        if self._state.state != "teleop":
            return
        for role in ("source", "producer_arm"):
            if not self._domain_ready(role, now_ns):
                self._enter_returning(f"{role} stale or unhealthy", now_ns)
                return
        if not self._domain_ready("executor_arm", now_ns) or not self._fresh(self._arm_state, now_ns):
            self._enter_fault("executor arm/state stale or unhealthy")
            return
        if self._hand_enabled():
            if not self._domain_ready("producer_hand", now_ns):
                self._enter_returning("producer_hand stale or unhealthy", now_ns)
                return
            if not self._hand_tracking_fresh(now_ns):
                self._enter_fault("hand executor/status state stale, unhealthy, or identity mismatch")
                return
        for side in self.profile.get("active_sides", ("left", "right")):
            if not self._fresh(self._proposals.get(side), now_ns):
                self._enter_returning("arm proposal timeout", now_ns)
                return

    def _enter_fault(self, reason: str) -> None:
        self._fault_reason = reason
        if self._state.state != "fault":
            self._return_start_command = {side: list(values) for side, values in self._safe_command.items()}
            self._return_started_ns = self.clock()
            self._sequence += 1
            self._state = self._make_state("fault", reason, self._state.intent_sequence)
            self._return_complete = LatchedBool(1, self._sequence, self._state.timestamp_ns, False, self.publisher_instance_id, self.router_zid)
    def _command(self, side: str, sequence: int, timestamp_ns: int) -> ArmJointCommand:
        mode = "teleop" if self._state.state == "teleop" else ("idle" if self._state.state == "idle" else "returning")
        proposal = self._proposals.get(side)
        proposal_seq = target_seq = None
        home = list(getattr(self.robot, f"{side}_home_rad"))
        position = home
        if mode == "teleop" and side in set(self.profile.get("active_sides", ("left", "right"))):
            if proposal is not None and self._fresh(proposal, timestamp_ns):
                candidate = proposal.value
                position = list(candidate.position_rad)
                proposal_seq, target_seq = candidate.sequence, candidate.target_sequence
        elif mode == "returning":
            start = (self._return_start_command or self._safe_command)[side]
            elapsed = max(0.0, (timestamp_ns - (self._return_started_ns or timestamp_ns)) / 1e9)
            distance = max(abs(x - y) for x, y in zip(start, home))
            duration = max(self.config["home_minimum_duration_s"], distance / self.config["home_max_speed_rad_s"])
            fraction = min(1.0, elapsed / duration)
            position = [x + fraction * (y - x) for x, y in zip(start, home)]
        self._safe_command[side] = position
        return ArmJointCommand(1, sequence, timestamp_ns, "coordinator", side, mode, proposal_seq, target_seq, list(ARM_JOINT_NAMES[side]), position, self.publisher_instance_id, self.router_zid)

    def _validate_proposals(self, now_ns: int) -> None:
        if self._state.state != "teleop":
            return
        active = set(self.profile.get("active_sides", ("left", "right")))
        for side in active:
            timed = self._proposals.get(side)
            if timed is None or not self._fresh(timed, now_ns):
                continue
            candidate = timed.value
            if not all(math.isfinite(x) and lo <= x <= hi for x, lo, hi in zip(candidate.position_rad, self.robot.lower_limits_rad, self.robot.upper_limits_rad)):
                self._enter_fault("proposal exceeds hard joint limits or is nonfinite")
                return
            if any(abs(x - old) > self.config["maximum_command_step_rad"] for x, old in zip(candidate.position_rad, self._safe_command[side])):
                self._enter_fault("proposal exceeds maximum command step")
    def _check_teleop_health(self, now_ns: int) -> None:
        if self._state.state != "teleop":
            return
        for role in ("source", "producer_arm"):
            if not self._domain_ready(role, now_ns):
                self._enter_returning(f"{role} stale or unhealthy", now_ns)
                return
        if not self._domain_ready("executor_arm", now_ns) or not self._fresh(self._arm_state, now_ns):
            self._enter_fault("executor arm/state stale or unhealthy")
            return
        if self._hand_enabled():
            if not self._domain_ready("producer_hand", now_ns):
                self._enter_returning("producer_hand stale or unhealthy", now_ns)
                return
            if not all(self._fresh(self._hand_status.get(side), now_ns) and self._hand_status[side].value.healthy for side in self.profile.get("hand_sides", ("left", "right"))):
                self._enter_fault("hand executor status stale or unhealthy")
                return
        for side in self.profile.get("active_sides", ("left", "right")):
            if not self._fresh(self._proposals.get(side), now_ns):
                self._enter_returning("arm proposal timeout", now_ns)
                return

    def tick(self, *, now_ns: int | None = None) -> dict[str, ArmJointCommand]:
        now_ns = self.clock() if now_ns is None else int(now_ns)
        self._validate_proposals(now_ns)
        self._check_teleop_health(now_ns)
        if (
            self._state.state == "returning"
            and self._hand_enabled()
            and self._return_started_ns is not None
            and now_ns - self._return_started_ns > int(self.config["hand_return_timeout_s"] * 1e9)
            and not self._hand_at_zero_ready(now_ns)
        ):
            self._enter_fault("hand return timeout")
        self._sequence += 1
        timestamp_ns = now_ns
        commands = {side: self._command(side, self._sequence, timestamp_ns) for side in ("left", "right")}
        if self._state.state == "returning" and self._return_ready(now_ns) and all(command.position_rad == list(getattr(self.robot, f"{side}_home_rad")) for side, command in commands.items()):
            self._state = self._make_state("idle", "return complete", self._state.intent_sequence)
            self._at_home = LatchedBool(1, self._sequence, timestamp_ns, True, self.publisher_instance_id, self.router_zid)
            self._return_complete = LatchedBool(1, self._sequence, timestamp_ns, True, self.publisher_instance_id, self.router_zid)
        else:
            at_home = all(command.position_rad == list(getattr(self.robot, f"{side}_home_rad")) for side, command in commands.items())
            self._at_home = LatchedBool(1, self._sequence, timestamp_ns, at_home, self.publisher_instance_id, self.router_zid)
            self._return_complete = LatchedBool(1, self._sequence, timestamp_ns, self._return_complete.value, self.publisher_instance_id, self.router_zid)
        self._state = SessionState(1, self._sequence, timestamp_ns, self._state.state, self._state.reason, "coordinator", self._state.intent_sequence, self.publisher_instance_id, self.router_zid)
        for side, command in commands.items():
            self._publish(side, command.to_dict())
        self._publish("state", self._state.to_dict())
        self._publish("home", self._at_home.to_dict())
        self._publish("complete", self._return_complete.to_dict())
        return commands

    def start(self) -> None:
        """订阅所有 authority 输入后，以 coordinator control rate 刷新输出。"""
        if self.session is None:
            raise RuntimeError("coordinator requires a Zenoh session")
        callbacks = [
            (topics.SESSION_INTENT, self._on_intent_payload),
            (topics.SOURCE_STATUS, self._on_component_payload),
            (topics.PRODUCER_STATUS, self._on_component_payload),
            (topics.EXECUTOR_STATUS, self._on_component_payload),
            (topics.ARM_STATE, self._on_arm_state_payload),
            (topics.ARM_PROPOSAL.format(side="left"), self._on_proposal_payload),
            (topics.ARM_PROPOSAL.format(side="right"), self._on_proposal_payload),
            (topics.HAND_EXECUTOR_STATUS.format(side="left"), self._on_hand_executor_status_payload),
            (topics.HAND_EXECUTOR_STATUS.format(side="right"), self._on_hand_executor_status_payload),
            (topics.HAND_STATE.format(side="left"), self._on_hand_state_payload),
            (topics.HAND_STATE.format(side="right"), self._on_hand_state_payload),
        ]
        resources = [self.session.declare_subscriber(key, callback) for key, callback in callbacks]
        try:
            period = 1.0 / self.config["rate_hz"]
            while True:
                started = time.monotonic()
                self.tick()
                time.sleep(max(0.0, period - (time.monotonic() - started)))
        finally:
            for resource in resources:
                try:
                    resource.undeclare()
                except Exception:
                    pass

    def _payload(self, sample: Any) -> Mapping[str, Any]:
        payload = getattr(sample, "payload", sample)
        if isinstance(payload, Mapping):
            return payload
        try:
            raw = bytes(payload)
        except (TypeError, ValueError) as exc:
            raise ProtocolError("Zenoh sample payload is not bytes-like") from exc
        if not raw:
            raise ProtocolError("empty Zenoh sample payload")
        return strict_loads(raw)
    def _on_intent_payload(self, sample: Any) -> None:
        try:
            payload = self._payload(sample)
            from ..protocol.messages import SessionIntent
            self.handle_intent(SessionIntent.from_dict(payload))
        except (ProtocolError, TypeError, ValueError, json.JSONDecodeError):
            self._enter_fault("malformed session intent")


    def _on_hand_executor_status_payload(self, sample: Any) -> None:
        try:
            self.update_hand_executor_status(self._payload(sample))
        except (ProtocolError, TypeError, ValueError, json.JSONDecodeError):
            self._enter_fault("malformed hand executor status")

    def _on_hand_state_payload(self, sample: Any) -> None:
        try:
            self.update_hand_state(self._payload(sample))
        except (ProtocolError, TypeError, ValueError, json.JSONDecodeError):
            self._enter_fault("malformed hand state")
    def _on_component_payload(self, sample: Any) -> None:
        try:
            self.update_component(self._payload(sample))
        except (ProtocolError, TypeError, ValueError, json.JSONDecodeError):
            self._enter_fault("malformed component status")

    def _on_arm_state_payload(self, sample: Any) -> None:
        try:
            self.update_arm_state(self._payload(sample))
        except (ProtocolError, TypeError, ValueError, json.JSONDecodeError):
            self._enter_fault("malformed arm state")

    def _on_proposal_payload(self, sample: Any) -> None:
        try:
            self.update_proposal(self._payload(sample))
        except (ProtocolError, TypeError, ValueError, json.JSONDecodeError):
            self._enter_fault("malformed arm proposal")

    def close(self) -> None:
        if self._liveliness_token is not None:
            try:
                self._liveliness_token.undeclare()
            except Exception:
                pass
            self._liveliness_token = None
        for item in (*self._publishers.values(), *self._queryables):
            try:
                item.undeclare()
            except Exception:
                pass
        self._publishers.clear()
        self._queryables.clear()



def main() -> int:
    endpoint = os.environ.get("TIANJI_ROUTER_ENDPOINT", "tcp/127.0.0.1:7447")
    instance = os.environ.get("TIANJI_COORDINATOR_INSTANCE_ID")
    router = os.environ.get("TIANJI_ROUTER_ZID")
    authorities_raw = os.environ.get("TIANJI_AUTHORITIES", "")
    if not instance or not router or not authorities_raw:
        raise RuntimeError(
            "TIANJI_COORDINATOR_INSTANCE_ID, TIANJI_ROUTER_ZID and "
            "TIANJI_AUTHORITIES are required"
        )
    try:
        authorities = json.loads(authorities_raw)
    except json.JSONDecodeError as exc:
        raise RuntimeError("TIANJI_AUTHORITIES must be valid JSON") from exc
    from ..zenoh_util import open_session, require_single_router
    session = open_session(endpoint)
    router = require_single_router(session, router)
    hand_mode = os.environ.get("TIANJI_HAND_MODE", "disabled")
    coordinator_config = os.environ.get("TIANJI_COORDINATOR_CONFIG")
    node = ArmCommandCoordinator(
        session,
        publisher_instance_id=instance,
        router_zid=router,
        profile={
            "active_sides": tuple(filter(None, os.environ.get("TIANJI_ACTIVE_SIDES", "left,right").split(","))),
            "required_capability": os.environ.get("TIANJI_REQUIRED_CAPABILITY", "simulation"),
            "hand_mode": hand_mode,
            "hand_enabled": hand_mode != "disabled",
            "hand_sides": tuple(filter(None, os.environ.get("TIANJI_ACTIVE_HAND_SIDES", os.environ.get("TIANJI_ACTIVE_SIDES", "left,right")).split(","))),
            "authorities": authorities,
        },
        coordinator_config=coordinator_config,
    )
    try:
        node.start()
    finally:
        node.close()
        session.close()
    return 0
__all__ = ["ArmRobotConfig", "ArmCommandCoordinator", "IntentResult", "main"]
