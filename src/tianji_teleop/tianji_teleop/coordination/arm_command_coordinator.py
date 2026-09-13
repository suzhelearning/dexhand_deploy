"""唯一的双臂 command/session authority。

该模块只负责 canonical arm proposal 的仲裁、回 Home 和生命周期状态；IK
producer 与执行器均通过严格 protocol messages 交换数据，不在此模块中重复
解析 JSON 或重新生成身份。
"""
from __future__ import annotations

from dataclasses import dataclass
from copy import deepcopy
import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Any, Callable, Mapping

from ..protocol import topics
from ..protocol.bilateral import (
    ArmBilateralProposal, ArmBilateralCommand, ARM_BILATERAL_PROPOSAL,
    ARM_BILATERAL_RECEIPT, ARM_BILATERAL_COMMAND,
)
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
from .bilateral_robot import BilateralArmRobotConfig

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

    def limits(self, side):
        if side not in ('left', 'right'):
            raise ValueError('side must be left or right')
        return self.lower_limits_rad, self.upper_limits_rad

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
        path = path or os.environ.get("TIANJI_ARM_CONFIG")
        path = Path(path) if path else Path(__file__).resolve().parents[4] / "src" / "tianji_teleop" / "config" / "robot" / "arm.yaml"
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
        robot_config: ArmRobotConfig | BilateralArmRobotConfig | Mapping[str, Any] | str | os.PathLike[str] | None = None,
        coordinator_config: Mapping[str, Any] | str | os.PathLike[str] | None = None,
        clock: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        if not publisher_instance_id or not router_zid:
            raise ValueError("publisher_instance_id and router_zid are required")
        self.session = session
        self.publisher_instance_id = publisher_instance_id
        self.router_zid = router_zid
        self.clock = clock
        self._lock = threading.RLock()
        self.profile = dict(profile or {})
        self._bilateral = self.profile.get('bilateral_proposals')
        if 'bilateral_proposals' in self.profile:
            if (not isinstance(self._bilateral, dict) or
                    set(self._bilateral) != {'run_id', 'execution_epoch'} or
                    not isinstance(self._bilateral['run_id'], str) or not self._bilateral['run_id'].strip() or
                    type(self._bilateral['execution_epoch']) is not int or not 0 < self._bilateral['execution_epoch'] < 2**63):
                raise ValueError('invalid bilateral_proposals session binding')
            if self.profile.get('required_capability') != 'simulation':
                raise ValueError('bilateral execution is currently simulation-only')
            if sorted(self.profile.get('active_sides', [])) != ['left', 'right']:
                raise ValueError('bilateral execution requires both active sides')
            self._bilateral = dict(self._bilateral)
        self._bilateral_tick = 0
        self._pending_bilateral = None
        self._last_bilateral_receipt = None
        self._last_bilateral_command = None
        if "authorities" in self.profile:
            self.authorities = self._validate_authorities(self.profile["authorities"])
        else:
            # Direct unit users may construct a coordinator without launcher
            # wiring; the production entry point always supplies this map.
            self.authorities = None
        if self.authorities is not None:
            coordinator_authority = self.authorities["coordinator_arm"]
            if (
                not coordinator_authority.get("enabled", True)
                or coordinator_authority["logical_id"] != "arm"
                or coordinator_authority["publisher_instance_id"] != publisher_instance_id
                or coordinator_authority["router_zid"] != router_zid
            ):
                raise ValueError("coordinator identity does not match coordinator_arm authority")
        if isinstance(robot_config, BilateralArmRobotConfig) and self._bilateral is None:
            raise ValueError('side-specific reference robot requires a bilateral simulation session')
        self.robot = robot_config if isinstance(robot_config, (ArmRobotConfig, BilateralArmRobotConfig)) else (ArmRobotConfig.from_mapping(robot_config) if isinstance(robot_config, Mapping) else ArmRobotConfig.load(robot_config))
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
        self._adopted_proposal_time_ns: dict[str, int] = {}
        self._step_rejection: dict[str, Any] | None = None
        self._hand_status: dict[str, _Timed] = {}
        self._hand_status_baseline: dict[str, tuple[str, int]] = {}
        self._hand_state: dict[str, _Timed] = {}
        self._hand_state_baseline: dict[str, tuple[str, int]] = {}
        self._return_started_ns: int | None = None
        self._teleop_started_ns: int | None = None
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

    def _profile_hand_sides(self) -> tuple[str, ...]:
        configured = self.profile.get(
            "active_hand_sides",
            self.profile.get("hand_sides", self.profile.get("active_sides", ("left", "right"))),
        )
        if isinstance(configured, str):
            configured = configured.split(",")
        return tuple(side for side in configured if side in ("left", "right"))

    @staticmethod
    def _identity_matches(expected: Mapping[str, Any], logical_id: str, instance: str, router: str) -> bool:
        return (
            bool(expected.get("enabled", True))
            and logical_id == expected.get("logical_id")
            and instance == expected.get("publisher_instance_id")
            and router == expected.get("router_zid")
        )

    def _expected_authority(self, role: str, side: str | None = None) -> Mapping[str, Any] | None:
        if self.authorities is None:
            return None
        value = self.authorities.get(role)
        if side is not None and isinstance(value, Mapping) and "logical_id" not in value:
            value = value.get(side)
        return value if isinstance(value, Mapping) else None

    def _matches_authority(self, role: str, logical_id: str, instance: str, router: str, *, side: str | None = None) -> bool:
        if self.authorities is None:
            return True
        value = self.authorities.get(role)
        if (
            role in {"producer_hand", "executor_hand"}
            and isinstance(value, Mapping)
            and "logical_id" not in value
        ):
            if side is not None:
                expected = value.get(side)
                return isinstance(expected, Mapping) and self._identity_matches(
                    expected, logical_id, instance, router
                )
            # A shared ComponentStatus has no side on the wire. It is valid
            # only when its identity is authorized for an active hand side;
            # this allows h5_direct/JointReplay to publish once while keeping
            # per-side executor authorities strict.
            return any(
                isinstance(expected := value.get(candidate), Mapping)
                and self._identity_matches(expected, logical_id, instance, router)
                for candidate in self._profile_hand_sides()
            )
        expected = self._expected_authority(role, side)
        return isinstance(expected, Mapping) and self._identity_matches(
            expected, logical_id, instance, router
        )


    @staticmethod
    def _coordinator_config(raw: Mapping[str, Any] | str | os.PathLike[str] | None) -> dict[str, float]:
        required = ("rate_hz", "proposal_timeout_s", "maximum_command_step_rad", "home_minimum_duration_s", "home_max_speed_rad_s", "home_tolerance_rad", "state_timeout_s", "hand_return_timeout_s")
        if raw is None:
            raw = Path(__file__).resolve().parents[4] / "src" / "tianji_teleop" / "config" / "coordinator" / "arm.yaml"
        if isinstance(raw, Mapping):
            value = dict(raw)
        else:
            try:
                import yaml
                value = yaml.safe_load(Path(raw).read_text(encoding="utf-8"))
            except Exception as exc:
                raise ValueError(f"unable to load coordinator config {raw}: {exc}") from exc
        if not isinstance(value, Mapping) or not set(required) <= set(value) or set(value) - set(required) - {'command_step_clipping_enabled', 'command_step_time_window_s'}:
            raise ValueError("coordinator config contains missing or unknown fields")
        result = {key: float(value[key]) for key in required}
        if any(not math.isfinite(x) or x <= 0.0 for x in result.values()):
            raise ValueError("coordinator config values must be finite and positive")
        clipping = value.get('command_step_clipping_enabled', True)
        if not isinstance(clipping, bool):
            raise ValueError('command_step_clipping_enabled must be boolean')
        result['command_step_clipping_enabled'] = clipping
        window = value.get('command_step_time_window_s', 0.0)
        if isinstance(window, bool) or not isinstance(window, (int, float)) or not math.isfinite(window) or not 0 <= window <= result['proposal_timeout_s']:
            raise ValueError('command_step_time_window_s must be finite, nonnegative and <= proposal_timeout_s')
        result['command_step_time_window_s'] = float(window)
        return result

    @property
    def state(self) -> SessionState:
        with self._lock:
            return self._state

    @property
    def at_home(self) -> LatchedBool:
        with self._lock:
            return self._at_home

    @property
    def return_complete(self) -> LatchedBool:
        with self._lock:
            return self._return_complete

    def _setup_transport(self, session: Any) -> None:
        # Deliberately use only canonical topic functions. Queryables reply with
        # the same typed payloads that subscribers receive.
        for key, getter in ((topics.SESSION_STATE, lambda: self._state), (topics.AT_HOME, lambda: self._at_home), (topics.RETURN_COMPLETE, lambda: self._return_complete)):
            if hasattr(session, "declare_queryable"):
                self._queryables.append(session.declare_queryable(key, lambda query, get=getter, key=key: self._reply_snapshot(query, key, get)))
        for side in ("left", "right"):
            if hasattr(session, "declare_publisher"):
                self._publishers[side] = session.declare_publisher(topics.arm_command(side))
        if hasattr(session, "declare_publisher"):
            self._publishers["state"] = session.declare_publisher(topics.SESSION_STATE)
            self._publishers["status"] = session.declare_publisher(topics.COORDINATOR_STATUS)
            self._publishers["home"] = session.declare_publisher(topics.AT_HOME)
            self._publishers["complete"] = session.declare_publisher(topics.RETURN_COMPLETE)
            if self._bilateral is not None:
                self._publishers['bilateral_receipt'] = session.declare_publisher(ARM_BILATERAL_RECEIPT)
                self._publishers['bilateral_command'] = session.declare_publisher(ARM_BILATERAL_COMMAND)

    def _publish(self, name: str, payload: Mapping[str, Any]) -> None:
        publisher = self._publishers.get(name)
        if publisher is not None:
            if hasattr(publisher, "put_json"):
                publisher.put_json(payload)
                return
            publisher.put(json.dumps(payload, separators=(",", ":")).encode("utf-8"), encoding="application/json")

    def _publish_session_snapshot(self) -> None:
        self._publish("state", self._state.to_dict())
        self._publish("home", self._at_home.to_dict())
        self._publish("complete", self._return_complete.to_dict())

    def _reply_snapshot(self, query: Any, key: str, getter: Callable[[], Any]) -> None:
        with self._lock:
            payload = json.dumps(getter().to_dict(), separators=(",", ":")).encode("utf-8")
            query.reply(key, payload)

    def _make_state(self, state: str, reason: str, intent_sequence: int | None) -> SessionState:
        return SessionState(1, self._sequence, self.clock(), state, reason, "coordinator", intent_sequence, self.publisher_instance_id, self.router_zid)

    def _next_state(self, state: str, reason: str, intent_sequence: int | None) -> SessionState:
        self._sequence += 1
        next_state = self._make_state(state, reason, intent_sequence)
        self._at_home = LatchedBool(
            1,
            self._sequence,
            next_state.timestamp_ns,
            self._at_home.value,
            self.publisher_instance_id,
            self.router_zid,
        )
        self._return_complete = LatchedBool(
            1,
            self._sequence,
            next_state.timestamp_ns,
            self._return_complete.value,
            self.publisher_instance_id,
            self.router_zid,
        )
        return next_state

    def _fresh(self, timed: _Timed | None, now_ns: int) -> bool:
        return timed is not None and 0 <= now_ns - timed.received_ns <= int(self.config["state_timeout_s"] * 1e9)

    def _domain_ready(self, role: str, now_ns: int) -> bool:
        required_capability = self.profile.get("required_capability", "simulation")
        side_map = self.authorities is not None and role in {"producer_hand", "executor_hand"} and isinstance(self.authorities.get(role), Mapping) and "logical_id" not in self.authorities[role]
        if side_map:
            for side in self._profile_hand_sides():
                expected = self._expected_authority(role, side)
                matches = [
                    timed for (entry_role, entry_id), timed in self._statuses.items()
                    if entry_role == role
                    and self._fresh(timed, now_ns)
                    and expected is not None
                    and self._matches_authority(role, entry_id, timed.value.publisher_instance_id, timed.value.router_zid, side=side)
                ]
                if (
                    len(matches) != 1
                    or not matches[0].value.ready
                    or not matches[0].value.healthy
                    or required_capability not in matches[0].value.capabilities
                ):
                    return False
            return True
        entries = [timed for (entry_role, _), timed in self._statuses.items() if entry_role == role and self._fresh(timed, now_ns)]
        if len(entries) != 1:
            return False
        status = entries[0].value
        if not self._matches_authority(role, status.component_id, status.publisher_instance_id, status.router_zid):
            return False
        return status.ready and status.healthy and required_capability in status.capabilities
    def update_component(self, status: ComponentStatus | Mapping[str, Any], *, received_ns: int | None = None) -> None:
        try:
            parsed = status if isinstance(status, ComponentStatus) else ComponentStatus.from_dict(status)
        except (ProtocolError, TypeError, ValueError) as exc:
            self._enter_fault(f"malformed component status: {exc}")
            return
        if parsed.router_zid != self.router_zid:
            self._enter_fault("component router_zid mismatch")
            return
        # Hand-tracking/XR sessions have a receive-only observation process in
        # addition to the target source.  It publishes the same status topic
        # for diagnostics, but it is deliberately not a lifecycle authority
        # and must not compete with the launcher's ``source`` identity.
        if (
            parsed.component_role == "source"
            and isinstance(parsed.diagnostics, Mapping)
            and parsed.diagnostics.get("observation_only") is True
        ):
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
        if not self._matches_authority(
            "executor_arm", parsed.executor, parsed.publisher_instance_id, parsed.router_zid
        ):
            self._enter_fault("arm state executor authority mismatch")
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
        if self._bilateral is not None:
            self._enter_fault('unpaired proposal on bilateral session')
            return False
        return self._update_single_proposal(proposal, received_ns=received_ns)

    def _update_single_proposal(self, proposal: ArmJointProposal | Mapping[str, Any], *, received_ns: int | None = None) -> bool:
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
            elif self.config['command_step_time_window_s'] > 0 and parsed.timestamp_ns < old.value.timestamp_ns:
                self._enter_fault("arm proposal timestamp rollback")
                return False
        self._proposals[parsed.side] = _Timed(parsed, observed_ns)
        return True

    @property
    def last_bilateral_receipt(self):
        """Coordinator disposition only; never represents actuator feedback."""
        return deepcopy(self._last_bilateral_receipt)

    def validate_bilateral_home_rearm(self, execution_epoch):
        """New simulation-only barrier; never clears an existing fault."""
        with self._lock:
            now = self.clock()
            if (self._bilateral is None or type(execution_epoch) is not int or
                    execution_epoch != self._bilateral['execution_epoch'] + 1 or execution_epoch >= 2**63 or
                    self._state.state != 'idle' or not self._return_ready(now) or not self._commands_at_home() or
                    list(self._arm_state.value.position_rad) != list(self.robot.home_all)):
                raise ValueError('bilateral rearm requires idle, fresh exact Home feedback and next epoch')

    def rearm_bilateral_at_home(self, execution_epoch):
        with self._lock:
            self.validate_bilateral_home_rearm(execution_epoch)
            self._bilateral['execution_epoch'] = execution_epoch
            self._bilateral_tick = 0
            self._pending_bilateral = None
            self._last_bilateral_receipt = None
            self._last_bilateral_command = None
            self._proposals.clear()
            self._adopted_proposal_time_ns.clear()
            self._step_rejection = None
            self._state = self._next_state('idle', 'explicit Home rearm; fresh input and start required', None)
            self._publish_session_snapshot()

    @property
    def last_bilateral_command(self):
        return deepcopy(self._last_bilateral_command)

    def update_bilateral_proposal(self, value, *, received_ns=None) -> bool:
        with self._lock:
            if self._bilateral is None:
                return False
            try:
                pair = ArmBilateralProposal.from_dict(value.to_dict() if isinstance(value, ArmBilateralProposal) else value)
            except (TypeError, ValueError) as exc:
                self._enter_fault(f'malformed bilateral proposal: {exc}')
                return False
            if (pair.run_id != self._bilateral['run_id'] or
                    pair.execution_epoch != self._bilateral['execution_epoch'] or
                    pair.tick_id <= self._bilateral_tick):
                return False  # old/foreign execution generations never roll back state
            if self._state.state != 'teleop':
                return False
            if self._pending_bilateral is not None:
                # No silent latest-only drop at this execution boundary. The
                # producer must account for every downstream disposition.
                self._enter_fault('bilateral proposal superseded before command tick')
                return False
            previous = dict(self._proposals)
            observed = self.clock() if received_ns is None else received_ns
            if not (self._update_single_proposal(pair.left, received_ns=observed) and
                    self._update_single_proposal(pair.right, received_ns=observed)):
                self._proposals = previous
                return False
            self._bilateral_tick = pair.tick_id
            self._pending_bilateral = pair
            return True

    def handle_proposal_dict(self, value: Mapping[str, Any]) -> bool:
        return self.update_proposal(value)

    def _arm_at_home(self, now_ns: int) -> bool:
        if not self._fresh(self._arm_state, now_ns):
            return False
        return all(abs(x - y) <= self.config["home_tolerance_rad"] for x, y in zip(self._arm_state.value.position_rad, self.robot.home_all))

    def _hand_enabled(self) -> bool:
        return bool(
            self.profile.get("hand_enabled")
            or self.profile.get("active_hand_sides")
            or self.profile.get("hand_sides")
        )

    def _hand_at_zero_ready(self, now_ns: int) -> bool:
        sides = self._profile_hand_sides()
        tolerance = float(self.profile.get("zero_tolerance_rad", 0.1))
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
        """Allow a healthy zeroed hand to wait for its first deadman input."""
        tolerance = float(self.profile.get("zero_tolerance_rad", 0.1))
        for side in self._profile_hand_sides():
            status = self._hand_status.get(side)
            state = self._hand_state.get(side)
            if not self._fresh(status, now_ns) or not self._fresh(state, now_ns):
                return False
            if status.value.publisher_instance_id != state.value.publisher_instance_id:
                return False
            waiting_at_zero = status.value.at_zero and all(
                abs(value) <= tolerance for value in state.value.position_rad
            )
            if not (
                status.value.ready
                and status.value.healthy
                and (status.value.tracking_allowed or waiting_at_zero)
            ):
                return False
        return True

    def _return_ready(self, now_ns: int) -> bool:
        return self._arm_at_home(now_ns) and (not self._hand_enabled() or self._hand_at_zero_ready(now_ns))

    def _domain_readiness_detail(self, role: str, now_ns: int) -> str | None:
        """Expose a component's own wait/failure reason in an intent result."""
        for (entry_role, _), timed in self._statuses.items():
            if entry_role != role or not self._fresh(timed, now_ns):
                continue
            if timed.value.error:
                return timed.value.error
            diagnostics = timed.value.diagnostics
            detail = diagnostics.get('readiness_reason') if isinstance(diagnostics, Mapping) else None
            if isinstance(detail, str) and detail:
                return detail
        return None

    def _start_ready(self, now_ns: int) -> tuple[bool, str]:
        for role in ("source", "producer_arm", "executor_arm"):
            if not self._domain_ready(role, now_ns):
                return False, f"{role} not exactly-one fresh healthy ready"
        if self._hand_enabled():
            if not self._domain_ready("producer_hand", now_ns):
                detail = self._domain_readiness_detail("producer_hand", now_ns)
                suffix = f": {detail}" if detail else ""
                return False, "producer_hand not exactly-one fresh healthy ready" + suffix
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
        action = getattr(intent, "action", None)
        sequence = getattr(intent, "sequence", None)
        reason = getattr(intent, "reason", "")
        source = getattr(intent, "source", None)
        intent_instance = getattr(intent, "publisher_instance_id", None)
        intent_router = getattr(intent, "router_zid", None)
        now_ns = self.clock()
        if self.authorities is not None and not self._matches_authority(
            "source", source, intent_instance, intent_router
        ):
            rejected = self._next_state(self._state.state, "intent source authority mismatch", sequence)
            self._state = rejected
            self._publish_session_snapshot()
            return IntentResult(False, rejected, "intent source authority mismatch")
        if self._state.state == "fault":
            rejected = self._next_state("fault", self._fault_reason or "fault latched", sequence)
            self._state = rejected
            self._publish_session_snapshot()
            return IntentResult(False, rejected, "fault latched; restart required")
        if action == "start" and self._state.state != "idle":
            rejected = self._next_state(self._state.state, "start requires idle", sequence)
            self._state = rejected
            self._publish_session_snapshot()
            return IntentResult(False, rejected, "start requires idle")
        if action == "start":
            ready, why = self._start_ready(now_ns)
            if not ready:
                rejected = self._next_state(self._state.state, why, sequence)
                self._state = rejected
                self._publish_session_snapshot()
                return IntentResult(False, rejected, why)
            self._proposals.clear()
            self._adopted_proposal_time_ns.clear()
            self._step_rejection = None
            self._teleop_started_ns = now_ns
            self._state = self._next_state("teleop", "accepted", sequence)
            self._at_home = LatchedBool(1, self._sequence, self._state.timestamp_ns, False, self.publisher_instance_id, self.router_zid)
            self._return_complete = LatchedBool(1, self._sequence, self._state.timestamp_ns, False, self.publisher_instance_id, self.router_zid)
            self._publish_session_snapshot()
            return IntentResult(True, self._state, "accepted")
        if action in ("return", "shutdown"):
            self._teleop_started_ns = None
            self._return_started_ns = now_ns
            self._return_start_command = {side: list(values) for side, values in self._safe_command.items()}
            self._state = self._next_state("returning", reason or action, sequence)
            self._return_complete = LatchedBool(1, self._sequence, self._state.timestamp_ns, False, self.publisher_instance_id, self.router_zid)
            self._publish_session_snapshot()
            return IntentResult(True, self._state, "accepted")
        return IntentResult(False, self._state, "unsupported intent")

    def _enter_returning(self, reason: str, now_ns: int) -> None:
        if self._bilateral is not None:
            self._enter_fault(reason)
            return
        if self._state.state in {"returning", "fault"}:
            return
        self._teleop_started_ns = None
        self._return_started_ns = now_ns
        self._return_start_command = {side: list(values) for side, values in self._safe_command.items()}
        self._state = self._next_state("returning", reason, self._state.intent_sequence)
        self._return_complete = LatchedBool(1, self._sequence, self._state.timestamp_ns, False, self.publisher_instance_id, self.router_zid)

    def _check_teleop_health(self, now_ns: int) -> None:
        if self._state.state != "teleop":
            return
        transitioning = (
            self._teleop_started_ns is not None
            and now_ns - self._teleop_started_ns
            <= int(self.config["state_timeout_s"] * 1e9)
        )
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
            if not self._hand_tracking_fresh(now_ns) and not transitioning:
                self._enter_fault("hand executor/status state stale, unhealthy, or identity mismatch")
                return
        for side in self.profile.get("active_sides", ("left", "right")):
            proposal = self._proposals.get(side)
            if proposal is None:
                continue
            if not self._fresh(proposal, now_ns):
                if transitioning:
                    continue
                self._enter_returning("arm proposal timeout", now_ns)
                return

    def _enter_fault(self, reason: str) -> None:
        self._teleop_started_ns = None
        self._fault_reason = reason
        if self._state.state != "fault":
            self._return_start_command = {side: list(values) for side, values in self._safe_command.items()}
            self._return_started_ns = self.clock()
            self._state = self._next_state("fault", reason, self._state.intent_sequence)
            self._return_complete = LatchedBool(1, self._sequence, self._state.timestamp_ns, False, self.publisher_instance_id, self.router_zid)
    def _command(self, side: str, sequence: int, timestamp_ns: int) -> ArmJointCommand:
        mode = "teleop" if self._state.state == "teleop" else ("idle" if self._state.state == "idle" else "returning")
        proposal = self._proposals.get(side)
        proposal_seq = target_seq = None
        home = list(getattr(self.robot, f"{side}_home_rad"))
        position = home
        if mode == "teleop" and side in set(self.profile.get("active_sides", ("left", "right"))):
            self._adopted_proposal_time_ns.setdefault(side, timestamp_ns)
            if proposal is not None and self._fresh(proposal, timestamp_ns):
                candidate = proposal.value
                maximum_step = self.config["maximum_command_step_rad"]
                position = [
                    old + max(-maximum_step, min(maximum_step, new - old))
                    for new, old in zip(candidate.position_rad, self._safe_command[side])
                ]
                if not self.config['command_step_clipping_enabled']:
                    position = list(candidate.position_rad)
                tracking_hold = candidate.diagnostics.get('tracking_hold') is True
                if tracking_hold:
                    position = list(self._safe_command[side])
                proposal_seq, target_seq = candidate.sequence, candidate.target_sequence
                # Heartbeats of the same proposal must not refresh its source
                # clock. A partially clipped target has not yet been adopted.
                stationary_failure_hold = candidate.diagnostics.get('hold') is True and position == self._safe_command[side]
                if position == candidate.position_rad and not stationary_failure_hold and not tracking_hold:
                    self._adopted_proposal_time_ns[side] = candidate.timestamp_ns
        elif mode == "returning":
            start = (self._return_start_command or self._safe_command)[side]
            elapsed = max(0.0, (timestamp_ns - (self._return_started_ns or timestamp_ns)) / 1e9)
            distance = max(abs(x - y) for x, y in zip(start, home))
            duration = max(self.config["home_minimum_duration_s"], distance / self.config["home_max_speed_rad_s"])
            fraction = min(1.0, elapsed / duration)
            # Preserve the exact endpoint required by the Home/return barrier;
            # x + 1 * (home - x) can differ from home by one floating-point ULP.
            position = home if fraction >= 1.0 else [x + fraction * (y - x) for x, y in zip(start, home)]
        self._safe_command[side] = position
        if self._bilateral is not None and self._state.state == 'fault':
            # New integration faults hold; old profiles retain bounded Home.
            position = list((self._return_start_command or self._safe_command)[side])
            self._safe_command[side] = position
        if mode != "teleop":
            self._adopted_proposal_time_ns.pop(side, None)
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
            tracking_hold = candidate.diagnostics.get('tracking_hold') is True
            if tracking_hold and self.profile.get('required_capability', 'simulation') != 'simulation':
                self._enter_fault("tracking hold is simulation-only")
                return
            if not all(math.isfinite(x) and lo <= x <= hi for x, lo, hi in zip(candidate.position_rad, *self.robot.limits(side))):
                self._enter_fault("proposal exceeds hard joint limits or is nonfinite")
                return
            allowed = 2.0 * self.config["maximum_command_step_rad"]
            window = self.config['command_step_time_window_s']
            elapsed_s = 0.0
            if window > 0:
                anchor = self._adopted_proposal_time_ns.get(side, self._teleop_started_ns)
                if anchor is not None:
                    elapsed_s = (candidate.timestamp_ns - anchor) / 1e9
                    if elapsed_s < 0:
                        self._enter_fault("arm proposal timestamp precedes adopted command")
                        return
                    speed = self.config['maximum_command_step_rad'] * self.config['rate_hz']
                    allowed = max(allowed, speed * min(elapsed_s, window))
                if not 0 <= now_ns-candidate.timestamp_ns <= self.config['proposal_timeout_s']*1e9:
                    self._enter_fault("arm proposal source timestamp stale")
                    return
            if tracking_hold:
                # Keep our own final command, never the producer's delayed
                # feedback. Authority, freshness and hard bounds still apply.
                continue
            delta = max(abs(x - old) for x, old in zip(candidate.position_rad, self._safe_command[side]))
            if delta > allowed + (1e-10 if window > 0 else 0.0):
                self._step_rejection = {'side': side, 'delta_rad': delta,
                    'allowed_rad': allowed, 'proposal_elapsed_s': elapsed_s,
                    'time_window_s': window, 'proposal_sequence': candidate.sequence}
                self._enter_fault("proposal exceeds maximum command step")

    def tick(self, *, now_ns: int | None = None) -> dict[str, ArmJointCommand]:
        with self._lock:
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
            if self._bilateral is not None:
                command_pair = ArmBilateralCommand(self._bilateral['run_id'], self._bilateral['execution_epoch'],
                    commands['left'].proposal_sequence if self._state.state == 'teleop' else None,
                    commands['left'], commands['right'])
                self._last_bilateral_command = command_pair.to_dict()
                self._publish('bilateral_command', self._last_bilateral_command)
            for side, command in commands.items():
                self._publish(side, command.to_dict())
            if self._pending_bilateral is not None:
                pair, self._pending_bilateral = self._pending_bilateral, None
                accepted = self._state.state == 'teleop' and all(
                    commands[side].proposal_sequence == getattr(pair, side).sequence for side in ('left', 'right'))
                self._last_bilateral_receipt = dict(schema_version=1, kind='arm_bilateral_receipt',
                    run_id=pair.run_id, execution_epoch=pair.execution_epoch, tick_id=pair.tick_id,
                    timestamp_ns=timestamp_ns, publisher_instance_id=self.publisher_instance_id,
                    router_zid=self.router_zid, stage='coordinator_command', accepted=accepted,
                    reason='accepted' if accepted else self._state.reason,
                    command_position_rad={side: list(commands[side].position_rad) for side in ('left', 'right')})
                self._publish('bilateral_receipt', self._last_bilateral_receipt)
            self._publish("state", self._state.to_dict())
            status = ComponentStatus(
                1,
                self._sequence,
                timestamp_ns,
                "coordinator_arm",
                "arm",
                self._state.state,
                self._state.state != "fault",
                self._state.state != "fault",
                [str(self.profile.get("required_capability", "simulation"))],
                self._fault_reason if self._state.state == "fault" else None,
                {"authority": "final_command_and_session_state",
                 "command_step_clipping_enabled": self.config['command_step_clipping_enabled'],
                 "command_step_time_window_s": self.config['command_step_time_window_s'],
                 "step_rejection": self._step_rejection},
                self.publisher_instance_id,
                self.router_zid,
            )
            self._publish("status", status.to_dict())
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
        if self._bilateral is not None:
            callbacks = [(key, callback) for key, callback in callbacks
                         if key not in (topics.arm_proposal('left'), topics.arm_proposal('right'))]
            callbacks.append((ARM_BILATERAL_PROPOSAL, self._on_bilateral_payload))
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

    def _on_bilateral_payload(self, sample: Any) -> None:
        with self._lock:
            try:
                self.update_bilateral_proposal(self._payload(sample))
            except (TypeError, ValueError):
                self._enter_fault('malformed bilateral proposal')

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
        with self._lock:
            try:
                payload = self._payload(sample)
                from ..protocol.messages import SessionIntent
                self.handle_intent(SessionIntent.from_dict(payload))
            except (ProtocolError, TypeError, ValueError, json.JSONDecodeError):
                self._enter_fault("malformed session intent")


    def _on_hand_executor_status_payload(self, sample: Any) -> None:
        with self._lock:
            try:
                self.update_hand_executor_status(self._payload(sample))
            except (ProtocolError, TypeError, ValueError, json.JSONDecodeError):
                self._enter_fault("malformed hand executor status")

    def _on_hand_state_payload(self, sample: Any) -> None:
        with self._lock:
            try:
                self.update_hand_state(self._payload(sample))
            except (ProtocolError, TypeError, ValueError, json.JSONDecodeError):
                self._enter_fault("malformed hand state")
    def _on_component_payload(self, sample: Any) -> None:
        with self._lock:
            try:
                self.update_component(self._payload(sample))
            except (ProtocolError, TypeError, ValueError, json.JSONDecodeError):
                self._enter_fault("malformed component status")

    def _on_arm_state_payload(self, sample: Any) -> None:
        with self._lock:
            try:
                self.update_arm_state(self._payload(sample))
            except (ProtocolError, TypeError, ValueError, json.JSONDecodeError):
                self._enter_fault("malformed arm state")

    def _on_proposal_payload(self, sample: Any) -> None:
        with self._lock:
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
    clipping = os.environ.get('TIANJI_COMMAND_STEP_CLIPPING')
    if clipping is not None:
        if clipping not in ('true', 'false'):
            raise ValueError('TIANJI_COMMAND_STEP_CLIPPING must be true or false')
        coordinator_config = ArmCommandCoordinator._coordinator_config(coordinator_config)
        coordinator_config['command_step_clipping_enabled'] = clipping == 'true'
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
