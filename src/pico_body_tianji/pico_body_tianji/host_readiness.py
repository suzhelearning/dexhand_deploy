"""Marvin canonical connection/start readiness gates.

The bridge consumes the same typed arm command, component status, session state,
and arm state as the coordinator.  Robot homes, limits, and joint ordering come
from ``config/robot/arm.yaml`` through :class:`ArmRobotConfig`; this module does
not carry a second set of robot constants.
"""
from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Mapping

import numpy as np

from .coordination.arm_command_coordinator import ArmRobotConfig
from .protocol.messages import (
    ARM_JOINT_NAMES,
    ArmJointCommand,
    ArmJointState,
    ComponentStatus,
    ProtocolError,
    SessionState,
)


SIDES = ("left", "right")


@dataclass(frozen=True)
class HostReadiness:
    ready: bool
    reason: str
    left_joints_deg: np.ndarray | None = None
    right_joints_deg: np.ndarray | None = None


@dataclass(frozen=True)
class _Timed:
    value: Any
    received_ns: int


class HostReadinessGate:
    """纯 canonical wire 的 Marvin connection/start/fault-return 门。"""

    def __init__(
        self,
        *,
        robot_config: ArmRobotConfig | Mapping[str, Any] | str | None = None,
        router_zid: str = "",
        freshness_timeout_s: float = 1.0,
        command_timeout_s: float = 0.2,
        maximum_pair_skew_s: float = 0.03,
        home_tolerance_rad: float | None = None,
    ) -> None:
        if isinstance(robot_config, ArmRobotConfig):
            self._robot = robot_config
        elif isinstance(robot_config, Mapping):
            self._robot = ArmRobotConfig.from_mapping(robot_config)
        else:
            self._robot = ArmRobotConfig.load(robot_config)
        self._router_zid = str(router_zid)
        for name, value in (
            ("freshness_timeout_s", freshness_timeout_s),
            ("command_timeout_s", command_timeout_s),
        ):
            if float(value) <= 0.0:
                raise ValueError(f"{name} must be positive")
        if float(maximum_pair_skew_s) < 0.0:
            raise ValueError("maximum_pair_skew_s must be non-negative")
        tolerance = self._robot.home_tolerance_rad if hasattr(self._robot, "home_tolerance_rad") else home_tolerance_rad
        self._freshness_timeout_ns = int(float(freshness_timeout_s) * 1e9)
        self._command_timeout_ns = int(float(command_timeout_s) * 1e9)
        self._maximum_pair_skew_ns = int(float(maximum_pair_skew_s) * 1e9)
        self._home_tolerance_rad = float(tolerance if tolerance is not None else np.deg2rad(1.0))
        if not np.isfinite(self._home_tolerance_rad) or self._home_tolerance_rad <= 0.0:
            raise ValueError("home_tolerance_rad must be positive and finite")
        self._components: dict[str, _Timed] = {}
        self._commands: dict[str, _Timed] = {}
        self._session_state: _Timed | None = None
        self._arm_state: _Timed | None = None
        self._executor_status: _Timed | None = None
        self._last_error: str | None = None

    @staticmethod
    def _received(received_at: float | int | None) -> int:
        if received_at is None:
            return time.monotonic_ns()
        value = float(received_at)
        # Existing bridge callbacks use monotonic seconds; canonical callers may
        # pass an ns integer.  Normalize both without consulting wall clock.
        return int(value if abs(value) > 1e12 else value * 1e9)

    def _fresh(self, timed: _Timed | None, now_ns: int, timeout_ns: int | None = None) -> bool:
        timeout_ns = self._freshness_timeout_ns if timeout_ns is None else timeout_ns
        return timed is not None and 0 <= now_ns - timed.received_ns <= timeout_ns

    @staticmethod
    def _typed(value: Any, cls: type, field: str) -> Any:
        try:
            return value if isinstance(value, cls) else cls.from_dict(value)
        except (ProtocolError, TypeError, ValueError) as exc:
            raise ValueError(f"invalid {field}: {exc}") from exc

    def observe_component(self, status: ComponentStatus | Mapping[str, Any], *, received_at: float | int | None = None) -> None:
        parsed = self._typed(status, ComponentStatus, "component status")
        if parsed.component_role not in {"source", "producer_arm", "executor_arm"}:
            raise ValueError(f"unsupported readiness component role: {parsed.component_role}")
        self._components[parsed.component_role] = _Timed(parsed, self._received(received_at))
        if parsed.component_role == "executor_arm":
            self._executor_status = self._components[parsed.component_role]

    def observe_command(self, command: ArmJointCommand | Mapping[str, Any], *, received_at: float | int | None = None) -> None:
        parsed = self._typed(command, ArmJointCommand, "arm command")
        if parsed.router_zid != self._robot_router():
            raise ValueError("arm command router_zid mismatch")
        if tuple(parsed.names) != ARM_JOINT_NAMES[parsed.side]:
            raise ValueError("arm command joint order mismatch")
        received_ns = self._received(received_at)
        if parsed.timestamp_ns > received_ns or received_ns - parsed.timestamp_ns > self._command_timeout_ns:
            raise ValueError("arm command timestamp is stale")
        self._commands[parsed.side] = _Timed(parsed, received_ns)

    def observe_session_state(self, state: SessionState | Mapping[str, Any], *, received_at: float | int | None = None) -> None:
        parsed = self._typed(state, SessionState, "session state")
        if parsed.router_zid != self._robot_router():
            raise ValueError("session state router_zid mismatch")
        self._session_state = _Timed(parsed, self._received(received_at))

    def observe_arm_state(self, state: ArmJointState | Mapping[str, Any], *, received_at: float | int | None = None) -> None:
        parsed = self._typed(state, ArmJointState, "arm state")
        if parsed.router_zid != self._robot_router() or tuple(parsed.names) != self._robot_names():
            raise ValueError("arm state identity/order mismatch")
        self._arm_state = _Timed(parsed, self._received(received_at))

    def set_router(self, router_zid: str) -> None:
        if not router_zid:
            raise ValueError("router_zid must be non-empty")
        self._router_zid = router_zid

    def _robot_router(self) -> str:
        # Set by the bridge/launcher before any wire sample is consumed.  An
        # unset router is a fail-closed state, never a guessed default.
        return getattr(self, "_router_zid", "")

    def _robot_names(self) -> tuple[str, ...]:
        return self._robot.left_joint_names + self._robot.right_joint_names

    def _commands_at_home(self) -> bool:
        for side in SIDES:
            timed = self._commands.get(side)
            if timed is None:
                return False
            command = timed.value
            home = getattr(self._robot, f"{side}_home_rad")
            if command.mode != "idle" or any(abs(x - y) > self._home_tolerance_rad for x, y in zip(command.position_rad, home)):
                return False
        return True
    def _base_connection(self, now_ns: int, required_capability: str) -> HostReadiness:
        source = self._components.get("source")
        if not self._fresh(source, now_ns):
            return HostReadiness(False, "source status stale or missing")
        if not (source.value.ready and source.value.healthy and required_capability in source.value.capabilities):
            return HostReadiness(False, f"source not {required_capability}-capable and healthy")
        producer = self._components.get("producer_arm")
        if not self._fresh(producer, now_ns):
            return HostReadiness(False, "producer_arm status stale or missing")
        if not (producer.value.ready and producer.value.healthy):
            return HostReadiness(False, "producer_arm not loaded and healthy")
        state = self._session_state
        if not self._fresh(state, now_ns):
            return HostReadiness(False, "coordinator state stale or missing")
        if state.value.state != "idle":
            return HostReadiness(False, "coordinator not idle")
        if not all(self._fresh(self._commands.get(side), now_ns, self._command_timeout_ns) for side in SIDES):
            return HostReadiness(False, "coordinator command stale or missing")
        if not self._commands_at_home():
            return HostReadiness(False, "coordinator command is not at Home")
        return HostReadiness(True, "ready")


    def evaluate_connection(self, *, now_ns: int, required_capability: str = "real") -> HostReadiness:
        decision = self._base_connection(int(now_ns), required_capability)
        if not decision.ready:
            return decision
        return HostReadiness(
            True,
            decision.reason,
            left_joints_deg=np.degrees(self._commands["left"].value.position_rad),
            right_joints_deg=np.degrees(self._commands["right"].value.position_rad),
        )

    def evaluate_start(self, *, now_ns: int) -> HostReadiness:
        decision = self._base_connection(int(now_ns), "real")
        if not decision.ready:
            return decision
        if not self._fresh(self._executor_status, int(now_ns)):
            return HostReadiness(False, "executor status stale or missing")
        if not (self._executor_status.value.ready and self._executor_status.value.healthy):
            return HostReadiness(False, "executor not ready and healthy")
        if not self._fresh(self._arm_state, int(now_ns)):
            return HostReadiness(False, "arm state stale or missing")
        home = np.asarray(self._robot.home_all, dtype=np.float64)
        if np.max(np.abs(np.asarray(self._arm_state.value.position_rad) - home), initial=0.0) > self._home_tolerance_rad:
            return HostReadiness(False, "arm state is not at Home")
        return HostReadiness(True, "ready")

    def evaluate_fault_return(self, *, now_ns: int) -> HostReadiness:
        state = self._session_state
        if not self._fresh(state, int(now_ns)) or state.value.state != "fault":
            return HostReadiness(False, "coordinator is not in fault")
        commands = [self._commands.get(side) for side in SIDES]
        if any(not self._fresh(value, int(now_ns), self._command_timeout_ns) for value in commands):
            return HostReadiness(False, "bounded Home command stale or missing")
        if any(value.value.mode != "returning" for value in commands):
            return HostReadiness(False, "fault command must be returning")
        return HostReadiness(True, "fault_return")

    def evaluate(self, *, now: float) -> HostReadiness:
        """Compatibility-free canonical connection evaluation in seconds."""
        return self.evaluate_connection(now_ns=self._received(now), required_capability="real")


__all__ = ["HostReadiness", "HostReadinessGate"]
