"""Marvin executor 的 connection/start/fault-return readiness。

Readiness 只消费 canonical coordinator/source/producer wire，不从 generic
legacy status 推断 authority。source 的 ``real`` capability 与 producer 的
``loaded/healthy`` 是两个独立条件。
"""
from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Mapping

import numpy as np

from ...coordination.arm_command_coordinator import ArmRobotConfig
from ...protocol.messages import (
    ARM_JOINT_NAMES,
    ArmJointCommand,
    ArmJointState,
    ComponentStatus,
    ProtocolError,
    SessionState,
)


@dataclass(frozen=True)
class ReadinessDecision:
    ready: bool
    reason: str


@dataclass(frozen=True)
class _Timed:
    value: Any
    received_ns: int


class MarvinReadiness:
    """Fail-closed canonical gates used before a Marvin SDK connection."""

    def __init__(
        self,
        *,
        robot_config: ArmRobotConfig | Mapping[str, Any] | str | None = None,
        router_zid: str,
        freshness_timeout_s: float = 1.0,
        command_timeout_s: float = 0.2,
        home_tolerance_rad: float = np.deg2rad(1.0),
        expected_authorities: Mapping[str, Mapping[str, Any]] | None = None,
    ) -> None:
        if not router_zid:
            raise ValueError("router_zid is required")
        if freshness_timeout_s <= 0.0 or command_timeout_s <= 0.0 or home_tolerance_rad <= 0.0:
            raise ValueError("readiness timeouts and tolerance must be positive")
        if isinstance(robot_config, ArmRobotConfig):
            self.robot = robot_config
        elif isinstance(robot_config, Mapping):
            self.robot = ArmRobotConfig.from_mapping(robot_config)
        else:
            self.robot = ArmRobotConfig.load(robot_config)
        self.router_zid = router_zid
        self.freshness_timeout_ns = int(float(freshness_timeout_s) * 1e9)
        self.command_timeout_ns = int(float(command_timeout_s) * 1e9)
        self.home_tolerance_rad = float(home_tolerance_rad)
        self.expected_authorities = {
            str(role): dict(value)
            for role, value in (expected_authorities or {}).items()
            if isinstance(value, Mapping) and value.get("enabled", True)
        }
        self._components: dict[tuple[str, str], _Timed] = {}
        self._component_instances: dict[tuple[str, str], str] = {}
        self._commands: dict[str, _Timed] = {}
        self._command_baseline: dict[str, tuple[str, int]] = {}
        self._state: _Timed | None = None
        self._arm_state: _Timed | None = None
        self.last_error = ""
    @staticmethod
    def _received(value: int | float | None) -> int:
        if value is None:
            return time.monotonic_ns()
        if isinstance(value, int):
            return value
        value = float(value)
        return int(value if abs(value) > 1e12 else value * 1e9)

    def _fresh(self, value: _Timed | None, now_ns: int, timeout_ns: int | None = None) -> bool:
        timeout_ns = self.freshness_timeout_ns if timeout_ns is None else timeout_ns
        return value is not None and 0 <= now_ns - value.received_ns <= timeout_ns
    def observe_component(self, status: ComponentStatus | Mapping[str, Any], *, received_ns: int | float | None = None) -> None:
        try:
            status = status if isinstance(status, ComponentStatus) else ComponentStatus.from_dict(status)
        except (ProtocolError, TypeError, ValueError) as exc:
            self.last_error = f"malformed component status: {exc}"
            return
        if status.router_zid != self.router_zid:
            self.last_error = "component router_zid mismatch"
            return
        expected = self.expected_authorities.get(status.component_role)
        if expected is not None:
            if (
                str(expected.get("logical_id", "")) != status.component_id
                or str(expected.get("publisher_instance_id", "")) != status.publisher_instance_id
                or str(expected.get("router_zid", self.router_zid)) != status.router_zid
            ):
                self.last_error = f"component authority mismatch for {status.component_role}/{status.component_id}"
                return
        key = (status.component_role, status.component_id)
        previous = self._component_instances.get(key)
        if previous is not None and previous != status.publisher_instance_id:
            self.last_error = f"duplicate authority for {status.component_role}/{status.component_id}"
            return
        previous_timed = self._components.get(key)
        if previous_timed is not None and status.sequence <= previous_timed.value.sequence:
            self.last_error = f"component status sequence rollback for {key}"
            return
        self._component_instances[key] = status.publisher_instance_id
        self._components[key] = _Timed(status, self._received(received_ns))

    def observe_command(self, command: ArmJointCommand | Mapping[str, Any], *, received_ns: int | float | None = None) -> bool:
        try:
            command = command if isinstance(command, ArmJointCommand) else ArmJointCommand.from_dict(command)
        except (ProtocolError, TypeError, ValueError) as exc:
            self.last_error = f"malformed arm command: {exc}"
            return False
        if command.router_zid != self.router_zid or tuple(command.names) != ARM_JOINT_NAMES[command.side]:
            self.last_error = "arm command identity/order mismatch"
            return False
        previous = self._command_baseline.get(command.side)
        if previous is not None:
            instance, sequence = previous
            if instance != command.publisher_instance_id or sequence >= command.sequence:
                self.last_error = f"arm command sequence rollback for {command.side}"
                return False
        received = self._received(received_ns)
        if command.timestamp_ns > received or received - command.timestamp_ns > self.command_timeout_ns:
            self.last_error = f"arm command stale for {command.side}"
            return False
        self._command_baseline[command.side] = (command.publisher_instance_id, command.sequence)
        self._commands[command.side] = _Timed(command, received)
        return True

    def observe_session_state(self, state: SessionState | Mapping[str, Any], *, received_ns: int | float | None = None) -> None:
        try:
            state = state if isinstance(state, SessionState) else SessionState.from_dict(state)
        except (ProtocolError, TypeError, ValueError) as exc:
            self.last_error = f"malformed session state: {exc}"
            return
        if state.router_zid != self.router_zid:
            self.last_error = "session state router_zid mismatch"
            return
        if self._state is not None and state.publisher_instance_id == self._state.value.publisher_instance_id and state.sequence <= self._state.value.sequence:
            self.last_error = "session state sequence rollback"
            return
        self._state = _Timed(state, self._received(received_ns))

    def observe_arm_state(self, state: ArmJointState | Mapping[str, Any], *, received_ns: int | float | None = None) -> None:
        try:
            state = state if isinstance(state, ArmJointState) else ArmJointState.from_dict(state)
        except (ProtocolError, TypeError, ValueError) as exc:
            self.last_error = f"malformed arm state: {exc}"
            return
        if state.router_zid != self.router_zid or tuple(state.names) != tuple(ARM_JOINT_NAMES["left"] + ARM_JOINT_NAMES["right"]):
            self.last_error = "arm state identity/order mismatch"
            return
        self._arm_state = _Timed(state, self._received(received_ns))

    def _one(self, role: str, now_ns: int) -> _Timed | None:
        values = [value for (candidate, _), value in self._components.items() if candidate == role and self._fresh(value, now_ns)]
        return values[0] if len(values) == 1 else None

    def _commands_at_home(self, now_ns: int) -> bool:
        for side in ("left", "right"):
            timed = self._commands.get(side)
            if not self._fresh(timed, now_ns, self.command_timeout_ns):
                return False
            command = timed.value
            home = getattr(self.robot, f"{side}_home_rad")
            if command.mode != "idle" or any(abs(x - y) > self.home_tolerance_rad for x, y in zip(command.position_rad, home)):
                return False
        return True

    def _commands_bounded_home(self, now_ns: int) -> bool:
        lower = self.robot.lower_limits_rad
        upper = self.robot.upper_limits_rad
        for side in ("left", "right"):
            timed = self._commands.get(side)
            if not self._fresh(timed, now_ns, self.command_timeout_ns):
                return False
            command = timed.value
            if command.mode != "returning":
                return False
            if any(value < lo or value > hi for value, lo, hi in zip(command.position_rad, lower, upper)):
                return False
        return True

    def connection_ready(self, *, now_ns: int) -> bool:
        source = self._one("source", int(now_ns))
        producer = self._one("producer_arm", int(now_ns))
        state = self._state
        if source is None or not (source.value.ready and source.value.healthy and "real" in source.value.capabilities):
            self.last_error = "source is not fresh real-capable"
            return False
        # The producer need only be loaded/healthy. Simulation capability is
        # acceptable here; source is the component that authorizes real input.
        if producer is None or not (producer.value.ready and producer.value.healthy):
            self.last_error = "arm producer is not fresh and healthy"
            return False
        if not self._fresh(state, int(now_ns)) or state.value.state != "idle":
            self.last_error = "coordinator is not fresh idle"
            return False
        if not self._commands_at_home(int(now_ns)):
            self.last_error = "coordinator command is not fresh at Home"
            return False
        return True

    def start_ready(self, *, now_ns: int) -> bool:
        return self.connection_ready(now_ns=now_ns) and self._fresh(self._arm_state, int(now_ns)) and self._arm_at_home()

    def _arm_at_home(self) -> bool:
        if self._arm_state is None:
            return False
        values = np.asarray(self._arm_state.value.position_rad, dtype=np.float64)
        home = np.asarray(self.robot.home_all, dtype=np.float64)
        return bool(np.max(np.abs(values - home), initial=0.0) <= self.home_tolerance_rad)

    def fault_return_ready(self, *, now_ns: int) -> bool:
        """允许 returning/fault 期间仅凭 fresh bounded command 安全重连。"""
        return bool(
            self._fresh(self._state, int(now_ns))
            and self._state.value.state in {"fault", "returning"}
            and self._commands_bounded_home(int(now_ns))
        )


__all__ = ["MarvinReadiness", "ReadinessDecision"]
