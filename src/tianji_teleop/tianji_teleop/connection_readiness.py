"""Marvin connection, start, and fault-return readiness gates."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


@dataclass(frozen=True)
class ReadinessDecision:
    ready: bool
    reason: str


def _field(value: Any, key: str, default: Any = None) -> Any:
    return value.get(key, default) if isinstance(value, Mapping) else getattr(value, key, default)


def _fresh(value: Any, now_ns: int, timeout_ns: int) -> bool:
    timestamp = _field(value, "timestamp_ns")
    return (
        isinstance(timestamp, int)
        and not isinstance(timestamp, bool)
        and timestamp >= 0
        and 0 <= now_ns - timestamp <= timeout_ns
    )


def _healthy(value: Any, capability: str) -> bool:
    caps = _field(value, "capabilities", ())
    return bool(
        _field(value, "ready", False)
        and _field(value, "healthy", False)
        and isinstance(caps, (list, tuple, set))
        and capability in caps
    )


def _check_identity(value: Any, *, router_zid: str | None, instance_id: str | None, label: str) -> str | None:
    if router_zid is not None and _field(value, "router_zid") != router_zid:
        return f"{label} router_zid mismatch"
    if instance_id is not None and _field(value, "publisher_instance_id") != instance_id:
        return f"{label} publisher instance mismatch"
    return None


def _command_values(command: Any, side: str, *, expected_names: Mapping[str, tuple[str, ...]] | None, home: Mapping[str, tuple[float, ...]] | None, tolerance: float) -> str | None:
    if _field(command, "side") not in (None, side):
        return f"{side} command side mismatch"
    if _field(command, "mode") != "idle":
        return f"{side} command is not idle"
    names = _field(command, "names", ())
    positions = _field(command, "position_rad", ())
    if not isinstance(names, (list, tuple)) or not isinstance(positions, (list, tuple)) or len(names) != len(positions):
        return f"{side} command shape invalid"
    if expected_names is not None and tuple(names) != tuple(expected_names[side]):
        return f"{side} command joint order mismatch"
    if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)) for value in positions):
        return f"{side} command contains non-finite position"
    if home is not None:
        if len(positions) != len(home[side]) or any(abs(float(value) - float(target)) > tolerance for value, target in zip(positions, home[side])):
            return f"{side} command is not at Home"
    return None


def evaluate_connection(
    *,
    source_status: Any,
    arm_producer_status: Any,
    coordinator_state: Any,
    arm_command: Any,
    now_ns: int,
    freshness_timeout_s: float = 1.0,
    required_capability: str = "real",
    expected_router_zid: str | None = None,
    expected_source_instance_id: str | None = None,
    expected_arm_producer_instance_id: str | None = None,
    expected_coordinator_instance_id: str | None = None,
    expected_arm_joint_names: Mapping[str, tuple[str, ...]] | None = None,
    home_positions_rad: Mapping[str, tuple[float, ...]] | None = None,
    home_tolerance_rad: float = 1e-6,
) -> ReadinessDecision:
    """Evaluate initial Marvin connection without requiring policy observation."""
    if freshness_timeout_s <= 0.0 or home_tolerance_rad < 0.0:
        raise ValueError("readiness timeouts/tolerance must be valid")
    cutoff = int(float(freshness_timeout_s) * 1e9)
    for name, value in (("source", source_status), ("arm producer", arm_producer_status), ("coordinator", coordinator_state)):
        if not _fresh(value, int(now_ns), cutoff):
            return ReadinessDecision(False, f"{name} stale or missing")
    for value, instance, label in (
        (source_status, expected_source_instance_id, "source"),
        (arm_producer_status, expected_arm_producer_instance_id, "arm producer"),
        (coordinator_state, expected_coordinator_instance_id, "coordinator"),
    ):
        reason = _check_identity(value, router_zid=expected_router_zid, instance_id=instance, label=label)
        if reason:
            return ReadinessDecision(False, reason)
    if not _healthy(source_status, required_capability):
        return ReadinessDecision(False, "source not real-capable and healthy")
    if not (_field(arm_producer_status, "ready", False) and _field(arm_producer_status, "healthy", False)):
        return ReadinessDecision(False, "arm producer not loaded and healthy")
    if _field(coordinator_state, "state") != "idle":
        return ReadinessDecision(False, "coordinator not idle")
    commands = arm_command if isinstance(arm_command, Mapping) and set(arm_command) >= {"left", "right"} else {"right": arm_command}
    for side in ("left", "right") if "left" in commands else ("right",):
        command = commands.get(side)
        if command is None or not _fresh(command, int(now_ns), cutoff):
            return ReadinessDecision(False, f"{side} command stale or missing")
        reason = _check_identity(command, router_zid=expected_router_zid, instance_id=expected_coordinator_instance_id, label=f"{side} command")
        if reason:
            return ReadinessDecision(False, reason)
        reason = _command_values(command, side, expected_names=expected_arm_joint_names, home=home_positions_rad, tolerance=home_tolerance_rad)
        if reason:
            return ReadinessDecision(False, reason)
    return ReadinessDecision(True, "ready")


def evaluate_start(*, executor_status: Any, arm_state: Any, coordinator_state: Any, now_ns: int, freshness_timeout_s: float = 1.0, expected_router_zid: str | None = None, expected_executor_instance_id: str | None = None) -> ReadinessDecision:
    """Evaluate the second gate after the executor has connected and reported state."""
    if freshness_timeout_s <= 0.0:
        raise ValueError("freshness_timeout_s must be positive")
    cutoff = int(float(freshness_timeout_s) * 1e9)
    for name, value in (("executor", executor_status), ("arm state", arm_state), ("coordinator", coordinator_state)):
        if not _fresh(value, int(now_ns), cutoff):
            return ReadinessDecision(False, f"{name} stale or missing")
    for value, instance, label in ((executor_status, expected_executor_instance_id, "executor"), (arm_state, expected_executor_instance_id, "arm state"), (coordinator_state, None, "coordinator")):
        reason = _check_identity(value, router_zid=expected_router_zid, instance_id=instance, label=label)
        if reason:
            return ReadinessDecision(False, reason)
    if not (_field(executor_status, "ready", False) and _field(executor_status, "healthy", False)):
        return ReadinessDecision(False, "executor not ready and healthy")
    if _field(coordinator_state, "state") != "idle":
        return ReadinessDecision(False, "coordinator not idle")
    return ReadinessDecision(True, "ready")


@dataclass(frozen=True)
class FaultReturnDecision:
    allowed: bool
    reason: str


def evaluate_fault_return(*, coordinator_state: Any, arm_command: Any, now_ns: int, freshness_timeout_s: float = 1.0, expected_router_zid: str | None = None, expected_coordinator_instance_id: str | None = None) -> FaultReturnDecision:
    """In fault, permit only fresh coordinator-owned bounded Home commands."""
    if _field(coordinator_state, "state") != "fault":
        return FaultReturnDecision(False, "coordinator is not fault")
    reason = _check_identity(coordinator_state, router_zid=expected_router_zid, instance_id=expected_coordinator_instance_id, label="coordinator")
    if reason:
        return FaultReturnDecision(False, reason)
    commands = arm_command if isinstance(arm_command, Mapping) and set(arm_command) >= {"left", "right"} else {"right": arm_command}
    cutoff = int(float(freshness_timeout_s) * 1e9)
    for side, command in commands.items():
        if not _fresh(command, int(now_ns), cutoff):
            return FaultReturnDecision(False, f"{side} bounded Home command stale")
        reason = _check_identity(command, router_zid=expected_router_zid, instance_id=expected_coordinator_instance_id, label=f"{side} command")
        if reason:
            return FaultReturnDecision(False, reason)
        if _field(command, "mode") != "returning":
            return FaultReturnDecision(False, "fault command must be returning")
    return FaultReturnDecision(True, "fault_return")


__all__ = ["ReadinessDecision", "FaultReturnDecision", "evaluate_connection", "evaluate_start", "evaluate_fault_return"]
