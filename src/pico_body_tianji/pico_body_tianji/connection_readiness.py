"""Marvin connection 与 teleop start 的独立 readiness gates。

连接门不依赖 policy observation；start 门只在执行器完成连接并报告 fresh
state 后判断。输入可以是 protocol ``ComponentStatus``/``ArmJointState`` 对象
或其 dict，未知/缺失字段一律拒绝。
"""
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


def _healthy(value: Any, capability: str) -> bool:
    caps = _field(value, "capabilities", ())
    return bool(_field(value, "ready", False) and _field(value, "healthy", False) and capability in caps)


def evaluate_connection(
    *,
    source_status: Any,
    arm_producer_status: Any,
    coordinator_state: Any,
    arm_command: Any,
    now_ns: int,
    freshness_timeout_s: float = 1.0,
    required_capability: str = "real",
) -> ReadinessDecision:
    """评估初次 Marvin connection；故意不读取 policy observation。"""
    if freshness_timeout_s <= 0.0:
        raise ValueError("freshness_timeout_s must be positive")
    cutoff = int(freshness_timeout_s * 1e9)
    for name, value in (("source", source_status), ("arm producer", arm_producer_status), ("coordinator", coordinator_state), ("arm command", arm_command)):
        timestamp = _field(value, "timestamp_ns")
        if not isinstance(timestamp, int) or isinstance(timestamp, bool) or timestamp < 0 or now_ns - timestamp > cutoff:
            return ReadinessDecision(False, f"{name} stale or missing")
    if not _healthy(source_status, required_capability):
        return ReadinessDecision(False, "source not real-capable and healthy")
    if not (_field(arm_producer_status, "ready", False) and _field(arm_producer_status, "healthy", False)):
        return ReadinessDecision(False, "arm producer not loaded and healthy")
    if _field(coordinator_state, "state") != "idle":
        return ReadinessDecision(False, "coordinator not idle")
    names = _field(arm_command, "names", ())
    positions = _field(arm_command, "position_rad", ())
    home = _field(arm_command, "home_position_rad", None)
    if _field(arm_command, "mode") != "idle" or not isinstance(names, (list, tuple)) or not isinstance(positions, (list, tuple)):
        return ReadinessDecision(False, "coordinator command is not idle")
    if home is not None and (len(home) != len(positions) or any(not math.isclose(float(x), float(y), abs_tol=1e-6) for x, y in zip(positions, home))):
        return ReadinessDecision(False, "coordinator command is not at Home")
    return ReadinessDecision(True, "ready")


def evaluate_start(*, executor_status: Any, arm_state: Any, coordinator_state: Any, now_ns: int, freshness_timeout_s: float = 1.0) -> ReadinessDecision:
    """执行器连接完成后的第二层 gate。"""
    cutoff = int(freshness_timeout_s * 1e9)
    for name, value in (("executor", executor_status), ("arm state", arm_state), ("coordinator", coordinator_state)):
        timestamp = _field(value, "timestamp_ns")
        if not isinstance(timestamp, int) or isinstance(timestamp, bool) or timestamp < 0 or now_ns - timestamp > cutoff:
            return ReadinessDecision(False, f"{name} stale or missing")
    if not (_field(executor_status, "ready", False) and _field(executor_status, "healthy", False)):
        return ReadinessDecision(False, "executor not ready and healthy")
    if _field(coordinator_state, "state") != "idle":
        return ReadinessDecision(False, "coordinator not idle")
    return ReadinessDecision(True, "ready")


@dataclass(frozen=True)
class FaultReturnDecision:
    allowed: bool
    reason: str


def evaluate_fault_return(*, coordinator_state: Any, arm_command: Any, now_ns: int, freshness_timeout_s: float = 1.0) -> FaultReturnDecision:
    """fault 中断时只允许消费 coordinator 的 bounded Home command。"""
    if _field(coordinator_state, "state") != "fault":
        return FaultReturnDecision(False, "coordinator is not fault")
    timestamp = _field(arm_command, "timestamp_ns")
    if not isinstance(timestamp, int) or now_ns - timestamp > int(freshness_timeout_s * 1e9):
        return FaultReturnDecision(False, "bounded Home command stale")
    if _field(arm_command, "mode") != "returning":
        return FaultReturnDecision(False, "fault command must be returning")
    return FaultReturnDecision(True, "fault_return")


__all__ = ["ReadinessDecision", "FaultReturnDecision", "evaluate_connection", "evaluate_start", "evaluate_fault_return"]
