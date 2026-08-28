"""Fail-closed, typed admission inputs for physical teleoperation."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


@dataclass(frozen=True)
class RealCapabilityInput:
    """One sampled real-admission result.

    The source may expose ``real`` only when every field is explicitly typed
    and all predicates are true.  In particular, strings such as ``"false"``
    are rejected rather than being coerced with Python truthiness.
    """

    speed: float
    yaw_deg: float
    deadman_available: bool
    preflight_passed: bool

    def __post_init__(self) -> None:
        if isinstance(self.speed, bool) or not isinstance(self.speed, (int, float)):
            raise ValueError("speed must be a finite number")
        if not math.isfinite(float(self.speed)) or float(self.speed) <= 0.0:
            raise ValueError("speed must be a finite positive number")
        if isinstance(self.yaw_deg, bool) or not isinstance(self.yaw_deg, (int, float)):
            raise ValueError("yaw_deg must be a finite number")
        if not math.isfinite(float(self.yaw_deg)):
            raise ValueError("yaw_deg must be finite")
        for name in ("deadman_available", "preflight_passed"):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f"{name} must be a boolean")

    @property
    def admitted(self) -> bool:
        return (
            float(self.speed) <= 0.25
            and float(self.yaw_deg) == 0.0
            and self.deadman_available
            and self.preflight_passed
        )

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "RealCapabilityInput":
        if not isinstance(value, Mapping):
            raise ValueError("real capability must be an object")
        expected = {"speed", "yaw_deg", "deadman_available", "preflight_passed"}
        missing = expected - set(value)
        extra = set(value) - expected
        if missing:
            raise ValueError(f"real capability missing fields: {', '.join(sorted(missing))}")
        if extra:
            raise ValueError(f"real capability unknown fields: {', '.join(sorted(extra))}")
        return cls(
            speed=value["speed"],
            yaw_deg=value["yaw_deg"],
            deadman_available=value["deadman_available"],
            preflight_passed=value["preflight_passed"],
        )


def parse_real_capability(value: Any) -> RealCapabilityInput:
    """Parse a capability or provider result without coercion."""
    if callable(value):
        value = value()
    if isinstance(value, RealCapabilityInput):
        return value
    if isinstance(value, Mapping):
        return RealCapabilityInput.from_mapping(value)
    raise ValueError("real capability must be RealCapabilityInput or mapping")


__all__ = ["RealCapabilityInput", "parse_real_capability"]
