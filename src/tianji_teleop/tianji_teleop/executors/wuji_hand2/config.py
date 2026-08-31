"""Wuji Hand 2 的唯一关节配置与 wire/SDK 边界校验。"""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

from ...protocol.messages import HAND_JOINT_NAMES, SIDES


@dataclass(frozen=True)
class WujiHandConfig:
    """20 关节的 canonical wire 顺序、零位和 rad 限位。"""

    joint_names: tuple[str, ...]
    lower_limits_rad: tuple[float, ...]
    upper_limits_rad: tuple[float, ...]
    zero_position_rad: tuple[float, ...]
    zero_tolerance_rad: tuple[float, ...]

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "WujiHandConfig":
        required = {
            "joint_names", "lower_limits_rad", "upper_limits_rad",
            "zero_position_rad", "zero_tolerance_rad",
        }
        if not isinstance(value, Mapping):
            raise ValueError("Wuji hand config must be an object")
        missing = required - set(value)
        extra = set(value) - required
        if missing or extra:
            raise ValueError(
                f"Wuji hand config fields mismatch; missing={sorted(missing)}, extra={sorted(extra)}"
            )
        names = tuple(value["joint_names"]) if isinstance(value["joint_names"], (list, tuple)) else ()
        expected = HAND_JOINT_NAMES["right"]
        if names != expected or len(set(names)) != 20:
            raise ValueError("joint_names must exactly match canonical 20-joint order")

        def vector(field: str) -> tuple[float, ...]:
            raw = value[field]
            if not isinstance(raw, (list, tuple)) or len(raw) != 20:
                raise ValueError(f"{field} must contain 20 values")
            result = tuple(float(item) for item in raw)
            if not all(math.isfinite(item) for item in result):
                raise ValueError(f"{field} must contain finite values")
            return result

        lower = vector("lower_limits_rad")
        upper = vector("upper_limits_rad")
        zero = vector("zero_position_rad")
        tolerance = vector("zero_tolerance_rad")
        if any(lo >= hi for lo, hi in zip(lower, upper)):
            raise ValueError("lower_limits_rad must be below upper_limits_rad")
        if any(tol <= 0.0 for tol in tolerance):
            raise ValueError("zero_tolerance_rad must be positive")
        if any(not lo <= value <= hi for value, lo, hi in zip(zero, lower, upper)):
            raise ValueError("zero_position_rad must be inside joint limits")
        return cls(names, lower, upper, zero, tolerance)

    @classmethod
    def load(cls, path: str | Path | None = None) -> "WujiHandConfig":
        if path is None:
            here = Path(__file__).resolve()
            for parent in here.parents:
                candidate = parent / "config" / "robot" / "wuji_hand2.yaml"
                if candidate.is_file():
                    path = candidate
                    break
            else:
                raise ValueError("unable to locate config/robot/wuji_hand2.yaml")
        try:
            import yaml
            value = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        except Exception as exc:
            raise ValueError(f"unable to load Wuji hand config {path}: {exc}") from exc
        return cls.from_mapping(value)

    def validate_positions(self, values: Sequence[float], *, field: str = "position_rad") -> list[float]:
        if isinstance(values, (str, bytes, Mapping)):
            raise ValueError(f"{field} must contain 20 values")
        try:
            result = [float(value) for value in values]
        except TypeError as exc:
            raise ValueError(f"{field} must contain 20 values") from exc
        if len(result) != 20:
            raise ValueError(f"{field} must contain 20 values")
        if not all(math.isfinite(value) for value in result):
            raise ValueError(f"{field} must contain finite values")
        if any(value < lo or value > hi for value, lo, hi in zip(result, self.lower_limits_rad, self.upper_limits_rad)):
            raise ValueError(f"{field} exceeds Wuji hard limits")
        return result

    def at_zero(self, values: Sequence[float]) -> bool:
        result = self.validate_positions(values)
        return all(abs(value - zero) <= tolerance for value, zero, tolerance in zip(result, self.zero_position_rad, self.zero_tolerance_rad))

    def sdk_joint_name(self, index: int, *, side: str) -> str:
        """将 canonical wire 名称映射到 SDK/URDF 名称；别名只留在适配器内。"""
        if side not in SIDES:
            raise ValueError("side must be left or right")
        if not isinstance(index, int) or not 0 <= index < 20:
            raise IndexError(index)
        prefix = "l" if side == "left" else "r"
        name = self.joint_names[index]
        if name.startswith("r_"):
            return prefix + name[1:]
        return prefix + name[1:]

    def sdk_joint_names(self, *, side: str) -> tuple[str, ...]:
        return tuple(self.sdk_joint_name(index, side=side) for index in range(20))


__all__ = ["WujiHandConfig"]
