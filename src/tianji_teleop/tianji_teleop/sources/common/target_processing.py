"""Optional processing of mapped TCP targets before IK."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Protocol

import numpy as np

from .pose_mapping import MappedArmPose
from .target_conditioner import TargetConditioner, TargetConditioningDiagnostics, TargetConditioningSettings


@dataclass(frozen=True)
class ProcessedArmTarget:
    side: str
    pose: np.ndarray | None
    valid: bool
    backend: str
    mapping_backend: str
    frame_association_id: str
    diagnostics: TargetConditioningDiagnostics | None = None

    def __post_init__(self) -> None:
        if self.side not in ("left", "right"):
            raise ValueError("side must be left or right")
        if self.valid and self.pose is None:
            raise ValueError("valid processed target requires pose")
        if self.pose is not None:
            value = np.asarray(self.pose, dtype=np.float64)
            if value.shape != (7,) or not np.isfinite(value).all() or np.linalg.norm(value[3:]) < 1.0e-12:
                raise ValueError("processed target pose must be finite and valid")
            object.__setattr__(self, "pose", value.copy())


class ArmTargetProcessor(Protocol):
    def reset(self, initial_target: Any = None) -> None: ...
    def process(self, mapped_target: MappedArmPose, *, dt_s: float) -> ProcessedArmTarget: ...


class PassthroughTargetProcessor:
    def reset(self, initial_target: Any = None) -> None:
        return None

    def process(self, mapped_target: MappedArmPose, *, dt_s: float) -> ProcessedArmTarget:
        if not isinstance(mapped_target, MappedArmPose):
            raise TypeError("mapped_target must be MappedArmPose")
        if not np.isfinite(float(dt_s)) or float(dt_s) <= 0.0:
            raise ValueError("dt_s must be a positive finite number")
        pose = None if mapped_target.pose is None else mapped_target.pose.copy()
        return ProcessedArmTarget(mapped_target.side, pose, mapped_target.valid, "passthrough", mapped_target.backend, mapped_target.frame_association_id)


def _config_value(config: Any, name: str, default: Any = None) -> Any:
    if isinstance(config, Mapping):
        return config.get(name, default)
    return getattr(config, name, default)


class ConditionedTargetProcessor:
    def __init__(self, config: Any):
        settings = TargetConditioningSettings(
            rate_hz=float(_config_value(config, "rate_hz", 90.0)),
            translation_gain=np.asarray(_config_value(config, "translation_gain", [1.0, 1.0, 1.0]), dtype=np.float64),
            rotation_gain=float(_config_value(config, "rotation_gain", 1.0)),
            workspace_relative_radii_m=np.asarray(_config_value(config, "workspace_relative_radii_m", [10.0, 10.0, 10.0]), dtype=np.float64),
            workspace_soft_zone_ratio=float(_config_value(config, "workspace_soft_zone_ratio", 0.99)),
            maximum_linear_speed_m_s=float(_config_value(config, "maximum_linear_speed_m_s", 100.0)),
            maximum_angular_speed_rad_s=float(_config_value(config, "maximum_angular_speed_rad_s", 100.0)),
            maximum_linear_acceleration_m_s2=float(_config_value(config, "maximum_linear_acceleration_m_s2", 10000.0)),
            maximum_angular_acceleration_rad_s2=float(_config_value(config, "maximum_angular_acceleration_rad_s2", 10000.0)),
        )
        self._conditioners = {
            side: TargetConditioner(
                _config_value(config, "initial_position", {}).get(side, [0.0, 0.0, 0.0]),
                _config_value(config, "initial_quaternion", {}).get(side, [0.0, 0.0, 0.0, 1.0]),
                settings,
            )
            for side in ("left", "right")
        }

    def reset(self, initial_target: Any = None) -> None:
        for conditioner in self._conditioners.values():
            conditioner.reset()

    def process(self, mapped_target: MappedArmPose, *, dt_s: float) -> ProcessedArmTarget:
        if not isinstance(mapped_target, MappedArmPose):
            raise TypeError("mapped_target must be MappedArmPose")
        if not np.isfinite(float(dt_s)) or float(dt_s) <= 0.0:
            raise ValueError("dt_s must be a positive finite number")
        if not mapped_target.valid or mapped_target.pose is None:
            return ProcessedArmTarget(mapped_target.side, None, False, "conditioned", mapped_target.backend, mapped_target.frame_association_id)
        position, quaternion, diagnostics = self._conditioners[mapped_target.side].condition(mapped_target.pose[:3], mapped_target.pose[3:])
        return ProcessedArmTarget(mapped_target.side, np.concatenate((position, quaternion)), True, "conditioned", mapped_target.backend, mapped_target.frame_association_id, diagnostics)


def create_arm_target_processor(backend: str, config: Any) -> ArmTargetProcessor:
    if backend == "passthrough":
        return PassthroughTargetProcessor()
    if backend == "conditioned":
        return ConditionedTargetProcessor(config)
    raise ValueError("unknown arm target processor backend: " + str(backend))


__all__ = ["ArmTargetProcessor", "ConditionedTargetProcessor", "PassthroughTargetProcessor", "ProcessedArmTarget", "create_arm_target_processor"]
