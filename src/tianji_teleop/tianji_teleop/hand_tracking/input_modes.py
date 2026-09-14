"""Pure input-combination contracts; resolving is NOT runtime authorization.

No device imports, process creation, or session defaults belong here. Backend
availability, calibration and real-device preflight remain separate checks.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping


SPARK_BACKEND = "spark_upper_qpoases_headroom_feedforward_velocity_qp"
MAPPED_PALM_BACKEND = "pico_ee_mapped_corrected_palm_velocity_qp"
_FIELDS = frozenset({"input_mode", "hand_input", "arm_input", "operator_input"})
_POSE_BACKENDS = frozenset({"pinocchio_cpp", "pinocchio_qp", "tianji_official", "pico_ee_dexhand_qp"})


@dataclass(frozen=True)
class InputMode:
    input_mode: str
    hand_input: str
    arm_input: str
    operator_input: str
    receivers: tuple[str, ...]
    requires_upper_limb_skeleton: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "input_mode": self.input_mode,
            "hand_input": self.hand_input,
            "arm_input": self.arm_input,
            "operator_input": self.operator_input,
            "receivers": list(self.receivers),
            "requires_upper_limb_skeleton": self.requires_upper_limb_skeleton,
        }


def resolve_input_mode(config: Mapping[str, Any]) -> InputMode:
    """Validate exactly one explicit combination without inferring defaults."""
    if not isinstance(config, Mapping):
        raise ValueError("input configuration must be a mapping")
    if set(config) - _FIELDS:
        raise ValueError("unknown input configuration fields")
    if _FIELDS - set(config):
        raise ValueError("missing input configuration fields")
    if any(not isinstance(config[key], str) or not config[key] for key in _FIELDS):
        raise ValueError("input configuration fields must be nonempty strings")
    mode, hand, arm, operator = (config[key] for key in
                                 ("input_mode", "hand_input", "arm_input", "operator_input"))
    if mode == "pico2_hands":
        if hand != "pico2":
            raise ValueError("pico2_hands requires hand_input=pico2")
        if arm != "pico2_head_wrist":
            raise ValueError("pico2_hands requires arm_input=pico2_head_wrist")
        if operator not in {"keyboard", "gesture"}:
            raise ValueError("pico2_hands operator_input must be keyboard or gesture")
        receivers = ("pico2",)
    elif mode == "vr_manus":
        if hand != "manus":
            raise ValueError("vr_manus requires hand_input=manus")
        if arm not in {"tjvr_corrected_palm", "xr_controller", "xr_tracker"}:
            raise ValueError("vr_manus has unsupported arm_input")
        if operator not in {"keyboard", "controller"}:
            raise ValueError("vr_manus operator_input must be keyboard or controller")
        receivers = ("manus", "tjvr" if arm == "tjvr_corrected_palm" else "xr")
    else:
        raise ValueError(f"unknown input_mode: {mode}")
    return InputMode(mode, hand, arm, operator, receivers, arm == "tjvr_corrected_palm")


def validate_ik_input(mode: InputMode, backend: str) -> None:
    """Check input shape only; does not claim a backend is built/real-ready."""
    if not isinstance(backend, str):
        raise ValueError("backend must be a string")
    if backend in (SPARK_BACKEND, MAPPED_PALM_BACKEND):
        if not mode.requires_upper_limb_skeleton:
            raise ValueError("bilateral reference backend requires corrected upper-limb skeleton input")
    elif backend not in _POSE_BACKENDS:
        raise ValueError(f"unknown IK backend: {backend}")
