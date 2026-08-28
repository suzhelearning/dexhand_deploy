"""Marvin feedback 到 canonical radian state 的无 SDK 适配。"""
from __future__ import annotations

from typing import Any
import numpy as np

from ...marvin_hardware import MarvinFeedback
from ...protocol.messages import ALL_ARM_JOINT_NAMES, ArmJointState


def feedback_to_joint_state(
    feedback: MarvinFeedback,
    *,
    sequence: int,
    timestamp_ns: int,
    publisher_instance_id: str,
    router_zid: str,
    executor: str = "marvin",
) -> ArmJointState:
    """Convert SDK degrees to the sole wire unit (radians)."""
    values = np.concatenate([feedback.left_joints_deg, feedback.right_joints_deg])
    if values.shape != (14,) or not np.isfinite(values).all():
        raise ValueError("Marvin feedback must contain fourteen finite joints")
    return ArmJointState(
        1, int(sequence), int(timestamp_ns), executor,
        list(ALL_ARM_JOINT_NAMES), np.radians(values).tolist(), None,
        publisher_instance_id, router_zid,
    )


def feedback_safety_reason(feedback: MarvinFeedback) -> str | None:
    """Classify unsafe feedback separately from coordinator fault/timeout."""
    if feedback.error_codes != (0, 0):
        return f"arm_error:{feedback.error_codes}"
    if feedback.servo_error_reports != ("None", "None"):
        return f"servo_error:{feedback.servo_error_reports}"
    if feedback.arm_states != (1, 1):
        return f"arm_state:{feedback.arm_states}"
    if any(not np.isfinite(values).all() for values in (feedback.left_joints_deg, feedback.right_joints_deg)):
        return "nonfinite_feedback"
    return None


__all__ = ["feedback_to_joint_state", "feedback_safety_reason"]
