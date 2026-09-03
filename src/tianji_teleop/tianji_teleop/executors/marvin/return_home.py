from __future__ import annotations

import argparse
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from ...coordination.arm_command_coordinator import ArmRobotConfig
from ...marvin_hardware import MarvinFeedback, MarvinHardwareError, MarvinHardwareSession
from .sdk_session import create_official_marvin_session

_RECOVERY_RATIO = 5
_RECOVERY_MAX_SPEED_DEG_S = 10.0
_RECOVERY_MAX_TRACKING_ERROR_DEG = 3.0


def _selected_mask(side: str) -> np.ndarray:
    if side not in {"left", "right", "both"}:
        raise ValueError("side must be left, right, or both")
    mask = np.zeros(14, dtype=bool)
    if side in {"left", "both"}:
        mask[:7] = True
    if side in {"right", "both"}:
        mask[7:] = True
    return mask


def run_return_home(
    devices: Mapping[str, Any],
    settings: Mapping[str, Any],
    side: str,
    *,
    recover_outside_limits: bool = False,
    hardware: MarvinHardwareSession | None = None,
) -> MarvinFeedback:
    selected = _selected_mask(side)
    robot = ArmRobotConfig.load()
    strict_lower = np.asarray(settings["feedback_hard_lower_limits_deg"], dtype=np.float64)
    strict_upper = np.asarray(settings["feedback_hard_upper_limits_deg"], dtype=np.float64)
    strict_padding = float(settings["feedback_hard_limit_padding_deg"])
    strict_bounds = MarvinHardwareSession._hard_limit_bounds(
        strict_lower, strict_upper, strict_padding
    )
    assert strict_bounds is not None

    home_deg = np.degrees(np.asarray(robot.home_all, dtype=np.float64))
    if np.any(home_deg < strict_bounds[0]) or np.any(home_deg > strict_bounds[1]):
        raise MarvinHardwareError("configured Home exceeds the normal Marvin safety limits")

    if recover_outside_limits:
        motion_lower = np.tile(np.degrees(robot.lower_limits_rad), 2)
        motion_upper = np.tile(np.degrees(robot.upper_limits_rad), 2)
        motion_padding = 0.0
        velocity_ratio = _RECOVERY_RATIO
        acceleration_ratio = _RECOVERY_RATIO
        maximum_speed = _RECOVERY_MAX_SPEED_DEG_S
        maximum_tracking_error = min(
            float(settings["maximum_tracking_error_deg"]),
            _RECOVERY_MAX_TRACKING_ERROR_DEG,
        )
    else:
        motion_lower = strict_lower
        motion_upper = strict_upper
        motion_padding = strict_padding
        velocity_ratio = int(settings["velocity_ratio"])
        acceleration_ratio = int(settings["acceleration_ratio"])
        maximum_speed = float(settings["return_max_speed_deg_s"])
        maximum_tracking_error = float(settings["maximum_tracking_error_deg"])

    session = hardware or create_official_marvin_session()
    try:
        feedback = session.connect_and_prepare(
            devices["marvin"]["ip"],
            velocity_ratio=velocity_ratio,
            acceleration_ratio=acceleration_ratio,
            lower_limits_deg=motion_lower,
            upper_limits_deg=motion_upper,
            hard_limit_padding_deg=motion_padding,
        )
        current_deg = np.concatenate(
            (feedback.left_joints_deg, feedback.right_joints_deg)
        )
        outside = (current_deg < strict_bounds[0]) | (
            current_deg > strict_bounds[1]
        )
        recovering_outside = recover_outside_limits and np.any(outside)
        if recover_outside_limits:
            if np.any(outside & ~selected):
                raise MarvinHardwareError(
                    "an unselected arm is outside the normal safety limits; select both arms"
                )
            if not recovering_outside:
                motion_lower = strict_lower
                motion_upper = strict_upper
                motion_padding = strict_padding

        target_deg = current_deg.copy()
        target_deg[selected] = home_deg[selected]
        minimum_duration = float(settings["return_minimum_duration_s"])
        label = {"left": "左臂", "right": "右臂", "both": "双臂"}[side]
        mode = "受限恢复" if recovering_outside else "平滑回零"
        print(
            f"Marvin 已连接；{label}距目标最大关节误差 "
            f"{np.max(np.abs(current_deg - target_deg)):.2f} deg，开始{mode}。",
            flush=True,
        )
        feedback = session.move_to_home(
            target_deg[:7],
            target_deg[7:],
            rate_hz=float(settings["rate_hz"]),
            minimum_duration_s=minimum_duration,
            max_speed_deg_s=maximum_speed,
            maximum_tracking_error_deg=maximum_tracking_error,
            home_tolerance_deg=float(settings["home_tolerance_deg"]),
            lower_limits_deg=motion_lower,
            upper_limits_deg=motion_upper,
            hard_limit_padding_deg=motion_padding,
            feedback_timeout_s=float(settings["feedback_timeout_s"]),
            require_monotonic_home_progress=recover_outside_limits,
        )
        final_deg = np.concatenate(
            (feedback.left_joints_deg, feedback.right_joints_deg)
        )
        if recover_outside_limits:
            MarvinHardwareSession._require_feedback_within_hard_limits(
                feedback, strict_bounds
            )
        print(
            f"完成：{label}已平滑回到目标；最大关节误差 "
            f"{np.max(np.abs(final_deg - target_deg)):.2f} deg。",
            flush=True,
        )
        return feedback
    finally:
        session.shutdown()


def _load_yaml(path: str) -> Mapping[str, Any]:
    with Path(path).open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, Mapping):
        raise ValueError(f"config must contain a mapping: {path}")
    return value


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device-config", required=True)
    parser.add_argument("--marvin-config", required=True)
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--recover-outside-limits", action="store_true")
    args = parser.parse_args(argv)
    run_return_home(
        _load_yaml(args.device_config),
        _load_yaml(args.marvin_config),
        args.side,
        recover_outside_limits=args.recover_outside_limits,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
