"""受 launcher 授权的 Marvin real-capability provider。"""
from __future__ import annotations

import os

from ...sources.common.real_admission import RealCapabilityInput


def trusted_real_capability() -> RealCapabilityInput:
    """只接受 operator/run_case 注入的 typed preflight 字段，缺失即不准入。"""
    return RealCapabilityInput(
        speed=float(os.environ.get("TIANJI_REAL_SPEED", "1")),
        yaw_deg=float(os.environ.get("TIANJI_REAL_YAW_DEG", "nan")),
        deadman_available=os.environ.get("TIANJI_REAL_DEADMAN_AVAILABLE") == "1",
        preflight_passed=os.environ.get("TIANJI_REAL_PREFLIGHT_PASSED") == "1",
    )


__all__ = ["trusted_real_capability"]
