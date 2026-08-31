"""Marvin SDK boundary; this module intentionally never imports IK libraries."""
from __future__ import annotations

from ...marvin_hardware import MarvinHardwareError, MarvinHardwareSession


def create_official_marvin_session() -> MarvinHardwareSession:
    """延迟加载已验证的 Marvin SDK，并拒绝 libKine 混入。"""
    from marvin_sdk.fx_robot import DCSS, Marvin_Robot

    session = MarvinHardwareSession(robot=Marvin_Robot(), dcss_factory=DCSS)
    try:
        with open("/proc/self/maps", encoding="utf-8") as maps_file:
            maps = maps_file.read()
    except OSError:
        maps = ""
    if "libKine" in maps:
        raise MarvinHardwareError("libKine unexpectedly loaded in joint-control process")
    return session


__all__ = ["create_official_marvin_session"]
