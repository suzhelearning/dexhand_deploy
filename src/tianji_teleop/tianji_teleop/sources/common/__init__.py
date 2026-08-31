"""Shared source lifecycle, conditioning, and target publishing helpers."""

_EXPORTS = {
    "ArmTargetBatch": (".target_mapper", "ArmTargetBatch"),
    "WristPoseFrame": (".wrist_pose_frame", "WristPoseFrame"),
    "EndEffectorTargetMapper": (".target_mapper", "EndEffectorTargetMapper"),
    "FreshnessGate": (".freshness", "FreshnessGate"),
    "FreshnessStatus": (".freshness", "FreshnessStatus"),
    "HoldToRunClock": (".replay_clock", "HoldToRunClock"),
    "SessionClient": (".session_client", "SessionClient"),
    "TargetConditioner": (".target_conditioner", "TargetConditioner"),
    "TargetConditioningDiagnostics": (".target_conditioner", "TargetConditioningDiagnostics"),
    "TargetConditioningSettings": (".target_conditioner", "TargetConditioningSettings"),
    "TargetPublisher": (".target_publisher", "TargetPublisher"),
}


def __getattr__(name: str):
    """Load optional source helpers only when requested.

    Lazy exports keep the protocol/safety boundary usable in headless and
    fake-device environments without importing optional mapper dependencies.
    """
    try:
        module_name, attribute = _EXPORTS[name]
    except KeyError as exc:
        raise AttributeError(name) from exc
    from importlib import import_module
    value = getattr(import_module(module_name, __name__), attribute)
    globals()[name] = value
    return value


__all__ = list(_EXPORTS)
