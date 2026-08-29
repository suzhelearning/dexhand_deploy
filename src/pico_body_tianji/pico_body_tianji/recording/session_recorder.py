"""CLI for the passive session-v1 recorder."""
from __future__ import annotations
import os
import signal
import threading
import time
from pathlib import Path

from ..config_loader import load_component_config, require_finite_positive
from ..zenoh_util import open_session, require_single_router
from .recorder import SessionRecorderNode

_RECORDING_CONFIG_KEYS = {"flush_interval_s", "schema_name", "schema_version"}


def _load_recording_config(path: str) -> dict[str, object]:
    config = load_component_config(
        path,
        allowed_keys=_RECORDING_CONFIG_KEYS,
        required_keys=_RECORDING_CONFIG_KEYS,
    )
    if config["schema_name"] != "tianji-teleop-session" or config["schema_version"] != "1.0":
        raise ValueError("unsupported session recording schema")
    config["flush_interval_s"] = require_finite_positive(
        config["flush_interval_s"], "recording.flush_interval_s"
    )
    return config


def main() -> int:
    output = os.environ.get("TIANJI_RECORD_PATH", "")
    source_type = os.environ.get("TIANJI_RECORD_SOURCE_TYPE", "")
    instance = os.environ.get("TIANJI_COMPONENT_INSTANCE_ID", "")
    config_path = os.environ.get("TIANJI_RECORDING_CONFIG", "")
    if not output or not source_type or not instance or not config_path:
        raise RuntimeError(
            "TIANJI_RECORD_PATH, TIANJI_RECORD_SOURCE_TYPE, "
            "TIANJI_COMPONENT_INSTANCE_ID and TIANJI_RECORDING_CONFIG are required"
        )
    recording_config = _load_recording_config(config_path)
    expected_router = os.environ.get("TIANJI_ROUTER_ZID", "")
    stop_event = threading.Event()

    def request_shutdown(_signum: int, _frame: object) -> None:
        stop_event.set()

    old_handlers = {
        signum: signal.getsignal(signum)
        for signum in (signal.SIGTERM, signal.SIGINT)
    }
    for signum in old_handlers:
        signal.signal(signum, request_shutdown)
    session = open_session()
    node = None
    try:
        router = require_single_router(session, expected_router or None)
        node = SessionRecorderNode(
            session,
            Path(output),
            source_type=source_type,
            robot_model=os.environ.get("TIANJI_ROBOT_MODEL", "marvin"),
            router_zid=router,
            publisher_instance_id=instance,
            recording_config=recording_config,
        )
        while not stop_event.wait(1.0):
            node.flush()
    finally:
        if node is not None:
            node.close()
        session.close()
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
