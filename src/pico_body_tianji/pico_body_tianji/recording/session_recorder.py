"""CLI for the passive session-v1 recorder."""
from __future__ import annotations

import os
import time
from pathlib import Path

from ..zenoh_util import open_session, require_single_router
from .recorder import SessionRecorderNode


def main() -> int:
    output = os.environ.get("TIANJI_RECORD_PATH", "")
    source_type = os.environ.get("TIANJI_RECORD_SOURCE_TYPE", "")
    instance = os.environ.get("TIANJI_COMPONENT_INSTANCE_ID", "")
    if not output or not source_type or not instance:
        raise RuntimeError(
            "TIANJI_RECORD_PATH, TIANJI_RECORD_SOURCE_TYPE and "
            "TIANJI_COMPONENT_INSTANCE_ID are required"
        )
    expected_router = os.environ.get("TIANJI_ROUTER_ZID", "")
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
        )
        while True:
            time.sleep(1.0)
            node.flush()
    except KeyboardInterrupt:
        return 0
    finally:
        if node is not None:
            node.close()
        session.close()


if __name__ == "__main__":
    raise SystemExit(main())
