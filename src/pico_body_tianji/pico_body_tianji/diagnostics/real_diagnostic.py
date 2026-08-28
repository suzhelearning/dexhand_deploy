"""Marvin/MuJoCo 只读 real readiness 诊断。

诊断器只订阅权威 SessionState 和 executor/status，不发布任何 target、joint
state 或 final command，也不能连接/驱动物理设备。
"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any

from ..protocol import topics
from ..protocol.messages import SessionState, strict_loads
from ..zenoh_util import open_session, require_single_router


class RealDiagnosticCollector:
    """把权威状态和安全相关 status 保存为本地 JSONL。"""

    def __init__(self, output: str | Path, *, router_zid: str) -> None:
        self.output = Path(output)
        self.router_zid = router_zid
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.output.open("a", encoding="utf-8")
        self._closed = False

    def receive(self, topic: str, payload: Any) -> None:
        if self._closed:
            raise RuntimeError("diagnostic collector is closed")
        value = strict_loads(bytes(payload)) if isinstance(payload, (bytes, bytearray)) else payload
        if topic == topics.SESSION_STATE:
            message = SessionState.from_dict(value)
            if message.router_zid != self.router_zid:
                return
        self._file.write(json.dumps({"time_ns": time.monotonic_ns(), "topic": topic, "payload": value}, ensure_ascii=False) + "\n")
        self._file.flush()

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self._file.close()


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="read-only Marvin readiness diagnostic")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    instance = os.environ.get("TIANJI_COMPONENT_INSTANCE_ID", "diagnostic")
    del instance  # diagnostics never claims a component authority token
    session = open_session()
    collector = None
    resources: list[Any] = []
    try:
        router = require_single_router(session, os.environ.get("TIANJI_ROUTER_ZID"))
        collector = RealDiagnosticCollector(args.output, router_zid=router)
        for topic in (topics.SESSION_STATE, topics.EXECUTOR_STATUS, topics.COORDINATOR_STATUS):
            resources.append(session.declare_subscriber(topic, lambda sample, topic=topic: collector.receive(topic, bytes(sample.payload))))
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        return 0
    finally:
        for resource in resources:
            try:
                resource.undeclare()
            except Exception:
                pass
        if collector is not None:
            collector.close()
        session.close()


__all__ = ["RealDiagnosticCollector", "main"]
