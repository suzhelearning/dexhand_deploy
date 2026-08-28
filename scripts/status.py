#!/usr/bin/env python3
"""打印 canonical ComponentStatus/SessionState，而不发布任何消息。"""
from __future__ import annotations

import json
import threading

from pico_body_tianji.protocol import topics
from pico_body_tianji.zenoh_util import open_session, require_single_router


class StatusMonitor:
    def __init__(self, session, router_zid: str):
        self._session = session
        self._router_zid = router_zid
        self._resources = []
        self._done = threading.Event()
        for topic in (
            topics.SESSION_STATE,
            topics.SOURCE_STATUS,
            topics.PRODUCER_STATUS,
            topics.COORDINATOR_STATUS,
            topics.EXECUTOR_STATUS,
        ):
            self._resources.append(session.declare_subscriber(topic, lambda sample, topic=topic: self._print(topic, sample)))

    def _print(self, topic: str, sample) -> None:
        try:
            value = json.loads(bytes(sample.payload).decode("utf-8"))
            if value.get("router_zid") != self._router_zid:
                return
            print(json.dumps({"topic": topic, "payload": value}, ensure_ascii=False), flush=True)
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    def run(self) -> None:
        self._done.wait()

    def close(self) -> None:
        for resource in self._resources:
            try:
                resource.undeclare()
            except Exception:
                pass
        self._session.close()


def main() -> int:
    session = open_session()
    try:
        router = require_single_router(session)
        monitor = StatusMonitor(session, router)
        try:
            monitor.run()
        finally:
            monitor.close()
    except KeyboardInterrupt:
        return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
