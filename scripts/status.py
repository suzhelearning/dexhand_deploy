#!/usr/bin/env python3
"""订阅 /pico_body_real/status（裸文本）并逐条打印，保持原输出格式。"""

from __future__ import annotations

import logging
import threading

import zenoh

from pico_body_tianji.zenoh_util import ZenohTextSub, open_session

_LOG = logging.getLogger("pico_body_real_status_monitor")

STATUS_KEY = "pico_body_real/status"


class StatusMonitor:
    def __init__(self, session: zenoh.Session):
        self._session = session
        self._sub = ZenohTextSub(
            session,
            STATUS_KEY,
            self._print_status,
        )
        _LOG.info("等待状态消息：%s", STATUS_KEY)

    @staticmethod
    def _print_status(message: str) -> None:
        print(message, flush=True)

    def run(self) -> None:
        # 订阅回调在 Zenoh 内部线程驱动；主线程阻塞等待直到被中断。
        self._done = threading.Event()
        try:
            self._done.wait()
        except KeyboardInterrupt:
            pass

    def close(self) -> None:
        try:
            self._sub.close()
        finally:
            self._session.close()


def main() -> None:
    session = open_session()
    monitor = StatusMonitor(session)
    try:
        monitor.run()
    finally:
        monitor.close()


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    main()
