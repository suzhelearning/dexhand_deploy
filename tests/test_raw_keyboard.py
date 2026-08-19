from __future__ import annotations

import os
import threading
import time
import unittest

from pico_body_tianji.controller_only.raw_keyboard import raw_keyboard


class RawKeyboardPipeTest(unittest.TestCase):
    def test_pipe_path_delivers_keys(self) -> None:
        read_fd, write_fd = os.pipe()
        received: list[str] = []
        stop = threading.Event()

        def on_key(key: str) -> None:
            received.append(key)

        thread = threading.Thread(
            target=raw_keyboard, args=(on_key, stop, read_fd),
            daemon=True,
        )
        thread.start()
        os.write(write_fd, b"s")
        os.write(write_fd, b"q")  # 非 's' 也应回调（由调用方过滤）
        os.write(write_fd, b"\n")

        # 等待回调送达
        for _ in range(100):
            if len(received) >= 3:
                break
            threading.Event().wait(0.01)
        stop.set()
        thread.join(timeout=1.0)
        os.close(read_fd)
        os.close(write_fd)

        self.assertEqual(received, ["s", "q", "\n"])

    def test_eof_does_not_end_listener(self) -> None:
        """FIFO 一次性写入者关闭产生 EOF，监听应继续等下一个写入者。"""
        read_fd, write_fd = os.pipe()
        stop = threading.Event()
        received: list[str] = []

        thread = threading.Thread(
            target=raw_keyboard,
            args=(received.append, stop, read_fd),
            daemon=True,
        )
        thread.start()
        os.close(write_fd)  # EOF（模拟一次性写入者关闭）
        time.sleep(0.1)
        self.assertTrue(thread.is_alive(), "EOF 不应结束监听线程")
        stop.set()
        thread.join(timeout=1.0)
        os.close(read_fd)
        self.assertFalse(thread.is_alive())

    def test_stop_event_ends_listener(self) -> None:
        read_fd, write_fd = os.pipe()
        stop = threading.Event()
        thread = threading.Thread(
            target=raw_keyboard, args=(lambda key: None, stop, read_fd),
            daemon=True,
        )
        thread.start()
        stop.set()
        thread.join(timeout=1.0)
        os.close(read_fd)
        os.close(write_fd)
        self.assertFalse(thread.is_alive())


if __name__ == "__main__":
    unittest.main()
