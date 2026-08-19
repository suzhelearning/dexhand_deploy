#!/usr/bin/env python3
"""termios 原始模式键盘监听（模式同 /home/current/syz/mocap/acquisition）。

用法（后台线程）:

    stop = threading.Event()
    thread = threading.Thread(
        target=raw_keyboard, args=(on_key, stop), daemon=True
    )
    thread.start()
    ...
    stop.set()          # 退出前置位，线程恢复终端后结束

行为：

- tty：进入 raw 模式，每个按键回调一次 on_key(chr)，无按键时以
  5ms 间隔 select 轮询；退出时恢复终端；
- 非 tty（管道/重定向，如经 ros2 launch + FIFO 的 stdin）：退化为
  select 轮询；FIFO 一次性写入者关闭后读端会看到 EOF——此时**不
  退出**，继续等待下一个写入者（mocap 回放经 run_mocap_sim.sh
  FIFO 转发键盘时走此路径，脚本侧另有常开写端避免 EOF）；
- 无按键空隙通过 on_idle 回调（默认 None）。
"""

from __future__ import annotations

import os
import select
import sys
import termios
import threading
import time
import tty
from collections.abc import Callable

KeyCallback = Callable[[str], None]

POLL_INTERVAL = 0.005


def _drain(fd: int, on_key: KeyCallback,
           on_idle: Callable[[], None] | None) -> None:
    """读一次可用数据并回调；返回是否应继续（False=致命错误）。"""
    readable, _, _ = select.select([fd], [], [], POLL_INTERVAL)
    if not readable:
        if on_idle is not None:
            on_idle()
        return True
    try:
        data = os.read(fd, 1024)
    except OSError:
        return False
    if not data:
        # FIFO 暂无写入者（EOF）：稍候再轮询，避免空转。
        if on_idle is not None:
            on_idle()
        time.sleep(0.02)
        return True
    for byte in data:
        on_key(chr(byte))
    return True


def raw_keyboard(
    on_key: KeyCallback,
    stop_event: threading.Event,
    fd: int | None = None,
    on_idle: Callable[[], None] | None = None,
) -> None:
    """阻塞式读取键盘，直到 stop_event 被置位。进入前保存、退出时恢复终端。

    fd 默认 sys.stdin；测试可注入管道读端。
    """
    fd = fd if fd is not None else sys.stdin.fileno()

    if os.isatty(fd):
        saved = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while not stop_event.is_set():
                if not _drain(fd, on_key, on_idle):
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        return

    # 非终端：无法设置 raw 模式，退化为 select 轮询；EOF 不退出。
    while not stop_event.is_set():
        if not _drain(fd, on_key, on_idle):
            break
