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
import ctypes

import glob
import os
import select
import struct
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
            # tty.setraw 同时关闭 OPOST/ONLCR，导致所有共享该终端的
            # 日志只下移不回到行首，最终呈“阶梯状”。键盘只需要 raw
            # 输入；恢复原输出模式，保留正常的 CRLF 换行。
            raw_attributes = termios.tcgetattr(fd)
            raw_attributes[1] = saved[1]
            termios.tcsetattr(fd, termios.TCSANOW, raw_attributes)
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


class _EvdevKeyState:
    _EVENT = struct.Struct("@llHHi")
    _KEY_CODES = {"Return": 28, "KP_Enter": 96}

    def __init__(
        self,
        key_names: tuple[str, ...],
        *,
        event_paths: tuple[str, ...] | None = None,
    ) -> None:
        try:
            self._keycodes = {self._KEY_CODES[name] for name in key_names}
        except KeyError as exc:
            raise RuntimeError(f"Wayland 不支持按键 {exc.args[0]!r}") from exc
        paths = list(event_paths or sorted(glob.glob("/dev/input/by-id/*-event-kbd")))
        if not paths:
            paths = sorted(glob.glob("/dev/input/event*"))
        self._fds: list[int] = []
        self._buffers: dict[int, bytes] = {}
        for path in paths:
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | getattr(os, "O_CLOEXEC", 0))
            except OSError:
                continue
            self._fds.append(fd)
            self._buffers[fd] = b""
        if not self._fds:
            raise RuntimeError(
                "Wayland 无法读取物理键盘；请将当前用户加入 input 组并重新登录"
            )
        self._pressed: dict[tuple[int, int], bool] = {}

    def is_pressed(self) -> bool:
        for fd in self._fds:
            data = self._buffers[fd]
            while True:
                try:
                    chunk = os.read(fd, 4096)
                except BlockingIOError:
                    break
                except OSError as exc:
                    raise RuntimeError(f"Wayland 键盘读取失败：{exc}") from exc
                if not chunk:
                    break
                data += chunk
            complete = len(data) - len(data) % self._EVENT.size
            for offset in range(0, complete, self._EVENT.size):
                _, _, event_type, code, value = self._EVENT.unpack_from(data, offset)
                if event_type == 1 and code in self._keycodes:
                    self._pressed[(fd, code)] = value != 0
            self._buffers[fd] = data[complete:]
        return any(self._pressed.values())

    def close(self) -> None:
        for fd in self._fds:
            os.close(fd)
        self._fds.clear()


class X11KeyState:
    """读取物理按键状态，包含按下与松开。

    termios 只能收到字节，无法区分按住与松开；自动轨迹的 deadman
    因此 X11 使用键位图，Wayland 使用 evdev。查询失败时构造函数抛错，调用方应禁止
    自动运动，不能退化为按键自动重复超时。
    """

    def __init__(
        self,
        key_names: tuple[str, ...],
        *,
        display_name: str | None = None,
    ) -> None:
        if not key_names:
            raise ValueError("key_names 不能为空")
        self._evdev: _EvdevKeyState | None = None
        if display_name is None and os.environ.get("XDG_SESSION_TYPE") == "wayland":
            self._evdev = _EvdevKeyState(key_names)
            self._display = None
            return
        display_spec = (
            os.environ.get("DISPLAY")
            if display_name is None
            else display_name
        )
        if not display_spec:
            raise RuntimeError("未设置 DISPLAY，无法读取 Enter 松开状态")
        try:
            x11 = ctypes.CDLL("libX11.so.6")
        except OSError as exc:
            raise RuntimeError(f"无法加载 libX11：{exc}") from exc

        x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        x11.XOpenDisplay.restype = ctypes.c_void_p
        x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        x11.XCloseDisplay.restype = ctypes.c_int
        x11.XStringToKeysym.argtypes = [ctypes.c_char_p]
        x11.XStringToKeysym.restype = ctypes.c_ulong
        x11.XKeysymToKeycode.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
        ]
        x11.XKeysymToKeycode.restype = ctypes.c_ubyte
        x11.XQueryKeymap.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_char),
        ]
        x11.XQueryKeymap.restype = ctypes.c_int

        display = x11.XOpenDisplay(display_spec.encode("utf-8"))
        if not display:
            raise RuntimeError(
                f"无法连接 X11 display {display_spec!r}，"
                "不能可靠检测 Enter 松开"
            )
        keycodes = tuple(
            {
                int(x11.XKeysymToKeycode(
                    display,
                    x11.XStringToKeysym(name.encode("ascii")),
                ))
                for name in key_names
            }
            - {0}
        )
        if not keycodes:
            x11.XCloseDisplay(display)
            raise RuntimeError(f"X11 无法解析按键：{key_names!r}")

        self._x11 = x11
        self._display: int | None = display
        self._keycodes = keycodes
        self._keymap = ctypes.create_string_buffer(32)

    def is_pressed(self) -> bool:
        if self._evdev is not None:
            return self._evdev.is_pressed()
        if self._display is None:
            raise RuntimeError("X11 键位查询器已经关闭")
        self._x11.XQueryKeymap(self._display, self._keymap)
        keymap = self._keymap.raw
        return any(
            bool(keymap[keycode >> 3] & (1 << (keycode & 7)))
            for keycode in self._keycodes
        )

    def close(self) -> None:
        if self._evdev is not None:
            self._evdev.close()
            self._evdev = None
            return
        if self._display is None:
            return
        self._x11.XCloseDisplay(self._display)
        self._display = None

    def __enter__(self) -> "X11KeyState":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
