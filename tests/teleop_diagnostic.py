#!/usr/bin/env python3
"""PICO -> Pinocchio -> Marvin 遥操作链路只读诊断器。

本文件只做被动读取：
  - 不发布任何控制消息；
  - 不加载或连接 Marvin SDK；
  - 不加载 PICO SDK，不调用 PXREAInit；
  - 不改变任何 ROS 参数；
  - 不会主动连接或驱动机械臂。
  - 只读取 ROS 话题、Linux /proc 和 ss/TCP_INFO 内核统计。

启动方法（所有命令都在项目根目录执行）：

  终端 1，先启动仿真：
    pixi run sim

  终端 2，启动本诊断器：
    bash -lc 'source scripts/common.sh; activate_bundle_runtime; exec .pixi/envs/default/bin/python tests/teleop_diagnostic.py'

  如果后续需要观察真机链路，保持前两个终端运行，再按项目原有流程在
  终端 3 执行 pixi run real。诊断器会自动发现真机桥，但仍然只订阅。

使用方法：
  1. 按 A 进入遥操作；
  2. 分别缓慢、快速做几次“抬手 -> 放手”；
  3. 发生“断流后必须重按 A”时，直接按键盘 p（不用按回车）；
  4. 发生“回落/返回明显变慢”时，直接按键盘 m（不用按回车）；
  5. 观察“当前判断”和“最近事件”；
  6. 按 Ctrl+C，查看整段测试的中文总结。

人工标记只写诊断日志，不会替代手柄 A，也不会发出任何控制消息。
后续需要增加热键时，只需在 HOTKEY_DEFINITIONS 中增加一项。

每次启动都会自动把终端中的实时状态、异常事件和最终总结保存到：
  tests/logs/teleop_diagnostic_年月日_时分秒.log

日志采用普通 UTF-8 文本格式，可以直接用编辑器打开。终端顶部会显示
本次日志的完整路径。

说明：脚本会对比三层证据：PICO 到 PC Service 的非回环 TCP
连接与收包字节率、PC Service 进程/60061 端口/本地 SDK 连接、
Python SDK 节点发布的
ROS 时间戳。网络连接存在不等于姿态帧有效；如果连接仍在但
SDK 时间戳停止，脚本会如实标为第 1/2 段边界。
"""

from __future__ import annotations

import atexit
import json
import math
import os
import re
import select
import socket
import subprocess
import sys
import termios
import time
import tty
from collections import Counter, deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


SIDES = ("left", "right")
SIDE_LABEL = {"left": "左臂", "right": "右臂"}
HEALTHY_SMPL_STATES = {"live", "live_signature_fallback"}
MARKER_LAYER_LOOKBACK_S = 15.0
COMMAND_PAIR_SKEW_LIMIT_MS = 30.0
COMMAND_TIMEOUT_MS = 150.0


@dataclass(frozen=True)
class HotkeyDefinition:
    """一个可扩展的人工诊断标记定义。"""

    key: str
    code: str
    title: str
    description: str


# 后续增加人工标记，只需在这里添加 HotkeyDefinition；其余记录、显示、
# 上下文冻结和总结代码会自动生效。按键统一按小写处理。
HOTKEY_DEFINITIONS = (
    HotkeyDefinition(
        key="p",
        code="P_RESTART_AFTER_DROPOUT",
        title="断流后必须重新按 A 才恢复",
        description="连接/跟踪突然退出，重新按手柄 A 后才能继续遥操作",
    ),
    HotkeyDefinition(
        key="m",
        code="M_SLOW_RETURN",
        title="回落或返回明显变慢",
        description="手臂向下或返回时速度异常缓慢，明显跟不上实际动作",
    ),
)
HOTKEY_BY_KEY = {definition.key: definition for definition in HOTKEY_DEFINITIONS}


@dataclass(frozen=True)
class UserMarker:
    """一次人工按键及其单调时间/墙上时间。"""

    definition: HotkeyDefinition
    monotonic_time: float
    wall_time: str
    elapsed_s: float


def _json_payload(text: str) -> dict[str, Any] | None:
    """解析节点发布的 JSON 状态；格式错误时返回 None。"""
    try:
        value = json.loads(text)
    except (TypeError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _finite_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _integer(value: Any) -> int | None:
    """将 SDK JSON 中的时间戳转为整数；无效或布尔值返回 None。"""
    if isinstance(value, bool):
        return None
    try:
        result = int(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result


def _format_age(age: float | None) -> str:
    if age is None:
        return "未收到"
    return f"{age:.2f}s 前"


def _format_rate(rate: float | None) -> str:
    return "--" if rate is None else f"{rate:5.1f}Hz"


def _format_byte_rate(rate: float | None) -> str:
    if rate is None:
        return "--"
    if rate >= 1024.0 * 1024.0:
        return f"{rate / (1024.0 * 1024.0):.2f}MiB/s"
    if rate >= 1024.0:
        return f"{rate / 1024.0:.1f}KiB/s"
    return f"{rate:.0f}B/s"


def _format_velocity(value: float | None) -> str:
    if value is None:
        return "   --   "
    return f"{value * 100.0:+7.1f}cm/s"


def _median(values: deque[float] | list[float]) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2.0


@dataclass
class TopicWatch:
    """记录一个话题最近到达时间和近似频率。"""

    times: deque[float] = field(default_factory=lambda: deque(maxlen=240))
    value: Any = None

    def observe(self, value: Any, now: float) -> None:
        self.value = value
        self.times.append(float(now))

    def age(self, now: float) -> float | None:
        if not self.times:
            return None
        return max(0.0, float(now) - self.times[-1])

    def rate(self, now: float, window_s: float = 2.0) -> float | None:
        recent = [stamp for stamp in self.times if now - stamp <= window_s]
        if len(recent) < 2:
            return None
        span = recent[-1] - recent[0]
        return None if span <= 1.0e-6 else (len(recent) - 1) / span


@dataclass
class TimestampWatch:
    """记录远端时间戳是否真正向前走，而不只看消息是否到达。"""

    value: int | None = None
    last_observed_at: float | None = None
    last_changed_at: float | None = None
    changes: deque[float] = field(default_factory=lambda: deque(maxlen=240))

    def observe(self, value: Any, now: float) -> None:
        timestamp = _integer(value)
        self.last_observed_at = float(now)
        if timestamp is None or timestamp <= 0:
            return
        if timestamp != self.value:
            self.value = timestamp
            self.last_changed_at = float(now)
            self.changes.append(float(now))

    def change_age(self, now: float) -> float | None:
        if self.last_changed_at is None:
            return None
        return max(0.0, float(now) - self.last_changed_at)

    def rate(self, now: float, window_s: float = 2.0) -> float | None:
        recent = [stamp for stamp in self.changes if now - stamp <= window_s]
        if len(recent) < 2:
            return None
        span = recent[-1] - recent[0]
        return None if span <= 1.0e-6 else (len(recent) - 1) / span


class PassiveHostMonitor:
    """
    只读 /proc 和 ss/TCP_INFO 的 PC Service/套接字监测器。

    重要：本类不加载 libPXREARobotSDK.so，不调用 PXREAInit，
    不创建任何网络连接。因此它不会抢占 sim 已有的 SDK 数据流。
    """

    SERVICE_NAME = "RoboticsServiceProcess"
    GRPC_PORT = 60061
    TCP_ESTABLISHED = "01"
    TCP_LISTEN = "0A"

    def __init__(self) -> None:
        self._events: deque[tuple[str, str]] = deque()
        self._last_poll_at = float("-inf")
        self._snapshot: dict[str, Any] | None = None
        self._previous_service_running: bool | None = None
        self._previous_grpc_listening: bool | None = None
        self._previous_peers: set[str] = set()
        self._ever_pico_peer = False
        self._last_peer_event = "未观察到"
        self._last_peer_event_at: float | None = None
        self._last_network_sample: tuple[float, int, int] | None = None
        self._last_transport_sample: tuple[float, int] | None = None
        self._ss_error: str | None = None

    @staticmethod
    def _process_identity(pid_path: Path) -> str:
        parts: list[str] = []
        for name in ("comm", "cmdline"):
            try:
                data = (pid_path / name).read_bytes()
            except OSError:
                continue
            parts.append(data.replace(b"\0", b" ").decode("utf-8", "replace"))
        try:
            parts.append(os.readlink(pid_path / "exe"))
        except OSError:
            pass
        return " ".join(parts)

    def _service_pids(self) -> list[int]:
        pids: list[int] = []
        try:
            entries = list(Path("/proc").iterdir())
        except OSError:
            return pids
        for entry in entries:
            if not entry.name.isdigit():
                continue
            if self.SERVICE_NAME in self._process_identity(entry):
                pids.append(int(entry.name))
        return sorted(pids)

    @staticmethod
    def _socket_inodes(pids: list[int]) -> tuple[set[str], bool]:
        inodes: set[str] = set()
        at_least_one_fd_directory_read = False
        for pid in pids:
            fd_directory = Path(f"/proc/{pid}/fd")
            try:
                descriptors = list(fd_directory.iterdir())
            except OSError:
                continue
            at_least_one_fd_directory_read = True
            for descriptor in descriptors:
                try:
                    target = os.readlink(descriptor)
                except OSError:
                    continue
                if target.startswith("socket:[") and target.endswith("]"):
                    inodes.add(target[8:-1])
        return inodes, at_least_one_fd_directory_read

    @staticmethod
    def _decode_endpoint(value: str, ipv6: bool) -> tuple[str, int]:
        address_hex, port_hex = value.split(":", 1)
        port = int(port_hex, 16)
        packed = bytes.fromhex(address_hex)
        if ipv6:
            # /proc/net/tcp6 以每 32bit 小端字的方式显示地址。
            packed = b"".join(
                packed[index : index + 4][::-1]
                for index in range(0, len(packed), 4)
            )
            address = socket.inet_ntop(socket.AF_INET6, packed)
        else:
            address = socket.inet_ntop(socket.AF_INET, packed[::-1])
        return address, port

    @staticmethod
    def _is_loopback(address: str) -> bool:
        if address.startswith("::ffff:"):
            address = address.removeprefix("::ffff:")
        return address == "::1" or address.startswith("127.")

    def _read_service_sockets(
        self, inodes: set[str]
    ) -> list[dict[str, Any]]:
        sockets: list[dict[str, Any]] = []
        for filename, ipv6 in (("/proc/net/tcp", False), ("/proc/net/tcp6", True)):
            try:
                lines = Path(filename).read_text(encoding="ascii").splitlines()[1:]
            except OSError:
                continue
            for line in lines:
                fields = line.split()
                if len(fields) < 10 or fields[9] not in inodes:
                    continue
                try:
                    local_address, local_port = self._decode_endpoint(
                        fields[1], ipv6
                    )
                    remote_address, remote_port = self._decode_endpoint(
                        fields[2], ipv6
                    )
                    tx_hex, rx_hex = fields[4].split(":", 1)
                except (OSError, ValueError):
                    continue
                sockets.append(
                    {
                        "state": fields[3],
                        "local_address": local_address,
                        "local_port": local_port,
                        "remote_address": remote_address,
                        "remote_port": remote_port,
                        "tx_queue_bytes": int(tx_hex, 16),
                        "rx_queue_bytes": int(rx_hex, 16),
                        "inode": fields[9],
                    }
                )
        return sockets

    def _network_totals(self) -> tuple[int, int] | None:
        try:
            lines = Path("/proc/net/dev").read_text(encoding="ascii").splitlines()[2:]
        except OSError:
            return None
        receive_bytes = 0
        receive_packets = 0
        for line in lines:
            if ":" not in line:
                continue
            interface, values = line.split(":", 1)
            if interface.strip() == "lo":
                continue
            fields = values.split()
            if len(fields) >= 2:
                receive_bytes += int(fields[0])
                receive_packets += int(fields[1])
        return receive_bytes, receive_packets

    def _network_rates(self, now: float) -> tuple[float | None, float | None]:
        totals = self._network_totals()
        if totals is None:
            return None, None
        previous = self._last_network_sample
        self._last_network_sample = (now, totals[0], totals[1])
        if previous is None or now <= previous[0]:
            return None, None
        span = now - previous[0]
        return (
            max(0.0, totals[0] - previous[1]) / span,
            max(0.0, totals[1] - previous[2]) / span,
        )

    @staticmethod
    def _ss_endpoint_host(endpoint: str) -> str:
        endpoint = endpoint.strip()
        if endpoint.startswith("[") and "]:" in endpoint:
            return endpoint[1 : endpoint.rfind("]:")]
        return endpoint.rsplit(":", 1)[0]

    def _transport_receive_total(self, pids: list[int]) -> int | None:
        """
        通过 ss/TCP_INFO 被动读取 PC Service 非回环连接累计收包字节。

        ss 只查询内核 netlink，不会向 PICO 或 PC Service 建立连接。
        如果当前系统无权限查询，返回 None 并自动降级。
        """
        if not pids:
            return None
        try:
            result = subprocess.run(
                ["ss", "-H", "-t", "-i", "-n", "-p", "-O", "state", "established"],
                capture_output=True,
                text=True,
                timeout=0.40,
                check=False,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            self._ss_error = f"{type(exc).__name__}: {exc}"
            return None
        if result.returncode != 0:
            self._ss_error = result.stderr.strip() or f"ss exit={result.returncode}"
            return None
        pid_tokens = tuple(f"pid={pid}," for pid in pids)
        total = 0
        matched = False
        for line in result.stdout.splitlines():
            if not any(token in line for token in pid_tokens):
                continue
            fields = line.split()
            if len(fields) < 5:
                continue
            remote_host = self._ss_endpoint_host(fields[4])
            if self._is_loopback(remote_host):
                continue
            match = re.search(r"(?:^|\s)bytes_received:(\d+)(?:\s|$)", line)
            if match is None:
                continue
            total += int(match.group(1))
            matched = True
        self._ss_error = None
        return total if matched else None

    def _transport_receive_rate(
        self, now: float, pids: list[int]
    ) -> float | None:
        total = self._transport_receive_total(pids)
        if total is None:
            self._last_transport_sample = None
            return None
        previous = self._last_transport_sample
        self._last_transport_sample = (now, total)
        if previous is None or now <= previous[0] or total < previous[1]:
            return None
        return (total - previous[1]) / (now - previous[0])

    def _queue_transition_events(
        self,
        now: float,
        service_running: bool,
        grpc_listening: bool,
        peers: set[str],
    ) -> None:
        if self._previous_service_running is not None:
            if service_running and not self._previous_service_running:
                self._events.append(("恢复", "已观察到 PC Service 进程重新启动。"))
            elif not service_running and self._previous_service_running:
                self._events.append(("严重", "PC Service 进程已退出。"))
        if self._previous_grpc_listening is not None:
            if grpc_listening and not self._previous_grpc_listening:
                self._events.append(("恢复", "PC Service 60061 本地端口已恢复。"))
            elif not grpc_listening and self._previous_grpc_listening:
                self._events.append(("严重", "PC Service 60061 本地端口已消失。"))
        for peer in sorted(peers - self._previous_peers):
            self._ever_pico_peer = True
            self._last_peer_event = f"connected:{peer}"
            self._last_peer_event_at = now
            self._events.append(("恢复", f"观察到 PICO/PC Service 非回环连接：{peer}。"))
        for peer in sorted(self._previous_peers - peers):
            self._last_peer_event = f"disconnected:{peer}"
            self._last_peer_event_at = now
            self._events.append(("严重", f"PICO/PC Service 非回环连接已断开：{peer}。"))
        self._previous_service_running = service_running
        self._previous_grpc_listening = grpc_listening
        self._previous_peers = set(peers)

    def snapshot(self, now: float) -> dict[str, Any]:
        if self._snapshot is not None and now - self._last_poll_at < 0.45:
            return dict(self._snapshot)
        self._last_poll_at = now
        pids = self._service_pids()
        inodes, fd_accessible = self._socket_inodes(pids)
        sockets = self._read_service_sockets(inodes)
        service_running = bool(pids)
        grpc_listening = any(
            item["state"] == self.TCP_LISTEN
            and item["local_port"] == self.GRPC_PORT
            for item in sockets
        )
        sdk_connections = [
            item
            for item in sockets
            if item["state"] == self.TCP_ESTABLISHED
            and self._is_loopback(item["remote_address"])
            and (
                item["local_port"] == self.GRPC_PORT
                or item["remote_port"] == self.GRPC_PORT
            )
        ]
        pico_connections = [
            item
            for item in sockets
            if item["state"] == self.TCP_ESTABLISHED
            and not self._is_loopback(item["remote_address"])
            and item["remote_address"] not in {"0.0.0.0", "::"}
        ]
        peers = {
            f"{item['remote_address']}:{item['remote_port']}"
            for item in pico_connections
        }
        self._queue_transition_events(
            now, service_running, grpc_listening, peers
        )
        receive_bytes_s, receive_packets_s = self._network_rates(now)
        transport_receive_bytes_s = self._transport_receive_rate(now, pids)
        self._snapshot = {
            "available": Path("/proc/net/tcp").is_file(),
            "start_error": None,
            "service_running": service_running,
            "service_pids": pids,
            "service_fd_accessible": fd_accessible,
            "grpc_listening": grpc_listening,
            "sdk_connection_count": len(sdk_connections),
            "pico_peers": sorted(peers),
            "ever_pico_peer": self._ever_pico_peer,
            "last_peer_event": self._last_peer_event,
            "last_peer_event_age_s": (
                None
                if self._last_peer_event_at is None
                else max(0.0, now - self._last_peer_event_at)
            ),
            "non_loopback_receive_bytes_s": receive_bytes_s,
            "non_loopback_receive_packets_s": receive_packets_s,
            "pico_transport_receive_bytes_s": transport_receive_bytes_s,
            "tcp_info_available": self._ss_error is None,
            "tcp_info_error": self._ss_error,
            "pico_socket_rx_queue_bytes": sum(
                item["rx_queue_bytes"] for item in pico_connections
            ),
            "socket_count": len(sockets),
            "monitor_kind": "passive_proc_only",
        }
        return dict(self._snapshot)

    def pop_events(self) -> list[tuple[str, str]]:
        events = list(self._events)
        self._events.clear()
        return events

    def close(self) -> None:
        """纯 /proc 监测没有会话需要关闭。"""


@dataclass
class PoseWatch(TopicWatch):
    """额外记录末端的物理竖直高度，用来比较向下跟踪速度。"""

    heights: deque[tuple[float, float]] = field(
        default_factory=lambda: deque(maxlen=240)
    )
    position: tuple[float, float, float] | None = None

    def observe_pose(self, message: PoseStamped, side: str, now: float) -> None:
        position = message.pose.position
        xyz = (float(position.x), float(position.y), float(position.z))
        if not all(math.isfinite(value) for value in xyz):
            return

        # Chest 坐标定义：左臂 Y+=向下，右臂 Y+=向上。
        # 统一成 height 增大=抬手、height 减小=放手。
        height = -xyz[1] if side == "left" else xyz[1]
        self.position = xyz
        self.heights.append((float(now), height))
        self.observe(message, now)

    def vertical_velocity(
        self, now: float, window_s: float = 0.8
    ) -> float | None:
        recent = [item for item in self.heights if now - item[0] <= window_s]
        if len(recent) < 3:
            return None
        span = recent[-1][0] - recent[0][0]
        if span < 0.30:
            return None
        return (recent[-1][1] - recent[0][1]) / span


class TeleopDiagnostic(Node):
    """只读订阅并把整条遥操作链路压缩成中文诊断。"""

    def __init__(self) -> None:
        super().__init__("pico_tianji_read_only_diagnostic")
        self._started_at = time.monotonic()
        self._wall_started_at = datetime.now().astimezone()
        self._last_render_at = 0.0
        self._state: str | None = None
        self._states_seen: set[str] = set()
        self._markers: list[UserMarker] = []
        self._last_marker: UserMarker | None = None
        self._marker_banner_until = float("-inf")
        self._last_hotkey_at: dict[str, float] = {}
        self._terminal_fd: int | None = None
        self._terminal_settings: list[Any] | None = None
        self._log_failed = False
        log_directory = Path(
            os.environ.get(
                "PICO_TIANJI_DIAGNOSTIC_LOG_DIR",
                str(Path(__file__).resolve().parent / "logs"),
            )
        ).expanduser()
        log_directory.mkdir(parents=True, exist_ok=True)
        log_name = self._wall_started_at.strftime(
            "teleop_diagnostic_%Y%m%d_%H%M%S.log"
        )
        self._log_path = (log_directory / log_name).resolve()
        self._log_file = self._log_path.open(
            "w", encoding="utf-8", buffering=1
        )
        self._write_log(
            "PICO / Pinocchio / Marvin 只读诊断日志\n"
            f"开始时间：{self._wall_started_at.isoformat()}\n"
            f"日志文件：{self._log_path}\n"
            "安全属性：仅订阅 ROS 并读取 Linux /proc/ss；不发布，"
            "不加载 PICO/Marvin SDK，不创建网络连接，不控制机械臂。\n"
            + "=" * 78
            + "\n"
        )

        self._input = TopicWatch()
        self._sim = TopicWatch()
        self._real = TopicWatch()
        self._teleop_state_topic = TopicWatch()
        self._targets = {side: PoseWatch() for side in SIDES}
        self._solved = {side: PoseWatch() for side in SIDES}
        self._commands = {side: TopicWatch() for side in SIDES}
        self._command_source_stamp_ns = {side: None for side in SIDES}
        self._feedback = {side: TopicWatch() for side in SIDES}

        self._input_history: deque[tuple[float, dict[str, Any]]] = deque(
            maxlen=40
        )
        self._pending_returns: list[float] = []
        self._return_reasons: Counter[str] = Counter()
        self._events: deque[tuple[float, str, str]] = deque(maxlen=12)
        self._issues: dict[str, tuple[str, str]] = {}
        self._issue_activations: Counter[str] = Counter()
        self._maximum_tracking_error = {side: 0.0 for side in SIDES}
        self._maximum_tracking_joint = {side: None for side in SIDES}
        self._down_observed = {side: False for side in SIDES}
        self._down_slow_ticks = {side: 0 for side in SIDES}
        self._down_good_ticks = {side: 0 for side in SIDES}
        self._ros_controller_timestamp = TimestampWatch()
        self._ros_body_timestamp = TimestampWatch()
        self._last_layer_code: str | None = None
        self._layer_diagnosis_counts: Counter[str] = Counter()
        self._pico_transport_live_rates: deque[float] = deque(maxlen=120)
        # 0.25s 一个样本，保留约 60s。人工标记会写入前 15s，
        # 适应“先看到现象，过几秒才按 p/m”的实际延迟。
        self._layer_history: deque[dict[str, Any]] = deque(maxlen=240)

        # 只读 /proc 和 ss/TCP_INFO，不再创建第二个 PXREA SDK 会话。
        self._host_monitor = PassiveHostMonitor()
        host_snapshot = self._host_monitor.snapshot(time.monotonic())
        if host_snapshot["available"]:
            self._event(
                "信息",
                "被动分层监测已启用：仅读 /proc 与 ss/TCP_INFO，"
                "未加载 PICO SDK，不会抢占 sim 数据流。",
            )
        else:
            self._event(
                "警告",
                "被动分层监测不可用："
                f"{host_snapshot['start_error']}；ROS 原有诊断仍会继续。",
            )

        self.create_subscription(
            String, "/pico_body/status", self._on_input_status, 10
        )
        self.create_subscription(
            String, "/pico_body_sim/status", self._on_sim_status, 10
        )
        self.create_subscription(
            String, "/pico_body_real/status", self._on_real_status, 10
        )
        self.create_subscription(
            String,
            "/pico_body/teleop_state",
            self._on_teleop_state,
            10,
        )

        for side in SIDES:
            self.create_subscription(
                PoseStamped,
                f"/pico_body/{side}_arm_target_pose",
                lambda message, side=side: self._on_target(side, message),
                10,
            )
            self.create_subscription(
                PoseStamped,
                f"/pico_body_sim/{side}_arm/solved_pose",
                lambda message, side=side: self._on_solved(side, message),
                10,
            )
            self.create_subscription(
                JointState,
                f"/pico_body_sim/{side}_arm/joint_commands",
                lambda message, side=side: self._on_command(side, message),
                10,
            )
            self.create_subscription(
                JointState,
                f"/{side}_arm/joint_states",
                lambda message, side=side: self._on_feedback(side, message),
                10,
            )

        self.create_timer(0.25, self._tick)
        self._event(
            "信息",
            "只读诊断器已启动：等待 pixi run sim 的话题。",
        )
        self._setup_keyboard()

    def _on_input_status(self, message: String) -> None:
        now = time.monotonic()
        payload = _json_payload(message.data)
        self._input.observe(payload, now)
        if payload is None:
            self._activate_issue(
                "input_json", "警告", "PICO 状态 JSON 无法解析。"
            )
            return
        self._clear_issue("input_json")
        self._input_history.append((now, payload))
        self._ros_controller_timestamp.observe(
            payload.get("source_timestamp_ns"), now
        )
        self._ros_body_timestamp.observe(
            payload.get("smpl_timestamp_ns"), now
        )
        state = payload.get("state")
        if isinstance(state, str):
            self._observe_state(state, now)

    def _on_sim_status(self, message: String) -> None:
        now = time.monotonic()
        payload = _json_payload(message.data)
        self._sim.observe(payload, now)
        if payload is None:
            self._activate_issue(
                "sim_json", "警告", "Pinocchio 状态 JSON 无法解析。"
            )
        else:
            self._clear_issue("sim_json")

    def _on_real_status(self, message: String) -> None:
        now = time.monotonic()
        payload = _json_payload(message.data)
        first_seen = self._real.value is None
        self._real.observe(payload, now)
        if first_seen and payload is not None:
            self._event(
                "信息",
                "已发现真机桥；继续保持只读，仅增加真机反馈诊断。",
            )
        if payload is None:
            self._activate_issue(
                "real_json", "警告", "真机桥状态 JSON 无法解析。"
            )
        else:
            self._clear_issue("real_json")

    def _on_teleop_state(self, message: String) -> None:
        now = time.monotonic()
        state = str(message.data)
        self._teleop_state_topic.observe(state, now)
        self._observe_state(state, now)

    def _observe_state(self, state: str, now: float) -> None:
        if state not in {"idle", "teleop", "returning"}:
            self._activate_issue(
                "invalid_state", "警告", f"收到未知遥操作状态：{state!r}。"
            )
            return
        self._clear_issue("invalid_state")
        if state == self._state:
            return
        previous = self._state
        self._state = state
        self._states_seen.add(state)
        if previous is None:
            self._event("信息", f"当前遥操作状态：{state}。")
            return
        self._event("信息", f"遥操作状态：{previous} -> {state}。")
        if previous == "teleop" and state == "returning":
            # 等待下一次 0.5s 状态快照，再综合判断 A 键和输入健康度。
            self._pending_returns.append(now)

    def _on_target(self, side: str, message: PoseStamped) -> None:
        self._targets[side].observe_pose(message, side, time.monotonic())

    def _on_solved(self, side: str, message: PoseStamped) -> None:
        self._solved[side].observe_pose(message, side, time.monotonic())

    def _on_command(self, side: str, message: JointState) -> None:
        now = time.monotonic()
        values = tuple(float(value) for value in message.position)
        self._commands[side].observe(values, now)
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        self._command_source_stamp_ns[side] = (
            stamp_ns if stamp_ns > 0 else None
        )

    def _on_feedback(self, side: str, message: JointState) -> None:
        values = tuple(float(value) for value in message.position)
        self._feedback[side].observe(values, time.monotonic())

    def _event(self, level: str, message: str) -> None:
        now = time.monotonic()
        self._events.append((now, level, message))
        self._write_log(
            f"[{self._wall_time()}] EVENT [{level}] "
            f"运行 {now - self._started_at:.3f}s：{message}\n"
        )

    def _wall_time(self) -> str:
        return datetime.now().astimezone().isoformat(timespec="milliseconds")

    def _write_log(self, text: str) -> None:
        """立即写入日志；单次日志故障不能中断只读诊断。"""
        if self._log_failed:
            return
        try:
            self._log_file.write(text)
            self._log_file.flush()
        except OSError as exc:
            self._log_failed = True
            print(f"警告：诊断日志写入失败：{exc}", file=sys.stderr)

    def close_log(self) -> None:
        if not self._log_file.closed:
            self._log_file.close()

    def _setup_keyboard(self) -> None:
        """把当前终端设为单键读取；保留 Ctrl+C 的信号行为。"""
        if not sys.stdin.isatty():
            self._event(
                "警告",
                "标准输入不是交互终端，p/m 人工标记不可用；日志记录仍正常。",
            )
            return
        try:
            terminal_fd = sys.stdin.fileno()
            terminal_settings = termios.tcgetattr(terminal_fd)
            tty.setcbreak(terminal_fd)
        except (OSError, termios.error, ValueError) as exc:
            self._event("警告", f"无法启用 p/m 单键人工标记：{exc}")
            return
        self._terminal_fd = terminal_fd
        self._terminal_settings = terminal_settings
        atexit.register(self.restore_terminal)
        keys = "，".join(
            f"{definition.key}={definition.title}"
            for definition in HOTKEY_DEFINITIONS
        )
        self._event("信息", f"人工标记热键已启用：{keys}（不用按回车）。")

    def restore_terminal(self) -> None:
        """恢复脚本启动前的终端设置；可重复调用。"""
        if self._terminal_fd is None or self._terminal_settings is None:
            return
        try:
            termios.tcsetattr(
                self._terminal_fd,
                termios.TCSADRAIN,
                self._terminal_settings,
            )
        except (OSError, termios.error):
            pass
        self._terminal_fd = None
        self._terminal_settings = None

    def _poll_hotkeys(self, now: float) -> None:
        """非阻塞读取终端按键，不影响 ROS 回调频率。"""
        if self._terminal_fd is None:
            return
        try:
            readable, _, _ = select.select([self._terminal_fd], [], [], 0.0)
            if not readable:
                return
            data = os.read(self._terminal_fd, 64).decode(
                "utf-8", errors="ignore"
            )
        except (OSError, ValueError) as exc:
            self.restore_terminal()
            self._event("警告", f"人工标记热键读取失败：{exc}")
            return

        for character in data.lower():
            definition = HOTKEY_BY_KEY.get(character)
            if definition is None:
                continue
            # 避免长按键盘产生自动连发；正常的多次人工标记不受影响。
            if now - self._last_hotkey_at.get(character, float("-inf")) < 0.5:
                continue
            self._last_hotkey_at[character] = now
            self._record_user_marker(definition, now)

    @staticmethod
    def _json_safe_status(watch: TopicWatch) -> Any:
        return watch.value if isinstance(watch.value, dict) else None

    def _command_timing(self, now: float) -> dict[str, float | None]:
        """检测进程观察到的左右关节消息时序，不参与任何控制。"""
        left_time = (
            self._commands["left"].times[-1]
            if self._commands["left"].times
            else None
        )
        right_time = (
            self._commands["right"].times[-1]
            if self._commands["right"].times
            else None
        )
        left_stamp = self._command_source_stamp_ns["left"]
        right_stamp = self._command_source_stamp_ns["right"]
        return {
            "left_age_ms": (
                None if left_time is None else max(0.0, now - left_time) * 1000.0
            ),
            "right_age_ms": (
                None if right_time is None else max(0.0, now - right_time) * 1000.0
            ),
            "receive_pair_skew_ms": (
                None
                if left_time is None or right_time is None
                else abs(left_time - right_time) * 1000.0
            ),
            "source_stamp_skew_ms": (
                None
                if left_stamp is None or right_stamp is None
                else abs(left_stamp - right_stamp) * 1.0e-6
            ),
            "pair_skew_limit_ms": COMMAND_PAIR_SKEW_LIMIT_MS,
            "command_timeout_ms": COMMAND_TIMEOUT_MS,
        }

    def _marker_context(
        self, marker: UserMarker
    ) -> dict[str, Any]:
        """冻结人工按键瞬间的关键原始值和派生诊断量。"""
        now = marker.monotonic_time
        native = self._host_monitor.snapshot(now)
        native["pico_transport_live_baseline_bytes_s"] = _median(
            self._pico_transport_live_rates
        )
        layer_code, layer_level, layer_message = self._layer_diagnosis(
            now, native
        )
        recent_layer_history = [
            item
            for item in self._layer_history
            if now - item["monotonic_time"] <= MARKER_LAYER_LOOKBACK_S
        ]
        recent_abnormal = [
            item
            for item in recent_layer_history
            if item["code"]
            not in {"layers_live", "layers_live_limited", "layers_waiting"}
        ]
        recent_finding = recent_abnormal[-1] if recent_abnormal else None
        arms: dict[str, Any] = {}
        for side in SIDES:
            command = self._commands[side].value
            feedback = self._feedback[side].value
            tracking = None
            if (
                isinstance(command, tuple)
                and isinstance(feedback, tuple)
                and len(command) >= 7
                and len(feedback) >= 7
            ):
                differences = [
                    abs(command[index] - feedback[index])
                    for index in range(7)
                ]
                maximum = max(differences)
                tracking = {
                    "joint_index": differences.index(maximum) + 1,
                    "maximum_error_deg": maximum,
                }
            arms[side] = {
                "target_position": self._targets[side].position,
                "solved_position": self._solved[side].position,
                "target_vertical_velocity_m_s": (
                    self._targets[side].vertical_velocity(now)
                ),
                "solved_vertical_velocity_m_s": (
                    self._solved[side].vertical_velocity(now)
                ),
                "target_rate_hz": self._targets[side].rate(now),
                "solved_rate_hz": self._solved[side].rate(now),
                "command_rate_hz": self._commands[side].rate(now),
                "feedback_rate_hz": self._feedback[side].rate(now),
                "command_joints_deg": command,
                "feedback_joints_deg": feedback,
                "tracking_error": tracking,
            }
        return {
            "marker": {
                "key": marker.definition.key,
                "code": marker.definition.code,
                "title": marker.definition.title,
                "description": marker.definition.description,
                "wall_time": marker.wall_time,
                "elapsed_s": marker.elapsed_s,
            },
            "teleop_state": self._state,
            "input_status": self._json_safe_status(self._input),
            "pinocchio_status": self._json_safe_status(self._sim),
            "real_status": self._json_safe_status(self._real),
            "diagnostic_command_timing": self._command_timing(now),
            "topic_age_s": {
                "input": self._input.age(now),
                "pinocchio": self._sim.age(now),
                "real": self._real.age(now),
            },
            "layer_diagnosis": {
                "code": layer_code,
                "level": layer_level,
                "message": layer_message,
            },
            "layer_lookback": {
                "window_s": MARKER_LAYER_LOOKBACK_S,
                "latest_abnormal_finding": recent_finding,
                "timeline": [
                    {
                        key: value
                        for key, value in item.items()
                        if key != "monotonic_time"
                    }
                    for item in recent_layer_history
                ],
            },
            "passive_host_observation": native,
            "python_sdk_via_ros": {
                "controller_timestamp_ns": self._ros_controller_timestamp.value,
                "controller_timestamp_change_age_s": (
                    self._ros_controller_timestamp.change_age(now)
                ),
                "controller_timestamp_rate_hz": (
                    self._ros_controller_timestamp.rate(now)
                ),
                "body_timestamp_ns": self._ros_body_timestamp.value,
                "body_timestamp_change_age_s": (
                    self._ros_body_timestamp.change_age(now)
                ),
                "body_timestamp_rate_hz": self._ros_body_timestamp.rate(now),
            },
            "active_issues": [
                {"level": level, "message": message}
                for level, message in self._issues.values()
            ],
            "arms": arms,
        }

    def _record_user_marker(
        self, definition: HotkeyDefinition, now: float
    ) -> None:
        marker = UserMarker(
            definition=definition,
            monotonic_time=now,
            wall_time=self._wall_time(),
            elapsed_s=now - self._started_at,
        )
        self._markers.append(marker)
        self._last_marker = marker
        self._marker_banner_until = now + 5.0
        self._event(
            "人工标记",
            f"[{definition.code}] {definition.title}",
        )
        context = self._marker_context(marker)
        self._write_log(
            "\n"
            + "#" * 78
            + "\n"
            + f"### USER MARKER key={definition.key} "
            + f"code={definition.code}\n"
            + f"### {definition.title}\n"
            + f"### 时间={marker.wall_time} 运行={marker.elapsed_s:.3f}s\n"
            + json.dumps(context, ensure_ascii=False, indent=2)
            + "\n"
            + "#" * 78
            + "\n"
        )
        # 下一次界面刷新立刻显示醒目的人工标签。
        self._last_render_at = float("-inf")

    def _activate_issue(self, key: str, level: str, message: str) -> None:
        if key not in self._issues:
            self._issues[key] = (level, message)
            self._issue_activations[key] += 1
            self._event(level, message)
        else:
            # 数值可能变化，但同一问题不重复刷事件日志。
            self._issues[key] = (level, message)

    def _clear_issue(self, key: str, resolved_message: str | None = None) -> None:
        if key not in self._issues:
            return
        del self._issues[key]
        if resolved_message:
            self._event("恢复", resolved_message)

    def _finalize_return_reasons(self, now: float) -> None:
        remaining = []
        for transition_at in self._pending_returns:
            if now - transition_at < 0.70:
                remaining.append(transition_at)
                continue
            nearby = [
                payload
                for stamp, payload in self._input_history
                if abs(stamp - transition_at) <= 0.85
            ]
            source_bad = any(
                payload.get("source") != "live"
                or payload.get("smpl_source") not in HEALTHY_SMPL_STATES
                or payload.get("smpl_used") is not True
                or payload.get("error") is not None
                for payload in nearby
            )
            a_seen = any(payload.get("right_a_pressed") is True for payload in nearby)

            # 状态机优先检查 signal_live，再检查 A 键，因此同时出现时归为断流。
            if source_bad:
                reason = "signal_lost"
                conclusion = "退出遥操作：检测到 PICO/SMPL 异常，基本判断为输入断流。"
                level = "警告"
            elif a_seen:
                reason = "right_a"
                conclusion = "退出遥操作：检测到右手柄 A，基本判断为 A 键触发。"
                level = "信息"
            else:
                reason = "uncertain"
                conclusion = (
                    "退出遥操作：未在低频状态快照中看到 A 或持续断流；"
                    "可能是极短 PICO/SMPL 丢帧，也可能是短 A 脉冲。"
                )
                level = "警告"
            self._return_reasons[reason] += 1
            self._event(level, conclusion)
        self._pending_returns = remaining

    def _tick(self) -> None:
        now = time.monotonic()
        self._evaluate_host_layers(now)
        self._finalize_return_reasons(now)
        self._evaluate_input(now)
        self._evaluate_sim(now)
        self._evaluate_motion(now)
        self._evaluate_real(now)
        self._poll_hotkeys(now)
        self._render(now)

    def _layer_diagnosis(
        self, now: float, native: dict[str, Any] | None = None
    ) -> tuple[str, str, str]:
        """返回（稳定代码，级别，中文分层结论）。"""
        native = native or self._host_monitor.snapshot(now)
        running = now - self._started_at
        if not native["available"]:
            return (
                "monitor_unavailable",
                "警告",
                "无法读取 /proc；仅保留 ROS/IK/真机分层判断。",
            )
        if not native["service_running"]:
            return (
                "segment2_service_down",
                "严重",
                "[第2段] MiniPC 上 RoboticsServiceProcess 未运行。",
            )
        if not native["service_fd_accessible"]:
            input_payload = self._input.value
            if (
                isinstance(input_payload, dict)
                and input_payload.get("source") == "live"
            ):
                return (
                    "layers_live_limited",
                    "正常",
                    "SDK/ROS 时间戳正常；但 PC Service 套接字权限不可读，"
                    "前两段只能间接判断。",
                )
            return (
                "monitor_socket_permission_limited",
                "警告",
                "PC Service 进程存在，但其 /proc/PID/fd 无读取权限；"
                "无法被动观察前两段套接字。",
            )
        if not native["grpc_listening"]:
            return (
                "segment2_endpoint_down",
                "严重",
                "[第2段] PC Service 进程在，但 127.0.0.1:60061 SDK 端口未监听。",
            )
        if native["ever_pico_peer"] and not native["pico_peers"]:
            return (
                "segment1_peer_disconnected",
                "严重",
                "[第1段] 之前存在的 PICO→PC Service TCP 连接已断开；"
                "检查头显 App/Wi-Fi。",
            )

        input_age = self._input.age(now)
        input_payload = self._input.value
        if running > 3.0 and (
            input_age is None
            or input_age > 1.5
            or not isinstance(input_payload, dict)
        ):
            if native["sdk_connection_count"] == 0:
                return (
                    "segment2_sdk_not_connected",
                    "严重",
                    "[第2/3段] PC Service 端口正常，但未观察到本地 SDK "
                    "连接且 /pico_body/status 缺失。",
                )
            return (
                "segment3_ros_missing",
                "严重",
                "[第3段] 本地 SDK 连接存在，但 pico_controller_input "
                "没有持续发布 ROS 状态。",
            )

        if isinstance(input_payload, dict):
            source_live = input_payload.get("source") == "live"
            ros_timestamp_age = self._ros_controller_timestamp.change_age(now)
            source_stale = not source_live or (
                ros_timestamp_age is not None and ros_timestamp_age > 1.25
            )
            if source_stale:
                if not native["pico_peers"]:
                    return (
                        "segment1_no_pico_peer",
                        "严重",
                        "[第1段] SDK 时间戳停止，且未观察到 PICO→PC Service "
                        "非回环 TCP 连接；首先检查 PICO App/Wi-Fi。",
                    )
                transport_rate = native.get("pico_transport_receive_bytes_s")
                live_baseline = _median(self._pico_transport_live_rates)
                if (
                    transport_rate is not None
                    and live_baseline is not None
                    and live_baseline >= 1024.0
                ):
                    ratio = transport_rate / max(live_baseline, 1.0)
                    if ratio <= 0.15:
                        return (
                            "segment1_transport_stopped",
                            "严重",
                            "[第1段] TCP 连接未断，但 PC Service 从 PICO 收到的"
                            f"字节率已降至正常基线的 {ratio * 100.0:.0f}%；"
                            "更像 PICO 姿态流停止/只剩心跳。",
                        )
                    if ratio >= 0.50:
                        return (
                            "segment2_forwarding_stale",
                            "严重",
                            "[第2段] PC Service 仍以接近正常速率从 PICO 收包，"
                            "但 SDK 姿态时间戳停止；更像 PC Service→SDK "
                            "转发/解析问题。",
                        )
                return (
                    "boundary12_sdk_stale",
                    "严重",
                    "[第1/2段边界] PICO TCP 与本地 SDK 连接都在，但 SDK "
                    "姿态时间戳已停；可能只剩心跳，也可能 PC Service "
                    "未向 SDK 转发新帧。",
                )
            body_bad = (
                input_payload.get("smpl_source") not in HEALTHY_SMPL_STATES
                or input_payload.get("smpl_used") is not True
            )
            if body_bad:
                return (
                    "segment3_body_stale",
                    "严重",
                    "[Body支路] 手柄 SDK 时间戳在更新，但 SMPL Body "
                    "不可用；检查 PICO Body 追踪开关和 SDK Body 数据。",
                )
            return (
                "layers_live",
                "正常",
                "可观测证据正常：PC Service/60061 存在，SDK/ROS "
                "时间戳在更新（已间接证明上游有效）。",
            )
        return (
            "layers_waiting",
            "信息",
            "正在等待足够的分层数据。",
        )

    def _evaluate_host_layers(self, now: float) -> None:
        """在 ROS 主线程中轮询 /proc/ss 被动证据并记录分层变化。"""
        for level, message in self._host_monitor.pop_events():
            self._event(level, message)
        native = self._host_monitor.snapshot(now)
        input_payload = (
            self._input.value if isinstance(self._input.value, dict) else {}
        )
        transport_rate = native["pico_transport_receive_bytes_s"]
        if (
            input_payload.get("source") == "live"
            and transport_rate is not None
            and transport_rate >= 1024.0
        ):
            self._pico_transport_live_rates.append(transport_rate)
        native["pico_transport_live_baseline_bytes_s"] = _median(
            self._pico_transport_live_rates
        )
        code, level, message = self._layer_diagnosis(now, native)
        if code != self._last_layer_code:
            self._last_layer_code = code
            self._layer_diagnosis_counts[code] += 1
            if code not in {
                "layers_waiting",
                "layers_live",
                "layers_live_limited",
            }:
                self._event(level, f"分层判断：{message}")

        self._layer_history.append(
            {
                "monotonic_time": now,
                "elapsed_s": now - self._started_at,
                "code": code,
                "level": level,
                "message": message,
                "service_running": native["service_running"],
                "service_fd_accessible": native["service_fd_accessible"],
                "grpc_listening": native["grpc_listening"],
                "sdk_connection_count": native["sdk_connection_count"],
                "pico_peers": native["pico_peers"],
                "peer_event": native["last_peer_event"],
                "host_receive_bytes_s": native[
                    "non_loopback_receive_bytes_s"
                ],
                "pico_transport_receive_bytes_s": native[
                    "pico_transport_receive_bytes_s"
                ],
                "pico_transport_live_baseline_bytes_s": native[
                    "pico_transport_live_baseline_bytes_s"
                ],
                "ros_source": input_payload.get("source"),
                "ros_smpl_source": input_payload.get("smpl_source"),
                "ros_timestamp_age_s": (
                    self._ros_controller_timestamp.change_age(now)
                ),
            }
        )

        self._set_condition(
            "host_monitor_unavailable",
            not native["available"],
            "警告",
            "被动 /proc 分层监测不可用。",
        )
        self._set_condition(
            "host_service_down",
            native["available"] and not native["service_running"],
            "严重",
            "MiniPC PC Service 进程未运行。",
        )
        self._set_condition(
            "host_grpc_down",
            native["service_running"]
            and native["service_fd_accessible"]
            and not native["grpc_listening"],
            "严重",
            "PC Service 进程存在，但 60061 SDK 端口未监听。",
        )
        self._set_condition(
            "host_pico_peer_missing",
            code
            in {
                "segment1_peer_disconnected",
                "segment1_no_pico_peer",
                "segment1_transport_stopped",
            },
            "严重",
            "PICO→PC Service 连接或入流字节率异常，且 SDK 数据已停。",
        )
        self._set_condition(
            "host_boundary_stale",
            code == "boundary12_sdk_stale",
            "严重",
            "网络与 SDK 连接仍在，但姿态时间戳已停（第1/2段边界）。",
        )
        self._set_condition(
            "host_service_forwarding_stale",
            code == "segment2_forwarding_stale",
            "严重",
            "PC Service 从 PICO 收包速率仍正常，但 SDK 姿态时间戳已停。",
        )
        self._set_condition(
            "python_sdk_layer_stale",
            code in {
                "segment3_ros_missing",
                "segment3_body_stale",
            },
            "严重",
            "异常已缩小到 Python SDK 输入节点/ROS 层。",
        )

    def _evaluate_input(self, now: float) -> None:
        age = self._input.age(now)
        running = now - self._started_at
        self._set_condition(
            "input_missing",
            running > 3.0 and age is None,
            "严重",
            "没有收到 /pico_body/status：PICO 输入节点可能未启动。",
        )
        self._set_condition(
            "input_stale",
            age is not None and age > 1.5,
            "严重",
            "PICO 状态话题已停止刷新，可能是节点退出或 ROS 通信中断。",
        )
        payload = self._input.value
        if not isinstance(payload, dict) or (age is not None and age > 1.5):
            return
        self._set_condition(
            "pico_source_bad",
            payload.get("source") != "live",
            "严重",
            f"PICO 手柄源异常：source={payload.get('source')!r}。",
        )
        self._set_condition(
            "smpl_source_bad",
            payload.get("smpl_source") not in HEALTHY_SMPL_STATES
            or payload.get("smpl_used") is not True,
            "严重",
            "SMPL Body 不可用或时间戳停止，可能触发 signal_lost。",
        )
        error = payload.get("error")
        self._set_condition(
            "input_error",
            error is not None,
            "严重",
            f"PICO/SMPL 读取或映射报错：{error}",
        )

    def _evaluate_sim(self, now: float) -> None:
        age = self._sim.age(now)
        running = now - self._started_at
        self._set_condition(
            "sim_missing",
            running > 3.0 and age is None,
            "严重",
            "没有收到 Pinocchio 状态，请确认 pixi run sim 正在运行。",
        )
        self._set_condition(
            "sim_stale",
            age is not None and age > 1.5,
            "严重",
            "Pinocchio 状态停止刷新。",
        )

        for side in SIDES:
            command_age = self._commands[side].age(now)
            self._set_condition(
                f"{side}_command_missing",
                running > 3.0 and command_age is None,
                "严重",
                f"没有收到{SIDE_LABEL[side]}关节命令。",
            )
            self._set_condition(
                f"{side}_command_stale",
                command_age is not None and command_age > 0.5,
                "严重",
                f"{SIDE_LABEL[side]}关节命令停止刷新。",
            )

            target_age = self._targets[side].age(now)
            target_expected = self._state == "teleop"
            self._set_condition(
                f"{side}_target_stale",
                target_expected
                and (target_age is None or target_age > 0.5),
                "严重",
                f"teleop 中{SIDE_LABEL[side]}目标位姿没有持续发布。",
            )

        payload = self._sim.value
        if not isinstance(payload, dict) or (age is not None and age > 1.5):
            return
        for side in SIDES:
            saturated = payload.get(f"{side}_target_saturated") is True
            singular = payload.get(f"{side}_singularity_active") is True
            limit_margin = _finite_float(
                payload.get(f"{side}_min_limit_margin_deg")
            )
            position_error = _finite_float(
                payload.get(f"{side}_position_error_mm")
            )
            self._set_condition(
                f"{side}_ik_saturated",
                saturated,
                "严重",
                f"{SIDE_LABEL[side]} IK 目标不可达/停滞，已保持安全边界。",
            )
            self._set_condition(
                f"{side}_ik_singular",
                singular,
                "警告",
                f"{SIDE_LABEL[side]}进入奇异区，IK 会增加阻尼并可能变慢。",
            )
            self._set_condition(
                f"{side}_ik_limit",
                limit_margin is not None and limit_margin <= 1.0,
                "警告",
                f"{SIDE_LABEL[side]}接近 IK 安全关节限位。",
            )
            self._set_condition(
                f"{side}_ik_error_large",
                position_error is not None and position_error >= 30.0,
                "警告",
                f"{SIDE_LABEL[side]}末端位置误差超过 30mm。",
            )

    def _evaluate_motion(self, now: float) -> None:
        for side in SIDES:
            target_velocity = self._targets[side].vertical_velocity(now)
            solved_velocity = self._solved[side].vertical_velocity(now)
            descending = (
                self._state == "teleop"
                and target_velocity is not None
                and target_velocity < -0.03
            )
            if descending:
                self._down_observed[side] = True
            ratio = None
            if descending and solved_velocity is not None:
                ratio = max(0.0, -solved_velocity) / max(
                    1.0e-6, -target_velocity
                )
            slow = descending and ratio is not None and ratio < 0.35
            if slow:
                self._down_slow_ticks[side] += 1
            elif descending and ratio is not None:
                self._down_good_ticks[side] += 1
            self._set_condition(
                f"{side}_down_slow",
                slow,
                "警告",
                f"{SIDE_LABEL[side]}目标正在下降，但 Pinocchio 末端下降明显偏慢。",
            )

    def _evaluate_real(self, now: float) -> None:
        age = self._real.age(now)
        real_fresh = age is not None and age <= 1.5
        payload = self._real.value
        if not real_fresh or not isinstance(payload, dict):
            # 真机桥没有运行是合法的纯仿真模式，不作为故障。
            for key in (
                "real_error",
                "real_soft_stop",
                "real_tracking_lead",
                "real_output_limit",
            ):
                self._clear_issue(key)
            for side in SIDES:
                self._clear_issue(f"{side}_feedback_stale")
                self._clear_issue(f"{side}_tracking_error")
            return

        error = payload.get("error")
        phase = str(payload.get("phase", "unknown"))
        action = str(payload.get("last_action", "none"))
        connected = payload.get("robot_connected") is True
        self._set_condition(
            "real_error",
            error is not None,
            "严重",
            f"真机桥报告错误：{error}",
        )
        self._set_condition(
            "real_soft_stop",
            phase in {"soft_stopped", "failed"},
            "严重",
            f"真机桥当前阶段为 {phase}。",
        )
        self._set_condition(
            "real_tracking_lead",
            "tracking_lead_limited" in action,
            "警告",
            "真机反馈跟不上命令，安全桥正在限制命令领先量。",
        )
        self._set_condition(
            "real_output_limit",
            "output_step_limited" in action,
            "警告",
            "真机桥输出斜坡达到当前速度上限。",
        )

        for side in SIDES:
            feedback_age = self._feedback[side].age(now)
            self._set_condition(
                f"{side}_feedback_stale",
                connected and (feedback_age is None or feedback_age > 0.5),
                "严重",
                f"真机已连接，但{SIDE_LABEL[side]}反馈没有持续刷新。",
            )
            command = self._commands[side].value
            feedback = self._feedback[side].value
            if not (
                isinstance(command, tuple)
                and isinstance(feedback, tuple)
                and len(command) >= 7
                and len(feedback) >= 7
                and self._commands[side].age(now) is not None
                and self._commands[side].age(now) <= 0.5
                and feedback_age is not None
                and feedback_age <= 0.5
            ):
                continue
            differences = [
                abs(command[index] - feedback[index]) for index in range(7)
            ]
            maximum = max(differences)
            joint = differences.index(maximum) + 1
            if maximum > self._maximum_tracking_error[side]:
                self._maximum_tracking_error[side] = maximum
                self._maximum_tracking_joint[side] = joint
            self._set_condition(
                f"{side}_tracking_error",
                maximum >= 4.0,
                "警告",
                f"{SIDE_LABEL[side]} J{joint} 命令与实测相差 {maximum:.1f}°。",
            )

    def _set_condition(
        self,
        key: str,
        active: bool,
        level: str,
        message: str,
    ) -> None:
        if active:
            self._activate_issue(key, level, message)
        else:
            self._clear_issue(key)

    def _render(self, now: float) -> None:
        # 限制刷新频率，避免终端闪烁和浪费 CPU。
        if now - self._last_render_at < 0.45:
            return
        self._last_render_at = now
        interactive = sys.stdout.isatty() and os.environ.get("TERM") != "dumb"
        if interactive:
            sys.stdout.write("\033[2J\033[H")

        elapsed = now - self._started_at
        real_age = self._real.age(now)
        real_active = real_age is not None and real_age <= 1.5
        mode = "SIM + REAL（只读监测）" if real_active else "SIM（只读监测）"
        input_payload = self._input.value if isinstance(self._input.value, dict) else {}
        sim_payload = self._sim.value if isinstance(self._sim.value, dict) else {}
        real_payload = self._real.value if isinstance(self._real.value, dict) else {}
        native = self._host_monitor.snapshot(now)
        native["pico_transport_live_baseline_bytes_s"] = _median(
            self._pico_transport_live_rates
        )
        layer_code, layer_level, layer_message = self._layer_diagnosis(
            now, native
        )

        if not native["available"]:
            service_text = "监测不可用"
        elif native["service_running"] and not native["service_fd_accessible"]:
            service_text = "进程在/套接字权限不可读"
        elif native["service_running"] and native["grpc_listening"]:
            service_text = "进程和60061正常"
        elif native["service_running"]:
            service_text = "进程在/60061未监听"
        else:
            service_text = "进程未运行"
        if native["pico_peers"]:
            pico_peer_text = ",".join(native["pico_peers"])
        elif native["ever_pico_peer"]:
            pico_peer_text = "之前有，现已断开"
        else:
            pico_peer_text = "未识别"

        lines = [
            "=" * 78,
            "PICO / Pinocchio / Marvin 实时诊断（本程序不会发布或控制）",
            f"模式：{mode}    已运行：{elapsed:6.1f}s    Ctrl+C：结束并输出总结",
            f"日志：{self._log_path}",
            "人工标记：p=断流后必须重按A    m=回落/返回明显变慢（直接按，不用回车）",
            "=" * 78,
            (
                "输入："
                f"state={self._state or '未知':<9} "
                f"PICO={str(input_payload.get('source', '未收到')):<18} "
                f"SMPL={str(input_payload.get('smpl_source', '未收到')):<24} "
                f"A={'按下' if input_payload.get('right_a_pressed') else '松开'}"
            ),
            (
                "状态话题："
                f"PICO {_format_age(self._input.age(now))} / "
                f"IK {_format_age(self._sim.age(now))} / "
                f"REAL {_format_age(real_age)}"
            ),
            "-" * 78,
            "三段分层监测：",
            (
                "  [1] PICO→MiniPC："
                f"TCP对端={pico_peer_text}  "
                f"PICO连接收包={_format_byte_rate(native['pico_transport_receive_bytes_s'])}  "
                f"正常基线={_format_byte_rate(native['pico_transport_live_baseline_bytes_s'])}  "
                f"最近事件={native['last_peer_event']}"
            ),
            (
                "  [2] PC Service→SDK："
                f"Service={service_text}  "
                f"PID={','.join(str(pid) for pid in native['service_pids']) or '--'}  "
                f"本地SDK连接={native['sdk_connection_count']}"
            ),
            (
                "  [3] Python SDK→ROS："
                f"时间戳={_format_rate(self._ros_controller_timestamp.rate(now))}  "
                f"时间戳龄={_format_age(self._ros_controller_timestamp.change_age(now))}"
            ),
            f"  分层结论 [{layer_level}/{layer_code}]：{layer_message}",
            "-" * 78,
        ]

        if self._last_marker is not None and now <= self._marker_banner_until:
            marker = self._last_marker
            lines.extend(
                [
                    "#" * 78,
                    "### 已记录人工标签："
                    f"[{marker.definition.code}] {marker.definition.title}",
                    f"### 按键={marker.definition.key}  "
                    f"时间={marker.wall_time}  运行={marker.elapsed_s:.3f}s",
                    "#" * 78,
                ]
            )

        for side in SIDES:
            target_velocity = self._targets[side].vertical_velocity(now)
            solved_velocity = self._solved[side].vertical_velocity(now)
            position_error = _finite_float(
                sim_payload.get(f"{side}_position_error_mm")
            )
            limit_margin = _finite_float(
                sim_payload.get(f"{side}_min_limit_margin_deg")
            )
            flags = []
            if sim_payload.get(f"{side}_target_saturated") is True:
                flags.append("目标不可达")
            if sim_payload.get(f"{side}_singularity_active") is True:
                flags.append("奇异区")
            if sim_payload.get(f"{side}_joint_step_limited") is True:
                flags.append("IK步长限制")
            lines.append(
                f"{SIDE_LABEL[side]}：目标竖速 {_format_velocity(target_velocity)}  "
                f"IK竖速 {_format_velocity(solved_velocity)}  "
                f"目标 {_format_rate(self._targets[side].rate(now))}  "
                f"IK {_format_rate(self._solved[side].rate(now))}"
            )
            lines.append(
                "      "
                f"位置误差={'--' if position_error is None else f'{position_error:.1f}mm':>8}  "
                f"限位余量={'--' if limit_margin is None else f'{limit_margin:.1f}°':>7}  "
                f"状态={','.join(flags) if flags else ('正常' if sim_payload else '无数据')}"
            )

        lines.append("-" * 78)
        timing = self._command_timing(now)

        def timing_value(name: str) -> str:
            value = _finite_float(timing.get(name))
            return "--" if value is None else f"{value:.2f}ms"

        lines.append(
            "指令时序（检测进程观测）："
            f"接收差={timing_value('receive_pair_skew_ms')}  "
            f"原始时间戳差={timing_value('source_stamp_skew_ms')}  "
            f"左龄={timing_value('left_age_ms')}  "
            f"右龄={timing_value('right_age_ms')}  "
            f"阈值={timing_value('pair_skew_limit_ms')}"
        )
        if real_active:
            lines.append(
                "真机桥："
                f"phase={real_payload.get('phase', '未知')}  "
                f"action={real_payload.get('last_action', '未知')}  "
                f"connected={real_payload.get('robot_connected', False)}  "
                f"软件速度上限={real_payload.get('maximum_output_speed_deg_s', '--')}°/s"
            )
            for side in SIDES:
                command = self._commands[side].value
                feedback = self._feedback[side].value
                difference_text = "--"
                if (
                    isinstance(command, tuple)
                    and isinstance(feedback, tuple)
                    and len(command) >= 7
                    and len(feedback) >= 7
                ):
                    diffs = [abs(command[i] - feedback[i]) for i in range(7)]
                    maximum = max(diffs)
                    difference_text = f"J{diffs.index(maximum) + 1} {maximum:.2f}°"
                lines.append(
                    f"  {SIDE_LABEL[side]}：命令 {_format_rate(self._commands[side].rate(now))}  "
                    f"反馈 {_format_rate(self._feedback[side].rate(now))}  "
                    f"最大命令-实测差={difference_text}"
                )
        else:
            lines.append("真机桥：未发现（这是正常的纯仿真检测状态）")

        lines.extend(["-" * 78, "当前判断："])
        if self._issues:
            severity_order = {"严重": 0, "警告": 1, "信息": 2}
            issues = sorted(
                self._issues.values(),
                key=lambda item: severity_order.get(item[0], 9),
            )
            for level, message in issues[:8]:
                lines.append(f"  [{level}] {message}")
            if len(issues) > 8:
                lines.append(f"  ……另有 {len(issues) - 8} 项")
        else:
            lines.append("  [正常] 当前没有检测到持续断流、IK 卡住或真机明显滞后。")

        if self._markers:
            marker_counts = Counter(
                marker.definition.key for marker in self._markers
            )
            count_text = "，".join(
                f"{definition.key}={marker_counts[definition.key]}次"
                for definition in HOTKEY_DEFINITIONS
            )
            last_marker = self._markers[-1]
            lines.append(
                "人工标记统计："
                f"{count_text}；最近=[{last_marker.definition.code}] "
                f"{last_marker.elapsed_s:.1f}s"
            )
        else:
            lines.append("人工标记统计：暂无（遇到现象时按 p 或 m）")

        lines.append("最近事件：")
        if not self._events:
            lines.append("  暂无")
        else:
            for stamp, level, message in list(self._events)[-6:]:
                lines.append(f"  {now - stamp:5.1f}s 前 [{level}] {message}")
        lines.append("=" * 78)
        rendered = "\n".join(lines)
        print(rendered, flush=True)
        self._write_log(
            f"\n[{self._wall_time()}] SNAPSHOT "
            f"运行 {elapsed:.3f}s\n{rendered}\n"
        )

    def print_summary(self) -> None:
        """在 Ctrl+C 时根据整段观测给出基本结论。"""
        elapsed = time.monotonic() - self._started_at
        conclusions = []
        signal_returns = self._return_reasons["signal_lost"]
        a_returns = self._return_reasons["right_a"]
        uncertain_returns = self._return_reasons["uncertain"]

        required_topic_failures = sum(
            self._issue_activations[key]
            for key in (
                "input_missing",
                "input_stale",
                "sim_missing",
                "sim_stale",
                "left_command_missing",
                "right_command_missing",
            )
        )
        if required_topic_failures:
            conclusions.append(
                "检测期间必需的 PICO、Pinocchio 或关节命令话题曾缺失/停止，"
                "该时段无法完整判断运动链路。"
            )
        if self._states_seen and "teleop" not in self._states_seen:
            conclusions.append(
                "检测期间没有进入 teleop；需要按 A 成功启动遥操作后，"
                "脚本才能分析抬手和放手。"
            )

        if signal_returns:
            conclusions.append(
                f"检测到 {signal_returns} 次退出与 PICO/SMPL 异常同时发生，"
                "首要怀疑输入断流或 Body 追踪丢失。"
            )
        if a_returns:
            conclusions.append(
                f"检测到 {a_returns} 次退出时 A 键为按下状态，基本判断为 A 键触发。"
            )
        if uncertain_returns:
            conclusions.append(
                f"有 {uncertain_returns} 次退出发生得太短，现有话题无法区分瞬时丢帧和短 A 脉冲。"
            )

        marker_counts = Counter(
            marker.definition.key for marker in self._markers
        )
        if marker_counts["p"]:
            conclusions.append(
                f"操作者人工标记了 {marker_counts['p']} 次“断流后必须重按 A”；"
                "应重点对齐标记前后的 PICO/SMPL 状态和 teleop 状态切换。"
            )
        if marker_counts["m"]:
            conclusions.append(
                f"操作者人工标记了 {marker_counts['m']} 次“回落/返回明显变慢”；"
                "应重点对齐标记时的目标/IK 竖速、IK 状态和真机跟踪误差。"
            )

        segment1_count = sum(
            self._layer_diagnosis_counts[key]
            for key in (
                "segment1_peer_disconnected",
                "segment1_no_pico_peer",
                "segment1_transport_stopped",
            )
        )
        if segment1_count:
            conclusions.append(
                "SDK 数据异常时，未观察到 PICO→PC Service 非回环 TCP "
                "连接；第 1 段（PICO App/Wi-Fi）为首要怀疑。"
            )
        segment2_count = sum(
            self._layer_diagnosis_counts[key]
            for key in (
                "segment2_service_down",
                "segment2_endpoint_down",
                "segment2_sdk_not_connected",
                "segment2_forwarding_stale",
            )
        )
        if segment2_count:
            conclusions.append(
                "PC Service 进程、60061 端口或本地 SDK 连接曾缺失；"
                "该次异常位于第 2 段。"
            )
        boundary_count = self._layer_diagnosis_counts[
            "boundary12_sdk_stale"
        ]
        if boundary_count:
            conclusions.append(
                "PICO TCP 和本地 SDK 连接仍存在，但 SDK 姿态时间戳停止；"
                "问题在第 1/2 段边界。被动监测不能将“只剩心跳”与"
                "“PC Service 未转发姿态帧”强行二分。"
            )
        segment3_count = sum(
            self._layer_diagnosis_counts[key]
            for key in (
                "segment3_ros_missing",
                "segment3_body_stale",
            )
        )
        if segment3_count:
            conclusions.append(
                "本地 SDK 连接存在，但 Python 输入节点/ROS 状态缺失或"
                "Body 支路异常；问题已缩小到第 3 段。"
            )

        ik_count = sum(
            count
            for key, count in self._issue_activations.items()
            if key.endswith("_ik_saturated")
            or key.endswith("_ik_singular")
            or key.endswith("_ik_limit")
        )
        if ik_count:
            conclusions.append(
                f"观察到 {ik_count} 次 IK 限位、奇异或目标停滞事件；"
                "如果放手时出现，应优先检查机器人工作空间和手柄姿态。"
            )

        for side in SIDES:
            if not self._targets[side].times:
                conclusions.append(
                    f"没有收到{SIDE_LABEL[side]}目标位姿，无法判断该侧下降跟踪。"
                )
            elif self._down_observed[side]:
                slow = self._down_slow_ticks[side]
                good = self._down_good_ticks[side]
                if slow > good and slow >= 2:
                    conclusions.append(
                        f"{SIDE_LABEL[side]}下降期间 Pinocchio 跟踪偏慢样本较多，"
                        "问题已出现在仿真/IK 层。"
                    )
                elif good:
                    conclusions.append(
                        f"{SIDE_LABEL[side]}观察到下降动作，Pinocchio 大部分时间能够跟随目标。"
                    )
            else:
                conclusions.append(
                    f"未观察到{SIDE_LABEL[side]}目标明显下降；若你确实做了放手动作，"
                    "应怀疑 SMPL 胸廓漂移或输入映射抵消了下降量。"
                )

        hardware_limit_count = sum(
            self._issue_activations[key]
            for key in ("real_tracking_lead", "real_output_limit")
        )
        tracking_count = sum(
            count
            for key, count in self._issue_activations.items()
            if key.endswith("_tracking_error")
        )
        if hardware_limit_count or tracking_count:
            conclusions.append(
                "真机桥检测到输出限速或命令-反馈滞后；若仿真正常而真机慢，"
                "问题更可能在安全桥、控制器或实体关节。"
            )

        if not conclusions:
            conclusions.append(
                "本次没有观察到足够的异常证据；建议再次执行多次抬手和放手，"
                "并让脚本覆盖问题发生时刻。"
            )

        summary_lines = [
            "=" * 78,
            f"检测总结（持续 {elapsed:.1f}s，本程序全程只读）",
            "-" * 78,
        ]
        for index, conclusion in enumerate(conclusions, start=1):
            summary_lines.append(f"{index}. {conclusion}")
        for side in SIDES:
            maximum = self._maximum_tracking_error[side]
            joint = self._maximum_tracking_joint[side]
            if joint is not None:
                summary_lines.append(
                    f"- {SIDE_LABEL[side]}本次最大命令-实测差："
                    f"J{joint} {maximum:.2f}°"
                )
        if self._markers:
            summary_lines.extend(["-" * 78, "人工标记索引："])
            for index, marker in enumerate(self._markers, start=1):
                summary_lines.append(
                    f"  {index}. key={marker.definition.key} "
                    f"code={marker.definition.code} "
                    f"运行={marker.elapsed_s:.3f}s "
                    f"时间={marker.wall_time}"
                )
        summary_lines.extend(
            [
                f"日志文件：{self._log_path}",
                "=" * 78,
            ]
        )
        summary = "\n".join(summary_lines)
        print("\n" + summary, flush=True)
        self._write_log(
            f"\n[{self._wall_time()}] FINAL SUMMARY\n{summary}\n"
        )

    def close_external_monitors(self) -> None:
        """关闭外部监测器；当前纯被动实现实际上无会话需关闭。"""
        self._host_monitor.close()


def main() -> int:
    rclpy.init()
    node = TeleopDiagnostic()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        try:
            node.restore_terminal()
            node.print_summary()
        finally:
            try:
                node.close_external_monitors()
            finally:
                node.close_log()
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
