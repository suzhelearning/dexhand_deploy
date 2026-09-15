"""Python cold-path adapter for the opt-in native session gateway.

The C++ gateway owns the fixed-rate scheduler, TJVR ingress, native IK worker,
MuJoCo executor and reset state machine.  This module only owns process
startup, a fixed binary control channel, and asynchronous side effects such as
optional Python publication, recording and rendering. Native publication is
explicitly negotiated before disabling Python output. It has no device
SDK imports and is never used by the default Python scheduler.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
import stat
import fcntl
from pathlib import Path
import queue
import select
import signal
import socket
import struct
import subprocess
import threading
import time
from typing import Any, Mapping


COMMAND_STRUCT = struct.Struct("<4sBBHQq")
FRAME_HEADER_STRUCT = struct.Struct("<4sBBHIQq")
CYCLE_PREFIX_STRUCT = struct.Struct("<qqQQQQqQQBBBB")
MAX_REASON = 4096
MAX_PACKET = 656
MAX_RESULT_WIRE = 1206
MAX_FRAME_PAYLOAD = 8 * 1024 * 1024

_ACTION_CODES = {
    "start": 1,
    "return": 2,
    "shutdown": 3,
    "rearm": 4,
    "calibrate": 5,
}
_FRAME_KINDS = {
    1: "reply",
    2: "receipt",
    3: "cycle",
    4: "complete",
    5: "failure",
    6: "raw",
    7: "summary",
}
_STATE_CODES = {1: "idle", 2: "teleop", 3: "returning", 4: "fault"}


class GatewayWireError(ValueError):
    """Malformed or incompatible native gateway wire data."""


class _Cursor:
    def __init__(self, payload: bytes) -> None:
        self._payload = memoryview(payload)
        self.offset = 0

    def take(self, size: int) -> bytes:
        if type(size) is not int or size < 0 or size > len(self._payload) - self.offset:
            raise GatewayWireError("truncated native gateway payload")
        start = self.offset
        self.offset += size
        return self._payload[start:self.offset].tobytes()

    def unpack(self, fmt: struct.Struct) -> tuple[Any, ...]:
        return fmt.unpack(self.take(fmt.size))

    def text(self, maximum: int = MAX_REASON) -> str:
        (size,) = self.unpack(struct.Struct("<I"))
        if size > maximum:
            raise GatewayWireError("oversized native gateway reason")
        value = self.take(size)
        if b"\0" in value:
            raise GatewayWireError("NUL in native gateway reason")
        try:
            return value.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise GatewayWireError("native gateway reason is not UTF-8") from exc

    def done(self) -> bool:
        return self.offset == len(self._payload)


def _finite_vector(cursor: _Cursor, count: int) -> list[float]:
    values = list(cursor.unpack(struct.Struct("<" + "d" * count)))
    if not all(math.isfinite(value) for value in values):
        raise GatewayWireError("non-finite native gateway value")
    return values


def _validate_reserved(value: bytes, message: str) -> None:
    if any(value):
        raise GatewayWireError(message)


def _decode_result_wire(data: bytes, prefix: str) -> dict[str, Any]:
    if prefix not in ("spark", "mapped_palm"):
        raise GatewayWireError("unsupported native gateway worker prefix")
    if not data:
        raise GatewayWireError("native gateway result wire is empty")
    try:
        from ...hand_tracking.native_binary_results import decode_result

        return decode_result(data, prefix)
    except (ValueError, TypeError, KeyError, struct.error) as exc:
        raise GatewayWireError(f"invalid native gateway result wire: {exc}") from exc


def decode_cycle_payload(payload: bytes, *, prefix: str) -> dict[str, Any]:
    """Decode one ``TJSO`` cycle payload without accepting trailing bytes."""
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise TypeError("native gateway cycle payload must be bytes-like")
    cursor = _Cursor(bytes(payload))
    (timestamp_ns, source_received_ns, source_revision, source_epoch,
     source_sequence, source_generation, state_epoch, ticks, late_ticks,
     state_code, flags, reset_pending, capture_failed) = cursor.unpack(CYCLE_PREFIX_STRUCT)
    if state_code not in _STATE_CODES:
        raise GatewayWireError("unknown native gateway session state")
    if any(value not in (0, 1) for value in (reset_pending, capture_failed)):
        raise GatewayWireError("invalid native gateway boolean")
    command_values = _finite_vector(cursor, 14)
    feedback_values = _finite_vector(cursor, 14)

    command = ({"left": command_values[:7], "right": command_values[7:]}
               if flags & (1 << 0) else None)
    feedback = ({"left": feedback_values[:7], "right": feedback_values[7:]}
                if flags & (1 << 1) else None)

    (request_id,) = cursor.unpack(struct.Struct("<Q"))
    (request_now_ns,) = cursor.unpack(struct.Struct("<q"))
    (request_received_ns,) = cursor.unpack(struct.Struct("<q"))
    (request_generation,) = cursor.unpack(struct.Struct("<Q"))
    (request_source_sequence,) = cursor.unpack(struct.Struct("<Q"))
    (discontinuity,) = cursor.unpack(struct.Struct("<B"))
    _validate_reserved(cursor.take(7), "native gateway request padding is nonzero")
    (packet_size,) = cursor.unpack(struct.Struct("<I"))
    if packet_size > MAX_PACKET:
        raise GatewayWireError("oversized native gateway request packet")
    packet = cursor.take(MAX_PACKET)
    if any(packet[packet_size:]):
        raise GatewayWireError("native gateway request padding is nonzero")
    if discontinuity not in (0, 1):
        raise GatewayWireError("invalid native gateway discontinuity")
    request = None
    if flags & (1 << 2):
        if request_id == 0 or request_now_ns <= 0:
            raise GatewayWireError("invalid native gateway request metadata")
        # The fixed-rate IK clock also ticks between input packets. WorkerTick
        # uses an empty packet and zero source metadata for that normal case.
        if packet_size == 0:
            if any((request_received_ns, request_generation, request_source_sequence, discontinuity)):
                raise GatewayWireError("native gateway empty request contains source metadata")
        elif request_received_ns <= 0:
            raise GatewayWireError("invalid native gateway request receive time")
        if packet_size and request_received_ns > request_now_ns:
            raise GatewayWireError("native gateway request receive time is in the future")
        request = {
            "id": request_id,
            "now_ns": request_now_ns,
            "received_ns": request_received_ns,
            "generation": request_generation,
            "source_sequence": request_source_sequence,
            "discontinuity": bool(discontinuity),
            "packet": packet[:packet_size],
        }
    elif any((request_id, request_now_ns, request_received_ns, request_generation,
              request_source_sequence,
              discontinuity, packet_size)) or any(packet):
        raise GatewayWireError("native gateway absent request contains data")

    (result_tick,) = cursor.unpack(struct.Struct("<Q"))
    (result_timestamp_ns,) = cursor.unpack(struct.Struct("<Q"))
    (result_epoch,) = cursor.unpack(struct.Struct("<Q"))
    (result_sequence,) = cursor.unpack(struct.Struct("<Q"))
    (result_size,) = cursor.unpack(struct.Struct("<I"))
    if result_size > MAX_RESULT_WIRE:
        raise GatewayWireError("oversized native gateway result wire")
    result_wire = cursor.take(MAX_RESULT_WIRE)
    if any(result_wire[result_size:]):
        raise GatewayWireError("native gateway result padding is nonzero")
    result = None
    if flags & (1 << 3):
        if not result_tick or not result_timestamp_ns or not result_size:
            raise GatewayWireError("invalid native gateway result metadata")
        result = _decode_result_wire(result_wire[:result_size], prefix)
    elif any((result_tick, result_timestamp_ns, result_epoch, result_sequence, result_size)) or any(result_wire):
        raise GatewayWireError("native gateway absent result contains data")

    reason = cursor.text()
    if not cursor.done():
        raise GatewayWireError("trailing native gateway cycle bytes")
    return {
        "timestamp_ns": timestamp_ns,
        "source": {
            "received_ns": source_received_ns,
            "revision": source_revision,
            "epoch": source_epoch,
            "sequence": source_sequence,
            "generation": source_generation,
            "accepted": bool(flags & (1 << 5)),
            "skeleton_valid": bool(flags & (1 << 6)),
            "rotations_valid": bool(flags & (1 << 7)),
        },
        "state": _STATE_CODES[state_code],
        "state_epoch": state_epoch,
        "ticks": ticks,
        "late_ticks": late_ticks,
        "reset_pending": bool(reset_pending),
        "capture_failed": bool(capture_failed),
        "command": command,
        "feedback": feedback,
        "request": request,
        "result": result,
        "ik_adopted": bool(flags & (1 << 4)),
        "reason": reason,
    }


def _decode_text_payload(payload: bytes) -> str:
    cursor = _Cursor(payload)
    result = cursor.text()
    if not cursor.done():
        raise GatewayWireError("trailing native gateway text bytes")
    return result


def decode_gateway_frame(frame: bytes, *, prefix: str) -> "GatewayFrame":
    if not isinstance(frame, (bytes, bytearray)) or len(frame) < FRAME_HEADER_STRUCT.size:
        raise GatewayWireError("short native gateway frame")
    magic, version, kind_code, flags, payload_size, sequence, timestamp_ns = FRAME_HEADER_STRUCT.unpack_from(frame)
    if magic != b"TJSO" or version != 1 or flags != 0:
        raise GatewayWireError("invalid native gateway frame header")
    if payload_size > MAX_FRAME_PAYLOAD or len(frame) != FRAME_HEADER_STRUCT.size + payload_size:
        raise GatewayWireError("native gateway frame size mismatch")
    kind = _FRAME_KINDS.get(kind_code)
    if kind is None:
        raise GatewayWireError("unknown native gateway frame kind")
    payload = frame[FRAME_HEADER_STRUCT.size:]
    if kind == "summary":
        cursor = _Cursor(payload)
        epoch, ticks, late, cycles, state, reset, failed, reserved = cursor.unpack(struct.Struct('<qQQQBBBB'))
        if (epoch <= 0 or cycles <= 0 or state not in _STATE_CODES or reset not in (0,1)
                or failed not in (0,1) or reserved or timestamp_ns <= 0 or sequence != ticks):
            raise GatewayWireError('invalid native gateway summary')
        value = dict(state_epoch=epoch, ticks=ticks, late_ticks=late, cycles=cycles,
                     state=_STATE_CODES[state], reset_pending=bool(reset), capture_failed=bool(failed),
                     reason=cursor.text())
        if not cursor.done(): raise GatewayWireError('trailing native gateway summary bytes')
    elif kind == "cycle":
        value = decode_cycle_payload(payload, prefix=prefix)
    elif kind == "reply":
        cursor = _Cursor(payload)
        (reply_id,) = cursor.unpack(struct.Struct("<Q"))
        (accepted,) = cursor.unpack(struct.Struct("<B"))
        _validate_reserved(cursor.take(3), "native gateway reply padding is nonzero")
        if accepted not in (0, 1):
            raise GatewayWireError("invalid native gateway reply status")
        value = {"id": reply_id, "accepted": bool(accepted), "reason": cursor.text()}
        if not cursor.done():
            raise GatewayWireError("trailing native gateway reply bytes")
    elif kind == "receipt":
        cursor = _Cursor(payload)
        (tick,) = cursor.unpack(struct.Struct("<Q"))
        (epoch,) = cursor.unpack(struct.Struct("<q"))
        (receipt_timestamp_ns,) = cursor.unpack(struct.Struct("<q"))
        (accepted,) = cursor.unpack(struct.Struct("<B"))
        _validate_reserved(cursor.take(3), "native gateway receipt padding is nonzero")
        if accepted not in (0, 1):
            raise GatewayWireError("invalid native gateway receipt status")
        positions = _finite_vector(cursor, 14)
        value = {
            "tick": tick,
            "epoch": epoch,
            "timestamp_ns": receipt_timestamp_ns,
            "accepted": bool(accepted),
            "positions": {"left": positions[:7], "right": positions[7:]},
            "run": cursor.text(),
            "router": cursor.text(),
            "reason": cursor.text(),
        }
        if not cursor.done():
            raise GatewayWireError("trailing native gateway receipt bytes")
    elif kind == "complete":
        cursor = _Cursor(payload)
        (complete,) = cursor.unpack(struct.Struct("<B"))
        (state_code,) = cursor.unpack(struct.Struct("<B"))
        _validate_reserved(cursor.take(2), "native gateway complete padding is nonzero")
        (epoch,) = cursor.unpack(struct.Struct("<q"))
        if complete not in (0, 1) or state_code not in _STATE_CODES:
            raise GatewayWireError("invalid native gateway completion")
        value = {"complete": bool(complete), "state": _STATE_CODES[state_code],
                 "epoch": epoch, "reason": cursor.text()}
        if not cursor.done():
            raise GatewayWireError("trailing native gateway completion bytes")
    elif kind == "raw":
        cursor = _Cursor(payload)
        raw_sequence, received_ns, accepted, packet_size = cursor.unpack(
            struct.Struct("<QqB3xI"))
        if raw_sequence <= 0 or received_ns <= 0 or accepted not in (0, 1):
            raise GatewayWireError("invalid native gateway raw metadata")
        if packet_size <= 0 or packet_size > MAX_PACKET:
            raise GatewayWireError("invalid native gateway raw packet size")
        packet = cursor.take(MAX_PACKET)
        if any(packet[packet_size:]):
            raise GatewayWireError("native gateway raw packet padding is nonzero")
        if not cursor.done():
            raise GatewayWireError("trailing native gateway raw bytes")
        value = {"sequence": raw_sequence, "received_ns": received_ns,
                 "accepted": bool(accepted), "packet": packet[:packet_size]}
    else:
        value = {"reason": _decode_text_payload(payload)}
    return GatewayFrame(kind, sequence, timestamp_ns, value)


@dataclass(frozen=True)
class GatewayFrame:
    kind: str
    sequence: int
    timestamp_ns: int
    payload: dict[str, Any]


def encode_gateway_command(action: str, command_id: int, next_epoch: int = 0) -> bytes:
    if action not in _ACTION_CODES:
        raise ValueError("unsupported native gateway action")
    if type(command_id) is not int or not 0 < command_id < 2**64:
        raise ValueError("native gateway command id must be positive uint64")
    if type(next_epoch) is not int or not 0 <= next_epoch < 2**63:
        raise ValueError("native gateway next_epoch must be nonnegative int64")
    return COMMAND_STRUCT.pack(b"TJAC", 1, _ACTION_CODES[action], 0, command_id, next_epoch)


class NativeGatewayProcess:
    """Own one native gateway process and its fixed binary control stream."""

    def __init__(self, executable: Path, manifest: Path, *, prefix: str,
                 startup_timeout_s: float = 10.0, io_timeout_s: float = 1.0,
                 frame_capacity: int = 2048, publication_backend: str = 'python',
                 recording_fd: int | None = None, viewer_backend: str = 'python',
                 diagnostic_transport: str = 'full', manus_fd: int | None = None) -> None:
        self.executable = Path(executable).resolve()
        self.manifest = Path(manifest).resolve()
        if prefix not in ("spark", "mapped_palm"):
            raise ValueError("native gateway prefix must be spark or mapped_palm")
        if not self.executable.is_file() or not os.access(self.executable, os.X_OK):
            raise RuntimeError(f"native session gateway missing: {self.executable}")
        if not self.manifest.is_file():
            raise RuntimeError(f"native session manifest missing: {self.manifest}")
        if not math.isfinite(startup_timeout_s) or startup_timeout_s <= 0:
            raise ValueError("native gateway startup timeout must be positive")
        if not math.isfinite(io_timeout_s) or io_timeout_s <= 0:
            raise ValueError("native gateway I/O timeout must be positive")
        if type(frame_capacity) is not int or not 0 < frame_capacity <= 8192:
            raise ValueError("native gateway frame capacity must be in 1..8192")
        self.prefix = prefix
        if publication_backend not in ('python', 'cpp'):
            raise ValueError('publication backend must be python or cpp')
        self.publication_backend = publication_backend
        if recording_fd is not None:
            if type(recording_fd) is not int or recording_fd < 3:
                raise ValueError('recording fd must be an open descriptor >= 3')
            os.fstat(recording_fd)
        self.recording_fd = recording_fd
        if viewer_backend not in ('python','cpp'):
            raise ValueError('viewer backend must be python or cpp')
        self.viewer_backend=viewer_backend
        if diagnostic_transport not in ('full', 'summary'):
            raise ValueError('invalid diagnostic transport')
        self.diagnostic_transport = diagnostic_transport
        if manus_fd is not None:
            if (type(manus_fd) is not int or manus_fd<3 or manus_fd==recording_fd or
                    publication_backend!='cpp' or viewer_backend!='cpp' or diagnostic_transport!='summary'):
                raise ValueError('Manus fd requires distinct descriptor and native output consumers')
            if (not stat.S_ISFIFO(os.fstat(manus_fd).st_mode) or
                    fcntl.fcntl(manus_fd,fcntl.F_GETFL)&os.O_ACCMODE != os.O_RDONLY):
                raise ValueError('Manus stdout must be a read-only pipe')
        self.manus_fd=manus_fd
        self.startup_timeout_s = float(startup_timeout_s)
        self.io_timeout_s = float(io_timeout_s)
        self._frames: queue.Queue[GatewayFrame] = queue.Queue(maxsize=frame_capacity)
        self._stop = threading.Event()
        self._cycle_ready = threading.Event()
        self._complete = threading.Event()
        self._lock = threading.Lock()
        self._failure: str | None = None
        self._process: subprocess.Popen[bytes] | None = None
        self._socket: socket.socket | None = None
        self._reader: threading.Thread | None = None
        self._stderr_reader: threading.Thread | None = None
        self._stdout_reader: threading.Thread | None = None
        self._next_id = 1
        self._sent_shutdown = False
        self._stderr = bytearray()

    @property
    def failure(self) -> str | None:
        with self._lock:
            return self._failure

    @property
    def process(self) -> subprocess.Popen[bytes] | None:
        return self._process

    @property
    def stderr(self) -> str:
        with self._lock:
            return self._stderr.decode("utf-8", errors="replace")

    def _fail(self, message: str) -> None:
        with self._lock:
            if self._failure is None:
                self._failure = str(message)[:MAX_REASON]

    def start(self) -> None:
        if self._process is not None:
            raise RuntimeError("native gateway process already started")
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
        parent.settimeout(self.io_timeout_s)
        child_fd = child.fileno()
        try:
            process = subprocess.Popen(
                [str(self.executable), "--manifest", str(self.manifest),
                 "--control-fd", str(child_fd)],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                pass_fds=tuple(fd for fd in (child_fd,self.recording_fd,self.manus_fd) if fd is not None),
                close_fds=True,
                bufsize=0,
                # Keep the native scheduler out of the terminal/wrapper
                # process group.  Python must receive Ctrl-C/SIGTERM first so
                # it can request the same controlled Home transition as `q`.
                start_new_session=True,
            )
        except BaseException:
            parent.close()
            child.close()
            raise
        child.close()
        self._process = process
        self._socket = parent
        self._stderr_reader = threading.Thread(target=self._drain_stderr,
                                                name="native-gateway-stderr", daemon=True)
        self._stderr_reader.start()
        try:
            self._wait_ready()
            self._reader = threading.Thread(target=self._read_frames,
                                            name="native-gateway-frames", daemon=True)
            self._reader.start()
            self._stdout_reader = threading.Thread(target=self._drain_stdout,
                                                    name="native-gateway-stdout", daemon=True)
            self._stdout_reader.start()
        except BaseException:
            self.close(graceful=False)
            raise

    def _wait_ready(self) -> None:
        process = self._process
        if process is None or process.stdout is None:
            raise RuntimeError("native gateway stdout is unavailable")
        fd = process.stdout.fileno()
        deadline = time.monotonic() + self.startup_timeout_s
        data = bytearray()
        while b"\n" not in data:
            if process.poll() is not None:
                raise RuntimeError(self._startup_failure("native gateway exited before ready"))
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([fd], [], [], remaining)[0]:
                raise TimeoutError(self._startup_failure("native gateway startup handshake timed out"))
            chunk = os.read(fd, 256)
            if not chunk:
                raise RuntimeError(self._startup_failure("native gateway stdout closed before ready"))
            data.extend(chunk)
            if len(data) > 256:
                raise RuntimeError("oversized native gateway startup handshake")
        expected = (b"native_session_gateway_ready" +
                    (b" publication=cpp" if self.publication_backend == 'cpp' else b"") +
                    (b" recording=cpp" if self.recording_fd is not None else b"") +
                    (b" viewer=cpp" if self.viewer_backend == 'cpp' else b"") +
                    (b" diagnostics=summary" if self.diagnostic_transport == 'summary' else b"") +
                    (b" hands=cpp" if self.manus_fd is not None else b"") + b"\n")
        if data != expected:
            raise RuntimeError(f"invalid native gateway startup handshake: {data!r}")

    def _startup_failure(self, message: str) -> str:
        error = self.stderr.strip()
        return message if not error else f"{message}: {error}"

    def _drain_stderr(self) -> None:
        process = self._process
        if process is None or process.stderr is None:
            return
        try:
            while True:
                chunk = process.stderr.read(4096)
                if not chunk:
                    return
                with self._lock:
                    self._stderr.extend(chunk)
                    if len(self._stderr) > 64 * 1024:
                        del self._stderr[:-64 * 1024]
        except Exception as exc:
            self._fail(f"native gateway stderr reader failed: {exc}")

    def _drain_stdout(self) -> None:
        process = self._process
        if process is None or process.stdout is None:
            return
        try:
            extra = process.stdout.read()
            if extra:
                self._fail("native gateway emitted unsolicited stdout")
        except Exception as exc:
            self._fail(f"native gateway stdout reader failed: {exc}")

    def _read_frames(self) -> None:
        stream = self._socket
        if stream is None:
            return
        buffer = bytearray()
        try:
            while not self._stop.is_set():
                try:
                    chunk = stream.recv(64 * 1024)
                except socket.timeout:
                    continue
                except OSError as exc:
                    if not self._stop.is_set():
                        raise RuntimeError(f"native gateway frame socket failed: {exc}") from exc
                    return
                if not chunk:
                    if not self._stop.is_set() and not self._complete.is_set():
                        raise RuntimeError("native gateway control stream closed")
                    return
                buffer.extend(chunk)
                while True:
                    if len(buffer) < FRAME_HEADER_STRUCT.size:
                        break
                    header = FRAME_HEADER_STRUCT.unpack_from(buffer)
                    payload_size = header[4]
                    if payload_size > MAX_FRAME_PAYLOAD:
                        raise GatewayWireError("oversized native gateway frame payload")
                    total = FRAME_HEADER_STRUCT.size + payload_size
                    if len(buffer) < total:
                        break
                    raw = bytes(buffer[:total])
                    del buffer[:total]
                    frame = decode_gateway_frame(raw, prefix=self.prefix)
                    if ((self.diagnostic_transport == 'summary' and frame.kind in ('cycle','raw','receipt'))
                            or (self.diagnostic_transport == 'full' and frame.kind == 'summary')):
                        raise GatewayWireError('native gateway diagnostic transport mismatch')
                    try:
                        self._frames.put_nowait(frame)
                    except queue.Full as exc:
                        raise RuntimeError("native gateway frame queue overflow") from exc
                    if frame.kind in ("cycle", "summary"):
                        self._cycle_ready.set()
                    elif frame.kind == "complete":
                        self._complete.set()
        except Exception as exc:
            if not self._stop.is_set():
                self._fail(f"native gateway frame reader failed: {type(exc).__name__}: {exc}")
                self._complete.set()

    def _next_command_id(self) -> int:
        with self._lock:
            if self._next_id >= 2**64:
                raise OverflowError("native gateway command id exhausted")
            value = self._next_id
            self._next_id += 1
            return value

    def reserve_command_id(self) -> int:
        """Allocate an ID before registering a response handler and sending."""
        return self._next_command_id()

    def send_action(self, action: str, *, next_epoch: int = 0, command_id: int | None = None) -> int:
        if self._socket is None:
            raise RuntimeError("native gateway process is not started")
        if command_id is None:
            command_id = self._next_command_id()
        data = encode_gateway_command(action, command_id, next_epoch)
        with self._lock:
            if self._sent_shutdown and action != "shutdown":
                raise RuntimeError("native gateway shutdown already requested")
            if action == "shutdown":
                self._sent_shutdown = True
        try:
            self._socket.sendall(data)
        except OSError as exc:
            self._fail(f"native gateway command send failed: {exc}")
            raise
        return command_id

    def get_frame(self, timeout_s: float | None = None) -> GatewayFrame | None:
        try:
            return self._frames.get(timeout=timeout_s)
        except queue.Empty:
            failure = self.failure
            if failure:
                raise RuntimeError(failure)
            return None

    def wait_for_cycle(self, timeout_s: float) -> None:
        if not self._cycle_ready.wait(timeout_s):
            failure = self.failure
            if failure:
                raise RuntimeError(failure)
            raise TimeoutError("native gateway produced no cycle before timeout")

    def wait_for_complete(self, timeout_s: float) -> bool:
        if self._complete.wait(timeout_s):
            return True
        failure = self.failure
        if failure:
            raise RuntimeError(failure)
        return False

    def close(self, *, graceful: bool = False, timeout_s: float = 5.0) -> None:
        process = self._process
        if process is None:
            return
        if graceful and process.poll() is None and not self._sent_shutdown:
            try:
                self.send_action("shutdown")
            except (OSError, RuntimeError):
                pass
            self._complete.wait(timeout_s)
        self._stop.set()
        stream = self._socket
        if stream is not None:
            try:
                stream.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            stream.close()
            self._socket = None
        if process.poll() is None:
            try:
                process.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except (OSError, ProcessLookupError):
                    process.terminate()
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except (OSError, ProcessLookupError):
                        process.kill()
                    process.wait(timeout=2.0)
        for thread in (self._reader, self._stdout_reader, self._stderr_reader):
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=2.0)
        if process.stdout is not None:
            process.stdout.close()
        if process.stderr is not None:
            process.stderr.close()
        self._process = None

    def __enter__(self) -> "NativeGatewayProcess":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close(graceful=exc_type is None)


def build_native_gateway_manifest(root: Path, resolved: Mapping[str, Any], *, run_id: str,
                                  instance_id: str, router_zid: str, tjvr_bind: str,
                                  tjvr_port: int, native_hands: bool = False) -> dict[str, Any]:
    """Build the C++ manifest from the same canonical assets as Python live."""
    if not all(isinstance(value, str) and value and "/" not in value
               for value in (run_id, instance_id, router_zid)):
        raise ValueError("native gateway identities must be nonempty path-safe strings")
    config = resolved.get("config")
    if (type(native_hands) is not bool or not isinstance(config, Mapping) or
            (config.get('hands_enabled') is not False and not (native_hands and config.get('hands_enabled') is True))):
        raise ValueError("native scheduler manifest requires hands disabled")
    backend = config.get("ik_backend")
    from .backend_assets import bilateral_assets
    selected = bilateral_assets(Path(root), backend)
    prefix = "mapped_palm" if backend == "pico_ee_mapped_corrected_palm_velocity_qp" else "spark"
    resync_policy = resolved.get('spark_resync_policy', 'reference')
    if resync_policy not in ('reference', 'resume') or (resync_policy == 'resume' and prefix != 'spark'):
        raise ValueError('SPARK resync policy requires reference or SPARK-only resume')
    required_paths = [selected["worker"], selected["config"], selected["model"], selected["urdf"]]
    if any(not Path(path).is_file() for path in required_paths):
        missing = next(Path(path) for path in required_paths if not Path(path).is_file())
        raise RuntimeError(f"native gateway asset missing: {missing}")

    from ...config_loader import load_yaml
    from ...coordination.arm_command_coordinator import ArmCommandCoordinator
    from ...coordination.bilateral_robot import BilateralArmRobotConfig

    worker_config = load_yaml(selected["config"])
    controller = worker_config.get("controller")
    if not isinstance(controller, Mapping) or controller.get("initial_posture_enabled") is not True:
        raise ValueError("native scheduler requires explicit worker initial posture")
    from .factory import reference_robot_config
    home_config = Path(root) / 'src/tianji_teleop/config/robot/arm.yaml'
    robot = reference_robot_config(selected['config'], selected['urdf'], home_config, use_arm_home=True)
    coordinator = ArmCommandCoordinator._coordinator_config(
        Path(root) / "src/tianji_teleop/config/coordinator/arm_v131.yaml")
    height = bool(resolved.get("mapped_palm_height_calibration", False))
    if height and prefix != "mapped_palm":
        raise ValueError("height calibration requires mapped-palm native scheduler")
    producer_id = selected["producer_id"]
    authorities = {
        "source": {"logical": "tjvr", "instance": instance_id + "-source", "router": router_zid},
        "producer": {"logical": producer_id, "instance": instance_id + "-" + prefix, "router": router_zid},
        "coordinator": {"logical": "arm", "instance": instance_id + "-coord", "router": router_zid},
        "executor": {"logical": "mujoco", "instance": instance_id + "-sim", "router": router_zid},
    }
    health_age_ns = int(round(coordinator["state_timeout_s"] * 1e9))
    return {
        "schema_version": 1,
        "run_id": run_id,
        "router_zid": router_zid,
        "source_authority": authorities["source"],
        "producer_authority": authorities["producer"],
        "coordinator_authority": authorities["coordinator"],
        "executor_authority": authorities["executor"],
        "worker_prefix": prefix,
        "algorithm": backend,
        "model": str(Path(selected["model"]).resolve()),
        "worker_command": [str(Path(selected["worker"]).resolve()),
                           str(Path(selected["config"]).resolve()),
                           str(Path(selected["model"]).resolve()),
                           str(Path(selected["urdf"]).resolve()),
                           "--startup-handshake", "--binary-results", "--home-config", str(home_config.resolve())] +
                           (["--resume-same-epoch"] if resync_policy == 'resume' else []),
        "spark_resync_policy": resync_policy,
        "home": [list(robot.left_home_rad), list(robot.right_home_rad)],
        "lower": [list(robot.left_lower_limits_rad), list(robot.right_lower_limits_rad)],
        "upper": [list(robot.left_upper_limits_rad), list(robot.right_upper_limits_rad)],
        "maximum_step": coordinator["maximum_command_step_rad"],
        "time_window": coordinator["command_step_time_window_s"],
        "rate_hz": coordinator["rate_hz"],
        "proposal_timeout_s": coordinator["proposal_timeout_s"],
        "home_duration_s": coordinator["home_minimum_duration_s"],
        "home_speed_rad_s": coordinator["home_max_speed_rad_s"],
        "ingress_age_ns": int(round(coordinator["proposal_timeout_s"] * 1e9)),
        "command_step_clipping": bool(config.get("command_step_clipping", False)),
        "health_age_ns": health_age_ns,
        "period_ns": int(round(1e9 / coordinator["rate_hz"])),
        "freshness_ns": health_age_ns,
        "raw_age_ns": min(health_age_ns, 200_000_000),
        "worker_timeout_ms": 2000,
        "mapped_palm": prefix == "mapped_palm",
        "height_calibration": height,
        "xz_calibration": bool(resolved.get("mapped_palm_xz_calibration", False)),
        "common_x_reference": bool(resolved.get("mapped_palm_common_x_reference", False)),
        "max_position_jump_m": float(resolved["tjvr_stream_contract"]["max_position_jump_m"]),
        "max_orientation_jump_rad": float(resolved["tjvr_stream_contract"]["max_orientation_jump_rad"]),
        "deterministic_worker": False,
        "tjvr_bind": str(tjvr_bind),
        "tjvr_port": int(tjvr_port),
        "home_tolerance_rad": coordinator["home_tolerance_rad"],
    }


def write_native_gateway_manifest(directory: Path, manifest: Mapping[str, Any]) -> Path:
    """Write a private immutable manifest; callers remove it after startup."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / ("native_gateway_" + str(os.getpid()) + "_" + str(time.time_ns()) + ".json")
    payload = json.dumps(dict(manifest), allow_nan=False, ensure_ascii=False,
                         separators=(",", ":")).encode("utf-8")
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    fd = os.open(path, flags, 0o600)
    try:
        offset = 0
        while offset < len(payload):
            offset += os.write(fd, payload[offset:])
    finally:
        os.close(fd)
    return path


__all__ = [
    "GatewayFrame",
    "GatewayWireError",
    "NativeGatewayProcess",
    "build_native_gateway_manifest",
    "decode_cycle_payload",
    "decode_gateway_frame",
    "encode_gateway_command",
    "write_native_gateway_manifest",
]
