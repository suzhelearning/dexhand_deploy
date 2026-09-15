"""Optional C++ fixed-rate scheduler bridge for the official Wuji Hand2 path.

This module is a cold-path process adapter. The driver/parser still delivers a
validated canonical 21-point frame, while the child process owns the fixed
rate admission, native geometry/optimizer/filter pipeline, freshness state and
bounded output queue. The Python reference worker and the default Python hand
loop remain untouched unless the caller explicitly selects this backend.
"""

from __future__ import annotations

import hashlib
import math
import os
from collections import OrderedDict, deque
from pathlib import Path
import selectors
import struct
import subprocess
from threading import Condition, Event, Lock, Thread, current_thread
import time

from ..protocol.messages import HAND_JOINT_NAMES, SessionState
from ..worker_environment import isolated_worker_environment


_INPUT = struct.Struct("<4sBBHQQQII126d")
_SESSION = struct.Struct("<4sBBHQQQBBHI")
_SHUTDOWN = struct.Struct("<4sBBH")
_OUTPUT = struct.Struct("<4sBBHQQQQQBBHI40d")
_HANDSHAKE = struct.Struct("<4sBBHQ")
_INPUT_SIZE = _INPUT.size
_SESSION_SIZE = _SESSION.size
_SHUTDOWN_SIZE = _SHUTDOWN.size
_OUTPUT_SIZE = _OUTPUT.size
_HANDSHAKE_SIZE = _HANDSHAKE.size


# The C++ manifest uses the pinned official model order internally, but this
# adapter must expose the repository's canonical protocol names just like the
# Python OfficialHandClient does. Keeping the lists immutable at module load
# avoids rebuilding aliases on every result while preserving that boundary.
_JOINT_NAMES = {side: list(HAND_JOINT_NAMES[side]) for side in ("left", "right")}


def _positive(value: int, field: str) -> int:
    if type(value) is not int or not 0 < value < 2**63:
        raise ValueError(f"{field} must be a positive int64")
    return value


def _finite_points(points):
    if not isinstance(points, list) or len(points) not in (63, 126):
        raise ValueError("native hand scheduler points must contain 63 or 126 values")
    result = [float(value) for value in points]
    if not all(math.isfinite(value) for value in result):
        raise ValueError("native hand scheduler points must be finite")
    return result


def _phase(state: str) -> int:
    values = {"idle": 0, "teleop": 1, "returning": 2, "fault": 3}
    if state not in values:
        raise ValueError("native hand scheduler requires idle/teleop/returning/fault")
    return values[state]


class NativeHandSchedulerClient:
    """Synchronous or asynchronous client for one C++ hand scheduler process."""

    async_retarget = True

    def __init__(self, *, python, scheduler=None, launcher=None, timeout_seconds=1.0,
                 single_hand_side="right", period_ns=5_000_000,
                 freshness_ns=200_000_000, output_capacity=256,
                 startup_handshake=True, generation=0, left_config=None, right_config=None,
                 async_mode=False):
        if single_hand_side not in ("left", "right"):
            raise ValueError("native hand scheduler side must be left or right")
        _positive(period_ns, "period_ns")
        _positive(freshness_ns, "freshness_ns")
        if type(output_capacity) is not int or not 0 < output_capacity <= 8192:
            raise ValueError("output_capacity must be in 1..8192")
        if type(generation) is not int or not 0 <= generation < 2**63:
            raise ValueError("generation must be a nonnegative int64")
        if type(async_mode) is not bool:
            raise ValueError("async_mode must be boolean")
        if (type(timeout_seconds) not in (int, float) or not math.isfinite(timeout_seconds)
                or timeout_seconds <= 0):
            raise ValueError("native hand scheduler timeout must be finite and positive")
        root = Path(__file__).resolve().parents[4]
        self.single_hand_side = single_hand_side
        self.freshness_ns = int(freshness_ns)
        self._timeout = float(timeout_seconds)
        self._lock = Lock()
        self._closed = False
        self._mode = None
        self.async_retarget = async_mode
        self._sequence = 0
        self._timestamp = 0
        self._epoch = 1
        self._pending_epoch = None
        self._last_session_phase = None
        self._session_phase = None
        self._generation = generation
        self._session_sequence = 0
        self._session_timestamp = 0
        self._output_capacity = output_capacity
        self._async_condition = Condition()
        self._async_stop = Event()
        self._async_thread = None
        self._async_failure = None
        self._async_outputs = deque()
        self._async_submissions = OrderedDict()
        self._async_last_output_sequence = 0
        self._async_dropped_outputs = 0
        self._async_discard_through = 0
        self.startup_ready = False
        scheduler_path = Path(scheduler or root / "build/hand-native/tianji_hand_native_scheduler")
        launcher_path = Path(launcher or root / "scripts/wuji_hand_native_scheduler_launcher.py")
        if not scheduler_path.is_file() or not os.access(scheduler_path, os.X_OK):
            raise RuntimeError("native hand scheduler missing; run pixi run build-native-hand-scheduler")
        if not launcher_path.is_file():
            raise RuntimeError("native hand scheduler launcher missing")
        python_path = Path(python).resolve(strict=True)
        if not python_path.is_file() or not os.access(python_path, os.X_OK):
            raise RuntimeError("native hand scheduler Python runtime is not executable")
        command = [str(python_path), str(launcher_path.resolve(strict=True)),
                   "--native-scheduler", str(scheduler_path.resolve(strict=True)),
                   "--period-ns", str(period_ns), "--freshness-ns", str(freshness_ns),
                   "--output-capacity", str(output_capacity)]
        if left_config is not None:
            command += ["--left-config", str(Path(left_config).resolve(strict=True))]
        if right_config is not None:
            command += ["--right-config", str(Path(right_config).resolve(strict=True))]
        if startup_handshake:
            command.append("--startup-handshake")
        self._process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                          bufsize=0, env=isolated_worker_environment())
        try:
            os.set_blocking(self._process.stdin.fileno(), False)
            os.set_blocking(self._process.stdout.fileno(), False)
            if startup_handshake:
                handshake = self._read_exact(_HANDSHAKE_SIZE)
                magic, version, reserved, declared_size, abi = _HANDSHAKE.unpack(handshake)
                if (magic != b"TJHK" or version != 1 or reserved != 0 or
                        declared_size != _HANDSHAKE_SIZE or abi != 1):
                    raise RuntimeError("invalid native hand scheduler handshake")
                self.startup_ready = True
        except BaseException:
            self._shutdown()
            raise

    def update_session(self, state, *, execution_epoch=None):
        if isinstance(state, SessionState):
            value = state
        else:
            value = SessionState.from_dict(state)
        with self._lock:
            if self._closed:
                return False
            if execution_epoch is not None:
                if type(execution_epoch) is not int or not 0 < execution_epoch < 2**63:
                    raise ValueError("execution_epoch must be a positive int64")
                if execution_epoch < (self._pending_epoch or self._epoch):
                    raise ValueError("execution_epoch cannot move backwards")
            sequence = value.sequence
            if type(sequence) is not int or not 0 <= sequence < 2**63:
                raise ValueError('session sequence must be a nonnegative int64')
            timestamp = _positive(value.timestamp_ns, "session timestamp_ns")
            if self._last_session_phase is not None and (sequence <= self._session_sequence or
                                            timestamp < self._session_timestamp):
                return False
            next_epoch = self._pending_epoch or self._epoch
            if execution_epoch is not None:
                next_epoch = execution_epoch
            elif (self._last_session_phase is not None and value.state == 'idle' and
                  self._last_session_phase != 'idle' and self._pending_epoch is None):
                next_epoch += 1
            frame = _SESSION.pack(b"TJHS", 1, _phase(value.state), _SESSION_SIZE,
                                  next_epoch,
                                  sequence, timestamp, 0, 0, 0, 0)
            if next_epoch != self._epoch or _phase(value.state) != self._session_phase:
                with self._async_condition:
                    self._async_discard_through = self._sequence
                    self._async_dropped_outputs += len(self._async_outputs)
                    self._async_outputs.clear()
            try:
                self._write_all(frame)
            except BaseException:
                self._shutdown()
                raise
            self._epoch = next_epoch
            self._pending_epoch = None
            self._last_session_phase = value.state
            self._session_phase = _phase(value.state)
            self._session_sequence, self._session_timestamp = sequence, timestamp
            return True

    def set_execution_epoch(self, execution_epoch: int) -> None:
        if type(execution_epoch) is not int or not 0 < execution_epoch < 2**63:
            raise ValueError("execution_epoch must be a positive int64")
        with self._lock:
            if self._closed:
                raise RuntimeError("native hand scheduler closed")
            if execution_epoch < (self._pending_epoch or self._epoch):
                raise ValueError("execution_epoch cannot move backwards")
            self._pending_epoch = execution_epoch

    def retarget(self, points, *, sequence: int, timestamp_ns: int):
        with self._lock:
            if self._closed:
                raise RuntimeError("native hand scheduler closed; no implicit restart")
            self._select_mode_locked("sync")
            sequence = _positive(sequence, "input sequence")
            timestamp_ns = _positive(timestamp_ns, "input timestamp_ns")
            if sequence <= self._sequence or timestamp_ns < self._timestamp:
                raise ValueError("native hand scheduler input ordering rollback")
            values = _finite_points(points)
            padded = [0.0] * 126
            if len(values) == 63:
                flags = 1 if self.single_hand_side == "left" else 2
                offset = 63 if self.single_hand_side == "left" else 0
                padded[offset:offset + 63] = values
            else:
                flags = 3
                padded[:] = values
            request = _INPUT.pack(b"TJHI", 1, flags, _INPUT_SIZE, sequence, timestamp_ns,
                                  self._generation, 126, 0, *padded)
            try:
                self._write_all(request)
                output = self._read_matching(sequence, timestamp_ns)
            except BaseException:
                self._shutdown()
                raise
            self._sequence, self._timestamp = sequence, timestamp_ns
            return self._result(output, sequence, timestamp_ns, expected_flags=flags)

    def submit_retarget(self, points, *, sequence: int, timestamp_ns: int):
        """Submit one frame without waiting for the fixed-rate result.

        The native scheduler continues to own the clock and stateful pipeline;
        the reader thread only drains its fixed-size output stream. Call
        :meth:`poll_retarget` from the application/control owner to consume the
        newest completed result. The synchronous :meth:`retarget` API remains
        available and cannot be mixed with this mode on one process.
        """
        with self._lock:
            if self._closed:
                raise RuntimeError("native hand scheduler closed; no implicit restart")
            self._select_mode_locked("async")
            self._raise_async_failure_locked()
            sequence = _positive(sequence, "input sequence")
            timestamp_ns = _positive(timestamp_ns, "input timestamp_ns")
            if sequence <= self._sequence or timestamp_ns < self._timestamp:
                raise ValueError("native hand scheduler input ordering rollback")
            _, flags, padded = self._pack_points(points)
            metadata = dict(sequence=sequence, timestamp_ns=timestamp_ns,
                            flags=flags, epoch=self._epoch,
                            submitted_at=time.monotonic(),
                            phase=self._session_phase if self._session_phase is not None else 0)
            with self._async_condition:
                self._async_submissions[sequence] = metadata
                while len(self._async_submissions) > max(1024, self._output_capacity * 64):
                    self._async_submissions.popitem(last=False)
            self._ensure_async_reader_locked()
            request = _INPUT.pack(b"TJHI", 1, flags, _INPUT_SIZE, sequence, timestamp_ns,
                                  self._generation, 126, 0, *padded)
            try:
                self._write_all(request)
            except BaseException:
                with self._async_condition:
                    self._async_submissions.pop(sequence, None)
                self._shutdown()
                raise
            self._sequence, self._timestamp = sequence, timestamp_ns
            return dict(sequence=sequence, timestamp_ns=timestamp_ns,
                        valid_sides=[side for side, bit in (("left", 1), ("right", 2))
                                     if flags & bit])

    def poll_retarget(self):
        """Return the newest completed asynchronous result, or ``None``.

        Older results are deliberately coalesced at this adapter boundary.
        They have already been consumed by the native scheduler and must not
        create a second command after a newer input has arrived.
        """
        with self._async_condition:
            self._raise_async_failure_locked()
            if not self._async_outputs:
                return None
            result = self._async_outputs[-1]
            self._async_dropped_outputs += len(self._async_outputs) - 1
            self._async_outputs.clear()
            if result['callback_sequence'] <= self._async_discard_through:
                self._async_dropped_outputs += 1
                return None
            return result

    @property
    def async_diagnostics(self):
        with self._async_condition:
            return dict(mode=self._mode, queued_outputs=len(self._async_outputs),
                        dropped_outputs=self._async_dropped_outputs,
                        last_output_sequence=self._async_last_output_sequence,
                        failure=self._async_failure)

    def _select_mode_locked(self, mode):
        if self._mode is None:
            self._mode = mode
        elif self._mode != mode:
            raise RuntimeError(f"native hand scheduler cannot switch from {self._mode} to {mode}")

    def _raise_async_failure_locked(self):
        with self._async_condition:
            failure = self._async_failure
        if failure:
            raise RuntimeError(failure)

    def _pack_points(self, points):
        values = _finite_points(points)
        padded = [0.0] * 126
        if len(values) == 63:
            flags = 1 if self.single_hand_side == "left" else 2
            offset = 63 if self.single_hand_side == "left" else 0
            padded[offset:offset + 63] = values
        else:
            flags = 3
            padded[:] = values
        return values, flags, padded

    def _ensure_async_reader_locked(self):
        if self._async_thread is not None:
            return
        self._async_stop.clear()
        self._async_thread = Thread(target=self._read_async_outputs,
                                    name="native-hand-scheduler-reader", daemon=True)
        self._async_thread.start()

    def _result(self, output, sequence, timestamp_ns, *, expected_flags,
                expected_epoch=None, expected_phase=None, allow_stale=False):
        (magic, version, header_flags, declared_size, output_sequence,
         input_sequence, input_timestamp, epoch, scheduler_timestamp,
         flags, phase, status, reserved, *positions) = _OUTPUT.unpack(output)
        if (magic != b"TJHO" or version != 1 or declared_size != _OUTPUT_SIZE or
                header_flags != flags or output_sequence <= 0 or input_sequence != sequence or
                input_timestamp != timestamp_ns or scheduler_timestamp <= 0 or epoch <= 0 or
                flags != expected_flags or flags == 0 or flags & ~3 or
                not ((phase == 1 and status in (2, 3)) or (phase in (0, 2) and status == 1)) or
                reserved != 0 or not all(math.isfinite(value) for value in positions)):
            raise RuntimeError("invalid native hand scheduler output")
        if expected_epoch is not None and epoch != expected_epoch:
            raise RuntimeError("native hand scheduler output epoch does not match input")
        if expected_phase is not None and phase != expected_phase:
            raise RuntimeError("native hand scheduler output phase does not match input")
        if expected_epoch is None and epoch != self._epoch:
            raise RuntimeError("native hand scheduler output epoch does not match session")
        if expected_phase is None and self._session_phase is not None and phase != self._session_phase:
            raise RuntimeError("native hand scheduler output phase does not match session")
        if phase == 1 and status != 2:
            if not allow_stale:
                raise RuntimeError("native hand scheduler teleop result is stale")
        return dict(schema_version=1, kind="wuji_hand_result", algorithm="official_wuji_hand2",
                    callback_sequence=sequence, timestamp_ns=timestamp_ns,
                    left=dict(valid=bool(flags & 1) and status != 3, joint_names=_JOINT_NAMES["left"],
                              position_rad=list(positions[:20])),
                    right=dict(valid=bool(flags & 2) and status != 3, joint_names=_JOINT_NAMES["right"],
                              position_rad=list(positions[20:])),
                    native_scheduler=dict(output_sequence=output_sequence, epoch=epoch,
                                          scheduler_timestamp_ns=scheduler_timestamp,
                                          phase=phase, status=status))

    def _read_async_outputs(self):
        selector = selectors.DefaultSelector()
        pending = bytearray()
        try:
            selector.register(self._process.stdout, selectors.EVENT_READ)
            while not self._async_stop.is_set():
                events = selector.select(.1)
                if not events:
                    if self._process.poll() is not None:
                        raise RuntimeError("native hand scheduler exited without result")
                    with self._async_condition:
                        outstanding = next((row for sequence, row in self._async_submissions.items()
                                            if sequence > self._async_discard_through), None)
                        if outstanding and time.monotonic() - outstanding['submitted_at'] > self._timeout:
                            raise RuntimeError('native hand scheduler result timeout')
                    continue
                chunk = os.read(self._process.stdout.fileno(), _OUTPUT_SIZE * 4)
                if not chunk:
                    if self._async_stop.is_set():
                        return
                    raise RuntimeError("native hand scheduler output pipe closed")
                pending.extend(chunk)
                while len(pending) >= _OUTPUT_SIZE:
                    output = bytes(pending[:_OUTPUT_SIZE])
                    del pending[:_OUTPUT_SIZE]
                    row = self._decode_async_output(output)
                    if row is None:
                        continue
                    with self._async_condition:
                        if row['callback_sequence'] <= self._async_discard_through:
                            self._async_dropped_outputs += 1
                            continue
                        if len(self._async_outputs) >= self._output_capacity:
                            self._async_outputs.popleft()
                            self._async_dropped_outputs += 1
                        self._async_outputs.append(row)
                        self._async_condition.notify_all()
        except Exception as exc:
            if not self._async_stop.is_set():
                with self._async_condition:
                    self._async_failure = f"native hand scheduler output failed: {exc}"
                    self._async_condition.notify_all()
        finally:
            selector.close()

    def _decode_async_output(self, output):
        if len(output) != _OUTPUT_SIZE:
            raise RuntimeError("native hand scheduler output size mismatch")
        unpacked = _OUTPUT.unpack(output)
        input_sequence = unpacked[5]
        input_timestamp = unpacked[6]
        output_sequence = unpacked[4]
        flags = unpacked[9]
        with self._async_condition:
            metadata = self._async_submissions.get(input_sequence)
            last_output_sequence = self._async_last_output_sequence
            discarded = input_sequence <= self._async_discard_through
        if metadata is None:
            raise RuntimeError("native hand scheduler output is not associated with a submission")
        if output_sequence <= last_output_sequence:
            raise RuntimeError("native hand scheduler output sequence rolled back")
        if input_timestamp != metadata["timestamp_ns"] or flags != metadata["flags"]:
            raise RuntimeError("native hand scheduler output metadata does not match submission")
        row = self._result(output, input_sequence, input_timestamp,
                           expected_flags=metadata["flags"],
                           expected_epoch=unpacked[7] if discarded else metadata["epoch"],
                           expected_phase=unpacked[10] if discarded else metadata["phase"], allow_stale=True)
        with self._async_condition:
            self._async_last_output_sequence = output_sequence
            for key in list(self._async_submissions):
                if key <= input_sequence:
                    del self._async_submissions[key]
        return None if discarded else row

    def _write_all(self, payload):
        deadline = time.monotonic() + self._timeout
        offset = 0
        with selectors.DefaultSelector() as selector:
            selector.register(self._process.stdin, selectors.EVENT_WRITE)
            while offset < len(payload):
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not selector.select(remaining):
                    raise TimeoutError("native hand scheduler input timeout")
                try:
                    count = os.write(self._process.stdin.fileno(), payload[offset:])
                except BlockingIOError:
                    continue
                if count <= 0:
                    raise RuntimeError("native hand scheduler input pipe closed")
                offset += count

    def _read_exact(self, size):
        deadline = time.monotonic() + self._timeout
        output = bytearray()
        with selectors.DefaultSelector() as selector:
            selector.register(self._process.stdout, selectors.EVENT_READ)
            while len(output) < size:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not selector.select(remaining):
                    raise TimeoutError("native hand scheduler output timeout")
                chunk = os.read(self._process.stdout.fileno(), size - len(output))
                if not chunk:
                    raise RuntimeError("native hand scheduler exited without output")
                output.extend(chunk)
        return bytes(output)

    def _read_matching(self, sequence, timestamp_ns):
        output = self._read_exact(_OUTPUT_SIZE)
        unpacked = _OUTPUT.unpack(output)
        if unpacked[5] != sequence or unpacked[6] != timestamp_ns:
            raise RuntimeError("native hand scheduler output is not associated with input")
        return output

    def _shutdown(self):
        self._closed = True
        self._async_stop.set()
        with self._async_condition:
            self._async_condition.notify_all()
        process = self._process
        if process.poll() is None:
            try:
                self._write_all(_SHUTDOWN.pack(b"TJHX", 1, 0, _SHUTDOWN_SIZE))
            except Exception:
                pass
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
        for stream in (process.stdin, process.stdout):
            try:
                stream.close()
            except Exception:
                pass
        reader = self._async_thread
        if reader is not None and reader is not current_thread():
            reader.join(timeout=2)

    def close(self):
        with self._lock:
            if not self._closed:
                self._shutdown()


def recording_metadata(environment):
    """Return portable scheduler provenance for explicit C++ selection."""
    if environment.get("TIANJI_HAND_SCHEDULER_BACKEND", "python") != "cpp":
        return None
    root = Path(__file__).resolve().parents[4]
    scheduler = root / "build/hand-native/tianji_hand_native_scheduler"
    optimizer = root / "build/hand-native/libtianji_hand_optimizer.so"
    launcher = root / "scripts/wuji_hand_native_scheduler_launcher.py"
    for path, label in ((scheduler, "native hand scheduler"),
                        (optimizer, "native hand optimizer library"),
                        (launcher, "native hand scheduler launcher")):
        if not path.is_file():
            raise RuntimeError(f"{label} missing; run pixi run build-native-hand-scheduler")
    if not os.access(scheduler, os.X_OK):
        raise RuntimeError("native hand scheduler is not executable")
    source_paths = {
        "scheduler": root / "native/hand/scheduler.hpp",
        "scheduler_wire": root / "native/hand/scheduler_wire.hpp",
        "scheduler_main": root / "native/hand/scheduler_main.cpp",
        "pipeline": root / "native/hand/pipeline.hpp",
        "geometry": root / "native/hand/geometry.cpp",
        "filter": root / "native/hand/lowpass.cpp",
        "optimizer": root / "native/hand/optimizer.cpp",
        # The pinned Python environment is only a startup manifest builder,
        # but its model/frame/parameter extraction still affects the native
        # runtime. Keep that provenance alongside the C++ sources.
        "manifest_builder": root / "scripts/wuji_hand_native_launcher.py",
        "launcher": launcher,
        "adapter": Path(__file__),
    }
    for label, path in source_paths.items():
        if not path.is_file():
            raise RuntimeError(f"native hand scheduler provenance source missing: {label}")
    return {
        "backend": "cpp",
        "abi": 1,
        "protocol": "TJHS/TJHI/TJHO",
        "input_size_bytes": _INPUT_SIZE,
        "session_size_bytes": _SESSION_SIZE,
        "output_size_bytes": _OUTPUT_SIZE,
        "scheduler_sha256": hashlib.sha256(scheduler.read_bytes()).hexdigest(),
        "optimizer_library_sha256": hashlib.sha256(optimizer.read_bytes()).hexdigest(),
        "source_sha256": {name: hashlib.sha256(path.read_bytes()).hexdigest()
                          for name, path in source_paths.items()},
    }


__all__ = ["NativeHandSchedulerClient", "recording_metadata"]
