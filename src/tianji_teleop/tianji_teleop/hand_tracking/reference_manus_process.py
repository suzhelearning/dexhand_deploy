"""Owned rawviz subprocess, original parser, bounded ordered callbacks.

SDK clocks are metadata only; receive timestamps use the host monotonic clock.
No ROS dependency, retargeting, actuator publication, or automatic restart.
The consumer must drain callbacks in order; overload is explicit failure.
"""
from collections import deque
from dataclasses import dataclass
import os
import selectors
import subprocess
from threading import Event, Lock, Thread
import time

from .reference_manus import HandInputAssembler, RawvizHandInputProcessor


@dataclass(frozen=True)
class ManusCallback:
    receiver_instance_id: str
    sequence: int
    received_timestamp_ns: int
    points: tuple
    source_sequences: dict
    source_timestamps_ns: dict


class ReferenceManusProcess:
    def __init__(self, *, command, receiver_instance_id, sides=('left', 'right'),
                 capacity=256, cwd=None, env=None, clock=time.monotonic_ns,
                 right_glove=None, left_glove=None, raw_line_sink=None, callback_sink=None,
                 parser_backend='python', parser_library=None):
        if (not isinstance(command, (list, tuple)) or not command or
                any(not isinstance(v, str) or not v for v in command)):
            raise ValueError('rawviz command must be an explicit nonempty argument vector')
        if not isinstance(receiver_instance_id, str) or not receiver_instance_id.strip():
            raise ValueError('receiver identity required')
        if not sides or len(set(sides)) != len(sides) or set(sides) - {'left', 'right'}:
            raise ValueError('explicit distinct hand sides required')
        if type(capacity) is not int or not 0 < capacity <= 65536:
            raise ValueError('callback capacity must be in 1..65536')
        if right_glove and right_glove == left_glove:
            raise ValueError('left and right glove bindings must differ')
        self._identity = receiver_instance_id
        self._capacity = capacity
        self._clock = clock
        self._raw_line_sink = raw_line_sink
        self._callback_sink = callback_sink
        self._queue = deque()
        self._lock = Lock()
        self._stop = Event()
        self._failure = None
        self._sequence = 0
        self._received_ns = 0
        if parser_backend not in ('python','cpp'):
            raise ValueError('Manus parser backend must be python or cpp')
        if parser_backend == 'cpp':
            from .native_manus_parser import NativeManusProcessor
            self._processor = NativeManusProcessor(self._enqueue,sides=sides,
                right_glove=right_glove,left_glove=left_glove,library=parser_library)
        else:
            self._processor = RawvizHandInputProcessor(
                HandInputAssembler('right' in sides, 'left' in sides), self._enqueue,
                right_glove=right_glove, left_glove=left_glove)
        try:
            self._process = subprocess.Popen(command, cwd=cwd, env=env,
                stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, bufsize=0)
        except BaseException:
            self._close_parser()
            raise
        self._thread = Thread(target=self._receive, name='reference-manus-input', daemon=True)
        try:
            self._thread.start()
        except BaseException:
            self._process.terminate()
            self._process.wait(timeout=2)
            self._process.stdout.close()
            self._close_parser()
            raise

    @property
    def failure(self):
        with self._lock:
            return self._failure

    @property
    def pending_count(self):
        with self._lock:
            return len(self._queue)

    def drain_pending(self):
        """Return callbacks accepted before shutdown, preserving their order."""
        with self._lock:
            pending = list(self._queue)
            self._queue.clear()
            return pending

    def try_read(self):
        with self._lock:
            if self._failure or self._stop.is_set() or not self._queue:
                return None
            return self._queue.popleft()

    def _enqueue(self, frame):
        self._sequence += 1
        row = ManusCallback(self._identity, self._sequence, self._received_ns,
            tuple(frame.values.tolist()), dict(frame.sequences), dict(frame.source_timestamps_ns))
        if self._callback_sink is not None:
            self._callback_sink(row)
        with self._lock:
            if len(self._queue) >= self._capacity:
                raise RuntimeError('Manus callback queue overflow; no silent coalescing')
            self._queue.append(row)

    def _receive(self):
        try:
            with selectors.DefaultSelector() as selector:
                selector.register(self._process.stdout, selectors.EVENT_READ)
                pending = bytearray()
                while not self._stop.is_set():
                    if not selector.select(.05):
                        continue
                    chunk = os.read(self._process.stdout.fileno(), 8192)
                    if not chunk:
                        raise RuntimeError('rawviz stdout closed; explicit restart required')
                    pending.extend(chunk)
                    while b'\n' in pending:
                        line, _, rest = pending.partition(b'\n')
                        pending = bytearray(rest)
                        if len(line) > 65536:
                            raise ValueError('rawviz line exceeds 65536 bytes')
                        now = self._clock()
                        if type(now) is not int or not 0 < now < 2**63 or now < self._received_ns:
                            raise ValueError('Manus receive clock rolled back or is invalid')
                        self._received_ns = now
                        text = line.decode('utf-8', errors='strict')
                        if self._raw_line_sink is not None:
                            self._raw_line_sink(text, now)
                        self._processor.process_line(text)
                    if len(pending) > 65536:
                        raise ValueError('rawviz unterminated line exceeds 65536 bytes')
        except Exception as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._failure = f'Manus input failed: {exc}'
                    self._queue.clear()

    def _close_parser(self):
        close = getattr(self._processor, 'close', None)
        if close is not None: close()

    def close(self, *, clear_pending=True):
        if type(clear_pending) is not bool:
            raise TypeError('clear_pending must be a bool')
        self._stop.set()
        if self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=2)
        self._thread.join(timeout=1)
        if self._thread.is_alive():
            raise RuntimeError('Manus input did not stop; raw line sink must not block')
        self._process.stdout.close()
        self._close_parser()
        if clear_pending:
            with self._lock:
                self._queue.clear()
