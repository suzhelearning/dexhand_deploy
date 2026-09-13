"""Scheduling primitives for the live SPARK control path.

The control owner must never wait for rendering, recording, or diagnostics.
These small queues make that boundary explicit: the fixed-rate loop only
performs one non-blocking enqueue, while a side-effect worker owns the slower
consumer.
"""
from __future__ import annotations

from dataclasses import dataclass
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread
import time
from typing import Any, Callable

from ...recording.live_capture import LiveCycleSnapshot


class FixedRateControlLoop:
    """Run a callback on an absolute-deadline fixed-rate schedule.

    ``tick_sink`` is required to be non-blocking.  It is intentionally called
    after ``tick`` so the control callback remains the only authority over
    control state and all slower work can be handed to a bounded queue.
    """

    def __init__(self, *, period_s: float, tick: Callable[[], Any],
                 stop_event: Event, tick_sink: Callable[[Any], Any] | None = None,
                 clock: Callable[[], float] = time.monotonic) -> None:
        if isinstance(period_s, bool) or not isinstance(period_s, (int, float)) or period_s <= 0:
            raise ValueError('control period must be positive')
        if not callable(tick) or not isinstance(stop_event, Event):
            raise TypeError('tick and stop_event are required')
        if tick_sink is not None and not callable(tick_sink):
            raise TypeError('tick_sink must be callable')
        self.period_s = float(period_s)
        self._tick = tick
        self._stop = stop_event
        self._tick_sink = tick_sink
        self._clock = clock
        self._thread: Thread | None = None
        self._lock = Lock()
        self._failure: str | None = None
        self._tick_count = 0
        self._late_cycles = 0

    @property
    def failure(self) -> str | None:
        with self._lock:
            return self._failure

    @property
    def tick_count(self) -> int:
        with self._lock:
            return self._tick_count

    @property
    def late_cycles(self) -> int:
        with self._lock:
            return self._late_cycles

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError('control loop already started')
        self._thread = Thread(target=self._run, name='spark-control-loop', daemon=True)
        self._thread.start()

    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def join(self, timeout: float | None = None) -> None:
        if self._thread is not None:
            self._thread.join(timeout)

    def _latch_failure(self, exc: BaseException) -> None:
        with self._lock:
            if self._failure is None:
                self._failure = f'control loop failed: {type(exc).__name__}: {exc}'
        self._stop.set()

    def _run(self) -> None:
        deadline = self._clock()
        try:
            while not self._stop.is_set():
                value = self._tick()
                with self._lock:
                    self._tick_count += 1
                if self._tick_sink is not None:
                    self._tick_sink(value)
                deadline += self.period_s
                remaining = deadline - self._clock()
                if remaining < 0:
                    with self._lock:
                        self._late_cycles += 1
                    # Keep the absolute schedule.  The reference C++ loop
                    # catches up after an overrun instead of converting one
                    # late tick into a fresh period of additional latency.
                else:
                    self._stop.wait(remaining)
        except Exception as exc:
            self._latch_failure(exc)


@dataclass(frozen=True)
class LiveOperatorEvent:
    """An operator audit event ordered with control snapshots."""

    report: dict[str, Any]


class SnapshotSideEffectLoop:
    """Consume control snapshots without ever blocking the control owner."""

    _SENTINEL = object()

    def __init__(self, consumer: Callable[[Any], Any], *, capacity: int = 1024) -> None:
        if not callable(consumer):
            raise TypeError('side-effect consumer must be callable')
        if type(capacity) is not int or not 0 < capacity <= 65536:
            raise ValueError('side-effect capacity must be in 1..65536')
        self._consumer = consumer
        self._queue: Queue[Any] = Queue(maxsize=capacity)
        self._thread: Thread | None = None
        self._lock = Lock()
        self._failure: str | None = None
        self._closed = False

    @property
    def failure(self) -> str | None:
        with self._lock:
            return self._failure

    def _set_failure(self, message: str) -> None:
        with self._lock:
            if self._failure is None:
                self._failure = message

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError('side-effect loop already started')
        self._thread = Thread(target=self._run, name='spark-live-side-effects', daemon=True)
        self._thread.start()

    def submit(self, value: Any) -> bool:
        with self._lock:
            if self._closed or self._failure is not None:
                return False
            try:
                self._queue.put_nowait(value)
            except Full:
                self._failure = 'side-effect snapshot queue overflow'
                return False
        return True

    def _run(self) -> None:
        while True:
            try:
                value = self._queue.get()
            except Exception as exc:
                self._set_failure(f'side-effect queue failed: {type(exc).__name__}: {exc}')
                return
            if value is self._SENTINEL:
                return
            try:
                self._consumer(value)
            except Exception as exc:
                self._set_failure(f'side-effect worker failed: {type(exc).__name__}: {exc}')
                return

    def close(self, *, drain: bool = True) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            failed = self._failure is not None
        if self._thread is None:
            return
        if drain and not failed:
            try:
                self._queue.put(self._SENTINEL, timeout=30)
            except Full:
                self._set_failure('side-effect queue drain timeout')
        else:
            # A failed consumer has already stopped or will stop shortly; the
            # pending items are no longer a complete stream. Discard them so
            # the sentinel can release a consumer that is finishing one item.
            while True:
                try:
                    self._queue.get_nowait()
                except Empty:
                    break
            try:
                self._queue.put(self._SENTINEL, timeout=5)
            except Full:
                self._set_failure('side-effect queue shutdown timeout')
        self._thread.join(timeout=30)
        if self._thread.is_alive():
            self._set_failure('side-effect worker did not stop')


class SparkControlLoop:
    """Own a live ``SparkLiveSimulation`` on a dedicated control thread."""

    _ACTIONS = frozenset(('start', 'return', 'shutdown', 'rearm'))

    def __init__(self, core: Any, receiver: Any, *, stop_event: Event,
                 period_s: float, failure_fn: Callable[[], str | None],
                 snapshot_sink: SnapshotSideEffectLoop,
                 clock: Callable[[], float] = time.monotonic) -> None:
        if not isinstance(stop_event, Event):
            raise TypeError('stop_event is required')
        if not callable(failure_fn) or not hasattr(snapshot_sink, 'submit'):
            raise TypeError('failure_fn and snapshot_sink are required')
        if isinstance(period_s, bool) or not isinstance(period_s, (int, float)) or period_s <= 0:
            raise ValueError('control period must be positive')
        self._core = core
        self._receiver = receiver
        self._stop = stop_event
        self._period_s = float(period_s)
        self._failure_fn = failure_fn
        self._snapshot_sink = snapshot_sink
        self._clock = clock
        self._actions: Queue[str] = Queue(maxsize=64)
        self._reports: Queue[dict[str, Any]] = Queue(maxsize=64)
        self._thread: Thread | None = None
        self._lock = Lock()
        self._failure: str | None = None
        self._latest: LiveCycleSnapshot | None = None
        self._state = 'idle'
        self._reason = 'startup'
        self._tick_count = 0
        self._native_ticks = 0
        self._late_cycles = 0
        self._exit_after_home = False

    @property
    def failure(self) -> str | None:
        with self._lock:
            return self._failure

    @property
    def latest(self) -> LiveCycleSnapshot | None:
        with self._lock:
            return self._latest

    @property
    def state(self) -> str:
        with self._lock:
            return self._state

    @property
    def reason(self) -> str:
        with self._lock:
            return self._reason

    @property
    def tick_count(self) -> int:
        with self._lock:
            return self._tick_count

    @property
    def native_ticks(self) -> int:
        with self._lock:
            return self._native_ticks

    @property
    def late_cycles(self) -> int:
        with self._lock:
            return self._late_cycles

    def submit(self, action: str) -> bool:
        if action not in self._ACTIONS:
            raise ValueError(f'unsupported control action: {action}')
        try:
            self._actions.put_nowait(action)
        except Full:
            self._latch_failure('control action queue overflow')
            return False
        return True

    def poll_reports(self) -> list[dict[str, Any]]:
        reports = []
        while True:
            try:
                reports.append(self._reports.get_nowait())
            except Empty:
                return reports

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError('Spark control loop already started')
        self._thread = Thread(target=self._run, name='spark-control-loop', daemon=True)
        self._thread.start()

    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def join(self, timeout: float | None = None) -> None:
        if self._thread is not None:
            self._thread.join(timeout)

    def _latch_failure(self, message: str | BaseException) -> None:
        if isinstance(message, BaseException):
            message = f'control loop failed: {type(message).__name__}: {message}'
        with self._lock:
            if self._failure is None:
                self._failure = str(message)
        self._stop.set()

    def _emit_report(self, report: dict[str, Any]) -> None:
        if not self._snapshot_sink.submit(LiveOperatorEvent(report)):
            self._latch_failure('control side-effect queue rejected operator report')
        try:
            self._reports.put_nowait(report)
        except Full:
            self._latch_failure('control operator report queue overflow')

    def _process_actions(self) -> bool:
        skip_tick = False
        while True:
            try:
                action = self._actions.get_nowait()
            except Empty:
                return skip_tick
            if action == 'rearm':
                try:
                    ack = self._core.rearm_at_home()
                except ValueError as exc:
                    report = dict(kind='operator_result', action='rearm', accepted=False, reason=str(exc))
                else:
                    report = dict(kind='operator_result', action='rearm', accepted=True,
                                  execution_epoch=ack['execution_epoch'], reset_ack=ack,
                                  reason='fresh input and explicit start required')
                    skip_tick = True
            else:
                outcome = self._core.request(action)
                report = dict(kind='operator_result', action=action,
                              accepted=outcome.accepted, reason=outcome.reason)
                if action == 'shutdown':
                    self._exit_after_home = True
            self._emit_report(report)

    def _run(self) -> None:
        deadline = self._clock()
        try:
            while not self._stop.is_set():
                if self._process_actions():
                    deadline += self._period_s
                    remaining = deadline - self._clock()
                    if remaining > 0:
                        self._stop.wait(remaining)
                    else:
                        with self._lock:
                            self._late_cycles += 1
                        # Preserve the absolute deadline so a late control
                        # tick can catch up on the next iteration.
                    continue
                result = self._core.step(
                    self._receiver.try_read_latest(),
                    source_failure=self._failure_fn(),
                )
                snapshot = LiveCycleSnapshot.from_core(self._core, result)
                with self._lock:
                    self._latest = snapshot
                    self._state = snapshot.session_state.state
                    self._reason = snapshot.session_state.reason
                    self._tick_count += 1
                    if snapshot.result.native_result is not None:
                        self._native_ticks = snapshot.result.native_result.get('tick_id', self._native_ticks)
                # This call is deliberately a put_nowait boundary.  It must
                # never run recording, serialization, rendering, or network IO.
                self._snapshot_sink.submit(snapshot)
                if self._core.failure or self.state == 'fault':
                    self._stop.set()
                elif self._exit_after_home and self.state == 'idle':
                    self._stop.set()
                deadline += self._period_s
                remaining = deadline - self._clock()
                if remaining < 0:
                    with self._lock:
                        self._late_cycles += 1
                    # Preserve the absolute deadline; resetting it here
                    # would add a full period after every overrun.
                else:
                    self._stop.wait(remaining)
        except Exception as exc:
            self._latch_failure(exc)
