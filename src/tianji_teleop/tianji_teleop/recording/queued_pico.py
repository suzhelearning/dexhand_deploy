"""New PICO recorder: bounded receipt FIFO, one HDF5 dispatcher.

Network callbacks copy payloads and receipt times only. They must never wait
for HDF5 or compete for the writer lock with a high-rate status subscription.
Legacy SessionRecorderNode scheduling is unchanged.
"""
from copy import deepcopy
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread
import time

from .recorder import RecorderProtocolError, SessionRecorderNode
from .buffered_writer import BufferedSessionWriter


class QueuedPicoRecorder(SessionRecorderNode):
    def __init__(self, *args, capacity=4096, **kwargs):
        if kwargs.get('source_type') != 'pico2_hands_sim':
            raise ValueError('queued recording is restricted to new PICO simulation')
        if type(capacity) is not int or not 0 < capacity <= 65536:
            raise ValueError('record queue capacity must be in 1..65536')
        self._incoming = Queue(maxsize=capacity)
        self._ingress_lock = Lock()
        self._accepting = True
        self._dispatch_stop = Event()
        self._dispatch_failure = None
        super().__init__(*args, writer_factory=BufferedSessionWriter, **kwargs)
        self._dispatcher = Thread(target=self._dispatch, name='pico-hdf5-dispatcher', daemon=True)
        try:
            self._dispatcher.start()
        except BaseException:
            self._failed = RecorderProtocolError('recorder dispatcher startup failed')
            super().close()
            raise

    def _on_sample(self, key, sample):
        received = time.monotonic_ns()
        value = getattr(sample, 'payload', sample)
        value = value.to_bytes() if hasattr(value, 'to_bytes') else deepcopy(value)
        with self._ingress_lock:
            if not self._accepting:
                return
            try:
                self._incoming.put_nowait((key, value, received))
            except Full:
                self._dispatch_failure = RecorderProtocolError('PICO recording queue overflow; incomplete capture')
                self._accepting = False

    def _dispatch(self):
        try:
            while not self._dispatch_stop.is_set() or not self._incoming.empty():
                if self._dispatch_failure is not None:
                    return
                try:
                    key, value, received = self._incoming.get(timeout=.05)
                except Empty:
                    super().flush()
                    continue
                self.receive(key, value, received_time_ns=received)
        except Exception as exc:
            with self._ingress_lock:
                self._accepting = False
                self._dispatch_failure = self._dispatch_failure or RecorderProtocolError(
                    f'PICO recording dispatch failed: {exc}')

    @property
    def failure(self):
        return self._dispatch_failure or super().failure

    @property
    def failed(self):
        return self.failure is not None

    def flush(self):
        # Appends retain SessionH5Writer's periodic flush; idle flushing is on
        # the same dispatcher. The CLI checks health without taking its lock.
        if self._dispatch_failure is not None:
            raise self._dispatch_failure

    def close(self):
        if self._closed:
            return
        with self._ingress_lock:
            self._accepting = False
        # No writer lock here: undeclare may wait for an in-flight callback.
        for resource in self._resources:
            try:
                resource.undeclare()
            except Exception as exc:
                self._dispatch_failure = self._dispatch_failure or RecorderProtocolError(
                    f'PICO recorder subscription cleanup failed: {exc}')
        self._resources.clear()
        self._dispatch_stop.set()
        self._dispatcher.join(timeout=30)
        if self._dispatcher.is_alive():
            self._dispatch_failure = RecorderProtocolError('PICO recorder drain timeout; completion unverified')
            raise self._dispatch_failure  # never close HDF5 concurrently
        if self._dispatch_failure is not None:
            self._failed = self._dispatch_failure
        super().close()
        if self._dispatch_failure is not None:
            raise self._dispatch_failure
