"""Bounded ingress FIFO for the XR + Manus dual-input recorder.

Zenoh callbacks only copy a sample into the bounded queue.  One dispatcher owns
the HDF5 writer, so high-rate XR/status traffic cannot make the transport
callback perform disk I/O.  An overflow or dispatch failure invalidates the
capture instead of silently dropping input.
"""
from copy import deepcopy
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread
import time

from .buffered_writer import BufferedSessionWriter
from .recorder import RecorderProtocolError, SessionRecorderNode


class QueuedXrManusRecorder(SessionRecorderNode):
    """The queued recorder used only by ``vr_manus_xr_sim``."""

    def __init__(self, *args, capacity=16384, **kwargs):
        if kwargs.get("source_type") != "vr_manus_xr_sim":
            raise ValueError("queued XR recording is restricted to vr_manus_xr_sim")
        if type(capacity) is not int or not 0 < capacity <= 65536:
            raise ValueError("record queue capacity must be in 1..65536")
        self._incoming = Queue(maxsize=capacity)
        self._ingress_lock = Lock()
        self._accepting = True
        self._dispatch_stop = Event()
        self._dispatch_failure: RecorderProtocolError | None = None
        super().__init__(*args, writer_factory=BufferedSessionWriter, **kwargs)
        self._dispatcher = Thread(target=self._dispatch, name="xr-manus-hdf5-dispatcher", daemon=True)
        try:
            self._dispatcher.start()
        except BaseException:
            self._failed = RecorderProtocolError("XR/Manus recorder dispatcher startup failed")
            super().close()
            raise

    def _on_sample(self, key, sample):
        """Copy transport data only; never wait for HDF5."""
        received = time.monotonic_ns()
        value = getattr(sample, "payload", sample)
        value = value.to_bytes() if hasattr(value, "to_bytes") else deepcopy(value)
        with self._ingress_lock:
            if not self._accepting:
                return
            try:
                self._incoming.put_nowait((key, value, received))
            except Full:
                self._dispatch_failure = RecorderProtocolError(
                    "XR/Manus recording queue overflow; incomplete capture"
                )
                self._accepting = False
                self._dispatch_stop.set()

    def _dispatch(self):
        try:
            while not self._dispatch_stop.is_set() or not self._incoming.empty():
                if self._dispatch_failure is not None:
                    return
                try:
                    key, value, received = self._incoming.get(timeout=0.05)
                except Empty:
                    super().flush()
                    continue
                self.receive(key, value, received_time_ns=received)
        except Exception as exc:
            with self._ingress_lock:
                self._accepting = False
                self._dispatch_failure = self._dispatch_failure or RecorderProtocolError(
                    f"XR/Manus recording dispatch failed: {exc}"
                )

    @property
    def failure(self):
        return self._dispatch_failure or super().failure

    @property
    def failed(self):
        return self.failure is not None

    def flush(self):
        if self._dispatch_failure is not None:
            raise self._dispatch_failure

    def close(self):
        if self._closed:
            return
        with self._ingress_lock:
            self._accepting = False
        # Stop ingress before undeclaring.  The dispatcher drains the accepted
        # prefix before the HDF5 owner is closed.
        for resource in self._resources:
            try:
                resource.undeclare()
            except Exception as exc:
                self._dispatch_failure = self._dispatch_failure or RecorderProtocolError(
                    f"XR/Manus recorder subscription cleanup failed: {exc}"
                )
        self._resources.clear()
        self._dispatch_stop.set()
        self._dispatcher.join(timeout=30)
        if self._dispatcher.is_alive():
            self._dispatch_failure = RecorderProtocolError(
                "XR/Manus recorder drain timeout; completion unverified"
            )
            raise self._dispatch_failure
        if self._dispatch_failure is not None:
            self._failed = self._dispatch_failure
        super().close()
        if self._dispatch_failure is not None:
            raise self._dispatch_failure


__all__ = ["QueuedXrManusRecorder"]
