"""Non-blocking Zenoh output for the live SPARK session."""
from __future__ import annotations

from dataclasses import dataclass
import json
from queue import Empty, Full, Queue
from threading import Lock, Thread
from typing import Any


@dataclass(frozen=True)
class _OutputItem:
    kind: str
    target: Any
    payload: Any = None
    encoding: str | None = None


class _QueuedPublisher:
    def __init__(self, owner: 'AsyncLiveOutput', publisher: Any) -> None:
        self._owner = owner
        self._publisher = publisher
        self._closed = False

    def put(self, payload: bytes | bytearray, *, encoding: str | None = None) -> None:
        if not isinstance(payload, (bytes, bytearray)):
            raise TypeError('Zenoh publisher payload must be bytes-like')
        if self._closed:
            raise RuntimeError('Zenoh publisher is undeclared')
        if not self._owner._submit(_OutputItem('publisher', self._publisher, bytes(payload), encoding)):
            raise RuntimeError(self._owner.failure or 'Zenoh output queue unavailable')

    def put_json(self, value: Any) -> None:
        if self._closed:
            raise RuntimeError('Zenoh publisher is undeclared')
        if not self._owner._submit(_OutputItem('publisher_json', self._publisher, value,
                                               'application/json')):
            raise RuntimeError(self._owner.failure or 'Zenoh output queue unavailable')

    def undeclare(self) -> None:
        if self._closed:
            return
        self._closed = True
        if not self._owner._submit(_OutputItem('undeclare', self._publisher)):
            try:
                self._publisher.undeclare()
            except (AttributeError, RuntimeError):
                pass


class _AsyncSessionProxy:
    """Delegate input/query operations; queue only publisher output."""

    def __init__(self, owner: 'AsyncLiveOutput', session: Any) -> None:
        self._owner = owner
        self._session = session

    def declare_publisher(self, topic: str) -> _QueuedPublisher:
        return self._owner.declare_publisher(topic)

    def put(self, topic: str, payload: bytes | bytearray, *, encoding: str | None = None) -> None:
        self._owner.put(topic, payload, encoding=encoding)

    def __getattr__(self, name: str) -> Any:
        return getattr(self._session, name)


class AsyncLiveOutput:
    """Serialize actual Zenoh writes on a dedicated output thread.

    Queue insertion is non-blocking.  Overflow or a publisher error is
    latched and exposed to the control owner so safety handling remains
    explicit instead of silently dropping control messages.
    """

    _SENTINEL = object()

    def __init__(self, session: Any, *, capacity: int = 8192) -> None:
        if session is None:
            raise ValueError('Zenoh session is required')
        if type(capacity) is not int or not 0 < capacity <= 65536:
            raise ValueError('Zenoh output capacity must be in 1..65536')
        self._session = session
        self._queue: Queue[Any] = Queue(maxsize=capacity)
        self._lock = Lock()
        self._failure: str | None = None
        self._closed = False
        self._thread = Thread(target=self._run, name='spark-zenoh-output', daemon=True)
        self._thread.start()

    @property
    def failure(self) -> str | None:
        with self._lock:
            return self._failure

    def session_proxy(self) -> _AsyncSessionProxy:
        return _AsyncSessionProxy(self, self._session)

    def declare_publisher(self, topic: str) -> _QueuedPublisher:
        if not isinstance(topic, str) or not topic:
            raise ValueError('Zenoh topic must be nonempty')
        return _QueuedPublisher(self, self._session.declare_publisher(topic))

    def put(self, topic: str, payload: bytes | bytearray, *, encoding: str | None = None) -> None:
        if not isinstance(topic, str) or not topic:
            raise ValueError('Zenoh topic must be nonempty')
        if not isinstance(payload, (bytes, bytearray)):
            raise TypeError('Zenoh session payload must be bytes-like')
        if not self._submit(_OutputItem('session', topic, bytes(payload), encoding)):
            raise RuntimeError(self.failure or 'Zenoh output queue unavailable')

    def put_json(self, topic: str, value: Any) -> None:
        if not isinstance(topic, str) or not topic:
            raise ValueError('Zenoh topic must be nonempty')
        if not self._submit(_OutputItem('session_json', topic, value,
                                        'application/json')):
            raise RuntimeError(self.failure or 'Zenoh output queue unavailable')

    def _submit(self, item: _OutputItem) -> bool:
        with self._lock:
            if self._closed or self._failure is not None:
                return False
            try:
                self._queue.put_nowait(item)
            except Full:
                self._failure = 'Zenoh output queue overflow'
                return False
        return True

    @staticmethod
    def _publisher_put(publisher: Any, payload: bytes, encoding: str | None) -> None:
        if encoding is None:
            publisher.put(payload)
            return
        try:
            publisher.put(payload, encoding=encoding)
        except TypeError:
            publisher.put(payload)

    def _set_failure(self, exc: BaseException) -> None:
        with self._lock:
            if self._failure is None:
                self._failure = f'Zenoh output failed: {type(exc).__name__}: {exc}'

    def _run(self) -> None:
        while True:
            try:
                item = self._queue.get()
            except Exception as exc:
                self._set_failure(exc)
                return
            if item is self._SENTINEL:
                return
            try:
                if item.kind == 'publisher':
                    self._publisher_put(item.target, item.payload, item.encoding)
                elif item.kind == 'publisher_json':
                    payload = json.dumps(item.payload, allow_nan=False,
                                         separators=(',', ':')).encode()
                    self._publisher_put(item.target, payload, item.encoding)
                elif item.kind == 'session':
                    if item.encoding is None:
                        self._session.put(item.target, item.payload)
                    else:
                        try:
                            self._session.put(item.target, item.payload, encoding=item.encoding)
                        except TypeError:
                            self._session.put(item.target, item.payload)
                elif item.kind == 'session_json':
                    payload = json.dumps(item.payload, allow_nan=False,
                                         separators=(',', ':')).encode()
                    try:
                        self._session.put(item.target, payload, encoding=item.encoding)
                    except TypeError:
                        self._session.put(item.target, payload)
                elif item.kind == 'undeclare':
                    item.target.undeclare()
                else:
                    raise RuntimeError(f'unknown Zenoh output action: {item.kind}')
            except Exception as exc:
                self._set_failure(exc)
                # Do not keep issuing puts after the first transport failure;
                # close still needs a chance to release queued publishers.
                while True:
                    try:
                        pending = self._queue.get_nowait()
                    except Empty:
                        return
                    if pending is self._SENTINEL:
                        return
                    if pending.kind == 'undeclare':
                        try:
                            pending.target.undeclare()
                        except (AttributeError, RuntimeError):
                            pass

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            failed = self._failure is not None
        if failed:
            while True:
                try:
                    self._queue.get_nowait()
                except Empty:
                    break
        try:
            self._queue.put(self._SENTINEL, timeout=30)
        except Full:
            self._set_failure(RuntimeError('Zenoh output queue shutdown timeout'))
        self._thread.join(timeout=30)
        if self._thread.is_alive():
            self._set_failure(RuntimeError('Zenoh output worker did not stop'))
