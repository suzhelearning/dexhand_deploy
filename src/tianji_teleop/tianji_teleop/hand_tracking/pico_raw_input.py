"""Bounded PICO raw-topic input for an independently authorized hand producer.

Consumes the existing ObservationRuntime raw payload, not its legacy
wrist-relative hand observation. Does not open another TCP/ADB receiver or
relabel PICO data as Manus. The session owner supplies transport callbacks.
"""
from collections import deque
from threading import Lock

from .models import PicoRawFrame
from .pico import parse_pico_packet
from ..protocol.messages import strict_loads


class PicoRawInputQueue:
    def __init__(self, *, receiver_instance_id, router_zid, connection_generation, capacity=256):
        if any(not isinstance(v, str) or not v.strip() for v in (receiver_instance_id, router_zid)):
            raise ValueError('explicit PICO receiver/router required')
        if (type(connection_generation) is not int or not 0 <= connection_generation < 2**63 or
                type(capacity) is not int or not 0 < capacity <= 65536):
            raise ValueError('invalid PICO generation or input queue capacity')
        self.receiver_instance_id, self.router_zid = receiver_instance_id, router_zid
        self.connection_generation = connection_generation
        self._capacity = capacity
        self._lock = Lock()
        self._queue = deque()
        self._failure = None
        self._closed = False
        self._sequence = -1
        self._timestamp = 0

    @property
    def failure(self):
        with self._lock:
            return self._failure

    def ingest(self, payload):
        with self._lock:
            if self._closed or self._failure:
                return False
        try:
            value = strict_loads(payload) if isinstance(payload, (bytes, bytearray, str)) else dict(payload)
            if (value.get('receiver_instance_id') != self.receiver_instance_id or
                    value.get('router_zid') != self.router_zid):
                return False
            value = dict(value)
            value.pop('router_zid')
            frame = PicoRawFrame.from_dict(value)
            if frame.connection_generation != self.connection_generation:
                raise ValueError('PICO connection generation changed; explicit reinitialization required')
            decoded = parse_pico_packet(frame.raw_packet, receiver_instance_id=frame.receiver_instance_id,
                connection_generation=frame.connection_generation, receiver_frame_sequence=frame.receiver_frame_sequence,
                received_timestamp_ns=frame.received_timestamp_ns)
            if decoded.to_dict() != frame.to_dict():
                raise ValueError('PICO raw fields disagree with original packet')
            with self._lock:
                if self._closed or self._failure:
                    return False
                if frame.receiver_frame_sequence <= self._sequence or frame.received_timestamp_ns < self._timestamp:
                    return False
                if len(self._queue) >= self._capacity:
                    raise ValueError('PICO hand input queue overflow; no silent frame coalescing')
                self._queue.append(decoded)
                self._sequence, self._timestamp = frame.receiver_frame_sequence, frame.received_timestamp_ns
            return True
        except (ValueError, TypeError, KeyError) as exc:
            with self._lock:
                self._failure = self._failure or 'PICO hand input failed: ' + str(exc)
                self._queue.clear()
            return False

    def try_read(self):
        with self._lock:
            if self._closed or self._failure or not self._queue:
                return None
            return self._queue.popleft()

    def close(self):
        with self._lock:
            self._closed = True
            self._queue.clear()
