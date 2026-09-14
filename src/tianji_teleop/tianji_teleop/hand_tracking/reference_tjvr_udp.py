"""Receive-only TJVR transport around the exact reference stream gate.

The receive thread timestamps every datagram independently of the IK clock.
The control owner consumes at most one latest frame. Socket/recording failures
latch failure; no automatic rebind, reinitialization or session authorization.
raw_sink must be bounded/nonblocking; it is the recorder enqueue boundary.
"""
import socket
from threading import Event, Lock, Thread
import time

from .reference_tjvr_receiver import ReferenceTjvrReceiver


class ReferenceTjvrUdp:
    def __init__(self, *, receiver_instance_id, host='127.0.0.1', port=15000,
                 max_position_jump_m=.15, max_orientation_jump_rad=.6,
                 raw_sink=None, raw_frame_sink=None, clock=time.monotonic_ns, decision_sink=None,
                 target_source='packet'):
        if not isinstance(host, str) or not host:
            raise ValueError('explicit UDP bind host required')
        if type(port) is not int or not 0 <= port <= 65535:
            raise ValueError('UDP port must be an integer in 0..65535')
        self._receiver = ReferenceTjvrReceiver(receiver_instance_id, max_position_jump_m,
                                               max_orientation_jump_rad, raw_sink, raw_frame_sink,
                                               decision_sink=decision_sink, target_source=target_source)
        self._clock = clock
        self._stop = Event()
        self._lock = Lock()
        self._failure = None
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # No SO_REUSEADDR: a second input session must not share this port.
            self._socket.bind((host, port))
            self._socket.settimeout(.1)
            self.address = self._socket.getsockname()
            self._thread = Thread(target=self._receive, name='reference-tjvr-input', daemon=True)
            self._thread.start()
        except BaseException:
            self._socket.close()
            raise

    @property
    def failure(self):
        with self._lock:
            return self._failure

    @property
    def running(self):
        return not self._stop.is_set() and self.failure is None and self._thread.is_alive()

    def stats(self):
        return self._receiver.stats()

    def try_read_latest(self):
        if self.failure is not None or self._stop.is_set():
            return None
        return self._receiver.try_read_latest()

    def _receive(self):
        try:
            while not self._stop.is_set():
                try:
                    packet, _peer = self._socket.recvfrom(65536)
                except socket.timeout:
                    continue
                received = self._clock()
                self._receiver.ingest(packet, received)
        except Exception as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._failure = f'TJVR receiver failed: {exc}'
        finally:
            self._socket.close()

    def close(self):
        self._stop.set()
        self._socket.close()
        self._thread.join(timeout=1.)
        if self._thread.is_alive():
            raise RuntimeError('TJVR receiver did not stop; raw sink must not block')
