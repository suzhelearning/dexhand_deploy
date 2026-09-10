"""Official Hand2 process adapter. No inference imports or actuator access.

Input is callback-stage MediaPipe21 (right then left), not raw Manus25.
This boundary translates joint aliases only; it must not retarget or clamp
the official result again. Publishing is a separately authorized operation.
"""
from copy import deepcopy
import math
import json
import os
from pathlib import Path
import selectors
import subprocess
from threading import Lock
import time

from ..protocol.messages import HAND_JOINT_NAMES, HandJointCommand, SessionState, strict_loads
from ..worker_environment import isolated_worker_environment


def validate_result(row, *, sequence, timestamp_ns):
    expected = dict(schema_version=1, kind='wuji_hand_result', algorithm='official_wuji_hand2',
                    callback_sequence=sequence, timestamp_ns=timestamp_ns)
    if (not isinstance(row, dict) or set(row) != set(expected) | {'left', 'right'} or
            any(type(row.get(k)) is not type(v) or row[k] != v for k, v in expected.items())):
        raise ValueError('unassociated or incompatible official hand result')
    result = deepcopy(row)
    for side in ('left', 'right'):
        hand = result[side]
        if (not isinstance(hand, dict) or set(hand) != {'valid', 'joint_names', 'position_rad'} or
                type(hand['valid']) is not bool):
            raise ValueError('official hand result requires both side validity flags')
        canonical = list(HAND_JOINT_NAMES[side])
        official = [name.replace('_index_', '_index_finger_')
                    .replace('_middle_', '_middle_finger_').replace('_ring_', '_ring_finger_')
                    for name in canonical]
        if hand['joint_names'] != official:
            raise ValueError('official hand joint order or side mismatch')
        values = hand['position_rad']
        if (not isinstance(values, list) or len(values) != 20 or
                any(type(v) not in (float, int) or not math.isfinite(v) for v in values)):
            raise ValueError('official hand result must contain 20 finite radians')
        hand['joint_names'] = canonical
    return result


class OfficialHandClient:
    """Serialized bounded IPC; a failed transaction cannot silently restart."""
    def __init__(self, *, python, script, timeout_seconds=20., single_hand_side='right', startup_handshake=False):
        if (type(timeout_seconds) not in (int, float) or
                not math.isfinite(timeout_seconds) or timeout_seconds <= 0):
            raise ValueError('timeout must be finite and positive')
        if single_hand_side not in ('left', 'right'):
            raise ValueError('single_hand_side must be explicit left/right')
        if type(startup_handshake) is not bool:
            raise ValueError('startup_handshake must be boolean')
        command = [str(Path(p).resolve(strict=True)) for p in (python, script)]
        command += ['--single-hand-side', single_hand_side]
        if startup_handshake:
            command.append('--startup-handshake')
        self.startup_ready = False
        self._lock = Lock()
        self._timeout = timeout_seconds
        self._sequence = 0
        self._timestamp = 0
        self._closed = False
        self._process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0,
                                         env=isolated_worker_environment())
        try:
            os.set_blocking(self._process.stdin.fileno(), False)
            os.set_blocking(self._process.stdout.fileno(), False)
            if startup_handshake:
                ready = self._exchange(b'')
                expected = dict(schema_version=1, kind='wuji_worker_ready',
                                algorithm='official_wuji_hand2', callbacks=0)
                if (not isinstance(ready, dict) or ready != expected or
                        any(type(ready[k]) is not type(v) for k, v in expected.items())):
                    raise ValueError('invalid official hand startup handshake')
                self.startup_ready = True
        except BaseException:
            self._shutdown()
            raise

    def retarget(self, points, *, sequence, timestamp_ns):
        with self._lock:
            if self._closed:
                raise RuntimeError('official hand worker closed; no implicit restart')
            if (type(sequence) is not int or not self._sequence < sequence < 2**63 or
                    type(timestamp_ns) is not int or not 0 < timestamp_ns < 2**63 or
                    timestamp_ns < self._timestamp):
                raise ValueError('callback sequence must increase and receive time cannot roll back')
            if (not isinstance(points, list) or len(points) not in (63, 126) or
                    any(type(v) not in (int, float) or not math.isfinite(v) for v in points)):
                raise ValueError('callback points must be 63/126 finite values')
            request = (json.dumps(dict(schema_version=1, kind='wuji_hand_input',
                callback_sequence=sequence, timestamp_ns=timestamp_ns, points=points),
                allow_nan=False, separators=(',', ':')) + '\n').encode()
            if len(request) > 16384:
                raise ValueError('official hand request exceeds IPC size limit')
            try:
                result = self._exchange(request)
                result = validate_result(result, sequence=sequence, timestamp_ns=timestamp_ns)
            except BaseException:
                self._shutdown()
                raise
            self._sequence, self._timestamp = sequence, timestamp_ns
            return result

    def _exchange(self, request):
        deadline = time.monotonic() + self._timeout
        # Requests can exceed PIPE_BUF: nonblocking partial writes are bounded
        # by the SAME transaction deadline as response reads.
        with selectors.DefaultSelector() as selector:
            selector.register(self._process.stdin, selectors.EVENT_WRITE)
            offset = 0
            while offset < len(request):
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not selector.select(remaining):
                    raise TimeoutError('official hand request timeout')
                try:
                    written = os.write(self._process.stdin.fileno(), request[offset:])
                except BlockingIOError:
                    continue
                if written == 0:
                    raise RuntimeError('incomplete official hand request')
                offset += written
            selector.unregister(self._process.stdin)
            selector.register(self._process.stdout, selectors.EVENT_READ)
            output = bytearray()
            while b'\n' not in output:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not selector.select(remaining):
                    raise TimeoutError('official hand response timeout')
                chunk = os.read(self._process.stdout.fileno(), 16385)
                if not chunk:
                    raise RuntimeError('official hand worker exited without result')
                output.extend(chunk)
                if len(output) > 16384:
                    raise RuntimeError('oversized official hand response')
            if output.count(b'\n') != 1 or not output.endswith(b'\n'):
                raise RuntimeError('unsolicited official hand output')
            return strict_loads(output)

    def _shutdown(self):
        self._closed = True
        if self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=2)
        self._process.stdin.close()
        self._process.stdout.close()

    def close(self):
        with self._lock:
            if not self._closed:
                self._shutdown()


class HandRetargetProducer:
    """Single-owner callback processing plus canonical session authorization.

    Filters advance on actual callbacks even while idle, as in the reference
    bridge. No stale callback is repeatedly fed through the retargeter. Backend
    failure latches unhealthy; neither late input nor session start clears it.
    """
    def __init__(self, backend, *, publisher_instance_id, router_zid,
                 coordinator_instance_id, receiver_instance_id, freshness_ns,
                 producer_id='official_wuji_hand2'):
        for value in (publisher_instance_id, router_zid, coordinator_instance_id,
                      receiver_instance_id, producer_id):
            if not isinstance(value, str) or not value.strip():
                raise ValueError('hand producer identities must be explicit')
        if type(freshness_ns) is not int or not 0 < freshness_ns < 2**63:
            raise ValueError('freshness_ns must be positive int64')
        self.backend = backend
        self.publisher_instance_id = publisher_instance_id
        self.router_zid = router_zid
        self.coordinator_instance_id = coordinator_instance_id
        self.receiver_instance_id = receiver_instance_id
        self.producer_id = producer_id
        self.freshness_ns = freshness_ns
        self.healthy = True
        self.reason = None
        self._session = None
        self._sequence = 0
        self._input_time = 0
        self._pending = None
        self._input_snapshot = None

    @property
    def input_snapshot(self):
        return deepcopy(self._input_snapshot)

    def update_session(self, value):
        state = SessionState.from_dict(value.to_dict() if isinstance(value, SessionState) else value)
        if (state.publisher_instance_id != self.coordinator_instance_id or
                state.router_zid != self.router_zid or state.source != 'coordinator'):
            return False
        if self._session is not None and (state.sequence <= self._session.sequence or
                                         state.timestamp_ns < self._session.timestamp_ns):
            return False
        self._session = state
        if state.state != 'teleop':
            self._pending = None
        return True

    def update_input(self, points, *, sequence, timestamp_ns, receiver_instance_id, now_ns):
        if not self.healthy or receiver_instance_id != self.receiver_instance_id:
            return False
        if (type(sequence) is not int or not self._sequence < sequence < 2**63 or
                type(now_ns) is not int or not 0 < now_ns < 2**63 or
                type(timestamp_ns) is not int or not 0 < timestamp_ns < 2**63 or
                timestamp_ns < self._input_time or not 0 <= now_ns - timestamp_ns <= self.freshness_ns):
            return False
        try:
            result = self.backend.retarget(points, sequence=sequence, timestamp_ns=timestamp_ns)
        except Exception as exc:
            self.healthy = False
            self.reason = f'official hand backend failed: {exc}'
            self._pending = None
            return False
        self._sequence, self._input_time = sequence, timestamp_ns
        self._input_snapshot = dict(sequence=sequence, timestamp_ns=timestamp_ns,
                                   valid_sides=[side for side in ('left', 'right') if result[side]['valid']])
        self._pending = result
        return True

    def tracking_authorized(self, now_ns):
        state = self._session
        return bool(self.healthy and state is not None and state.state == 'teleop' and
                    0 <= now_ns - state.timestamp_ns <= self.freshness_ns)

    def commands(self, now_ns):
        if type(now_ns) is not int or not 0 < now_ns < 2**63:
            raise ValueError('now_ns must be positive int64')
        if not self.tracking_authorized(now_ns):
            return {}
        row, self._pending = self._pending, None
        if row is None or not 0 <= now_ns - row['timestamp_ns'] <= self.freshness_ns:
            return {}
        return {side: HandJointCommand(1, row['callback_sequence'], row['timestamp_ns'],
                    self.producer_id, side, row[side]['joint_names'], row[side]['position_rad'],
                    self.publisher_instance_id, self.router_zid)
                for side in ('left', 'right') if row[side]['valid']}
