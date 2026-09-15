"""Serialized, bounded IPC to the isolated SPARK process; no actuator authority.

The owner supplies the control clock and at most one new receiver sample per
tick. This client does not schedule, resample, retry, restart or publish. A
transport failure poisons the instance: recovery requires an explicit new
session and its authorization checks. Offline deterministic mode is NOT a
real-time performance qualification.
"""
import math
import os
from pathlib import Path
import selectors
import subprocess
from threading import Lock
import time

from .input_modes import SPARK_BACKEND
from .spark_replay import ReplayTick, encode_tick
from ..protocol.messages import strict_loads
from ..worker_environment import isolated_worker_environment


class SparkWorkerClient:
    ALGORITHM = SPARK_BACKEND
    WIRE_PREFIX = 'spark'
    def __init__(self, *, worker, config, model, urdf,
                 required_capability='simulation', timeout_seconds=20.,
                 deterministic_test=False, startup_handshake=False, result_format='json', home_config=None):
        if required_capability != 'simulation':
            raise ValueError('SPARK worker currently supports simulation only')
        if (type(timeout_seconds) not in (int, float) or
                not math.isfinite(timeout_seconds) or timeout_seconds <= 0):
            raise ValueError('timeout must be finite and positive')
        if type(deterministic_test) is not bool:
            raise ValueError('deterministic_test must be boolean')
        if type(startup_handshake) is not bool:
            raise ValueError('startup_handshake must be boolean')
        if result_format not in ('json', 'binary'):
            raise ValueError('native result format must be json or binary')
        self._binary_results = result_format == 'binary'
        command = [str(Path(p).resolve(strict=True)) for p in (worker, config, model, urdf)]
        if home_config is not None:
            command.extend(['--home-config', str(Path(home_config).resolve(strict=True))])
        if deterministic_test:
            command.append('--deterministic-test')
        if startup_handshake:
            command.append('--startup-handshake')
        if self._binary_results:
            command.append('--binary-results')
        self.startup_ready = False
        self._timeout = float(timeout_seconds)
        self._deterministic = deterministic_test
        self._lock = Lock()
        self._closed = False
        self._tick = 0
        self._now = 0
        self._execution_epoch = 1
        self.last_timing = {}
        self._process = subprocess.Popen(command, stdin=subprocess.PIPE,
                                         stdout=subprocess.PIPE, bufsize=0, env=isolated_worker_environment())
        self._selector = selectors.DefaultSelector()
        try:
            os.set_blocking(self._process.stdin.fileno(), False)
            os.set_blocking(self._process.stdout.fileno(), False)
            self._selector.register(self._process.stdout, selectors.EVENT_READ)
            if startup_handshake:
                deadline = time.monotonic() + self._timeout
                data = bytearray()
                while b'\n' not in data:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0 or not self._selector.select(remaining):
                        raise TimeoutError('SPARK startup handshake timed out')
                    chunk = os.read(self._process.stdout.fileno(), 1025)
                    if not chunk:
                        raise RuntimeError('SPARK worker exited before startup handshake')
                    data.extend(chunk)
                    if len(data) > 1024:
                        raise ValueError('oversized SPARK startup handshake')
                if data.count(b'\n') != 1 or not data.endswith(b'\n'):
                    raise ValueError('unsolicited SPARK startup output')
                expected = dict(schema_version=1, kind=self.WIRE_PREFIX + '_worker_ready',
                                algorithm=self.ALGORITHM, native_ticks=0)
                ready = strict_loads(data)
                if (not isinstance(ready, dict) or ready != expected or
                        any(type(ready[k]) is not type(v) for k, v in expected.items())):
                    raise ValueError('invalid SPARK startup handshake')
                self.startup_ready = True
        except BaseException:
            self._shutdown()
            raise

    def step(self, tick: ReplayTick):
        with self._lock:
            if self._closed:
                raise RuntimeError('SPARK worker is closed; no implicit restart')
            if (type(tick.tick_id) is not int or tick.tick_id != self._tick + 1 or
                    type(tick.now_ns) is not int or not self._now < tick.now_ns < 2**63):
                raise ValueError('SPARK requires consecutive ticks and increasing int64 time')
            if tick.sample is not None:
                received = tick.sample.observation.frame.received_timestamp_ns
                if not 0 < received <= tick.now_ns:
                    raise ValueError('sample receive time must be positive and not in the future')
            self.last_timing = {}
            started = time.perf_counter()
            request = encode_tick(tick).encode('ascii')
            if len(request) > 2048:
                raise ValueError('SPARK request exceeds bounded IPC size')
            try:
                encoded = time.perf_counter()
                result = (self._exchange_binary(request, f'tick {tick.tick_id}')
                          if self._binary_results else self._exchange(request, f'tick {tick.tick_id}'))
                exchanged = time.perf_counter()
                _validate_result(result, tick, self._deterministic,
                                 algorithm=self.ALGORITHM, wire_prefix=self.WIRE_PREFIX)
                validated = time.perf_counter()
            except BaseException:
                self._shutdown()
                raise
            self._tick, self._now = tick.tick_id, tick.now_ns
            # Owner-thread diagnostics, deliberately excluded from wire results.
            self.last_timing.update(request_encode=encoded-started,
                ipc_roundtrip_decode=exchanged-encoded,
                result_validate=validated-exchanged)
            return result

    def _exchange_binary(self, request, context):
        from .native_binary_results import payload_size, decode_result, native_timing
        if len(request) > 2048:
            raise ValueError('SPARK request exceeds bounded IPC size')
        if os.write(self._process.stdin.fileno(), request) != len(request):
            raise RuntimeError('incomplete SPARK request write')
        data = bytearray()
        expected = None
        deadline = time.monotonic() + self._timeout
        while expected is None or len(data) < expected:
            remaining = deadline-time.monotonic()
            if remaining <= 0 or not self._selector.select(remaining):
                raise TimeoutError(f'native binary worker timed out at {context}')
            chunk = os.read(self._process.stdout.fileno(), 2049)
            if not chunk:
                raise RuntimeError('native worker exited without a complete result')
            data.extend(chunk)
            if len(data) >= 8 and expected is None:
                expected = 8 + payload_size(data[:8], self.WIRE_PREFIX)
            if expected is not None and len(data) > expected:
                raise RuntimeError('unsolicited native binary output')
        decoded = decode_result(data, self.WIRE_PREFIX)
        self.last_timing.update(native_timing(data))
        return decoded

    def _exchange(self, request, context):
        # Every request fits PIPE_BUF. The owner serializes request/response.
        if len(request) > 2048:
            raise ValueError('SPARK request exceeds bounded IPC size')
        if os.write(self._process.stdin.fileno(), request) != len(request):
            raise RuntimeError('incomplete SPARK request write')
        data = bytearray()
        deadline = time.monotonic() + self._timeout
        while b'\n' not in data:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not self._selector.select(remaining):
                raise TimeoutError(f'SPARK worker timed out at {context}')
            chunk = os.read(self._process.stdout.fileno(), 65536)
            if not chunk:
                raise RuntimeError('SPARK worker exited without a complete result')
            data.extend(chunk)
            if len(data) > 65536:
                raise RuntimeError('oversized SPARK result')
        if data.count(b'\n') != 1 or not data.endswith(b'\n'):
            raise RuntimeError('unsolicited SPARK output')
        return strict_loads(data)

    def reset_at_rest(self, positions, *, execution_epoch):
        """Owner must verify stopped, fresh bilateral feedback before calling.

        This operation resets model/history only; it cannot authorize commands.
        Source tracking-epoch changes must never call this deployment reset.
        """
        with self._lock:
            if self._closed:
                raise RuntimeError('SPARK worker is closed; no implicit restart')
            if type(execution_epoch) is not int or not self._execution_epoch < execution_epoch < 2**63:
                raise ValueError('execution epoch must increase')
            if (not isinstance(positions, (list, tuple)) or len(positions) != 14 or
                    any(type(v) not in (int, float) or not math.isfinite(v) for v in positions)):
                raise ValueError('reset needs fourteen finite bilateral positions at rest')
            q = list(map(float, positions))
            request = ('TJSR1 ' + str(execution_epoch) + ' ' + ' '.join(map(repr, q)) + '\n').encode('ascii')
            try:
                ack = self._exchange(request, f'reset epoch {execution_epoch}')
                expected = dict(schema_version=1, kind=self.WIRE_PREFIX + '_reset_ack', execution_epoch=execution_epoch,
                    position_rad=q, velocity_rad_s=[0.] * 14, acceleration_rad_s2=[0.] * 14)
                if (not isinstance(ack, dict) or ack != expected or
                        type(ack.get('schema_version')) is not int or type(ack.get('execution_epoch')) is not int or
                        any(type(v) not in (int, float) for field in
                            ('position_rad', 'velocity_rad_s', 'acceleration_rad_s2') for v in ack[field])):
                    raise ValueError('reset state acknowledgement does not match trusted feedback')
            except BaseException:
                self._shutdown()
                raise
            self._tick = 0
            self._execution_epoch = execution_epoch
            return ack

    def _shutdown(self):
        self._closed = True
        self._selector.close()
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

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def _validate_result(row, tick, deterministic, *, algorithm=SPARK_BACKEND, wire_prefix='spark'):
    def finite(value):
        if isinstance(value, dict):
            return all(finite(v) for v in value.values())
        if isinstance(value, list):
            return all(finite(v) for v in value)
        return type(value) is not float or math.isfinite(value)
    if not finite(row):
        raise ValueError('non-finite SPARK result or diagnostic')
    expected = dict(schema_version=1, kind=wire_prefix + '_bilateral_result',
                    algorithm=algorithm, state_source='model_reference',
                    simulation_only=True, deterministic_test=deterministic,
                    tick_id=tick.tick_id, timestamp_ns=tick.now_ns)
    if not isinstance(row, dict) or any(type(row.get(k)) is not type(v) or row[k] != v
                                        for k, v in expected.items()):
        raise ValueError('unassociated or incompatible SPARK result')
    for side in ('left', 'right'):
        arm = row.get(side)
        if not isinstance(arm, dict) or type(arm.get('accepted')) is not bool:
            raise ValueError('SPARK requires a complete bilateral result')
        for name in ('q', 'qdot', 'qddot'):
            values = arm.get(name)
            if (not isinstance(values, list) or len(values) != 7 or
                    any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
                raise ValueError(f'invalid SPARK {side} {name}')
