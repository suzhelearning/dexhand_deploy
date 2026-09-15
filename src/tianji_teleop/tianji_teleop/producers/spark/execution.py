"""Nonblocking, bounded acknowledgement supervision, separate from QP feedback.

No function here authorizes motion, resets native state, or interprets accepted
commands as measured joints. The session owner must jointly reset/re-authorize
by constructing a new execution epoch after a latched integration fault.
"""
import math


def execution_guard_type(implementation='python'):
    if implementation == 'python':
        return ExecutionGuard
    if implementation == 'cpp':
        from .native_execution import NativeExecutionGuard, load_native
        load_native()  # fail explicitly before entering the control loop
        return NativeExecutionGuard
    raise ValueError('execution guard must be python or cpp')


def _positive(value, name):
    if type(value) is not int or not 0 < value < 2**63:
        raise ValueError(f'{name} must be positive int64')
    return value


def _positions(value):
    if not isinstance(value, dict) or set(value) != {'left', 'right'}:
        raise ValueError('positions must include exactly both arms')
    result = {}
    for side in ('left', 'right'):
        q = value[side]
        if (not isinstance(q, (list, tuple)) or len(q) != 7 or
                any(type(v) not in (float, int) or not math.isfinite(v) for v in q)):
            raise ValueError('positions must be finite seven-joint vectors')
        result[side] = tuple(float(v) for v in q)
    return result


class ExecutionGuard:
    def __init__(self, *, run_id, execution_epoch, coordinator_instance_id, router_zid,
                 maximum_receipt_age_ns, max_in_flight, mode='reference_direct'):
        for value in (run_id, coordinator_instance_id, router_zid):
            if not isinstance(value, str) or not value.strip():
                raise ValueError('execution identities must be explicit nonempty strings')
        if mode != 'reference_direct':
            raise ValueError('processed_guarded requires explicit validated command/feedback limits; not enabled')
        self.run_id = run_id
        self.execution_epoch = _positive(execution_epoch, 'execution_epoch')
        self.coordinator_instance_id = coordinator_instance_id
        self.router_zid = router_zid
        self.maximum_receipt_age_ns = _positive(maximum_receipt_age_ns, 'maximum_receipt_age_ns')
        self.max_in_flight = _positive(max_in_flight, 'max_in_flight')
        self._pending = {}
        self._last_tick = 0
        self._last_now = 0
        self.reason = None

    @property
    def paused(self):
        return self.reason is not None

    @property
    def in_flight(self):
        return len(self._pending)

    def pause(self, reason):
        if not isinstance(reason, str) or not reason:
            raise ValueError('pause reason required')
        if not self.paused:
            self.reason = reason
        self._pending.clear()

    def check(self, now_ns):
        _positive(now_ns, 'now_ns')
        if now_ns < self._last_now:
            self.pause('execution clock rollback')
        self._last_now = max(now_ns, self._last_now)
        if any(now_ns - sent > self.maximum_receipt_age_ns for sent, _ in self._pending.values()):
            self.pause('coordinator receipt timeout')
        return not self.paused

    def register(self, tick_id, timestamp_ns, reference_positions):
        _positive(tick_id, 'tick_id')
        if tick_id != self._last_tick + 1:
            raise ValueError('execution ticks must be consecutive')
        q = _positions(reference_positions)
        if not self.check(timestamp_ns):
            raise RuntimeError(self.reason)
        if len(self._pending) >= self.max_in_flight:
            self.pause('execution in-flight limit exceeded')
            raise RuntimeError(self.reason)
        self._pending[tick_id] = (timestamp_ns, q)
        self._last_tick = tick_id

    def observe(self, receipt, now_ns):
        if not self.check(now_ns):
            return False
        if not isinstance(receipt, dict):
            return False
        identity = dict(run_id=self.run_id, execution_epoch=self.execution_epoch,
                        publisher_instance_id=self.coordinator_instance_id, router_zid=self.router_zid)
        if any(type(receipt.get(k)) is not type(v) or receipt[k] != v for k, v in identity.items()):
            return False
        tick = receipt.get('tick_id')
        if type(tick) is not int or tick not in self._pending:
            return False
        sent, reference = self._pending[tick]
        required = {'schema_version', 'kind', 'run_id', 'execution_epoch', 'tick_id',
                    'timestamp_ns', 'publisher_instance_id', 'router_zid', 'stage',
                    'accepted', 'reason', 'command_position_rad'}
        try:
            if (set(receipt) != required or type(receipt['schema_version']) is not int or
                    receipt['schema_version'] != 1 or receipt['kind'] != 'arm_bilateral_receipt' or
                    receipt['stage'] != 'coordinator_command' or type(receipt['accepted']) is not bool or
                    not isinstance(receipt['reason'], str) or type(receipt['timestamp_ns']) is not int or
                    not sent <= receipt['timestamp_ns'] <= now_ns):
                raise ValueError('invalid receipt schema/time')
            command = _positions(receipt['command_position_rad'])
        except (TypeError, ValueError) as exc:
            self.pause(str(exc))
            return False
        if not receipt['accepted']:
            self.pause('coordinator rejected bilateral command: ' + receipt['reason'])
            return False
        if command != reference:
            self.pause('reference_direct command was modified downstream')
            return False
        del self._pending[tick]
        return True
