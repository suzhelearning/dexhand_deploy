"""Independent official-hand callback owner, never run on the arm clock.

Session transitions are ordered and applied again after a potentially slow
retarget transaction. Publishing and session enqueue share a lock, preventing
a completed stop update from racing a later command publication. The publish
callback must be bounded; source/backend ownership remains with the caller.
"""
from collections import deque
from threading import Event, RLock, Thread
import time

from ..protocol.messages import ComponentStatus, SessionState
from .retarget_input import manus_callback_input


class HandRetargetLoop:
    def __init__(self, producer, source, *, publish, clock=time.monotonic_ns,
                 input_adapter=manus_callback_input, processed_input_sink=None,
                 drop_expired_inputs=False, expired_input_sink=None, latest_input_only=False):
        if not callable(input_adapter):
            raise ValueError('explicit callable retarget input adapter required')
        if processed_input_sink is not None and not callable(processed_input_sink):
            raise ValueError('processed input sink must be callable')
        self._processed_input_sink = processed_input_sink
        if type(drop_expired_inputs) is not bool:
            raise TypeError('drop_expired_inputs must be bool')
        if expired_input_sink is not None and not callable(expired_input_sink):
            raise TypeError('expired_input_sink must be callable')
        self._drop_expired_inputs = drop_expired_inputs
        if type(latest_input_only) is not bool or (latest_input_only and not drop_expired_inputs):
            raise ValueError('latest input selection requires explicit expiry handling')
        self._latest_input_only = latest_input_only
        self._superseded_callbacks = 0
        self._expired_input_sink = expired_input_sink
        self._expired_callbacks = 0
        self._last_received_sequence = 0
        self._last_received_time = 0
        self._input_adapter = input_adapter
        self.producer, self.source = producer, source
        self._publish, self._clock = publish, clock
        self._lock = RLock()
        self._stop = Event()
        self._sessions = deque()
        self._failure = None
        self._processed = 0
        self._input_snapshot = None
        self._input_history = deque(maxlen=256)
        self._input_rows = {}
        self._coalesced_output_callbacks = 0
        self._unresolved_output_callbacks = 0
        self._last_accounted_sequence = 0
        self._seen_valid_sides = set()
        self._status_sequence = 0
        self._max_retarget_duration_ns = 0
        self._max_callback_age_ns = 0
        self._thread = Thread(target=self._run, name='official-hand-retarget', daemon=True)
        self._thread.start()

    @property
    def failure(self):
        with self._lock:
            return self._failure

    @property
    def processed_callbacks(self):
        with self._lock:
            return self._processed

    def update_session(self, state):
        state = SessionState.from_dict(state.to_dict() if isinstance(state, SessionState) else state)
        with self._lock:
            if self._stop.is_set() or self._failure:
                return False
            if len(self._sessions) >= 1024:
                self._failure = 'hand session event queue overflow'
                return False
            self._sessions.append(state)
            return True

    def status(self, now_ns, required_sides, *, allow_tracking_hold=False):
        if type(now_ns) is not int or not 0 < now_ns < 2**63:
            raise ValueError('status clock must be positive int64')
        if not required_sides or len(set(required_sides)) != len(required_sides) or set(required_sides) - {'left', 'right'}:
            raise ValueError('required hand sides must be explicit and distinct')
        with self._lock:
            self._status_sequence += 1
            # The arm owner samples its control clock before calling us. A
            # concurrent hand callback may have a later receive time by now.
            # Select the actual processed input at that cutoff, never retime a
            # newer callback or hide a stale previous input by refreshing it.
            snapshot = next((item for item in reversed(self._input_history)
                             if item['timestamp_ns'] <= now_ns), None)
            reason = self._failure or self.source.failure
            healthy = not reason and not self._stop.is_set()
            fresh = bool(healthy and snapshot and
                         0 <= now_ns - snapshot['timestamp_ns'] <= self.producer.freshness_ns)
            valid = set(snapshot['valid_sides']) if snapshot else set()
            holding = bool(allow_tracking_hold and fresh and set(required_sides) <= self._seen_valid_sides and
                           self.producer.tracking_authorized(now_ns))
            ready = bool(fresh and (set(required_sides) <= valid or holding))
            if snapshot is None:
                readiness_reason = 'waiting for first Manus callback'
            elif not fresh:
                readiness_reason = 'latest Manus callback is stale'
            elif not set(required_sides) <= valid and not holding:
                readiness_reason = 'latest Manus callback is missing a required side'
            else:
                readiness_reason = None
            return ComponentStatus(1, self._status_sequence, now_ns, 'producer_hand', self.producer.producer_id,
                'ready' if ready else 'waiting_input' if healthy else 'fault', ready, healthy,
                ['simulation'], reason, dict(processed_callbacks=self._processed,
                    valid_sides=list(snapshot['valid_sides']) if snapshot else [],
                    max_retarget_duration_ns=self._max_retarget_duration_ns,
                    max_callback_age_ns=self._max_callback_age_ns,
                    expired_callbacks=self._expired_callbacks,
                    superseded_callbacks=self._superseded_callbacks,
                    coalesced_output_callbacks=self._coalesced_output_callbacks,
                    pending_output_callbacks=len(self._input_rows),
                    unresolved_output_callbacks=self._unresolved_output_callbacks,
                    tracking_hold_sides=sorted(set(required_sides) - valid) if holding else [],
                    input_timestamp_ns=snapshot['timestamp_ns'] if snapshot else None,
                    latest_input_timestamp_ns=(self.producer.latest_input_snapshot or
                                               self._input_snapshot or {}).get('timestamp_ns'),
                    readiness_reason=readiness_reason),
                self.producer.publisher_instance_id, self.producer.router_zid)

    def _drain_sessions(self):
        while self._sessions:
            self.producer.update_session(self._sessions.popleft())

    def _validate_received(self, row, now):
        if (not self.producer.healthy or
                row.receiver_instance_id != self.producer.receiver_instance_id or
                type(row.sequence) is not int or
                not self._last_received_sequence < row.sequence < 2**63 or
                type(row.received_timestamp_ns) is not int or
                not 0 < row.received_timestamp_ns < 2**63 or
                row.received_timestamp_ns < self._last_received_time or now < row.received_timestamp_ns):
            raise RuntimeError('invalid hand callback identity, ordering or clock')
        self._last_received_sequence = row.sequence
        self._last_received_time = row.received_timestamp_ns

    def _audit_skipped(self, row, now, reason):
        if self._expired_input_sink is not None:
            self._expired_input_sink(dict(callback_sequence=row.sequence,
                receiver_instance_id=row.receiver_instance_id,
                timestamp_ns=row.received_timestamp_ns, age_ns=now - row.received_timestamp_ns,
                freshness_ns=self.producer.freshness_ns, reason=reason))

    def _remember_input(self, row):
        with self._lock:
            self._input_rows[row.sequence] = row
            # Never silently discard an association that a delayed worker may
            # still return. Fault explicitly if the bounded backlog is full.
            if len(self._input_rows) > 1024:
                raise RuntimeError('hand output association queue overflow')

    def _collect_commands(self, now_ns):
        """Poll completed retarget work independently of source callbacks."""
        accounted = None
        snapshot = None
        with self._lock:
            # Keep the session transition and publication in one owner lock.
            # A stop/rearm event queued concurrently therefore cannot be
            # overtaken by a deferred native result from the previous phase.
            self._drain_sessions()
            commands = self.producer.commands(now_ns)
            if not self.producer.healthy:
                raise RuntimeError(self.producer.reason or 'hand producer failed')
            snapshot = self.producer.input_snapshot
            if snapshot is not None and snapshot['sequence'] > self._last_accounted_sequence:
                sequence = snapshot['sequence']
                row = self._input_rows.pop(sequence, None)
                if row is None:
                    raise RuntimeError('hand output has no admitted input association')
                for skipped in [key for key in self._input_rows if key < sequence]:
                    self._input_rows.pop(skipped, None)
                    self._coalesced_output_callbacks += 1
                self._last_accounted_sequence = sequence
                self._input_snapshot = snapshot
                self._input_history.append(snapshot)
                self._seen_valid_sides.update(snapshot['valid_sides'])
                self._processed += 1
                accounted = row
        if accounted is not None and self._processed_input_sink is not None:
            self._processed_input_sink(accounted, snapshot)
        with self._lock:
            # Audit can block or fail. Accept stop events while it runs, then
            # fence the already generated commands before publication.
            interrupted = any(state.state != 'teleop' for state in self._sessions)
            self._drain_sessions()
            now = self._clock()
            if (self._stop.is_set() or self._failure or self.source.failure or interrupted or
                    not self.producer.tracking_authorized(now) or
                    any(not 0 <= now - row.timestamp_ns <= self.producer.freshness_ns
                        for row in commands.values())):
                return {}
            if commands:
                self._publish(commands)
        return commands

    def _run(self):
        try:
            while not self._stop.is_set():
                with self._lock:
                    if self._failure:
                        return
                    self._drain_sessions()
                if self.source.failure:
                    raise RuntimeError(self.source.failure)
                self._collect_commands(self._clock())
                row = self.source.try_read()
                if row is None:
                    self._stop.wait(.005)
                    continue
                row = self._input_adapter(row)
                if self._latest_input_only:
                    # Bounded drain: acquisition still records every callback.
                    # Only live solver work is coalesced, with explicit audit.
                    for _ in range(255):
                        newer = self.source.try_read()
                        if newer is None:
                            break
                        selected_at = self._clock()
                        self._validate_received(row, selected_at)
                        self._audit_skipped(row, selected_at, 'superseded_before_retarget')
                        with self._lock:
                            self._superseded_callbacks += 1
                        row = self._input_adapter(newer)
                now = self._clock()
                age = now - row.received_timestamp_ns
                if self._drop_expired_inputs:
                    # Only ordinary transport age is recoverable. Identity,
                    # ordering, future clocks and backend failures remain fatal.
                    self._validate_received(row, now)
                    if age > self.producer.freshness_ns:
                        self._audit_skipped(row, now, 'expired_before_retarget')
                        with self._lock:
                            self._expired_callbacks += 1
                            self._max_callback_age_ns = max(self._max_callback_age_ns, age)
                        # Do not refresh readiness or generate a command. Drain
                        # the expired prefix and wait for genuinely fresh input.
                        continue
                started = time.monotonic_ns()
                accepted = self.producer.update_input(row.payload, sequence=row.sequence,
                    timestamp_ns=row.received_timestamp_ns, receiver_instance_id=row.receiver_instance_id,
                    now_ns=now)
                duration = time.monotonic_ns() - started
                with self._lock:
                    self._max_callback_age_ns = max(self._max_callback_age_ns, age)
                    self._max_retarget_duration_ns = max(self._max_retarget_duration_ns, duration)
                if not accepted:
                    raise RuntimeError(self.producer.reason or
                        f'hand callback rejected: sequence={row.sequence}, age_ns={age}, '
                        f'freshness_ns={self.producer.freshness_ns}, receiver={row.receiver_instance_id}')
                self._remember_input(row)
                with self._lock:
                    self._drain_sessions()
                    if self._stop.is_set() or self._failure:
                        return
                    if self.source.failure:
                        raise RuntimeError(self.source.failure)
                self._collect_commands(self._clock())
        except Exception as exc:
            with self._lock:
                self._failure = f'hand processing failed: {exc}'
        finally:
            with self._lock:
                self._unresolved_output_callbacks += len(self._input_rows)
                self._input_rows.clear()

    def close(self):
        self._stop.set()
        self._thread.join(timeout=25)
        if self._thread.is_alive():
            raise RuntimeError('hand processing did not stop within bounded worker transaction timeout')
