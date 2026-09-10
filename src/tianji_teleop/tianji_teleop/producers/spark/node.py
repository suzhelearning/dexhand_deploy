"""Single-owner SPARK producer core. Publishes proposals, never final commands.

Transport callbacks must queue events to the owning control loop. The session
owner handles lifecycle and re-authorization; a latched fault cannot be cleared
by a later input frame, receipt or session heartbeat.
"""
from copy import deepcopy

from ...hand_tracking.input_modes import SPARK_BACKEND
from ...hand_tracking.reference_tjvr_receiver import ReceivedTjvrFrame
from ...hand_tracking.spark_replay import ReplayTick
from ...protocol.bilateral import ArmBilateralProposal
from ...protocol.messages import ArmJointProposal, ARM_JOINT_NAMES, ComponentStatus, SessionState
from .execution import ExecutionGuard, _positive


class SparkProducer:
    def __init__(self, backend, *, run_id, execution_epoch, publisher_instance_id,
                 coordinator_instance_id, router_zid, receiver_instance_id,
                 maximum_receipt_age_ns, session_timeout_ns, max_in_flight,
                 producer_id='ik_spark_headroom'):
        for value in (publisher_instance_id, receiver_instance_id, producer_id):
            if not isinstance(value, str) or not value.strip():
                raise ValueError('producer/input identities must be explicit')
        self.backend = backend
        self.publisher_instance_id = publisher_instance_id
        self.router_zid = router_zid
        self.receiver_instance_id = receiver_instance_id
        self.producer_id = producer_id
        self.guard = ExecutionGuard(run_id=run_id, execution_epoch=execution_epoch,
            coordinator_instance_id=coordinator_instance_id, router_zid=router_zid,
            maximum_receipt_age_ns=maximum_receipt_age_ns, max_in_flight=max_in_flight)
        self.session_timeout_ns = _positive(session_timeout_ns, 'session_timeout_ns')
        self._session = None
        self._input = None
        self._input_sequence = 0
        self._input_time = 0
        self._tick = 0
        self._result = None
        self._last_attempt = None
        self._ready = False
        self._status_sequence = 0

    @property
    def paused(self):
        return self.guard.paused

    @property
    def last_attempt(self):
        """Exact consumed receiver sample, including failed native attempts."""
        return deepcopy(self._last_attempt)

    @property
    def last_result(self):
        return deepcopy(self._result)

    def update_session(self, value):
        state = SessionState.from_dict(value.to_dict() if isinstance(value, SessionState) else value)
        if (state.publisher_instance_id != self.guard.coordinator_instance_id or
                state.router_zid != self.router_zid or state.source != 'coordinator'):
            return False
        if self._session is not None and (state.sequence <= self._session.sequence or
                                         state.timestamp_ns < self._session.timestamp_ns):
            return False
        self._session = state
        if self._tick and state.state != 'teleop':
            self.guard.pause('session left teleop; joint reset and re-authorization required')
        return True

    def update_input(self, value):
        sample = ReceivedTjvrFrame.from_dict(value.to_dict() if isinstance(value, ReceivedTjvrFrame) else value)
        frame = sample.observation.frame
        if (frame.receiver_instance_id != self.receiver_instance_id or
                frame.receiver_frame_sequence <= self._input_sequence or
                frame.received_timestamp_ns < self._input_time):
            return False
        self._input_sequence = frame.receiver_frame_sequence
        self._input_time = frame.received_timestamp_ns
        self._input = sample
        self._ready = bool(frame.upper_limb_skeleton_valid)
        return True

    def observe_execution(self, receipt, now_ns):
        return self.guard.observe(receipt, now_ns)

    def rearm_at_rest(self, positions, *, execution_epoch):
        """Session owner must first verify actual, stationary bilateral state."""
        if (not self.paused or self._session is None or self._session.state != 'idle' or
                type(execution_epoch) is not int or execution_epoch != self.guard.execution_epoch + 1):
            raise ValueError('rearm requires explicit idle pause and next execution epoch')
        ack = self.backend.reset_at_rest(positions, execution_epoch=execution_epoch)
        self.guard = ExecutionGuard(run_id=self.guard.run_id, execution_epoch=execution_epoch,
            coordinator_instance_id=self.guard.coordinator_instance_id, router_zid=self.router_zid,
            maximum_receipt_age_ns=self.guard.maximum_receipt_age_ns, max_in_flight=self.guard.max_in_flight)
        # Keep input identity/sequence/time and status counters monotonic. Drop
        # cached input and require a NEW valid frame before another start.
        self._input = None
        self._ready = False
        self._tick = 0
        self._result = None
        self._last_attempt = None
        return ack

    def tick(self, now_ns):
        if not self.guard.check(now_ns):
            return None
        state = self._session
        if state is None or state.state != 'teleop':
            return None
        if not 0 <= now_ns - state.timestamp_ns <= self.session_timeout_ns:
            self.guard.pause('session authorization stale or future-dated')
            return None
        if self._tick == 0 and (not self._ready or not 0 <= now_ns - self._input_time <= self.session_timeout_ns):
            return None
        if self.guard.in_flight >= self.guard.max_in_flight:
            self.guard.pause('execution in-flight limit exceeded before native step')
            return None
        sample, self._input = self._input, None
        self._last_attempt = dict(tick_id=self._tick + 1, timestamp_ns=now_ns,
                                  sample=sample.to_dict() if sample is not None else None)
        try:
            result = self.backend.step(ReplayTick(self._tick + 1, now_ns, sample))
            if result['algorithm'] != SPARK_BACKEND or result['tick_id'] != self._tick + 1:
                raise ValueError('unexpected native algorithm or tick')
            proposals = {}
            for side in ('left', 'right'):
                diagnostics = dict(algorithm=SPARK_BACKEND, state_source=result['state_source'],
                    reference_execution_mode='reference_direct', execution_epoch=self.guard.execution_epoch,
                    tick_id=result['tick_id'], applied_epoch=result['applied_epoch'],
                    applied_sequence=result['applied_sequence'], input_live=result['input_live'],
                    native=deepcopy(result[side]))
                proposals[side] = ArmJointProposal(1, result['tick_id'], now_ns, self.producer_id, side,
                    result['applied_sequence'], list(ARM_JOINT_NAMES[side]), result[side]['q'], diagnostics,
                    self.publisher_instance_id, self.router_zid)
            pair = ArmBilateralProposal(self.guard.run_id, self.guard.execution_epoch, result['tick_id'],
                                        proposals['left'], proposals['right'])
            self.guard.register(pair.tick_id, now_ns, {s: getattr(pair, s).position_rad for s in ('left', 'right')})
        except Exception as exc:
            self.guard.pause(f'SPARK producer failed: {exc}')
            return None
        self._tick = result['tick_id']
        self._result = deepcopy(result)
        return pair

    def status(self, now_ns):
        self._status_sequence += 1
        initial_ready = self._ready and 0 <= now_ns - self._input_time <= self.session_timeout_ns
        ready = bool(initial_ready or self._tick) and not self.paused
        return ComponentStatus(1, self._status_sequence, now_ns, 'producer_arm', self.producer_id,
            'fault' if self.paused else 'ready' if ready else 'waiting_input',
            ready, not self.paused, ['simulation'], self.guard.reason,
            dict(algorithm=SPARK_BACKEND, state_source='model_reference', native_ticks=self._tick,
                 reference_execution_mode='reference_direct', in_flight=self.guard.in_flight,
                 latest_skeleton_valid=self._ready),
            self.publisher_instance_id, self.router_zid)

    def close(self):
        self.guard.pause('producer closed')
        self.backend.close()
