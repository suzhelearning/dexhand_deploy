"""Live boundary adapters; acquisition/control enqueue, one recorder writes."""
from dataclasses import dataclass
from typing import Any
import time


@dataclass(frozen=True)
class LiveCycleSnapshot:
    """Owned values copied by the control owner for asynchronous consumers."""

    result: Any
    source_status: Any
    producer_status: Any
    executor_status: Any
    arm_state: Any
    session_state: Any
    hand_telemetry: dict
    hand_states: dict
    hand_commands: dict
    execution_epoch: int
    coordinator_receipt: dict | None
    bilateral_command: dict | None
    received_timestamp_ns: int

    @classmethod
    def from_core(cls, core, result):
        now = core.clock()
        return cls(
            result=result,
            source_status=core.source_status,
            producer_status=core.producer.status(now),
            executor_status=core.sim.status,
            arm_state=core.sim.arm_state,
            session_state=core.coordinator.state,
            hand_telemetry=core.hand_telemetry,
            hand_states={side: core.sim.hand_state(side) for side in core.hand_sides},
            hand_commands=core.last_hand_commands,
            execution_epoch=core.producer.guard.execution_epoch,
            coordinator_receipt=core.coordinator.last_bilateral_receipt,
            bilateral_command=core.coordinator.last_bilateral_command,
            received_timestamp_ns=now,
        )


class LiveCapture:
    def __init__(self, recorder, *, run_id, clock=time.monotonic_ns):
        self.recorder, self.run_id, self.clock = recorder, run_id, clock
        self._rawviz_sequence = 0

    def _recorder_failed(self):
        return getattr(self.recorder, 'failure', None)

    def _append(self, method, *args, owned=False, **kwargs):
        """Avoid secondary callback errors after a recording has failed."""
        if self._recorder_failed():
            return False
        append = getattr(self.recorder, 'append_owned', None) if owned else None
        if append is None:
            append = self.recorder.append
        try:
            append(method, *args, **kwargs)
        except RuntimeError:
            if self._recorder_failed():
                return False
            raise
        return True

    def audit(self, kind, payload, timestamp_ns=None):
        return self._append('append_dual_audit', kind, dict(payload, run_id=self.run_id),
                            received_timestamp_ns=self.clock() if timestamp_ns is None else timestamp_ns)

    def _owned_append(self, method, *args, **kwargs):
        return self._append(method, *args, owned=True, **kwargs)

    def _owned_audit(self, kind, payload, timestamp_ns):
        self._owned_append('append_dual_audit', kind, dict(payload, run_id=self.run_id),
                           received_timestamp_ns=timestamp_ns)

    def tjvr(self, frame):
        return self._append('append_raw_reference_tjvr', frame)

    def tjvr_decision(self, observation, decision, generation):
        frame = observation.frame
        self.audit('tjvr_stream_decision', dict(version=1,
            receiver_instance_id=frame.receiver_instance_id,
            receiver_frame_sequence=frame.receiver_frame_sequence,
            received_timestamp_ns=frame.received_timestamp_ns,
            accepted=decision.accepted, reason=decision.reason,
            epoch_changed=decision.epoch_changed,
            stream_discontinuity=decision.stream_discontinuity,
            resynchronization_generation=generation), frame.received_timestamp_ns)

    def rawviz(self, line, timestamp_ns):
        self._rawviz_sequence += 1
        self.audit('manus_rawviz_line', dict(line_sequence=self._rawviz_sequence,
            text=line, terminator='LF', input_stage='rawviz_stdout_before_parser'), timestamp_ns)

    def callback(self, row):
        side = next(iter(row.source_sequences)) if len(row.points) == 63 else 'right'
        self._append('append_manus_callback', row.points, callback_sequence=row.sequence,
            received_timestamp_ns=row.received_timestamp_ns, receiver_instance_id=row.receiver_instance_id,
            single_hand_side=side)
        self.audit('manus_callback_metadata', dict(callback_sequence=row.sequence,
            receiver_instance_id=row.receiver_instance_id, source_sequences=row.source_sequences,
            source_timestamps_ns=row.source_timestamps_ns), row.received_timestamp_ns)

    def hand_output(self, commands):
        now = self.clock()
        for command in commands.values():
            self._append('append_hand_command', command, received_time_ns=now)
        self.audit('hand_output', dict(stage='producer_authorized_before_executor',
            commands={side: command.to_dict() for side, command in commands.items()}), now)

    def cycle(self, core, result):
        return self.cycle_snapshot(LiveCycleSnapshot.from_core(core, result))

    def cycle_snapshot(self, snapshot: LiveCycleSnapshot) -> None:
        """Record a control snapshot without reading mutable live core state."""
        if not isinstance(snapshot, LiveCycleSnapshot):
            raise TypeError('snapshot must be LiveCycleSnapshot')
        if self._recorder_failed():
            return
        batch = getattr(self.recorder, 'append_live_cycle_snapshot', None)
        if batch is not None:
            try:
                batch(snapshot, run_id=self.run_id)
            except RuntimeError:
                if not self._recorder_failed():
                    raise
            return
        now = snapshot.received_timestamp_ns
        if snapshot.hand_telemetry:
            self._owned_audit('component_status', snapshot.hand_telemetry, now)
        for command in snapshot.result.commands.values():
            self._owned_append('append_arm_command', command, received_time_ns=now)
        self._owned_append('append_arm_state', snapshot.arm_state, received_time_ns=now)
        self._owned_append('append_session_state', snapshot.session_state, received_time_ns=now)
        for state in snapshot.hand_states.values():
            self._owned_append('append_hand_state', state, received_time_ns=now)
        self._owned_audit('native_cycle', dict(execution_epoch=snapshot.execution_epoch,
            native=snapshot.result.native_result, native_attempt=snapshot.result.native_attempt,
            coordinator_receipt=snapshot.coordinator_receipt,
            receipt_accepted=snapshot.result.receipt_accepted,
            bilateral_command=snapshot.bilateral_command,
            accepted_hand_commands={s: c.to_dict() for s, c in snapshot.hand_commands.items()},
            source_status=snapshot.source_status.to_dict()), now)
