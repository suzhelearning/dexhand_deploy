"""Live boundary adapters; acquisition/control enqueue, one recorder writes."""
import time


class LiveCapture:
    def __init__(self, recorder, *, run_id, clock=time.monotonic_ns):
        self.recorder, self.run_id, self.clock = recorder, run_id, clock
        self._rawviz_sequence = 0

    def audit(self, kind, payload, timestamp_ns=None):
        self.recorder.append('append_dual_audit', kind, dict(payload, run_id=self.run_id),
                             received_timestamp_ns=self.clock() if timestamp_ns is None else timestamp_ns)

    def tjvr(self, frame):
        self.recorder.append('append_raw_reference_tjvr', frame)

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
        self.recorder.append('append_manus_callback', row.points, callback_sequence=row.sequence,
            received_timestamp_ns=row.received_timestamp_ns, receiver_instance_id=row.receiver_instance_id,
            single_hand_side=side)
        self.audit('manus_callback_metadata', dict(callback_sequence=row.sequence,
            receiver_instance_id=row.receiver_instance_id, source_sequences=row.source_sequences,
            source_timestamps_ns=row.source_timestamps_ns), row.received_timestamp_ns)

    def hand_output(self, commands):
        now = self.clock()
        for command in commands.values():
            self.recorder.append('append_hand_command', command, received_time_ns=now)
        self.audit('hand_output', dict(stage='producer_authorized_before_executor',
            commands={side: command.to_dict() for side, command in commands.items()}), now)

    def cycle(self, core, result):
        now = self.clock()
        if core.hand_telemetry:
            self.audit('component_status', core.hand_telemetry, now)
        for command in result.commands.values():
            self.recorder.append('append_arm_command', command, received_time_ns=now)
        self.recorder.append('append_arm_state', core.sim.arm_state, received_time_ns=now)
        self.recorder.append('append_session_state', core.coordinator.state, received_time_ns=now)
        for side in core.hand_sides:
            self.recorder.append('append_hand_state', core.sim.hand_state(side), received_time_ns=now)
        self.audit('native_cycle', dict(execution_epoch=core.producer.guard.execution_epoch,
            native=result.native_result, native_attempt=result.native_attempt,
            coordinator_receipt=core.coordinator.last_bilateral_receipt,
            receipt_accepted=result.receipt_accepted,
            bilateral_command=core.coordinator.last_bilateral_command,
            accepted_hand_commands={s: c.to_dict() for s, c in core.last_hand_commands.items()},
            source_status=core.source_status.to_dict()), now)
