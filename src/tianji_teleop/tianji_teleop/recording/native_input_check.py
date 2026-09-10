"""Cross-check recorded SPARK consumption without replaying control or IK."""
from ..hand_tracking.reference_tjvr_receiver import ReceivedTjvrFrame


def check_native_inputs(audits, accepted_frames, *, run_id):
    """accepted_frames maps receiver ordinal to the reconstructed gate envelope.

    Does not infer latest-frame scheduling. Null sample ticks remain null and
    may advance native dynamics; checking those dynamics is outside this module.
    """
    rows = [(index, row) for index, row in enumerate(audits) if row['kind'] == 'native_cycle']
    gate_positions = {row['payload']['receiver_frame_sequence']: index
        for index, row in enumerate(audits)
        if row['kind'] == 'tjvr_stream_decision' and row['payload']['accepted']}
    report = dict(passed=True, first_difference=None, native_input_check='not_recorded',
        native_attempts=0, matched_native_inputs=0, native_ticks_without_new_input=0,
        limitations=['does not prove latest-frame scheduling or replay native dynamics/reset/authorization'])

    def difference(index, field):
        report['passed'] = False
        if report['first_difference'] is None:
            report['first_difference'] = dict(stage='native_input', native_cycle_index=index, field=field)

    if not rows:
        return report
    if all('native_attempt' not in row['payload'] for _, row in rows):
        report['native_input_check'] = 'unavailable_legacy_audit'
        report['limitations'].append('legacy cycles have no consumed sample boundary; not inferred from raw')
        return report
    report['native_input_check'] = 'no_native_attempts'
    previous_epoch, previous_tick, previous_time, previous_input = 0, 0, 0, 0
    for index, (audit_index, row) in enumerate(rows):
        payload = row['payload']
        if payload.get('run_id') != run_id:
            difference(index, 'run_id')
        if 'native_attempt' not in payload:
            difference(index, 'missing_native_attempt')
            continue
        attempt = payload['native_attempt']
        if attempt is None:
            continue
        report['native_input_check'] = 'checked'
        report['native_attempts'] += 1
        if not isinstance(attempt, dict) or set(attempt) != {'tick_id', 'timestamp_ns', 'sample'}:
            difference(index, 'native_attempt_fields')
            continue
        epoch, tick, stamp = payload.get('execution_epoch'), attempt['tick_id'], attempt['timestamp_ns']
        if any(type(value) is not int or not 0 < value < 2**63 for value in (epoch, tick, stamp)):
            difference(index, 'native_attempt_identity_or_clock')
            continue
        if epoch < previous_epoch or tick != (previous_tick + 1 if epoch == previous_epoch else 1):
            difference(index, 'execution_epoch_or_tick_order')
        if stamp <= previous_time or stamp > row['received_timestamp_ns']:
            difference(index, 'native_attempt_timestamp_ns')
        previous_epoch, previous_tick, previous_time = epoch, tick, stamp
        sample = attempt['sample']
        if sample is None:
            report['native_ticks_without_new_input'] += 1
            continue
        try:
            received = ReceivedTjvrFrame.from_dict(sample)
        except (ValueError, TypeError, KeyError):
            difference(index, 'invalid_sample_envelope')
            continue
        frame = received.observation.frame
        sequence = frame.receiver_frame_sequence
        valid = True
        if sequence <= previous_input:
            difference(index, 'reused_or_reordered_consumed_input')
            valid = False
        previous_input = sequence
        if frame.received_timestamp_ns > stamp:
            difference(index, 'input_received_after_native_attempt')
            valid = False
        expected = accepted_frames.get(sequence)
        if expected is None:
            difference(index, 'sample_not_gate_accepted')
            continue
        if gate_positions.get(sequence, len(audits)) >= audit_index:
            difference(index, 'sample_gate_decision_after_native_cycle')
            valid = False
        for field, value in expected.items():
            if type(sample.get(field)) is not type(value) or sample.get(field) != value:
                difference(index, 'sample.' + field)
                valid = False
        report['matched_native_inputs'] += int(valid)
    return report
