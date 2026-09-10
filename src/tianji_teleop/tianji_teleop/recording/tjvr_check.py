"""Read-only TJVR raw-to-stream-decision reconstruction, not IK replay."""
import math

from ..hand_tracking.reference_tjvr import parse_reference_tjvr_packet
from ..hand_tracking.reference_tjvr_stream import ReferenceTjvrStreamGate
from ..hand_tracking.reference_tjvr_receiver import ReceivedTjvrFrame
from .native_input_check import check_native_inputs
from .session_h5 import SessionH5Reader


def check_tjvr_recording(path):
    with SessionH5Reader(path) as reader:
        raw = reader.read_raw_reference_tjvr()
        all_audits = reader.read_dual_audit()
        audits = [row for row in all_audits if row['kind'] == 'tjvr_stream_decision']
        metadata = reader.read_hand_tracking_metadata()
    configuration = metadata.get('resolved_configuration', {})
    contract = configuration.get('tjvr_stream_contract') if isinstance(configuration, dict) else None
    if (not isinstance(contract, dict) or type(contract.get('version')) is not int or
            contract['version'] != 1 or contract.get('initial_state') != 'reset'):
        raise ValueError('recorded reset-state tjvr_stream_contract version 1 required')
    thresholds = [contract.get(name) for name in ('max_position_jump_m', 'max_orientation_jump_rad')]
    if any(type(value) not in (int, float) or not math.isfinite(value) or value <= 0 for value in thresholds):
        raise ValueError('recorded TJVR jump thresholds must be finite and positive')
    run_id = metadata.get('run_id')
    if not isinstance(run_id, str) or not run_id.strip():
        raise ValueError('recorded run_id required')
    if not raw or not audits:
        raise ValueError('raw TJVR and recorded stream decisions both required; cannot infer missing history')
    gate = ReferenceTjvrStreamGate(*thresholds)
    report = dict(passed=True, scope='tjvr_raw_to_stream_decision_consistency',
        raw_frames=len(raw), recorded_decisions=len(audits), matched_decisions=0,
        accepted=0, rejected=0, resynchronizations=0, first_difference=None,
        operator_events_executed=0, complete_plan_acceptance=False,
        limitations=['uses installed gate, not an independent original algorithm oracle',
            'no mapping, IK, authorization or actuator replay',
            'malformed datagrams are not recorded; receiver ordinals may have gaps',
            'cannot prove completeness if raw and matching audits were both removed'])

    def difference(index, field):
        report['passed'] = False
        if report['first_difference'] is None:
            report['first_difference'] = dict(raw_index=index, field=field)

    if len(raw) != len(audits):
        difference(min(len(raw), len(audits)), 'decision_count')
    receiver = raw[0]['receiver_instance_id']
    previous_sequence, previous_time, generation = 0, 0, 0
    accepted_frames = {}
    for index, row in enumerate(raw):
        sequence, stamp = row['receiver_frame_sequence'], row['received_timestamp_ns']
        if (row['receiver_instance_id'] != receiver or sequence <= previous_sequence or
                stamp < previous_time or stamp <= 0):
            raise ValueError('TJVR raw must preserve one receiver and strictly increasing receive ordinals')
        previous_sequence, previous_time = sequence, stamp
        observation = parse_reference_tjvr_packet(row['raw_packet'], receiver_instance_id=receiver,
            receiver_frame_sequence=sequence, received_timestamp_ns=stamp)
        decision = gate.evaluate(observation.frame)
        generation += int(decision.stream_discontinuity)
        if decision.accepted:
            accepted_frames[sequence] = ReceivedTjvrFrame(
                observation, decision.stream_discontinuity, generation).to_dict()
        report['accepted' if decision.accepted else 'rejected'] += 1
        report['resynchronizations'] = generation
        expected = dict(version=1, run_id=run_id, receiver_instance_id=receiver,
            receiver_frame_sequence=sequence, received_timestamp_ns=stamp,
            accepted=decision.accepted, reason=decision.reason, epoch_changed=decision.epoch_changed,
            stream_discontinuity=decision.stream_discontinuity, resynchronization_generation=generation)
        if index >= len(audits):
            continue
        audit = audits[index]
        actual = audit['payload']
        if set(actual) != set(expected):
            difference(index, 'decision_fields')
        equal = set(actual) == set(expected)
        for field, value in expected.items():
            if type(actual.get(field)) is not type(value) or actual.get(field) != value:
                difference(index, field)
                equal = False
        if audit['received_timestamp_ns'] != stamp:
            difference(index, 'audit_received_timestamp_ns')
            equal = False
        report['matched_decisions'] += int(equal)
    if report['passed']:
        native = check_native_inputs(all_audits, accepted_frames, run_id=run_id)
        report['limitations'].extend(native.pop('limitations'))
        report.update(native)
    else:
        report['native_input_check'] = 'not_checked_gate_difference'
    return report
