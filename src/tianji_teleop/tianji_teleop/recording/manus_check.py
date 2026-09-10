"""Offline rawviz-to-Manus21 callback reconstruction, without device/control I/O."""
import math

import numpy as np

from ..hand_tracking.reference_manus import HandInputAssembler, RawvizHandInputProcessor
from .session_h5 import SessionH5Reader


def check_manus_recording(path, *, atol=1e-9):
    """Require recorded parser bindings; never guess them from observed hands.

    Reconstruct original per-POSE callbacks with recorded receive times. This
    verifies recording consistency, not an independent retarget/IK oracle.
    """
    if isinstance(atol, bool) or not math.isfinite(atol) or atol < 0:
        raise ValueError('atol must be finite and nonnegative')
    with SessionH5Reader(path) as reader:
        configuration = reader.read_hand_tracking_metadata().get('resolved_configuration', {})
        if not isinstance(configuration, dict):
            raise ValueError('invalid recorded resolved_configuration')
        contract = configuration.get('manus_input_contract')
        callbacks = reader.read_manus_callbacks()
        audits = reader.read_dual_audit()
    if not isinstance(contract, dict) or contract.get('version') != 1:
        raise ValueError('recorded manus_input_contract version 1 required; cannot infer glove bindings')
    sides = contract.get('sides')
    if (sides not in (['right'], ['left'], ['right', 'left']) or
            contract.get('callback_order') != 'right_then_left' or
            contract.get('callback_trigger') != 'each_accepted_pose'):
        raise ValueError('unsupported or disabled recorded Manus parser contract')
    for field in ('right_glove', 'left_glove'):
        if field not in contract or (contract[field] is not None and
                (not isinstance(contract[field], str) or not contract[field].strip())):
            raise ValueError('invalid recorded glove binding')
    if contract['right_glove'] is not None and contract['right_glove'] == contract['left_glove']:
        raise ValueError('recorded glove bindings must differ')
    lines = [row for row in audits if row['kind'] == 'manus_rawviz_line']
    metadata = [row for row in audits if row['kind'] == 'manus_callback_metadata']
    if not lines or not callbacks:
        raise ValueError('rawviz lines and Manus callbacks both required; callback-only is not raw replay')
    rebuilt = []
    now = 0

    def publish(frame):
        rebuilt.append(dict(callback_sequence=len(rebuilt) + 1,
            received_timestamp_ns=now, points=frame.values.tolist(),
            source_sequences=frame.sequences, source_timestamps_ns=frame.source_timestamps_ns))

    processor = RawvizHandInputProcessor(
        HandInputAssembler('right' in sides, 'left' in sides), publish,
        right_glove=contract['right_glove'], left_glove=contract['left_glove'])
    run_ids = set()
    for sequence, row in enumerate(lines, 1):
        payload = row['payload']
        text = payload.get('text')
        if (payload.get('line_sequence') != sequence or payload.get('terminator') != 'LF' or
                payload.get('input_stage') != 'rawviz_stdout_before_parser' or
                not isinstance(text, str) or '\n' in text or len(text.encode('utf-8')) > 65536 or
                row['received_timestamp_ns'] < now):
            raise ValueError(f'invalid, missing or reordered rawviz line at {sequence}')
        run_id = payload.get('run_id')
        if not isinstance(run_id, str) or not run_id.strip():
            raise ValueError('rawviz capture requires an explicit run identity')
        run_ids.add(run_id)
        now = row['received_timestamp_ns']
        processor.process_line(text)
    if len(run_ids) != 1 or None in run_ids:
        raise ValueError('rawviz capture must have one explicit run identity')
    report = dict(passed=True, scope='manus_rawviz_to_callback_consistency',
        raw_lines=len(lines), rebuilt_callbacks=len(rebuilt), recorded_callbacks=len(callbacks),
        matched_callbacks=0, first_difference=None, max_absolute_error=0., atol=atol,
        operator_events_executed=0,
        limitations=['uses installed original parser, not an independent algorithm oracle',
                     'no retarget, TJVR/IK, operator or actuator replay'])

    def difference(sequence, field):
        report['passed'] = False
        if report['first_difference'] is None:
            report['first_difference'] = dict(stage='manus_callback',
                                             callback_sequence=sequence, field=field)

    if len(rebuilt) != len(callbacks):
        difference(min(len(rebuilt), len(callbacks)) + 1, 'callback_count')
    if len(metadata) != len(callbacks):
        difference(min(len(metadata), len(callbacks)) + 1, 'metadata_count')
    identity = callbacks[0]['receiver_instance_id']
    for expected, actual, meta in zip(rebuilt, callbacks, metadata):
        sequence = expected['callback_sequence']
        for field in ('callback_sequence', 'received_timestamp_ns'):
            if expected[field] != actual[field]:
                difference(sequence, field)
        expected_count = 63 * len(sides)
        if (actual['point_count'] != expected_count or
                actual['single_hand_side'] != ('left' if sides == ['left'] else 'right')):
            difference(sequence, 'hand_shape')
        a, b = np.asarray(expected['points']), np.asarray(actual['points'])
        if a.shape != b.shape:
            difference(sequence, 'points_shape')
        else:
            error = float(np.abs(a - b).max(initial=0))
            report['max_absolute_error'] = max(report['max_absolute_error'], error)
            if error > atol:
                difference(sequence, 'points')
        payload = meta['payload']
        for field in ('callback_sequence', 'source_sequences', 'source_timestamps_ns'):
            if payload.get(field) != expected[field]:
                difference(sequence, field)
        if (meta['received_timestamp_ns'] != expected['received_timestamp_ns'] or
                payload.get('run_id') != next(iter(run_ids)) or
                payload.get('receiver_instance_id') != identity or actual['receiver_instance_id'] != identity):
            difference(sequence, 'callback_metadata_identity_or_clock')
        report['matched_callbacks'] += 1
    return report
