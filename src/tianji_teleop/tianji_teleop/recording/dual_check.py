"""Read-only PICO wire-to-observation consistency check; never executes events.

This uses the installed converter, not an independent algorithm oracle. Older
raw H5 rows lack the receiver clock. For those files receiver timestamps can
only be cross-checked across associated observations; this is reported.
"""
from collections import defaultdict
import math

import numpy as np

from ..hand_tracking.pico import parse_pico_packet
from ..hand_tracking.runtime import pico_frame_observations
from ..hand_tracking.pico_gestures import PicoGestureRuntime, TOPIC, validate_observation
from .session_h5 import SessionH5Reader


def check_pico_recording(path, *, atol=1e-9):
    """Return coverage and first difference; malformed/incomplete files raise.

    Missing and duplicate associations fail closed. No alignment by nearest
    timestamp or interpolation, and no operator/control modules are invoked.
    """
    if isinstance(atol, bool) or not math.isfinite(atol) or atol < 0:
        raise ValueError('atol must be finite and nonnegative')
    with SessionH5Reader(path) as reader:
        router = reader.attrs['router_zid']
        raw = reader.read_raw_pico()
        observations = {
            (stage, side): read(side)
            for stage, read in (
                ('hand_observation', reader.read_hand_observation),
                ('arm_observation', reader.read_arm_input_observation))
            for side in ('left', 'right')
        }
        audit = reader.read_dual_audit()
    if not raw:
        raise ValueError('recording has no raw PICO frames to reconstruct')
    report = dict(passed=True, raw_frames=len(raw), matched_observations=0,
                  raw_receiver_clock_frames=sum('received_timestamp_ns' in row for row in raw),
                  matched_gesture_observations=0, gesture_check='not_recorded',
                  duplicate_observations=0, duplicate_raw_frames=0,
                  missing_observations=0, extra_observations=0,
                  operator_events_executed=0, audit_rows=len(audit),
                  first_difference=None, max_absolute_error=0., atol=atol,
                  scope='pico_wire_to_observation_consistency',
                  limitations=['not an independent algorithm equivalence oracle',
                               'no target, IK, retarget, operator action or actuator replay'])
    if report['raw_receiver_clock_frames'] != len(raw):
        report['limitations'].append('raw receiver clock unavailable; observation clocks cross-checked only')
    gestures = [validate_observation(row['payload']) for row in audit
                if row['kind'] == 'operator_observation']
    if gestures:
        if report['raw_receiver_clock_frames'] != len(raw):
            report['gesture_check'] = 'not_checked_missing_raw_clock'
        else:
            report['gesture_check'] = 'checked'
            observations['gesture_observation', None] = gestures

    def difference(stage, side, frame, field):
        report['passed'] = False
        if report['first_difference'] is None:
            report['first_difference'] = dict(stage=stage, side=side,
                receiver_frame_sequence=frame['receiver_frame_sequence'], field=field)

    def compare(expected, actual, stage, side, frame, prefix=''):
        for field, value in expected.items():
            other = actual.get(field)
            name = prefix + field
            if isinstance(value, dict) and isinstance(other, dict):
                compare(value, other, stage, side, frame, prefix=name + '.')
                continue
            if isinstance(value, (list, np.ndarray, float)):
                a, b = np.asarray(value), np.asarray(other)
                equal = a.shape == b.shape and b.dtype.kind in 'biuf'
                if equal:
                    error = np.abs(a.astype(float) - b.astype(float))
                    equal = bool(np.isfinite(error).all())
                    if equal:
                        maximum = float(error.max(initial=0))
                        report['max_absolute_error'] = max(report['max_absolute_error'], maximum)
                        equal = maximum == 0 if a.dtype.kind == 'b' else maximum <= atol
            else:
                equal = value == other
            if not equal:
                difference(stage, side, frame, name)

    indices = {}
    for key, rows in observations.items():
        index = defaultdict(list)
        for row in rows:
            index[row['frame_association_id']].append(row)
        indices[key] = index
        report['duplicate_observations'] += sum(max(0, len(items) - 1) for items in index.values())
    seen = set()
    runtimes = {}
    generated_gesture = []

    def capture_gesture(topic, value):
        if topic == TOPIC:
            generated_gesture[:] = [value]

    for row in raw:
        identity = row['association_id']
        if identity in seen:
            report['duplicate_raw_frames'] += 1
            difference('raw', None, row, 'duplicate_association')
            continue
        seen.add(identity)
        frame = parse_pico_packet(row['raw_packet'],
            receiver_instance_id=row['receiver_instance_id'],
            connection_generation=row['connection_generation'],
            receiver_frame_sequence=row['receiver_frame_sequence'],
            received_timestamp_ns=row.get('received_timestamp_ns', 0))
        metadata = {key: getattr(frame, key) for key in (
            'association_id', 'source_timestamp_ns', 'source_timestamp_ms',
            'protocol_version', 'flags', 'joint_count', 'head_valid', 'head_pose')}
        compare(metadata, row, 'raw', None, row)
        for side, hand in frame.hands.items():
            compare(dict(valid=hand.valid, wrist_valid=hand.wrist_valid,
                         wrist_pose=hand.wrist_pose,
                         joint_valid=[joint.valid for joint in hand.joints],
                         joint_poses=[joint.pose for joint in hand.joints],
                         joint_radii_m=[joint.radius_m for joint in hand.joints]),
                    row['hands'][side], 'raw_hand', side, row)
        clocks = []
        for side, pair in pico_frame_observations(frame).items():
            for stage, observation in zip(('hand_observation', 'arm_observation'), pair):
                matches = indices[stage, side].get(identity, [])
                if not matches:
                    report['missing_observations'] += 1
                    difference(stage, side, row, 'missing_association')
                    continue
                if len(matches) != 1:
                    difference(stage, side, row, 'duplicate_association')
                    continue
                expected = observation.to_dict()
                expected.pop('side')  # side is the HDF5 group, not a dataset
                if 'received_timestamp_ns' not in row:
                    expected.pop('received_timestamp_ns')
                compare(expected, matches[0], stage, side, row)
                clocks.append(matches[0]['received_timestamp_ns'])
                report['matched_observations'] += 1
        if clocks and (min(clocks) < 0 or len(set(clocks)) != 1):
            difference('observation', None, row, 'received_timestamp_ns')
        if report['gesture_check'] == 'checked':
            source = frame.receiver_instance_id
            if source not in runtimes:
                runtimes[source] = PicoGestureRuntime(publish=capture_gesture,
                    publisher_instance_id=source, router_zid=router)
            generated_gesture.clear()
            runtimes[source].ingest_pico(frame)
            matches = indices['gesture_observation', None].get(identity, [])
            if not matches:
                report['missing_observations'] += 1
                difference('gesture_observation', None, row, 'missing_association')
            elif len(matches) != 1:
                difference('gesture_observation', None, row, 'duplicate_association')
            else:
                compare(generated_gesture[0], matches[0], 'gesture_observation', None, row)
                report['matched_gesture_observations'] += 1
    for (stage, side), index in indices.items():
        for identity, items in index.items():
            if identity not in seen:
                report['extra_observations'] += len(items)
                difference(stage, side, items[0], 'extra_association')
    if any(report[key] for key in ('duplicate_observations', 'duplicate_raw_frames',
                                   'missing_observations', 'extra_observations')):
        report['passed'] = False
    return report
