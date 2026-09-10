#!/usr/bin/env python3
"""Read-only staged reference comparison; does not launch solvers or devices.

Provide paired SPARK JSONL and/or official-hand JSON traces. A passing report
applies only to supplied stages, not synchronized acquisition or full migration
acceptance. No operator events are executed. Exit: 0 pass, 1 difference, 2 input error.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.compare_spark_reference import compare_traces


def _validate_hand(row):
    if not isinstance(row, dict):
        raise ValueError('hand trace must be an object')
    contract = dict(kind='original_manus_hand_reference', input_stage='mediapipe21',
        output_order='left20_right20', sequence_policy='one_callback_per_recorded_row_starting_at_1')
    if (type(row.get('schema_version')) is not int or row['schema_version'] != 1 or
            any(row.get(key) != value for key, value in contract.items())):
        raise ValueError('unsupported hand trace contract')
    count = row.get('frames')
    if type(count) is not int or count <= 0:
        raise ValueError('nonempty hand trace required')
    names = row.get('joint_names')
    if (not isinstance(names, list) or len(names) != 40 or
            any(not isinstance(name, str) or not name.startswith('l_' if i < 20 else 'r_')
                for i, name in enumerate(names)) or len(set(names)) != 40):
        raise ValueError('40 distinct left20/right20 hand joint names required')
    digests = row.get('sha256')
    if (not isinstance(digests, dict) or set(digests) != {'input', 'left_config', 'right_config', 'bridge'} or
            any(not isinstance(value, str) or re.fullmatch('[0-9a-f]{64}', value) is None
                for value in digests.values())):
        raise ValueError('invalid hand provenance digests')
    positions, timestamps = row.get('position_rad'), row.get('timestamps_s')
    if (not isinstance(positions, list) or not isinstance(timestamps, list) or
            any(not isinstance(values, list) or any(type(v) not in (int, float) for v in values)
                for values in positions) or any(type(v) not in (int, float) for v in timestamps)):
        raise ValueError('invalid hand trace numbers')
    try:
        q = np.asarray(positions, dtype='<f8')
        times = np.asarray(timestamps, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError('invalid hand trace numbers') from exc
    if (q.shape != (count, 40) or times.shape != (count,) or
            not np.isfinite(q).all() or not np.isfinite(times).all() or
            np.any(times < 0) or np.any(np.diff(times) < 0)):
        raise ValueError('invalid hand frame shape, numbers or timestamp order')
    if hashlib.sha256(q.tobytes()).hexdigest() != row.get('output_sha256'):
        raise ValueError('hand output checksum mismatch')
    return q, times


def compare_hand_records(reference, migrated):
    """Compare the existing official-bridge trace schema at fixed 1e-5 rad."""
    a, ta = _validate_hand(reference)
    b, tb = _validate_hand(migrated)
    if reference['sha256'] != migrated['sha256']:
        raise ValueError('hand provenance mismatch: input/config/bridge')
    if reference['joint_names'] != migrated['joint_names']:
        raise ValueError('hand joint order mismatch')
    report = dict(passed=True, reference_frames=len(a), migrated_frames=len(b),
                  maximum_error_rad=0., tolerance_rad=1e-5, first_divergence=None)

    def fail(index, field, **detail):
        report['passed'] = False
        if report['first_divergence'] is None:
            report['first_divergence'] = dict(callback_sequence=index + 1, field=field, **detail)

    for index, (qa, qb, timestamp_a, timestamp_b) in enumerate(zip(a, b, ta, tb)):
        if timestamp_a != timestamp_b:
            fail(index, 'timestamp_s', reference=float(timestamp_a), migrated=float(timestamp_b))
        error = np.abs(qa - qb)
        maximum = float(error.max())
        if not np.isfinite(maximum):
            raise ValueError('hand difference overflow')
        report['maximum_error_rad'] = max(report['maximum_error_rad'], maximum)
        joints = np.flatnonzero(error > 1e-5)
        if len(joints):
            joint = int(joints[0])
            fail(index, 'position_rad', side='left' if joint < 20 else 'right',
                 joint_name=reference['joint_names'][joint], timestamp_s=float(timestamp_a),
                 error_rad=float(error[joint]))
    if len(a) != len(b):
        fail(min(len(a), len(b)), 'frame_count')
    return report


def compare_hand_traces(reference, migrated):
    with Path(reference).open() as left, Path(migrated).open() as right:
        return compare_hand_records(json.load(left), json.load(right))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for stage in ('spark', 'hand'):
        for role in ('reference', 'migrated'):
            parser.add_argument(f'--{stage}-{role}', type=Path)
    args = parser.parse_args()
    if not any((args.spark_reference, args.hand_reference)):
        parser.error('at least one paired stage is required')
    for stage in ('spark', 'hand'):
        if bool(getattr(args, stage + '_reference')) != bool(getattr(args, stage + '_migrated')):
            parser.error(f'{stage} requires both reference and migrated traces')
    report = dict(passed=True, complete_plan_acceptance=False, synchronized_inputs_verified=False,
        operator_events_executed=0, stages={},
        limitations=['input provenance is compared, not independently authenticated',
                     'SPARK deterministic-test relaxations do not qualify live timing',
                     'XR acquisition, applied calibration, synchronized mapping and hardware unverified'])
    try:
        if args.spark_reference:
            report['stages']['spark_emitted_boundaries'] = compare_traces(args.spark_reference, args.spark_migrated)
        if args.hand_reference:
            report['stages']['official_hand_bridge'] = compare_hand_traces(args.hand_reference, args.hand_migrated)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        report.update(passed=False, error=str(exc))
        print(json.dumps(report, allow_nan=False))
        return 2
    report['passed'] = all(stage['passed'] for stage in report['stages'].values())
    print(json.dumps(report, allow_nan=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
