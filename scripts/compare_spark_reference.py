#!/usr/bin/env python3
"""Compare COMPLETE deterministic SPARK JSONL traces at every emitted boundary.

Fixed tolerances: q/IK/feedforward positions 1e-5 rad; qdot 1e-5 rad/s;
qddot 1e-4 rad/s^2; TCP position 1e-6 m; quaternion geodesic 1e-6 rad;
dimensionless scalars 1e-9. Discrete states/consumed frames must match exactly.
Missing completion records, mismatched provenance and non-finite data fail.
"""
import argparse
import hashlib
from itertools import zip_longest
import json
import math
from pathlib import Path

from scipy.spatial.transform import Rotation


def _invalid_constant(value):
    raise ValueError(f'non-finite JSON value: {value}')


def _json(line):
    return json.loads(line, parse_constant=_invalid_constant)


class Trace:
    def __init__(self, stream):
        self.stream = stream
        self.manifest = _json(stream.readline())
        if self.manifest.get('kind') != 'spark_trace_manifest' or self.manifest.get('schema_version') != 1:
            raise ValueError('unsupported SPARK trace manifest')
        self.footer = None

    def rows(self):
        digest = hashlib.sha256()
        count = 0
        for line in self.stream:
            row = _json(line)
            if row.get('kind') == 'spark_trace_complete':
                if count == 0:
                    raise ValueError('empty SPARK trace cannot establish equivalence')
                if row.get('ticks') != count or row.get('results_sha256') != digest.hexdigest():
                    raise ValueError('SPARK completion count/checksum mismatch')
                if self.stream.read().strip():
                    raise ValueError('trailing data after SPARK completion')
                self.footer = row
                return
            count += 1
            if row.get('kind') != 'spark_bilateral_result' or row.get('tick_id') != count:
                raise ValueError('invalid/nonconsecutive SPARK result')
            digest.update(line.encode())
            yield row
        raise ValueError('incomplete SPARK trace (no completion record)')


def compare_traces(reference: Path, migrated: Path):
    report = dict(passed=True, ticks=0, first_divergence=None, maximum_errors={})

    def fail(tick, field, detail):
        report['passed'] = False
        if report['first_divergence'] is None:
            report['first_divergence'] = dict(tick_id=tick, field=field, detail=detail)

    def compare(a, b, field, tick):
        if field.endswith('.target_quaternion_xyzw'):
            try:
                error = float((Rotation.from_quat(a) * Rotation.from_quat(b).inv()).magnitude())
            except (TypeError, ValueError):
                fail(tick, field, 'invalid quaternion')
                return
            report['maximum_errors'][field] = max(error, report['maximum_errors'].get(field, 0.))
            if error > 1e-6:
                fail(tick, field, f'rotation error {error} rad > 1e-6')
        elif isinstance(a, dict) and isinstance(b, dict):
            if set(a) != set(b):
                fail(tick, field, 'different fields')
            for key in sorted(set(a) & set(b)):
                compare(a[key], b[key], f'{field}.{key}' if field else key, tick)
        elif isinstance(a, list) and isinstance(b, list):
            if len(a) != len(b):
                fail(tick, field, 'different array lengths')
            for left, right in zip(a, b):
                compare(left, right, field, tick)
        elif (type(a) in (int, float) and type(b) in (int, float) and
              field.split('.')[-1] in {
                  'q', 'qdot', 'qddot', 'stage1_q', 'ik_q', 'feedforward_q',
                  'feedforward_qdot', 'feedforward_qddot', 'headroom_scale',
                  'task_scale_position', 'task_scale_orientation', 'target_position'}):
            if not math.isfinite(a) or not math.isfinite(b):
                fail(tick, field, 'non-finite float')
                return
            key = field.split('.')[-1]
            tolerance = (1e-4 if key in {'qddot', 'feedforward_qddot'} else
                         1e-5 if key in {'q', 'qdot', 'stage1_q', 'ik_q', 'feedforward_q', 'feedforward_qdot'} else
                         1e-6 if key == 'target_position' else 1e-9)
            error = abs(a - b)
            report['maximum_errors'][field] = max(error, report['maximum_errors'].get(field, 0.))
            if error > tolerance:
                fail(tick, field, f'error {error} > {tolerance}')
        elif type(a) is not type(b) or a != b:
            fail(tick, field, f'{a!r} != {b!r}')

    with reference.open() as left, migrated.open() as right:
        a, b = Trace(left), Trace(right)
        for key in ('period_ns', 'receive_origin_ns', 'tail_ns', 'scheduling',
                    'deterministic_test', 'model_semantic_sha256', 'model_meshes_sha256'):
            if key not in a.manifest or key not in b.manifest or a.manifest[key] != b.manifest[key]:
                raise ValueError(f'SPARK provenance mismatch: {key}')
        if a.manifest['deterministic_test'] is not True:
            raise ValueError('numerical equivalence requires explicit deterministic test traces')
        for name in ('input', 'config', 'urdf'):
            if a.manifest['files'][name]['sha256'] != b.manifest['files'][name]['sha256']:
                raise ValueError(f'SPARK provenance mismatch: {name}')
        for index, (ra, rb) in enumerate(zip_longest(a.rows(), b.rows()), 1):
            report['ticks'] = index
            if ra is None or rb is None:
                fail(index, 'tick_id', 'different trace lengths')
            else:
                compare(ra, rb, '', index)
        if a.footer['receiver'] != b.footer['receiver']:
            fail(0, 'receiver', 'different input acceptance/consumption counts')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reference', type=Path)
    parser.add_argument('migrated', type=Path)
    args = parser.parse_args()
    report = compare_traces(args.reference, args.migrated)
    print(json.dumps(report, indent=2, allow_nan=False))
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
