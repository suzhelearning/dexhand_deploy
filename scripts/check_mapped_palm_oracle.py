#!/usr/bin/env python3
"""Compare port against original Viewer's deterministic-source-time joint CSV.

The original Viewer CSV must be produced with the approved bandwidth YAML,
model-state-only, velocity, mapped-palm and --deterministic-pico-replay TRACE.
No hardware, sockets, or actuator publishing. Does not qualify real-time latency.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import struct
import sys
import zlib
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tianji_teleop'))
from tianji_teleop.hand_tracking.tjvr_trace import iter_tjvr_records
from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver
from tianji_teleop.hand_tracking.spark_replay import ReplayTick
from tianji_teleop.hand_tracking.input_modes import MAPPED_PALM_BACKEND
from tianji_teleop.producers.spark.backend_assets import bilateral_assets

PERIOD_NS = 5_000_000
DIAGNOSTIC_COLUMNS = (('headroom_scale', 'pico_ee_headroom_scale'),
                      ('task_scale_position', 'task_scale_position'),
                      ('task_scale_orientation', 'task_scale_orientation'))


def finite_scalar(value):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError('non-finite reference value')
    return result


def validate_reference_rows(rows, duration_ns, telemetry=None):
    """Fail closed before launching a worker; this command checks full traces."""
    if not rows:
        raise ValueError('nonempty reference joint CSV required')
    if (len(rows) - 1) * PERIOD_NS < duration_ns:
        raise ValueError('reference joint CSV does not cover the full trace')
    if telemetry is not None and len(telemetry) != len(rows):
        raise ValueError('reference telemetry/joint CSV lengths differ')
    zero = {field: [0.] * 7 for field in ('q', 'qdot', 'qddot')}
    for tick, row in enumerate(rows):
        if int(row['sequence']) != tick:
            raise ValueError('reference CSV must start at tick zero without missing cycles')
        for side in ('left', 'right'):
            joint_errors(row, zero, side)
        if telemetry is not None:
            diagnostic = telemetry[tick]
            if int(diagnostic['sequence']) != tick:
                raise ValueError('telemetry sequence must match joint CSV')
            for side in ('left', 'right'):
                for _, column in DIAGNOSTIC_COLUMNS:
                    finite_scalar(diagnostic[side + '_' + column])
            for column in ('pico_live', 'pico_tracking_epoch', 'pico_sequence'):
                int(diagnostic[column])


def joint_errors(original, arm, side):
    """Compare controller state only where the original CSV defines it.

    Viewer qddot is a plotting differentiator, reset on every stream reset or
    invalid output. Its invalid zero is not the controller's acceleration.
    Position and velocity remain checked on those ticks.
    """
    errors = {}
    for field in ('q', 'qdot', 'qddot'):
        values = [finite_scalar(original[f'{side}_j{j}_reference_{field}']) for j in range(1, 8)]
        actual = np.asarray(arm[field], dtype=float)
        if actual.shape != (7,) or not np.isfinite(actual).all():
            raise ValueError('invalid native joint vector')
        if field == 'qddot':
            valid = int(original[f'{side}_reference_acceleration_valid'])
            if valid not in (0, 1):
                raise ValueError('invalid reference acceleration validity flag')
            if not valid:
                continue
        errors[field] = finite_scalar(np.max(np.abs(np.asarray(values) - actual)))
    return errors


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trace', required=True, type=Path)
    p.add_argument('--reference-joints', required=True, type=Path)
    p.add_argument('--reference-telemetry', type=Path)
    args = p.parse_args()
    with args.trace.open('rb') as f:
        records = list(iter_tjvr_records(f))
    if not records:
        raise ValueError('nonempty TJVR trace required')
    first = struct.unpack_from('<q', records[0].packet, 24)[0]
    frames = []
    for i, record in enumerate(records):
        packet = bytearray(record.packet)
        relative = struct.unpack_from('<q', packet, 24)[0] - first
        if relative < 0 or (frames and relative < frames[-1][0]):
            raise ValueError('trace source timestamps must be nondecreasing')
        struct.pack_into('<Q', packet, 8, i + 1)
        struct.pack_into('<qq', packet, 24, 1_000_000_000 + relative, 1_000_000_000 + relative)
        struct.pack_into('<I', packet, len(packet)-4, zlib.crc32(packet[:-4]))
        frames.append((relative, bytes(packet)))
    with args.reference_joints.open() as f:
        expected = list(csv.DictReader(f))
    telemetry = None
    if args.reference_telemetry:
        with args.reference_telemetry.open() as f:
            telemetry = list(csv.DictReader(f))
    validate_reference_rows(expected, frames[-1][0], telemetry)
    selected = bilateral_assets(ROOT, MAPPED_PALM_BACKEND)
    receiver = ReferenceTjvrReceiver('oracle', .15, .6, target_source='mapped_corrected_palm')
    index = 0
    maxima = dict(q=0., qdot=0., qddot=0.)
    diagnostic_error = 0.
    first_difference = None
    with selected['client'](**{k: selected[k] for k in ('worker', 'config', 'model', 'urdf')},
                            deterministic_test=True, startup_handshake=True) as worker:
        for tick, original in enumerate(expected):
            if int(original['sequence']) != tick:
                raise ValueError('reference CSV must start at tick zero without missing cycles')
            relative = tick * PERIOD_NS
            while index < len(frames) and frames[index][0] <= relative:
                stamp, packet = frames[index]
                receiver.ingest(packet, 1_000_000_000 + stamp)
                index += 1
            row = worker.step(ReplayTick(tick+1, 1_000_000_000 + relative, receiver.try_read_latest()))
            for side in ('left', 'right'):
                for field, error in joint_errors(original, row[side], side).items():
                    maxima[field] = max(maxima[field], error)
                    if error > 1e-7 and first_difference is None:
                        first_difference = dict(tick=tick, side=side, field=field, error=error)
                if telemetry is not None:
                    for field, column in DIAGNOSTIC_COLUMNS:
                        error = finite_scalar(abs(row[side][field] - finite_scalar(telemetry[tick][side + '_' + column])))
                        diagnostic_error = max(diagnostic_error, error)
                        if error > 1e-7 and first_difference is None:
                            first_difference = dict(tick=tick, side=side, field=field, error=error)
            if telemetry is not None:
                for field, column in (('input_live', 'pico_live'), ('applied_epoch', 'pico_tracking_epoch'),
                                      ('applied_sequence', 'pico_sequence')):
                    if int(row[field]) != int(telemetry[tick][column]) and first_difference is None:
                        first_difference = dict(tick=tick, field=field)
    result = dict(passed=first_difference is None, cycles=len(expected), maximum_absolute_error=maxima,
                  input_frames=len(frames), consumed_frames=index, full_trace_consumed=index == len(frames),
                  maximum_diagnostic_error=diagnostic_error if telemetry is not None else None,
                  first_difference=first_difference, scope='original_viewer_source_time_replay')
    print(json.dumps(result))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
