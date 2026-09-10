#!/usr/bin/env python3
"""Isolated official Wuji2 retarget worker; bounded stdin/stdout, no devices.

Run with tools/wuji_hand_native's Python. Inputs are the original callback-stage
MediaPipe21 arrays (126 values: right then left; 63: explicitly selected side).
callback_sequence is the bridge callback sequence, NOT the Manus device counter.
This worker emits solver results only; publishing/authorization belongs to the
session producer. Exceptions end this process, never silently restart filters.
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
OFFICIAL = ROOT / 'third_party/wuji_hand_retargeting'
sys.path.insert(0, str(ROOT / 'src/tianji_teleop'))
sys.path.insert(0, str(OFFICIAL / 'example'))
from tianji_teleop.protocol.messages import strict_loads
from tj_wuji2_hand_bridge import OfficialWujiHand2Bridge, HAND2_JOINT_NAMES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--single-hand-side', choices=('left', 'right'), default='right')
    parser.add_argument('--startup-handshake', action='store_true')
    parser.add_argument('--left-config', type=Path,
        default=OFFICIAL / 'example/config/adaptive_analytical_manus_wuji_hand_2_left.yaml')
    parser.add_argument('--right-config', type=Path,
        default=OFFICIAL / 'example/config/adaptive_analytical_manus_wuji_hand_2_right.yaml')
    args = parser.parse_args()
    bridge = OfficialWujiHand2Bridge(OFFICIAL, single_hand_side=args.single_hand_side,
                                     left_config=args.left_config, right_config=args.right_config)
    if args.startup_handshake:
        # The pinned retargeter imports Rotation inside its first callback.
        # Load that dependency before advertising readiness, without feeding
        # fabricated geometry or changing any original filter/optimizer state.
        from scipy.spatial.transform import Rotation  # noqa: F401
        print(json.dumps(dict(schema_version=1, kind='wuji_worker_ready',
                              algorithm='official_wuji_hand2', callbacks=0)), flush=True)
    while True:
        line = sys.stdin.buffer.readline(16385)
        if not line:
            return
        if len(line) > 16384 or not line.endswith(b'\n'):
            raise ValueError('incomplete/oversized official hand request')
        row = strict_loads(line)
        if set(row) != {'schema_version', 'kind', 'callback_sequence', 'timestamp_ns', 'points'}:
            raise ValueError('invalid official hand request fields')
        if type(row['schema_version']) is not int or row['schema_version'] != 1 or row['kind'] != 'wuji_hand_input':
            raise ValueError('unsupported official hand request schema')
        for key in ('callback_sequence', 'timestamp_ns'):
            if type(row[key]) is not int or not 0 < row[key] < 2**63:
                raise ValueError(f'{key} must be positive int64')
        points = row['points']
        if (not isinstance(points, list) or len(points) not in (63, 126) or
                any(type(v) not in (int, float) for v in points)):
            raise ValueError('points must be 63/126 numbers at the MediaPipe21 callback stage')
        # Official split_hand_input checks finite values, bridge owns exactly
        # the original gap reset, retarget, URDF clamp and name permutation.
        frame = bridge.retarget(points, row['callback_sequence'], row['timestamp_ns'])
        result = dict(schema_version=1, kind='wuji_hand_result', algorithm='official_wuji_hand2',
                      callback_sequence=frame.sequence, timestamp_ns=frame.source_timestamp_ns)
        for side in ('left', 'right'):
            result[side] = dict(valid=getattr(frame, side + '_valid'),
                                joint_names=list(HAND2_JOINT_NAMES[side]),
                                position_rad=getattr(frame, side).tolist())
        print(json.dumps(result, allow_nan=False, separators=(',', ':')), flush=True)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'official Wuji2 worker failed: {exc}', file=sys.stderr)
        raise SystemExit(1)
