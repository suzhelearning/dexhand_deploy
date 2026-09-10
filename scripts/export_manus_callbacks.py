#!/usr/bin/env python3
"""Export recorded Manus21 callbacks with pinned NumPy, without retargeting.

No device access. This recording is not raw Manus25 or synchronized with TJVR.
Use the tools/wuji_hand_native Python to decode the original NumPy pickle.
"""
import argparse
import hashlib
import json

import numpy as np
from reference_manus_trace import RecordingUnpickler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True)
    args = parser.parse_args()
    with open(args.input, 'rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
        stream.seek(0)
        rows = RecordingUnpickler(stream).load()
        if stream.read(1):
            raise ValueError('trailing recording bytes')
    if not isinstance(rows, list) or not rows:
        raise ValueError('expected nonempty callback list')
    def emit(row):
        print(json.dumps(row, separators=(',', ':'), allow_nan=False))
    emit(dict(schema_version=1, kind='manus_callback_header', input_stage='mediapipe21_callback',
              order='right21_left21', synchronized_with_tjvr=False, input_sha256=digest))
    previous = 0
    for index, row in enumerate(rows, 1):
        t = float(row['t'])
        if not np.isfinite(t) or t < 0:
            raise ValueError('invalid callback time')
        time_ns = round(t * 1e9)
        if time_ns < previous or time_ns >= 2**63 - 1_000_000_000:
            raise ValueError('callback time rollback or overflow')
        previous = time_ns
        hands = [np.asarray(row[key]) for key in ('right_fingers', 'left_fingers')]
        if any(hand.shape != (21, 3) or not np.isfinite(hand).all() for hand in hands):
            raise ValueError('invalid recorded MediaPipe21 hand')
        emit(dict(kind='manus_callback', callback_sequence=index, relative_receive_ns=time_ns,
                  points=np.concatenate([hand.reshape(-1) for hand in hands]).tolist()))
    emit(dict(kind='manus_callback_complete', frames=len(rows)))


if __name__ == '__main__':
    main()
