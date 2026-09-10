"""Actual independent remote hand baseline vs locally migrated source closure.

The recording is MediaPipe21, not raw Manus25 and not synchronized with TJVR.
This gate proves the hand solver bridge only, not device acquisition or timing.
"""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get('WUJI_REFERENCE_TEST'), 'optional official hand environment and baseline')
class WujiReferenceEquivalenceTest(unittest.TestCase):
    def test_real_manus_recording_matches_remote_original_bridge(self):
        assets = ROOT / 'vendor/reference_inputs/remote-192.168.110.210-20260908'
        port = ROOT / 'third_party/wuji_hand_retargeting'
        python = ROOT / 'tools/wuji_hand_native/.pixi/envs/default/bin/python'
        reference = json.loads((assets / 'original_hand_trace.json').read_text())
        result = subprocess.run([str(python), str(ROOT / 'scripts/reference_manus_trace.py'),
            '--repository', str(port), '--input', str(assets / 'manus_gjy_dual_20260827.pkl'),
            '--left-config', str(port / 'example/config/adaptive_analytical_manus_wuji_hand_2_left.yaml'),
            '--right-config', str(port / 'example/config/adaptive_analytical_manus_wuji_hand_2_right.yaml')],
            text=True, capture_output=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stderr)
        migrated = json.loads(result.stdout)
        for key in ('schema_version', 'input_stage', 'output_order', 'sequence_policy',
                    'frames', 'sha256', 'joint_names', 'timestamps_s'):
            self.assertEqual(migrated[key], reference[key], key)
        a, b = (np.asarray(row['position_rad'], dtype='<f8') for row in (reference, migrated))
        self.assertEqual(a.shape, (1339, 40))
        self.assertEqual(b.shape, a.shape)
        self.assertTrue(np.isfinite(a).all() and np.isfinite(b).all())
        for row, q in ((reference, a), (migrated, b)):
            self.assertEqual(hashlib.sha256(q.tobytes()).hexdigest(), row['output_sha256'])
        error = float(np.max(np.abs(a - b)))
        self.assertLessEqual(error, 1e-5)
        print(f'Wuji2 migrated vs original: frames={len(a)}, max_error_rad={error:.17g}, '
              f'output_sha256={migrated["output_sha256"]}')
