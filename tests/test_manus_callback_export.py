import os
from pathlib import Path
import subprocess
import json
import unittest

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get('WUJI_REFERENCE_TEST'), 'optional recorded Manus input')
class ManusCallbackExportTest(unittest.TestCase):
    def test_exact_callback_count_order_and_provenance(self):
        script = ROOT / 'scripts/export_manus_callbacks.py'
        self.assertTrue(script.is_file())
        recording = ROOT / 'vendor/reference_inputs/remote-192.168.110.210-20260908/manus_gjy_dual_20260827.pkl'
        result = subprocess.run([str(ROOT / 'tools/wuji_hand_native/.pixi/envs/default/bin/python'),
            str(script), '--input', str(recording)], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        rows = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual(rows[0]['input_stage'], 'mediapipe21_callback')
        self.assertFalse(rows[0]['synchronized_with_tjvr'])
        self.assertEqual(rows[-1]['frames'], 1339)
        self.assertEqual(len(rows), 1341)
        for index, row in enumerate(rows[1:-1], 1):
            self.assertEqual(row['callback_sequence'], index)
            self.assertEqual(len(row['points']), 126)
