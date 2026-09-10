import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np


class DualReferenceComparisonTest(unittest.TestCase):
    def trace(self, path, *, error=0., input_digest='a' * 64):
        q = np.zeros((2, 40), dtype='<f8')
        q[1, 23] = error
        row = dict(schema_version=1, kind='original_manus_hand_reference',
            input_stage='mediapipe21', output_order='left20_right20',
            sequence_policy='one_callback_per_recorded_row_starting_at_1',
            frames=2, sha256={key: input_digest if key == 'input' else 'b' * 64
                for key in ('input', 'left_config', 'right_config', 'bridge')},
            output_sha256=hashlib.sha256(q.tobytes()).hexdigest(),
            joint_names=[f'{side}_j{i}' for side in ('l', 'r') for i in range(20)],
            timestamps_s=[0., .01], position_rad=q.tolist())
        path.write_text(json.dumps(row))

    def test_hand_first_divergence_has_callback_side_joint_and_time(self):
        from scripts.compare_dual_input_reference import compare_hand_traces
        with tempfile.TemporaryDirectory() as directory:
            left, right = Path(directory) / 'a.json', Path(directory) / 'b.json'
            self.trace(left)
            self.trace(right, error=.01)
            report = compare_hand_traces(left, right)
            self.assertFalse(report['passed'])
            self.assertEqual(report['first_divergence']['callback_sequence'], 2)
            self.assertEqual(report['first_divergence']['side'], 'right')
            self.assertEqual(report['first_divergence']['joint_name'], 'r_j3')
            self.assertEqual(report['first_divergence']['timestamp_s'], .01)

    def test_hash_and_input_mismatch_cannot_pass(self):
        from scripts.compare_dual_input_reference import compare_hand_traces
        with tempfile.TemporaryDirectory() as directory:
            left, right = Path(directory) / 'a.json', Path(directory) / 'b.json'
            self.trace(left)
            self.trace(right, input_digest='c' * 64)
            with self.assertRaisesRegex(ValueError, 'provenance'):
                compare_hand_traces(left, right)
            self.trace(right)
            row = json.loads(right.read_text())
            row['position_rad'][0][0] = 1.
            right.write_text(json.dumps(row))
            with self.assertRaisesRegex(ValueError, 'checksum'):
                compare_hand_traces(left, right)

    def test_unified_cli_reports_only_verified_stages_not_synchronized_acceptance(self):
        script = Path(__file__).resolve().parents[1] / 'scripts/compare_dual_input_reference.py'
        with tempfile.TemporaryDirectory() as directory:
            left, right = Path(directory) / 'a.json', Path(directory) / 'b.json'
            self.trace(left)
            self.trace(right)
            result = subprocess.run([sys.executable, str(script), '--hand-reference', str(left),
                '--hand-migrated', str(right)], capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stderr)
            report = json.loads(result.stdout)
            self.assertTrue(report['passed'])
            self.assertFalse(report['complete_plan_acceptance'])
            self.assertFalse(report['synchronized_inputs_verified'])
            self.assertEqual(list(report['stages']), ['official_hand_bridge'])
            result = subprocess.run([sys.executable, str(script), '--hand-reference', str(left)],
                capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 2)

    def test_non_numeric_joint_values_do_not_silently_coerce(self):
        from scripts.compare_dual_input_reference import compare_hand_traces
        with tempfile.TemporaryDirectory() as directory:
            left, right = Path(directory) / 'a.json', Path(directory) / 'b.json'
            self.trace(left)
            for value in ('0', False):
                with self.subTest(value=value):
                    self.trace(right)
                    row = json.loads(right.read_text())
                    row['position_rad'][0][0] = value
                    right.write_text(json.dumps(row))
                    with self.assertRaisesRegex(ValueError, 'numbers'):
                        compare_hand_traces(left, right)

    def test_unified_cli_includes_spark_failure_without_hiding_hand_success(self):
        from tests import test_compare_spark_reference as spark_fixture
        script = Path(__file__).resolve().parents[1] / 'scripts/compare_dual_input_reference.py'
        with tempfile.TemporaryDirectory() as directory:
            a, b, c, d = [Path(directory) / name for name in ('a.json', 'b.json', 'c.jsonl', 'd.jsonl')]
            self.trace(a)
            self.trace(b)
            spark_fixture.SparkComparisonTest().trace(c)
            spark_fixture.SparkComparisonTest().trace(d, value=1.1)
            result = subprocess.run([sys.executable, str(script), '--hand-reference', str(a),
                '--hand-migrated', str(b), '--spark-reference', str(c), '--spark-migrated', str(d)],
                capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 1, result.stderr)
            report = json.loads(result.stdout)
            self.assertFalse(report['stages']['spark_emitted_boundaries']['passed'])
            self.assertTrue(report['stages']['official_hand_bridge']['passed'])
