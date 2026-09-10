import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class SparkComparisonTest(unittest.TestCase):
    def comparator(self):
        path = ROOT / 'scripts/compare_spark_reference.py'
        self.assertTrue(path.is_file(), 'SPARK numerical comparator missing')
        spec = importlib.util.spec_from_file_location('spark_compare', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.compare_traces

    def trace(self, path, value=1., complete=True, quaternion=None):
        manifest = dict(kind='spark_trace_manifest', schema_version=1,
            period_ns=5000000, receive_origin_ns=1000000000, tail_ns=250000000,
            scheduling='arrivals_le_tick_then_latest', deterministic_test=True,
            files={name: {'sha256': name} for name in ('input', 'config', 'urdf')},
            model_semantic_sha256='model', model_meshes_sha256='meshes')
        row = dict(kind='spark_bilateral_result', tick_id=1,
                   left={'q': [value] * 7, 'target_quaternion_xyzw': quaternion or [0., 0., 0., 1.]})
        line = json.dumps(row, sort_keys=True) + '\n'
        footer = dict(kind='spark_trace_complete', ticks=1, receiver={'datagrams': 1},
                      results_sha256=hashlib.sha256(line.encode()).hexdigest())
        path.write_text(json.dumps(manifest) + '\n' + line + (json.dumps(footer) + '\n' if complete else ''))

    def test_equal_and_quaternion_sign_equivalence(self):
        compare = self.comparator()
        with tempfile.TemporaryDirectory() as tmp:
            a, b = Path(tmp) / 'a', Path(tmp) / 'b'
            self.trace(a)
            self.trace(b, quaternion=[0., 0., 0., -1.])
            self.assertTrue(compare(a, b)['passed'])

    def test_first_divergence_and_fixed_joint_tolerance(self):
        compare = self.comparator()
        with tempfile.TemporaryDirectory() as tmp:
            a, b = Path(tmp) / 'a', Path(tmp) / 'b'
            self.trace(a)
            self.trace(b, value=1.01)
            result = compare(a, b)
            self.assertFalse(result['passed'])
            self.assertEqual(result['first_divergence']['tick_id'], 1)
            self.assertIn('left.q', result['first_divergence']['field'])

    def test_continuous_numbers_do_not_depend_on_json_integer_spelling(self):
        compare = self.comparator()
        with tempfile.TemporaryDirectory() as tmp:
            a, b = Path(tmp) / 'a', Path(tmp) / 'b'
            self.trace(a, value=1)
            self.trace(b, value=1.0000001)
            self.assertTrue(compare(a, b)['passed'])

    def test_incomplete_or_tampered_trace_cannot_pass(self):
        compare = self.comparator()
        with tempfile.TemporaryDirectory() as tmp:
            a, b = Path(tmp) / 'a', Path(tmp) / 'b'
            self.trace(a)
            self.trace(b, complete=False)
            with self.assertRaises(ValueError):
                compare(a, b)
            self.trace(b)
            b.write_text(b.read_text().replace('"ticks": 1', '"ticks": 2'))
            with self.assertRaisesRegex(ValueError, 'count/checksum'):
                compare(a, b)

    def test_empty_complete_traces_cannot_claim_equivalence(self):
        compare = self.comparator()
        with tempfile.TemporaryDirectory() as tmp:
            a, b = Path(tmp) / 'a', Path(tmp) / 'b'
            self.trace(a)
            manifest = a.read_text().splitlines()[0]
            footer = dict(kind='spark_trace_complete', ticks=0, receiver={},
                          results_sha256=hashlib.sha256(b'').hexdigest())
            empty = manifest + '\n' + json.dumps(footer) + '\n'
            a.write_text(empty)
            b.write_text(empty)
            with self.assertRaisesRegex(ValueError, 'empty'):
                compare(a, b)
