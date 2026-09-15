import ast
import importlib.util
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which('c++'), 'C++ required')
class NativeHandFilterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        assert importlib.util.find_spec('tianji_teleop.producers.native_hand_filter'), 'native hand filter missing'
        cls.temp = tempfile.TemporaryDirectory(prefix='hand-filter-test-')
        cls.addClassCleanup(cls.temp.cleanup)
        cls.library = Path(cls.temp.name) / 'filter.so'
        subprocess.run(['c++', '-std=c++17', '-O2', '-shared', '-fPIC', '-ffp-contract=off',
            '-Wall', '-Wextra', '-Werror', str(ROOT/'native/hand/lowpass.cpp'), '-o', str(cls.library)], check=True)

    def test_pinned_filter_parity_reset_and_independent_sides(self):
        from tianji_teleop.producers.native_hand_filter import NativeHandLPFilter
        source = ROOT/'third_party/wuji_hand_retargeting/wuji_retargeting/opt/base.py'
        tree = ast.parse(source.read_text())
        klass = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'LPFilter')
        namespace = {'np': np}
        exec(compile(ast.Module(body=[klass], type_ignores=[]), str(source), 'exec'), namespace)
        rng = np.random.default_rng(24)
        for alpha in (.01, .2, .5, 1.):
            reference = [namespace['LPFilter'](alpha) for _ in range(2)]
            actual = [NativeHandLPFilter(alpha, library=self.library) for _ in range(2)]
            for native in actual: self.addCleanup(native.close)
            for i in range(300):
                side = i % 2
                if i % 37 == 0: reference[side].reset(); actual[side].reset()
                x = rng.normal(size=20).astype(np.float32 if i % 53 < 40 else np.float64)
                result = actual[side].next(x)
                expected = reference[side].next(x)
                self.assertEqual(result.dtype, expected.dtype)
                np.testing.assert_allclose(result, expected, rtol=0, atol=1e-15)
                result[:] = 999  # Returned storage must not alias native state.

    def test_invalid_input_does_not_mutate_state_and_close_is_idempotent(self):
        from tianji_teleop.producers.native_hand_filter import NativeHandLPFilter
        for alpha in (0, -1, 1.1, float('nan')):
            with self.assertRaises(ValueError): NativeHandLPFilter(alpha, library=self.library)
        native = NativeHandLPFilter(.2, library=self.library)
        self.addCleanup(native.close)
        native.next(np.ones(20))
        for value in (np.zeros(19), np.zeros((2, 10)), np.full(20, np.nan), np.full(20, np.inf)):
            with self.assertRaises(ValueError): native.next(value)
        np.testing.assert_allclose(native.next(np.zeros(20)), .8)
        native.close(); native.close()
        with self.assertRaises(RuntimeError): native.next(np.zeros(20))

    def test_actual_worker_filter_backend_parity(self):
        import os
        import json
        python = ROOT/'tools/wuji_hand_native/.pixi/envs/default/bin/python'
        if not python.is_file(): self.skipTest('pinned hand environment required')
        points = [0., 0., 0.]
        for finger in range(5):
            for joint in range(4): points += [.02*(finger-2), .02*(joint+1), .002*joint]
        rows = [dict(schema_version=1, kind='wuji_hand_input', callback_sequence=i,
                     timestamp_ns=100+i, points=points+points) for i in (1, 2, 4, 5)]
        payload = ''.join(json.dumps(row)+'\n' for row in rows)
        results = []
        for backend in ('python', 'cpp'):
            environment = {k:v for k,v in os.environ.items() if k not in ('PYTHONPATH','PYTHONHOME','LD_LIBRARY_PATH','LD_PRELOAD')}
            environment['TIANJI_HAND_FILTER_BACKEND'] = backend
            result = subprocess.run([str(python), str(ROOT/'scripts/wuji_hand_worker.py'),
                '--filter-library', str(self.library)], input=payload, env=environment,
                text=True, capture_output=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stderr)
            results.append([json.loads(line) for line in result.stdout.splitlines()])
        for left, right in zip(*results):
            for side in ('left', 'right'):
                np.testing.assert_allclose(left[side]['position_rad'], right[side]['position_rad'], atol=1e-10, rtol=0)
            self.assertEqual(left['callback_sequence'], right['callback_sequence'])
        self.assertEqual(len(results[0]), 4); self.assertEqual(len(results[1]), 4)

    def test_cpp_filter_provenance_is_recorded_for_pico(self):
        import json
        from unittest.mock import patch
        from tianji_teleop.recording.session_recorder import _session_metadata
        from tianji_teleop.producers import native_hand_filter as module
        env = dict(TIANJI_RUN_ID='offline', TIANJI_HAND_FILTER_BACKEND='cpp',
            TIANJI_RESOLVED_DUAL_SESSION=json.dumps(dict(profile='pico2_hands_sim',
                config=dict(input_mode='pico2_hands', active_hand_sides=['left', 'right']))))
        with patch.object(module, 'library_path', return_value=self.library), patch(
                'tianji_teleop.recording.hand_command_check.pico_hand_replay_asset_hashes', return_value={}):
            result = _session_metadata('pico2_hands_sim', env)
        self.assertEqual(result['hand_filter']['backend'], 'cpp')
        self.assertEqual(len(result['hand_filter']['library_sha256']), 64)
