import ast
import importlib.util
from pathlib import Path
import subprocess
import shutil
import tempfile
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
EIGEN = next((p for p in (ROOT/'.pixi/envs/ik-build/include/eigen3',
    ROOT/'tools/wuji_hand_native/.pixi/envs/default/include/eigen3',
    ROOT/'.pixi/envs/default/include/eigen3') if (p/'Eigen/SVD').is_file()), None)


@unittest.skipUnless(shutil.which('c++') and EIGEN, 'C++ and project Eigen required')
class NativeHandGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        assert importlib.util.find_spec('tianji_teleop.producers.native_hand_geometry'), 'native geometry missing'
        cls.temp = tempfile.TemporaryDirectory(prefix='hand-geometry-')
        cls.addClassCleanup(cls.temp.cleanup)
        cls.library = Path(cls.temp.name)/'geometry.so'
        subprocess.run(['c++', '-std=c++17', '-O2', '-shared', '-fPIC', '-ffp-contract=off',
            '-I'+str(EIGEN), '-Wall', '-Wextra', '-Werror',
            str(ROOT/'native/hand/geometry.cpp'), '-o', str(cls.library)], check=True)

    def reference(self, side, signs, rotation, wrist, thumb):
        from scipy.spatial.transform import Rotation
        source = ROOT/'third_party/wuji_hand_retargeting/wuji_retargeting/mediapipe.py'
        namespace = {}
        exec(compile(source.read_text(), str(source), 'exec'), namespace)
        source = ROOT/'third_party/wuji_hand_retargeting/wuji_retargeting/retarget.py'
        tree = ast.parse(source.read_text())
        klass = next(x for x in tree.body if isinstance(x, ast.ClassDef))
        klass.body = [x for x in klass.body if isinstance(x, ast.FunctionDef) and
                      x.name in ('_prepare_keypoints', '_apply_rotation', '_apply_offset')]
        exec(compile(ast.Module(body=[klass], type_ignores=[]), str(source), 'exec'), namespace)
        obj = namespace['Retargeter']()
        obj.hand_side, obj.input_axis_sign = side, np.asarray(signs)
        obj.rotation_xyz = dict(zip(('x','y','z'), rotation))
        obj.wrist_offset_m, obj.thumb_offset_m = np.array(wrist), np.array(thumb)
        obj._has_offset = True
        return obj, Rotation.from_euler('xyz', rotation, degrees=True).as_matrix()

    def test_reference_geometry_both_sides_and_reflections(self):
        from tianji_teleop.producers.native_hand_geometry import NativeHandGeometry
        rng = np.random.default_rng(25)
        for side in ('left', 'right'):
            for signs in ([1,1,1], [-1,1,1], [1,-1,-1]):
                ref, rotation = self.reference(side, signs, [30,-20,90], [.01,.02,-.03], [-.01,0,.03])
                native = NativeHandGeometry(side, signs, rotation, ref.wrist_offset_m,
                                            ref.thumb_offset_m, library=self.library)
                for _ in range(100):
                    points = rng.normal(size=(21,3))*.03 + rng.normal(size=3)
                    np.testing.assert_allclose(native(points), ref._prepare_keypoints(points), atol=2e-13, rtol=0)

    def test_reject_invalid_and_degenerate_without_poisoning_next_frame(self):
        from tianji_teleop.producers.native_hand_geometry import NativeHandGeometry
        native = NativeHandGeometry('right', [1,1,1], np.eye(3), [0]*3, [0]*3, library=self.library)
        for points in (np.zeros((21,3)), np.full((21,3), np.nan), np.zeros((20,3)),
                       np.repeat(np.arange(21)[:,None], 3, axis=1)):
            with self.assertRaises(ValueError): native(points)
        points = np.random.default_rng(1).normal(size=(21,3))
        self.assertTrue(np.isfinite(native(points)).all())

    def test_worker_backend_and_recording_provenance(self):
        import json
        import os
        from unittest.mock import patch
        from tianji_teleop.recording.session_recorder import _session_metadata
        from tianji_teleop.producers import native_hand_geometry as module
        env = dict(TIANJI_RUN_ID='offline', TIANJI_HAND_GEOMETRY_BACKEND='cpp',
            TIANJI_RESOLVED_DUAL_SESSION=json.dumps(dict(profile='pico2_hands_sim',
                config=dict(input_mode='pico2_hands', active_hand_sides=['left','right']))))
        with patch.object(module, 'library_path', return_value=self.library), patch(
                'tianji_teleop.recording.hand_command_check.pico_hand_replay_asset_hashes', return_value={}):
            metadata = _session_metadata('pico2_hands_sim', env)
        self.assertEqual(metadata['hand_geometry']['backend'], 'cpp')
        self.assertEqual(len(metadata['hand_geometry']['library_sha256']), 64)
        python = ROOT/'tools/wuji_hand_native/.pixi/envs/default/bin/python'
        if not python.is_file(): self.skipTest('pinned hand environment required')
        points = [[0.,0.,0.]]
        for finger in range(5):
            for joint in range(4): points.append([.02*(finger-2), .02*(joint+1), .002*joint])
        rows = []
        for i in (1,2,3,5,6,7):
            value = np.array(points)
            value[4::4, 2] += .001*i
            rows.append(dict(schema_version=1, kind='wuji_hand_input', callback_sequence=i,
                timestamp_ns=100+i, points=list(value.ravel())*2))
        payload = ''.join(json.dumps(row)+'\n' for row in rows)
        results = []
        for backend in ('python','cpp'):
            environment = {k:v for k,v in os.environ.items() if k not in
                ('PYTHONPATH','PYTHONHOME','LD_LIBRARY_PATH','LD_PRELOAD','TIANJI_HAND_FILTER_BACKEND')}
            environment['TIANJI_HAND_GEOMETRY_BACKEND'] = backend
            result = subprocess.run([str(python), str(ROOT/'scripts/wuji_hand_worker.py'),
                '--geometry-library', str(self.library)], input=payload, env=environment,
                text=True, capture_output=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stderr)
            results.append([json.loads(line) for line in result.stdout.splitlines()])
        self.assertEqual([len(x) for x in results], [6,6])
        for a,b in zip(*results):
            self.assertEqual(a['callback_sequence'], b['callback_sequence'])
            for side in ('left','right'):
                np.testing.assert_allclose(a[side]['position_rad'], b[side]['position_rad'], atol=1e-7, rtol=0)


if __name__ == '__main__': unittest.main()
