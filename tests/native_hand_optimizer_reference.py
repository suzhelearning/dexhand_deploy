"""Compare complete native solve and analytical gradients to pinned original."""
from pathlib import Path
import sys
import unittest
import importlib.util
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/tianji_teleop'))
sys.path.insert(0, str(ROOT/'third_party/wuji_hand_retargeting'))


class OptimizerReferenceTest(unittest.TestCase):
    def test_failed_install_preserves_python_instances(self):
        from tianji_teleop.producers.native_hand_optimizer import install_native_optimizers
        sys.path.insert(0, str(ROOT/'third_party/wuji_hand_retargeting/example'))
        from tj_wuji2_hand_bridge import OfficialWujiHand2Bridge
        official = ROOT/'third_party/wuji_hand_retargeting'
        bridge = OfficialWujiHand2Bridge(official,
            left_config=official/'example/config/adaptive_analytical_manus_wuji_hand_2_left.yaml',
            right_config=official/'example/config/adaptive_analytical_manus_wuji_hand_2_right.yaml')
        originals = {side:r.optimizer for side,r in bridge._retargeters.items()}
        # A failure after constructing the first native side must not partially install it.
        originals['right'].w_hyper = float('nan')
        with self.assertRaises(ValueError): install_native_optimizers(bridge)
        for side, original in originals.items():
            self.assertIs(bridge._retargeters[side].optimizer, original)
            self.assertIsNone(original.last_qpos)
        originals['right'].w_hyper = 0.
        install_native_optimizers(bridge)
        for side, original in originals.items():
            self.assertIsNone(original.last_qpos)
            self.assertIsNot(bridge._retargeters[side].optimizer, original)
            self.addCleanup(bridge._retargeters[side].optimizer.close)

    def test_cost_gradient_and_warm_start(self):
        self.assertIsNotNone(importlib.util.find_spec('tianji_teleop.producers.native_hand_optimizer'))
        from tianji_teleop.producers.native_hand_optimizer import NativeHandOptimizer
        from wuji_retargeting.retarget import Retargeter
        rng = np.random.default_rng(26)
        for side in ('left','right'):
            yaml = ROOT/f'third_party/wuji_hand_retargeting/example/config/adaptive_analytical_manus_wuji_hand_2_{side}.yaml'
            for penalties in (False, True):
                retargeter = Retargeter.from_yaml(str(yaml), side)
                original = retargeter.optimizer
                original.set_timing_enabled(False)
                if penalties:
                    original.w_hyper = .7; original.soft_min = .1
                    original.w_couple = .3; original.couple_ratio = .6
                    original.thumb_skip_pip = True
                native = NativeHandOptimizer(original)
                self.addCleanup(native.close)
                for i in range(20):
                    points = rng.normal(0, .035, size=(21,3))
                    if i%2:
                        points[[8,12,16,20]] = points[4]+rng.normal(0,.012,size=(4,3))
                    q = original.robot.joint_limits.mean(axis=1) + rng.normal(0,.1,20)
                    reg = rng.normal(0,.1,20) if i%2 else None
                    expected = original._loss_and_grad_analytical(q,
                        original._compute_tip_vectors(points, original.scaling),
                        original._compute_tip_dirs(points),
                        original._compute_full_hand_vectors(points, original.segment_scaling),
                        original._compute_pinch_alpha(points), reg)
                    actual = native.loss_and_gradient(q, points, reg)
                    np.testing.assert_allclose(actual[0], expected[0], rtol=1e-12, atol=1e-11)
                    np.testing.assert_allclose(actual[1], expected[1], rtol=1e-11, atol=1e-10)
                # Smooth realistic geometry, reset, and explicit initial/regularization q.
                raw = [[0.,0.,0.]]
                for finger in range(5):
                    for joint in range(4): raw.append([.02*(finger-2), .02*(joint+1), .002*joint])
                for i in range(20):
                    if i == 10: original.last_qpos = None; native.last_qpos = None
                    points = np.array(raw); points[4::4,2] += .0003*i
                    points = retargeter._prepare_keypoints(points)
                    explicit = original.robot.joint_limits.mean(axis=1) if i == 15 else None
                    expected = original.solve(points, explicit)
                    actual = native.solve(points, explicit)
                    self.assertEqual(actual.dtype, np.dtype('float32'))
                    np.testing.assert_allclose(actual, expected, rtol=0, atol=2e-5)
                    np.testing.assert_array_equal(native.last_qpos, actual.astype(np.float64))
                previous = native.last_qpos
                import threading
                errors = []
                def foreign_close():
                    try: native.close()
                    except RuntimeError as exc: errors.append(str(exc))
                thread = threading.Thread(target=foreign_close)
                thread.start(); thread.join()
                self.assertEqual(len(errors), 1, 'foreign thread close must be rejected')
                with self.assertRaises(ValueError): native.solve(np.full((21,3), np.nan))
                np.testing.assert_array_equal(native.last_qpos, previous)
                # An invalid C output buffer must not commit a reset/state update.
                zeros = np.zeros(20, dtype=np.float64)
                status = native._library.tianji_hand_optimizer_state(native._handle,
                    native._ptr(zeros), None, 1)
                self.assertEqual(status, -1)
                np.testing.assert_array_equal(native.last_qpos, previous)
                native.close(); native.close()
                with self.assertRaises(RuntimeError): native.solve(points)

    def test_worker_opt_in_matches_original(self):
        import json
        import os
        import subprocess
        raw = [[0.,0.,0.]]
        for finger in range(5):
            for joint in range(4): raw.append([.02*(finger-2), .02*(joint+1), .002*joint])
        rows = []
        for i in range(1,41):
            points = np.array(raw); points[4::4,2] += .005*np.sin(i*.2)
            rows.append(dict(schema_version=1, kind='wuji_hand_input', callback_sequence=i+(i>=20),
                timestamp_ns=100+i, points=list(points.ravel())*2))
        payload = ''.join(json.dumps(row)+'\n' for row in rows)
        outputs = []
        for backend in ('python','cpp'):
            env = dict(os.environ, TIANJI_HAND_OPTIMIZER_BACKEND=backend)
            env.pop('TIANJI_HAND_GEOMETRY_BACKEND', None)
            env.pop('TIANJI_HAND_FILTER_BACKEND', None)
            result = subprocess.run([sys.executable, str(ROOT/'scripts/wuji_hand_worker.py'),
                                     '--startup-handshake'], input=payload, env=env,
                                     capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode,0,result.stderr)
            outputs.append([json.loads(line) for line in result.stdout.splitlines()])
        self.assertEqual([len(x) for x in outputs],[41,41])
        self.assertEqual(outputs[0][0],outputs[1][0])
        for a,b in zip(outputs[0][1:],outputs[1][1:]):
            for side in ('left','right'):
                self.assertEqual(a[side]['valid'],b[side]['valid'])
                np.testing.assert_allclose(a[side]['position_rad'],b[side]['position_rad'],atol=2e-5,rtol=0)
        # Invalid selection must fail instead of running the original silently.
        result = subprocess.run([sys.executable, str(ROOT/'scripts/wuji_hand_worker.py')], input='',
            env=dict(os.environ, TIANJI_HAND_OPTIMIZER_BACKEND='invalid'),
            capture_output=True, text=True, timeout=30)
        self.assertNotEqual(result.returncode,0)
        self.assertIn('TIANJI_HAND_OPTIMIZER_BACKEND',result.stderr)


if __name__ == '__main__': unittest.main()
