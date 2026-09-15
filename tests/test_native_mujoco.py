import importlib.util
import unittest
import subprocess
import sysconfig
from unittest.mock import patch
from pathlib import Path

import mujoco
import numpy as np
from tests import test_authorized_hand_mujoco as reference

BUILT = (Path(__file__).resolve().parents[1] / 'build/control-native' /
         ('_tianji_mujoco' + sysconfig.get_config_var('EXT_SUFFIX'))).is_file()


class SimulationSelectionTest(unittest.TestCase):
    def test_missing_extension_is_explicit_without_fallback(self):
        from tianji_teleop.executors.mujoco import native_kernel
        native_kernel.load_native.cache_clear()
        try:
            with patch.object(native_kernel, 'extension_path', return_value=Path('/missing/native-mujoco')):
                with self.assertRaisesRegex(RuntimeError, 'build-native-mujoco'):
                    native_kernel.load_native()
        finally:
            native_kernel.load_native.cache_clear()

    def test_pico2_rejects_cpp_option_before_device_startup(self):
        root = Path(__file__).resolve().parents[1]
        result = subprocess.run(['bash', str(root / 'scripts/run_session.sh'), '--profile',
            'pico2_hands_sim', '--simulation-backend', 'cpp'], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 2)
        self.assertIn('--simulation-backend 仅支持 VR/TJVR', result.stderr)


@unittest.skipUnless(BUILT, 'run pixi run build-native-mujoco')
class NativeExecutorTest(reference.AuthorizedHandMujocoTest):
    def make(self):
        return super().make(simulation_backend='cpp')

    def test_native_and_python_joint_and_feedback_parity(self):
        native = self.make()
        python = super().make()
        for seq, phase in enumerate(('teleop', 'teleop', 'returning', 'idle', 'fault'), 1):
            for sim in (native, python):
                sim.on_session_state(self.state(phase, seq))
                if phase == 'teleop':
                    self.assertTrue(sim.on_hand_command(self.command(seq)))
                sim.tick()
            np.testing.assert_array_equal(native.data.qpos, python.data.qpos)
            np.testing.assert_array_equal(native.data.xpos, python.data.xpos)
            self.assertEqual(native.arm_state, python.arm_state)
            self.assertEqual(native.status, python.status)
            self.now += 5_000_000

    def test_native_failure_locks_executor_instead_of_falling_back(self):
        sim = self.make()
        with patch.object(sim._native_kernel, 'apply', side_effect=RuntimeError('test failure')):
            with self.assertRaisesRegex(RuntimeError, 'test failure'):
                sim.tick()
        self.assertTrue(sim.safety_locked)
        sim.tick()
        self.assertFalse(sim.status.healthy)


@unittest.skipUnless(BUILT, 'run pixi run build-native-mujoco')
class NativeMujocoTest(unittest.TestCase):
    def test_capsule_retains_owners_after_caller_drops_references(self):
        model = self.model()
        data = mujoco.MjData(model)
        kernel = self.kernel(model, data, ((0,), (1,)))
        del model, data
        kernel.apply(([.1], [.2]))
        self.assertEqual(kernel.positions(0, 2), [.1, .2])

    def kernel(self, model, data, groups):
        name = 'tianji_teleop.executors.mujoco.native_kernel'
        self.assertIsNotNone(importlib.util.find_spec(name), 'native MuJoCo kernel missing')
        from tianji_teleop.executors.mujoco.native_kernel import NativeMujocoKernel
        return NativeMujocoKernel(model, data, groups)

    def model(self):
        return mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><joint name="a"/>'
            '<geom size=".1"/><body pos="0 0 .2"><joint name="b"/>'
            '<geom size=".1"/></body></body></worldbody></mujoco>')

    def test_batch_matches_forward_and_owns_model_data(self):
        model = self.model()
        data = mujoco.MjData(model)
        reference = mujoco.MjData(model)
        kernel = self.kernel(model, data, ((0,), (1,)))
        for i in range(30):
            values = (.01 * i, -.02 * i)
            kernel.apply(([values[0]], [values[1]]))
            reference.qpos[:] = values
            mujoco.mj_forward(model, reference)
            for name in ('qpos', 'xpos', 'xmat', 'site_xpos', 'qM'):
                np.testing.assert_array_equal(getattr(data, name), getattr(reference, name))
        kernel.apply((None, [0.5]))
        self.assertEqual(data.qpos[0], .29)
        self.assertEqual(data.qpos[1], .5)
        self.assertTrue(hasattr(kernel, 'positions'), 'native feedback snapshot missing')
        snapshot = kernel.positions(0, 2)
        self.assertEqual(snapshot, [.29, .5])
        snapshot[0] = 9.
        self.assertEqual(data.qpos[0], .29)
        self.assertEqual(kernel.positions(1, 1), [.5])
        with self.assertRaises(ValueError):
            kernel.positions(1, 2)

    def test_invalid_batch_cannot_partially_write(self):
        model = self.model()
        data = mujoco.MjData(model)
        kernel = self.kernel(model, data, ((0,), (1,)))
        for batch in (([.3], [float('nan')]), ([.3], []), ([.3], [True]), ([.3],),
                      ([.3], ['.4'])):
            with self.subTest(batch=batch), self.assertRaises((ValueError, TypeError)):
                kernel.apply(batch)
            np.testing.assert_array_equal(data.qpos, [0., 0.])

    def test_invalid_binding_rejected(self):
        model = self.model()
        for groups in (((-1,),), ((2,),), ((0,), (0,)), ((True,),)):
            with self.subTest(groups=groups), self.assertRaises((ValueError, TypeError)):
                self.kernel(model, mujoco.MjData(model), groups)
        with self.assertRaises((ValueError, TypeError)):
            self.kernel(model, mujoco.MjData(self.model()), ((0,),))


if __name__ == '__main__':
    unittest.main()
