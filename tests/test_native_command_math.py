import importlib.util
import math
import random
import unittest
import subprocess
import sysconfig
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
from tests import test_bilateral_coordinator as reference

from tianji_teleop.coordination.arm_command_coordinator import ArmCommandCoordinator

ROOT = Path(__file__).resolve().parents[1]
BUILT = (ROOT / 'build/control-native' /
         ('_tianji_command_math' + sysconfig.get_config_var('EXT_SUFFIX'))).is_file()


class CommandMathSelectionTest(unittest.TestCase):
    def test_pico2_rejects_new_option_before_startup(self):
        result = subprocess.run(['bash', str(ROOT / 'scripts/run_session.sh'), '--profile',
            'pico2_hands_sim', '--coordinator-math', 'cpp'], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 2)
        self.assertIn('--coordinator-math 仅支持 VR/TJVR', result.stderr)

    def test_missing_extension_has_no_fallback(self):
        from tianji_teleop.coordination import native_command_math as module
        module.load_native.cache_clear()
        try:
            with patch.object(module, 'extension_path', return_value=Path('/missing/native-command-math')):
                with self.assertRaisesRegex(RuntimeError, 'build-native-control'):
                    module.load_native()
        finally:
            module.load_native.cache_clear()


@unittest.skipUnless(BUILT, 'run pixi run build-native-control')
class NativeCoordinatorTest(reference.BilateralCoordinatorTest):
    command_math = 'cpp'

    @unittest.skipUnless((ROOT / 'build/control-native' /
        ('_tianji_mujoco' + sysconfig.get_config_var('EXT_SUFFIX'))).is_file(),
        'run pixi run build-native-mujoco')
    def test_combined_cpp_options_keep_home_rearm_and_explicit_start(self):
        from tianji_teleop.producers.spark.live_simulation import SparkLiveSimulation
        from tianji_teleop.producers.spark.backend_assets import bilateral_assets
        from tianji_teleop.hand_tracking.input_modes import SPARK_BACKEND, MAPPED_PALM_BACKEND
        from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver
        from tests.test_reference_tjvr_receiver import packet
        if not all(bilateral_assets(ROOT, backend)['worker'].is_file()
                   for backend in (SPARK_BACKEND, MAPPED_PALM_BACKEND)):
            self.skipTest('build both native IK workers')
        for backend in (SPARK_BACKEND, MAPPED_PALM_BACKEND):
            with self.subTest(backend=backend):
                now = [1_000_000_000]
                core = SparkLiveSimulation(ROOT, run_id='cpp', router_zid='router', instance_id='owner',
                    clock=lambda: now[0], backend=backend, coordinator_math='cpp',
                    simulation_backend='cpp', execution_guard='cpp', native_result_format='binary')
                try:
                    receiver = ReferenceTjvrReceiver(core.source_instance_id, .15, .6,
                        **({'target_source':'mapped_corrected_palm'} if backend == MAPPED_PALM_BACKEND else {}))
                    receiver.ingest(packet(1), now[0])
                    core.step(receiver.try_read_latest())
                    core.producer.guard.pause('explicit Home rearm')
                    self.assertEqual(core.rearm_at_home()['execution_epoch'], 2)
                    self.assertIsNotNone(core.coordinator._command_math)
                    self.assertFalse(core.request('start').accepted)
                    now[0] += 5_000_000
                    receiver.ingest(packet(2), now[0])
                    core.step(receiver.try_read_latest())
                    self.assertTrue(core.request('start').accepted)
                    now[0] += 5_000_000
                    result = core.step()
                    self.assertTrue(result.receipt_accepted)
                    self.assertEqual(core.sim.arm_state.position_rad,
                        result.native_result['left']['q'] + result.native_result['right']['q'])
                finally:
                    core.close()

    def test_reference_commands_receipts_home_and_fault_match(self):
        native = self.make()
        self.command_math = 'python'
        python = self.make()
        self.command_math = 'cpp'
        for tick in range(1, 40):
            now = 1_000_000_000 + tick * 1_000_000
            if tick < 10:
                for co in (python, native):
                    self.assertTrue(self.ingest(co, self.proposal(co, tick, .005*tick)))
            elif tick == 10:
                for co in (python, native):
                    self.assertTrue(co.handle_intent(SimpleNamespace(action='return', sequence=2,
                        source='src', reason='test')).accepted)
            self.assertEqual(python.tick(now_ns=now), native.tick(now_ns=now))
            self.assertEqual(python.state, native.state)
            self.assertEqual(python.last_bilateral_receipt, native.last_bilateral_receipt)
            self.assertEqual(python.at_home, native.at_home)

    def test_numeric_checks_preserve_fault_reason_and_diagnostics(self):
        for delta in (.01, .3, 99.):
            with self.subTest(delta=delta):
                native = self.make()
                self.command_math = 'python'
                python = self.make()
                self.command_math = 'cpp'
                for co in (python, native):
                    co.config['command_step_time_window_s'] = .1
                    self.assertTrue(self.ingest(co, self.proposal(co, 1, delta)))
                self.assertEqual(python.tick(), native.tick())
                self.assertEqual(python.state, native.state)
                self.assertEqual(python._step_rejection, native._step_rejection)


@unittest.skipUnless(BUILT, 'run pixi run build-native-control')
class NativeCommandMathTest(unittest.TestCase):
    def native(self):
        name = 'tianji_teleop.coordination.native_command_math'
        self.assertIsNotNone(importlib.util.find_spec(name), 'native command math missing')
        from tianji_teleop.coordination.native_command_math import load_native
        return load_native()

    def test_clipping_and_exact_home_match_python_arithmetic(self):
        native = self.native()
        rng = random.Random(713)
        for _ in range(500):
            old = [rng.uniform(-2., 2.) for _ in range(7)]
            target = [rng.uniform(-2., 2.) for _ in range(7)]
            step = rng.uniform(.001, .2)
            for clipping, hold in ((True, False), (False, False), (True, True)):
                expected = [x + max(-step, min(step, y-x)) for x,y in zip(old,target)]
                if not clipping:
                    expected = target
                if hold:
                    expected = old
                self.assertEqual(native.track(target, old, step, clipping, hold), expected)
            elapsed = rng.uniform(0., 10.)
            duration = max(.5, max(abs(x-y) for x,y in zip(old,target)) / .7)
            fraction = min(1., elapsed / duration)
            expected = target if fraction >= 1 else [x+fraction*(y-x) for x,y in zip(old,target)]
            self.assertEqual(native.home(old, target, elapsed, .5, .7), expected)

    def test_validation_boundaries_and_hold_does_not_bypass_limits(self):
        native = self.native()
        config = ArmCommandCoordinator._coordinator_config(None)
        config.update(maximum_command_step_rad=.1, command_step_time_window_s=.1,
                      rate_hz=200., proposal_timeout_s=.2)
        def validate(q, now=1_000_000_000, source=1_000_000_000, anchor=None, hold=False):
            return native.validate(q, [0.]*7, ([-1.]*7, [1.]*7), config,
                                   now, source, anchor, hold, True)
        self.assertEqual(validate([.2]*7)[0], 0)
        self.assertEqual(validate([.2 + 2e-10]*7)[0], 4)
        self.assertEqual(validate([1.01]*7, hold=True)[0], 1)
        self.assertEqual(validate([math.nan]*7)[0], 1)
        self.assertEqual(validate([0.]*7, source=999_999_999, anchor=1_000_000_000)[0], 2)
        self.assertEqual(validate([0.]*7, source=799_999_999)[0], 3)
        self.assertEqual(validate([0.]*7, source=800_000_000)[0], 0)
        self.assertEqual(validate([.8]*7, hold=True)[0], 0)
        self.assertEqual(validate([.8]*7, anchor=950_000_000)[0], 0)
        config['proposal_timeout_s'] = float(2**24)
        deadline = int(config['proposal_timeout_s'] * 1e9)
        self.assertEqual(validate([0.]*7, now=deadline, source=0)[0], 0)
        self.assertEqual(validate([0.]*7, now=deadline+1, source=0)[0], 3)

    def test_rejects_malformed_arguments_without_poisoning_module(self):
        native = self.native()
        for values in ([0.]*6, [True]*7, ['0']*7, [float('inf')]*7):
            with self.subTest(values=values), self.assertRaises((ValueError, TypeError)):
                native.track(values, [0.]*7, .1, True, False)
        self.assertEqual(native.track([.2]*7, [0.]*7, .1, True, False), [.1]*7)

    def test_numeric_validation_matches_reference_over_random_events(self):
        native = self.native()
        cfg = ArmCommandCoordinator._coordinator_config(None)
        rng = random.Random(17)
        for _ in range(2000):
            q = [rng.uniform(-1.1, 1.1) for _ in range(7)]
            old = [rng.uniform(-1., 1.) for _ in range(7)]
            cfg.update(command_step_time_window_s=rng.choice((0., .05, .1)),
                       maximum_command_step_rad=rng.choice((.01, .1, .5)))
            now = 1_000_000_000
            source = now + rng.randint(-300_000_000, 50_000_000)
            anchor = rng.choice((None, now-100_000_000, now-50_000_000))
            hold = rng.choice((True, False))
            code, delta, allowed, elapsed = native.validate(q, old, ([-1.]*7, [1.]*7),
                cfg, now, source, anchor, hold, True)
            expected = 0
            a = 2*cfg['maximum_command_step_rad']
            e = 0.
            if any(not -1 <= v <= 1 for v in q):
                expected = 1
            elif cfg['command_step_time_window_s'] > 0:
                if anchor is not None:
                    e = (source-anchor)/1e9
                    if e < 0:
                        expected = 2
                    else:
                        a = max(a, cfg['maximum_command_step_rad'] * cfg['rate_hz'] *
                                min(e, cfg['command_step_time_window_s']))
                if not expected and not 0 <= now-source <= cfg['proposal_timeout_s']*1e9:
                    expected = 3
            if not expected and not hold:
                d = max(abs(x-y) for x,y in zip(q,old))
                if d > a + (1e-10 if cfg['command_step_time_window_s'] else 0.):
                    expected = 4
                self.assertEqual((delta, allowed, elapsed), (d, a, e))
            self.assertEqual(code, expected)


if __name__ == '__main__':
    unittest.main()
