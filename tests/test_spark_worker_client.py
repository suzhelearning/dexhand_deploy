import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

from tianji_teleop.hand_tracking.spark_replay import ReplayTick

ROOT = Path(__file__).resolve().parents[1]
MODULE = 'tianji_teleop.hand_tracking.spark_worker_client'


class SparkWorkerContractTest(unittest.TestCase):
    def client_class(self):
        self.assertIsNotNone(importlib.util.find_spec(MODULE), 'missing isolated worker client')
        from tianji_teleop.hand_tracking.spark_worker_client import SparkWorkerClient
        return SparkWorkerClient

    def options(self):
        return dict(worker=ROOT / 'build/spark-native/spark_native_worker',
                    config=ROOT / 'src/tianji_teleop/config/producers/spark_reference.yaml',
                    model=ROOT / 'src/tianji_teleop/assets/spark/marvin_m6_wuji2.xml',
                    urdf=ROOT / 'src/tianji_teleop/assets/spark/marvin_m6_s_ccs_696_v4_local.urdf')

    def test_reject_real_before_process_creation(self):
        cls = self.client_class()
        with self.assertRaisesRegex(ValueError, 'simulation'):
            cls(**self.options(), required_capability='real')

    def test_invalid_timeout_rejected(self):
        cls = self.client_class()
        for timeout in (0, -1, float('nan'), float('inf'), True):
            with self.assertRaises(ValueError):
                cls(**self.options(), timeout_seconds=timeout)

    def test_isolated_worker_does_not_inherit_python_or_library_overrides(self):
        cls = self.client_class()
        real_popen = subprocess.Popen
        def spawn(*args, **kwargs):
            environment = kwargs.get('env', os.environ)
            for key in ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH', 'LD_PRELOAD'):
                self.assertFalse(key in environment, f'inherited worker override: {key}')
            return real_popen([sys.executable, '-c', 'import sys; sys.stdin.readline()'], **kwargs)
        with patch.dict(os.environ, {key: '/unrelated-test-environment' for key in
                                     ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH', 'LD_PRELOAD')}):
            with patch(MODULE + '.subprocess.Popen', side_effect=spawn):
                with cls(**self.options()):
                    pass

    def test_partial_line_times_out_and_poisoned_client_cannot_retry(self):
        cls = self.client_class()
        real_popen = subprocess.Popen
        def broken_worker(*args, **kwargs):
            return real_popen([sys.executable, '-c',
                "import sys; sys.stdin.readline(); print('{', end='', flush=True); sys.stdin.readline()"],
                **kwargs)
        options = self.options()
        options['worker'] = Path(sys.executable)
        with patch(MODULE + '.subprocess.Popen', side_effect=broken_worker):
            with cls(**options, timeout_seconds=.2) as client:
                with self.assertRaises(TimeoutError):
                    client.step(ReplayTick(1, 1_000_000_000, None))
                with self.assertRaisesRegex(RuntimeError, 'closed'):
                    client.step(ReplayTick(1, 1_000_000_000, None))

    def test_worker_eof_is_not_a_hold_command(self):
        cls = self.client_class()
        real_popen = subprocess.Popen
        options = self.options()
        options['worker'] = Path(sys.executable)
        with patch(MODULE + '.subprocess.Popen', side_effect=lambda *args, **kw:
                   real_popen([sys.executable, '-c', 'import sys; sys.stdin.readline()'], **kw)):
            with cls(**options) as client:
                with self.assertRaisesRegex(RuntimeError, 'complete result'):
                    client.step(ReplayTick(1, 1_000_000_000, None))

    def test_startup_timeout_terminates_owned_process_without_control_tick(self):
        cls = self.client_class()
        real_popen, processes = subprocess.Popen, []
        def unready(*args, **kwargs):
            process = real_popen([sys.executable, '-c', 'import time; time.sleep(10)'], **kwargs)
            processes.append(process)
            return process
        options = self.options()
        options['worker'] = Path(sys.executable)
        with patch(MODULE + '.subprocess.Popen', side_effect=unready):
            with self.assertRaisesRegex(TimeoutError, 'startup'):
                cls(**options, startup_handshake=True, timeout_seconds=.05)
        self.assertIsNotNone(processes[0].poll())

    @unittest.skipUnless(os.environ.get('SPARK_NATIVE_TEST'), 'optional native SPARK build')
    def test_ambiguous_json_and_nonfinite_diagnostics_rejected(self):
        cls = self.client_class()
        options = self.options()
        real_popen = subprocess.Popen
        command = [str(options[k]) for k in ('worker', 'config', 'model', 'urdf')]
        valid = subprocess.check_output(command, input='TJSC1 1 1000000000 0 0 0 -\n', text=True)
        self.assertEqual(json.loads(valid)['tick_id'], 1)
        invalids = [valid.replace('"tick_id":1', '"tick_id":999,"tick_id":1')]
        row = json.loads(valid)
        row['left']['headroom_scale'] = float('nan')
        invalids.append(json.dumps(row) + '\n')
        invalids.append((json.dumps(row) + '\n').replace('NaN', '1e999'))
        for invalid in invalids:
            program = 'import sys; sys.stdin.readline(); sys.stdout.write(' + repr(invalid) + '); sys.stdout.flush()'
            with patch(MODULE + '.subprocess.Popen', side_effect=lambda *args, **kw:
                       real_popen([sys.executable, '-c', program], **kw)):
                with cls(**options) as client:
                    with self.assertRaises(ValueError):
                        client.step(ReplayTick(1, 1_000_000_000, None))

    @unittest.skipUnless(os.environ.get('SPARK_NATIVE_TEST'), 'optional native SPARK build')
    def test_reset_ack_starts_new_tick_epoch_but_not_a_new_clock(self):
        cls = self.client_class()
        with cls(**self.options(), deterministic_test=True, startup_handshake=True) as client:
            first = client.step(ReplayTick(1, 1_000_000_000, None))
            positions = [v + .01 for v in first['left']['q'] + first['right']['q']]
            self.assertTrue(hasattr(client, 'reset_at_rest'))
            ack = client.reset_at_rest(positions, execution_epoch=2)
            self.assertEqual(ack['position_rad'], positions)
            with self.assertRaises(ValueError):
                client.reset_at_rest(positions, execution_epoch=2)
            with self.assertRaises(ValueError):
                client.step(ReplayTick(1, 999_000_000, None))
            next_tick = client.step(ReplayTick(1, 1_005_000_000, None))
            self.assertEqual(next_tick['guidance_updates'], 1)
            self.assertEqual(ack['velocity_rad_s'], [0.] * 14)

    @unittest.skipUnless(os.environ.get('SPARK_NATIVE_TEST'), 'optional native SPARK build')
    def test_startup_handshake_loads_model_without_advancing_reference(self):
        cls = self.client_class()
        with cls(**self.options(), startup_handshake=True, deterministic_test=True) as client:
            self.assertTrue(client.startup_ready)
            first = client.step(ReplayTick(1, 1_000_000_000, None))
            self.assertEqual(first['tick_id'], 1)
            self.assertEqual(first['guidance_updates'], 1)

    @unittest.skipUnless(os.environ.get('SPARK_NATIVE_TEST'), 'optional native SPARK build')
    def test_bilateral_result_and_explicit_close(self):
        cls = self.client_class()
        with cls(**self.options(), deterministic_test=True) as client:
            first = client.step(ReplayTick(1, 1_000_000_000, None))
            self.assertEqual(first['tick_id'], 1)
            self.assertEqual(len(first['left']['q']), 7)
            self.assertEqual(len(first['right']['q']), 7)
            # Invalid local requests must not advance the native state.
            with self.assertRaises(ValueError):
                client.step(ReplayTick(1, 1_005_000_000, None))
            second = client.step(ReplayTick(2, 1_005_000_000, None))
            self.assertEqual(second['guidance_updates'], 2)
        with self.assertRaises(RuntimeError):
            client.step(ReplayTick(3, 1_010_000_000, None))
        client.close()  # idempotent; never restart implicitly


if __name__ == '__main__':
    unittest.main()
