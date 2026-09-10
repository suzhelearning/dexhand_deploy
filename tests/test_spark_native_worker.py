import json
import os
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get('SPARK_NATIVE_TEST'), 'optional native SPARK build')
class SparkNativeWorkerTest(unittest.TestCase):
    def command(self):
        binary = ROOT / 'build/spark-native/spark_native_worker'
        self.assertTrue(binary.is_file(), 'native SPARK worker is not built')
        return [str(binary),
            str(ROOT / 'src/tianji_teleop/config/producers/spark_reference.yaml'),
            str(ROOT / 'src/tianji_teleop/assets/spark/marvin_m6_wuji2.xml'),
            str(ROOT / 'src/tianji_teleop/assets/spark/marvin_m6_s_ccs_696_v4_local.urdf'),
            '--deterministic-test']

    def test_two_ticks_and_diagnostics(self):
        result = subprocess.run(self.command(), input='TJSC1 1 1000000000 0 0 0 -\nTJSC1 2 1005000000 0 0 0 -\n',
            text=True, capture_output=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)
        rows = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual([r['tick_id'] for r in rows], [1, 2])
        for row in rows:
            self.assertEqual(row['state_source'], 'model_reference')
            self.assertTrue(row['deterministic_test'])
            self.assertEqual(len(row['left']['q']), 7)
            self.assertEqual(len(row['right']['qdot']), 7)
            self.assertEqual(row['guidance_updates'], row['tick_id'])
            self.assertEqual(row['headroom_updates'], row['tick_id'])

    def test_bad_transport_fails_without_a_joint_result(self):
        for line in ('bad\n', 'TJSC1 1 1000000000 0 0 0 0000\n',
                     'TJSC1 1 1000000000 0 -1 0 -\n',
                     'TJSC1 1 1000000000 0 0 0 -'):
            result = subprocess.run(self.command(), input=line, text=True,
                capture_output=True, timeout=15)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(result.stdout, '')

    def test_future_receive_time_is_rejected_before_control(self):
        from tests.test_legacy_pico_palm import _packet
        line = 'TJSC1 1 1000000000 2000000000 0 0 ' + _packet(flags=0xff).hex() + '\n'
        result = subprocess.run(self.command(), input=line, text=True,
                                capture_output=True, timeout=15)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, '')

    def test_explicit_rest_reset_uses_both_measured_positions_without_advancing_tick(self):
        positions = [1.11, -1.51, -1.51, -1.09, .01, .01, .01,
                     -1.09, -1.51, 1.53, -1.09, .01, .01, .01]
        reset = 'TJSR1 2 ' + ' '.join(map(str, positions)) + '\n'
        result = subprocess.run(self.command(), input='TJSC1 1 1000000000 0 0 0 -\n' + reset +
            'TJSC1 1 1100000000 0 0 0 -\n', text=True, capture_output=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)
        rows = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual(rows[1]['kind'], 'spark_reset_ack')
        self.assertEqual(rows[1]['execution_epoch'], 2)
        self.assertEqual(rows[1]['position_rad'], positions)
        self.assertEqual(rows[1]['velocity_rad_s'], [0.] * 14)
        self.assertEqual(rows[1]['acceleration_rad_s2'], [0.] * 14)
        self.assertEqual(rows[2]['guidance_updates'], 1)
        # QP may update q numerically on the next authorized tick. The reset
        # equality contract applies BEFORE that new solver invocation.
        self.assertEqual(rows[2]['tick_id'], 1)

    def test_reset_rejects_out_of_bounds_side_before_acknowledgement(self):
        result = subprocess.run(self.command(), input='TJSR1 2 ' + ' '.join(['99'] * 14) + '\n',
                                text=True, capture_output=True, timeout=15)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, '')
