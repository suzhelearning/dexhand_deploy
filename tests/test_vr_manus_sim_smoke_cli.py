import os
from pathlib import Path
import subprocess
import sys
import json
import unittest
import tempfile

ROOT = Path(__file__).resolve().parents[1]


class VrManusSimSmokeCliTest(unittest.TestCase):
    def test_help_identifies_offline_scope(self):
        script = ROOT / 'scripts/vr_manus_sim_smoke.py'
        self.assertTrue(script.is_file())
        result = subprocess.run([sys.executable, str(script), '--help'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--tjvr', result.stdout)
        self.assertIn('offline', result.stdout)

    @unittest.skipUnless(os.environ.get('SPARK_NATIVE_TEST'), 'optional actual native smoke')
    def test_short_trace_through_public_cli(self):
        trace = ROOT / 'vendor/reference_inputs/remote-192.168.110.210-20260908/output_continuity_retest.tjvr'
        if not trace.exists():
            self.skipTest('private input unavailable')
        with tempfile.TemporaryDirectory() as directory:
            recording = Path(directory) / 'take.h5'
            result = subprocess.run([sys.executable, str(ROOT / 'scripts/vr_manus_sim_smoke.py'),
                '--tjvr', str(trace), '--disable-hands', '--record', str(recording)],
                capture_output=True, text=True, timeout=120)
            self.assertEqual(result.returncode, 0, result.stderr)
            from tianji_teleop.recording.session_h5 import SessionH5Reader
            with SessionH5Reader(recording) as reader:
                self.assertEqual(reader.attrs['schema_version'], '1.2')
                self.assertEqual(len(reader.read_raw_reference_tjvr()), 2444)
                self.assertGreater(len(reader.read_arm_command('left')), 5000)
                self.assertGreater(len(reader.read_arm_state()), 5000)
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report['maximum_command_error_rad'], 0.)
        self.assertEqual(report['maximum_sim_error_rad'], 0.)
        self.assertGreater(report['control_ticks'], 5000)
        self.assertTrue(report['simulation_only'])
        self.assertFalse(report['real_time_qualified'])

    @unittest.skipUnless(os.environ.get('SPARK_NATIVE_TEST') and os.environ.get('WUJI_REFERENCE_TEST'),
                         'optional actual dual-input processes')
    def test_independent_manus_recording_drives_both_simulated_hands(self):
        data = ROOT / 'vendor/reference_inputs/remote-192.168.110.210-20260908'
        with tempfile.TemporaryDirectory() as directory:
            recording = Path(directory) / 'both.h5'
            result = subprocess.run([sys.executable, str(ROOT / 'scripts/vr_manus_sim_smoke.py'),
                '--tjvr', str(data / 'output_continuity_retest.tjvr'),
                '--manus-recording', str(data / 'manus_gjy_dual_20260827.pkl'), '--record', str(recording)],
                capture_output=True, text=True, timeout=120)
            self.assertEqual(result.returncode, 0, result.stderr)
            from tianji_teleop.recording.session_h5 import SessionH5Reader
            with SessionH5Reader(recording) as reader:
                self.assertEqual(len(reader.read_manus_callbacks()), 1339)
                self.assertEqual(len(reader.read_hand_command('left')), 1339)
                self.assertEqual(len(reader.read_hand_command('right')), 1339)
                self.assertEqual(reader.read_raw_manus(), [])
        report = json.loads(result.stdout)
        self.assertTrue(report['hands_enabled'])
        self.assertFalse(report['synchronized_hardware_recording'])
        self.assertEqual(report['manus_callbacks'], 1339)
        self.assertEqual(report['hand_commands'], 2678)
        self.assertEqual(report['maximum_hand_sim_error_rad'], 0.)
