import unittest
import socket
import subprocess
import sys
import struct
import tempfile
from pathlib import Path
from threading import Event, Thread

ROOT = Path(__file__).resolve().parents[1]


class DeviceInputProbeTest(unittest.TestCase):
    def test_manus_shutdown_delay_does_not_age_the_observation(self):
        import importlib.util
        import contextlib
        import io
        import json
        import time
        from types import SimpleNamespace
        from unittest.mock import patch
        spec = importlib.util.spec_from_file_location('probe_shutdown_test', ROOT / 'scripts/probe_teleop_input.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        class Source:
            failure = None
            sequence = 0
            def try_read(self):
                self.sequence += 1
                return SimpleNamespace(source_sequences={'left': self.sequence, 'right': self.sequence},
                                       received_timestamp_ns=time.monotonic_ns())
            def close(self):
                time.sleep(.3)
        with tempfile.TemporaryDirectory() as directory:
            rawviz = Path(directory, 'rawviz.out')
            rawviz.touch()
            rawviz.chmod(0o700)
            calibration = Path(directory, 'calibration')
            calibration.mkdir()
            for side in ('Left', 'Right'):
                (calibration / f'gjy{side}MetaglovePro.mcal').touch()
            output = io.StringIO()
            with patch('tianji_teleop.hand_tracking.reference_manus_process.ReferenceManusProcess',
                       return_value=Source()), contextlib.redirect_stdout(output):
                result = module.main(['--mode', 'manus', '--manus-rawviz', str(rawviz),
                                      '--manus-user', 'gjy', '--duration-s', '.03', '--minimum-frames', '1'])
            self.assertEqual(result, 0, output.getvalue())
            report = json.loads(output.getvalue())
            self.assertTrue(report['sides']['left']['fresh'])
            self.assertTrue(report['sides']['right']['fresh'])

    def test_both_sides_must_be_fresh_not_just_seen_once(self):
        from tianji_teleop.hand_tracking.device_probe import InputProbe
        probe = InputProbe()
        probe.observe('left', 1, 100)
        probe.observe('right', 1, 100)
        probe.observe('right', 2, 400_000_100)
        result = probe.report(400_000_100, minimum_frames=1)
        self.assertFalse(result['passed'])
        self.assertFalse(result['sides']['left']['fresh'])
        self.assertFalse(result['robot_commands_enabled'])

    def test_cached_manus_side_does_not_refresh_liveness(self):
        from tianji_teleop.hand_tracking.device_probe import InputProbe
        probe = InputProbe()
        probe.observe('left', 1, 100)
        probe.observe('left', 1, 400_000_100)
        probe.observe('right', 2, 400_000_100)
        result = probe.report(400_000_100, minimum_frames=1)
        self.assertFalse(result['passed'])
        self.assertEqual(result['sides']['left']['new_frames'], 1)

    def test_liveness_is_not_device_or_control_qualification(self):
        from tianji_teleop.hand_tracking.device_probe import InputProbe
        probe = InputProbe()
        for side in ('left', 'right'):
            probe.observe(side, 1, 100)
            probe.observe(side, 2, 200)
        result = probe.report(300, minimum_frames=2)
        self.assertTrue(result['passed'])
        self.assertFalse(result['hardware_acceptance_complete'])

    def test_receive_only_cli_with_loopback_pico_and_tjvr(self):
        import json
        from tests.test_pico_hand_tracking import _packet
        from tests.test_reference_tjvr_receiver import packet
        for mode in ('pico', 'tjvr'):
            with self.subTest(mode=mode), socket.socket() as listener:
                listener.bind(('127.0.0.1', 0))
                port = listener.getsockname()[1]
                listener.listen()
                listener.settimeout(5)
                stop = Event()
                def feed():
                    try:
                        if mode == 'pico':
                            connection, _ = listener.accept()
                        else:
                            connection = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                        with connection:
                            sequence = 0
                            while not stop.wait(.01):
                                sequence += 1
                                if mode == 'pico':
                                    data = bytearray(_packet())
                                    struct.pack_into('<q', data, 2, sequence)
                                    connection.sendall(data)
                                else:
                                    connection.sendto(packet(sequence), ('127.0.0.1', port))
                    except OSError:
                        pass
                thread = Thread(target=feed)
                thread.start()
                try:
                    result = subprocess.run([sys.executable, str(ROOT / 'scripts/probe_teleop_input.py'),
                        '--mode', mode, '--port', str(port), '--duration-s', '1', '--minimum-frames', '2'],
                        text=True, capture_output=True, timeout=8)
                finally:
                    stop.set()
                    thread.join(timeout=6)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertFalse(json.loads(result.stdout)['robot_commands_enabled'])

    def test_probe_refuses_invalid_duration_before_connecting(self):
        result = subprocess.run([sys.executable, str(ROOT / 'scripts/probe_teleop_input.py'),
            '--mode', 'pico', '--duration-s', 'nan'], capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 2)

    def test_xr_probe_rejects_non_directory_native_library_path(self):
        with tempfile.TemporaryDirectory() as directory:
            library_file = Path(directory, 'not-a-directory')
            library_file.write_text('', encoding='utf-8')
            result = subprocess.run([
                sys.executable, str(ROOT / 'scripts/probe_teleop_input.py'),
                '--mode', 'xr', '--xr-sdk-library-dir', str(library_file),
            ], capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 2)
        self.assertIn('directory', result.stderr.lower())

    def test_receive_only_xr_probe_reports_hmd_controllers_and_bound_trackers(self):
        sdk_source = '''
_timestamp = 0

def init():
    return True

def close():
    return None

def _pose(x):
    return [x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

def get_headset_pose():
    return _pose(0.0)

def get_left_controller_pose():
    return _pose(-0.2)

def get_right_controller_pose():
    return _pose(0.2)

def num_motion_data_available():
    return 4

def get_motion_tracker_pose():
    return [_pose(0.1), _pose(0.2), _pose(0.3), _pose(0.4)]

def get_motion_tracker_serial_numbers():
    return ['190058', '190600', '190046', '190023']

def get_left_trigger():
    return 0.0

def get_right_trigger():
    return 0.0

def get_left_grip():
    return 0.0

def get_right_grip():
    return 0.0

def get_left_axis():
    return [0.0, 0.0]

def get_right_axis():
    return [0.0, 0.0]

def get_motion_timestamp_ns():
    global _timestamp
    _timestamp += 1
    return _timestamp
'''
        with tempfile.TemporaryDirectory() as directory:
            Path(directory, 'xrobotoolkit_sdk.py').write_text(sdk_source, encoding='utf-8')
            Path(directory, 'xr.yaml').write_text(
                '''xr:
  arm_input: xr_tracker
  tracker_serials:
    left: "190058"
    right: "190600"
  elbow_tracker_serials:
    left: "190046"
    right: "190023"
''',
                encoding='utf-8',
            )
            result = subprocess.run([
                sys.executable, str(ROOT / 'scripts/probe_teleop_input.py'),
                '--mode', 'xr',
                '--xr-sdk-pythonpath', directory,
                '--xr-config', str(Path(directory, 'xr.yaml')),
                '--arm-input', 'xr_tracker',
                '--duration-s', '0.5',
                '--minimum-frames', '2',
                '--minimum-trackers', '4',
            ], capture_output=True, text=True, timeout=8)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        report = __import__('json').loads(result.stdout)
        self.assertTrue(report['passed'])
        self.assertEqual(report['scope'], 'xr_input_liveness_only')
        self.assertGreaterEqual(report['frames'], 2)
        self.assertTrue(report['hmd']['fresh'])
        self.assertTrue(report['controllers']['left']['fresh'])
        self.assertTrue(report['controllers']['right']['fresh'])
        self.assertEqual(report['trackers']['required_count'], 4)
        self.assertEqual(report['trackers']['fresh_count'], 4)
        self.assertFalse(report['robot_commands_enabled'])

    def test_xr_probe_rejects_non_directory_sdk_path_before_connecting(self):
        with tempfile.TemporaryDirectory() as directory:
            sdk_file = Path(directory, 'not-a-directory')
            sdk_file.write_text('', encoding='utf-8')
            result = subprocess.run([
                sys.executable, str(ROOT / 'scripts/probe_teleop_input.py'),
                '--mode', 'xr', '--xr-sdk-pythonpath', str(sdk_file),
            ], capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 2)
        self.assertIn('directory', result.stderr.lower())

    def test_xr_controller_probe_allows_a_controller_only_binding(self):
        sdk_source = '''
_timestamp = 0

def init():
    return True

def close():
    return None

def _pose(x):
    return [x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

def get_headset_pose():
    return _pose(0.0)

def get_left_controller_pose():
    return _pose(-0.2)

def get_right_controller_pose():
    return _pose(0.2)

def num_motion_data_available():
    return 0

def get_motion_tracker_pose():
    return []

def get_motion_tracker_serial_numbers():
    return []

def get_left_trigger():
    return 0.0

def get_right_trigger():
    return 0.0

def get_left_grip():
    return 0.0

def get_right_grip():
    return 0.0

def get_left_axis():
    return [0.0, 0.0]

def get_right_axis():
    return [0.0, 0.0]

def get_motion_timestamp_ns():
    global _timestamp
    _timestamp += 1
    return _timestamp
'''
        config = '''
xr:
  arm_input: xr_controller
  tracker_serials: {}
  elbow_tracker_serials: {}
'''
        with tempfile.TemporaryDirectory() as directory:
            Path(directory, 'xrobotoolkit_sdk.py').write_text(sdk_source, encoding='utf-8')
            config_path = Path(directory, 'xr.yaml')
            config_path.write_text(config, encoding='utf-8')
            result = subprocess.run([
                sys.executable, str(ROOT / 'scripts/probe_teleop_input.py'),
                '--mode', 'xr', '--xr-sdk-pythonpath', directory,
                '--xr-config', str(config_path), '--duration-s', '0.3',
                '--minimum-frames', '2',
            ], capture_output=True, text=True, timeout=8)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        report = __import__('json').loads(result.stdout)
        self.assertTrue(report['passed'])
        self.assertEqual(report['trackers']['required_count'], 0)
        self.assertEqual(report['trackers']['minimum_trackers'], 0)
        self.assertFalse(report['robot_commands_enabled'])

    def test_xr_controller_probe_ignores_optional_tracker_bindings_by_default(self):
        sdk_source = '''
_timestamp = 0

def init():
    return True

def close():
    return None

def _pose(x):
    return [x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

def get_headset_pose():
    return _pose(0.0)

def get_left_controller_pose():
    return _pose(-0.2)

def get_right_controller_pose():
    return _pose(0.2)

def num_motion_data_available():
    return 0

def get_motion_tracker_pose():
    return []

def get_motion_tracker_serial_numbers():
    return []

def get_left_trigger():
    return 0.0

def get_right_trigger():
    return 0.0

def get_left_grip():
    return 0.0

def get_right_grip():
    return 0.0

def get_left_axis():
    return [0.0, 0.0]

def get_right_axis():
    return [0.0, 0.0]

def get_motion_timestamp_ns():
    global _timestamp
    _timestamp += 1
    return _timestamp
'''
        config = '''
xr:
  arm_input: xr_controller
  tracker_serials:
    left: "190058"
    right: "190600"
  elbow_tracker_serials:
    left: "190046"
    right: "190023"
'''
        with tempfile.TemporaryDirectory() as directory:
            Path(directory, 'xrobotoolkit_sdk.py').write_text(sdk_source, encoding='utf-8')
            config_path = Path(directory, 'xr.yaml')
            config_path.write_text(config, encoding='utf-8')
            result = subprocess.run([
                sys.executable, str(ROOT / 'scripts/probe_teleop_input.py'),
                '--mode', 'xr', '--xr-sdk-pythonpath', directory,
                '--xr-config', str(config_path), '--duration-s', '0.3',
                '--minimum-frames', '2',
            ], capture_output=True, text=True, timeout=8)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        report = __import__('json').loads(result.stdout)
        self.assertTrue(report['passed'])
        self.assertEqual(report['trackers']['required_count'], 0)
        self.assertEqual(report['trackers']['minimum_trackers'], 0)
        self.assertFalse(report['robot_commands_enabled'])
