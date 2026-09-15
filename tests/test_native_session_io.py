"""Offline native transport/consumer tests; no network listeners or devices."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which('c++'), 'C++ compiler required')
class NativeSessionIoTest(unittest.TestCase):
    def compile_run(self, name):
        header = ROOT / 'native/control' / (name + '.hpp')
        self.assertTrue(header.is_file(), f'{name} implementation missing')
        flags = ['-O2']
        if os.environ.get('NATIVE_SESSION_IO_SANITIZERS') == '1':
            flags = ['-O1', '-g', '-fsanitize=address,undefined',
                     '-fno-omit-frame-pointer', '-fno-pie', '-no-pie']
        with tempfile.TemporaryDirectory(prefix='native-session-io-') as directory:
            binary = Path(directory) / 'test'
            subprocess.run(['c++', '-std=c++17', '-pthread', *flags,
                            '-Wall', '-Wextra', '-Werror',
                            str(ROOT / 'tests/cpp' / ('test_' + name + '.cpp')),
                            '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True, timeout=15)

    def test_datagram_receiver(self):
        self.compile_run('datagram_receiver')

    def test_bounded_output(self):
        self.compile_run('bounded_output')

    def test_session_output_bridge(self):
        self.compile_run('session_output_bridge')

    def test_received_frames_use_runtime_gate_and_output_thread(self):
        from tests.test_native_raw_input import compile_driver
        from tests.test_reference_tjvr_receiver import packet
        with tempfile.TemporaryDirectory(prefix='native-session-io-integration-') as directory:
            binary = Path(directory) / 'test'
            compile_driver('native_session_io_driver.cpp', binary)
            data = '\n'.join(p.hex() for p in [packet(1), packet(1), b'bad', packet(2)])
            for mode in ('packet', 'mapped'):
                result = subprocess.run([str(binary), mode], input=data + '\n',
                                        text=True, capture_output=True, timeout=15)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn('native_io_gate_and_receipt_ok', result.stdout)
