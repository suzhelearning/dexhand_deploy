"""Offline C++ gateway ownership tests; no devices or network listeners."""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which('c++'), 'C++ compiler required')
class NativeSessionGatewayTest(unittest.TestCase):
    def test_cpp_wire_accepts_tick_between_two_input_packets(self):
        from tianji_teleop.producers.spark.native_gateway import decode_gateway_frame, FRAME_HEADER_STRUCT
        with tempfile.TemporaryDirectory(prefix='native-empty-tick-') as directory:
            binary = Path(directory) / 'wire'
            subprocess.run(['c++', '-std=c++17', '-pthread', '-O2', '-Wall', '-Wextra', '-Werror',
                            str(ROOT / 'tests/cpp/gateway_empty_tick_wire.cpp'), '-o', str(binary)], check=True)
            data = subprocess.check_output([str(binary)], timeout=10)
        frames = []
        while data:
            size = FRAME_HEADER_STRUCT.size + FRAME_HEADER_STRUCT.unpack_from(data)[4]
            frames.append(decode_gateway_frame(data[:size], prefix='spark').payload)
            data = data[size:]
        self.assertEqual([row['request']['id'] for row in frames], [1, 2, 3])
        self.assertEqual([row['request']['packet'] for row in frames], [b'\x01\x02\x03', b'', b'\x01\x02\x03'])
        self.assertEqual(frames[1]['request']['received_ns'], 0)

    def test_native_gateway_owns_workers_scheduler_and_output(self):
        with tempfile.TemporaryDirectory(prefix='native-gateway-') as directory:
            binary = Path(directory) / 'gateway'
            subprocess.run([
                'c++', '-std=c++17', '-pthread', '-O2', '-Wall', '-Wextra', '-Werror',
                str(ROOT / 'tests/cpp/test_session_gateway.cpp'), '-o', str(binary),
            ], check=True)
            for _ in range(5):
                for args in ([], ['--fail-publication'], ['--fail-final'],
                             ['--fail-recording'], ['--fail-recording-close'], ['--viewer-input'],
                             ['--summary'], ['--summary-fail-final'], ['--fail-input-start'], ['--fail-input-stop']):
                    subprocess.run([str(binary), *args], check=True, timeout=10)


if __name__ == '__main__':
    unittest.main()
