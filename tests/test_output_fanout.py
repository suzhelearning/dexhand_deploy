"""Independent bounded output consumers; no networking or hardware."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which('c++'), 'C++ compiler required')
class OutputFanoutTest(unittest.TestCase):
    def test_slow_consumer_isolation_overflow_and_last_failure(self):
        with tempfile.TemporaryDirectory(prefix='native-fanout-') as directory:
            binary = Path(directory) / 'test'
            flags = (['-O1', '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-pie', '-no-pie']
                     if os.environ.get('NATIVE_OUTPUT_SANITIZERS') == '1' else ['-O2'])
            subprocess.run(['c++', '-std=c++17', '-pthread', *flags, '-Wall', '-Wextra', '-Werror',
                            str(ROOT / 'tests/cpp/test_output_fanout.cpp'), '-o', str(binary)], check=True)
            for _ in range(10):
                subprocess.run([str(binary)], check=True, timeout=10)
