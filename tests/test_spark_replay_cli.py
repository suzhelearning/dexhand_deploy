import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

from tests.test_reference_tjvr_receiver import packet

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get('SPARK_NATIVE_TEST'), 'optional native SPARK build')
class SparkReplayCliTest(unittest.TestCase):
    def test_recording_completion_and_no_overwrite(self):
        script = ROOT / 'scripts/spark_trace_replay.py'
        self.assertTrue(script.is_file(), 'native replay CLI missing')
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / 'input.tjvr'
            output = Path(tmp) / 'output.jsonl'
            source.write_bytes(struct.pack('<4sHHQ', b'TJVT', 1, 656, 2) +
                struct.pack('<q', 0) + packet(1) + struct.pack('<q', 10_000_000) + packet(2))
            command = [sys.executable, str(script), '--input', str(source), '--output', str(output),
                       '--deterministic-test']
            result = subprocess.run(command, capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stderr)
            data = [json.loads(line) for line in output.read_text().splitlines()]
            self.assertEqual(data[0]['kind'], 'spark_trace_manifest')
            self.assertEqual(data[-1]['kind'], 'spark_trace_complete')
            self.assertEqual(data[-1]['receiver']['datagrams'], 2)
            self.assertEqual(data[-1]['ticks'], 53)
            before = output.read_bytes()
            retry = subprocess.run(command, capture_output=True, text=True, timeout=30)
            self.assertNotEqual(retry.returncode, 0)
            self.assertEqual(output.read_bytes(), before)
