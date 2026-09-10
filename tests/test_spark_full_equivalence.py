"""Optional actual C++ algorithms + original Viewer block on injected transitions.

Uses a private recording as a pose template, then explicitly SYNTHETICALLY
changes button states, epoch and receive gaps. Never labels it a new live capture.
"""
import importlib.util
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib

from tianji_teleop.hand_tracking.tjvr_trace import iter_tjvr_records

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get('SPARK_REFERENCE_TEST') and os.environ.get('TJVR_TRACE_DIRECTORY'),
                     'optional original worker and private pose template')
class SparkFullEquivalenceTest(unittest.TestCase):
    def test_button_epoch_stale_recovery_match_original_cycle(self):
        recording = Path(os.environ['TJVR_TRACE_DIRECTORY']) / 'output_continuity_retest.tjvr'
        with recording.open('rb') as stream:
            records = list(iter_tjvr_records(stream))[:500]
        self.assertEqual(len(records), 500)
        with tempfile.TemporaryDirectory(prefix='spark-transitions-') as tmp:
            trace = Path(tmp) / 'synthetic-transitions.tjvr'
            with trace.open('xb') as stream:
                stream.write(struct.pack('<4sHHQ', b'TJVT', 1, 656, len(records)))
                for index, record in enumerate(records):
                    packet = bytearray(record.packet)
                    sequence = index + 1
                    epoch = 137 + (index >= 250)
                    struct.pack_into('<QQ', packet, 8, sequence, epoch)
                    flags, = struct.unpack_from('<I', packet, 40)
                    flags = flags | 256 if 100 <= index < 140 else flags & ~256
                    struct.pack_into('<I', packet, 40, flags)
                    if 350 <= index < 355:
                        # Pose-only jump exercises the stream gate and its
                        # persistent generation; preserve the skeleton itself.
                        x, = struct.unpack_from('<d', packet, 44)
                        struct.pack_into('<d', packet, 44, x + .5)
                    struct.pack_into('<I', packet, 652, zlib.crc32(packet[:-4]))
                    received = record.relative_receive_ns + (500_000_000 if index >= 400 else 0)
                    stream.write(struct.pack('<q', received))
                    stream.write(packet)
            outputs = []
            for name in ('reference', 'native'):
                output = Path(tmp) / (name + '.jsonl')
                result = subprocess.run([sys.executable, str(ROOT / 'scripts/spark_trace_replay.py'),
                    '--input', str(trace), '--output', str(output), '--deterministic-test',
                    '--worker', str(ROOT / f'build/spark-native/spark_{name}_worker')],
                    capture_output=True, text=True, timeout=60)
                self.assertEqual(result.returncode, 0, result.stderr)
                outputs.append(output)
            spec = importlib.util.spec_from_file_location('spark_compare', ROOT / 'scripts/compare_spark_reference.py')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            comparison = module.compare_traces(*outputs)
            self.assertTrue(comparison['passed'], comparison['first_divergence'])
            rows = [json.loads(line) for line in outputs[1].read_text().splitlines()][1:-1]
            self.assertIn(1, {row['button_action'] for row in rows})
            self.assertIn(2, {row['button_action'] for row in rows})
            self.assertIn(138, {row['applied_epoch'] for row in rows})
            self.assertTrue(any(not row['input_live'] for row in rows))
