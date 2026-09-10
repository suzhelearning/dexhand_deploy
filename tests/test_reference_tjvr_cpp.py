"""Opt-in cross-language oracle; compiles ORIGINAL sources, never a live driver.

Set TJVR_REFERENCE_ROOT and TJVR_EIGEN_INCLUDE to enable. Optionally set
TJVR_TRACE_DIRECTORY to compare all .tjvr files there. No source checkout edits.
"""
import os
from pathlib import Path
import subprocess
import struct
import tempfile
import unittest
import zlib

import numpy as np
from scipy.spatial.transform import Rotation

from tests.test_legacy_pico_palm import _packet
from tests.test_reference_tjvr import changed, decode
from tianji_teleop.hand_tracking.tjvr_trace import iter_tjvr_records
from tianji_teleop.hand_tracking.reference_tjvr_stream import ReferenceTjvrStreamGate


@unittest.skipUnless(os.environ.get('TJVR_REFERENCE_ROOT'), 'optional original C++ source oracle')
class ReferenceCppTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix='tjvr-reference-')
        cls.addClassCleanup(cls.directory.cleanup)
        root = Path(os.environ['TJVR_REFERENCE_ROOT'])
        cls.binary = Path(cls.directory.name) / 'protocol_probe'
        subprocess.run(['g++', '-std=c++17', '-O2',
            '-I' + str(root / 'include'), '-I' + os.environ['TJVR_EIGEN_INCLUDE'],
            str(Path(__file__).parent / 'cpp/reference_tjvr_probe.cpp'),
            str(root / 'src/pico_teleop_protocol.cpp'), str(root / 'src/so3.cpp'),
            '-o', str(cls.binary)], check=True)

    def compare_packets(self, packets):
        gate = ReferenceTjvrStreamGate(.15, .6)
        reasons = {'none': 0, 'zero_epoch': 1, 'epoch_rollback': 2,
            'out_of_order': 3, 'position_jump': 4, 'orientation_jump': 5}
        with subprocess.Popen([str(self.binary)], stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, text=True, bufsize=1) as process:
            try:
                for index, packet in enumerate(packets):
                    process.stdin.write(packet.hex() + '\n')
                    process.stdin.flush()
                    output = process.stdout.readline().split()
                    self.assertTrue(output, f'oracle exited at packet {index}')
                    try:
                        decoded = decode(packet)
                    except ValueError:
                        self.assertEqual(output, ['rejected'], f'packet {index}')
                        continue
                    self.assertEqual(output[0], 'accepted', f'packet {index}')
                    f = decoded.frame
                    metadata = [f.sequence, f.tracking_epoch, f.source_timestamp_ns,
                        f.bridge_send_monotonic_ns, int(decoded.user_button_pressed),
                        int(decoded.left_arm_direction_valid), int(decoded.right_arm_direction_valid),
                        int(f.upper_limb_skeleton_valid), int(f.upper_limb_rotations_valid)]
                    self.assertEqual(list(map(int, output[1:10])), metadata, f'packet {index}')
                    values = []
                    for pose in (f.left_pose, f.right_pose):
                        values.extend(pose[:3])
                        values.extend(Rotation.from_quat(pose[3:]).as_matrix().ravel())
                    values.extend(decoded.left_arm_direction)
                    values.extend(decoded.right_arm_direction)
                    values.extend(f.upper_limb_points.ravel())
                    values.extend(Rotation.from_quat(f.upper_limb_rotations_xyzw).as_matrix().ravel())
                    np.testing.assert_allclose(np.array(output[10:-4], dtype=float), values,
                        rtol=0, atol=1e-12, err_msg=f'packet {index}')
                    decision = gate.evaluate(f)
                    self.assertEqual(list(map(int, output[-4:])), [int(decision.accepted),
                        int(decision.epoch_changed), reasons[decision.reason],
                        int(decision.stream_discontinuity)], f'gate packet {index}')
            finally:
                process.stdin.close()
                process.stdout.close()
                if process.poll() is None:
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
            self.assertEqual(process.returncode, 0)

    def test_synthetic_protocol_boundaries(self):
        packets = [_packet(version=v, flags=f)
            for v, f in ((1, 15), (2, 63), (3, 127), (4, 255), (4, 511))]
        for offset in (68, 124, 396, 620):
            for w in (0., 1.0005, 1.01, float('nan'), float('inf')):
                packets.append(changed(_packet(), offset, [0., 0., 0., w]))
        for flags in (0xcf, 0xff):
            for direction in ([3., 4., 0.], [0., 0., 0.], [1e-10, 0., 0.], [float('nan'), 0., 0.]):
                packets.append(changed(_packet(flags=flags), 156, direction))
        packets.extend([b'bad', bytes(656)])
        self.compare_packets(packets)

    def test_stream_transitions(self):
        packets = []
        for sequence, epoch, x, w in ((1, 9, 10., 1.), (3, 9, 10.3, 1.),
                (2, 9, 10.31, 1.), (4, 9, 10.31, 1.), (5, 9, 10.32, 1.),
                (6, 9, 11., 1.), (7, 9, 10.33, 1.), (8, 9, 11., 1.),
                (9, 9, 12., 1.), (10, 9, 12.01, 1.), (11, 9, 12.02, 1.),
                (1, 10, 15., 1.), (12, 9, 10., 1.), (2, 10, 15., 0.),
                (3, 10, 15., 0.), (4, 10, 15., 0.), (5, 10, 15., 1.)):
            packet = bytearray(changed(_packet(), 44, [x, 11., 12.,
                1. if w == 0. else 0., 0., 0., w]))
            struct.pack_into('<QQ', packet, 8, sequence, epoch)
            struct.pack_into('<I', packet, len(packet) - 4, zlib.crc32(packet[:-4]))
            packets.append(bytes(packet))
        self.compare_packets(packets)

    @unittest.skipUnless(os.environ.get('TJVR_TRACE_DIRECTORY'), 'optional private recordings')
    def test_recordings(self):
        paths = sorted(Path(os.environ['TJVR_TRACE_DIRECTORY']).glob('*.tjvr'))
        self.assertTrue(paths, 'no .tjvr recordings found')
        for path in paths:
            with self.subTest(trace=path.name), path.open('rb') as stream:
                self.compare_packets(record.packet for record in iter_tjvr_records(stream))
