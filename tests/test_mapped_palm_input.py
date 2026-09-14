from pathlib import Path
import struct
import tempfile
import unittest
import zlib
from dataclasses import replace
import numpy as np
from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver
from tianji_teleop.hand_tracking.reference_tjvr import parse_reference_tjvr_packet
from tianji_teleop.hand_tracking.mapped_palm_input import select_mapped_palm_frame
from tests.test_reference_tjvr_receiver import packet


class MappedPalmInputTest(unittest.TestCase):
    def test_hdf5_invalid_skeleton_rows_still_require_raw_identity_and_order(self):
        from tianji_teleop.recording.session_h5 import SessionH5Writer
        from tianji_teleop.recording.live_capture import LiveCapture
        from tianji_teleop.recording.tjvr_check import check_tjvr_recording
        for corruption in (None, 'identity', 'sequence', 'time'):
            with self.subTest(corruption=corruption), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'mapped.h5'
                with SessionH5Writer(path, source_type='vr_manus_sim', robot_model='mapped_palm',
                        router_zid='router', schema_version='1.2', metadata=dict(run_id='run',
                        resolved_configuration=dict(tjvr_stream_contract=dict(version=1, initial_state='reset',
                            max_position_jump_m=.15, max_orientation_jump_rad=.6,
                            target_source='mapped_corrected_palm')))) as writer:
                    class Recorder:
                        def append(self, method, *args, **kwargs):
                            getattr(writer, method)(*args, **kwargs)
                    capture = LiveCapture(Recorder(), run_id='run')
                    def record(frame):
                        if not frame.frame.upper_limb_rotations_valid:
                            changes = {'identity': dict(receiver_instance_id='foreign'),
                                       'sequence': dict(receiver_frame_sequence=1),
                                       'time': dict(received_timestamp_ns=1)}.get(corruption, {})
                            frame = replace(frame, frame=replace(frame.frame, **changes))
                        capture.tjvr(frame)
                    receiver = ReferenceTjvrReceiver('mapped', .15, .6, target_source='mapped_corrected_palm',
                        raw_frame_sink=record, decision_sink=capture.tjvr_decision)
                    receiver.ingest(packet(1), 1_000_000_000)
                    invalid = bytearray(packet(2))
                    struct.pack_into('<I', invalid, 40, 0x7f)
                    struct.pack_into('<I', invalid, len(invalid)-4, zlib.crc32(invalid[:-4]))
                    receiver.ingest(bytes(invalid), 1_005_000_000)
                    receiver.ingest(packet(3), 1_010_000_000)
                if corruption:
                    with self.assertRaises(ValueError):
                        check_tjvr_recording(path)
                else:
                    report = check_tjvr_recording(path)
                    self.assertTrue(report['passed'], report)
                    self.assertEqual(report['raw_frames'], 3)
                    self.assertEqual(report['invalid_selected_targets'], 1)
                    self.assertEqual(report['matched_decisions'], 2)

    def test_selected_skeleton_not_packet_pose_drives_jump_gate(self):
        receiver = ReferenceTjvrReceiver('mapped', .15, .6, target_source='mapped_corrected_palm')
        self.assertTrue(receiver.ingest(packet(1, x=1), 1_000_000_000).accepted)
        self.assertTrue(receiver.ingest(packet(2, x=10), 1_005_000_000).accepted)
        changed = bytearray(packet(3, x=10))
        struct.pack_into('<d', changed, 204 + 3 * 24, 100.)
        struct.pack_into('<I', changed, len(changed)-4, zlib.crc32(changed[:-4]))
        self.assertEqual(receiver.ingest(bytes(changed), 1_010_000_000).reason, 'position_jump')

    def test_mapped_basis_and_raw_provenance(self):
        raw = packet(1)
        decoded = parse_reference_tjvr_packet(raw, receiver_instance_id='x', receiver_frame_sequence=1,
                                              received_timestamp_ns=1_000_000_000).frame
        mapped = select_mapped_palm_frame(decoded)
        np.testing.assert_allclose(mapped.left_pose[:3], decoded.upper_limb_points[3])
        np.testing.assert_allclose(mapped.right_pose[:3], decoded.upper_limb_points[7])
        self.assertEqual(mapped.raw_packet, raw)

    def test_hdf5_check_replays_selected_target_contract(self):
        from tianji_teleop.recording.session_h5 import SessionH5Writer
        from tianji_teleop.recording.live_capture import LiveCapture
        from tianji_teleop.recording.tjvr_check import check_tjvr_recording
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'mapped.h5'
            with SessionH5Writer(path, source_type='vr_manus_sim', robot_model='mapped_palm',
                    router_zid='router', schema_version='1.2', metadata=dict(run_id='run',
                    resolved_configuration=dict(tjvr_stream_contract=dict(version=1, initial_state='reset',
                        max_position_jump_m=.15, max_orientation_jump_rad=.6,
                        target_source='mapped_corrected_palm')))) as writer:
                class Recorder:
                    def append(self, method, *args, **kwargs):
                        getattr(writer, method)(*args, **kwargs)
                capture = LiveCapture(Recorder(), run_id='run')
                receiver = ReferenceTjvrReceiver('mapped', .15, .6, target_source='mapped_corrected_palm',
                    raw_frame_sink=capture.tjvr, decision_sink=capture.tjvr_decision)
                receiver.ingest(packet(1, x=1), 1_000_000_000)
                receiver.ingest(packet(2, x=10), 1_005_000_000)
            report = check_tjvr_recording(path)
            self.assertTrue(report['passed'], report)
            self.assertEqual(report['accepted'], 2)
