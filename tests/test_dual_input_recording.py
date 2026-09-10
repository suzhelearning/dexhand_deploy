from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import h5py

from tianji_teleop.recording.session_h5 import SessionH5Writer, SessionH5Reader, SessionH5Error
from tianji_teleop.hand_tracking.reference_tjvr import parse_reference_tjvr_packet
from tests.test_reference_tjvr_receiver import packet


class DualInputRecordingTest(unittest.TestCase):
    def test_callback_stage_is_distinct_from_raw_manus_nodes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'test.h5'
            with SessionH5Writer(path, source_type='vr_manus_sim', robot_model='spark',
                    router_zid='router', schema_version='1.2') as writer:
                self.assertTrue(hasattr(writer, 'append_manus_callback'))
                writer.append_manus_callback([.01] * 126, callback_sequence=1,
                    received_timestamp_ns=1000, receiver_instance_id='manus-callback')
            with SessionH5Reader(path) as reader:
                rows = reader.read_manus_callbacks()
                self.assertEqual(reader.stream('manus_callbacks'), rows)
                self.assertEqual(rows[0]['points'], [.01] * 126)
                self.assertEqual(rows[0]['callback_sequence'], 1)
                self.assertEqual(reader.read_raw_manus(), [])  # do not fabricate raw25

    def test_no_overwrite_uses_atomic_exclusive_create(self):
        original = h5py.File
        modes = []
        def opening(path, mode, *args, **kwargs):
            modes.append(mode)
            return original(path, mode, *args, **kwargs)
        with tempfile.TemporaryDirectory() as directory:
            with patch('tianji_teleop.recording.session_h5.h5py.File', side_effect=opening):
                with SessionH5Writer(Path(directory) / 'test.h5', source_type='vr_manus_sim',
                        robot_model='spark', router_zid='router', schema_version='1.2'):
                    pass
        self.assertEqual(modes, ['x'])

    def test_reference_raw_roundtrip_preserves_duplicates_and_original_receive_time(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'session.h5'
            with SessionH5Writer(path, source_type='vr_manus_sim', robot_model='spark',
                    router_zid='router', schema_version='1.2', metadata={'calibration_sha256': 'test'}) as writer:
                self.assertTrue(hasattr(writer, 'append_raw_reference_tjvr'))
                for seq, receive in ((1, 1000), (2, 1010)):
                    frame = parse_reference_tjvr_packet(packet(1), receiver_instance_id='receiver',
                        receiver_frame_sequence=seq, received_timestamp_ns=receive)
                    writer.append_raw_reference_tjvr(frame)
            with SessionH5Reader(path) as reader:
                rows = reader.read_raw_reference_tjvr()
                self.assertEqual(reader.stream('raw_reference_tjvr'), rows)
                self.assertEqual(len(rows), 2)
                self.assertEqual([row['time_ns'] for row in rows], [0, 10])
                self.assertEqual([row['received_timestamp_ns'] for row in rows], [1000, 1010])
                for row in rows:
                    self.assertEqual(row['raw_packet'], packet(1))
                    self.assertEqual(row['receiver_instance_id'], 'receiver')
                self.assertEqual(reader.read_arm_command('left'), [])
                self.assertEqual(reader.read_hand_tracking_metadata()['calibration_sha256'], 'test')

    def test_legacy_schema_does_not_accept_new_raw_stream(self):
        with tempfile.TemporaryDirectory() as directory:
            with SessionH5Writer(Path(directory) / 'old.h5', source_type='hand_tracking_sim',
                    robot_model='old', router_zid='router', schema_version='1.1') as writer:
                frame = parse_reference_tjvr_packet(packet(1), receiver_instance_id='receiver',
                    receiver_frame_sequence=1, received_timestamp_ns=1000)
                self.assertTrue(hasattr(writer, 'append_raw_reference_tjvr'))
                with self.assertRaises(SessionH5Error):
                    writer.append_raw_reference_tjvr(frame)
