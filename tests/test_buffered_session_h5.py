import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import h5py
import numpy as np

from tianji_teleop.recording.buffered_session_h5 import BufferedSessionH5Writer
from tests import test_manus_recording_check as manus_fixture


class BufferedSessionH5Test(unittest.TestCase):
    def test_equal_length_udp_packets_flush_without_vlen_broadcast(self):
        from tianji_teleop.hand_tracking.reference_tjvr_receiver import ReferenceTjvrReceiver
        from tests.test_reference_tjvr_receiver import packet
        from tianji_teleop.recording.session_h5 import SessionH5Reader
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'packets.h5'
            with BufferedSessionH5Writer(path, source_type='vr_manus_sim', robot_model='spark',
                    router_zid='router', schema_version='1.2') as writer:
                receiver = ReferenceTjvrReceiver('test', .15, .6, raw_frame_sink=writer.append_raw_reference_tjvr)
                for sequence in range(1, 5):
                    receiver.ingest(packet(sequence), sequence * 10_000_000)
            with SessionH5Reader(path) as reader:
                self.assertEqual(len(reader.read_raw_reference_tjvr()), 4)

    def test_buffered_raw_callbacks_and_audit_match_immediate_writer(self):
        with tempfile.TemporaryDirectory() as directory:
            immediate, buffered = [Path(directory) / name for name in ('immediate.h5', 'buffered.h5')]
            fixture = manus_fixture.ManusRecordingCheckTest()
            fixture.recording(immediate)
            with patch('tests.test_manus_recording_check.SessionH5Writer', BufferedSessionH5Writer):
                fixture.recording(buffered)
            with h5py.File(immediate) as expected, h5py.File(buffered) as actual:
                def compare(name, value):
                    if isinstance(value, h5py.Dataset):
                        a, b = value[()], actual[name][()]
                        self.assertEqual(a.shape, b.shape, name)
                        if h5py.check_dtype(vlen=value.dtype) not in (None, str, bytes):
                            for left, right in zip(a, b):
                                np.testing.assert_array_equal(left, right, err_msg=name)
                        else:
                            np.testing.assert_array_equal(a, b, err_msg=name)
                expected.visititems(compare)
                self.assertTrue(actual.attrs['complete'])

    def test_buffer_is_bounded_and_abort_keeps_incomplete_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'bounded.h5'
            writer = BufferedSessionH5Writer(path, source_type='vr_manus_sim', robot_model='spark',
                router_zid='router', schema_version='1.2', flush_interval_s=60)
            for index in range(300):
                writer.append_dual_audit('lifecycle', {'index': index}, received_timestamp_ns=index + 1)
                self.assertTrue(all(len(rows) < 128 for _, rows in writer._columns.values()))
            writer.abort()
            with h5py.File(path) as file:
                self.assertFalse(file.attrs['complete'])
                self.assertEqual(len(file['meta/dual_audit/kind']), 300)
