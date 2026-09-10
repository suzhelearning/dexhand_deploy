from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import h5py
import numpy as np

from tianji_teleop.protocol.messages import HAND_JOINT_NAMES, HandJointCommand
from tianji_teleop.recording.session_h5 import SessionH5Writer


class BufferedSessionWriterTest(unittest.TestCase):
    def test_failed_buffer_flush_cannot_mark_file_complete(self):
        from tianji_teleop.recording.buffered_writer import BufferedSessionWriter
        with tempfile.TemporaryDirectory() as directory:
            writer = BufferedSessionWriter(Path(directory) / 'partial.h5',
                source_type='pico2_hands_sim', robot_model='test', router_zid='router',
                schema_version='1.2', flush_interval_s=60)
            writer.append_dual_audit('operator_result', {'test': True}, received_timestamp_ns=1000)
            try:
                with patch.object(h5py.Dataset, 'resize', side_effect=OSError('disk failed')):
                    with self.assertRaisesRegex(OSError, 'disk failed'):
                        writer.close()
                self.assertFalse(writer._file.attrs['complete'])
            finally:
                writer.abort()

    def test_buffers_remain_bounded_and_explicit_flush_preserves_audit_order(self):
        from tianji_teleop.recording.buffered_writer import BufferedSessionWriter
        with tempfile.TemporaryDirectory() as directory:
            writer = BufferedSessionWriter(Path(directory) / 'audit.h5',
                source_type='pico2_hands_sim', robot_model='test', router_zid='router',
                schema_version='1.2', flush_interval_s=60)
            try:
                for seq in range(600):
                    writer.append_dual_audit('operator_observation', {'sequence': seq},
                        received_timestamp_ns=1000 + seq)
                    self.assertTrue(all(len(rows) < 256 for _, rows in writer._rows.values()))
                writer.flush()
                np.testing.assert_array_equal(writer._file['meta/dual_audit/received_timestamp_ns'][:],
                                              np.arange(1000, 1600))
            finally:
                writer.close()

    def test_buffered_rows_match_legacy_file_with_fewer_dataset_resizes(self):
        from tianji_teleop.recording.buffered_writer import BufferedSessionWriter
        with tempfile.TemporaryDirectory() as directory:
            paths, counts = [], []
            resize = h5py.Dataset.resize
            for cls in (SessionH5Writer, BufferedSessionWriter):
                path = Path(directory) / (cls.__name__ + '.h5')
                paths.append(path)
                writer = cls(path, source_type='pico2_hands_sim', robot_model='test',
                    router_zid='router', schema_version='1.2', flush_interval_s=60)
                calls = []
                def resize_count(dataset, *args, **kwargs):
                    calls.append(dataset.name)
                    return resize(dataset, *args, **kwargs)
                with patch.object(h5py.Dataset, 'resize', resize_count):
                    from tests.test_pico_hand_service import wire_frame
                    for seq in range(20):
                        command = HandJointCommand(1, seq, 1000 + seq, 'hand', 'left',
                            list(HAND_JOINT_NAMES['left']), [.01 * seq] * 20, 'producer', 'router')
                        writer.append_hand_command(command, received_time_ns=1000 + seq)
                        writer.append_dual_audit('operator_observation', {'sequence': seq},
                            received_timestamp_ns=1000 + seq)
                        writer.append_raw_pico(wire_frame(), received_time_ns=1000 + seq)
                    writer.close()
                counts.append(len(calls))
            self.assertLess(counts[1], counts[0] / 2)
            with h5py.File(paths[0]) as first, h5py.File(paths[1]) as second:
                def compare(name, item):
                    if isinstance(item, h5py.Dataset):
                        self.assertEqual(item.shape, second[name].shape, name)
                        if isinstance(h5py.check_dtype(vlen=item.dtype), np.dtype):
                            for old, new in zip(item[...], second[name][...]):
                                np.testing.assert_array_equal(old, new, err_msg=name)
                        else:
                            np.testing.assert_array_equal(item[...], second[name][...], err_msg=name)
                first.visititems(compare)
