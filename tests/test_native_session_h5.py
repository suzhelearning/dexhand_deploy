"""Native backend must preserve the canonical recording contract."""
from tests import test_buffered_session_h5 as fixture
from unittest.mock import patch
import tempfile
from pathlib import Path
import h5py
import numpy as np


class NativeSessionH5Test(fixture.BufferedSessionH5Test):
    def setUp(self):
        from tianji_teleop.recording.native_session_h5 import NativeSessionH5Writer
        replacement = patch('tests.test_buffered_session_h5.BufferedSessionH5Writer', NativeSessionH5Writer)
        replacement.start()
        self.addCleanup(replacement.stop)

    def test_joint_data_and_dynamic_attributes_match_canonical_writer(self):
        from tianji_teleop.recording.native_session_h5 import NativeSessionH5Writer
        from tianji_teleop.recording.session_h5 import SessionH5Writer
        from tests.test_session_h5 import _arm_command, _arm_state, _hand_command
        with tempfile.TemporaryDirectory() as directory:
            paths = [Path(directory) / name for name in ('python.h5', 'native.h5')]
            for cls, path in zip((SessionH5Writer, NativeSessionH5Writer), paths):
                with cls(path, source_type='vr_manus_sim', robot_model='spark', router_zid='router',
                         schema_version='1.2') as writer:
                    for index in range(150):
                        writer.append_arm_command(_arm_command(), received_time_ns=index + 1)
                        writer.append_arm_state(_arm_state(), received_time_ns=index + 1)
                        writer.append_hand_command(_hand_command(), received_time_ns=index + 1)
                        writer.append_dual_audit('lifecycle', {'text': '记录✓' * 100}, received_timestamp_ns=index + 1)
            with h5py.File(paths[0]) as expected, h5py.File(paths[1]) as actual:
                self.assertEqual(dict(expected.attrs), dict(actual.attrs))
                def compare(name, value):
                    self.assertEqual(dict(value.attrs), dict(actual[name].attrs), name)
                    if isinstance(value, h5py.Dataset):
                        self.assertEqual(value.dtype, actual[name].dtype, name)
                        self.assertEqual(value.maxshape, actual[name].maxshape, name)
                        np.testing.assert_array_equal(value[()], actual[name][()], err_msg=name)
                expected.visititems(compare)

    def test_native_protocol_error_leaves_incomplete_file_and_reaps_child(self):
        from tianji_teleop.recording.native_session_h5 import NativeSessionH5Writer
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'bad.h5'
            writer = NativeSessionH5Writer(path, source_type='vr_manus_sim', robot_model='spark',
                router_zid='router', schema_version='1.2')
            try:
                with self.assertRaisesRegex(RuntimeError, 'rejected block'):
                    writer._exchange(b'Z')
            finally:
                writer.abort()
            self.assertFalse(writer._process.is_alive())
            with h5py.File(path) as f:
                self.assertFalse(f.attrs['complete'])

    def test_missing_binary_fails_before_creating_recording(self):
        from tianji_teleop.recording.native_session_h5 import NativeSessionH5Writer
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'missing.h5'
            with self.assertRaisesRegex(RuntimeError, 'build-hdf5-recorder'):
                NativeSessionH5Writer(path, executable=Path(directory) / 'missing', source_type='vr_manus_sim')
            self.assertFalse(path.exists())
