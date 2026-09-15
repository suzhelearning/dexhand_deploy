"""Native encoding/transport against existing HDF5 writer; no live sockets."""
from pathlib import Path
import os
import shutil
import socket
import subprocess
import tempfile
import threading
import unittest
import h5py
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which('c++'), 'C++ compiler required')
class NativeRecordingTransportTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        assert (ROOT / 'native/control/hdf5_stream_client.hpp').is_file(), 'native recorder client missing'
        cls.temp = tempfile.TemporaryDirectory(prefix='native-recording-tests-')
        cls.addClassCleanup(cls.temp.cleanup)
        cls.binary = Path(cls.temp.name) / 'driver'
        flags = ['-O2']
        if os.environ.get('NATIVE_RECORDING_SANITIZERS') == '1':
            flags = ['-O1', '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-pie', '-no-pie']
        subprocess.run(['c++', '-std=c++17', '-pthread', *flags, '-Wall', '-Wextra', '-Werror',
                        str(ROOT / 'tests/cpp/native_recording_transport.cpp'), '-o', str(cls.binary)], check=True)

    def test_batch_validation(self):
        subprocess.run([str(self.binary), 'validate'], check=True, timeout=10)

    def test_arm_cycle_columns_match_canonical_writer(self):
        from tianji_teleop.recording.session_h5 import SessionH5Writer, SessionH5Reader
        from tianji_teleop.protocol.messages import ArmJointCommand, ArmJointState, SessionState
        writer_binary = ROOT / 'build/hdf5_recorder/tianji_hdf5_recorder'
        if not writer_binary.is_file(): self.skipTest('build-hdf5-recorder required')
        with tempfile.TemporaryDirectory() as directory:
            expected, actual = [Path(directory) / name for name in ('expected.h5', 'actual.h5')]
            kwargs = dict(source_type='vr_manus_sim', robot_model='spark', router_zid='offline', schema_version='1.2')
            writer = SessionH5Writer(expected, **kwargs)
            names = [f'Joint{j}_{side}' for side in ('L', 'R') for j in range(1, 8)]
            for i in range(2):
                phase = 'returning' if i else 'teleop'
                for s, side in enumerate(('left', 'right')):
                    writer.append_arm_command(ArmJointCommand(1, 3+i+s, 100+i, 'arm', side, phase,
                        None if s else 2, None if s else 1, names[s*7:s*7+7],
                        [(-1. if s else 1.)+i]*7, 'command', 'offline'), received_time_ns=100+i)
                writer.append_arm_state(ArmJointState(1, 5+i, 100+i, 'mujoco', names, [.5+i]*14,
                    None, 'feedback', 'offline'), received_time_ns=100+i)
                writer.append_session_state(SessionState(1, i+1, 100+i, phase, '记录✓', 'keyboard',
                    None if i else 3, 'session', 'offline'), received_time_ns=100+i)
            writer.close()
            initialized = SessionH5Writer(actual, **kwargs); initialized.abort()
            parent, child = socket.socketpair()
            with parent, child:
                disk = subprocess.Popen([str(writer_binary), str(actual)], stdin=child, stdout=child, stderr=subprocess.PIPE)
                try:
                    child.close()
                    result = subprocess.run([str(self.binary), str(parent.fileno()), 'arm_cycle'],
                        pass_fds=(parent.fileno(),), capture_output=True, text=True, timeout=10)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    parent.close(); _, errors = disk.communicate(timeout=5)
                    self.assertEqual(disk.returncode, 0, errors)
                finally:
                    if disk.poll() is None: disk.kill()
                    disk.communicate()
            with h5py.File(expected) as left, h5py.File(actual) as right:
                self.assertEqual(dict(left.attrs), dict(right.attrs))
                def compare(name, value):
                    self.assertEqual(dict(value.attrs), dict(right[name].attrs), name)
                    if isinstance(value, h5py.Dataset):
                        self.assertEqual(value.dtype, right[name].dtype)
                        np.testing.assert_array_equal(value[()], right[name][()], err_msg=name)
                left.visititems(compare)
            with SessionH5Reader(actual) as reader:
                self.assertEqual(len(reader.read_arm_state()), 2)

    def test_existing_schema_and_native_disk_writer(self):
        from tianji_teleop.recording.session_h5 import SessionH5Writer
        from tests.test_reference_tjvr_receiver import packet
        from tianji_teleop.hand_tracking.reference_tjvr import parse_reference_tjvr_packet
        writer_binary = ROOT / 'build/hdf5_recorder/tianji_hdf5_recorder'
        if not writer_binary.is_file():
            self.skipTest('build-hdf5-recorder required')
        for complete in (True, False):
            with self.subTest(complete=complete), tempfile.TemporaryDirectory() as directory:
                expected, actual = [Path(directory) / name for name in ('expected.h5', 'actual.h5')]
                kwargs = dict(source_type='vr_manus_sim', robot_model='spark', router_zid='offline', schema_version='1.2')
                reference = SessionH5Writer(expected, **kwargs)
                packets = [packet(1), packet(2)]
                for index in range(2):
                    reference.append_dual_audit('lifecycle', {'text': '记录✓'}, received_timestamp_ns=100 + index)
                for index, raw in enumerate(packets):
                    observation = parse_reference_tjvr_packet(raw, receiver_instance_id='offline-source',
                        receiver_frame_sequence=index+1, received_timestamp_ns=102+index)
                    reference.append_raw_reference_tjvr(observation)
                reference.close() if complete else reference.abort()
                initialized = SessionH5Writer(actual, **kwargs)
                initialized.abort()  # Schema creation is still cold-path Python.
                parent, child = socket.socketpair()
                with parent, child:
                    disk = subprocess.Popen([str(writer_binary), str(actual)], stdin=child, stdout=child, stderr=subprocess.PIPE)
                    try:
                        child.close()
                        client = subprocess.run([str(self.binary), str(parent.fileno()), 'complete' if complete else 'incomplete'],
                            pass_fds=(parent.fileno(),), input='\n'.join(p.hex() for p in packets)+'\n',
                            text=True, capture_output=True, timeout=10)
                        self.assertEqual(client.returncode, 0, client.stderr)
                        parent.close()
                        _, errors = disk.communicate(timeout=5)
                        self.assertEqual(disk.returncode, 0, errors)
                    finally:
                        if disk.poll() is None:
                            disk.kill()
                        disk.communicate()
                with h5py.File(expected) as left, h5py.File(actual) as right:
                    self.assertEqual(dict(left.attrs), dict(right.attrs))
                    def compare(name, value):
                        self.assertEqual(dict(value.attrs), dict(right[name].attrs), name)
                        if isinstance(value, h5py.Dataset):
                            self.assertEqual(value.dtype, right[name].dtype, name)
                            a, b = value[()], right[name][()]
                            if value.dtype.kind == 'O' and len(a) and isinstance(a[0], np.ndarray):
                                for x, y in zip(a, b): np.testing.assert_array_equal(x, y)
                                self.assertEqual(len(a), len(b))
                            else:
                                np.testing.assert_array_equal(a, b, err_msg=name)
                    left.visititems(compare)

    def test_transport_rejects_bad_ack_eof_timeout_and_cancellation(self):
        for mode in ('error', 'eof', 'timeout', 'cancel', 'oversized', 'bad_ready', 'write_timeout'):
            with self.subTest(mode=mode):
                parent, peer = socket.socketpair()
                release = threading.Event()
                def server():
                    with peer:
                        try:
                            peer.sendall(b'BAD\n' if mode == 'bad_ready' else b'READY 1\n')
                            if mode == 'bad_ready': return
                            if mode == 'write_timeout':
                                release.wait(2)
                                return
                            peer.recv(4096)
                            if mode == 'error': peer.sendall(b'ERROR\n')
                            elif mode == 'oversized': peer.sendall(b'x' * 257)
                            elif mode in ('timeout', 'cancel'): release.wait(2)
                        except (BrokenPipeError, ConnectionResetError):
                            pass
                thread = threading.Thread(target=server)
                thread.start()
                try:
                    result = subprocess.run([str(self.binary), str(parent.fileno()), mode],
                        pass_fds=(parent.fileno(),), capture_output=True, text=True, timeout=5)
                    self.assertEqual(result.returncode, 0, result.stderr)
                finally:
                    parent.close(); release.set(); thread.join(3)
                self.assertFalse(thread.is_alive())

    def test_numeric_wire_and_attributes(self):
        writer_binary = ROOT / 'build/hdf5_recorder/tianji_hdf5_recorder'
        if not writer_binary.is_file():
            self.skipTest('build-hdf5-recorder required')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'numeric.h5'
            with h5py.File(path, 'x') as file:
                file.attrs['complete'] = False
                for name, dtype, tail in [('i', '<i8', ()), ('f', '<f8', (2,)), ('u', 'u1', ()),
                                          ('v', h5py.vlen_dtype(np.dtype('u1')), ())]:
                    file.create_dataset(name, shape=(0, *tail), maxshape=(None, *tail), dtype=dtype)
            parent, child = socket.socketpair()
            with parent, child:
                disk = subprocess.Popen([str(writer_binary), str(path)], stdin=child, stdout=child, stderr=subprocess.PIPE)
                try:
                    child.close()
                    result = subprocess.run([str(self.binary), str(parent.fileno()), 'numeric'],
                        pass_fds=(parent.fileno(),), capture_output=True, text=True, timeout=10)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    parent.close()
                    _, errors = disk.communicate(timeout=5)
                    self.assertEqual(disk.returncode, 0, errors)
                finally:
                    if disk.poll() is None: disk.kill()
                    disk.communicate()
            with h5py.File(path) as file:
                self.assertTrue(file.attrs['complete'])
                self.assertEqual(file.attrs['logical_id'], '记录✓')
                np.testing.assert_array_equal(file['i'][:], [-1, -(2**63)])
                np.testing.assert_array_equal(file['f'][:], [[1.25, -2.5], [np.nan, 0.]])
                np.testing.assert_array_equal(file['u'][:], [0, 255])
                self.assertEqual(file['v'][0].size, 0)
                np.testing.assert_array_equal(file['v'][1], [1, 255])

    def test_disk_rejection_cannot_be_finalized_complete(self):
        writer_binary = ROOT / 'build/hdf5_recorder/tianji_hdf5_recorder'
        if not writer_binary.is_file():
            self.skipTest('build-hdf5-recorder required')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'failed.h5'
            with h5py.File(path, 'x') as file:
                file.attrs['complete'] = False
            parent, child = socket.socketpair()
            with parent, child:
                disk = subprocess.Popen([str(writer_binary), str(path)], stdin=child, stdout=child, stderr=subprocess.PIPE)
                try:
                    child.close()
                    result = subprocess.run([str(self.binary), str(parent.fileno()), 'invalid_column'],
                        pass_fds=(parent.fileno(),), capture_output=True, text=True, timeout=10)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    parent.close()
                    disk.communicate(timeout=5)
                    self.assertNotEqual(disk.returncode, 0)
                finally:
                    if disk.poll() is None: disk.kill()
                    disk.communicate()
            with h5py.File(path) as file:
                self.assertFalse(file.attrs['complete'])
