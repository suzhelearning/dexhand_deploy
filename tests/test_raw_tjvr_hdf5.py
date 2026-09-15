"""C++ raw packet + ingress audit blocks against the canonical HDF5 writer."""
import json
from pathlib import Path
import socket
import subprocess
import tempfile
import unittest

import h5py
import numpy as np

from tests.test_native_raw_input import compile_driver
from tests.test_reference_tjvr_receiver import packet
from tianji_teleop.hand_tracking.reference_tjvr import parse_reference_tjvr_packet
from tianji_teleop.recording.session_h5 import SessionH5Writer

ROOT = Path(__file__).resolve().parents[1]


class RawTjvrHdf5Test(unittest.TestCase):
    def test_complete_raw_and_audit_match_and_invalid_stays_incomplete(self):
        self.assertTrue((ROOT / 'native/control/raw_tjvr_hdf5.hpp').is_file(),
                        'native raw recording encoder missing')
        disk_binary = ROOT / 'build/hdf5_recorder/tianji_hdf5_recorder'
        if not disk_binary.is_file():
            self.skipTest('build-hdf5-recorder required')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            binary = root / 'raw'
            compile_driver('raw_tjvr_hdf5_fixture.cpp', binary)
            kwargs = dict(source_type='vr_manus_sim', robot_model='spark', router_zid='offline',
                          schema_version='1.2')
            rows = [dict(sequence=i+1, received_ns=100+i, origin_ns=100, receiver='source-记录',
                         accepted=i != 1, bytes=list(packet(1 if i < 2 else 2))) for i in range(3)]
            # Separate output queues need not be globally timestamp ordered.
            rows[1]['received_ns'] = 99
            expected = root / 'expected.h5'
            reference = SessionH5Writer(expected, **kwargs)
            for row in rows:
                reference.append_raw_reference_tjvr(parse_reference_tjvr_packet(bytes(row['bytes']),
                    receiver_instance_id=row['receiver'], receiver_frame_sequence=row['sequence'],
                    received_timestamp_ns=row['received_ns']))
                reference.append_dual_audit('native_tjvr_ingress', dict(version=1,
                    receiver_instance_id=row['receiver'], receiver_frame_sequence=row['sequence'],
                    received_timestamp_ns=row['received_ns'], accepted=row['accepted'],
                    parser_error=None, run_id='offline-run'), received_timestamp_ns=row['received_ns'])
            reference.close()
            variants = [None, dict(rows[0], bytes=[1, 2, 3]), dict(rows[0], sequence=2**63),
                        dict(rows[0], received_ns=0), dict(rows[0], origin_ns=-1),
                        dict(rows[0], receiver=''), dict(rows[0], receiver='x\x00y')]
            for index, invalid in enumerate(variants):
                actual = root / f'actual-{index}.h5'
                SessionH5Writer(actual, **kwargs).abort()
                parent, child = socket.socketpair()
                with parent, child:
                    disk = subprocess.Popen([str(disk_binary), str(actual)], stdin=child,
                                            stdout=child, stderr=subprocess.PIPE)
                    try:
                        child.close()
                        result = subprocess.run([str(binary), str(parent.fileno())],
                            input='\n'.join(map(json.dumps, rows if invalid is None else [rows[0], invalid])),
                            pass_fds=(parent.fileno(),), text=True, capture_output=True, timeout=10)
                        parent.close()
                        _, errors = disk.communicate(timeout=5)
                        if invalid is None:
                            self.assertEqual(result.returncode, 0, result.stderr)
                            self.assertEqual(disk.returncode, 0, errors)
                        else:
                            self.assertNotEqual(result.returncode, 0)
                    finally:
                        if disk.poll() is None:
                            disk.kill()
                        disk.communicate()
                with h5py.File(actual) as actual_file:
                    self.assertEqual(bool(actual_file.attrs['complete']), invalid is None)
                    if invalid is not None:
                        self.assertEqual(len(actual_file['raw/tjvr_upper_limb/time_ns']), 1)
                        self.assertEqual(len(actual_file['meta/dual_audit/time_ns']), 1)
                        continue
                    with h5py.File(expected) as expected_file:
                        def compare(name, value):
                            other = actual_file[name]
                            self.assertEqual(dict(value.attrs), dict(other.attrs), name)
                            if not isinstance(value, h5py.Dataset):
                                return
                            self.assertEqual(value.dtype, other.dtype, name)
                            if name.endswith('payload_json'):
                                self.assertEqual(list(map(json.loads, value[:])), list(map(json.loads, other[:])))
                            elif name.endswith('raw_packet'):
                                self.assertEqual(len(value), len(other))
                                for a, b in zip(value, other):
                                    np.testing.assert_array_equal(a, b)
                            else:
                                np.testing.assert_array_equal(value[()], other[()], err_msg=name)
                        expected_file.visititems(compare)
