import json
from pathlib import Path
import tempfile
import unittest

import h5py

from tianji_teleop.recording.session_h5 import SessionH5Writer, SessionH5Reader, SessionH5Error


class DualAuditRecordingTest(unittest.TestCase):
    def make(self, path):
        return SessionH5Writer(path, source_type='vr_manus_sim', robot_model='spark',
                               router_zid='router', schema_version='1.2')

    def test_audit_preserves_native_state_and_never_becomes_control_events(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'session.h5'
            with self.make(path) as writer:
                self.assertTrue(hasattr(writer, 'append_dual_audit'))
                writer.append_dual_audit('operator_result', {'action': 'start', 'accepted': True},
                                         received_timestamp_ns=1000)
                writer.append_dual_audit('native_cycle', {'execution_epoch': 2, 'q': [1., 2.]},
                                         received_timestamp_ns=1005)
            with SessionH5Reader(path) as reader:
                rows = reader.read_dual_audit()
                self.assertEqual([row['kind'] for row in rows], ['operator_result', 'native_cycle'])
                self.assertEqual(rows[1]['payload'], {'execution_epoch': 2, 'q': [1., 2.]})
                self.assertEqual(rows[1]['received_timestamp_ns'], 1005)
                self.assertEqual(reader.read_arm_command('left'), [])

    def test_old_12_without_optional_audit_remains_readable(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'old.h5'
            with self.make(path):
                pass
            with h5py.File(path, 'a') as file:
                if 'dual_audit' in file['meta']:
                    del file['meta/dual_audit']
            with SessionH5Reader(path) as reader:
                self.assertTrue(hasattr(reader, 'read_dual_audit'))
                self.assertEqual(reader.read_dual_audit(), [])

    def test_batch_matches_single_rows_without_reordering_or_partial_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            batch_path, single_path = (Path(directory) / name for name in ('batch.h5', 'single.h5'))
            rows = [('native_cycle', {'sequence': i}, stamp)
                    for i, stamp in enumerate((1000, 1005, 1003, 1010))]
            with self.make(batch_path) as writer:
                self.assertTrue(hasattr(writer, 'append_dual_audit_batch'))
                with self.assertRaises(SessionH5Error):
                    writer.append_dual_audit_batch(rows + [('native_cycle', {'q': float('nan')}, 1011)])
                self.assertIsNone(writer._timeline_start)
                self.assertEqual(writer._file['meta/dual_audit/time_ns'].shape, (0,))
                writer.append_dual_audit_batch(rows)
            with self.make(single_path) as writer:
                for kind, payload, stamp in rows:
                    writer.append_dual_audit(kind, payload, received_timestamp_ns=stamp)
            with SessionH5Reader(batch_path) as batch, SessionH5Reader(single_path) as single:
                self.assertEqual(batch.read_dual_audit(), single.read_dual_audit())

    def test_bad_audit_is_rejected_before_append_and_on_read(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'bad.h5'
            with self.make(path) as writer:
                self.assertTrue(hasattr(writer, 'append_dual_audit'))
                for kind, payload, stamp in [('typo', {}, 1), ([], {}, 1), ('native_cycle', {'v': float('nan')}, 1),
                                             ('native_cycle', {}, True), ('native_cycle', [], 1)]:
                    with self.assertRaises(SessionH5Error):
                        writer.append_dual_audit(kind, payload, received_timestamp_ns=stamp)
                writer.append_dual_audit('native_cycle', {}, received_timestamp_ns=1000)
            with h5py.File(path, 'a') as file:
                file['meta/dual_audit/payload_json'][0] = json.dumps({'q': float('nan')})
            with self.assertRaises(SessionH5Error):
                SessionH5Reader(path)
