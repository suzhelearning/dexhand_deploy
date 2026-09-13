import importlib.util
import os
import signal
from pathlib import Path
import tempfile
from threading import Event
import unittest
from unittest.mock import patch

from tianji_teleop.recording.session_h5 import SessionH5Reader, IncompleteSessionError


class AsyncDualRecordingTest(unittest.TestCase):
    def test_drain_timeout_interrupts_owned_writer_and_joins_dispatcher(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = self.make(Path(directory) / 'blocked.h5')
            os.kill(recorder._writer.worker_pid, signal.SIGSTOP)
            original_join = recorder._thread.join
            def expired_join(timeout=None):
                # Simulate expiration of the existing 30s drain budget only.
                if timeout != 30:
                    original_join(timeout)
            try:
                recorder.append('append_dual_audit', 'native_cycle', {'large': 'x' * 2_000_000},
                    received_timestamp_ns=1000)
                with patch.object(recorder._thread, 'join', side_effect=expired_join):
                    with self.assertRaisesRegex(RuntimeError, 'drain timeout'):
                        recorder.close()
                self.assertFalse(recorder._thread.is_alive(), 'dispatcher left running after timeout')
                self.assertFalse(recorder._writer._process.is_alive(), 'writer orphaned after timeout')
            finally:
                if recorder._writer._process.is_alive():
                    recorder._writer._process.kill()
                    recorder._writer._process.join(2)
                original_join(5)

    def test_pico_raw_process_recording_preserves_source_packet_and_geometry(self):
        from tests.test_pico_hand_service import wire_frame
        frame = wire_frame()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'pico.h5'
            with self.make(path, source_type='pico2_hands_sim', robot_model='marvin_pico') as recorder:
                recorder.append('append_raw_pico', frame)
            with SessionH5Reader(path) as reader:
                self.assertEqual(reader.file.attrs['source_type'], 'pico2_hands_sim')
                self.assertEqual(reader.file.attrs['robot_model'], 'marvin_pico')
                rows = reader.read_raw_pico()
                self.assertEqual(len(rows), 1)
                self.assertEqual(rows[0]['raw_packet'], frame.raw_packet)
                self.assertEqual(rows[0]['receiver_frame_sequence'], frame.receiver_frame_sequence)
                for side, hand in frame.hands.items():
                    self.assertEqual(rows[0]['hands'][side]['joint_poses'],
                                     [list(joint.pose) for joint in hand.joints])
                self.assertEqual(reader.read_manus_callbacks(), [])

    def test_new_recording_source_requires_explicit_model_and_simulation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'invalid.h5'
            for options in ({'source_type': 'pico2_hands_sim'},
                            {'source_type': 'pico2_hands_real', 'robot_model': 'robot'},
                            {'source_type': 'vr_manus_real', 'robot_model': 'robot'}):
                with self.subTest(options=options), self.assertRaises(ValueError):
                    self.make(path, **options)
                self.assertFalse(path.exists())

    def test_exclusive_input_recording_rejects_other_mode_before_enqueue(self):
        with tempfile.TemporaryDirectory() as directory:
            for source, methods in (
                    ('pico2_hands_sim', ('append_manus_callback', 'append_raw_reference_tjvr')),
                    ('vr_manus_sim', ('append_raw_pico',))):
                with self.subTest(source=source):
                    with self.make(Path(directory) / (source + '.h5'), source_type=source,
                                   robot_model='explicit-model') as recorder:
                        for method in methods:
                            with self.assertRaisesRegex(ValueError, 'input mode'):
                                recorder.append(method)
                        self.assertIsNone(recorder.failure)

    def make(self, path, **kwargs):
        self.assertIsNotNone(importlib.util.find_spec('tianji_teleop.recording.async_dual'))
        from tianji_teleop.recording.async_dual import AsyncDualRecorder
        return AsyncDualRecorder(path, router_zid='router', metadata={'run_id': 'test'}, **kwargs)

    def test_close_drains_in_order_and_copies_mutable_payloads(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'record.h5'
            with self.make(path) as recorder:
                self.assertTrue(hasattr(recorder._writer, 'worker_pid'))
                self.assertNotEqual(recorder._writer.worker_pid, os.getpid())
                payload = {'q': [1.]}
                for sequence in range(1, 10):
                    recorder.append('append_dual_audit', 'native_cycle', payload,
                                    received_timestamp_ns=sequence)
                payload['q'][0] = 9.
            stats = recorder.statistics
            self.assertEqual(stats['backend'], 'native_cpp')
            self.assertEqual(stats['accepted'], stats['processed'])
            self.assertEqual(stats['accepted'], 9)
            self.assertEqual(stats['queue_depth'], 0)
            self.assertLessEqual(stats['queue_high_water'], stats['queue_capacity'])
            with SessionH5Reader(path) as reader:
                rows = reader.read_dual_audit()
                self.assertEqual(len(rows), 9)
                self.assertEqual([r['received_timestamp_ns'] for r in rows], list(range(1, 10)))
                self.assertTrue(all(r['payload']['q'] == [1.] for r in rows))

    def test_overload_never_blocks_or_marks_partial_file_complete(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'partial.h5'
            entered, release = Event(), Event()
            recorder = self.make(path, capacity=1)
            original = recorder._writer.append_dual_audit
            def stalled(*args, **kwargs):
                entered.set()
                if not release.wait(3):
                    raise RuntimeError('test stalled')
                return original(*args, **kwargs)
            try:
                with patch.object(recorder._writer, 'append_dual_audit', side_effect=stalled):
                    recorder.append('append_dual_audit', 'native_cycle', {}, received_timestamp_ns=1)
                    self.assertTrue(entered.wait(2))
                    recorder.append('append_dual_audit', 'native_cycle', {}, received_timestamp_ns=2)
                    with self.assertRaisesRegex(RuntimeError, 'overflow'):
                        recorder.append('append_dual_audit', 'native_cycle', {}, received_timestamp_ns=3)
                    self.assertIn('overflow', recorder.failure)
                    release.set()
                    with self.assertRaisesRegex(RuntimeError, 'overflow'):
                        recorder.close()
            finally:
                release.set()
            with self.assertRaises(IncompleteSessionError):
                SessionH5Reader(path)
            with SessionH5Reader(path, allow_incomplete=True):
                pass

    def test_consecutive_audits_batch_without_crossing_callback_boundary(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'batch.h5'
            entered, release = Event(), Event()
            recorder = self.make(path)
            original = recorder._writer.append_dual_audit
            def stalled(*args, **kwargs):
                entered.set()
                if not release.wait(3):
                    raise RuntimeError('test stalled')
                return original(*args, **kwargs)
            try:
                with patch.object(recorder._writer, 'append_dual_audit', side_effect=stalled), \
                     patch.object(recorder._writer, 'append_dual_audit_batch',
                                  wraps=recorder._writer.append_dual_audit_batch) as batches:
                    recorder.append('append_dual_audit', 'native_cycle', {'i': 0}, received_timestamp_ns=1000)
                    self.assertTrue(entered.wait(2))
                    for i in range(1, 41):
                        if i == 21:
                            recorder.append('append_manus_callback', [0.] * 126, callback_sequence=1,
                                received_timestamp_ns=1020, receiver_instance_id='manus', single_hand_side='right')
                        recorder.append('append_dual_audit', 'native_cycle', {'i': i}, received_timestamp_ns=1000+i)
                    release.set()
                    recorder.close()
                    # The first singleton is dispatched by append_dual_audit;
                    # its internal HDF5 implementation now runs in the child.
                    # Native adapter shares canonical singleton validation in
                    # this thread; then the two dispatcher batches follow.
                    self.assertEqual([len(call.args[0]) for call in batches.call_args_list], [1, 20, 20])
            finally:
                release.set()
                recorder.close()
            with SessionH5Reader(path) as reader:
                self.assertEqual([r['payload']['i'] for r in reader.read_dual_audit()], list(range(41)))
                self.assertEqual(len(reader.read_manus_callbacks()), 1)

    def test_consecutive_live_snapshots_batch_without_crossing_other_records(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'snapshot-batch.h5'
            recorder = self.make(path)
            calls = []

            def capture_batch(rows):
                calls.append(len(rows))

            try:
                with patch.object(recorder._writer, 'append_live_cycle_snapshot_batch',
                                  side_effect=capture_batch):
                    for index in range(4):
                        recorder.append_live_cycle_snapshot(
                            object(), run_id=f'run-{index}')
                    recorder.append('append_dual_audit', 'operator_result', {'ok': True},
                                    received_timestamp_ns=100)
                    recorder.close(complete=False)
            except Exception:
                recorder.close(complete=False)
                raise
            self.assertEqual(calls, [4])

    def test_exception_aborts_and_existing_files_are_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'partial.h5'
            with self.assertRaisesRegex(ValueError, 'operator test'):
                with self.make(path):
                    raise ValueError('operator test')
            with self.assertRaises(IncompleteSessionError):
                SessionH5Reader(path)
            previous = path.read_bytes()
            with self.assertRaises((ValueError, FileExistsError)):
                self.make(path)
            self.assertEqual(path.read_bytes(), previous)

    def test_close_error_attempts_abort_and_releases_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'close_error.h5'
            recorder = self.make(path)
            with patch.object(recorder._writer, 'close', side_effect=OSError('disk close error')):
                with self.assertRaisesRegex(RuntimeError, 'close failed'):
                    recorder.close()
            self.assertTrue(recorder._writer._closed)
            with self.assertRaises(IncompleteSessionError):
                SessionH5Reader(path)
