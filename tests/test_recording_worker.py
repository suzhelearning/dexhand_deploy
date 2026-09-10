import importlib.util
import os
import signal
from threading import Event, Thread
from pathlib import Path
import tempfile
import unittest

from tianji_teleop.recording.session_h5 import SessionH5Reader, IncompleteSessionError


class RecordingWorkerTest(unittest.TestCase):
    def test_interrupt_releases_blocked_pipe_send_and_never_completes_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = self.make(Path(directory) / 'blocked.h5')
            self.addCleanup(writer.abort)
            self.assertTrue(hasattr(writer, 'interrupt'), 'missing owned-worker interruption')
            entered = Event()
            errors = []
            def send():
                entered.set()
                try:
                    writer.append_dual_audit('native_cycle', {'large': 'x' * 2_000_000},
                        received_timestamp_ns=1000)
                except RuntimeError as exc:
                    errors.append(str(exc))
            os.kill(writer.worker_pid, signal.SIGSTOP)
            thread = Thread(target=send, daemon=True)
            try:
                thread.start()
                self.assertTrue(entered.wait(1))
                thread.join(.05)
                self.assertTrue(thread.is_alive(), 'fixture did not block IPC')
                writer.interrupt()
                thread.join(3)
                self.assertFalse(thread.is_alive())
                # interrupt only signals; the caller owns process reap. Pipe
                # EOF can precede the OS reporting the child fully exited.
                writer._process.join(3)
                self.assertFalse(writer._process.is_alive())
                self.assertTrue(errors)
                with self.assertRaises(RuntimeError):
                    writer.close()
            finally:
                if writer._process.is_alive():
                    writer._process.kill()
                    writer._process.join(2)
                thread.join(3)

    def make(self, path):
        self.assertIsNotNone(importlib.util.find_spec('tianji_teleop.recording.writer_process'))
        from tianji_teleop.recording.writer_process import ProcessSessionWriter
        return ProcessSessionWriter(path, source_type='vr_manus_sim', robot_model='spark',
                                    router_zid='router', schema_version='1.2')

    def test_owns_hdf5_in_another_process_and_preserves_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'session.h5'
            writer = self.make(path)
            self.addCleanup(writer.abort)
            self.assertNotEqual(writer.worker_pid, os.getpid())
            writer.append_dual_audit_batch([('native_cycle', {'i': i}, 1000+i) for i in range(10)])
            writer.close()
            self.assertTrue(writer._closed)
            self.assertFalse(writer._process.is_alive())
            with SessionH5Reader(path) as reader:
                self.assertEqual([r['payload']['i'] for r in reader.read_dual_audit()], list(range(10)))

    def test_invalid_write_can_only_be_aborted_and_existing_file_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'session.h5'
            writer = self.make(path)
            self.addCleanup(writer.abort)
            with self.assertRaisesRegex(RuntimeError, 'write failed'):
                writer.append_dual_audit('bad-kind', {}, received_timestamp_ns=1000)
            with self.assertRaisesRegex(RuntimeError, 'failed'):
                writer.close()
            writer.abort()
            self.assertFalse(writer._process.is_alive())
            with self.assertRaises(IncompleteSessionError):
                SessionH5Reader(path)
            previous = path.read_bytes()
            with self.assertRaises(FileExistsError):
                self.make(path)
            self.assertEqual(path.read_bytes(), previous)

    def test_dead_worker_is_not_restarted_or_marked_complete(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = self.make(Path(directory) / 'partial.h5')
            self.addCleanup(writer.abort)
            writer._process.terminate()
            writer._process.join(2)
            with self.assertRaises(RuntimeError):
                writer.flush()
            with self.assertRaises(RuntimeError):
                writer.close()
