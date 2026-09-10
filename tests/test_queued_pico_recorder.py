from pathlib import Path
from threading import Event, Thread
import tempfile
import unittest
from unittest.mock import patch

from tests.test_dual_input_recorder import Session


class QueuedPicoRecorderTest(unittest.TestCase):
    def test_overflow_is_visible_and_never_marks_capture_complete(self):
        from tianji_teleop.recording.session_h5 import SessionH5Reader, IncompleteSessionError
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'session.h5'
            node = self.node(path, capacity=1)
            entered, release = Event(), Event()
            def slow_receive(*args, **kwargs):
                entered.set()
                release.wait(3)
            try:
                with patch.object(node, 'receive', side_effect=slow_receive):
                    node._on_sample('first', b'first')
                    self.assertTrue(entered.wait(1))
                    node._on_sample('second', b'second')
                    node._on_sample('third', b'third')
                    self.assertTrue(node.failed)
                    self.assertIn('overflow', str(node.failure))
                    with self.assertRaisesRegex(ValueError, 'overflow'):
                        node.flush()
                    release.set()
                    with self.assertRaisesRegex(ValueError, 'overflow'):
                        node.close()
                self.assertFalse(node._dispatcher.is_alive())
                with self.assertRaises(IncompleteSessionError):
                    SessionH5Reader(path)
            finally:
                release.set()
                node.close()

    def test_dispatch_error_is_propagated_and_preserves_incomplete_file(self):
        from tianji_teleop.recording.session_h5 import SessionH5Reader, IncompleteSessionError
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'session.h5'
            node = self.node(path)
            try:
                with patch.object(node, 'receive', side_effect=OSError('disk failed')):
                    node._on_sample('event', b'event')
                    node._dispatcher.join(2)
                    with self.assertRaisesRegex(ValueError, 'disk failed'):
                        node.flush()
                    with self.assertRaisesRegex(ValueError, 'disk failed'):
                        node.close()
                with self.assertRaises(IncompleteSessionError):
                    SessionH5Reader(path)
            finally:
                node.close()

    def node(self, path, **kwargs):
        from tianji_teleop.recording.queued_pico import QueuedPicoRecorder
        return QueuedPicoRecorder(Session(), path, source_type='pico2_hands_sim',
            input_profile='pico', robot_model='test', router_zid='router',
            recording_config={'schema_name': 'tianji-teleop-session', 'schema_version': '1.2',
                              'flush_interval_s': .5}, **kwargs)

    def test_receive_does_not_wait_for_disk_and_close_drains_fifo(self):
        with tempfile.TemporaryDirectory() as directory:
            node = self.node(Path(directory) / 'session.h5')
            entered, release, returned = Event(), Event(), Event()
            rows = []
            def slow_receive(key, payload, **kwargs):
                if not rows:
                    entered.set()
                    release.wait(3)
                rows.append((key, payload))
            try:
                with patch.object(node, 'receive', side_effect=slow_receive):
                    node._on_sample('status', b'first')
                    self.assertTrue(entered.wait(1))
                    def event_callback():
                        node._on_sample('operator_result', b'one-shot')
                        returned.set()
                    callback = Thread(target=event_callback)
                    callback.start()
                    self.assertTrue(returned.wait(.5), 'network callback blocked behind HDF5')
                    release.set()
                    callback.join(1)
                    node.close()
                self.assertEqual(rows, [('status', b'first'), ('operator_result', b'one-shot')])
                self.assertFalse(node._dispatcher.is_alive())
            finally:
                release.set()
                node.close()

    def test_only_new_pico_runtime_selects_fifo_recorder(self):
        from tianji_teleop.recording import session_recorder
        from tianji_teleop.recording.queued_pico import QueuedPicoRecorder
        self.assertIs(session_recorder._recorder_class('pico2_hands_sim'), QueuedPicoRecorder)
        self.assertIs(session_recorder._recorder_class('hand_tracking_sim'), session_recorder.SessionRecorderNode)

    def test_subscription_cleanup_failure_still_stops_dispatcher_and_aborts(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as directory:
            node = self.node(Path(directory) / 'session.h5')
            def broken():
                raise OSError('unsubscribe failed')
            node._resources.append(SimpleNamespace(undeclare=broken))
            try:
                with self.assertRaisesRegex(ValueError, 'unsubscribe failed'):
                    node.close()
                self.assertFalse(node._dispatcher.is_alive())
            finally:
                node._resources.clear()
                node.close()
