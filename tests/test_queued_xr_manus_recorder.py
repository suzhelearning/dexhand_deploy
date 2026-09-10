from pathlib import Path
from threading import Event, Thread
import tempfile
import unittest
from unittest.mock import patch

from tests.test_dual_input_recorder import Session


class QueuedXrManusRecorderTest(unittest.TestCase):
    def _node(self, path, **kwargs):
        from tianji_teleop.recording.queued_xr_manus import QueuedXrManusRecorder

        return QueuedXrManusRecorder(
            Session(),
            path,
            source_type="vr_manus_xr_sim",
            input_profile="manus",
            robot_model="test",
            router_zid="router",
            recording_config={
                "schema_name": "tianji-teleop-session",
                "schema_version": "1.2",
                "flush_interval_s": 0.5,
            },
            **kwargs,
        )

    def test_only_xr_manus_uses_fifo_recorder(self):
        from tianji_teleop.recording import session_recorder
        from tianji_teleop.recording.queued_xr_manus import QueuedXrManusRecorder

        self.assertIs(session_recorder._recorder_class("vr_manus_xr_sim"), QueuedXrManusRecorder)
        self.assertIsNot(session_recorder._recorder_class("vr_manus_sim"), QueuedXrManusRecorder)

    def test_network_ingress_does_not_wait_for_hdf5_and_close_drains_fifo(self):
        with tempfile.TemporaryDirectory() as directory:
            node = self._node(Path(directory) / "session.h5")
            entered, release, returned = Event(), Event(), Event()
            rows = []

            def slow_receive(key, payload, **kwargs):
                if not rows:
                    entered.set()
                    release.wait(3)
                rows.append((key, payload))

            try:
                with patch.object(node, "receive", side_effect=slow_receive):
                    node._on_sample("first", b"first")
                    self.assertTrue(entered.wait(1))

                    def enqueue_second():
                        node._on_sample("second", b"second")
                        returned.set()

                    callback = Thread(target=enqueue_second)
                    callback.start()
                    self.assertTrue(returned.wait(0.5), "network callback blocked behind HDF5")
                    release.set()
                    callback.join(1)
                    node.close()
                self.assertEqual(rows, [("first", b"first"), ("second", b"second")])
                self.assertFalse(node._dispatcher.is_alive())
            finally:
                release.set()
                node.close()


if __name__ == "__main__":
    unittest.main()
