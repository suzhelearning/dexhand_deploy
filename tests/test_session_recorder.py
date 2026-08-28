from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from pico_body_tianji.protocol.messages import ProtocolEnvelope, RawPicoControllerSample
from pico_body_tianji.recording.recorder import RecorderProtocolError, SessionRecorderNode
from pico_body_tianji.recording.session_h5 import IncompleteSessionError, SessionH5Reader
from pico_body_tianji.protocol import topics


class _Session:
    def __init__(self):
        self.subscriptions = []
    def declare_subscriber(self, key, callback):
        self.subscriptions.append((key, callback))
        return SimpleNamespace(undeclare=lambda: None)


class SessionRecorderTest(unittest.TestCase):
    def test_profile_selects_only_matching_raw_and_strictly_records_typed_message(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "session.h5"
            session = _Session()
            node = SessionRecorderNode(session, path, source_type="pico_controller", robot_model="marvin", router_zid="router")
            keys = {key for key, _ in session.subscriptions}
            self.assertIn(topics.RAW_PICO_CONTROLLER, keys)
            self.assertNotIn(topics.RAW_MOCAP_LIVE, keys)
            sample = RawPicoControllerSample(ProtocolEnvelope(1, "source-instance", "router", 1, 100), None, [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], False)
            node.receive(topics.RAW_PICO_CONTROLLER, json.dumps(sample.to_dict()).encode(), received_time_ns=1000)
            node.close()
            with SessionH5Reader(path) as reader:
                self.assertEqual(reader.read_raw_pico()[0]["publisher_instance_id"], "source-instance")

    def test_unknown_raw_type_fails_and_keeps_file_incomplete(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "session.h5"
            node = SessionRecorderNode(_Session(), path, source_type="pico_controller", robot_model="marvin", router_zid="router")
            with self.assertRaises(RecorderProtocolError):
                node.receive(topics.RAW_PICO_CONTROLLER, b'{"schema_version":1,"source_type":"mocap_live"}')
            node.abort()
            with self.assertRaises(IncompleteSessionError):
                SessionH5Reader(path)

    def test_recording_never_overwrites_existing_path(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "session.h5"
            path.touch()
            with self.assertRaises(FileExistsError):
                SessionRecorderNode(_Session(), path, source_type="pico_controller", robot_model="marvin", router_zid="router")


if __name__ == "__main__":
    unittest.main()
