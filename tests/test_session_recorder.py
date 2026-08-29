from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import tempfile
import textwrap
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

    def test_recording_config_is_applied_to_writer(self):
        with tempfile.TemporaryDirectory() as directory:
            node = SessionRecorderNode(
                _Session(),
                Path(directory) / "session.h5",
                source_type="pico_controller",
                robot_model="marvin",
                router_zid="router",
                recording_config={
                    "flush_interval_s": 0.25,
                    "schema_name": "tianji-teleop-session",
                    "schema_version": "1.0",
                },
            )
            self.assertEqual(node.recording_config["flush_interval_s"], 0.25)
            self.assertEqual(node.writer._flush_interval_s, 0.25)
            node.close()

    def test_sigterm_gracefully_closes_recorder(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "marker.json"
            config = Path(__file__).parents[1] / "src" / "pico_body_tianji" / "config" / "recording" / "session.yaml"
            code = textwrap.dedent(
                """
                import json
                import os
                import pico_body_tianji.recording.session_recorder as module

                marker = os.environ["MARKER"]
                class FakeSession:
                    def close(self):
                        pass
                class FakeNode:
                    def __init__(self, *args, **kwargs):
                        with open(marker, "w", encoding="utf-8") as handle:
                            json.dump({"closed": False, "kwargs": kwargs}, handle)
                    def flush(self):
                        pass
                    def close(self):
                        with open(marker, "r+", encoding="utf-8") as handle:
                            value = json.load(handle)
                            value["closed"] = True
                            handle.seek(0)
                            json.dump(value, handle)
                            handle.truncate()
                module.open_session = lambda: FakeSession()
                module.require_single_router = lambda session, expected: "router"
                module.SessionRecorderNode = FakeNode
                raise SystemExit(module.main())
                """
            )
            env = os.environ.copy()
            env.update(
                {
                    "PYTHONPATH": str(Path(__file__).parents[1] / "src" / "pico_body_tianji")
                    + os.pathsep
                    + env.get("PYTHONPATH", ""),
                    "TIANJI_RECORD_PATH": str(Path(directory) / "session.h5"),
                    "TIANJI_RECORD_SOURCE_TYPE": "pico_controller",
                    "TIANJI_COMPONENT_INSTANCE_ID": "recorder-instance",
                    "TIANJI_ROUTER_ZID": "router",
                    "TIANJI_RECORDING_CONFIG": str(config),
                    "MARKER": str(marker),
                }
            )
            process = subprocess.Popen([sys.executable, "-c", code], env=env)
            try:
                for _ in range(100):
                    if marker.exists():
                        break
                    __import__("time").sleep(0.01)
                self.assertTrue(marker.exists(), "recorder child did not initialize")
                process.send_signal(signal.SIGTERM)
                process.wait(timeout=3)
                value = json.loads(marker.read_text(encoding="utf-8"))
                self.assertTrue(value["closed"])
                self.assertEqual(value["kwargs"]["recording_config"]["schema_version"], "1.0")
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
    def test_unexpected_flush_exception_aborts_and_keeps_recording_incomplete(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "marker.json"
            config = Path(__file__).parents[1] / "src" / "pico_body_tianji" / "config" / "recording" / "session.yaml"
            code = textwrap.dedent(
                """
                import json
                import os
                import pico_body_tianji.recording.session_recorder as module

                marker = os.environ["MARKER"]
                class FakeSession:
                    def close(self):
                        pass
                class FakeNode:
                    def __init__(self, *args, **kwargs):
                        with open(marker, "w", encoding="utf-8") as handle:
                            json.dump({"closed": False, "aborted": False}, handle)
                    def flush(self):
                        raise RuntimeError("flush failed")
                    def close(self):
                        with open(marker, "r+", encoding="utf-8") as handle:
                            value = json.load(handle)
                            value["closed"] = True
                            handle.seek(0)
                            json.dump(value, handle)
                            handle.truncate()
                    def abort(self):
                        with open(marker, "r+", encoding="utf-8") as handle:
                            value = json.load(handle)
                            value["aborted"] = True
                            handle.seek(0)
                            json.dump(value, handle)
                            handle.truncate()
                module.open_session = lambda: FakeSession()
                module.require_single_router = lambda session, expected: "router"
                module.SessionRecorderNode = FakeNode
                module.main()
                """
            )
            env = os.environ.copy()
            env.update(
                {
                    "PYTHONPATH": str(Path(__file__).parents[1] / "src" / "pico_body_tianji")
                    + os.pathsep
                    + env.get("PYTHONPATH", ""),
                    "TIANJI_RECORD_PATH": str(Path(directory) / "session.h5"),
                    "TIANJI_RECORD_SOURCE_TYPE": "pico_controller",
                    "TIANJI_COMPONENT_INSTANCE_ID": "recorder-instance",
                    "TIANJI_ROUTER_ZID": "router",
                    "TIANJI_RECORDING_CONFIG": str(config),
                    "MARKER": str(marker),
                }
            )
            process = subprocess.Popen([sys.executable, "-c", code], env=env)
            process.wait(timeout=3)
            value = json.loads(marker.read_text(encoding="utf-8"))
            self.assertNotEqual(process.returncode, 0)
            self.assertTrue(value["aborted"])
            self.assertFalse(value["closed"])


if __name__ == "__main__":
    unittest.main()
