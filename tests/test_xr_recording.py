from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace

import numpy as np

from tianji_teleop.hand_tracking.reference_manus_process import ManusCallback
from tianji_teleop.hand_tracking.xr_input import (
    XrControllerState,
    XrFrame,
    XrTrackerState,
)
from tianji_teleop.hand_tracking.xr_manus_runtime import (
    decode_manus_callback,
    encode_manus_callback,
)
from tianji_teleop.hand_tracking.xr_operator import (
    encode_xr_operator_observation,
)
from tianji_teleop.hand_tracking.operator_input import OperatorObservation
from tianji_teleop.protocol import topics
from tianji_teleop.protocol.messages import HandJointCommand, HandTargetCommand, HAND_JOINT_NAMES
from tianji_teleop.recording.recorder import SessionRecorderNode
from tianji_teleop.recording.session_h5 import SessionH5Reader, SessionH5Writer


class _Session:
    def __init__(self):
        self.keys = []

    def declare_subscriber(self, key, callback):
        self.keys.append(key)
        return SimpleNamespace(undeclare=lambda: None)


def _xr_frame() -> XrFrame:
    controller = {
        side: XrControllerState(
            side=side,
            pose=np.array([0.1 if side == "left" else 0.2, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]),
            valid=True,
            available=True,
            trigger=0.25,
            grip=0.75,
            axis=np.array([0.1, -0.2]),
        )
        for side in ("left", "right")
    }
    trackers = (
        XrTrackerState(
            serial_number="190058",
            side="left",
            pose=np.array([0.3, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0]),
            valid=True,
            velocity=np.arange(6, dtype=np.float64),
            acceleration=np.arange(6, dtype=np.float64) + 10.0,
        ),
        XrTrackerState(
            serial_number="190600",
            side="right",
            pose=np.array([0.4, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0]),
            valid=True,
            velocity=np.arange(6, dtype=np.float64) + 20.0,
            acceleration=np.arange(6, dtype=np.float64) + 30.0,
        ),
    )
    return XrFrame(
        sequence=4,
        source_timestamp_ns=1234,
        received_timestamp_ns=2000,
        hmd_pose=np.array([0.0, 0.0, 1.6, 0.0, 0.0, 0.0, 1.0]),
        controllers=controller,
        trackers=trackers,
        receiver_instance_id="xr-receiver",
        connection_generation=2,
    )


class XrRecordingTest(unittest.TestCase):
    def test_xr_frame_wire_round_trip_is_strict_and_complete(self):
        frame = _xr_frame()
        decoded = XrFrame.from_dict(frame.to_dict())
        self.assertEqual(decoded.to_dict(), frame.to_dict())
        with self.assertRaises(ValueError):
            XrFrame.from_dict({**frame.to_dict(), "unexpected": True})

    def test_writer_round_trips_complete_xr_frame(self):
        frame = _xr_frame()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "xr.h5"
            with SessionH5Writer(
                path,
                source_type="vr_manus_xr_sim",
                robot_model="marvin",
                router_zid="router",
                schema_version="1.2",
            ) as writer:
                writer.append_raw_xr(frame)
            with SessionH5Reader(path) as reader:
                rows = reader.read_raw_xr()
                self.assertEqual(len(rows), 1)
                self.assertEqual(rows[0]["receiver_instance_id"], "xr-receiver")
                self.assertEqual(rows[0]["connection_generation"], 2)
                self.assertEqual(rows[0]["receiver_frame_sequence"], 4)
                self.assertEqual(rows[0]["tracker_count"], 2)
                self.assertEqual(rows[0]["frame"]["controllers"]["right"]["grip"], 0.75)
                self.assertEqual(rows[0]["frame"]["trackers"][1]["serial_number"], "190600")
                np.testing.assert_allclose(rows[0]["right_controller_pose"], frame.controller("right").pose)

    def test_new_recorder_subscribes_to_xr_and_manus_callback_only_for_new_profile(self):
        frame = _xr_frame()
        callback = ManusCallback(
            receiver_instance_id="manus-receiver",
            sequence=8,
            received_timestamp_ns=3000,
            points=tuple(np.arange(126, dtype=np.float64)),
            source_sequences={"right": 11, "left": 12},
                source_timestamps_ns={"right": 101, "left": 102},
        )
        operator_observation = encode_xr_operator_observation(
            OperatorObservation(
                source="xr-observation",
                side="right",
                sequence=frame.sequence,
                epoch=frame.connection_generation,
                receive_time_ns=frame.received_timestamp_ns,
                valid=True,
                available=True,
                action="start_request",
                pressed=True,
                confidence=1.0,
            ),
            router_zid="router",
            publisher_instance_id="xr-observation",
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "recorded.h5"
            session = _Session()
            node = SessionRecorderNode(
                session,
                path,
                source_type="vr_manus_xr_sim",
                input_profile="manus",
                robot_model="marvin",
                router_zid="router",
                recording_config={
                    "flush_interval_s": 1.0,
                    "schema_name": "tianji-teleop-session",
                    "schema_version": "1.2",
                },
            )
            try:
                self.assertIn(topics.RAW_XR_INPUT, session.keys)
                self.assertEqual(session.keys.count(topics.RAW_MANUS_CALLBACK), 1)
                self.assertEqual(session.keys.count(topics.MANUS_INPUT_AUDIT), 1)
                self.assertIn(topics.XR_OPERATOR_OBSERVATION, session.keys)
                node.receive(topics.RAW_XR_INPUT, {**frame.to_dict(), "router_zid": "router"})
                node.receive(topics.RAW_MANUS_CALLBACK, encode_manus_callback(callback, "router"))
                node.receive(topics.MANUS_INPUT_AUDIT, {
                    "schema_version": 1,
                    "kind": "manus_rawviz_line",
                    "router_zid": "router",
                    "run_id": "run-1",
                    "received_timestamp_ns": 3000,
                    "line_sequence": 1,
                    "text": "HAND 1 1",
                    "terminator": "LF",
                    "input_stage": "rawviz_stdout_before_parser",
                })
                node.receive(topics.MANUS_INPUT_AUDIT, {
                    "schema_version": 1,
                    "kind": "manus_callback_metadata",
                    "router_zid": "router",
                    "run_id": "run-1",
                    "received_timestamp_ns": 3000,
                    "callback_sequence": 8,
                    "receiver_instance_id": "manus-receiver",
                    "source_sequences": {"right": 11, "left": 12},
                    "source_timestamps_ns": {"right": 101, "left": 102},
                })
                node.receive(topics.XR_OPERATOR_OBSERVATION, operator_observation)
            finally:
                node.close()
            with SessionH5Reader(path) as reader:
                self.assertEqual(len(reader.read_raw_xr()), 1)
                callbacks = reader.read_manus_callbacks()
                self.assertEqual(callbacks[0]["source_sequences"], {"right": 11, "left": 12})
                audit = reader.read_dual_audit()
                self.assertEqual(
                    [row["kind"] for row in audit if row["kind"].startswith("manus_")],
                    ["manus_rawviz_line", "manus_callback_metadata"],
                )
                operator_rows = [
                    row for row in audit if row["kind"] == "operator_observation"
                ]
                self.assertEqual(len(operator_rows), 1)
                self.assertEqual(operator_rows[0]["payload"], operator_observation)

    def test_manus_input_audit_rejects_missing_run_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            session = _Session()
            node = SessionRecorderNode(
                session,
                Path(directory) / "recorded.h5",
                source_type="vr_manus_xr_sim",
                input_profile="manus",
                robot_model="marvin",
                router_zid="router",
                recording_config={
                    "flush_interval_s": 1.0,
                    "schema_name": "tianji-teleop-session",
                    "schema_version": "1.2",
                },
            )
            try:
                with self.assertRaises(ValueError):
                    node.receive(topics.MANUS_INPUT_AUDIT, {
                        "schema_version": 1,
                        "kind": "manus_rawviz_line",
                        "router_zid": "router",
                        "run_id": "",
                        "received_timestamp_ns": 3000,
                        "line_sequence": 1,
                        "text": "HAND 1 1",
                        "terminator": "LF",
                        "input_stage": "rawviz_stdout_before_parser",
                    })
            finally:
                node.close()

    def test_xr_manus_recorder_keeps_target_to_command_output_audit(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "recorded.h5"
            session = _Session()
            node = SessionRecorderNode(
                session,
                path,
                source_type="vr_manus_xr_sim",
                input_profile="manus",
                robot_model="marvin",
                router_zid="router",
                recording_config={
                    "flush_interval_s": 1.0,
                    "schema_name": "tianji-teleop-session",
                    "schema_version": "1.2",
                },
            )
            target = HandTargetCommand(
                1,
                17,
                5000,
                4000,
                "hand_tracking_target",
                "right",
                "wrist_relative_mediapipe",
                np.zeros((21, 3), dtype=np.float64).tolist(),
                "target-publisher",
                "router",
            )
            command = HandJointCommand(
                1,
                31,
                6000,
                "wuji_retarget_right",
                "right",
                list(HAND_JOINT_NAMES["right"]),
                [0.1] * 20,
                "hand-producer",
                "router",
            )
            try:
                self.assertIn(topics.HAND_OUTPUT_AUDIT, session.keys)
                node.receive(topics.HAND_OUTPUT_AUDIT, {
                    "schema_version": 1,
                    "kind": "hand_output",
                    "router_zid": "router",
                    "run_id": "run-1",
                    "executor_instance_id": "executor-right",
                    "side": "right",
                    "target": target.to_dict(),
                    "command": command.to_dict(),
                })
            finally:
                node.close()
            with SessionH5Reader(path) as reader:
                rows = [row for row in reader.read_dual_audit() if row["kind"] == "hand_output"]
                self.assertEqual(len(rows), 1)
                self.assertEqual(rows[0]["payload"]["target"]["sequence"], 17)
                self.assertEqual(rows[0]["payload"]["command"]["sequence"], 31)

    def test_legacy_vr_manus_profile_does_not_subscribe_to_xr_input(self):
        with tempfile.TemporaryDirectory() as directory:
            session = _Session()
            node = SessionRecorderNode(
                session,
                Path(directory) / "legacy.h5",
                source_type="vr_manus_sim",
                input_profile="manus",
                robot_model="marvin",
                router_zid="router",
                recording_config={
                    "flush_interval_s": 1.0,
                    "schema_name": "tianji-teleop-session",
                    "schema_version": "1.2",
                },
            )
            try:
                self.assertNotIn(topics.RAW_XR_INPUT, session.keys)
                self.assertNotIn(topics.RAW_MANUS_CALLBACK, session.keys)
            finally:
                node.close()


if __name__ == "__main__":
    unittest.main()
