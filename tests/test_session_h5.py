from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np

from pico_body_tianji.protocol.messages import (
    ArmJointCommand,
    ArmJointState,
    ArmTargetCommand,
    HandJointCommand,
    HandJointState,
    HandTargetCommand,
    ProtocolEnvelope,
    RawH5ReplaySample,
    RawMocapLiveSample,
    RawPicoControllerSample,
    SessionState,
)
from pico_body_tianji.recording.session_h5 import (
    IncompleteSessionError,
    SessionH5Reader,
    SessionH5Writer,
    UnsafeSessionLinkError,
)


def _env(sequence: int, timestamp_ns: int = 10) -> ProtocolEnvelope:
    return ProtocolEnvelope(1, "instance", "router", sequence, timestamp_ns)


def _arm_target() -> ArmTargetCommand:
    return ArmTargetCommand(_env(1), None, "replay", "right", "Base_R", [1, 2, 3], [0, 0, 0, 1], [1, 0, 0])


def _hand_target() -> HandTargetCommand:
    return HandTargetCommand(1, 2, 20, None, "replay", "right", "wrist_relative_mediapipe", np.zeros((21, 3)).tolist(), "instance", "router")


def _arm_command() -> ArmJointCommand:
    return ArmJointCommand(1, 3, 30, "ik", "right", "teleop", 2, 1, [f"Joint{i}_R" for i in range(1, 8)], [0.1] * 7, "arm-command", "router")


def _arm_state() -> ArmJointState:
    return ArmJointState(1, 4, 40, "mujoco", [f"Joint{i}_L" for i in range(1, 8)] + [f"Joint{i}_R" for i in range(1, 8)], [0.0] * 14, None, "arm-state", "router")


def _hand_command() -> HandJointCommand:
    return HandJointCommand(1, 5, 50, "retarget", "right", [f"r_{x}" for x in ("thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp", "thumb_ip", "index_mcp_flex", "index_mcp_abd", "index_pip", "index_dip", "middle_mcp_flex", "middle_mcp_abd", "middle_pip", "middle_dip", "ring_mcp_flex", "ring_mcp_abd", "ring_pip", "ring_dip", "pinky_mcp_flex", "pinky_mcp_abd", "pinky_pip", "pinky_dip")], [0.0] * 20, "hand-command", "router")


class SessionH5Test(unittest.TestCase):
    def test_appendable_chunked_layout_and_nullable_time_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "session.h5"
            with SessionH5Writer(path, source_type="pico_controller", robot_model="marvin", router_zid="router") as writer:
                writer.append_raw_pico(RawPicoControllerSample(_env(1), None, [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], False), received_time_ns=100)
                writer.append_arm_target(_arm_target(), received_time_ns=125)
                writer.append_hand_target(_hand_target(), received_time_ns=130)
                writer.append_arm_command(_arm_command(), received_time_ns=135)
                writer.append_arm_state(_arm_state(), received_time_ns=140)
                writer.append_hand_command(_hand_command(), received_time_ns=145)
                writer.append_hand_state(HandJointState(1, 6, 60, "wuji", "right", _hand_command().names, [0.0] * 20, None, "hand-state", "router"), received_time_ns=150)
                writer.append_session_state(SessionState(1, 7, 70, "teleop", "accepted", "coordinator", 3, "coordinator", "router"), received_time_ns=155)
            with h5py.File(path, "r") as file:
                self.assertEqual(file.attrs["schema_name"], "tianji-teleop-session")
                self.assertEqual(file.attrs["schema_version"], "1.0")
                self.assertTrue(bool(file.attrs["complete"]))
                for name in ("raw/pico_controller/time_ns", "target/arm/right/time_ns", "joint/state/hand/right/time_ns"):
                    self.assertIsNotNone(file[name].chunks)
                    self.assertEqual(file[name].maxshape[0], None)
                self.assertEqual(file["raw/pico_controller/time_ns"][:].tolist(), [0])
                self.assertEqual(file["target/arm/right"].attrs["frame_id"], "Base_R")
                self.assertNotIn("sequence", file["meta/session_events"])
                self.assertEqual(json.loads(file["joint/command/hand/right"].attrs["joint_names"]), _hand_command().names)
                self.assertFalse(bool(file["joint/state/hand/right/velocity_valid"][0]))
            record = SessionH5Reader(path).read_arm_target("right")[0]
            self.assertIsNone(record["source_timestamp_ns"])
            self.assertEqual(record["time_ns"], 25)

    def test_mocap_valid_and_invalid_rows_keep_big_source_time_exact(self):
        hands = {
            "left": {"valid": True, "wrist_pose": [1, 2, 3, 0, 0, 0, 1], "keypoints_world_m": np.zeros((21, 3)).tolist()},
            "right": {"valid": False, "wrist_pose": None, "keypoints_world_m": None},
        }
        sample = RawMocapLiveSample(_env(8), 2**60 + 17, "stream", 4, 99, hands)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mocap.h5"
            with SessionH5Writer(path, source_type="mocap_live", robot_model="marvin", router_zid="router", clock=lambda: 1000) as writer:
                writer.append_raw_mocap(sample, received_time_ns=None)
            with h5py.File(path, "r") as file:
                self.assertEqual(file["raw/mocap_live/source_time_ns"].dtype, np.dtype("int64"))
                self.assertEqual(int(file["raw/mocap_live/source_time_ns"][0]), 2**60 + 17)
                self.assertFalse(bool(file["raw/mocap_live/right_valid"][0]))
                self.assertTrue(np.isnan(file["raw/mocap_live/right_keypoints_world"][0]).all())
            row = SessionH5Reader(path).read_raw_mocap()[0]
            self.assertEqual(row["source_timestamp_ns"], 2**60 + 17)
            self.assertIsNone(row["hands"]["right"]["wrist_pose"])
    def test_h5_replay_optional_hand_joints_and_parent_rows_round_trip(self):
        hands = {
            "left": {"valid": False, "wrist_pose": None, "keypoints_world_m": None, "wuji2_joints_rad": None},
            "right": {"valid": True, "wrist_pose": [1, 2, 3, 0, 0, 0, 1], "keypoints_world_m": np.zeros((21, 3)).tolist(), "wuji2_joints_rad": [0.25] * 20},
        }
        sample = RawH5ReplaySample(_env(9), 123, hands)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "h5.h5"
            with SessionH5Writer(path, source_type="h5_replay", robot_model="marvin", router_zid="router") as writer:
                writer.append_raw_h5(sample, received_time_ns=100)
            with h5py.File(path, "r") as file:
                self.assertEqual(file["raw/h5_replay/hands/right/wuji2_joints"].shape, (1, 20))
                self.assertEqual(file["raw/h5_replay/hands/left/valid"].shape[0], 1)
            row = SessionH5Reader(path).read_raw_h5()[0]
            self.assertEqual(row["hands"]["right"]["wuji2_joints_rad"], [0.25] * 20)
            self.assertIsNone(row["hands"]["left"]["wuji2_joints_rad"])
    def test_incomplete_is_rejected_by_default_and_allowed_for_diagnostics(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "session.h5"
            writer = SessionH5Writer(path, source_type="h5_replay", robot_model="marvin", router_zid="router")
            writer.append_arm_target(_arm_target())
            writer.abort()
            with self.assertRaises(IncompleteSessionError):
                SessionH5Reader(path)
            reader = SessionH5Reader(path, allow_incomplete=True)
            self.assertEqual(len(reader.read_arm_target("right")), 1)
            reader.close()

    def test_external_and_soft_links_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "session.h5"
            with h5py.File(path, "w") as file:
                file.attrs.update(schema_name="tianji-teleop-session", schema_version="1.0", source_type="x", robot_model="x", router_zid="r", complete=True)
                file["unsafe"] = h5py.SoftLink("/missing")
            with self.assertRaises(UnsafeSessionLinkError):
                SessionH5Reader(path)


if __name__ == "__main__":
    unittest.main()
