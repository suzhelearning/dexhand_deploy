from __future__ import annotations

import json
import struct
import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np

from tianji_teleop.hand_tracking.manus import MEDIAPIPE_SEMANTIC_ORDER, parse_manus_payload, manus_to_mediapipe
from tianji_teleop.hand_tracking.models import ArmInputObservation
from tianji_teleop.hand_tracking.pico import parse_pico_packet
from tianji_teleop.recording.session_h5 import EXTENDED_SCHEMA_VERSION, SessionH5Error, SessionH5Reader, SessionH5Writer


def _pico_pose(index: int) -> list[float]:
    return [float(index), float(index) + 0.1, float(index) + 0.2, 0.0, 0.0, 0.0, 1.0]


def _pico_packet() -> bytes:
    payload = bytearray(struct.pack("<BBBB", 1, 0x07, 26, 0))
    payload.extend(struct.pack("<7f", *_pico_pose(100)))
    for side_offset in (0, 1000):
        payload.extend(struct.pack("<BBBB", 1, 0, 0, 0))
        payload.extend(struct.pack("<7f", *_pico_pose(200 + side_offset)))
        for joint in range(26):
            payload.extend(struct.pack("<BBBB", 1, 0, 0, 0))
            payload.extend(struct.pack("<7f", *_pico_pose(side_offset + joint)))
            payload.extend(struct.pack("<f", 0.01 * (joint + 1)))
    return struct.pack("<BBqI", 0xAB, 0x40, 1234, len(payload)) + payload


def _manus_payload() -> dict:
    semantics = list(MEDIAPIPE_SEMANTIC_ORDER)
    nodes = []
    positions = []
    for index, (chain, joint) in enumerate(semantics):
        nodes.append({
            "array_index": index,
            "node_id": 100 + index,
            "parent_id": 99 + index,
            "chain_type": 13 if chain == "hand" else {"thumb": 5, "index": 6, "middle": 7, "ring": 8, "pinky": 9}[chain],
            "side": 0,
            "finger_joint_type": 0 if chain == "hand" else {"mcp": 1, "pip": 2, "ip": 3, "dip": 4, "tip": 5}[joint],
        })
        positions.append([float(index), float(index + 1), float(index + 2)])
    return {
        "glove_id": "manus-1",
        "side": "right",
        "seq": 4,
        "source_monotonic_ns": 500,
        "sdk_publish_time": 6,
        "nodes": positions,
        "node_quaternions_wxyz": [[1.0, 0.0, 0.0, 0.0] for _ in positions],
        "node_semantics": nodes,
    }


class SessionH5HandTrackingTest(unittest.TestCase):
    def test_schema_11_metadata_rejects_non_finite_json_values(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid_metadata.h5"
            with self.assertRaises(SessionH5Error):
                SessionH5Writer(
                    path,
                    source_type="hand_tracking_observation",
                    robot_model="marvin",
                    router_zid="router",
                    schema_version=EXTENDED_SCHEMA_VERSION,
                    metadata={"invalid": float("nan")},
                )
            self.assertFalse(path.exists())

    def test_schema_11_round_trips_complete_raw_and_derived_inputs(self) -> None:
        pico = parse_pico_packet(
            _pico_packet(),
            receiver_instance_id="pico-receiver",
            connection_generation=3,
            receiver_frame_sequence=9,
            received_timestamp_ns=1000,
        )
        manus = parse_manus_payload(
            _manus_payload(),
            receiver_instance_id="manus-receiver",
            receiver_frame_sequence=12,
            received_timestamp_ns=1100,
        )
        hand = manus_to_mediapipe(manus)
        arm = ArmInputObservation(
            source="pico",
            side="right",
            tracked_frame="wrist",
            reference_frame="pico_head_current",
            pose=np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]),
            valid=True,
            source_timestamp_ns=pico.source_timestamp_ns,
            received_timestamp_ns=1000,
            receiver_instance_id="pico-receiver",
            receiver_frame_sequence=9,
            mapping_version="pico_head_current_v1",
            frame_association_id=pico.association_id,
            elbow_pose=np.array([0.4, 0.5, 0.6, 0.0, 0.0, 0.0, 1.0]),
        )

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "hand_tracking.h5"
            with SessionH5Writer(
                path,
                source_type="hand_tracking_observation",
                robot_model="marvin",
                router_zid="router",
                schema_version=EXTENDED_SCHEMA_VERSION,
                metadata={
                    "input_profile": "pico",
                    "receiver_instance_id": "pico-receiver",
                    "mapping_versions": {"hand": "pico26_to_mediapipe21_v1"},
                },
            ) as writer:
                writer.append_raw_pico(pico)
                writer.append_raw_manus(manus)
                writer.append_hand_observation(hand)
                writer.append_arm_input_observation(arm)

            with h5py.File(path, "r") as file:
                self.assertEqual(file.attrs["schema_version"], EXTENDED_SCHEMA_VERSION)
                self.assertEqual(
                    set(file.keys()), {"raw", "target", "joint", "meta", "observation"}
                )
                self.assertEqual(
                    set(file["raw"].keys()),
                    {"mocap_live", "h5_replay", "pico_hand_tracking", "manus_hand_tracking", "legacy_pico_palm"},
                )
                self.assertEqual(bytes(file["raw/pico_hand_tracking/raw_packet"][0]), pico.raw_packet)
                self.assertEqual(file["raw/pico_hand_tracking/hands/right/joint_poses"].shape, (1, 26, 7))
                self.assertEqual(file["raw/manus_hand_tracking/node_positions"].shape, (1, 64, 3))
                self.assertEqual(file["observation/hand_tracking/right/keypoints_m"].shape, (1, 21, 3))
                self.assertEqual(file["observation/arm_input/right/elbow_pose"].shape, (1, 7))
                metadata = json.loads(file["meta/hand_tracking"].attrs["metadata_json"])
                self.assertEqual(metadata["input_profile"], "pico")

            with SessionH5Reader(path) as reader:
                pico_row = reader.read_raw_pico()[0]
                manus_row = reader.read_raw_manus()[0]
                hand_row = reader.read_hand_observation("right")[0]
                arm_row = reader.read_arm_input_observation("right")[0]
                metadata = reader.read_hand_tracking_metadata()

            self.assertEqual(pico_row["raw_packet"], pico.raw_packet)
            self.assertEqual(pico_row["hands"]["left"]["joint_valid"], [True] * 26)
            self.assertEqual(manus_row["glove_id"], "manus-1")
            self.assertEqual(manus_row["node_count"], 21)
            np.testing.assert_allclose(hand_row["keypoints_m"], hand.keypoints_m)
            self.assertEqual(hand_row["source_sequence"], hand.source_sequence)
            np.testing.assert_allclose(arm_row["pose"], arm.pose)
            np.testing.assert_allclose(arm_row["elbow_pose"], arm.elbow_pose)
            self.assertEqual(arm_row["reference_frame"], "pico_head_current")
            self.assertEqual(metadata["receiver_instance_id"], "pico-receiver")


if __name__ == "__main__":
    unittest.main()
