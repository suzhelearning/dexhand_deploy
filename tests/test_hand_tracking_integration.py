from __future__ import annotations

import struct
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from tianji_teleop.hand_tracking.manus import MEDIAPIPE_SEMANTIC_ORDER, parse_manus_payload
from tianji_teleop.hand_tracking.pico import tracking_pose_to_current_head, parse_pico_packet
from tianji_teleop.hand_tracking.runtime import ObservationRuntime
from tianji_teleop.protocol import topics


def _pose(index: int) -> list[float]:
    return [float(index), float(index) + 0.1, float(index) + 0.2, 0.0, 0.0, 0.0, 1.0]


def _pico_packet() -> bytes:
    payload = bytearray(struct.pack("<BBBB", 1, 0x07, 26, 0))
    payload.extend(struct.pack("<7f", *_pose(100)))
    for side_offset in (0, 1000):
        payload.extend(struct.pack("<BBBB", 1, 0, 0, 0))
        payload.extend(struct.pack("<7f", *_pose(200 + side_offset)))
        for joint in range(26):
            payload.extend(struct.pack("<BBBB", 1, 0, 0, 0))
            payload.extend(struct.pack("<7f", *_pose(side_offset + joint)))
            payload.extend(struct.pack("<f", 0.01 * (joint + 1)))
    return struct.pack("<BBqI", 0xAB, 0x40, 1234, len(payload)) + payload


def _manus_payload() -> dict:
    nodes = []
    positions = []
    for index, (chain, joint) in enumerate(MEDIAPIPE_SEMANTIC_ORDER):
        chain_type = 13 if chain == "hand" else {"thumb": 5, "index": 6, "middle": 7, "ring": 8, "pinky": 9}[chain]
        finger_joint_type = 0 if chain == "hand" else {"mcp": 1, "pip": 2, "ip": 3, "dip": 4, "tip": 5}[joint]
        nodes.append({
            "array_index": index,
            "node_id": 100 + index,
            "parent_id": 99 + index,
            "chain_type": chain_type,
            "side": 0,
            "finger_joint_type": finger_joint_type,
        })
        positions.append([float(index), float(index + 1), float(index + 2)])
    return {
        "glove_id": "manus-integration",
        "side": "left",
        "seq": 12,
        "source_monotonic_ns": 500,
        "sdk_publish_time": 6,
        "nodes": positions,
        "node_quaternions_wxyz": [[1.0, 0.0, 0.0, 0.0] for _ in positions],
        "node_semantics": nodes,
    }


class HandTrackingIntegrationTest(unittest.TestCase):
    def test_pico_profile_associates_raw_hand_and_arm_observations_without_control_topics(self) -> None:
        published: list[tuple[str, dict]] = []
        runtime = ObservationRuntime(
            publish=lambda key, payload: published.append((key, payload)),
            publisher_instance_id="integration-publisher",
            router_zid="router",
        )
        frame = parse_pico_packet(
            _pico_packet(),
            receiver_instance_id="pico-receiver",
            connection_generation=2,
            receiver_frame_sequence=7,
            received_timestamp_ns=100,
        )

        runtime.ingest_pico(frame)

        payload_by_key = {key: payload for key, payload in published}
        self.assertIn(topics.RAW_PICO_HAND_TRACKING, payload_by_key)
        self.assertNotIn(topics.RAW_MANUS_HAND_TRACKING, payload_by_key)
        for side in ("left", "right"):
            self.assertEqual(
                payload_by_key[topics.hand_observation(side)]["frame_association_id"],
                frame.association_id,
            )
            self.assertEqual(
                payload_by_key[topics.arm_input_observation(side)]["frame_association_id"],
                frame.association_id,
            )
        self.assertFalse(any(key.startswith("tianji/target/") or key.startswith("tianji/command/") for key, _ in published))

    def test_manus_profile_publishes_only_manus_hand_observation(self) -> None:
        published: list[tuple[str, dict]] = []
        runtime = ObservationRuntime(
            publish=lambda key, payload: published.append((key, payload)),
            publisher_instance_id="integration-publisher",
            router_zid="router",
        )
        frame = parse_manus_payload(
            _manus_payload(),
            receiver_instance_id="manus-receiver",
            receiver_frame_sequence=9,
            received_timestamp_ns=100,
        )

        runtime.ingest_manus_frame(frame)

        payload_by_key = {key: payload for key, payload in published}
        self.assertIn(topics.RAW_MANUS_HAND_TRACKING, payload_by_key)
        self.assertNotIn(topics.RAW_PICO_HAND_TRACKING, payload_by_key)
        observation = payload_by_key[topics.hand_observation("left")]
        self.assertEqual(observation["frame_association_id"], frame.association_id)
        self.assertEqual(observation["source"], "manus")
        self.assertFalse(any(key.startswith("tianji/target/") or key.startswith("tianji/command/") for key, _ in published))

    def test_pico_current_head_pose_is_invariant_to_a_common_tracking_transform(self) -> None:
        head_rotation = Rotation.from_euler("zy", [25.0, -10.0], degrees=True)
        wrist_rotation = head_rotation * Rotation.from_euler("x", 30.0, degrees=True)
        head = np.concatenate(([1.0, 2.0, 3.0], head_rotation.as_quat()))
        wrist = np.concatenate(([1.3, 2.4, 3.2], wrist_rotation.as_quat()))

        common_rotation = Rotation.from_euler("xyz", [12.0, -20.0, 35.0], degrees=True)
        common_translation = np.array([4.0, -2.0, 1.5])
        transformed_head = np.concatenate((common_translation + common_rotation.apply(head[:3]), (common_rotation * head_rotation).as_quat()))
        transformed_wrist = np.concatenate((common_translation + common_rotation.apply(wrist[:3]), (common_rotation * wrist_rotation).as_quat()))

        original_relative = tracking_pose_to_current_head(head, wrist)
        transformed_relative = tracking_pose_to_current_head(transformed_head, transformed_wrist)

        np.testing.assert_allclose(transformed_relative[:3], original_relative[:3], atol=1.0e-12)
        np.testing.assert_allclose(
            Rotation.from_quat(transformed_relative[3:]).as_matrix(),
            Rotation.from_quat(original_relative[3:]).as_matrix(),
            atol=1.0e-12,
        )


if __name__ == "__main__":
    unittest.main()
