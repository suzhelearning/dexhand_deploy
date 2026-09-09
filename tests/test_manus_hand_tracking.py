from __future__ import annotations

import unittest

import numpy as np

from tianji_teleop.hand_tracking.manus import (
    MEDIAPIPE_SEMANTIC_ORDER,
    ManusMappingError,
    manus_to_mediapipe,
    parse_manus_payload,
)


def _payload() -> dict:
    nodes = []
    positions = []
    index = 0
    # The source array is deliberately not in MediaPipe order.
    semantics = list(MEDIAPIPE_SEMANTIC_ORDER)
    semantics = semantics[::2] + semantics[1::2]
    for chain, joint in semantics:
        if chain == "hand":
            chain_type, joint_type = 13, 0
        else:
            chain_type = {"thumb": 5, "index": 6, "middle": 7, "ring": 8, "pinky": 9}[chain]
            joint_type = {"mcp": 1, "pip": 2, "ip": 3, "dip": 4, "tip": 5}[joint]
        nodes.append({
            "array_index": index,
            "node_id": 100 + index,
            "parent_id": 99 + index,
            "chain_type": chain_type,
            "side": 0,
            "finger_joint_type": joint_type,
        })
        positions.append([float(index + 1), float(index + 2), float(index + 3)])
        index += 1
    return {
        "glove_id": "glove-1",
        "side": "left_hand",
        "seq": 4,
        "source_monotonic_ns": 500,
        "sdk_publish_time": 6,
        "nodes": positions,
        "node_quaternions_wxyz": [[1.0, 0.0, 0.0, 0.0] for _ in positions],
        "node_semantics": nodes,
    }


class ManusHandTrackingTest(unittest.TestCase):
    def test_semantic_mapping_ignores_source_array_order_and_flips_y_once(self) -> None:
        raw = parse_manus_payload(
            _payload(),
            receiver_instance_id="receiver",
            receiver_frame_sequence=8,
            received_timestamp_ns=1000,
        )

        mapped = manus_to_mediapipe(raw)

        self.assertEqual(mapped.side, "left")
        self.assertEqual(mapped.source, "manus")
        self.assertEqual(mapped.keypoints_m.shape, (21, 3))
        self.assertEqual(mapped.keypoints_m[0].tolist(), [0.0, 0.0, 0.0])
        self.assertEqual(mapped.coordinate_frame, "manus_local_vuh_y_flipped_wrist_relative")
        self.assertTrue(bool(mapped.joint_valid.all()))

    def test_missing_and_duplicate_required_semantics_are_rejected(self) -> None:
        payload = _payload()
        payload["node_semantics"] = payload["node_semantics"][:-1]
        raw = parse_manus_payload(
            payload,
            receiver_instance_id="receiver",
            receiver_frame_sequence=1,
            received_timestamp_ns=1,
        )
        with self.assertRaises(ManusMappingError):
            manus_to_mediapipe(raw)

        duplicate_payload = _payload()
        duplicate_payload["node_semantics"].append(dict(duplicate_payload["node_semantics"][0]))
        duplicate_payload["nodes"].append([99.0, 100.0, 101.0])
        duplicate_payload["node_quaternions_wxyz"].append([1.0, 0.0, 0.0, 0.0])
        duplicate_payload["node_semantics"][-1]["array_index"] = 21
        duplicate_raw = parse_manus_payload(
            duplicate_payload,
            receiver_instance_id="receiver",
            receiver_frame_sequence=1,
            received_timestamp_ns=1,
        )
        with self.assertRaises(ManusMappingError):
            manus_to_mediapipe(duplicate_raw)

    def test_payload_requires_complete_positions_and_metadata(self) -> None:
        payload = _payload()
        payload["nodes"][0] = [1.0, 2.0]
        with self.assertRaises(ValueError):
            parse_manus_payload(
                payload,
                receiver_instance_id="receiver",
                receiver_frame_sequence=1,
                received_timestamp_ns=1,
            )

        bad = _payload()
        bad["node_quaternions_wxyz"][0] = [float("nan"), 0.0, 0.0, 0.0]
        with self.assertRaises(ValueError):
            parse_manus_payload(
                bad,
                receiver_instance_id="receiver",
                receiver_frame_sequence=1,
                received_timestamp_ns=1,
            )

    def test_payload_rejects_non_object_semantics_and_invalid_identity(self) -> None:
        payload = _payload()
        payload["node_semantics"][0] = "not-an-object"
        with self.assertRaises(ValueError):
            parse_manus_payload(
                payload,
                receiver_instance_id="receiver",
                receiver_frame_sequence=1,
                received_timestamp_ns=1,
            )

        payload = _payload()
        payload["glove_id"] = None
        with self.assertRaises(ValueError):
            parse_manus_payload(
                payload,
                receiver_instance_id="receiver",
                receiver_frame_sequence=1,
                received_timestamp_ns=1,
            )


if __name__ == "__main__":
    unittest.main()
