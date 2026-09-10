from __future__ import annotations

import json
import unittest

from tianji_teleop.protocol import topics
from tianji_teleop.protocol.messages import (
    ArmInputObservation,
    HandSkeletonObservation,
    ProtocolError,
    parse_message,
)


def _hand() -> HandSkeletonObservation:
    return HandSkeletonObservation(
        schema_version=1,
        sequence=1,
        timestamp_ns=2,
        source_timestamp_ns=3,
        received_timestamp_ns=4,
        source="pico",
        side="right",
        source_instance_id="device",
        source_sequence=5,
        receiver_instance_id="receiver",
        receiver_frame_sequence=6,
        coordinate_frame="pico_tracking_initial",
        mapping_version="pico26_to_mediapipe21_v1",
        keypoints_m=[[float(i), 0.0, 0.0] for i in range(21)],
        joint_valid=[True] * 21,
        valid=True,
        wrist_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        frame_association_id="receiver:1:6",
        publisher_instance_id="publisher",
        router_zid="router",
    )


class HandTrackingProtocolTest(unittest.TestCase):
    def test_observation_topics_are_separate_from_control_topics(self) -> None:
        self.assertEqual(topics.hand_observation("left"), "tianji/observation/hand/left")
        self.assertEqual(topics.arm_input_observation("right"), "tianji/observation/arm_input/right")
        self.assertEqual(topics.RAW_PICO_HAND_TRACKING, "tianji/raw/pico_hand_tracking")
        self.assertNotEqual(topics.hand_observation("right"), topics.hand_target("right"))
        self.assertNotEqual(topics.arm_input_observation("right"), topics.arm_target("right"))

    def test_hand_observation_round_trips_strictly(self) -> None:
        message = _hand()
        decoded = parse_message(json.dumps(message.to_dict()), HandSkeletonObservation)
        self.assertEqual(decoded, message)

        payload = message.to_dict()
        payload["unexpected"] = True
        with self.assertRaises(ProtocolError):
            HandSkeletonObservation.from_dict(payload)

    def test_invalid_hand_mask_cannot_claim_valid(self) -> None:
        payload = _hand().to_dict()
        payload["valid"] = True
        payload["joint_valid"][-1] = False
        with self.assertRaises(ProtocolError):
            HandSkeletonObservation.from_dict(payload)

    def test_arm_observation_keeps_independent_validity(self) -> None:
        message = ArmInputObservation(
            schema_version=1,
            sequence=7,
            timestamp_ns=8,
            source_timestamp_ns=None,
            received_timestamp_ns=9,
            source="manus",
            side="left",
            tracked_frame="palm",
            reference_frame="legacy_pico_tracking",
            source_instance_id="palm-source",
            source_sequence=None,
            receiver_instance_id="receiver",
            receiver_frame_sequence=10,
            mapping_version="legacy_palm_v1",
            pose=None,
            valid=False,
            frame_association_id="receiver:10",
            publisher_instance_id="publisher",
            router_zid="router",
        )
        self.assertFalse(parse_message(json.dumps(message.to_dict()), ArmInputObservation).valid)

        valid_message = ArmInputObservation(
            schema_version=1, sequence=8, timestamp_ns=9, source_timestamp_ns=10,
            received_timestamp_ns=11, source="xr", side="left", tracked_frame="wrist_tracker",
            reference_frame="xr_tracking", source_instance_id="xr-source", source_sequence=12,
            receiver_instance_id="receiver", receiver_frame_sequence=13,
            mapping_version="xr_tracker_v1", pose=[0, 0, 0, 0, 0, 0, 1], valid=True,
            frame_association_id="receiver:1:13", publisher_instance_id="publisher",
            router_zid="router", elbow_pose=[0.1, 0.2, 0.3, 0, 0, 0, 1],
        )
        decoded = parse_message(json.dumps(valid_message.to_dict()), ArmInputObservation)
        self.assertEqual(decoded, valid_message)
        legacy_payload = valid_message.to_dict()
        legacy_payload.pop("elbow_pose")
        legacy_decoded = ArmInputObservation.from_dict(legacy_payload)
        self.assertIsNone(legacy_decoded.elbow_pose)

        payload = message.to_dict()
        payload["pose"] = [0.0] * 7
        with self.assertRaises(ProtocolError):
            ArmInputObservation.from_dict(payload)


if __name__ == "__main__":
    unittest.main()
