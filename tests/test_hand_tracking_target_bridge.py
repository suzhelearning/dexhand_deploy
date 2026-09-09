from __future__ import annotations

import unittest

import numpy as np

from tianji_teleop.hand_tracking.hand_target_adapter import create_hand_target_adapter
from tianji_teleop.hand_tracking.target_bridge import (
    ObservationTargetBridge,
    TargetBridgeInputRejected,
)
from tianji_teleop.protocol.messages import ArmInputObservation, ArmInputObservation as ArmWire
from tianji_teleop.protocol.messages import HandSkeletonObservation
from tianji_teleop.sources.common.pose_mapping import create_arm_pose_mapper
from tianji_teleop.sources.common.target_processing import create_arm_target_processor


def _hand(sequence: int = 1, *, timestamp_ns: int = 1_000_000_000, valid: bool = True, publisher: str = "obs") -> HandSkeletonObservation:
    points = np.zeros((21, 3), dtype=np.float64)
    points[4] = [0.1, 0.2, 0.3]
    return HandSkeletonObservation(
        1, sequence, timestamp_ns, timestamp_ns, timestamp_ns,
        "pico", "right", "pico-source", sequence, "receiver", sequence,
        "pico_tracking_initial_wrist_relative", "pico-v1", points.tolist(),
        [True] * 21 if valid else [False] * 21, valid, None, f"frame-{sequence}",
        publisher, "router",
    )


def _arm(sequence: int = 1, *, timestamp_ns: int = 1_000_000_000, valid: bool = True, publisher: str = "obs", source: str = "pico", reference: str = "pico_head_current", tracked: str = "wrist", pose: list[float] | None = None) -> ArmWire:
    return ArmWire(
        1, sequence, timestamp_ns, timestamp_ns, timestamp_ns, source, "right",
        tracked, reference, "pico-source", sequence, "receiver", sequence,
        "pico-arm-v1", pose if valid else None, valid, f"frame-{sequence}",
        publisher, "router",
    )


def _bridge(*, mapper_name: str = "relative_home", processor_name: str = "passthrough", profile: str = "pico") -> ObservationTargetBridge:
    hand = create_hand_target_adapter(
        "fixed_rotation",
        {
            "source": "pico",
            "coordinate_frame": "pico_tracking_initial_wrist_relative",
            "max_age_s": 0.5,
            "rotation": {"left": np.eye(3).tolist(), "right": np.eye(3).tolist()},
        },
    )
    if mapper_name == "relative_home":
        mapper_config = {
            "home_pose": {"left": [0, 0, 0, 0, 0, 0, 1], "right": [1, 2, 3, 0, 0, 0, 1]},
            "input_to_base_rotation": {"left": np.eye(3).tolist(), "right": np.eye(3).tolist()},
            "expected_reference_frame": "pico_head_current",
            "expected_tracked_frame": "wrist",
        }
    else:
        mapper_config = {
            "input_to_base_rotation": {"left": np.eye(3).tolist(), "right": np.eye(3).tolist()},
            "input_origin_in_base_m": {"left": [0, 0, 0], "right": [10, 20, 30]},
            "expected_reference_frame": "legacy_pico_tracking",
            "expected_tracked_frame": "palm",
        }
    return ObservationTargetBridge(
        input_profile=profile,
        router_zid="router",
        observation_publisher_instance_id="obs",
        hand_adapter=hand,
        arm_input_source="pico" if profile == "pico" else "legacy_pico_palm",
        pose_mapper=create_arm_pose_mapper(mapper_name, mapper_config),
        target_processor=create_arm_target_processor(processor_name, {}),
        active_sides=("right",),
        active_hand_sides=("right",),
        elbow_reference_direction={"left": [0, -1, 0], "right": [0, 1, 0]},
        arm_input_max_age_s=0.5,
    )


class ObservationTargetBridgeTest(unittest.TestCase):
    def test_no_target_is_ready_before_explicit_teleop_start(self) -> None:
        bridge = _bridge()
        bridge.ingest_hand_observation(_hand())
        bridge.ingest_arm_observation(_arm(pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]))

        self.assertEqual(bridge.tick(now_ns=1_100_000_000).hand, ())
        self.assertEqual(bridge.tick(now_ns=1_100_000_000).arm, ())

    def test_pico_relative_home_emits_both_targets_and_preserves_association(self) -> None:
        bridge = _bridge()
        bridge.ingest_hand_observation(_hand())
        bridge.ingest_arm_observation(_arm(pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]))
        bridge.start(now_ns=1_000_000_000)
        bridge.ingest_hand_observation(_hand(2, timestamp_ns=1_100_000_000))
        bridge.ingest_arm_observation(_arm(2, timestamp_ns=1_100_000_000, pose=[0.1, -0.2, 0.3, 0.0, 0.0, 0.0, 1.0]))

        result = bridge.tick(now_ns=1_100_000_000)

        self.assertEqual(len(result.hand), 1)
        self.assertEqual(len(result.arm), 1)
        np.testing.assert_allclose(result.arm[0].pose[:3], [1.1, 1.8, 3.3])
        np.testing.assert_allclose(result.hand[0].keypoints_m[4], [0.1, 0.2, 0.3])
        self.assertEqual(result.arm[0].frame_association_id, "frame-2")
        self.assertEqual(result.hand[0].frame_association_id, "frame-2")
        self.assertEqual(result.arm[0].elbow_reference_direction, (0.0, 1.0, 0.0))

    def test_manus_direct_pose_does_not_require_reference_initialization(self) -> None:
        bridge = _bridge(mapper_name="direct_pose", profile="manus")
        bridge.ingest_hand_observation(_hand())
        bridge.ingest_arm_observation(
            _arm(
                source="legacy_pico_palm",
                reference="legacy_pico_tracking",
                tracked="palm",
                pose=[0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
            )
        )
        bridge.start(now_ns=1_000_000_000)
        result = bridge.tick(now_ns=1_100_000_000)

        np.testing.assert_allclose(result.arm[0].pose[:3], [10.1, 20.2, 30.3])

    def test_invalid_stale_and_sequence_rollback_fail_closed(self) -> None:
        bridge = _bridge()
        bridge.ingest_hand_observation(_hand())
        bridge.ingest_arm_observation(_arm(pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]))
        bridge.start(now_ns=1_000_000_000)
        with self.assertRaises(TargetBridgeInputRejected):
            bridge.ingest_hand_observation(_hand(2, valid=False, timestamp_ns=1_100_000_000))
        bridge.ingest_arm_observation(_arm(2, timestamp_ns=1_100_000_000, pose=[0, 0, 0, 0, 0, 0, 1]))
        with self.assertRaises(TargetBridgeInputRejected):
            bridge.ingest_arm_observation(_arm(1, timestamp_ns=1_200_000_000, pose=[0, 0, 0, 0, 0, 0, 1]))
        bridge.ingest_hand_observation(_hand(3, timestamp_ns=1_100_000_000))
        with self.assertRaises(TargetBridgeInputRejected):
            bridge.tick(now_ns=1_600_000_001)


if __name__ == "__main__":
    unittest.main()
