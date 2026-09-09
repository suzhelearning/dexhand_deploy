from __future__ import annotations

import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from tianji_teleop.hand_tracking.models import (
    ArmInputObservation,
    HandObservation,
    PICO_HEAD_CURRENT_FRAME,
)


class HandTrackingModelTest(unittest.TestCase):
    def test_hand_observation_to_dict_preserves_masks_and_metadata(self) -> None:
        points = np.arange(63, dtype=np.float64).reshape(21, 3)
        observation = HandObservation(
            source="pico",
            side="right",
            source_instance_id="pico-device",
            source_sequence=12,
            source_timestamp_ns=123,
            received_timestamp_ns=456,
            receiver_instance_id="receiver",
            receiver_frame_sequence=7,
            coordinate_frame="pico_tracking_initial",
            mapping_version="pico26_to_mediapipe21_v1",
            keypoints_m=points,
            joint_valid=np.array([True] * 20 + [False]),
            valid=False,
            wrist_pose=None,
            frame_association_id="receiver:1:7",
        )

        payload = observation.to_dict()

        self.assertEqual(payload["side"], "right")
        self.assertEqual(payload["coordinate_frame"], "pico_tracking_initial")
        self.assertEqual(payload["keypoints_m"][20], [60.0, 61.0, 62.0])
        self.assertEqual(payload["joint_valid"][-1], False)
        self.assertIsNone(payload["wrist_pose"])

    def test_arm_input_observation_rejects_invalid_pose_and_keeps_reference(self) -> None:
        observation = ArmInputObservation(
            source="pico",
            side="left",
            tracked_frame="wrist",
            reference_frame=PICO_HEAD_CURRENT_FRAME,
            pose=np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]),
            valid=True,
            source_timestamp_ns=100,
            received_timestamp_ns=200,
            receiver_instance_id="receiver",
            receiver_frame_sequence=3,
            mapping_version="pico_head_current_v1",
            frame_association_id="receiver:1:3",
        )

        self.assertEqual(observation.to_dict()["reference_frame"], PICO_HEAD_CURRENT_FRAME)
        with self.assertRaises(ValueError):
            ArmInputObservation(
                source="pico",
                side="left",
                tracked_frame="wrist",
                reference_frame=PICO_HEAD_CURRENT_FRAME,
                pose=np.zeros(7),
                valid=True,
                source_timestamp_ns=None,
                received_timestamp_ns=200,
                receiver_instance_id="receiver",
                receiver_frame_sequence=3,
                mapping_version="x",
                frame_association_id="receiver:1:3",
            )

    def test_common_rigid_transform_does_not_change_current_head_pose(self) -> None:
        from tianji_teleop.hand_tracking.pico import tracking_pose_to_current_head

        head = np.array([0.4, -0.2, 1.2, *Rotation.from_euler("xyz", [0.1, -0.2, 0.3]).as_quat()])
        wrist = np.array([0.7, 0.1, 1.0, *Rotation.from_euler("xyz", [-0.2, 0.3, 0.4]).as_quat()])
        common_rotation = Rotation.from_euler("zyx", [0.5, -0.1, 0.2])
        common_translation = np.array([-1.0, 0.4, 0.8])

        def transform(pose: np.ndarray) -> np.ndarray:
            return np.concatenate((
                common_rotation.apply(pose[:3]) + common_translation,
                (common_rotation * Rotation.from_quat(pose[3:])).as_quat(),
            ))

        expected = tracking_pose_to_current_head(head, wrist)
        transformed = tracking_pose_to_current_head(transform(head), transform(wrist))

        np.testing.assert_allclose(transformed, expected, atol=1.0e-12)


if __name__ == "__main__":
    unittest.main()
