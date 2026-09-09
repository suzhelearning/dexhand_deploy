from __future__ import annotations

import unittest

import numpy as np

from tianji_teleop.hand_tracking.hand_target_adapter import (
    HandObservationRejected,
    PreparedHandTarget,
    create_hand_target_adapter,
)
from tianji_teleop.protocol.messages import HandSkeletonObservation, ProtocolError


def _observation(
    *,
    side: str = "right",
    source: str = "pico",
    coordinate_frame: str = "pico_tracking_initial_wrist_relative",
    sequence: int = 1,
    timestamp_ns: int = 1_000_000_000,
    valid: bool = True,
    publisher_instance_id: str = "observation-instance",
    keypoints: np.ndarray | None = None,
) -> HandSkeletonObservation:
    points = np.zeros((21, 3), dtype=np.float64) if keypoints is None else np.asarray(keypoints, dtype=np.float64)
    return HandSkeletonObservation(
        1,
        sequence,
        timestamp_ns,
        timestamp_ns,
        timestamp_ns,
        source,
        side,
        "source-instance",
        sequence,
        "receiver-instance",
        sequence,
        coordinate_frame,
        "mapping-v1",
        points.tolist(),
        [True] * 21 if valid else [False] * 21,
        valid,
        None,
        f"frame-{sequence}",
        publisher_instance_id,
        "router",
    )


class HandTargetAdapterTest(unittest.TestCase):
    @staticmethod
    def _config() -> dict:
        return {
            "source": "pico",
            "coordinate_frame": "pico_tracking_initial_wrist_relative",
            "max_age_s": 0.5,
            "rotation": {
                "left": np.eye(3).tolist(),
                "right": [
                    [0.0, -1.0, 0.0],
                    [1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
            },
        }

    def test_factory_applies_one_configured_rotation_and_preserves_association(self) -> None:
        points = np.zeros((21, 3), dtype=np.float64)
        points[4] = [0.1, 0.2, 0.3]
        adapter = create_hand_target_adapter("fixed_rotation", self._config())

        result = adapter.adapt(_observation(keypoints=points), now_ns=1_100_000_000)

        self.assertIsInstance(result, PreparedHandTarget)
        np.testing.assert_allclose(result.keypoints_m[4], [-0.2, 0.1, 0.3])
        np.testing.assert_array_equal(result.keypoints_m[0], [0.0, 0.0, 0.0])
        self.assertEqual(result.side, "right")
        self.assertEqual(result.observation_sequence, 1)
        self.assertEqual(result.frame_association_id, "frame-1")
        self.assertEqual(result.source_timestamp_ns, 1_000_000_000)

    def test_invalid_or_stale_observation_never_becomes_target(self) -> None:
        adapter = create_hand_target_adapter("fixed_rotation", self._config())

        with self.assertRaises(HandObservationRejected):
            adapter.adapt(_observation(valid=False), now_ns=1_100_000_000)
        with self.assertRaises(HandObservationRejected):
            adapter.adapt(_observation(), now_ns=1_600_000_001)

    def test_source_frame_and_sequence_are_strict(self) -> None:
        adapter = create_hand_target_adapter("fixed_rotation", self._config())
        adapter.adapt(_observation(), now_ns=1_100_000_000)

        with self.assertRaises(HandObservationRejected):
            adapter.adapt(
                _observation(source="manus", sequence=2),
                now_ns=1_100_000_000,
            )
        with self.assertRaises(HandObservationRejected):
            adapter.adapt(
                _observation(coordinate_frame="other", sequence=2),
                now_ns=1_100_000_000,
            )
        with self.assertRaises(HandObservationRejected):
            adapter.adapt(_observation(sequence=1), now_ns=1_100_000_000)

    def test_unknown_adapter_and_reflection_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            create_hand_target_adapter("does-not-exist", self._config())
        config = self._config()
        config["rotation"] = {"left": [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]}
        with self.assertRaises((ValueError, ProtocolError)):
            create_hand_target_adapter("fixed_rotation", config)


if __name__ == "__main__":
    unittest.main()
