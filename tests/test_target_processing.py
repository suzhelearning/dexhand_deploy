from __future__ import annotations

import unittest

import numpy as np

from tianji_teleop.sources.common.pose_mapping import MappedArmPose
from tianji_teleop.sources.common.target_processing import (
    create_arm_target_processor,
)


def _mapped(pose=None) -> MappedArmPose:
    return MappedArmPose(
        side="right",
        pose=np.asarray(pose or [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0], dtype=np.float64),
        valid=True,
        backend="direct_pose",
        mapping_version="direct-v1",
        frame_association_id="frame-1",
    )


class TargetProcessingFactoryTest(unittest.TestCase):
    def test_passthrough_preserves_geometry_exactly(self) -> None:
        processor = create_arm_target_processor("passthrough", {})
        source = _mapped([0.123, -0.2, 0.3, 0.0, 0.0, 0.1, 0.995])

        result = processor.process(source, dt_s=0.01)

        np.testing.assert_array_equal(result.pose, source.pose)
        self.assertEqual(result.backend, "passthrough")

    def test_conditioned_processor_reset_does_not_reuse_old_state(self) -> None:
        processor = create_arm_target_processor(
            "conditioned",
            {
                "rate_hz": 100.0,
                "translation_gain": [1.0, 1.0, 1.0],
                "rotation_gain": 1.0,
                "workspace_relative_radii_m": [10.0, 10.0, 10.0],
                "workspace_soft_zone_ratio": 0.9,
                "maximum_linear_speed_m_s": 0.1,
                "maximum_angular_speed_rad_s": 1.0,
                "maximum_linear_acceleration_m_s2": 10.0,
                "maximum_angular_acceleration_rad_s2": 10.0,
                "initial_position": {"right": [0.0, 0.0, 0.0]},
                "initial_quaternion": {"right": [0.0, 0.0, 0.0, 1.0]},
            },
        )
        first = processor.process(_mapped([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]), dt_s=0.01)
        processor.reset()
        after_reset = processor.process(_mapped([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]), dt_s=0.01)

        self.assertTrue(first.valid)
        np.testing.assert_allclose(after_reset.pose[:3], [0.0, 0.0, 0.0])

    def test_factory_rejects_unknown_backend(self) -> None:
        with self.assertRaises(ValueError):
            create_arm_target_processor("does_not_exist", {})


if __name__ == "__main__":
    unittest.main()
