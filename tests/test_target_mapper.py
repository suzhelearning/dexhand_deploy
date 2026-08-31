from __future__ import annotations

import unittest

import numpy as np

from tianji_teleop.sources.common.wrist_pose_frame import WristPoseFrame
from tianji_teleop.sources.common.target_mapper import (
    EndEffectorTargetMapper,
)
from tianji_teleop.sources.common.target_conditioner import (
    TargetConditioner,
    TargetConditioningSettings,
)
from tianji_world_output.config_loader import TianjiConfig


class TargetMapperTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = TianjiConfig.load()
        self.mapper = EndEffectorTargetMapper(
            self.config,
            rate=90.0,
        )
        self.initial_frame = WristPoseFrame.from_poses(
            [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
            [-0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
        )

    def test_initial_frame_maps_to_robot_safe_initial_poses(self) -> None:
        initialized = self.mapper.initialize(self.initial_frame)
        targets = self.mapper.map_relative_wrist_frame(self.initial_frame)

        self.assertEqual(
            initialized,
            {"left_wrist", "right_wrist"},
        )
        np.testing.assert_allclose(
            targets.left_pose[:3],
            self.config.init_pos["left"],
            atol=1.0e-10,
        )
        np.testing.assert_allclose(
            targets.right_pose[:3],
            self.config.init_pos["right"],
            atol=1.0e-10,
        )
        self.assertAlmostEqual(
            np.linalg.norm(targets.left_pose[3:]),
            1.0,
        )
        self.assertAlmostEqual(
            np.linalg.norm(targets.right_pose[3:]),
            1.0,
        )

    def test_hand_motion_changes_target_without_body(self) -> None:
        self.mapper.initialize(self.initial_frame)
        initial_targets = self.mapper.map_relative_wrist_frame(self.initial_frame)
        moved_frame = WristPoseFrame.from_poses(
            [0.12, 0.2, 0.25, 0.0, 0.0, 0.0, 1.0],
            [-0.08, 0.22, 0.3, 0.0, 0.0, 0.0, 1.0],
        )
        moved_targets = self.mapper.map_relative_wrist_frame(moved_frame)

        self.assertFalse(
            np.array_equal(
                initial_targets.left_pose,
                moved_targets.left_pose,
            )
        )
        self.assertFalse(
            np.array_equal(
                initial_targets.right_pose,
                moved_targets.right_pose,
            )
        )

    def test_target_conditioner_soft_limits_workspace_and_speed(self) -> None:
        settings = TargetConditioningSettings(
            rate_hz=100.0,
            translation_gain=np.ones(3),
            rotation_gain=1.0,
            workspace_relative_radii_m=np.array([0.2, 0.2, 0.2]),
            workspace_soft_zone_ratio=0.8,
            maximum_linear_speed_m_s=0.1,
            maximum_angular_speed_rad_s=0.5,
            maximum_linear_acceleration_m_s2=10.0,
            maximum_angular_acceleration_rad_s2=50.0,
        )
        conditioner = TargetConditioner(
            np.zeros(3), [0.0, 0.0, 0.0, 1.0], settings
        )
        position, quaternion, diagnostics = conditioner.condition(
            [2.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]
        )

        self.assertLessEqual(np.linalg.norm(position), 0.001000001)
        self.assertAlmostEqual(np.linalg.norm(quaternion), 1.0)
        self.assertTrue(diagnostics.workspace_soft_limited)
        self.assertTrue(diagnostics.linear_speed_limited)
        self.assertTrue(diagnostics.angular_speed_limited)

        conditioner.synchronize([0.05, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
        held_position, _, held = conditioner.condition(
            [0.05, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
        )
        np.testing.assert_allclose(held_position, [0.05, 0.0, 0.0])
        self.assertEqual(held.applied_linear_speed_m_s, 0.0)

    def test_explicit_home_zsp_overrides_legacy_default(self) -> None:
        mapper = EndEffectorTargetMapper(
            self.config,
            default_zsp_directions={
                "left": [1.0, 2.0, 3.0],
                "right": [-1.0, 2.0, 3.0],
            },
        )
        mapper.initialize(self.initial_frame)
        targets = mapper.map_relative_wrist_frame(self.initial_frame)

        np.testing.assert_allclose(
            targets.left_default_elbow_direction,
            np.array([1.0, 2.0, 3.0]) / np.sqrt(14.0),
        )
        np.testing.assert_allclose(
            targets.right_default_elbow_direction,
            np.array([-1.0, 2.0, 3.0]) / np.sqrt(14.0),
        )

if __name__ == "__main__":
    unittest.main()
