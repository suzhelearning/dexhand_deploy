from __future__ import annotations

import unittest

import numpy as np

from pico_body_tianji.controller_frame import ControllerFrame
from pico_body_tianji.controller_only.controller_only_mapper import (
    ControllerOnlyTeleopMapper,
)
from pico_body_tianji.controller_only.controller_only_source import (
    XRoboControllerOnlySource,
)
from pico_body_tianji.controller_only.target_conditioner import (
    ControllerTargetConditioner,
    TargetConditioningSettings,
)
from tianji_world_output.config_loader import TianjiConfig


class _ControllerOnlySdk:
    def __init__(self):
        self.opened = False
        self.body_api_calls = 0

    def init(self) -> None:
        self.opened = True

    def close(self) -> None:
        self.opened = False

    @staticmethod
    def get_left_controller_pose():
        return [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]

    @staticmethod
    def get_right_controller_pose():
        return [-0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]

    @staticmethod
    def get_time_stamp_ns() -> int:
        return 123

    @staticmethod
    def get_A_button() -> bool:
        return False

    def is_body_data_available(self) -> bool:
        self.body_api_calls += 1
        raise AssertionError("controller-only source accessed Body API")


class ControllerOnlyMapperTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = TianjiConfig.load()
        self.mapper = ControllerOnlyTeleopMapper(
            self.config,
            rate=90.0,
            min_cutoff=1.0,
            beta=0.7,
        )
        self.initial_frame = ControllerFrame.from_poses(
            [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
            [-0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
        )

    def test_initial_frame_maps_to_robot_safe_initial_poses(self) -> None:
        initialized = self.mapper.initialize(self.initial_frame)
        targets = self.mapper.map_frame(self.initial_frame)

        self.assertEqual(
            initialized,
            {"pico_left_wrist", "pico_right_wrist"},
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
        initial_targets = self.mapper.map_frame(self.initial_frame)
        moved_frame = ControllerFrame.from_poses(
            [0.12, 0.2, 0.25, 0.0, 0.0, 0.0, 1.0],
            [-0.08, 0.22, 0.3, 0.0, 0.0, 0.0, 1.0],
        )
        moved_targets = self.mapper.map_frame(moved_frame)

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
        conditioner = ControllerTargetConditioner(
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

    def test_explicit_home_zsp_overrides_legacy_default(self) -> None:
        mapper = ControllerOnlyTeleopMapper(
            self.config,
            default_zsp_directions={
                "left": [1.0, 2.0, 3.0],
                "right": [-1.0, 2.0, 3.0],
            },
        )
        mapper.initialize(self.initial_frame)
        targets = mapper.map_frame(self.initial_frame)

        np.testing.assert_allclose(
            targets.left_default_elbow_direction,
            np.array([1.0, 2.0, 3.0]) / np.sqrt(14.0),
        )
        np.testing.assert_allclose(
            targets.right_default_elbow_direction,
            np.array([-1.0, 2.0, 3.0]) / np.sqrt(14.0),
        )

    def test_source_can_skip_body_api_completely(self) -> None:
        sdk = _ControllerOnlySdk()
        source = XRoboControllerOnlySource(sdk=sdk)
        source.open()
        try:
            sample = source.read()
        finally:
            source.close()

        self.assertIsNotNone(sample)
        self.assertFalse(hasattr(sample, "body_frame"))
        self.assertEqual(sample.source_timestamp_ns, 123)
        self.assertEqual(sdk.body_api_calls, 0)


if __name__ == "__main__":
    unittest.main()
