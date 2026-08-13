from __future__ import annotations

import unittest

import numpy as np

from pico_body_tianji.controller_frame import ControllerFrame
from pico_body_tianji.controller_only_mapper import (
    ControllerOnlyTeleopMapper,
)
from pico_body_tianji.controller_only_source import (
    XRoboControllerOnlySource,
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
