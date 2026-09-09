from __future__ import annotations

import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from tianji_world_output.config_loader import TianjiConfig
from tianji_world_output.transform_utils import (
    apply_world_rotation_to_chest_pose,
    transform_world_to_chest,
)


class TianjiWorldOutputCompatibilityTest(unittest.TestCase):
    def test_config_contains_legacy_mocap_mapping_and_arm_home(self) -> None:
        config = TianjiConfig.load(use_ros=False)
        np.testing.assert_allclose(config.mocap_to_robot, np.eye(3))
        self.assertEqual(config.init_pos["left"].shape, (3,))
        self.assertEqual(config.init_quat["right"].shape, (4,))

    def test_transform_conventions_match_legacy_world_chest_contract(self) -> None:
        np.testing.assert_allclose(
            transform_world_to_chest([1.0, 2.0, 3.0], "left"),
            [1.0, -3.0, 2.0],
        )
        np.testing.assert_allclose(
            transform_world_to_chest([1.0, 2.0, 3.0], "right"),
            [1.0, 3.0, -2.0],
        )
        base = Rotation.identity().as_matrix()
        result = apply_world_rotation_to_chest_pose(
            base, Rotation.from_euler("z", 90.0, degrees=True), "left"
        )
        np.testing.assert_allclose(result @ result.T, np.eye(3), atol=1.0e-12)
        self.assertAlmostEqual(np.linalg.det(result), 1.0)


if __name__ == "__main__":
    unittest.main()
