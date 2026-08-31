from __future__ import annotations

import unittest

import numpy as np

from tianji_teleop.sources.mocap.h5 import compose_pose, invert_pose
from tianji_teleop.sources.regrind_policy_node import _pose_error


class RegrindRealPreflightTest(unittest.TestCase):
    def test_one_wrist_alignment_preserves_hammer_relative_pose(self) -> None:
        live_wrist = np.asarray([0.5, -0.2, 0.3, 0.0, 0.0, 0.0, 1.0])
        live_hammer = np.asarray([0.6, -0.1, 0.2, 0.0, 0.0, 0.0, 1.0])
        reference_wrist = np.asarray([0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0])
        training_from_motive = compose_pose(reference_wrist, invert_pose(live_wrist))

        np.testing.assert_allclose(
            compose_pose(training_from_motive, live_wrist), reference_wrist, atol=1e-9
        )
        aligned_hammer = compose_pose(training_from_motive, live_hammer)
        position_error, orientation_error = _pose_error(
            aligned_hammer,
            np.asarray([0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 1.0]),
        )
        self.assertAlmostEqual(position_error, 0.0)
        self.assertAlmostEqual(orientation_error, 0.0)


if __name__ == "__main__":
    unittest.main()
