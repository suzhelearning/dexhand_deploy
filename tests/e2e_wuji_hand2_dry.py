"""Canonical Wuji Hand 2 dry-run regressions without legacy transport topics."""
from __future__ import annotations

import unittest

import numpy as np

from pico_body_tianji.executors.wuji_hand2.config import WujiHandConfig
from pico_body_tianji.executors.wuji_hand2.node import _retarget_keypoints


class WujiHand2DryRunTest(unittest.TestCase):
    @staticmethod
    def _open_hand_pose() -> np.ndarray:
        points = np.zeros((21, 3), dtype=np.float64)
        for index, base in enumerate((1, 5, 9, 13, 17)):
            points[base : base + 4, 1] = np.linspace(0.01, 0.10, 4) + index * 0.004
            points[base : base + 4, 2] = np.linspace(-0.01, -0.05, 4)
        return points

    def test_retarget_is_finite_and_has_twenty_joints(self) -> None:
        values = _retarget_keypoints(self._open_hand_pose(), WujiHandConfig.load())
        self.assertEqual(len(values), 20)
        self.assertTrue(np.isfinite(values).all())

    def test_wrist_translation_does_not_change_retarget(self) -> None:
        config = WujiHandConfig.load()
        pose = self._open_hand_pose()
        translated = pose + np.array([0.5, -0.3, 0.2])
        np.testing.assert_allclose(
            _retarget_keypoints(pose, config),
            _retarget_keypoints(translated, config),
            atol=5.0e-3,
        )

    def test_nonfinite_keypoints_are_rejected(self) -> None:
        pose = self._open_hand_pose()
        pose[2, 1] = np.nan
        with self.assertRaises(ValueError):
            _retarget_keypoints(pose, WujiHandConfig.load())


if __name__ == "__main__":
    unittest.main()
