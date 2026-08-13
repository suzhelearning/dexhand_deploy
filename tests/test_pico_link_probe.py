from __future__ import annotations

import unittest

import numpy as np

from pico_body_tianji.controller_frame import ControllerFrame
from pico_body_tianji.controller_source import ControllerSample
from pico_body_tianji.pico_link_probe import PicoLinkProbeStats


def _sample(timestamp_ns: int, offset: float = 0.0) -> ControllerSample:
    left = np.array([offset, 0.1, 0.2, 0.0, 0.0, 0.0, 1.0])
    right = np.array([0.3, -offset, 0.2, 0.0, 0.0, 0.0, 1.0])
    return ControllerSample(
        frame=ControllerFrame.from_poses(left, right),
        source_timestamp_ns=timestamp_ns,
        right_a_pressed=False,
        body_frame=None,
        body_timestamp_ns=0,
        body_timestamp_fallback=False,
    )


class PicoLinkProbeStatsTest(unittest.TestCase):
    def test_controllers_can_be_live_without_body_or_trackers(self) -> None:
        stats = PicoLinkProbeStats()
        stats.observe(_sample(100), tracker_count=0)
        stats.observe(_sample(200, offset=0.01), tracker_count=0)

        self.assertTrue(stats.controller_link_live)
        self.assertTrue(stats.controller_timestamp_live)
        self.assertTrue(stats.left_controller_updated)
        self.assertTrue(stats.right_controller_updated)
        self.assertEqual(stats.body_samples, 0)
        self.assertEqual(stats.last_tracker_count, 0)
        self.assertEqual(stats.left_pose_updates, 1)
        self.assertEqual(stats.right_pose_updates, 1)

    def test_repeated_cached_timestamp_is_not_live(self) -> None:
        stats = PicoLinkProbeStats()
        stats.observe(_sample(100), tracker_count=None)
        stats.observe(_sample(100), tracker_count=None)

        self.assertFalse(stats.controller_link_live)
        self.assertFalse(stats.controller_timestamp_live)
        self.assertEqual(stats.valid_controller_samples, 2)
        self.assertEqual(stats.controller_timestamp_updates, 0)

    def test_one_stale_controller_does_not_pass_pair_probe(self) -> None:
        stats = PicoLinkProbeStats()
        stats.observe(_sample(100), tracker_count=0)
        second = _sample(200, offset=0.01)
        second.frame.right_pose[:] = stats.last_sample.frame.right_pose
        stats.observe(second, tracker_count=0)

        self.assertTrue(stats.controller_timestamp_live)
        self.assertTrue(stats.left_controller_updated)
        self.assertFalse(stats.right_controller_updated)
        self.assertFalse(stats.controller_link_live)

    def test_invalid_samples_do_not_prove_link(self) -> None:
        stats = PicoLinkProbeStats()
        stats.observe(None, tracker_count=0)
        stats.observe(None, tracker_count=0)

        self.assertFalse(stats.controller_link_live)
        self.assertEqual(stats.invalid_controller_samples, 2)


if __name__ == "__main__":
    unittest.main()
