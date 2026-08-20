from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

import numpy as np

from pico_body_tianji.controller_only.mocap_circle_compare import (
    CaptureSnapshot,
    CircleTrajectoryCapture,
    PositionSample,
    compare_trajectory_samples,
    write_capture_outputs,
)


class MocapCircleCompareTest(unittest.TestCase):
    @staticmethod
    def _sample(time_s: float, position_m, **metadata) -> PositionSample:
        return PositionSample(
            received_ns=1_000_000_000 + round(time_s * 1.0e9),
            position_m=np.asarray(position_m, dtype=np.float64),
            metadata=metadata,
        )

    def _snapshot(
        self,
        times: np.ndarray,
        target_positions: np.ndarray,
        motive_positions: np.ndarray,
    ) -> CaptureSnapshot:
        target = tuple(
            self._sample(time_s, position, frame_id="right_chest")
            for time_s, position in zip(times, target_positions)
        )
        solved = tuple(
            self._sample(time_s, position, frame_id="right_chest")
            for time_s, position in zip(times, target_positions)
        )
        motive = tuple(
            self._sample(
                time_s,
                position,
                frame_number=index,
                rigid_id=42,
            )
            for index, (time_s, position) in enumerate(
                zip(times, motive_positions)
            )
        )
        return CaptureSnapshot(
            active_ns=1_000_000_000,
            end_ns=1_000_000_000 + round(float(times[-1]) * 1.0e9),
            complete=True,
            stop_reason="圆轨迹完成",
            target=target,
            solved=solved,
            motive=motive,
            statuses=(),
        )

    def test_maps_motive_relative_motion_into_right_chest(self) -> None:
        times = np.linspace(-0.5, 4.0, 451)
        active = np.maximum(times, 0.0)
        motive_delta = np.column_stack(
            (
                0.1 * np.sin(active),
                0.1 * (1.0 - np.cos(active)),
                np.zeros_like(active),
            )
        )
        matrix = np.asarray(
            [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]]
        )
        chest_delta = motive_delta @ matrix.T
        target = chest_delta + np.asarray([0.41, -0.12, 0.28])
        motive = motive_delta + np.asarray([1.5, 2.0, -0.7])
        result = compare_trajectory_samples(
            self._snapshot(times, target, motive),
            matrix,
            maximum_lag_s=0.0,
        )
        self.assertLess(
            result["summary"]["target_vs_motive_direct"]["rmse_3d_mm"],
            0.05,
        )
        self.assertLess(
            result["summary"]["solved_vs_motive_direct"]["rmse_3d_mm"],
            0.2,
        )

    def test_estimates_positive_motive_tracking_lag(self) -> None:
        times = np.linspace(-0.5, 5.0, 551)

        def curve(value):
            active = np.maximum(value, 0.0)
            return np.column_stack(
                (
                    0.04 * np.sin(2.1 * active),
                    0.06 * (1.0 - np.cos(1.3 * active)),
                    0.03 * np.sin(0.7 * active),
                )
            )

        target = curve(times) + np.asarray([0.41, -0.12, 0.28])
        motive = curve(times - 0.12) + np.asarray([1.5, 2.0, -0.7])
        result = compare_trajectory_samples(
            self._snapshot(times, target, motive),
            np.eye(3),
            maximum_lag_s=0.3,
        )
        lag = result["summary"][
            "solved_vs_motive_lag_compensated"
        ]["estimated_lag_s"]
        self.assertAlmostEqual(lag, 0.12, delta=0.01)
        self.assertLess(
            result["summary"]["solved_vs_motive_lag_compensated"][
                "rmse_3d_mm"
            ],
            0.3,
        )

    def test_inactive_status_proves_recorder_started_before_circle(self) -> None:
        capture = CircleTrajectoryCapture("right_arm")
        capture.on_status(
            {
                "phase": "armed",
                "circle_trajectory": {"active": False, "complete": False},
            }
        )
        capture.on_status(
            {
                "phase": "stepping",
                "circle_trajectory": {
                    "active": True,
                    "complete": False,
                    "elapsed_hold_s": 0.4,
                },
            }
        )
        self.assertFalse(capture.state()["started_late"])

        late_capture = CircleTrajectoryCapture("right_arm")
        late_capture.on_status(
            {
                "phase": "stepping",
                "circle_trajectory": {
                    "active": True,
                    "complete": False,
                    "elapsed_hold_s": 0.4,
                },
            }
        )
        self.assertTrue(late_capture.state()["started_late"])

    def test_baseline_uses_earliest_preroll_before_status_delay(self) -> None:
        times = np.linspace(-1.0, 2.0, 301)
        delayed_motion = np.maximum(times + 0.4, 0.0)
        delta = np.column_stack(
            (
                np.zeros_like(times),
                0.02 * delayed_motion,
                np.zeros_like(times),
            )
        )
        target_base = np.asarray([0.41, -0.12, 0.28])
        motive_base = np.asarray([1.5, 2.0, -0.7])
        result = compare_trajectory_samples(
            self._snapshot(
                times,
                delta + target_base,
                delta + motive_base,
            ),
            np.eye(3),
            baseline_s=0.25,
            maximum_lag_s=0.0,
        )
        np.testing.assert_allclose(
            result["summary"]["baselines_m"]["target"],
            target_base,
            atol=1.0e-12,
        )
        np.testing.assert_allclose(
            result["summary"]["baselines_m"]["motive"],
            motive_base,
            atol=1.0e-12,
        )

    def test_writes_raw_aligned_summary_and_svg_outputs(self) -> None:
        times = np.linspace(-0.5, 2.0, 251)
        active = np.maximum(times, 0.0)
        delta = np.column_stack(
            (
                np.zeros_like(active),
                0.1 * (1.0 - np.cos(active)),
                0.1 * np.sin(active),
            )
        )
        snapshot = self._snapshot(
            times,
            delta + np.asarray([0.41, -0.12, 0.28]),
            delta + np.asarray([1.5, 2.0, -0.7]),
        )
        result = compare_trajectory_samples(
            snapshot, np.eye(3), maximum_lag_s=0.0
        )
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "recording"
            paths = write_capture_outputs(output, snapshot, result)
            self.assertEqual(
                set(paths),
                {
                    "target",
                    "solved",
                    "motive",
                    "status",
                    "comparison",
                    "summary",
                    "figure",
                },
            )
            for path in paths.values():
                self.assertTrue(path.is_file(), path)
            self.assertIn(
                "Mocap circle trajectory comparison",
                paths["figure"].read_text(encoding="utf-8"),
            )
            self.assertIn(
                "solved_vs_motive_direct",
                paths["summary"].read_text(encoding="utf-8"),
            )


if __name__ == "__main__":
    unittest.main()
