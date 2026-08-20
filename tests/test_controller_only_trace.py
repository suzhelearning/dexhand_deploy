from __future__ import annotations

import unittest

from pico_body_tianji.controller_only.controller_only_trace import (
    _assert_replay_graph_is_safe,
    calculate_metrics,
)


class ControllerOnlyTraceMetricsTest(unittest.TestCase):
    def test_replay_rejects_hardware_bridge_graph(self) -> None:
        class _Graph:
            @staticmethod
            def get_node_names_and_namespaces():
                return [
                    ("tianji_kinematic_sim", "/"),
                    ("marvin_hardware_bridge", "/"),
                ]

        with self.assertRaisesRegex(RuntimeError, "marvin_hardware_bridge"):
            _assert_replay_graph_is_safe(_Graph())

    def test_replay_rejects_another_mocap_h5_replay(self) -> None:
        class _Graph:
            @staticmethod
            def get_node_names_and_namespaces():
                return [(("mocap_h5_replay"), "/")]

        with self.assertRaisesRegex(RuntimeError, "mocap_h5_replay"):
            _assert_replay_graph_is_safe(_Graph())

    def test_metrics_aggregate_official_diagnostics(self) -> None:
        frames = []
        for index in range(3):
            status = {}
            for side in ("left", "right"):
                status.update(
                    {
                        f"{side}_solve_time_ms": 1.0 + index,
                        f"{side}_transport_time_ms": 2.0 + index,
                        f"{side}_position_error_mm": float(index),
                        f"{side}_orientation_error_deg": 0.1 * index,
                        f"{side}_min_limit_margin_deg": 8.0 - index,
                        f"{side}_requested_max_joint_step_deg": 0.3,
                        f"{side}_max_joint_step_deg": 0.2,
                        f"{side}_target_saturated": index == 2,
                        f"{side}_workspace_backoff_active": index == 2,
                        f"{side}_soft_limit_active": False,
                        f"{side}_transport_restart_count": 0,
                        f"{side}_consecutive_rejections": index,
                    }
                )
            frames.append(
                {
                    "type": "frame",
                    "elapsed_s": index * 0.01,
                    "ik_status": status,
                }
            )

        metrics = calculate_metrics(frames)

        self.assertEqual(metrics["frame_count"], 3)
        self.assertAlmostEqual(metrics["effective_rate_hz"], 100.0)
        self.assertEqual(metrics["left"]["workspace_backoff_frames"], 1)
        self.assertEqual(metrics["right"]["peak_consecutive_rejections"], 2)
        self.assertAlmostEqual(
            metrics["left"]["applied_joint_step_deg"]["max"], 0.2
        )
