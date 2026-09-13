from __future__ import annotations

import unittest


class XrManusSimSmokeTest(unittest.TestCase):
    def test_cli_defaults_to_controller_only_input(self):
        from unittest.mock import patch

        import scripts.xr_manus_sim_smoke as smoke

        captured = {}

        def fake_run_offline_smoke(*, arm_input, frame_count):
            captured.update(arm_input=arm_input, frame_count=frame_count)
            return {"passed": True}

        with patch(
            "tianji_teleop.hand_tracking.xr_manus_smoke.run_offline_smoke",
            side_effect=fake_run_offline_smoke,
        ):
            self.assertEqual(smoke.main([]), 0)

        self.assertEqual(captured["arm_input"], "xr_controller")

    def test_default_smoke_uses_controller_only_input(self):
        from tianji_teleop.hand_tracking.xr_manus_smoke import run_offline_smoke

        report = run_offline_smoke(frame_count=100)

        self.assertTrue(report["passed"])
        self.assertEqual(report["arm_input"], "xr_controller")
        self.assertTrue(report["controller_arm_binding_verified"])

    def test_tracker_route_exercises_observation_bridge_and_operator_edge(self):
        from tianji_teleop.hand_tracking.xr_manus_smoke import run_offline_smoke

        report = run_offline_smoke(arm_input="xr_tracker", frame_count=100)

        self.assertTrue(report["passed"])
        self.assertEqual(report["arm_input"], "xr_tracker")
        self.assertEqual(report["arm_targets"], report["target_frames"] * 2)
        self.assertEqual(report["hand_targets"], report["target_frames"] * 2)
        self.assertEqual(report["start_events"], 1)
        self.assertGreater(report["arm_motion_m"], 0.001)
        self.assertGreater(report["hand_motion_m"], 0.001)
        self.assertFalse(report["robot_commands_enabled"])
        self.assertFalse(report["hardware_acceptance_complete"])

    def test_controller_route_changes_only_arm_binding(self):
        from tianji_teleop.hand_tracking.xr_manus_smoke import run_offline_smoke

        report = run_offline_smoke(arm_input="xr_controller", frame_count=100)

        self.assertTrue(report["passed"])
        self.assertEqual(report["arm_input"], "xr_controller")
        self.assertEqual(report["arm_targets"], report["target_frames"] * 2)
        self.assertEqual(report["hand_targets"], report["target_frames"] * 2)
        self.assertEqual(report["start_events"], 1)
        self.assertTrue(report["controller_arm_binding_verified"])
        self.assertFalse(report["robot_commands_enabled"])

    def test_smoke_rejects_short_trace_before_start_event(self):
        from tianji_teleop.hand_tracking.xr_manus_smoke import run_offline_smoke

        with self.assertRaisesRegex(ValueError, "frame_count"):
            run_offline_smoke(arm_input="xr_tracker", frame_count=10)


if __name__ == "__main__":
    unittest.main()
