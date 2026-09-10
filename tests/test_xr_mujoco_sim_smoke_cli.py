import json
import os
from pathlib import Path
import subprocess
import sys
import unittest

from scripts.xr_mujoco_sim_smoke import (
    _fake_controller_grip,
    _fake_home_grip,
    _failure_report,
    _stop_launcher,
)


ROOT = Path(__file__).resolve().parents[1]


class XrMujocoSimSmokeCliTest(unittest.TestCase):
    def test_stop_launcher_is_bounded_and_reaps_process(self):
        process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
        returncode = _stop_launcher(process, timeout_s=1.0)
        self.assertIsNotNone(returncode)
        self.assertIsNotNone(process.returncode)

    def test_fake_controller_start_sequence_is_release_press_release(self):
        self.assertEqual(_fake_controller_grip(0), 0.0)
        self.assertEqual(_fake_controller_grip(299), 0.0)
        self.assertEqual(_fake_controller_grip(300), 1.0)
        self.assertEqual(_fake_controller_grip(499), 1.0)
        self.assertEqual(_fake_controller_grip(500), 0.0)
        self.assertEqual(_fake_home_grip(699), 0.0)
        self.assertEqual(_fake_home_grip(700), 1.0)
        self.assertEqual(_fake_home_grip(899), 1.0)
        self.assertEqual(_fake_home_grip(900), 0.0)

    def test_smoke_failure_report_never_remains_passed(self):
        result = _failure_report({}, "home", TimeoutError("not idle"))
        self.assertFalse(result["passed"])
        self.assertEqual(result["stage"], "home")
        self.assertIn("TimeoutError", result["error"])

    def test_help_declares_full_simulation_only_scope(self):
        result = subprocess.run(
            [sys.executable, str(ROOT / "scripts/xr_mujoco_sim_smoke.py"), "--help"],
            cwd=ROOT,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("simulation-only", result.stdout)
        self.assertIn("--arm-input", result.stdout)
        self.assertIn("--disable-hands", result.stdout)
        self.assertIn("--with-manus", result.stdout)

    @unittest.skipUnless(
        os.environ.get("TIANJI_RUN_XR_MUJOCO_SMOKE") == "1",
        "requires an explicit opt-in because it starts a managed local session",
    )
    def test_full_smoke_runs_xr_manus_hands_in_mujoco(self):
        result = subprocess.run(
            [
                sys.executable,
                str(ROOT / "scripts/xr_mujoco_sim_smoke.py"),
                "--arm-input",
                "xr_tracker",
                "--frames",
                "100",
                "--with-manus",
            ],
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=180,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout)
        self.assertTrue(report["passed"])
        self.assertTrue(report["hands_enabled"])
        self.assertTrue(report["mujoco_execution_verified"])
        self.assertTrue(report["mujoco_hand_overlay_verified"])
        self.assertGreater(report["hand_command_frames"], 10)
        self.assertGreater(report["hand_motion_rad"], 1.0e-4)
        self.assertGreater(report["raw_manus_callbacks"], 10)
        self.assertGreater(report["recorded_hand_command_frames"], 10)
        self.assertFalse(report["hardware_acceptance_complete"])

    @unittest.skipUnless(
        os.environ.get("TIANJI_RUN_XR_MUJOCO_SMOKE") == "1",
        "requires an explicit opt-in because it starts a managed local session",
    )
    def test_full_smoke_runs_both_xr_arm_bindings(self):
        for arm_input in ("xr_tracker", "xr_controller"):
            result = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "scripts/xr_mujoco_sim_smoke.py"),
                    "--arm-input",
                    arm_input,
                    "--frames",
                    "100",
                    "--disable-hands",
                ],
                cwd=ROOT,
                capture_output=True,
                text=True,
                timeout=120,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            report = json.loads(result.stdout)
            self.assertTrue(report["passed"])
            self.assertEqual(report["arm_input"], arm_input)
            self.assertTrue(report["simulation_only"])
            self.assertTrue(report["mujoco_execution_verified"])
            self.assertFalse(report["hardware_acceptance_complete"])


if __name__ == "__main__":
    unittest.main()
