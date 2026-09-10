from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]


class XrManusSimSmokeCliTest(unittest.TestCase):
    def test_help_declares_receive_only_scope(self):
        result = subprocess.run(
            [sys.executable, str(ROOT / "scripts/xr_manus_sim_smoke.py"), "--help"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("receive-only", result.stdout)
        self.assertIn("--arm-input", result.stdout)

    def test_cli_runs_both_binding_variants_without_router_or_devices(self):
        for arm_input in ("xr_tracker", "xr_controller"):
            result = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "scripts/xr_manus_sim_smoke.py"),
                    "--arm-input",
                    arm_input,
                    "--frames",
                    "100",
                ],
                cwd=ROOT,
                env={"PATH": __import__("os").environ["PATH"]},
                capture_output=True,
                text=True,
                timeout=30,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            report = json.loads(result.stdout)
            self.assertTrue(report["passed"])
            self.assertEqual(report["arm_input"], arm_input)
            self.assertFalse(report["robot_commands_enabled"])
            self.assertFalse(report["hardware_acceptance_complete"])


if __name__ == "__main__":
    unittest.main()
