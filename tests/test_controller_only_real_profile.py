from __future__ import annotations

import re
import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
REAL_CONFIG = (
    ROOT
    / "src/pico_body_tianji/config/mode/controller_only/controller_only_real.yaml"
)
IK_CONFIG = (
    ROOT
    / "src/pico_body_tianji/config/mode/controller_only/controller_only_ik.yaml"
)
REAL_SCRIPT = ROOT / "scripts/run_controller_only_real.sh"


def _parameters(path: Path, node_name: str) -> dict:
    with path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    return document[node_name]["ros__parameters"]


class ControllerOnlyRealProfileTest(unittest.TestCase):
    def test_90_hz_profile_has_coherent_effective_limits(self):
        real = _parameters(REAL_CONFIG, "marvin_hardware_bridge")
        ik = _parameters(IK_CONFIG, "tianji_kinematic_sim")

        self.assertEqual(real["rate"], 90.0)
        self.assertEqual(real["velocity_ratio"], 50)
        self.assertEqual(real["acceleration_ratio"], 70)

        ratio_step = (
            real["maximum_output_step_deg"]
            * real["velocity_ratio"]
            / 10
        )
        speed_step = real["maximum_teleop_speed_deg_s"] / real["rate"]
        effective_step = min(ratio_step, speed_step)

        self.assertAlmostEqual(effective_step, 0.25)
        self.assertAlmostEqual(effective_step * real["rate"], 22.5)
        self.assertGreater(effective_step, ik["max_joint_step_deg"])

    def test_shell_defaults_match_yaml_profile(self):
        real = _parameters(REAL_CONFIG, "marvin_hardware_bridge")
        script = REAL_SCRIPT.read_text(encoding="utf-8")

        velocity = re.search(r"^VELOCITY_RATIO=(\d+)$", script, re.MULTILINE)
        acceleration = re.search(
            r"^ACCELERATION_RATIO=(\d+)$", script, re.MULTILINE
        )
        self.assertIsNotNone(velocity)
        self.assertIsNotNone(acceleration)
        self.assertEqual(int(velocity.group(1)), real["velocity_ratio"])
        self.assertEqual(
            int(acceleration.group(1)), real["acceleration_ratio"]
        )


if __name__ == "__main__":
    unittest.main()
