from __future__ import annotations

import os
import subprocess
from pathlib import Path
import unittest

import yaml

from pico_body_tianji.config_loader import (
    DEFAULT_ROUTER_ENDPOINT,
    canonical_config_root,
    load_component_config,
    router_endpoint,
)


ROOT = Path(__file__).parents[1]
CONFIG = ROOT / "src" / "pico_body_tianji" / "config"
SCRIPTS = ROOT / "scripts"


class Task8ConfigTreeTest(unittest.TestCase):
    def test_canonical_config_tree_is_complete(self) -> None:
        required = (
            "robot/arm.yaml",
            "robot/wuji_hand2.yaml",
            "sources/pico_controller.yaml",
            "sources/mocap_live.yaml",
            "sources/h5_replay.yaml",
            "producers/ik.yaml",
            "producers/policy_hold.yaml",
            "coordinator/arm.yaml",
            "executors/mujoco.yaml",
            "executors/marvin.yaml",
            "executors/wuji_hand2.yaml",
            "recording/session.yaml",
            "replay/target.yaml",
            "replay/joint.yaml",
            "diagnostics/mocap_calibration.yaml",
            "sessions/pico_sim.yaml",
            "sessions/pico_real.yaml",
            "sessions/mocap_live_sim.yaml",
            "sessions/mocap_live_real.yaml",
            "sessions/h5_sim.yaml",
            "sessions/h5_real.yaml",
            "sessions/target_replay_sim.yaml",
            "sessions/joint_replay_sim.yaml",
            "sessions/diagnostic_mocap_calibration_sim.yaml",
        )
        self.assertTrue(all((CONFIG / path).is_file() for path in required))

    def test_session_config_cannot_copy_router_or_ik_authority(self) -> None:
        session = yaml.safe_load((CONFIG / "sessions/pico_sim.yaml").read_text())
        self.assertNotIn("router_endpoint", session)
        self.assertNotIn("ik_backend", session)
        ik = yaml.safe_load((CONFIG / "producers/ik.yaml").read_text())
        self.assertIn("ik_backend", ik)

    def test_loader_and_endpoint_are_strict(self) -> None:
        with self.subTest("unknown key"):
            bad = CONFIG / "producers" / "_task8_bad.yaml"
            bad.write_text("ik_backend: pinocchio_cpp\nunexpected: true\n")
            try:
                with self.assertRaises(ValueError):
                    load_component_config(bad, allowed_keys={"ik_backend"})
            finally:
                bad.unlink()


class Task8LauncherTest(unittest.TestCase):
    def test_new_launcher_scripts_exist_and_are_executable(self) -> None:
        for name in ("run_source.sh", "run_producer.sh", "run_executor.sh", "run_session.sh"):
            path = SCRIPTS / name
            self.assertTrue(path.is_file(), name)
            self.assertTrue(os.access(path, os.X_OK), name)

    def test_replay_record_is_rejected_before_router_access(self) -> None:
        result = subprocess.run(
            [str(SCRIPTS / "run_session.sh"), "--profile", "target_replay_sim", "--record", "/tmp/nope.h5"],
            text=True,
            capture_output=True,
            env={"PATH": os.environ["PATH"]},
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("replay profile cannot be recorded", result.stderr)


if __name__ == "__main__":
    unittest.main()
