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

    def test_hand_enabled_profiles_select_one_hand_executor_authority(self) -> None:
        for profile in ("h5_sim", "h5_real", "target_replay_sim", "joint_replay_sim"):
            with self.subTest(profile=profile):
                value = yaml.safe_load((CONFIG / "sessions" / f"{profile}.yaml").read_text())
                self.assertEqual(value["hand_executor"], "wuji_hand2")
                self.assertIn(value["hand_overlay"], {"none", "mujoco"})
                self.assertNotEqual(value["hand_executor"], value["hand_overlay"])

    def test_session_launcher_wires_authorities_and_enables_passive_mujoco_hand_overlay(self) -> None:
        launcher = (SCRIPTS / "run_session.sh").read_text(encoding="utf-8")
        self.assertIn("TIANJI_AUTHORITIES", launcher)
        self.assertIn("TIANJI_RECORDING_CONFIG=", launcher)
        self.assertIn('hand_args+=(--hand-sides "${active_hand_sides}" --hand-overlay)', launcher)
        self.assertIn('hand_executor}" == wuji_hand2', launcher)
    def test_deploy_and_doctor_match_only_deleted_entries(self) -> None:
        deploy = (SCRIPTS / "deploy_ik_runtime.sh").read_text(encoding="utf-8")
        doctor = (SCRIPTS / "doctor.sh").read_text(encoding="utf-8")
        for script in (deploy, doctor):
            self.assertIn("pico_controller_input*", script)
            self.assertIn("pico_link_probe*", script)
            self.assertIn("mocap_keyboard_step*", script)
            self.assertIn("tianji_kinematic_sim*", script)
            self.assertNotIn('"/pico_controller_*"', script)
        self.assertIn("pico_controller_source", doctor)
        cmake = (ROOT / "src" / "pico_body_tianji" / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn('PATTERN "full_body" EXCLUDE', cmake)
        self.assertIn('PATTERN "controller_only" EXCLUDE', cmake)
        self.assertIn("--delete-excluded", deploy)


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
