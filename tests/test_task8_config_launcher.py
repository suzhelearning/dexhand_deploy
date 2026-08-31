from __future__ import annotations

import os
import subprocess
import sys
import tempfile
from pathlib import Path
import unittest

import yaml

from tianji_teleop.config_loader import (
    DEFAULT_ROUTER_ENDPOINT,
    canonical_config_root,
    load_component_config,
    router_endpoint,
)


ROOT = Path(__file__).parents[1]
CONFIG = ROOT / "src" / "tianji_teleop" / "config"
SCRIPTS = ROOT / "scripts"


class Task8ConfigTreeTest(unittest.TestCase):
    def test_canonical_config_tree_is_complete(self) -> None:
        required = (
            "robot/arm.yaml",
            "robot/wuji_hand2.yaml",
            "robot/devices.yaml",
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
        session = yaml.safe_load((CONFIG / "sessions/mocap_live_sim.yaml").read_text())
        self.assertNotIn("router_endpoint", session)
        self.assertNotIn("ik_backend", session)
        ik = yaml.safe_load((CONFIG / "producers/ik.yaml").read_text())
        self.assertIn("ik_backend", ik)

    def test_real_device_defaults_are_canonical_and_used(self) -> None:
        devices = yaml.safe_load((CONFIG / "robot/devices.yaml").read_text())
        self.assertEqual(devices["marvin"]["ip"], "192.168.1.190")
        self.assertEqual(devices["wuji_hand2"]["right"]["ip"], "192.168.1.111")
        self.assertEqual(
            devices["wuji_hand2"]["right"]["serial"], "WH2KA01260814006"
        )
        launcher = (SCRIPTS / "run_executor.sh").read_text(encoding="utf-8")
        self.assertIn("TIANJI_DEVICE_CONFIG", launcher)
        self.assertIn('args+=(--serial "${wuji_serial}")', launcher)

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
            self.assertIn("arm_ik_producer", script)
            self.assertIn("mujoco_executor", script)
            self.assertNotIn("legacy", script.lower())
        cmake = (ROOT / "src" / "tianji_teleop" / "CMakeLists.txt").read_text(encoding="utf-8")
        python_install = cmake.split("install(\n  DIRECTORY tianji_teleop", 1)[1].split("install(\n  DIRECTORY assets", 1)[0]
        self.assertIn('PATTERN "__pycache__" EXCLUDE', python_install)
        self.assertNotIn('PATTERN "mode"', python_install)
        self.assertIn("--delete-excluded", deploy)


class Task8LauncherTest(unittest.TestCase):
    def test_new_launcher_scripts_exist_and_are_executable(self) -> None:
        for name in ("run_source.sh", "run_producer.sh", "run_executor.sh", "run_session.sh"):
            path = SCRIPTS / name
            self.assertTrue(path.is_file(), name)
            self.assertTrue(os.access(path, os.X_OK), name)

    def test_confirmed_real_launcher_issues_sealed_capability(self) -> None:
        command = (
            "from tianji_teleop.executors.marvin.preflight import "
            "trusted_real_capability; print(trusted_real_capability().admitted)"
        )
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPTS / "run_confirmed_real_session.py"),
                "--profile", "h5_real", "--speed", "1", "--yaw-deg", "0",
                "--", sys.executable, "-c", command,
            ],
            cwd=ROOT,
            text=True,
            capture_output=True,
            env={**os.environ, "PYTHONPATH": str(ROOT / "src/tianji_teleop")},
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "True")
        rejected = subprocess.run(
            [
                sys.executable,
                str(SCRIPTS / "run_confirmed_real_session.py"),
                "--profile", "h5_real", "--speed", "1.01", "--yaw-deg", "0",
                "--", sys.executable, "-c", "print('must not run')",
            ],
            cwd=ROOT,
            text=True,
            capture_output=True,
        )
        self.assertEqual(rejected.returncode, 2)
        self.assertIn("speed must be in (0, 1]", rejected.stderr)

    def test_replay_record_is_rejected_before_router_access(self) -> None:
        result = subprocess.run(
            [str(SCRIPTS / "run_session.sh"), "--profile", "target_replay_sim", "--record", "/tmp/nope.h5"],
            text=True,
            capture_output=True,
            env={"PATH": os.environ["PATH"]},
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("replay profile cannot be recorded", result.stderr)

    def _run_mujoco_executor(
        self, *display_args: str
    ) -> tuple[subprocess.CompletedProcess[str], list[str]]:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bin_dir = root / "bin"
            bin_dir.mkdir()
            capture = root / "python-args.txt"
            python = bin_dir / "python"
            python.write_text(
                "#!/bin/sh\nprintf '%s\\n' \"$@\" > \"$CAPTURE\"\n",
                encoding="utf-8",
            )
            python.chmod(0o755)
            pixi = bin_dir / "pixi"
            pixi.write_text(
                "#!/bin/sh\n"
                "[ \"${1:-}\" = run ] && [ \"${2:-}\" = python ] || exit 2\n"
                "shift 2\n"
                "exec \"$REAL_PYTHON\" \"$@\"\n",
                encoding="utf-8",
            )
            pixi.chmod(0o755)
            config = CONFIG / "executors" / "mujoco.yaml"
            env = os.environ.copy()
            env.update(
                {
                    "PATH": f"{bin_dir}:{env['PATH']}",
                    "CAPTURE": str(capture),
                    "REAL_PYTHON": sys.executable,
                    "TIANJI_ROUTER_ZID": "router",
                    "TIANJI_COORDINATOR_INSTANCE_ID": "coordinator",
                    "TIANJI_COMPONENT_INSTANCE_ID": "executor",
                }
            )
            result = subprocess.run(
                [
                    str(SCRIPTS / "run_executor.sh"),
                    "--executor",
                    "mujoco",
                    *display_args,
                    "--config",
                    str(config),
                ],
                cwd=ROOT,
                text=True,
                capture_output=True,
                env=env,
            )
            arguments = (
                capture.read_text(encoding="utf-8").splitlines()
                if capture.exists()
                else []
            )
            return result, arguments

    def test_mujoco_viewer_consumes_override_without_forwarding_headless(
        self,
    ) -> None:
        result, arguments = self._run_mujoco_executor("--viewer")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("--viewer", arguments)
        self.assertNotIn("--headless", arguments)
        self.assertEqual(arguments.count("--config"), 1)
        config = CONFIG / "executors" / "mujoco.yaml"
        self.assertEqual(arguments[arguments.index("--config") + 1], str(config))

    def test_mujoco_headless_is_forwarded_and_display_flags_are_exclusive(
        self,
    ) -> None:
        result, arguments = self._run_mujoco_executor()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(arguments.count("--headless"), 1)

        result, arguments = self._run_mujoco_executor("--headless")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(arguments.count("--headless"), 1)

        result, _ = self._run_mujoco_executor("--viewer", "--headless")
        self.assertEqual(result.returncode, 2)
        self.assertIn("互斥", result.stderr)

    def _run_h5_session_until_components_are_wired(
        self, *display_args: str
    ) -> list[list[str]]:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bin_dir = root / "bin"
            bin_dir.mkdir()
            capture = root / "launches.tsv"

            python = bin_dir / "python"
            python.write_text(
                "#!/bin/sh\n"
                "case \"${1:-}\" in\n"
                "  -) printf '%s\\n' router-zid ;;\n"
                "  -c) exit 0 ;;\n"
                "  *) exit 1 ;;\n"
                "esac\n",
                encoding="utf-8",
            )
            python.chmod(0o755)

            pixi = bin_dir / "pixi"
            pixi.write_text(
                "#!/bin/sh\n"
                "[ \"${1:-}\" = run ] && [ \"${2:-}\" = python ] || exit 2\n"
                "shift 2\n"
                "exec \"$REAL_PYTHON\" \"$@\"\n",
                encoding="utf-8",
            )
            pixi.chmod(0o755)

            setsid = bin_dir / "setsid"
            setsid.write_text(
                "#!/bin/sh\n"
                "{ printf 'CALL'; for arg in \"$@\"; do "
                "printf '\\t%s' \"$arg\"; done; "
                "printf '\\n'; } >> \"$CAPTURE\"\n"
                "case \" $* \" in\n"
                "  *'/run_source.sh '*) "
                "exec /usr/bin/setsid /bin/sleep 0.05 ;;\n"
                "  *) exec /usr/bin/setsid /bin/sleep 2 ;;\n"
                "esac\n",
                encoding="utf-8",
            )
            setsid.chmod(0o755)

            sleep = bin_dir / "sleep"
            sleep.write_text(
                "#!/bin/sh\nexec /bin/sleep 0.01\n",
                encoding="utf-8",
            )
            sleep.chmod(0o755)

            h5_path = root / "take.h5"
            h5_path.touch()
            env = os.environ.copy()
            env.update(
                {
                    "PATH": f"{bin_dir}:{env['PATH']}",
                    "CAPTURE": str(capture),
                    "REAL_PYTHON": sys.executable,
                    "TIANJI_TELEOP_NODE_LIST_OVERRIDE": "",
                    "TIANJI_TELEOP_RUNTIME_DIR": str(root / "runtime"),
                    "TIANJI_VALIDATION_HAND_MODE": "retarget",
                }
            )
            result = subprocess.run(
                [
                    str(SCRIPTS / "run_session.sh"),
                    "--profile",
                    "h5_sim",
                    "--h5",
                    str(h5_path),
                    *display_args,
                ],
                cwd=ROOT,
                text=True,
                capture_output=True,
                env=env,
                timeout=10,
            )
            self.assertNotEqual(result.returncode, 0)
            return [
                line.split("\t")[1:]
                for line in capture.read_text(encoding="utf-8").splitlines()
            ]

    def test_h5_session_selects_default_viewer_and_explicit_headless(
        self,
    ) -> None:
        cases = (((), "--viewer"), (("--headless",), "--headless"))
        for display_args, expected in cases:
            with self.subTest(display_args=display_args):
                launches = self._run_h5_session_until_components_are_wired(
                    *display_args
                )
                arm_executor = next(
                    args
                    for args in launches
                    if any(arg.endswith("/run_executor.sh") for arg in args)
                    and args[args.index("--executor") + 1] == "mujoco"
                )
                source = next(
                    args
                    for args in launches
                    if any(arg.endswith("/run_source.sh") for arg in args)
                )
                self.assertIn(expected, arm_executor)
                self.assertNotIn(
                    "--headless" if expected == "--viewer" else "--viewer",
                    arm_executor,
                )
                self.assertNotIn("--viewer", source)
                self.assertNotIn("--headless", source)


if __name__ == "__main__":
    unittest.main()
