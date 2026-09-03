from __future__ import annotations

import os
import runpy
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
            "sources/regrind_policy.yaml",
            "producers/ik.yaml",
            "producers/ik_regrind.yaml",
            "producers/policy_hold.yaml",
            "coordinator/arm.yaml",
            "coordinator/arm_regrind.yaml",
            "executors/mujoco.yaml",
            "executors/marvin.yaml",
            "executors/marvin_impedance.yaml",
            "executors/wuji_hand2.yaml",
            "executors/wuji_hand2_regrind.yaml",
            "recording/session.yaml",
            "replay/target.yaml",
            "replay/joint.yaml",
            "diagnostics/mocap_calibration.yaml",
            "sessions/mocap_live_sim.yaml",
            "sessions/mocap_live_real.yaml",
            "sessions/h5_sim.yaml",
            "sessions/h5_real.yaml",
            "sessions/regrind_real.yaml",
            "sessions/target_replay_sim.yaml",
            "sessions/joint_replay_sim.yaml",
            "sessions/diagnostic_mocap_calibration_sim.yaml",
        )
        self.assertTrue(all((CONFIG / path).is_file() for path in required))

    def test_regrind_coordinator_profile_is_isolated_from_shared_safety_step(self) -> None:
        shared = yaml.safe_load((CONFIG / "coordinator/arm.yaml").read_text())
        self.assertEqual(float(shared["maximum_command_step_rad"]), 0.00596902599)
        regrind = yaml.safe_load((CONFIG / "coordinator/arm_regrind.yaml").read_text())
        self.assertEqual(float(regrind["maximum_command_step_rad"]), 1000.0)
        profile = yaml.safe_load((CONFIG / "sessions/regrind_real.yaml").read_text())
        self.assertEqual(profile["coordinator_config"], "coordinator/arm_regrind.yaml")

    def test_regrind_ik_profile_is_isolated_from_shared_h5_and_mocap_settings(self) -> None:
        shared = yaml.safe_load((CONFIG / "producers/ik.yaml").read_text())
        regrind = yaml.safe_load((CONFIG / "producers/ik_regrind.yaml").read_text())
        self.assertEqual(float(shared["maximum_joint_step_rad"]), 0.00596902599)
        self.assertEqual(float(shared["qp_position_time_constant_s"]), 0.30)
        self.assertEqual(float(shared["qp_orientation_time_constant_s"]), 0.40)
        self.assertEqual(float(regrind["maximum_joint_step_rad"]), 1000.0)
        self.assertEqual(float(regrind["qp_position_time_constant_s"]), 0.02)
        self.assertEqual(float(regrind["qp_orientation_time_constant_s"]), 0.03)
        regrind_profile = yaml.safe_load((CONFIG / "sessions/regrind_real.yaml").read_text())
        self.assertEqual(regrind_profile["arm_producer_config"], "producers/ik_regrind.yaml")
        for profile in ("h5_sim", "h5_real", "mocap_live_sim", "mocap_live_real"):
            value = yaml.safe_load((CONFIG / "sessions" / f"{profile}.yaml").read_text())
            self.assertEqual(value["arm_producer_config"], "producers/ik.yaml")

    def test_hand_executor_profiles_keep_shared_rate_and_enable_regrind_interpolation(self) -> None:
        shared = yaml.safe_load((CONFIG / "executors/wuji_hand2.yaml").read_text())
        regrind = yaml.safe_load((CONFIG / "executors/wuji_hand2_regrind.yaml").read_text())
        self.assertEqual(float(shared["rate_hz"]), 60.0)
        self.assertFalse(shared["linear_interpolation"])
        self.assertEqual(float(regrind["rate_hz"]), 100.0)
        self.assertTrue(regrind["linear_interpolation"])
        regrind_profile = yaml.safe_load((CONFIG / "sessions/regrind_real.yaml").read_text())
        self.assertEqual(regrind_profile["hand_executor_config"], "executors/wuji_hand2_regrind.yaml")
        for profile in ("mocap_live_sim", "mocap_live_real", "h5_sim", "h5_real", "wuji_direct_real"):
            value = yaml.safe_load((CONFIG / "sessions" / f"{profile}.yaml").read_text())
            self.assertEqual(value.get("hand_executor_config", "executors/wuji_hand2.yaml"), "executors/wuji_hand2.yaml")


    def test_session_config_cannot_copy_router_or_ik_authority(self) -> None:
        session = yaml.safe_load((CONFIG / "sessions/mocap_live_sim.yaml").read_text())
        self.assertNotIn("router_endpoint", session)
        self.assertNotIn("ik_backend", session)
        ik = yaml.safe_load((CONFIG / "producers/ik.yaml").read_text())
        self.assertIn("ik_backend", ik)

    def test_real_device_defaults_are_canonical_and_used(self) -> None:
        devices = yaml.safe_load((CONFIG / "robot/devices.yaml").read_text())
        marvin = yaml.safe_load((CONFIG / "executors/marvin.yaml").read_text())
        self.assertEqual(devices["marvin"]["ip"], "192.168.1.190")
        self.assertGreaterEqual(float(marvin["connection_wait_s"]), 5.0)
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
        for profile in ("h5_sim", "h5_real", "regrind_real", "target_replay_sim", "joint_replay_sim"):
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
        self.assertIn('hand_executor_config="$(profile_value hand_executor_config)"', launcher)
        self.assertIn('hand_executor_config=executors/wuji_hand2.yaml', launcher)
        executor_launcher = (SCRIPTS / "run_executor.sh").read_text(encoding="utf-8")
        self.assertIn('value.get("linear_interpolation") is True', executor_launcher)
        self.assertIn('native_args+=(--linear-interpolation)', executor_launcher)
        bridge = (ROOT / "src/tianji_teleop/src/wuji_hand2/wuji_hand2_bridge_node.cpp").read_text(encoding="utf-8")
        self.assertIn("bool linear_interpolation{false};", bridge)
        self.assertIn("if (params_.linear_interpolation) direct_interpolator_.accept", bridge)
        self.assertIn("params_.linear_interpolation ? direct_interpolator_.sample(current) : direct_command_", bridge)

    def test_regrind_real_uses_direct_wuji_and_existing_arm_ik(self) -> None:
        profile = yaml.safe_load((CONFIG / "sessions/regrind_real.yaml").read_text())
        self.assertEqual(profile["required_capability"], "real")
        self.assertEqual(profile["arm_producer_config"], "producers/ik_regrind.yaml")
        self.assertEqual(profile["arm_executor_config"], "executors/marvin_impedance.yaml")
        self.assertEqual(profile["hand_mode"], "direct")
        self.assertEqual(profile["hand_executor"], "wuji_hand2")
        launcher = (SCRIPTS / "run_session.sh").read_text(encoding="utf-8")
        self.assertIn('hand_producer_id_array+=("regrind_policy")', launcher)
        self.assertIn(
            'launch regrind_alignment_viewer "${base_env[@]}" python "${viewer_entry}" "${extra_args[@]}" --viewer',
            launcher,
        )
        self.assertIn('source_args+=("${extra_args[@]}")', launcher)
        self.assertIn('TIANJI_ARM_COMMAND_PATH=${arm_command_path}', launcher)
        self.assertIn('TIANJI_ARM_PRODUCER_LOGICAL_ID=${arm_producer_id}', launcher)
        self.assertTrue(os.access(ROOT / "src/tianji_teleop/scripts/regrind_policy", os.X_OK))
        self.assertIn("regrind_real 的策略 capability 要求 --speed 1.0", launcher)

    def test_regrind_real_uses_direct_arm_path_and_bounded_target_conditioning(self) -> None:
        profile = yaml.safe_load((CONFIG / "sessions/regrind_real.yaml").read_text())
        self.assertEqual(profile["arm_command_path"], "direct")
        source = yaml.safe_load((CONFIG / "sources/regrind_policy.yaml").read_text())
        self.assertEqual(source["maximum_linear_speed_m_s"], 0.09)
        self.assertEqual(source["maximum_angular_speed_rad_s"], 0.3875)
        self.assertEqual(source["maximum_linear_acceleration_m_s2"], 0.875)
        self.assertEqual(source["maximum_angular_acceleration_rad_s2"], 2.25)
        self.assertEqual(source["hammer_start_position_tolerance_m"], 0.02)
        self.assertEqual(source["hammer_start_orientation_tolerance_deg"], 10.0)
        self.assertEqual(source["hand_maximum_step_rad"], 0.01)

    def test_real_launcher_starts_hand_feedback_before_arm_executor(self) -> None:
        launcher = (SCRIPTS / "run_session.sh").read_text(encoding="utf-8")
        marker = 'if [[ "${required_capability}" == real ]]; then\n  launch_arm_producer\n  launch source'
        start = launcher.index(marker)
        real_launches = launcher[start:launcher.index("\nelse\n", start)]
        self.assertLess(
            real_launches.index("launch_hand_executor"),
            real_launches.index("launch_arm_executor"),
        )

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

    def test_regrind_real_accepts_equivalent_numeric_speed_one(self) -> None:
        result = subprocess.run(
            [
                str(SCRIPTS / "run_session.sh"),
                "--profile", "regrind_real", "--confirm-real", "--speed", "1e0",
            ],
            cwd=ROOT,
            text=True,
            capture_output=True,
            env={**os.environ, "TIANJI_TELEOP_NODE_LIST_OVERRIDE": ""},
        )
        self.assertNotEqual(result.returncode, 2)
        self.assertNotIn("regrind_real 的策略 capability 要求 --speed 1.0", result.stderr)

        rejected = subprocess.run(
            [
                str(SCRIPTS / "run_session.sh"),
                "--profile", "regrind_real", "--confirm-real", "--speed", "0.25",
            ],
            cwd=ROOT,
            text=True,
            capture_output=True,
            env={**os.environ, "TIANJI_TELEOP_NODE_LIST_OVERRIDE": ""},
        )
        self.assertEqual(rejected.returncode, 2)
        self.assertIn("regrind_real 的策略 capability 要求 --speed 1.0", rejected.stderr)

    def test_regrind_viewer_and_preflight_share_hammer_start_tolerance(self) -> None:
        scope = runpy.run_path(
            str(SCRIPTS / "regrind_live_infer.py"),
            run_name="regrind_viewer_tolerance_check",
        )
        self.assertEqual(scope.get("_HAMMER_START_POSITION_TOLERANCE_M"), 0.02)
        self.assertEqual(
            scope.get("_HAMMER_START_ORIENTATION_TOLERANCE_DEG"), 10.0
        )
        aligned = scope["_hammer_pose_is_aligned"]
        self.assertTrue(aligned(0.02, 10.0))
        self.assertFalse(aligned(0.0200001, 10.0))
        self.assertFalse(aligned(0.02, 10.0001))

    def test_regrind_alignment_viewer_accepts_reference_speed(self) -> None:
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPTS / "regrind_live_infer.py"),
                "--reference-speed",
                "0.5",
                "--viewer",
                "--help",
            ],
            cwd=ROOT,
            text=True,
            capture_output=True,
            env={
                **os.environ,
                "PYTHONPATH": os.pathsep.join(
                    (str(ROOT / "src/tianji_teleop"), str(ROOT / "vendor/python"))
                ),
            },
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--reference-speed", result.stdout)

    def test_regrind_alignment_viewer_rejects_invalid_reference_speed(self) -> None:
        for value in ("0", "-0.1", "1.1", "nan", "inf", "-inf"):
            with self.subTest(value=value):
                result = subprocess.run(
                    [
                        sys.executable,
                        str(SCRIPTS / "regrind_live_infer.py"),
                        "--reference",
                        __file__,
                        "--model",
                        __file__,
                        "--viewer",
                        f"--reference-speed={value}",
                    ],
                    cwd=ROOT,
                    text=True,
                    capture_output=True,
                    env={
                        **os.environ,
                        "PYTHONPATH": os.pathsep.join(
                            (
                                str(ROOT / "src/tianji_teleop"),
                                str(ROOT / "vendor/python"),
                            )
                        ),
                    },
                )
                self.assertEqual(result.returncode, 2)
                self.assertIn(
                    "--reference-speed must be finite and in (0, 1]",
                    result.stderr,
                )


    def test_home_wrapper_runs_previous_speed_dual_arm_recovery(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bin_dir = root / "bin"
            bin_dir.mkdir()
            capture = root / "pixi-args.txt"
            pixi = bin_dir / "pixi"
            pixi.write_text(
                "#!/bin/sh\nprintf '%s\\n' \"$@\" > \"$CAPTURE\"\n",
                encoding="utf-8",
            )
            pixi.chmod(0o755)
            env = {
                **os.environ,
                "PATH": f"{bin_dir}:{os.environ['PATH']}",
                "CAPTURE": str(capture),
            }
            result = subprocess.run(
                ["bash", str(ROOT / "home.sh")],
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

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            arguments,
            [
                "run",
                "bash",
                "scripts/return_home.sh",
                "--confirm-real",
                "--side",
                "both",
                "--recover-outside-limits",
            ],
        )



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

    def _run_session_until_components_are_wired(
        self, profile: str, *display_args: str
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
                    "TIANJI_REAL_PREFLIGHT_FD": "9",
                }
            )
            arguments = [
                str(SCRIPTS / "run_session.sh"),
                "--profile",
                profile,
            ]
            if profile.startswith("h5_"):
                arguments.extend(("--h5", str(h5_path)))
            if profile.endswith("_real"):
                arguments.append("--confirm-real")
            arguments.extend(display_args)
            result = subprocess.run(
                arguments,
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
                launches = self._run_session_until_components_are_wired(
                    "h5_sim", *display_args
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

    def test_regrind_direct_path_launches_dedicated_config_under_shared_ik_authority(
        self,
    ) -> None:
        launches = self._run_session_until_components_are_wired("regrind_real")
        arm_producer = next(
            args
            for args in launches
            if any(arg.endswith("/run_producer.sh") for arg in args)
        )
        self.assertEqual(
            arm_producer[arm_producer.index("--producer") + 1],
            "ik",
        )
        self.assertTrue(
            arm_producer[arm_producer.index("--config") + 1].endswith(
                "/producers/ik_regrind.yaml"
            )
        )
        self.assertIn(
            "TIANJI_ARM_PRODUCER_LOGICAL_ID=arm_ik_producer",
            arm_producer,
        )
        authorities = next(
            arg.removeprefix("TIANJI_AUTHORITIES=")
            for arg in arm_producer
            if arg.startswith("TIANJI_AUTHORITIES=")
        )
        self.assertEqual(
            __import__("json").loads(authorities)["producer_arm"]["logical_id"],
            "arm_ik_producer",
        )


if __name__ == "__main__":
    unittest.main()
