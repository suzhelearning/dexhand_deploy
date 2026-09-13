from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
BUNDLE = ROOT / "vendor" / "pico_tracker"


def _load_script(name: str):
    path = ROOT / "scripts" / name
    spec = importlib.util.spec_from_file_location(path.stem, path)
    if spec is None or spec.loader is None:
        raise AssertionError(f"unable to load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class EmbeddedPicoRuntimeTest(unittest.TestCase):
    def test_legacy_scripts_use_portable_calibration_override(self):
        driver = (BUNDLE / "scripts" / "start_pico_driver.sh").read_text()
        m0 = (BUNDLE / "scripts" / "start_pico_m0.sh").read_text()
        calibration = (BUNDLE / "scripts" / "calibrate_pico_arm.sh").read_text()
        orientation = (BUNDLE / "scripts" / "calibrate_pico_palm_orientation.sh").read_text()

        self.assertIn("PICO_TRACKER_CONFIG_DIR", driver)
        self.assertIn("PICO_TRACKER_CONFIG_DIR", m0)
        self.assertIn("PICO_TRACKER_CONFIG_DIR", calibration)
        self.assertIn("PICO_TRACKER_CONFIG_DIR", orientation)
        self.assertIn("ADB_SERIAL", driver)
        self.assertIn("pico_left_arm_geometry.yaml", m0)
        self.assertIn("pico_right_arm_geometry.yaml", m0)
        self.assertNotIn("/home/zj/current_robotics/", driver + m0 + calibration + orientation)

    def test_embedded_supervisor_rejects_zero_tjvr_port_before_adb(self):
        result = subprocess.run(
            [
                "bash",
                str(ROOT / "scripts" / "run_embedded_pico_vr_session.sh"),
                "--profile",
                "pico_vr_manus_sim",
                "--disable-hands",
                "--tjvr-port",
                "0",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("1..65535", result.stderr)
        self.assertNotIn("adb", result.stderr.lower())

    def test_preflight_validates_ros_domain_before_importing_ros(self):
        path = ROOT / "scripts" / "embedded_pico_preflight.py"
        source = path.read_text()
        self.assertIn("ROS_DOMAIN_ID", source)
        self.assertIn("0 <= domain <= 232", source)
        self.assertIn("import rclpy", source)
        self.assertGreater(source.index("import rclpy"), source.index("0 <= domain <= 232"))
        invalid_domain = subprocess.run(
            [sys.executable, str(path), "--mode", "raw", "--domain", "233"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(invalid_domain.returncode, 2)
        self.assertNotIn("rclpy", invalid_domain.stderr)
        invalid_mode = subprocess.run(
            [sys.executable, str(path), "--mode", "invalid"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(invalid_mode.returncode, 2)
        self.assertNotIn("No module named", invalid_mode.stderr)

    def test_preflight_topic_sets_match_component_ownership(self):
        module = _load_script("embedded_pico_preflight.py")
        self.assertEqual(module.DRIVER_TOPICS, ("/pico/smpl_raw",))
        self.assertEqual(
            module.M0_TOPICS,
            (
                "/pico/smpl_raw",
                "/pico/palm_left",
                "/pico/palm_right",
                "/pico/smpl_palm_corrected",
                "/pico/smpl_palm_corrected_ik",
                "/pico/smpl_palm_corrected/status",
                "/pico/tracking_epoch",
                "/pico/tracking_epoch/status",
            ),
        )

    def test_supervisor_scopes_preflight_environment_and_lock(self):
        source = (ROOT / "scripts" / "run_embedded_pico_vr_session.sh").read_text()
        self.assertIn('env "${runtime_env[@]}" "$pixi_bin" run', source)
        self.assertIn('acquire_teleop_guard pico_vr_manus_sim', source)
        self.assertIn('embedded_lock_path="$runtime_base/embedded-pico.lock"', source)
        self.assertIn('flock -n "$embedded_lock_path" -c true', source)
        self.assertNotIn('embedded_lock_fd', source)
        self.assertNotIn('flock -n "$embedded_lock_fd"', source)

    def test_manifest_generator_is_deterministic_and_relative(self):
        generator = _load_script("generate_embedded_pico_manifest.py")
        with tempfile.TemporaryDirectory() as directory:
            source_root = Path(directory) / "source"
            (source_root / "src" / "pkg").mkdir(parents=True)
            (source_root / "src" / "pkg" / "b.txt").write_text("b\n")
            (source_root / "src" / "pkg" / "a.txt").write_text("a\n")
            (source_root / ".pixi").mkdir()
            (source_root / ".pixi" / "ignored.txt").write_text("ignored\n")
            output = source_root / "source_manifest.json"

            self.assertEqual(
                generator.main(
                    [
                        "--source-root",
                        str(source_root),
                        "--output",
                        str(output),
                        "--source-commit",
                        "test-commit",
                    ]
                ),
                0,
            )
            first = output.read_text()
            self.assertEqual(
                generator.main(
                    [
                        "--source-root",
                        str(source_root),
                        "--output",
                        str(output),
                        "--source-commit",
                        "test-commit",
                    ]
                ),
                0,
            )
            self.assertEqual(output.read_text(), first)

            manifest = json.loads(first)
            self.assertEqual(manifest["source_commit"], "test-commit")
            paths = [entry["path"] for entry in manifest["files"]]
            self.assertEqual(paths, ["src/pkg/a.txt", "src/pkg/b.txt"])
            self.assertTrue(all(not Path(path).is_absolute() and ".." not in Path(path).parts for path in paths))

    def test_manifest_generator_rejects_output_outside_source(self):
        generator = _load_script("generate_embedded_pico_manifest.py")
        with tempfile.TemporaryDirectory() as directory:
            source_root = Path(directory) / "source"
            source_root.mkdir()
            with self.assertRaisesRegex(ValueError, "inside source root"):
                generator.main(
                    [
                        "--source-root",
                        str(source_root),
                        "--output",
                        str(Path(directory) / "outside.json"),
                    ]
                )
