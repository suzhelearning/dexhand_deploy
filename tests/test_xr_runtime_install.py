import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class XrRuntimeInstallTest(unittest.TestCase):
    def test_build_script_help_describes_portable_xr_runtime(self):
        result = subprocess.run(
            ["bash", str(ROOT / "scripts/build_xr_sdk.sh"), "--help"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--source", result.stdout)
        self.assertIn("--library-dir", result.stdout)
        self.assertIn("--output-dir", result.stdout)

    def test_build_script_rejects_missing_source_before_build(self):
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run(
                [
                    "bash",
                    str(ROOT / "scripts/build_xr_sdk.sh"),
                    "--source",
                    str(Path(directory) / "missing"),
                ],
                capture_output=True,
                text=True,
                timeout=5,
            )
        self.assertEqual(result.returncode, 2)
        self.assertIn("source", result.stderr.lower())

    def test_xr_adb_reverse_maps_only_xr_ports(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            log_path = directory / "adb.log"
            adb_path = directory / "adb"
            adb_path.write_text(
                """#!/usr/bin/env bash
set -eu
printf '%s\\n' \"$*\" >> \"$FAKE_ADB_LOG\"
if [[ \"${1:-}\" == devices ]]; then
  printf 'List of devices attached\\nPICO-TEST\\tdevice usb:1-1\\n'
fi
""",
                encoding="utf-8",
            )
            adb_path.chmod(adb_path.stat().st_mode | stat.S_IXUSR)
            result = subprocess.run(
                ["bash", str(ROOT / "scripts/xr_adb_reverse.sh"), "up"],
                env=dict(os.environ, ADB=str(adb_path), FAKE_ADB_LOG=str(log_path)),
                capture_output=True,
                text=True,
                timeout=5,
            )
            calls = log_path.read_text(encoding="utf-8").splitlines()

        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue(
            any("reverse tcp:60061 tcp:60061" in call for call in calls)
        )
        self.assertTrue(
            any("reverse tcp:63901 tcp:63901" in call for call in calls)
        )
        self.assertNotIn("10002", "\\n".join(calls))

    def test_xr_adb_reverse_applies_auto_discovered_serial_to_reverse(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            log_path = directory / "adb.log"
            adb_path = directory / "adb"
            adb_path.write_text(
                """#!/usr/bin/env bash
set -eu
printf '%s\\n' "$*" >> "$FAKE_ADB_LOG"
if [[ "${1:-}" == devices ]]; then
  printf 'List of devices attached\\nPICO-FIRST\\tdevice usb:1-1\\nPICO-SECOND\\tdevice usb:2-1\\n'
fi
""",
                encoding="utf-8",
            )
            adb_path.chmod(adb_path.stat().st_mode | stat.S_IXUSR)
            result = subprocess.run(
                ["bash", str(ROOT / "scripts/xr_adb_reverse.sh"), "up"],
                env=dict(os.environ, ADB=str(adb_path), FAKE_ADB_LOG=str(log_path)),
                capture_output=True,
                text=True,
                timeout=5,
            )
            calls = log_path.read_text(encoding="utf-8").splitlines()

        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("-s PICO-FIRST reverse tcp:60061 tcp:60061", calls)
        self.assertIn("-s PICO-FIRST reverse tcp:63901 tcp:63901", calls)

    def test_launcher_can_auto_discover_ignored_xr_runtime(self):
        script = (ROOT / "scripts/run_session.sh").read_text(encoding="utf-8")
        self.assertIn("vendor/xr_sdk/python", script)
        self.assertIn("vendor/xr_sdk/lib", script)
        self.assertIn("xr_sdk_pythonpath", script)
        self.assertIn("xr_sdk_library_dir", script)

    def test_pc_service_launcher_help_describes_portable_root(self):
        result = subprocess.run(
            ["bash", str(ROOT / "scripts/run_xr_pc_service.sh"), "--help"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--root", result.stdout)
        self.assertIn("--check", result.stdout)

    def test_pc_service_launcher_executes_foreground_with_scoped_runtime(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "roboticsservice"
            binary = root / "RoboticsServiceProcess"
            library = root / "SDK" / "x64" / "libPXREARobotSDK.so"
            library.parent.mkdir(parents=True)
            binary.write_text(
                "#!/usr/bin/env bash\nprintf 'service_ld=%s\\n' \"$LD_LIBRARY_PATH\"\n",
                encoding="utf-8",
            )
            binary.chmod(binary.stat().st_mode | stat.S_IXUSR)
            library.touch()
            result = subprocess.run(
                ["bash", str(ROOT / "scripts/run_xr_pc_service.sh"), "--root", str(root)],
                capture_output=True,
                text=True,
                timeout=5,
            )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(str(root), result.stdout)

    def test_pc_service_launcher_rejects_missing_root_before_start(self):
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run(
                [
                    "bash",
                    str(ROOT / "scripts/run_xr_pc_service.sh"),
                    "--root",
                    str(Path(directory) / "missing"),
                    "--check",
                ],
                capture_output=True,
                text=True,
                timeout=5,
            )
        self.assertEqual(result.returncode, 2)
        self.assertIn("PC-Service", result.stderr)


if __name__ == "__main__":
    unittest.main()
