import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


_API = (
    "init",
    "get_headset_pose",
    "get_left_controller_pose",
    "get_right_controller_pose",
    "num_motion_data_available",
    "get_motion_tracker_pose",
    "get_motion_tracker_serial_numbers",
    "get_left_trigger",
    "get_right_trigger",
    "get_left_grip",
    "get_right_grip",
    "get_left_axis",
    "get_right_axis",
    "get_motion_timestamp_ns",
)


def _fake_sdk_source(*, missing: str | None = None) -> str:
    lines = []
    for name in _API:
        if name == missing:
            continue
        lines.append(f"def {name}(*args):\n    return None\n")
    return "\n".join(lines)


class XrSdkPreflightTest(unittest.TestCase):
    def run_preflight(self, module_source: str, arm_input: str | None = None):
        with tempfile.TemporaryDirectory() as directory:
            module = Path(directory) / "xrobotoolkit_sdk.py"
            module.write_text(module_source, encoding="utf-8")
            environment = dict(os.environ, TIANJI_XR_SDK_PYTHONPATH=directory)
            command = [sys.executable, str(ROOT / "scripts/check_xr_sdk.py")]
            if arm_input is not None:
                command += ["--arm-input", arm_input]
            return subprocess.run(
                command,
                env=environment,
                capture_output=True,
                text=True,
                timeout=10,
            )

    def test_preflight_accepts_reference_api_without_calling_init(self):
        result = self.run_preflight(_fake_sdk_source())
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report["module"], "xrobotoolkit_sdk")
        self.assertEqual(report["missing"], [])

    def test_preflight_rejects_incomplete_api_before_runtime(self):
        result = self.run_preflight(_fake_sdk_source(missing="get_right_axis"))
        self.assertEqual(result.returncode, 1)
        self.assertIn("get_right_axis", result.stderr)

    def test_controller_only_preflight_accepts_sdk_without_tracker_symbols(self):
        tracker_symbols = {
            "num_motion_data_available",
            "get_motion_tracker_pose",
            "get_motion_tracker_serial_numbers",
        }
        source = _fake_sdk_source()
        for symbol in tracker_symbols:
            source = source.replace(
                f"def {symbol}(*args):\n    return None\n", ""
            )
        result = self.run_preflight(source, arm_input="xr_controller")
        self.assertEqual(result.returncode, 0, result.stderr)

        tracker_result = self.run_preflight(source, arm_input="xr_tracker")
        self.assertEqual(tracker_result.returncode, 1)
        self.assertIn("num_motion_data_available", tracker_result.stderr)


if __name__ == "__main__":
    unittest.main()
