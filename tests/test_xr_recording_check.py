from dataclasses import replace
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import h5py

from tianji_teleop.recording.session_h5 import SessionH5Reader, SessionH5Writer
from tianji_teleop.hand_tracking.operator_input import OperatorObservation
from tianji_teleop.hand_tracking.xr_operator import encode_xr_operator_observation
from tianji_teleop.recording.xr_check import check_xr_recording
from tests.test_xr_recording import _xr_frame


class XrRecordingCheckTest(unittest.TestCase):
    def write_recording(self, path: Path) -> None:
        first = _xr_frame()
        second = replace(
            first,
            sequence=5,
            received_timestamp_ns=2001,
            source_timestamp_ns=1235,
        )
        with SessionH5Writer(
            path,
            source_type="vr_manus_xr_sim",
            robot_model="marvin",
            router_zid="router",
            schema_version="1.2",
        ) as writer:
            writer.append_raw_xr(first)
            writer.append_raw_xr(second)
            writer.append_dual_audit(
                "operator_observation",
                encode_xr_operator_observation(
                    OperatorObservation(
                        source="xr-observation",
                        side="right",
                        sequence=first.sequence,
                        epoch=first.connection_generation,
                        receive_time_ns=first.received_timestamp_ns,
                        valid=True,
                        available=True,
                        action="start_request",
                        pressed=False,
                        confidence=1.0,
                    ),
                    router_zid="router",
                    publisher_instance_id="xr-observation",
                ),
                received_timestamp_ns=first.received_timestamp_ns,
            )

    def test_xr_recording_check_matches_raw_columns_and_json_payload(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "xr.h5"
            self.write_recording(path)
            report = check_xr_recording(path)
            self.assertTrue(report["passed"], report)
            self.assertEqual(report["raw_frames"], 2)
            self.assertEqual(report["connection_generations"], [2])
            self.assertEqual(report["operator_observations"], 1)
            self.assertEqual(report["operator_events_executed"], 0)

    def test_xr_recording_check_rejects_corrupt_operator_observation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "xr.h5"
            self.write_recording(path)
            with h5py.File(path, "r+") as handle:
                payload = json.loads(handle["meta/dual_audit/payload_json"][0])
                payload["observation"]["epoch"] = 99
                handle["meta/dual_audit/payload_json"][0] = json.dumps(
                    payload, separators=(",", ":")
                )
            report = check_xr_recording(path)
            self.assertFalse(report["passed"])
            self.assertEqual(report["first_difference"]["field"], "operator_observation")

    def test_xr_recording_check_rejects_operator_sequence_without_raw_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "xr.h5"
            self.write_recording(path)
            with h5py.File(path, "r+") as handle:
                payload = json.loads(handle["meta/dual_audit/payload_json"][0])
                payload["observation"]["sequence"] = 999
                handle["meta/dual_audit/payload_json"][0] = json.dumps(
                    payload, separators=(",", ":")
                )
            report = check_xr_recording(path)
            self.assertFalse(report["passed"])
            self.assertEqual(report["first_difference"]["field"], "operator_observation")

    def test_xr_recording_check_rejects_column_payload_divergence(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "xr.h5"
            self.write_recording(path)
            with h5py.File(path, "r+") as handle:
                handle["raw/xr_input/right_grip"][0] = 0.1
            report = check_xr_recording(path)
            self.assertFalse(report["passed"])
            self.assertEqual(report["first_difference"]["field"], "right_grip")

    def test_xr_recording_check_covers_source_clock_columns(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "xr.h5"
            self.write_recording(path)
            with h5py.File(path, "r+") as handle:
                handle["raw/xr_input/source_time_ns"][0] += 1
            report = check_xr_recording(path)
            self.assertFalse(report["passed"])
            self.assertEqual(report["first_difference"]["field"], "source_time_ns")

    def test_xr_recording_check_requires_explicit_connection_generation_in_json(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "xr.h5"
            self.write_recording(path)
            with h5py.File(path, "r+") as handle:
                frame = json.loads(handle["raw/xr_input/frame_json"][0])
                frame.pop("connection_generation")
                handle["raw/xr_input/connection_generation"][0] = 1
                handle["raw/xr_input/frame_json"][0] = json.dumps(frame, separators=(",", ":"))
            report = check_xr_recording(path)
            self.assertFalse(report["passed"])
            self.assertEqual(
                report["first_difference"]["field"],
                "frame_json.connection_generation",
            )

    def test_xr_recording_check_rejects_generation_rollback(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "xr.h5"
            self.write_recording(path)
            with h5py.File(path, "r+") as handle:
                handle["raw/xr_input/connection_generation"][1] = 1
            with self.assertRaisesRegex(ValueError, "generation"):
                check_xr_recording(path)

    def test_dual_recording_cli_exposes_xr_mode(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "xr.h5"
            self.write_recording(path)
            result = subprocess.run(
                [sys.executable, str(Path(__file__).parents[1] / "scripts/check_dual_recording.py"),
                 "--mode", "xr", "--input", str(path)],
                env=dict(os.environ, PYTHONPATH="src/tianji_teleop"),
                capture_output=True,
                text=True,
                timeout=10,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(json.loads(result.stdout)["passed"])


if __name__ == "__main__":
    unittest.main()
