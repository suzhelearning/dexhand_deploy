from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).parents[1]
RUN_CASE = ROOT / "scripts" / "validation" / "run_case.py"
ANALYZE = ROOT / "scripts" / "validation" / "analyze_runs.py"
MATRIX = ROOT / "src" / "pico_body_tianji" / "config" / "validation" / "test_matrix.yaml"


CASE_IDS = {
    "acquisition_live", "pico_sim", "mocap_live_sim", "h5_sim",
    "ik_pinocchio_cpp", "ik_pinocchio_qp", "ik_tianji_official",
    "target_replay_sim", "joint_replay_sim", "policy_hold_sim",
    "marvin_pico_real_10pct", "marvin_mocap_live_real_10pct",
    "marvin_h5_real_10pct", "wuji_retarget_dry", "wuji_retarget_real",
    "wuji_direct_real", "fault_recovery_sim", "fault_recovery_real",
}


class ValidationToolsTest(unittest.TestCase):
    def test_matrix_is_machine_readable_and_has_fixed_cases(self):
        matrix = yaml.safe_load(MATRIX.read_text(encoding="utf-8"))
        self.assertEqual(set(matrix), {"schema_name", "schema_version", "cases"})
        self.assertEqual(matrix["schema_name"], "tianji-validation-matrix")
        self.assertEqual(matrix["schema_version"], "1.0")
        self.assertEqual(set(matrix["cases"]), CASE_IDS)
        required = {
            "profile", "required_devices", "required_capability", "active_sides",
            "hand_mode", "velocity_ratio", "acceleration_ratio", "max_duration_s",
            "prerequisites", "stop_criteria",
        }
        for case in matrix["cases"].values():
            self.assertEqual(set(case), required)
            self.assertIn(case["required_capability"], {"simulation", "real"})
            self.assertGreater(case["max_duration_s"], 0)
            self.assertTrue(case["stop_criteria"])

    def test_list_prints_all_case_ids(self):
        result = subprocess.run(
            [sys.executable, str(RUN_CASE), "--list"],
            cwd=ROOT, text=True, capture_output=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(set(result.stdout.split()), CASE_IDS)

    def test_fake_headless_bundle_is_safe_and_analyzable(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            result = subprocess.run(
                [sys.executable, str(RUN_CASE), "--case", "pico_sim", "--output", str(root), "--fake", "--headless"],
                cwd=ROOT, text=True, capture_output=True,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            bundle = next(root.iterdir())
            for name in ("manifest.yaml", "session.h5", "status.jsonl", "operator_events.jsonl", "liveliness.jsonl", "protocol.jsonl", "operator_result.yaml", "checksums.sha256"):
                self.assertTrue((bundle / name).is_file(), name)
            operator = yaml.safe_load((bundle / "operator_result.yaml").read_text())
            self.assertEqual(operator["outcome"], "aborted")
            analysis = subprocess.run(
                [sys.executable, str(ANALYZE), str(root)], cwd=ROOT, text=True, capture_output=True,
            )
            self.assertEqual(analysis.returncode, 0, analysis.stderr)
            self.assertTrue((bundle / "analysis.json").is_file())
            self.assertTrue((bundle / "analysis.md").is_file())

    def test_analyzer_rejects_checksum_schema_status_and_manifest_hash_tampering(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            result = subprocess.run(
                [sys.executable, str(RUN_CASE), "--case", "pico_sim", "--output", str(root), "--fake", "--headless"],
                cwd=ROOT, text=True, capture_output=True,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            bundle = next(root.iterdir())
            checks = [
                ("checksums.sha256", lambda p: p.write_text(p.read_text() + "bad\n")),
                ("manifest.yaml", lambda p: p.write_text(p.read_text().replace("tianji-validation-run", "wrong-schema", 1))),
                ("status.jsonl", lambda p: p.write_text("{\"schema_version\": 999}\n")),
            ]
            for name, mutate in checks:
                original = (bundle / name).read_bytes()
                mutate(bundle / name)
                failed = subprocess.run([sys.executable, str(ANALYZE), str(root)], cwd=ROOT, text=True, capture_output=True)
                self.assertNotEqual(failed.returncode, 0, name)
                (bundle / name).write_bytes(original)

            manifest = yaml.safe_load((bundle / "manifest.yaml").read_text())
            manifest["hashes"]["config_sha256"] = "0" * 64
            (bundle / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))
            failed = subprocess.run([sys.executable, str(ANALYZE), str(root)], cwd=ROOT, text=True, capture_output=True)
            self.assertNotEqual(failed.returncode, 0)

    def test_real_case_requires_confirmation_and_prerequisites(self):
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run(
                [sys.executable, str(RUN_CASE), "--case", "marvin_pico_real_10pct", "--output", directory],
                cwd=ROOT, text=True, capture_output=True,
            )
            self.assertEqual(result.returncode, 2)
            self.assertIn("--confirm-real", result.stderr)

    def test_safety_supervisor_rejects_missing_ack(self):
        from scripts.validation.run_case import SafetyStopSupervisor

        supervisor = SafetyStopSupervisor("run", "supervisor", "router", clock=lambda: 10)
        result = supervisor.issue("collision_risk", ["arm", "hand"], publish=lambda request: None, wait_ack=lambda _: {})
        self.assertFalse(result.accepted)
        self.assertIn("ack", result.reason)
        self.assertTrue(result.locked)

    def test_manifest_instance_ids_are_exported_for_session_handoff(self):
        from scripts.validation.run_case import build_parser, load_matrix, _build_manifest, instance_handoff_environment
        args = build_parser().parse_args(["--case", "pico_sim", "--headless"])
        case = load_matrix()["cases"]["pico_sim"]
        manifest = _build_manifest("pico_sim", case, "pico_sim", "run-id", "supervisor-id", "router-id", "started", args)
        env = instance_handoff_environment(manifest)
        self.assertEqual(env["TIANJI_RUN_ID"], "run-id")
        self.assertEqual(env["TIANJI_SOURCE_INSTANCE_ID"], manifest["publisher_instance_ids"]["source"])
        self.assertEqual(env["TIANJI_ARM_PRODUCER_INSTANCE_ID"], manifest["publisher_instance_ids"]["producer_arm"])
        self.assertEqual(env["TIANJI_ARM_EXECUTOR_INSTANCE_ID"], manifest["publisher_instance_ids"]["executor_arm"])
        self.assertEqual(env["TIANJI_COORDINATOR_INSTANCE_ID"], manifest["publisher_instance_ids"]["coordinator_arm"])
    def test_session_launcher_consumes_handoff_ids(self):
        launcher = (ROOT / "scripts" / "run_session.sh").read_text(encoding="utf-8")
        for expression in (
            'TIANJI_RUN_ID:-$(new_instance_id)',
            'TIANJI_COORDINATOR_INSTANCE_ID:-$(new_instance_id)',
            'TIANJI_SOURCE_INSTANCE_ID:-$(new_instance_id)',
            'TIANJI_ARM_PRODUCER_INSTANCE_ID:-$(new_instance_id)',
            'TIANJI_ARM_EXECUTOR_INSTANCE_ID:-$(new_instance_id)',
            'TIANJI_HAND_PRODUCER_INSTANCES:-',
            'TIANJI_HAND_EXECUTOR_INSTANCES:-',
            'TIANJI_RECORDER_INSTANCE_ID:-$(new_instance_id)',
        ):
            self.assertIn(expression, launcher)
    def test_case_contract_routes_profile_backend_and_rejects_conflicting_backend(self):
        from scripts.validation.run_case import build_session_contract

        command = build_session_contract("ik_pinocchio_qp", {"ik_backend": None})
        self.assertEqual(command["profile"], "pico_sim")
        self.assertEqual(command["producer"], "ik")
        self.assertEqual(command["ik_backend"], "pinocchio_qp")
        with self.assertRaises(ValueError):
            build_session_contract("ik_pinocchio_qp", {"ik_backend": "pinocchio_cpp"})

    def test_operator_finalization_rejects_collision_pass(self):
        from scripts.validation.run_case import validate_operator_finalization

        with self.assertRaises(ValueError):
            validate_operator_finalization("pass", ["collision_risk"], rc=0)
        self.assertEqual(validate_operator_finalization("fail", ["collision_risk"], rc=1), "fail")

    def test_acquisition_observation_requires_samples_and_tracks_instance(self):
        from scripts.validation.run_case import AlignedStreamObservation

        observation = AlignedStreamObservation()
        self.assertFalse(observation.complete)
        observation.accept({"stream_instance_id": "stream-a", "stream_sequence": 1, "router_zid": "router", "left_valid": True, "right_valid": False})
        observation.accept({"stream_instance_id": "stream-a", "stream_sequence": 2, "router_zid": "router", "left_valid": True, "right_valid": False})
        self.assertTrue(observation.complete)
        self.assertEqual(observation.samples, 2)
        self.assertFalse(observation.accept({"stream_instance_id": "stream-b", "stream_sequence": 1, "router_zid": "router", "left_valid": True, "right_valid": False}))


if __name__ == "__main__":
    unittest.main()
