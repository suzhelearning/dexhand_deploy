from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import yaml

ROOT = Path(__file__).parents[1]
RUN_CASE = ROOT / "scripts" / "validation" / "run_case.py"
ANALYZE = ROOT / "scripts" / "validation" / "analyze_runs.py"
MATRIX = ROOT / "src" / "tianji_teleop" / "config" / "validation" / "test_matrix.yaml"


CASE_IDS = {
    "acquisition_live", "mocap_live_sim", "h5_sim",
    "ik_pinocchio_cpp", "ik_pinocchio_qp", "ik_tianji_official",
    "target_replay_sim", "joint_replay_sim", "policy_hold_sim",
    "marvin_mocap_live_real_10pct",
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
                [sys.executable, str(RUN_CASE), "--case", "mocap_live_sim", "--output", str(root), "--fake", "--headless"],
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
                [sys.executable, str(RUN_CASE), "--case", "mocap_live_sim", "--output", str(root), "--fake", "--headless"],
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
                [sys.executable, str(RUN_CASE), "--case", "marvin_mocap_live_real_10pct", "--output", directory],
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
        args = build_parser().parse_args(["--case", "mocap_live_sim", "--headless"])
        case = load_matrix()["cases"]["mocap_live_sim"]
        manifest = _build_manifest("mocap_live_sim", case, "mocap_live_sim", "run-id", "supervisor-id", "router-id", "started", args)
        env = instance_handoff_environment(manifest)
        self.assertEqual(env["TIANJI_RUN_ID"], "run-id")
        self.assertEqual(env["TIANJI_SOURCE_INSTANCE_ID"], manifest["publisher_instance_ids"]["source"])
        self.assertEqual(env["TIANJI_ARM_PRODUCER_INSTANCE_ID"], manifest["publisher_instance_ids"]["producer_arm"])
        self.assertEqual(env["TIANJI_ARM_EXECUTOR_INSTANCE_ID"], manifest["publisher_instance_ids"]["executor_arm"])
        self.assertEqual(env["TIANJI_COORDINATOR_INSTANCE_ID"], manifest["publisher_instance_ids"]["coordinator_arm"])

    def test_managed_h5_session_forwards_headless_only_when_requested(self):
        from scripts.validation import run_case

        for requested in (False, True):
            with self.subTest(headless=requested), tempfile.TemporaryDirectory() as directory:
                bundle = Path(directory)
                (bundle / "logs").mkdir()
                (bundle / "session.h5").touch()
                cli = ["--case", "h5_sim", "--output", directory]
                if requested:
                    cli.append("--headless")
                args = run_case.build_parser().parse_args(cli)
                manifest = {
                    "profile": "h5_sim",
                    "case_id": "h5_sim",
                    "run_id": "run-id",
                    "router_zid": "router-id",
                    "router": {"endpoint": "tcp/127.0.0.1:7447"},
                    "publisher_instance_ids": {
                        "validation_supervisor": "supervisor-id",
                    },
                    "resolved_hand_mode": "retarget",
                    "ik_backend": "pinocchio_cpp",
                    "producer": "ik",
                    "max_duration_s": 1.0,
                }
                process = mock.Mock(returncode=0)
                process.poll.return_value = 0
                with (
                    mock.patch.object(run_case, "_write_status"),
                    mock.patch.object(run_case, "_managed_stop", return_value=None),
                    mock.patch.object(run_case, "ManagedEvidenceCapture"),
                    mock.patch.object(
                        run_case.subprocess, "Popen", return_value=process
                    ) as popen,
                ):
                    self.assertEqual(
                        run_case._run_session(bundle, manifest, mock.Mock(), args),
                        0,
                    )
                command = popen.call_args.args[0]
                self.assertEqual(command.count("--headless"), int(requested))

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
        self.assertIn('"TIANJI_COORDINATOR_INSTANCE_ID=${coordinator_id}"', launcher)
    def test_case_contract_routes_profile_backend_and_rejects_conflicting_backend(self):
        from scripts.validation.run_case import build_session_contract

        command = build_session_contract("ik_pinocchio_qp", {"ik_backend": None})
        self.assertEqual(command["profile"], "mocap_live_sim")
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

    def test_hand_case_profiles_and_retarget_contract_are_strict(self):
        matrix = yaml.safe_load(MATRIX.read_text(encoding="utf-8"))["cases"]
        self.assertEqual(matrix["wuji_direct_real"]["profile"], "wuji_direct_real")
        self.assertEqual(matrix["wuji_direct_real"]["required_capability"], "real")
        self.assertIn("marvin_arm", matrix["wuji_direct_real"]["required_devices"])
        self.assertIn("marvin_mocap_live_real_10pct", matrix["wuji_direct_real"]["prerequisites"])
        self.assertEqual(matrix["wuji_retarget_dry"]["hand_mode"], "retarget")
        self.assertEqual(matrix["wuji_retarget_real"]["hand_mode"], "retarget")
        from scripts.validation.run_case import build_session_contract
        direct = build_session_contract("wuji_direct_real")
        self.assertFalse(direct["recordable"])
        self.assertEqual(direct["hand_mode"], "direct")
        self.assertEqual(direct["source_capability"], "real")
        self.assertEqual(build_session_contract("policy_hold_sim")["producer"], "policy_hold")
        self.assertEqual(build_session_contract("ik_pinocchio_qp")["ik_backend"], "pinocchio_qp")

    def test_safety_ack_requires_executor_envelope_identity(self):
        from scripts.validation.run_case import SafetyStopSupervisor
        from tianji_teleop.protocol.messages import ProtocolEnvelope

        supervisor = SafetyStopSupervisor("run", "supervisor", "router", clock=lambda: 10)
        result = supervisor.issue(
            "collision_risk",
            ["arm"],
            publish=lambda request: None,
            wait_ack=lambda request: {
                "arm": {
                    "schema_version": 1,
                    "publisher_instance_id": "other",
                    "router_zid": "router",
                    "sequence": request.envelope.sequence,
                    "timestamp_ns": 11,
                    "executor_id": "arm",
                    "run_id": "run",
                    "latched": True,
                    "reason": "collision_risk",
                }
            },
        )
        self.assertFalse(result.accepted)
        self.assertTrue(result.locked)

    def test_sequence_metric_reports_duplicate_and_rollback(self):
        from scripts.validation.analyze_runs import _sequence_metric

        metric = _sequence_metric([1, 2, 2, 1, 3], ["instance"] * 5)
        self.assertEqual(metric["drops"], 0)
        self.assertEqual(metric["order_errors"], 2)

    def test_analyzer_evidence_is_case_specific(self):
        from scripts.validation.analyze_runs import REQUIRED_EVIDENCE

        self.assertNotIn("state_arm", REQUIRED_EVIDENCE["acquisition_live"]["streams"])
        self.assertNotIn("home_feedback", REQUIRED_EVIDENCE["target_replay_sim"]["checks"])
        self.assertIn("target_to_solved", REQUIRED_EVIDENCE["target_replay_sim"]["checks"])
        self.assertIn("home_feedback", REQUIRED_EVIDENCE["mocap_live_sim"]["checks"])
    def test_target_replay_cli_omits_joint_only_capabilities(self):
        from unittest.mock import Mock, patch
        from tianji_teleop.recording import replay_cli

        session = Mock()
        node = Mock()
        node.start.side_effect = KeyboardInterrupt
        with patch.dict("os.environ", {
            "TIANJI_COMPONENT_INSTANCE_ID": "source",
            "TIANJI_COORDINATOR_INSTANCE_ID": "coord",
        }, clear=False), patch.object(replay_cli, "open_session", return_value=session), patch.object(
            replay_cli, "require_single_router", return_value="router"
        ), patch.object(replay_cli, "TargetReplaySource", return_value=node) as constructor:
            self.assertEqual(replay_cli.main(["target", "recording.h5", "--headless"]), 0)
        self.assertNotIn("capabilities", constructor.call_args.kwargs)

    def test_capture_rejects_missing_protocol_envelope(self):
        import json
        from types import SimpleNamespace
        from scripts.validation.run_case import ManagedEvidenceCapture

        capture = object.__new__(ManagedEvidenceCapture)
        capture.run_id = "run"
        captured = []
        capture._append = lambda filename, row: captured.append((filename, row))
        sample = SimpleNamespace(
            key_expr="tianji/source/status",
            payload=json.dumps({"component_role": "source"}).encode(),
        )
        capture._on_data(sample)
        self.assertEqual(captured, [])

    def test_authority_uses_latest_valid_sequence_and_side_less_hand_status(self):
        from scripts.validation.analyze_runs import _authority_statuses

        hand_authorities = [
            {
                "component_role": "producer_hand",
                "logical_id": "wuji_retarget_left",
                "side": side,
                "publisher_instance_id": "hand",
                "router_zid": "router",
            }
            for side in ("left", "right")
        ]
        manifest = {
            "router_zid": "router",
            "authority_contract": [
                {
                    "component_role": "coordinator_arm",
                    "logical_id": "arm",
                    "side": None,
                    "publisher_instance_id": "coord",
                    "router_zid": "router",
                },
                *hand_authorities,
            ],
        }
        statuses = [
            {
                "component_role": "coordinator_arm",
                "component_id": "arm",
                "publisher_instance_id": "coord",
                "router_zid": "router",
                "sequence": 1,
                "ready": False,
                "healthy": True,
                "phase": "startup",
            },
            {
                "component_role": "coordinator_arm",
                "component_id": "arm",
                "publisher_instance_id": "coord",
                "router_zid": "router",
                "sequence": 2,
                "ready": True,
                "healthy": True,
                "phase": "idle",
            },
            {
                "component_role": "producer_hand",
                "component_id": "wuji_retarget_left",
                "publisher_instance_id": "hand",
                "router_zid": "router",
                "sequence": 3,
                "ready": True,
                "healthy": True,
                "phase": "teleop",
            },
        ]
        found, unhealthy = _authority_statuses(statuses, manifest)
        self.assertEqual(found, {__import__("json").dumps(item, sort_keys=True) for item in manifest["authority_contract"]})
        self.assertEqual(unhealthy, [])

    def test_protocol_sequence_drop_merges_topics_per_instance(self):
        from scripts.validation.analyze_runs import _protocol_metrics

        rows = [
            {"topic": "tianji/source/status", "publisher_instance_id": "source", "sequence": 1},
            {
                "topic": "tianji/target/arm/left",
                "publisher_instance_id": "source",
                "sequence": 2,
                "side": "left",
                "position_m": [0, 0, 0],
                "orientation_xyzw": [0, 0, 0, 1],
                "elbow_reference_direction": [1, 0, 0],
            },
            {"topic": "tianji/source/status", "publisher_instance_id": "source", "sequence": 3},
        ]
        self.assertEqual(_protocol_metrics(rows)["protocol_drops"], 0)

    def test_aligned_observer_requires_continuous_multiframe_rate(self):
        from scripts.validation.run_case import AlignedStreamObservation

        observation = AlignedStreamObservation(min_samples=3, min_rate_hz=50.0)
        base = {"stream_instance_id": "stream", "router_zid": "router", "left_valid": True, "right_valid": False}
        observation.accept({**base, "stream_sequence": 1, "timestamp_ns": 0})
        self.assertFalse(observation.complete)
        observation.accept({**base, "stream_sequence": 2, "timestamp_ns": 16_666_667})
        self.assertFalse(observation.complete)
        observation.accept({**base, "stream_sequence": 3, "timestamp_ns": 33_333_334})
        self.assertTrue(observation.complete)
        self.assertGreaterEqual(observation.rate_hz, 50.0)
        self.assertFalse(observation.accept({**base, "stream_sequence": 5, "timestamp_ns": 50_000_000}))

    def test_target_solved_association_requires_authorized_instances_and_unique_key(self):
        from scripts.validation.analyze_runs import AnalysisError, _protocol_metrics

        manifest = {
            "publisher_instance_ids": {"source": "source", "producer_arm": "producer"},
            "authority_contract": [],
        }
        target = {
            "topic": "tianji/target/arm/left",
            "publisher_instance_id": "source",
            "sequence": 7,
            "side": "left",
            "position_m": [0, 0, 0],
            "orientation_xyzw": [0, 0, 0, 1],
            "elbow_reference_direction": [1, 0, 0],
        }
        solved = {
            "topic": "tianji/producer/arm/left/solved_pose",
            "publisher_instance_id": "producer",
            "sequence": 9,
            "target_sequence": 7,
            "side": "left",
            "position_m": [0, 0, 0],
            "orientation_xyzw": [0, 0, 0, 1],
        }
        unauthorized = dict(target, publisher_instance_id="other", position_m=[100, 0, 0])
        self.assertEqual(_protocol_metrics([unauthorized, target, solved], manifest)["target_to_solved_error"]["samples"], 1)
        with self.assertRaises(AnalysisError):
            _protocol_metrics([target, target, solved], manifest)

    def test_each_ik_case_requires_solved_pose_evidence(self):
        from scripts.validation.analyze_runs import REQUIRED_EVIDENCE

        for case_id in ("ik_pinocchio_cpp", "ik_pinocchio_qp", "ik_tianji_official"):
            self.assertIn("target_to_solved", REQUIRED_EVIDENCE[case_id]["checks"])

    def test_nonrecordable_replay_metrics_include_protocol_tracking(self):
        from scripts.validation.analyze_runs import _protocol_metrics

        rows = [
            {
                "topic": "tianji/command/arm/left",
                "publisher_instance_id": "coord",
                "sequence": 1,
                "side": "left",
                "timestamp_ns": 1_000_000_000,
                "position_rad": [0, 0, 0, 0, 0, 0, 0],
            },
            {
                "topic": "tianji/state/arm",
                "publisher_instance_id": "exec",
                "sequence": 1,
                "timestamp_ns": 1_000_001_000,
                "position_rad": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            },
        ]
        metrics = _protocol_metrics(rows)
        self.assertEqual(metrics["command_feedback_tracking"]["samples"], 1)
        self.assertEqual(metrics["command_feedback_tracking"]["max_error_rad"], 0.0)

    def test_fault_recovery_fake_stop_covers_every_executor_and_lockout(self):
        from scripts.validation.run_case import _authority_contract

        manifest = {
            "profile": "h5_sim",
            "producer": "ik",
            "resolved_hand_mode": "retarget",
            "router_zid": "router",
            "publisher_instance_ids": {
                "source": "source",
                "producer_arm": "arm-producer",
                "coordinator_arm": "coord",
                "executor_arm": "arm-executor",
                "hand_producer_instances": {"left": "hand-producer-left", "right": "hand-producer-right"},
                "hand_executor_instances": {"left": "hand-executor-left", "right": "hand-executor-right"},
            },
        }
        executors = {
            item["publisher_instance_id"]
            for item in _authority_contract(manifest)
            if item["component_role"].startswith("executor_")
        }
        self.assertEqual(
            executors,
            {"arm-executor", "hand-executor-left", "hand-executor-right"},
        )

    def test_direct_real_replay_preflight_rejects_missing_recording_before_connect(self):
        from tianji_teleop.recording.replay import validate_direct_real_recording

        with self.assertRaises(ValueError):
            validate_direct_real_recording("/tmp/not-a-session-v1.h5")

    def test_authority_capture_accepts_actual_coordinator_and_hand_executor_topics(self):
        from scripts.validation.analyze_runs import _authority_capture

        manifest = {
            "authority_contract": [
                {
                    "component_role": "coordinator_arm",
                    "logical_id": "arm",
                    "side": None,
                    "publisher_instance_id": "coord",
                    "router_zid": "router",
                    "topics": ["tianji/coordinator/status"],
                    "liveliness": "tj/live/coordinator/arm/arm/coord",
                },
                {
                    "component_role": "executor_hand",
                    "logical_id": "wuji_left",
                    "side": "left",
                    "publisher_instance_id": "hand",
                    "router_zid": "router",
                    "topics": ["tianji/executor/hand/{side}/status"],
                    "liveliness": "tj/live/executor/hand/wuji_left/hand",
                },
            ]
        }
        rows = [
            {"topic": "tianji/coordinator/status", "publisher_instance_id": "coord", "router_zid": "router"},
            {"topic": "tianji/executor/hand/left/status", "publisher_instance_id": "hand", "router_zid": "router", "side": "left"},
        ]
        found, live = _authority_capture(rows, [], manifest)
        self.assertEqual(len(found), 2)
        self.assertFalse(live)

    def test_managed_stop_waits_for_explicit_launcher_startup_signal(self):
        from scripts.validation.run_case import _wait_for_launcher_startup

        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "session.log"
            clock = [0.0]

            def now():
                return clock[0]

            def advance(seconds):
                clock[0] += seconds
                if clock[0] >= 6.0:
                    marker.write_text(
                        "session_startup_complete run_id=run; profile=h5_real\n",
                        encoding="utf-8",
                    )

            self.assertTrue(
                _wait_for_launcher_startup(
                    marker,
                    timeout_s=10.0,
                    clock=now,
                    sleep=advance,
                )
            )

    def test_real_capability_uses_sealed_attestation_and_scanner_binding(self):
        import fcntl
        import ctypes
        import hashlib
        import json
        import os
        import stat
        from types import SimpleNamespace
        from unittest.mock import patch

        from tianji_teleop.executors.marvin.preflight import trusted_real_capability

        scanner = json.dumps(
            {
                "scanner_id": "scanner-1",
                "capability": {
                    "speed": 0.25,
                    "yaw_deg": 0.0,
                    "deadman_available": True,
                    "preflight_passed": True,
                },
            },
            separators=(",", ":"),
            sort_keys=True,
        ).encode()
        libc = ctypes.CDLL(None, use_errno=True)
        libc.memfd_create.argtypes = [ctypes.c_char_p, ctypes.c_uint]
        libc.memfd_create.restype = ctypes.c_int
        scanner_fd = libc.memfd_create(b"scanner", 0x0002)
        attestation_fd = libc.memfd_create(b"attestation", 0x0002)
        try:
            os.fchmod(scanner_fd, 0o400)
            os.write(scanner_fd, scanner)
            scanner_stat = os.fstat(scanner_fd)
            base = {
                "run_id": "run",
                "router_zid": "router",
                "validation_supervisor_instance_id": "supervisor",
                "launcher_nonce": "nonce",
                "scanner_id": "scanner-1",
                "scanner_sha256": hashlib.sha256(scanner).hexdigest(),
                "scanner_device": scanner_stat.st_dev,
                "scanner_inode": scanner_stat.st_ino,
                "capability": json.loads(scanner.decode())["capability"],
            }
            canonical = json.dumps(base, separators=(",", ":"), sort_keys=True).encode()
            payload = {
                **base,
                "payload_sha256": hashlib.sha256(canonical).hexdigest(),
            }
            payload_bytes = json.dumps(payload, separators=(",", ":"), sort_keys=True).encode()
            os.write(attestation_fd, payload_bytes)
            fcntl.fcntl(
                attestation_fd,
                getattr(fcntl, "F_ADD_SEALS", 1033),
                (getattr(fcntl, "F_SEAL_SEAL", 1)
                 | getattr(fcntl, "F_SEAL_SHRINK", 2)
                 | getattr(fcntl, "F_SEAL_GROW", 4)
                 | getattr(fcntl, "F_SEAL_WRITE", 8)),
            )

            real_fstat = os.fstat

            def fake_fstat(fd):
                result = real_fstat(fd)
                if fd == scanner_fd:
                    return SimpleNamespace(
                        st_mode=stat.S_IFREG | (result.st_mode & 0o777),
                        st_uid=0,
                        st_size=result.st_size,
                        st_dev=result.st_dev,
                        st_ino=result.st_ino,
                    )
                return result

            with patch.dict(
                os.environ,
                {
                    "TIANJI_REAL_PREFLIGHT_FD": str(attestation_fd),
                    "TIANJI_REAL_PREFLIGHT_SCANNER_FD": str(scanner_fd),
                    "TIANJI_RUN_ID": "run",
                    "TIANJI_ROUTER_ZID": "router",
                    "TIANJI_VALIDATION_SUPERVISOR_INSTANCE_ID": "supervisor",
                },
                clear=False,
            ), patch("tianji_teleop.executors.marvin.preflight.os.fstat", side_effect=fake_fstat):
                capability = trusted_real_capability()
            self.assertTrue(capability.admitted)

        finally:
            os.close(attestation_fd)
            os.close(scanner_fd)
    def test_real_capability_rejects_ordinary_path_and_digest_environment(self):
        import hashlib
        import json
        import os
        from unittest.mock import patch
        from tianji_teleop.executors.marvin.preflight import trusted_real_capability

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "attestation.json"
            payload = json.dumps(
                {
                    "run_id": "run",
                    "router_zid": "router",
                    "validation_supervisor_instance_id": "supervisor",
                    "launcher_nonce": "nonce",
                    "capability": {
                        "speed": 0.25,
                        "yaw_deg": 0.0,
                        "deadman_available": True,
                        "preflight_passed": True,
                    },
                },
                separators=(",", ":"),
            ).encode()
            path.write_bytes(payload)
            path.chmod(0o400)
            with patch.dict(
                os.environ,
                {
                    "TIANJI_REAL_PREFLIGHT_FILE": str(path),
                    "TIANJI_REAL_PREFLIGHT_NONCE": "nonce",
                    "TIANJI_REAL_PREFLIGHT_DIGEST": hashlib.sha256(payload).hexdigest(),
                    "TIANJI_RUN_ID": "run",
                    "TIANJI_ROUTER_ZID": "router",
                    "TIANJI_VALIDATION_SUPERVISOR_INSTANCE_ID": "supervisor",
                },
                clear=False,
            ):
                self.assertFalse(trusted_real_capability().admitted)
    def test_analyzer_returns_verified_attestation_bytes_for_single_parse(self):
        import hashlib
        from scripts.validation.analyze_runs import _verify_checksums

        with tempfile.TemporaryDirectory() as directory:
            bundle = Path(directory)
            payload = b'{"sealed":true}'
            names = [
                "manifest.yaml", "status.jsonl", "operator_events.jsonl",
                "liveliness.jsonl", "protocol.jsonl", "operator_result.yaml",
                "real-preflight.json",
            ]
            for name in names:
                (bundle / name).write_bytes(payload if name == "real-preflight.json" else b"")
            lines = [
                f"{hashlib.sha256((bundle / name).read_bytes()).hexdigest()}  {name}"
                for name in names
            ]
            (bundle / "checksums.sha256").write_text("\n".join(lines) + "\n", encoding="utf-8")
            verified = _verify_checksums(bundle, session_required=False)
            self.assertEqual(verified["real-preflight.json"], payload)
if __name__ == "__main__":
    unittest.main()
