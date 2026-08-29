#!/usr/bin/env python3
"""Run one Tianji validation case and create an auditable result bundle.

The command deliberately never synthesizes a passing device result.  ``--fake
--headless`` exercises bundle creation and preflight only; its operator result
is ``aborted`` until an operator records a real observation.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import socket
import subprocess
import sys
import time
import uuid
from typing import Any, Callable, Iterable, Mapping

import yaml

ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = ROOT / "src" / "pico_body_tianji"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from pico_body_tianji.config_loader import DEFAULT_ROUTER_ENDPOINT, canonical_config_root
from pico_body_tianji.protocol.messages import ProtocolEnvelope, SafetyStopAck, SafetyStopRequest

MATRIX_PATH = canonical_config_root() / "validation" / "test_matrix.yaml"
MATRIX_SCHEMA = "tianji-validation-matrix"
BUNDLE_SCHEMA = "tianji-validation-run"
BUNDLE_VERSION = "1.0"
OUTCOMES = frozenset({"pass", "fail", "aborted"})
DANGEROUS_STOPS = frozenset({
    "wrong_direction_or_side", "physical_limit", "collision_risk", "feedback_stale",
    "tracking_threshold", "device_or_servo_error", "duplicate_authority",
    "router_zid_change", "emergency_stop",
})


def monotonic_ns() -> int:
    return time.monotonic_ns()


def utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_tree(path: Path, *, suffixes: Iterable[str] | None = None) -> str:
    digest = hashlib.sha256()
    if not path.exists():
        return "unavailable"
    allowed = set(suffixes) if suffixes is not None else None
    files = [item for item in path.rglob("*") if item.is_file()]
    for item in sorted(files):
        if allowed is not None and item.suffix not in allowed:
            continue
        relative = item.relative_to(path).as_posix().encode()
        digest.update(relative)
        digest.update(b"\0")
        with item.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    return digest.hexdigest()


def git_fingerprint(path: Path) -> dict[str, Any]:
    if not (path / ".git").exists():
        return {"commit": "unavailable", "dirty": None}
    try:
        commit = subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip()
        dirty = bool(subprocess.check_output(["git", "-C", str(path), "status", "--porcelain"], text=True, stderr=subprocess.DEVNULL).strip())
        return {"commit": commit, "dirty": dirty}
    except (OSError, subprocess.CalledProcessError):
        return {"commit": "unavailable", "dirty": None}


def load_matrix(path: Path = MATRIX_PATH) -> dict[str, Any]:
    try:
        matrix = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot read validation matrix: {path}: {exc}") from exc
    if not isinstance(matrix, dict) or matrix.get("schema_name") != MATRIX_SCHEMA or matrix.get("schema_version") != BUNDLE_VERSION:
        raise ValueError("unsupported validation matrix schema")
    cases = matrix.get("cases")
    if not isinstance(cases, dict) or not cases:
        raise ValueError("validation matrix must contain cases")
    required = {"profile", "required_devices", "required_capability", "active_sides", "hand_mode", "velocity_ratio", "acceleration_ratio", "max_duration_s", "prerequisites", "stop_criteria"}
    for case_id, case in cases.items():
        if not isinstance(case_id, str) or not isinstance(case, dict) or set(case) != required:
            raise ValueError(f"invalid validation case schema: {case_id}")
        if case["required_capability"] not in {"simulation", "real"}:
            raise ValueError(f"invalid required capability for {case_id}")
        if not isinstance(case["required_devices"], list) or not isinstance(case["active_sides"], list):
            raise ValueError(f"invalid device or sides list for {case_id}")
        if not isinstance(case["prerequisites"], list) or float(case["max_duration_s"]) <= 0:
            raise ValueError(f"invalid duration or prerequisites for {case_id}")
        stops = case["stop_criteria"]
        if not isinstance(stops, dict) or not stops.get("dangerous") or not stops.get("controlled"):
            raise ValueError(f"invalid stop criteria for {case_id}")
    return matrix


def _payload(value: Any) -> Mapping[str, Any]:
    if isinstance(value, Mapping):
        return value
    if hasattr(value, "to_dict"):
        result = value.to_dict()
        if isinstance(result, Mapping):
            return result
    raise TypeError("safety ack must be a mapping or protocol message")


@dataclass(frozen=True)
class SafetyStopResult:
    accepted: bool
    locked: bool
    reason: str
    request: SafetyStopRequest
    acked_executor_ids: tuple[str, ...]


class SafetyStopSupervisor:
    """Issue one authorized, latched stop and require every executor ack.

    ``publish`` and ``wait_ack`` are injected so the same strict logic can be
    used by a Zenoh process and by deterministic fake-device tests.  Missing or
    mismatched acknowledgements never become a pass; the supervisor remains
    locked and instructs the operator to maintain physical E-stop.
    """

    def __init__(self, run_id: str, supervisor_instance_id: str, router_zid: str, *, clock: Callable[[], int] = monotonic_ns) -> None:
        if not run_id or not supervisor_instance_id or not router_zid:
            raise ValueError("run_id, supervisor_instance_id and router_zid are required")
        self.run_id = run_id
        self.supervisor_instance_id = supervisor_instance_id
        self.router_zid = router_zid
        self.clock = clock
        self.sequence = 0
        self.locked = False
        self.last_request: SafetyStopRequest | None = None

    def issue(
        self,
        reason: str,
        executor_ids: Iterable[str],
        *,
        publish: Callable[[SafetyStopRequest], Any],
        wait_ack: Callable[[SafetyStopRequest], Mapping[str, Any] | Iterable[Any]],
    ) -> SafetyStopResult:
        if reason not in DANGEROUS_STOPS and not reason.strip():
            raise ValueError(f"invalid safety stop reason: {reason!r}")
        expected = tuple(dict.fromkeys(str(item) for item in executor_ids if str(item)))
        if not expected:
            raise ValueError("at least one executor is required for safety stop")
        self.sequence += 1
        request = SafetyStopRequest(
            ProtocolEnvelope(1, self.supervisor_instance_id, self.router_zid, self.sequence, int(self.clock())),
            self.run_id,
            reason,
            True,
        )
        self.last_request = request
        self.locked = True
        try:
            publish(request)
            raw_acks = wait_ack(request)
        except Exception as exc:  # fail closed; physical E-stop remains required
            return SafetyStopResult(False, True, f"safety stop ack unavailable: {exc}", request, ())
        if isinstance(raw_acks, Mapping):
            values = raw_acks.items()
        else:
            values = ((None, value) for value in raw_acks)
        valid: dict[str, Any] = {}
        errors: list[str] = []
        for key, value in values:
            try:
                payload = _payload(value)
                executor_id = str(payload.get("executor_id", key or ""))
                if executor_id not in expected:
                    errors.append(f"unexpected executor {executor_id}")
                    continue
                ack = value if isinstance(value, SafetyStopAck) else SafetyStopAck.from_dict(payload)
                ack.validate_for(executor_id, self.run_id)
                if ack.envelope.router_zid != self.router_zid or ack.envelope.sequence != request.envelope.sequence:
                    raise ValueError("ack sequence/router mismatch")
                valid[executor_id] = ack
            except (TypeError, ValueError, KeyError) as exc:
                errors.append(str(exc))
        missing = [executor_id for executor_id in expected if executor_id not in valid]
        if missing:
            errors.append("missing ack: " + ", ".join(missing))
        if errors:
            return SafetyStopResult(False, True, "safety stop ack failure: " + "; ".join(errors), request, tuple(sorted(valid)))
        return SafetyStopResult(True, True, "all executor safety-stop acknowledgements received", request, tuple(sorted(valid)))
class ZenohSafetyTransport:
    """Small transport adapter for an explicit managed safety stop.

    The adapter is only constructed after the managed session has started.
    It never emits return or clear messages and closes after collecting the
    matching acknowledgements (or timeout).
    """

    def __init__(self, endpoint: str, *, timeout_s: float = 3.0) -> None:
        import threading
        from pico_body_tianji.zenoh_util import open_session

        self._threading = threading
        self.timeout_s = float(timeout_s)
        if self.timeout_s <= 0:
            raise ValueError("safety ack timeout must be positive")
        self.session = open_session(endpoint)
        self.publisher = self.session.declare_publisher("tianji/safety/stop")
        self._acks: list[Mapping[str, Any]] = []
        self._event = threading.Event()
        self.subscriber = self.session.declare_subscriber("tianji/safety/ack/**", self._on_ack)

    def _on_ack(self, sample: Any) -> None:
        try:
            payload = sample.payload.to_bytes()
            value = json.loads(payload.decode("utf-8"))
            if isinstance(value, Mapping):
                self._acks.append(value)
                self._event.set()
        except (AttributeError, UnicodeDecodeError, json.JSONDecodeError, TypeError):
            # A malformed ack is not accepted; the supervisor will report a
            # missing/mismatched acknowledgement and remain locked.
            return

    def publish(self, request: SafetyStopRequest) -> None:
        self.publisher.put(json.dumps(request.to_dict(), separators=(",", ":")).encode("utf-8"), encoding="application/json")

    def wait_ack(self, request: SafetyStopRequest) -> Mapping[str, Any]:
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            self._event.wait(timeout=max(0.0, min(0.1, deadline - time.monotonic())))
            if self._acks:
                # Return one row per executor; duplicate rows do not conceal a
                # missing executor because the supervisor validates the set.
                return {str(row.get("executor_id", "")): row for row in self._acks}
        return {str(row.get("executor_id", "")): row for row in self._acks}

    def close(self) -> None:
        for item in (getattr(self, "subscriber", None), getattr(self, "publisher", None), getattr(self, "session", None)):
            try:
                if item is not None:
                    item.close() if hasattr(item, "close") else item.undeclare()
            except Exception:
                pass


def _source_type(profile: str) -> str:
    values = {
        "acquisition_live": "mocap_live",
        "pico_sim": "pico_controller",
        "pico_real": "pico_controller",
        "mocap_live_sim": "mocap_live",
        "mocap_live_real": "mocap_live",
        "h5_sim": "h5_replay",
        "h5_real": "h5_replay",
        "target_replay_sim": "target_replay",
        "joint_replay_sim": "joint_replay",
    }
    return values.get(profile, "pico_controller")
def _profile_config(profile: str) -> dict[str, Any]:
    path = canonical_config_root() / "sessions" / f"{profile}.yaml"
    if not path.is_file():
        return {}
    value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return value if isinstance(value, dict) else {}


def _hashes() -> dict[str, str]:
    config_root = canonical_config_root()
    acl = Path("/home/current/syz/mocap/acquisition/config/zenohd_acl.yaml")
    return {
        "config_sha256": sha256_tree(config_root, suffixes={".yaml", ".yml"}),
        "runtime_sha256": sha256_tree(ROOT / "runtime"),
        "acl_sha256": sha256_file(acl) if acl.is_file() else "unavailable",
    }


def _write_status(stream: Any, *, event: str, component: str, supervisor: str, run_id: str, **fields: Any) -> None:
    record = {
        "schema_version": 1,
        "timestamp_ns": monotonic_ns(),
        "run_id": run_id,
        "component": component,
        "event": event,
        "publisher_instance_id": supervisor,
        **fields,
    }
    stream.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
    stream.flush()


def _write_operator_event(path: Path, supervisor: str, run_id: str, event: str, details: str = "") -> None:
    if not event.strip():
        raise ValueError("operator event cannot be empty")
    record = {
        "schema_version": 1,
        "time_ns": monotonic_ns(),
        "run_id": run_id,
        "publisher_instance_id": supervisor,
        "event": event.strip(),
        "details": details,
    }
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")


def _write_checksums(bundle: Path) -> None:
    names = [
        "manifest.yaml", "status.jsonl", "operator_events.jsonl",
        "liveliness.jsonl", "protocol.jsonl", "operator_result.yaml",
    ]
    if (bundle / "session.h5").is_file():
        names.insert(1, "session.h5")
    names += sorted(path.relative_to(bundle).as_posix() for path in (bundle / "logs").glob("*") if path.is_file())
    with (bundle / "checksums.sha256").open("w", encoding="utf-8") as stream:
        for name in names:
            stream.write(f"{sha256_file(bundle / name)}  {name}\n")


def _create_empty_session(path: Path, source_type: str, router_zid: str) -> None:
    # Empty is intentional: fake/headless mode proves the schema and safety
    # plumbing only. It is never reported as a successful device run.
    try:
        from pico_body_tianji.recording.session_h5 import SessionH5Writer
    except ModuleNotFoundError as exc:
        # ``validation-run --list`` and local preflight must work without the
        # optional Zenoh wheel.  Keep a minimal HDF5 artifact for that mode;
        # the Pixi runtime uses the canonical SessionH5Writer below.
        if exc.name != "zenoh":
            raise
        import h5py
        path.parent.mkdir(parents=True, exist_ok=True)
        with h5py.File(path, "w") as file:
            file.attrs.update(
                schema_name="tianji-teleop-session",
                schema_version="1.0",
                source_type=source_type,
                robot_model="unknown-unverified",
                router_zid=router_zid,
                complete=True,
            )
            for group in ("raw", "target", "joint", "meta"):
                file.create_group(group)
        return
    with SessionH5Writer(path, source_type=source_type, robot_model="unknown-unverified", router_zid=router_zid):
        pass


def _prerequisites_passed(root: Path, prerequisites: Iterable[str]) -> tuple[bool, list[str]]:
    """Accept only bundles validated by analyzer, never directory-name hints."""
    missing: list[str] = []
    if not prerequisites:
        return True, missing
    try:
        from scripts.validation.analyze_runs import AnalysisError, analyze_bundle
    except ImportError as exc:
        raise RuntimeError(f"cannot load prerequisite analyzer: {exc}") from exc
    matrix = load_matrix()
    for prerequisite in prerequisites:
        found = False
        if root.is_dir():
            for candidate in sorted(path for path in root.iterdir() if path.is_dir() and (path / "manifest.yaml").is_file()):
                try:
                    manifest = yaml.safe_load((candidate / "manifest.yaml").read_text(encoding="utf-8")) or {}
                    if manifest.get("case_id") != prerequisite:
                        continue
                    report = analyze_bundle(candidate, matrix)
                    found = report.get("operator_outcome") == "pass"
                except (AnalysisError, OSError, yaml.YAMLError, ValueError):
                    found = False
                if found:
                    break
        if not found:
            missing.append(prerequisite)
    return not missing, missing


def _h5_contains_hand_joints(path: Path | None) -> bool:
    if path is None or not path.is_file():
        return False
    try:
        import h5py
        with h5py.File(path, "r") as file:
            found = False
            def visit(name: str, _obj: Any) -> None:
                nonlocal found
                found = found or "wuji2_joints" in name
            file.visititems(visit)
            return found
    except (OSError, ModuleNotFoundError):
        return False


def _resolved_hand_mode(case: Mapping[str, Any], profile: str, input_path: Path | None) -> str:
    mode = str(case.get("hand_mode", "disabled"))
    if mode != "auto":
        return mode
    if profile == "target_replay_sim":
        return "retarget"
    return "direct" if _h5_contains_hand_joints(input_path) else "retarget"


def _instance_map(manifest: Mapping[str, Any], key: str) -> dict[str, str]:
    value = manifest.get("publisher_instance_ids", {}).get(key, {})
    return dict(value) if isinstance(value, Mapping) else {}


def instance_handoff_environment(manifest: Mapping[str, Any]) -> dict[str, str]:
    """Return exact preallocated IDs for run_session child processes."""
    ids = manifest.get("publisher_instance_ids", {})
    environment: dict[str, str] = {
        "TIANJI_RUN_ID": str(manifest["run_id"]),
        "TIANJI_VALIDATION_SUPERVISOR_INSTANCE_ID": str(ids.get("validation_supervisor", "")),
        "TIANJI_COORDINATOR_INSTANCE_ID": str(ids.get("coordinator_arm", "")),
        "TIANJI_SOURCE_INSTANCE_ID": str(ids.get("source", "")),
        "TIANJI_ARM_PRODUCER_INSTANCE_ID": str(ids.get("producer_arm", "")),
        "TIANJI_ARM_EXECUTOR_INSTANCE_ID": str(ids.get("executor_arm", "")),
        "TIANJI_RECORDER_INSTANCE_ID": str(ids.get("recorder", "")),
    }
    producer = _instance_map(manifest, "hand_producer_instances")
    executor = _instance_map(manifest, "hand_executor_instances")
    if producer:
        environment["TIANJI_HAND_PRODUCER_INSTANCES"] = ",".join(f"{side}={value}" for side, value in sorted(producer.items()))
    if executor:
        environment["TIANJI_HAND_EXECUTOR_INSTANCES"] = ",".join(f"{side}={value}" for side, value in sorted(executor.items()))
    return {key: value for key, value in environment.items() if value}


def _build_manifest(case_id: str, case: Mapping[str, Any], profile: str, run_id: str, supervisor: str, router_zid: str, started: str, args: argparse.Namespace) -> dict[str, Any]:
    profile_config = _profile_config(profile)
    source = profile_config.get("source_config", "")
    backend = args.ik_backend or ("pinocchio_cpp" if case_id == "ik_pinocchio_cpp" else "pinocchio_qp" if case_id == "ik_pinocchio_qp" else "tianji_official" if case_id == "ik_tianji_official" else None)
    input_path = Path(args.input).expanduser() if args.input else None
    instance_ids: dict[str, Any] = {"validation_supervisor": supervisor}
    if not args.fake:
        instance_ids.update({
            "source": str(uuid.uuid4()),
            "producer_arm": str(uuid.uuid4()),
            "coordinator_arm": str(uuid.uuid4()),
            "executor_arm": str(uuid.uuid4()),
            "recorder": str(uuid.uuid4()),
        })
        actual_hand_mode = _resolved_hand_mode(case, profile, input_path)
        hand_sides = list(case["active_sides"]) if case.get("hand_mode") != "disabled" else []
        if hand_sides:
            producer_instances: dict[str, str] = {}
            executor_instances: dict[str, str] = {}
            for side in hand_sides:
                executor_instances[side] = str(uuid.uuid4())
                if actual_hand_mode == "direct" and profile in {"h5_sim", "h5_real"}:
                    producer_instances[side] = instance_ids["source"]
                elif profile == "joint_replay_sim":
                    producer_instances[side] = instance_ids["producer_arm"]
                else:
                    producer_instances[side] = str(uuid.uuid4())
            instance_ids["hand_producer_instances"] = producer_instances
            instance_ids["hand_executor_instances"] = executor_instances
            instance_ids["producer_hand"] = producer_instances[hand_sides[0]]
            instance_ids["executor_hand"] = executor_instances[hand_sides[0]]
    hashes = _hashes()
    repositories = {"teleop": git_fingerprint(ROOT), "acquisition": git_fingerprint(Path("/home/current/syz/mocap/acquisition"))}
    manifest: dict[str, Any] = {
        "schema_name": BUNDLE_SCHEMA,
        "schema_version": BUNDLE_VERSION,
        "run_id": run_id,
        "case_id": case_id,
        "profile": profile,
        "required_devices": list(case["required_devices"]),
        "required_capability": case["required_capability"],
        "active_sides": list(case["active_sides"]),
        "hand": {"sides": list(case["active_sides"] if case.get("hand_mode") != "disabled" else []), "mode": case["hand_mode"]},
        "hand_sides": list(case["active_sides"] if case.get("hand_mode") != "disabled" else []),
        "hand_mode": case["hand_mode"],
        "robot": {"ip": args.robot_ip or "unrecorded", "model": args.robot_model or "unverified"},
        "robot_ip": args.robot_ip or "unrecorded",
        "robot_model": args.robot_model or "unverified",
        "motive_rigid_ids": list(args.motive_rigid_id or []),
        "h5_input_sha256": sha256_file(input_path) if input_path and input_path.is_file() else None,
        "ik_backend": backend,
        "safety": {"velocity_ratio": float(case["velocity_ratio"]), "acceleration_ratio": float(case["acceleration_ratio"])},
        "velocity_ratio": float(case["velocity_ratio"]),
        "acceleration_ratio": float(case["acceleration_ratio"]),
        "max_duration_s": float(case["max_duration_s"]),
        "repositories": repositories,
        "teleop_commit": repositories["teleop"]["commit"],
        "teleop_dirty": repositories["teleop"]["dirty"],
        "acquisition_commit": repositories["acquisition"]["commit"],
        "acquisition_dirty": repositories["acquisition"]["dirty"],
        "hashes": hashes,
        "config_sha256": hashes["config_sha256"],
        "runtime_sha256": hashes["runtime_sha256"],
        "acl_sha256": hashes["acl_sha256"],
        "router": {"endpoint": os.environ.get("TIANJI_ROUTER_ENDPOINT", DEFAULT_ROUTER_ENDPOINT), "zid": router_zid},
        "router_endpoint": os.environ.get("TIANJI_ROUTER_ENDPOINT", DEFAULT_ROUTER_ENDPOINT),
        "router_zid": router_zid,
        "publisher_instance_ids": instance_ids,
        "machine": {"hostname": socket.gethostname(), "os": platform.platform(), "python": platform.python_version()},
        "hostname": socket.gethostname(),
        "os": platform.platform(),
        "source_config": source,
        "fake": bool(args.fake),
        "headless": bool(args.headless),
        "started_at": started,
        "ended_at": None,
        "exit_reason": "not_finished",
    }
    return manifest


def _parse_event(value: str) -> tuple[str, str]:
    if "=" in value:
        event, details = value.split("=", 1)
    elif ":" in value:
        event, details = value.split(":", 1)
    else:
        event, details = value, ""
    return event.strip(), details.strip()


def _write_operator_result(path: Path, *, outcome: str, emergency_stop: bool = False, abnormal_direction: bool = False, jitter: bool = False, noise: bool = False, collision_risk: bool = False, notes: str = "") -> None:
    if outcome not in OUTCOMES:
        raise ValueError(f"invalid operator outcome: {outcome}")
    data = {
        "schema_version": BUNDLE_VERSION,
        "outcome": outcome,
        "emergency_stop": bool(emergency_stop),
        "abnormal_direction": bool(abnormal_direction),
        "jitter": bool(jitter),
        "noise": bool(noise),
        "collision_risk": bool(collision_risk),
        "notes": notes,
    }
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def _managed_stop(bundle: Path, manifest: Mapping[str, Any], status: Any, args: argparse.Namespace) -> SafetyStopResult | None:
    if not args.danger_stop:
        return None
    ids = [str(manifest["publisher_instance_ids"]["executor_arm"])]
    if manifest.get("hand_mode") != "disabled" and "executor_hand" in manifest["publisher_instance_ids"]:
        ids.append(str(manifest["publisher_instance_ids"]["executor_hand"]))
    supervisor_id = str(manifest["publisher_instance_ids"]["validation_supervisor"])
    supervisor = SafetyStopSupervisor(str(manifest["run_id"]), supervisor_id, str(manifest["router_zid"]))
    transport: ZenohSafetyTransport | None = None
    try:
        transport = ZenohSafetyTransport(str(manifest["router"]["endpoint"]))
        result = supervisor.issue(args.danger_stop, ids, publish=transport.publish, wait_ack=transport.wait_ack)
    except Exception as exc:
        request = SafetyStopRequest(
            ProtocolEnvelope(1, supervisor_id, str(manifest["router_zid"]), supervisor.sequence + 1, monotonic_ns()),
            str(manifest["run_id"]), args.danger_stop, True,
        )
        supervisor.locked = True
        result = SafetyStopResult(False, True, f"safety stop transport unavailable: {exc}", request, ())
    finally:
        if transport is not None:
            transport.close()
    _write_status(
        status,
        event="safety_stop",
        component="validation",
        supervisor=supervisor_id,
        run_id=str(manifest["run_id"]),
        reason=args.danger_stop,
        expected_executor_ids=ids,
        acked_executor_ids=list(result.acked_executor_ids),
        ack_complete=result.accepted,
        new_motion_commands_after_stop=False,
        lockout=True,
    )
    if not result.accepted:
        _write_operator_event(
            bundle / "operator_events.jsonl",
            supervisor_id,
            str(manifest["run_id"]),
            "physical_estop_required",
            result.reason,
        )
    return result


def _run_fake(bundle: Path, manifest: dict[str, Any], status: Any, args: argparse.Namespace) -> int:
    _write_status(status, event="preflight_passed", component="validation", supervisor=manifest["publisher_instance_ids"]["validation_supervisor"], run_id=manifest["run_id"], mode="fake_headless", physical_validation=False)
    capture = {
        "schema_version": 1,
        "timestamp_ns": monotonic_ns(),
        "run_id": manifest["run_id"],
        "publisher_instance_id": manifest["publisher_instance_ids"]["validation_supervisor"],
        "mode": "fake_headless",
    }
    if manifest["profile"] not in {"target_replay_sim", "joint_replay_sim", "acquisition_live"} and not (bundle / "session.h5").exists():
        _create_empty_session(bundle / "session.h5", _source_type(manifest["profile"]), manifest["router_zid"])
    for raw in args.operator_event or []:
        event, details = _parse_event(raw)
        _write_operator_event(bundle / "operator_events.jsonl", manifest["publisher_instance_ids"]["validation_supervisor"], manifest["run_id"], event, details)
    if args.danger_stop:
        expected = ["fake_arm_executor"]
        supervisor = SafetyStopSupervisor(manifest["run_id"], manifest["publisher_instance_ids"]["validation_supervisor"], manifest["router_zid"])
        stop = supervisor.issue(args.danger_stop, expected, publish=lambda _: None, wait_ack=lambda _: {
            executor: {"schema_version": 1, "publisher_instance_id": executor, "router_zid": manifest["router_zid"], "sequence": 1, "timestamp_ns": monotonic_ns(), "executor_id": executor, "run_id": manifest["run_id"], "latched": True, "reason": args.danger_stop}
            for executor in expected
        })
        _write_status(status, event="safety_stop", component="validation", supervisor=manifest["publisher_instance_ids"]["validation_supervisor"], run_id=manifest["run_id"], reason=args.danger_stop, expected_executor_ids=expected, acked_executor_ids=list(stop.acked_executor_ids), ack_complete=stop.accepted, new_motion_commands_after_stop=False, lockout=True)
        if not stop.accepted:
            _write_operator_event(bundle / "operator_events.jsonl", manifest["publisher_instance_ids"]["validation_supervisor"], manifest["run_id"], "physical_estop_required", stop.reason)
    _write_status(status, event="finished", component="validation", supervisor=manifest["publisher_instance_ids"]["validation_supervisor"], run_id=manifest["run_id"], exit_reason="fake_headless_only", physical_validation=False)
    return 0


def _run_session(bundle: Path, manifest: dict[str, Any], status: Any, args: argparse.Namespace) -> int:
    profile = manifest["profile"]
    if profile == "acquisition_live":
        _write_status(
            status,
            event="acquisition_observation_required",
            component="acquisition",
            supervisor=manifest["publisher_instance_ids"]["validation_supervisor"],
            run_id=manifest["run_id"],
            note="StreamHub is external; run acquisition observation in its own terminal",
        )
        return 0
    command = ["bash", str(ROOT / "scripts" / "run_session.sh"), "--profile", profile]
    if profile not in {"target_replay_sim", "joint_replay_sim"}:
        command += ["--record", str(bundle / "session.h5")]
    if args.input:
        command += ["--input", args.input]
    if args.confirm_real:
        command.append("--confirm-real")
    if args.extra:
        command += ["--", *args.extra]
    env = os.environ.copy()
    env.update(instance_handoff_environment(manifest))
    env.update({
        "TIANJI_SAFETY_SUPERVISOR_INSTANCE_ID": manifest["publisher_instance_ids"]["validation_supervisor"],
        "TIANJI_ROUTER_ZID": manifest["router_zid"],
        "TIANJI_IK_BACKEND": str(manifest.get("ik_backend") or ""),
    })
    log_path = bundle / "logs" / "session.log"
    with log_path.open("w", encoding="utf-8") as log:
        _write_status(status, event="session_starting", component="run_session", supervisor=manifest["publisher_instance_ids"]["validation_supervisor"], run_id=manifest["run_id"], command=command)
        capture = {
            "schema_version": 1,
            "timestamp_ns": monotonic_ns(),
            "run_id": manifest["run_id"],
            "publisher_instance_id": manifest["publisher_instance_ids"]["validation_supervisor"],
            "event": "managed_session_starting",
            "router_zid": manifest["router_zid"],
        }
        for name in ("liveliness.jsonl", "protocol.jsonl"):
            with (bundle / name).open("a", encoding="utf-8") as capture_stream:
                capture_stream.write(json.dumps(capture, separators=(",", ":")) + "\n")
        process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, text=True)
        stop_result = _managed_stop(bundle, manifest, status, args)
        deadline = time.monotonic() + float(manifest["max_duration_s"])
        exit_reason = "operator_interrupt"
        try:
            while process.poll() is None and time.monotonic() < deadline:
                if stop_result is not None:
                    exit_reason = "danger_stop"
                    process.terminate()
                    break
                time.sleep(0.2)
            if process.poll() is None:
                if stop_result is None:
                    exit_reason = "max_duration_reached"
                    process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            elif exit_reason == "operator_interrupt":
                exit_reason = f"session_exit_{process.returncode}"
        except KeyboardInterrupt:
            exit_reason = "operator_interrupt"
            process.terminate()
            process.wait(timeout=5)
        _write_status(status, event="session_finished", component="run_session", supervisor=manifest["publisher_instance_ids"]["validation_supervisor"], run_id=manifest["run_id"], exit_reason=exit_reason, process_returncode=process.returncode)
    runtime_dir = Path(env.get("PICO_TIANJI_RUNTIME_DIR", os.environ.get("PICO_TIANJI_RUNTIME_DIR", "/tmp")))
    for child_log in runtime_dir.glob(f"{manifest['run_id']}-*.log"):
        destination = bundle / "logs" / f"{child_log.stem}.log"
        try:
            shutil.copy2(child_log, destination)
        except OSError:
            # Missing child log is evidence of failed capture; retain the
            # validation log and let analyzer/checksum expose what is present.
            continue
    if profile not in {"target_replay_sim", "joint_replay_sim", "acquisition_live"} and not (bundle / "session.h5").exists():
        raise RuntimeError("session recorder did not produce session.h5")
    if stop_result is not None and not stop_result.accepted:
        return 1
    if process.returncode not in (0, -2, -15):
        return int(process.returncode or 1)
    return 0


def run_case(args: argparse.Namespace) -> int:
    matrix = load_matrix()
    cases = matrix["cases"]
    if args.list:
        print("\n".join(cases.keys()))
        return 0
    if not args.case:
        raise ValueError("--case is required (or use --list)")
    if args.case not in cases:
        raise ValueError(f"unknown validation case: {args.case}")
    case = cases[args.case]
    capability = case["required_capability"]
    if capability == "real" and not args.confirm_real:
        raise PermissionError("real validation requires explicit --confirm-real")
    prerequisite_root = Path(args.prerequisite_root or args.output).expanduser()
    prerequisites_ok, missing = _prerequisites_passed(prerequisite_root, case["prerequisites"])
    if capability == "real" and not prerequisites_ok:
        raise PermissionError("real validation prerequisites not passed: " + ", ".join(missing))
    if capability == "real" and args.fake:
        raise PermissionError("--fake cannot be used for real validation")
    if not args.fake and capability == "real" and not args.robot_ip:
        raise PermissionError("real validation requires --robot-ip during preflight")
    output_root = Path(args.output).expanduser()
    output_root.mkdir(parents=True, exist_ok=True)
    started = utc_now()
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + f"_{args.case}_{uuid.uuid4().hex[:8]}"
    bundle = output_root / run_id
    bundle.mkdir()
    (bundle / "logs").mkdir()
    supervisor = str(uuid.uuid4())
    router_zid = "fake-router-unverified" if args.fake else os.environ.get("TIANJI_ROUTER_ZID", "")
    if not router_zid:
        raise RuntimeError("router ZID unavailable; run the managed router before validation")
    manifest = _build_manifest(args.case, case, case["profile"], run_id, supervisor, router_zid, started, args)
    (bundle / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    (bundle / "operator_events.jsonl").touch()
    (bundle / "liveliness.jsonl").touch()
    (bundle / "protocol.jsonl").touch()
    with (bundle / "status.jsonl").open("w", encoding="utf-8") as status, (bundle / "logs" / "validation.log").open("w", encoding="utf-8") as log:
        _write_status(status, event="preflight_started", component="validation", supervisor=supervisor, run_id=run_id, required_capability=capability, required_devices=case["required_devices"], physical_validation=not args.fake)
        log.write(f"run_id={run_id}\ncase={args.case}\nmode={'fake_headless' if args.fake else 'managed_session'}\n")
        rc = _run_fake(bundle, manifest, status, args) if args.fake else _run_session(bundle, manifest, status, args)
    manifest["ended_at"] = utc_now()
    manifest["exit_reason"] = "fake_headless_only" if args.fake else "operator_or_session_exit"
    (bundle / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    if not args.fake:
        for raw in args.operator_event or []:
            event, details = _parse_event(raw)
            _write_operator_event(bundle / "operator_events.jsonl", supervisor, run_id, event, details)
    requested_outcome = "aborted" if args.fake else (args.operator_outcome or "aborted")
    if requested_outcome in {"pass", "fail"} and not (args.operator_event or []):
        requested_outcome = "fail"
        args.operator_notes = (args.operator_notes or "") + " operator outcome requires at least one explicit operator event"
        rc = max(rc, 1)
    if requested_outcome == "pass" and rc != 0:
        requested_outcome = "fail"
        args.operator_notes = (args.operator_notes or "") + " session exited non-zero"
    _write_operator_result(
        bundle / "operator_result.yaml",
        outcome=requested_outcome,
        notes=args.operator_notes or ("No physical acceptance claim; record operator result after completing the runbook." if requested_outcome == "aborted" else "Operator recorded outcome with explicit event."),
    )
    _write_checksums(bundle)
    print(bundle)
    return rc


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="list fixed validation case IDs")
    parser.add_argument("--case")
    parser.add_argument("--output", default="validation-results")
    parser.add_argument("--confirm-real", action="store_true")
    parser.add_argument("--fake", action="store_true", help="bundle/preflight-only fake mode; never reports pass")
    parser.add_argument("--headless", action="store_true", help="request headless execution (required with --fake for clarity)")
    parser.add_argument("--input", "--h5", dest="input")
    parser.add_argument("--prerequisite-root")
    parser.add_argument("--robot-ip")
    parser.add_argument("--robot-model")
    parser.add_argument("--motive-rigid-id", action="append")
    parser.add_argument("--ik-backend")
    parser.add_argument("--operator-outcome", choices=sorted(OUTCOMES), help="explicit operator result; fake mode remains aborted")
    parser.add_argument("--operator-notes", default="")
    parser.add_argument("--operator-event", "--event", action="append")
    parser.add_argument("--danger-stop", choices=sorted(DANGEROUS_STOPS), help="explicitly issue a latched danger stop; never automatic")
    parser.add_argument("--extra", nargs=argparse.REMAINDER)
    return parser


def main(argv: list[str] | None = None) -> int:
    try:
        return run_case(build_parser().parse_args(argv))
    except PermissionError as exc:
        print(f"preflight rejected: {exc}", file=sys.stderr)
        return 2
    except (ValueError, RuntimeError, OSError, yaml.YAMLError) as exc:
        print(f"validation-run failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
