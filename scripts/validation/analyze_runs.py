#!/usr/bin/env python3
"""Validate and summarize Tianji validation result bundles.

The analyzer is fail-closed. A bundle is never considered a physical pass
unless the case contract, authority identities, real wire evidence, safety
state, configured limits, and operator result all agree.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import re
import sys
from typing import Any, Iterable, Mapping

try:
    import h5py
    import numpy as np
except ModuleNotFoundError:
    h5py = None  # type: ignore[assignment]
    np = None  # type: ignore[assignment]
import yaml

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC_ROOT = ROOT / "src" / "pico_body_tianji"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))
from pico_body_tianji.config_loader import canonical_config_root
from scripts.validation.run_case import (
    BUNDLE_SCHEMA,
    BUNDLE_VERSION,
    MATRIX_PATH,
    _authority_contract,
    _profile_config,
    _source_type,
    build_session_contract,
    load_matrix,
    sha256_file,
    sha256_tree,
)

REQUIRED_FILES = frozenset({
    "manifest.yaml", "status.jsonl", "operator_events.jsonl",
    "liveliness.jsonl", "protocol.jsonl", "operator_result.yaml", "checksums.sha256",
})
SESSION_OPTIONAL_PROFILES = frozenset({"target_replay_sim", "joint_replay_sim", "acquisition_live", "wuji_direct_real"})
OUTCOMES = frozenset({"pass", "fail", "aborted"})

REQUIRED_EVIDENCE: dict[str, dict[str, set[str]]] = {
    "acquisition_live": {"streams": {"aligned_hands"}, "checks": {"aligned_hands"}},
    "target_replay_sim": {"streams": {"target_arm_left", "target_arm_right"}, "checks": {"target_to_solved"}},
    "joint_replay_sim": {"streams": {"command_arm_left", "command_arm_right"}, "checks": {"direct_command"}},
    "wuji_direct_real": {"streams": {"command_arm_left", "command_arm_right", "command_hand_left", "command_hand_right"}, "checks": {"direct_command", "hand_zero"}},
}
for _case_id in ("pico_sim", "mocap_live_sim", "h5_sim", "marvin_pico_real_10pct", "marvin_mocap_live_real_10pct", "marvin_h5_real_10pct", "wuji_retarget_dry", "wuji_retarget_real", "fault_recovery_sim", "fault_recovery_real", "policy_hold_sim", "ik_pinocchio_cpp", "ik_pinocchio_qp", "ik_tianji_official"):
    REQUIRED_EVIDENCE.setdefault(_case_id, {"streams": {"state_arm"}, "checks": {"home_feedback"}})
REQUIRED_EVIDENCE["h5_sim"]["checks"].add("target_to_solved")
REQUIRED_EVIDENCE["wuji_retarget_dry"]["checks"].update({"target_to_solved", "hand_zero"})
REQUIRED_EVIDENCE["wuji_retarget_real"]["checks"].update({"target_to_solved", "hand_zero", "home_feedback"})
for _case_id in ("ik_pinocchio_cpp", "ik_pinocchio_qp", "ik_tianji_official"):
    REQUIRED_EVIDENCE[_case_id]["checks"].add("target_to_solved")


class AnalysisError(ValueError):
    """Bundle is not a trustworthy validation result."""


def _yaml(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise AnalysisError(f"cannot read YAML {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise AnalysisError(f"YAML object required: {path}")
    return value


def _json_lines(path: Path) -> list[dict[str, Any]]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise AnalysisError(f"cannot read JSONL {path}: {exc}") from exc
    values: list[dict[str, Any]] = []
    for index, line in enumerate(lines, 1):
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as exc:
            raise AnalysisError(f"invalid JSONL {path}:{index}: {exc}") from exc
        if not isinstance(value, dict):
            raise AnalysisError(f"JSONL record must be an object: {path}:{index}")
        values.append(value)
    return values


def _bundle_paths(root: Path) -> list[Path]:
    if (root / "manifest.yaml").is_file():
        return [root]
    if not root.is_dir():
        raise AnalysisError(f"result root does not exist: {root}")
    paths = sorted(path for path in root.iterdir() if path.is_dir() and (path / "manifest.yaml").is_file())
    if not paths:
        raise AnalysisError(f"no validation bundles under {root}")
    return paths


def _verify_checksums(bundle: Path, *, session_required: bool = True) -> None:
    try:
        lines = (bundle / "checksums.sha256").read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise AnalysisError(f"cannot read checksum file: {exc}") from exc
    if not lines:
        raise AnalysisError("checksums.sha256 is empty")
    listed: dict[str, str] = {}
    for line in lines:
        parts = line.split()
        if len(parts) != 2 or len(parts[0]) != 64 or any(char not in "0123456789abcdef" for char in parts[0].lower()):
            raise AnalysisError(f"malformed checksum line: {line!r}")
        name = parts[1]
        path = bundle / name
        if name in listed or not path.is_file() or name == "checksums.sha256" or Path(name).is_absolute() or ".." in Path(name).parts:
            raise AnalysisError(f"invalid checksum path: {name}")
        listed[name] = parts[0].lower()
    expected = set(REQUIRED_FILES - {"checksums.sha256"})
    if session_required:
        expected.add("session.h5")
    expected |= {path.relative_to(bundle).as_posix() for path in (bundle / "logs").glob("*") if path.is_file()}
    if set(listed) != expected:
        missing = sorted(expected - set(listed)); extra = sorted(set(listed) - expected)
        raise AnalysisError(f"checksum set mismatch; missing={missing}, extra={extra}")
    for name, expected_hash in listed.items():
        if sha256_file(bundle / name) != expected_hash:
            raise AnalysisError(f"checksum mismatch: {name}")


def _verify_manifest(bundle: Path, matrix: Mapping[str, Any]) -> dict[str, Any]:
    manifest = _yaml(bundle / "manifest.yaml")
    if manifest.get("schema_name") != BUNDLE_SCHEMA or manifest.get("schema_version") != BUNDLE_VERSION:
        raise AnalysisError("unsupported validation manifest schema")
    case_id = manifest.get("case_id")
    cases = matrix.get("cases", {})
    if case_id not in cases:
        raise AnalysisError(f"manifest case is not in matrix: {case_id}")
    case = cases[case_id]
    if manifest.get("profile") != case["profile"] or manifest.get("required_capability") != case["required_capability"]:
        raise AnalysisError("manifest case/profile/capability does not match matrix")
    if manifest.get("active_sides") != case["active_sides"] or manifest.get("hand_mode") != case["hand_mode"]:
        raise AnalysisError("manifest active sides/hand mode does not match matrix")
    try:
        contract = build_session_contract(case_id, {"ik_backend": manifest.get("ik_backend")})
    except ValueError as exc:
        raise AnalysisError(f"manifest routing contract is invalid: {exc}") from exc
    if manifest.get("producer") != contract["producer"] or manifest.get("profile") != contract["profile"]:
        raise AnalysisError("manifest producer/profile does not match fixed case contract")
    if manifest.get("source_type") != _source_type(manifest["profile"]):
        raise AnalysisError("manifest source_type does not match fixed profile contract")
    if manifest.get("ik_backend") != contract["ik_backend"]:
        raise AnalysisError("manifest IK backend does not match fixed case contract")
    if manifest.get("resolved_hand_mode") not in {"disabled", "direct", "retarget"}:
        raise AnalysisError("manifest resolved hand mode is missing")
    if manifest.get("required_capability") == "real" and manifest.get("robot_ip") in {None, "", "unrecorded"}:
        raise AnalysisError("real manifest must include robot IP")
    expected_authorities = _authority_contract(manifest)
    if manifest.get("authority_contract") != expected_authorities:
        raise AnalysisError("manifest authority contract does not match preallocated identities")
    for name in ("run_id", "router_zid", "started_at", "ended_at", "exit_reason"):
        if not manifest.get(name):
            raise AnalysisError(f"manifest field is missing: {name}")
    hashes = manifest.get("hashes")
    if not isinstance(hashes, dict):
        raise AnalysisError("manifest hashes are missing")
    for name in ("config_sha256", "runtime_sha256", "acl_sha256"):
        if manifest.get(name) != hashes.get(name):
            raise AnalysisError(f"manifest {name} alias mismatch")
    if hashes.get("config_sha256") != sha256_tree(canonical_config_root(), suffixes={".yaml", ".yml"}):
        raise AnalysisError("canonical config hash mismatch")
    if hashes.get("runtime_sha256") != sha256_tree(ROOT / "runtime"):
        raise AnalysisError("runtime hash mismatch")
    acl = Path("/home/current/syz/mocap/acquisition/config/zenohd_acl.yaml")
    current_acl = sha256_file(acl) if acl.is_file() else "unavailable"
    if hashes.get("acl_sha256") != current_acl:
        raise AnalysisError("ACL hash mismatch")
    return manifest


def _verify_status(bundle: Path, manifest: Mapping[str, Any]) -> list[dict[str, Any]]:
    statuses = _json_lines(bundle / "status.jsonl")
    if not statuses:
        raise AnalysisError("status.jsonl is empty")
    for row in statuses:
        if row.get("schema_version") != 1 or row.get("run_id") != manifest["run_id"]:
            raise AnalysisError("status schema or run_id mismatch")
        timestamp = row.get("timestamp_ns")
        if isinstance(timestamp, bool) or not isinstance(timestamp, int) or timestamp < 0:
            raise AnalysisError("status timestamp_ns must be a non-negative integer")
        if row.get("router_zid") not in {None, "", manifest["router_zid"]}:
            raise AnalysisError("status router_zid differs from manifest")
    for row in (item for item in statuses if item.get("event") == "safety_stop"):
        expected = set(str(item) for item in row.get("expected_executor_ids") or [])
        acked = set(str(item) for item in row.get("acked_executor_ids") or [])
        if not expected or acked != expected or row.get("ack_complete") is not True or row.get("lockout") is not True:
            raise AnalysisError("danger stop did not record all matching acknowledgements or remained unlocked")
    return statuses
def _verify_capture(bundle: Path, manifest: Mapping[str, Any]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    captures: dict[str, list[dict[str, Any]]] = {}
    for name in ("liveliness.jsonl", "protocol.jsonl"):
        rows = _json_lines(bundle / name)
        captures[name] = rows
        for row in rows:
            topic = _topic(row)
            external = topic == "mocap/aligned/hands"
            if row.get("run_id") != manifest["run_id"]:
                raise AnalysisError(f"{name} run_id mismatch")
            if external:
                if not row.get("stream_instance_id") or not isinstance(row.get("stream_sequence"), int) or not row.get("router_zid"):
                    raise AnalysisError(f"{name} aligned stream envelope is incomplete")
            elif row.get("schema_version") != 1:
                raise AnalysisError(f"{name} missing or unsupported protocol schema")
            router = row.get("router_zid")
            if not router or router != manifest["router_zid"]:
                raise AnalysisError(f"{name} router_zid is missing or differs from manifest")
            if topic.startswith("tianji/") and _authority_for_row(row, manifest) is None:
                raise AnalysisError(f"{name} publisher authority violation for {topic}")
    return captures["protocol.jsonl"], captures["liveliness.jsonl"]


def _verify_operator_result(bundle: Path) -> dict[str, Any]:
    value = _yaml(bundle / "operator_result.yaml")
    if value.get("schema_version") != BUNDLE_VERSION or value.get("outcome") not in OUTCOMES:
        raise AnalysisError("invalid operator_result outcome/schema")
    for key in ("emergency_stop", "abnormal_direction", "jitter", "noise", "collision_risk"):
        if not isinstance(value.get(key), bool):
            raise AnalysisError(f"operator_result.{key} must be boolean")
    if not isinstance(value.get("notes"), str):
        raise AnalysisError("operator_result.notes must be text")
    return value


def _verify_h5(bundle: Path, manifest: Mapping[str, Any]) -> dict[str, Any]:
    if h5py is None or np is None:
        raise AnalysisError("HDF5 analysis requires h5py and numpy")
    try:
        from pico_body_tianji.recording.session_h5 import IncompleteSessionError, SessionH5Reader
        try:
            reader = SessionH5Reader(bundle / "session.h5")
        except IncompleteSessionError as exc:
            raise AnalysisError(str(exc)) from exc
        try:
            if reader.attrs.get("router_zid") != manifest["router_zid"]:
                raise AnalysisError("session router_zid differs from manifest")
            if reader.attrs.get("schema_name") != "tianji-teleop-session" or reader.attrs.get("schema_version") != "1.0":
                raise AnalysisError("unsupported session HDF5 schema")
        finally:
            reader.close()
    except ModuleNotFoundError as exc:
        if exc.name != "zenoh":
            raise
        with h5py.File(bundle / "session.h5", "r") as file:
            attrs = {str(key): value.decode() if isinstance(value, bytes) else value for key, value in file.attrs.items()}
            if attrs.get("complete") is not True or attrs.get("schema_name") != "tianji-teleop-session" or attrs.get("schema_version") != "1.0":
                raise AnalysisError("unsupported or incomplete session HDF5 schema")
            if attrs.get("router_zid") != manifest["router_zid"]:
                raise AnalysisError("session router_zid differs from manifest")
    with h5py.File(bundle / "session.h5", "r") as file:
        return _metrics_from_h5(file, manifest)


def _sequence_metric(sequences: Iterable[Any], instances: Iterable[Any] | None = None) -> dict[str, int]:
    values = [int(value) for value in sequences]
    owners = [str(value.decode() if isinstance(value, bytes) else value) for value in instances] if instances is not None else [""] * len(values)
    if len(owners) != len(values):
        raise AnalysisError("sequence and publisher instance lengths differ")
    drops = 0
    order_errors = 0
    previous: dict[str, int] = {}
    invalid_owner: set[str] = set()
    for sequence, owner in zip(values, owners):
        if owner in previous:
            delta = sequence - previous[owner]
            if delta <= 0:
                order_errors += 1
                invalid_owner.add(owner)
            elif delta > 1 and owner not in invalid_owner:
                drops += delta - 1
        previous[owner] = sequence
    return {"drops": drops, "order_errors": order_errors}

def _folded_protocol_sequence_metric(
    rows: Iterable[tuple[str, int, int, str]],
) -> dict[str, int]:
    """Check global wire ordering while folding legal cross-topic pairs.

    Coordinators publish both arm commands with one sequence and the IK
    producer publishes proposal/solved pairs with one sequence.  Those are
    one protocol point, not duplicate output.  A repeated sequence on the
    same topic remains an error, as does a rollback after a newer sequence.
    """
    by_instance: defaultdict[str, list[tuple[int, int, str, int]]] = defaultdict(list)
    for index, (owner, timestamp, sequence, topic) in enumerate(rows):
        by_instance[str(owner)].append((int(timestamp), int(sequence), str(topic), index))
    drops = 0
    order_errors = 0
    for values in by_instance.values():
        values.sort(key=lambda item: (item[0], item[3]))
        last_by_topic: dict[str, int] = {}
        previous: int | None = None
        for _timestamp, sequence, topic, _index in values:
            previous_topic = last_by_topic.get(topic)
            if previous_topic is not None and sequence <= previous_topic:
                order_errors += 1
            last_by_topic[topic] = sequence
            if previous is None:
                previous = sequence
                continue
            if sequence < previous:
                order_errors += 1
            elif sequence > previous:
                drops += max(0, sequence - previous - 1)
                previous = sequence
            # Equal sequence from a different topic is the legal folded pair.
    return {"drops": drops, "order_errors": order_errors}


def _authority_for_row(
    row: Mapping[str, Any],
    manifest: Mapping[str, Any] | None,
) -> Mapping[str, Any] | None:
    """Return the preallocated authority owning a captured wire row."""
    if manifest is None:
        return {"component_role": "unfiltered"}
    payload = _payload(row)
    topic = _topic(row)
    router = str(payload.get("router_zid", row.get("router_zid", "")))
    if topic == "mocap/aligned/hands":
        return (
            {"component_role": "external_source", "logical_id": "acquisition"}
            if router == str(manifest.get("router_zid", ""))
            else None
        )
    if not topic or router != str(manifest.get("router_zid", "")):
        return None
    instance = str(payload.get("publisher_instance_id", row.get("publisher_instance_id", "")))
    side_match = re.search(r"/(left|right)(?:/|$)", topic)
    side = str(payload.get("side") or (side_match.group(1) if side_match else ""))
    # Unit-level protocol association callers may provide only the two
    # preallocated stream IDs.  Keep that strict fallback; real bundles always
    # carry the expanded authority contract and use the branch below.
    if not manifest.get("authority_contract"):
        ids = manifest.get("publisher_instance_ids", {})
        if ("/target/arm/" in topic and str(ids.get("source", "")) == instance) or (
            topic.endswith("solved_pose") and str(ids.get("producer_arm", "")) == instance
        ):
            return {"publisher_instance_id": instance, "side": side}
    for authority in manifest.get("authority_contract", []):
        if authority.get("publisher_instance_id") != instance:
            continue
        expected_side = authority.get("side")
        if expected_side is not None and side not in {"", str(expected_side)}:
            continue
        expanded = [
            str(template).replace("{side}", str(expected_side or side))
            for template in authority.get("topics", [])
        ]
        if topic in expanded:
            return authority
    return None


def _h5_authority(path: str, manifest: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    """Resolve the authority contract for one recorded HDF5 stream."""
    role: str | None = None
    side: str | None = None
    if path.startswith(("raw/", "target/")):
        role = "source"
    elif path.startswith("joint/command/arm") or path == "meta/session_events":
        role = "coordinator_arm"
    elif path.startswith("joint/state/arm"):
        role = "executor_arm"
    elif path.startswith("joint/command/hand/"):
        role, side = "producer_hand", path.rsplit("/", 1)[-1]
    elif path.startswith("joint/state/hand/"):
        role, side = "executor_hand", path.rsplit("/", 1)[-1]
    if role is None:
        return []
    return [
        authority
        for authority in manifest.get("authority_contract", [])
        if authority.get("component_role") == role
        and (authority.get("side") is None or authority.get("side") == side)
    ]


def _h5_mask(group: Any, path: str, manifest: Mapping[str, Any]) -> np.ndarray:
    """Select only rows owned by the profile's role/logical/side authority."""
    authorities = _h5_authority(path, manifest)
    count = int(group["time_ns"].shape[0]) if "time_ns" in group else 0
    if not authorities:
        return np.zeros(count, dtype=bool)
    logical = group.attrs.get("logical_id")
    if isinstance(logical, bytes):
        logical = logical.decode()
    expected_logical = {str(authority["logical_id"]) for authority in authorities}
    # Arm final-command wire producer is the coordinator; its launcher
    # logical id is the arm-domain authority.
    if role := next((authority.get("component_role") for authority in authorities), None):
        if role == "coordinator_arm" and path.startswith("joint/command/arm"):
            expected_logical.add("coordinator")
    if logical is not None and str(logical) not in expected_logical:
        raise AnalysisError(f"HDF5 {path} logical authority mismatch")
    instances = np.asarray(group["publisher_instance_id"][:]) if "publisher_instance_id" in group else np.asarray([], dtype=object)
    values = np.asarray([
        value.decode() if isinstance(value, (bytes, np.bytes_)) else str(value)
        for value in instances
    ])
    if len(values) != count:
        raise AnalysisError(f"HDF5 {path} publisher identity length mismatch")
    allowed = {str(authority["publisher_instance_id"]) for authority in authorities}
    if not np.isin(values, tuple(allowed)).all():
        raise AnalysisError(f"HDF5 {path} publisher authority violation")
    return np.ones(count, dtype=bool)


def _masked(group: Any, name: str, mask: np.ndarray) -> np.ndarray:
    values = np.asarray(group[name][:])
    return values[mask]


def _stream_metric(
    file: Any,
    path: str,
    sequence_name: str | None = None,
    *,
    manifest: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    if path not in file:
        return {"samples": 0, "rate_hz": 0.0, "drops": 0, "order_errors": 0}
    group = file[path]
    if "time_ns" not in group:
        return {"samples": 0, "rate_hz": 0.0, "drops": 0, "order_errors": 0}
    mask = np.ones(int(group["time_ns"].shape[0]), dtype=bool)
    if manifest is not None:
        mask = _h5_mask(group, path, manifest)
    times = np.asarray(group["time_ns"][:], dtype=np.int64)[mask]
    count = int(len(times))
    duration = float(times[-1] - times[0]) / 1e9 if count > 1 and times[-1] > times[0] else 0.0
    ordering = {"drops": 0, "order_errors": 0}
    if sequence_name and sequence_name in group and count:
        sequences = np.asarray(group[sequence_name][:])[mask]
        instances = group["publisher_instance_id"][:][mask] if "publisher_instance_id" in group else None
        ordering = _sequence_metric(sequences, instances)
    return {"samples": count, "rate_hz": count / duration if duration > 0 else 0.0, "duration_s": duration, **ordering}


def _decode(value: Any) -> Any:
    if isinstance(value, bytes) or (np is not None and isinstance(value, np.bytes_)):
        return value.decode()
    return value


def _tracking_threshold_rad(manifest: Mapping[str, Any] | None) -> float:
    """Load the active executor's finite tracking threshold."""
    if manifest is None:
        return math.radians(8.0)
    profile = str(manifest.get("profile", ""))
    profile_config = _profile_config(profile)
    configured = profile_config.get("arm_executor_config") or "executors/marvin.yaml"
    path = canonical_config_root() / str(configured)
    try:
        executor = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if "tracking_error_threshold_rad" in executor:
            threshold = float(executor["tracking_error_threshold_rad"])
        elif "maximum_tracking_error_deg" in executor:
            threshold = math.radians(float(executor["maximum_tracking_error_deg"]))
        else:
            raise KeyError("tracking threshold")
    except (OSError, TypeError, ValueError, KeyError, yaml.YAMLError) as exc:
        raise AnalysisError(f"active executor tracking threshold is unavailable: {exc}") from exc
    if not math.isfinite(threshold) or threshold <= 0.0:
        raise AnalysisError("active executor tracking threshold must be finite and positive")
    return threshold

def _metrics_from_h5(file: Any, manifest: Mapping[str, Any]) -> dict[str, Any]:
    streams = {name: _stream_metric(file, path, seq, manifest=manifest) for name, path, seq in (
        ("raw_pico_controller", "raw/pico_controller", "sequence"),
        ("raw_mocap_live", "raw/mocap_live", "stream_sequence"),
        ("raw_h5_replay", "raw/h5_replay", "sequence"),
        ("target_arm_left", "target/arm/left", "sequence"),
        ("target_arm_right", "target/arm/right", "sequence"),

        ("target_hand_left", "target/hand/left", "sequence"),
        ("target_hand_right", "target/hand/right", "sequence"),
        ("command_arm_left", "joint/command/arm/left", "sequence"),
        ("command_arm_right", "joint/command/arm/right", "sequence"),
        ("command_hand_left", "joint/command/hand/left", "sequence"),
        ("command_hand_right", "joint/command/hand/right", "sequence"),
        ("state_arm", "joint/state/arm", "sequence"),
        ("state_hand_left", "joint/state/hand/left", "sequence"),
        ("state_hand_right", "joint/state/hand/right", "sequence"),
    )}
    # Sequence is allocated per publisher across every topic.  A side/topic
    # HDF5 group is only a projection of that stream, so checking its local
    # gaps would report valid interleaving as drops.  The wire capture below
    # performs the cross-topic ordering check.
    for stream in streams.values():
        stream["drops"] = 0
        stream["order_errors"] = 0
    command_step: dict[str, float | None] = {}
    command_velocity: dict[str, float | None] = {}
    phase_velocity: dict[str, dict[str, float | None]] = {}
    saturation: dict[str, int] = {}
    hard_limit_violations: dict[str, int] = {}
    arm_config = yaml.safe_load((canonical_config_root() / "robot" / "arm.yaml").read_text(encoding="utf-8")) or {}
    lower = np.asarray(arm_config.get("lower_limits_rad", []), dtype=float)
    upper = np.asarray(arm_config.get("upper_limits_rad", []), dtype=float)
    coordinator_config = yaml.safe_load((canonical_config_root() / "coordinator" / "arm.yaml").read_text(encoding="utf-8")) or {}
    marvin_path = canonical_config_root() / "executors" / "marvin.yaml"
    marvin_config = yaml.safe_load(marvin_path.read_text(encoding="utf-8")) if marvin_path.is_file() else {}
    for side in ("left", "right"):
        path = f"joint/command/arm/{side}"
        if path not in file:
            continue
        group = file[path]
        mask = _h5_mask(group, path, manifest)
        positions = np.asarray(group["position_rad"][:], dtype=float)[mask]
        command_step[side] = float(np.max(np.abs(np.diff(positions, axis=0)))) if len(positions) > 1 else None
        times = np.asarray(group["time_ns"][:], dtype=np.int64)[mask]
        modes = [_decode(value) for value in group["mode"][:][mask]] if "mode" in group else ["teleop"] * len(positions)
        if len(positions) > 1 and np.all(np.diff(times) > 0):
            velocities = np.abs(np.diff(positions, axis=0) / (np.diff(times)[:, None] / 1e9))
            command_velocity[side] = float(np.max(velocities))
            phase_velocity[side] = {phase: (max(float(np.max(velocities[index])) for index in range(len(velocities)) if modes[index + 1] == phase) if any(modes[index + 1] == phase for index in range(len(velocities)) ) else None) for phase in ("idle", "teleop", "returning")}
        else:
            command_velocity[side] = None
            phase_velocity[side] = {phase: None for phase in ("idle", "teleop", "returning")}
        if len(lower) == positions.shape[1] == len(upper):
            hard_limit_violations[side] = int(np.sum((positions < lower) | (positions > upper)))
            saturation[side] = int(np.sum((positions <= lower + 1e-9) | (positions >= upper - 1e-9)))
        else:
            hard_limit_violations[side] = -1
    event_group = file["meta/session_events"] if "meta/session_events" in file else None
    event_mask = _h5_mask(event_group, "meta/session_events", manifest) if event_group is not None else np.asarray([], dtype=bool)
    event_states = [_decode(value) for value in event_group["state"][:][event_mask]] if event_group is not None else []
    event_times = [int(value) for value in event_group["time_ns"][:][event_mask]] if event_group is not None else []
    return_time = next((event_times[index] for index in range(len(event_states) - 1, -1, -1) if event_states[index] == "returning"), None)
    idle_time = next((event_times[index] for index in range(len(event_states)) if event_states[index] == "idle" and return_time is not None and event_times[index] >= return_time), None)
    home_time: float | str = (idle_time - return_time) / 1e9 if return_time is not None and idle_time is not None else "unavailable"
    home_tolerance = float(coordinator_config.get("home_tolerance_rad", 0.0174532925199433))
    home_feedback_ok = False
    if idle_time is not None and "joint/state/arm" in file:
        state_group = file["joint/state/arm"]
        state_mask = _h5_mask(state_group, "joint/state/arm", manifest)
        state_times = np.asarray(state_group["time_ns"][:], dtype=np.int64)[state_mask]
        state_positions = np.asarray(state_group["position_rad"][:], dtype=float)[state_mask]
        home = np.asarray(arm_config.get("left_home_rad", []) + arm_config.get("right_home_rad", []), dtype=float)
        valid = state_times >= idle_time
        home_feedback_ok = bool(np.any(valid) and len(home) == state_positions.shape[1] and np.all(np.abs(state_positions[valid][-1] - home) <= home_tolerance))
    tracking_values: list[float] = []
    if "joint/state/arm" in file:
        state_group = file["joint/state/arm"]
        state_mask = _h5_mask(state_group, "joint/state/arm", manifest)
        state_times = np.asarray(state_group["time_ns"][:], dtype=np.int64)[state_mask]
        state_positions = np.asarray(state_group["position_rad"][:], dtype=float)[state_mask]
        for side_index, side in enumerate(("left", "right")):
            command_path = f"joint/command/arm/{side}"
            if command_path not in file or not len(state_times):
                continue
            group = file[command_path]
            command_mask = _h5_mask(group, command_path, manifest)
            command_times = np.asarray(group["time_ns"][:], dtype=np.int64)[command_mask]
            command_positions = np.asarray(group["position_rad"][:], dtype=float)[command_mask]
            for timestamp, position in zip(command_times, command_positions):
                state_index = min(int(np.searchsorted(state_times, timestamp, side="left")), len(state_times) - 1)
                error = position - state_positions[state_index, side_index * 7:(side_index + 1) * 7]
                if np.isfinite(error).all():
                    tracking_values.append(float(np.max(np.abs(error))))
    hand_config = yaml.safe_load((canonical_config_root() / "robot" / "wuji_hand2.yaml").read_text(encoding="utf-8")) or {}
    zero = np.asarray(hand_config.get("zero_position_rad", []), dtype=float)
    tolerance = np.asarray(hand_config.get("zero_tolerance_rad", []), dtype=float)
    hand_zero_times: list[float] = []
    for side in ("left", "right"):
        path = f"joint/state/hand/{side}"
        if path not in file or len(zero) != 20 or len(tolerance) != 20:
            continue
        group = file[path]
        mask = _h5_mask(group, path, manifest)
        positions = np.asarray(group["position_rad"][:], dtype=float)[mask]
        times = np.asarray(group["time_ns"][:], dtype=np.int64)[mask]
        for index, position in enumerate(positions):
            if idle_time is not None and times[index] >= idle_time and np.isfinite(position).all() and np.all(np.abs(position - zero) <= tolerance):
                hand_zero_times.append(float(times[index]) / 1e9)
                break
    return {"rates": streams, "target_to_solved_error": {"samples": 0, "max_position_error_m": "unavailable", "max_orientation_error_rad": "unavailable", "note": "solved pose is collected from protocol capture"}, "joint_step_rad": command_step, "joint_velocity_rad_s": command_velocity, "phase_velocity_rad_s": phase_velocity, "saturation_count": saturation, "hard_limit_violations": hard_limit_violations, "proposal_rejections": "unavailable", "command_feedback_tracking": {"samples": len(tracking_values), "max_error_rad": max(tracking_values) if tracking_values else "unavailable"}, "tracking_threshold_rad": _tracking_threshold_rad(manifest), "home_time_s": home_time, "home_feedback_ok": home_feedback_ok, "hand_zero_time_s": max(hand_zero_times) if hand_zero_times else "unavailable", "hand_zero_feedback_ok": bool(manifest.get("hand_sides") and len(hand_zero_times) == len(manifest.get("hand_sides", []))), "fault_reasons": [], "soft_stop_reasons": [], "session_event_states": Counter(event_states)}


def _payload(row: Mapping[str, Any]) -> Mapping[str, Any]:
    value = row.get("payload")
    return value if isinstance(value, Mapping) else row


def _topic(row: Mapping[str, Any]) -> str:
    return str(row.get("topic") or row.get("key_expr") or row.get("key") or "")


def _protocol_metrics(
    protocol: Iterable[Mapping[str, Any]],
    manifest: Mapping[str, Any] | None = None,
    statuses: Iterable[Mapping[str, Any]] = (),
) -> dict[str, Any]:
    rows = list(protocol)
    status_rows = list(statuses)
    manifest_ids = manifest.get("publisher_instance_ids", {}) if manifest else {}
    source_instance = str(manifest_ids.get("source", "")) if manifest else ""
    producer_instance = str(manifest_ids.get("producer_arm", "")) if manifest else ""
    targets: dict[tuple[str, int], Mapping[str, Any]] = {}
    solved: dict[tuple[str, int], Mapping[str, Any]] = {}
    stream_counts: Counter[str] = Counter()
    stream_times: defaultdict[str, list[int]] = defaultdict(list)
    sequence_rows: list[tuple[str, int, int, str]] = []
    arm_commands: list[Mapping[str, Any]] = []
    arm_states: list[Mapping[str, Any]] = []
    hand_states: defaultdict[str, list[Mapping[str, Any]]] = defaultdict(list)
    session_states: list[Mapping[str, Any]] = []
    for row in rows + status_rows:
        if manifest is not None and _authority_for_row(row, manifest) is None:
            continue
        payload = _payload(row)
        topic = _topic(row)
        instance = str(payload.get("publisher_instance_id", row.get("publisher_instance_id", "")))
        sequence = payload.get("sequence")
        timestamp = payload.get("timestamp_ns", payload.get("time_ns"))
        if isinstance(sequence, int) and not isinstance(sequence, bool) and instance:
            order_time = int(timestamp) if isinstance(timestamp, int) and not isinstance(timestamp, bool) else len(sequence_rows)
            sequence_rows.append((instance, order_time, sequence, topic))
        side_match = re.search(r"/(left|right)(?:/|$)", topic)
        side = str(payload.get("side") or (side_match.group(1) if side_match else ""))
        stream_name = ""
        if topic == "tianji/session/state":
            session_states.append(payload)
        if topic == "mocap/aligned/hands" or payload.get("stream_instance_id"):
            stream_name = "aligned_hands"
        elif "/target/arm/" in topic:
            stream_name = f"target_arm_{side}"
            if isinstance(sequence, int) and not isinstance(sequence, bool):
                key = (side, sequence)
                if key in targets:
                    raise AnalysisError(f"duplicate authorized target key: {side}/{sequence}")
                targets[key] = payload
        elif topic.endswith("solved_pose"):
            stream_name = "solved_arm"
            target_sequence = payload.get("target_sequence")
            if isinstance(target_sequence, int) and not isinstance(target_sequence, bool):
                key = (side, target_sequence)
                if key in solved:
                    raise AnalysisError(f"duplicate authorized solved key: {side}/{target_sequence}")
                solved[key] = payload
        elif "/proposal/arm/" in topic:
            stream_name = f"proposal_arm_{side}"
        elif "/command/arm/" in topic:
            stream_name = f"command_arm_{side}"
            arm_commands.append(payload)
        elif "/command/hand/" in topic:
            stream_name = f"command_hand_{side}"
        elif topic.endswith("/state/arm"):
            stream_name = "state_arm"
            arm_states.append(payload)
        elif "/state/hand/" in topic:
            stream_name = f"state_hand_{side}"
            hand_states[side].append(payload)
        if stream_name:
            stream_counts[stream_name] += 1
            if isinstance(timestamp, int) and not isinstance(timestamp, bool):
                stream_times[stream_name].append(timestamp)
    ordering = _folded_protocol_sequence_metric(sequence_rows)
    position_errors: list[float] = []
    orientation_errors: list[float] = []
    for key, target in targets.items():
        pose = solved.get(key)
        if pose is None:
            continue
        try:
            target_position = [float(value) for value in target["position_m"]]
            solved_position = [float(value) for value in pose["position_m"]]
            difference = [a - b for a, b in zip(target_position, solved_position)]
            if len(difference) != 3 or not all(math.isfinite(value) for value in difference):
                continue
            position_errors.append(math.sqrt(sum(value * value for value in difference)))
            qa = [float(value) for value in target["orientation_xyzw"]]
            qb = [float(value) for value in pose["orientation_xyzw"]]
            na = math.sqrt(sum(value * value for value in qa)); nb = math.sqrt(sum(value * value for value in qb))
            if na == 0.0 or nb == 0.0:
                continue
            dot = abs(sum(a * b for a, b in zip(qa, qb)) / (na * nb))
            orientation_errors.append(2 * math.acos(min(1.0, max(-1.0, dot))))
        except (KeyError, TypeError, ValueError, ZeroDivisionError):
            continue
    tracking_values: list[float] = []
    state_by_time = sorted(
        (
            int(row.get("timestamp_ns", row.get("time_ns", 0))),
            row,
        )
        for row in arm_states
        if isinstance(row.get("position_rad"), (list, tuple))
    )
    for command in arm_commands:
        position = command.get("position_rad")
        if not isinstance(position, (list, tuple)) or len(position) != 7 or not state_by_time:
            continue
        command_time = int(command.get("timestamp_ns", command.get("time_ns", 0)))
        _, state = min(state_by_time, key=lambda item: abs(item[0] - command_time))
        state_position = state.get("position_rad")
        side = str(command.get("side", ""))
        offset = 0 if side == "left" else 7
        if not isinstance(state_position, (list, tuple)) or len(state_position) != 14:
            continue
        try:
            errors = [abs(float(a) - float(b)) for a, b in zip(position, state_position[offset:offset + 7])]
            if all(math.isfinite(value) for value in errors):
                tracking_values.append(max(errors))
        except (TypeError, ValueError):
            continue
    hand_zero_ok: dict[str, bool] = {}
    if manifest:
        try:
            hand_config = yaml.safe_load((canonical_config_root() / "robot" / "wuji_hand2.yaml").read_text(encoding="utf-8")) or {}
            zero = [float(value) for value in hand_config.get("zero_position_rad", [])]
            tolerance = [float(value) for value in hand_config.get("zero_tolerance_rad", [])]
            return_times = [
                int(row.get("timestamp_ns", row.get("time_ns", 0)))
                for row in session_states
                if row.get("state") == "returning"
            ]
            last_return = max(return_times) if return_times else None
            idle_times = [
                int(row.get("timestamp_ns", row.get("time_ns", 0)))
                for row in session_states
                if row.get("state") == "idle"
                and (last_return is None or int(row.get("timestamp_ns", row.get("time_ns", 0))) >= last_return)
            ]
            idle_after_return = max(idle_times) if idle_times else None
            for side in manifest.get("hand_sides", []):
                hand_zero_ok[side] = len(zero) == len(tolerance) == 20 and idle_after_return is not None and any(
                    isinstance(row.get("position_rad"), (list, tuple))
                    and len(row["position_rad"]) == 20
                    and int(row.get("timestamp_ns", row.get("time_ns", 0))) >= idle_after_return
                    and all(math.isfinite(float(value)) and abs(float(value) - z) <= t for value, z, t in zip(row["position_rad"], zero, tolerance))
                    for row in hand_states.get(side, [])
                )
        except (OSError, TypeError, ValueError):
            hand_zero_ok = {}
    arm_limits = yaml.safe_load((canonical_config_root() / "robot" / "arm.yaml").read_text(encoding="utf-8")) or {}
    lower = [float(value) for value in arm_limits.get("lower_limits_rad", [])]
    upper = [float(value) for value in arm_limits.get("upper_limits_rad", [])]
    protocol_step: dict[str, float | None] = {}
    protocol_limits: dict[str, int] = {}
    protocol_phase_velocity: dict[str, dict[str, float | None]] = {}
    grouped_commands: defaultdict[str, list[Mapping[str, Any]]] = defaultdict(list)
    for command in arm_commands:
        grouped_commands[str(command.get("side", ""))].append(command)
    for side, values in grouped_commands.items():
        values.sort(key=lambda row: int(row.get("timestamp_ns", row.get("time_ns", 0))))
        positions = [row.get("position_rad") for row in values]
        deltas = []
        violations = 0
        for position in positions:
            if not isinstance(position, (list, tuple)) or len(position) != 7:
                continue
            try:
                if len(lower) == len(upper) == 7 and (not all(math.isfinite(float(item)) for item in position) or any(float(item) < lo or float(item) > hi for item, lo, hi in zip(position, lower, upper))):
                    violations += 1
            except (TypeError, ValueError):
                violations += 1
        for previous, current in zip(positions, positions[1:]):
            if isinstance(previous, (list, tuple)) and isinstance(current, (list, tuple)) and len(previous) == len(current) == 7:
                try:
                    deltas.append(max(abs(float(a) - float(b)) for a, b in zip(previous, current)))
                except (TypeError, ValueError):
                    continue
        protocol_step[side] = max(deltas) if deltas else None
        protocol_limits[side] = violations
        phase_values: dict[str, list[float]] = defaultdict(list)
        for previous, current in zip(values, values[1:]):
            previous_position = previous.get("position_rad"); current_position = current.get("position_rad")
            previous_time = int(previous.get("timestamp_ns", previous.get("time_ns", 0))); current_time = int(current.get("timestamp_ns", current.get("time_ns", 0)))
            if current_time <= previous_time or not isinstance(previous_position, (list, tuple)) or not isinstance(current_position, (list, tuple)):
                continue
            try:
                speed = max(abs(float(a) - float(b)) for a, b in zip(previous_position, current_position)) / ((current_time - previous_time) / 1e9)
                phase = str(current.get("mode", "teleop"))
                if math.isfinite(speed):
                    phase_values[phase].append(speed)
            except (TypeError, ValueError, ZeroDivisionError):
                continue
        protocol_phase_velocity[side] = {phase: (max(values) if values else None) for phase, values in ((phase, phase_values.get(phase, [])) for phase in ("idle", "teleop", "returning"))}
    rates = {}
    for name, count in stream_counts.items():
        times = stream_times.get(name, [])
        duration = (times[-1] - times[0]) / 1e9 if len(times) > 1 and times[-1] > times[0] else 0.0
        rates[name] = {"samples": count, "rate_hz": count / duration if duration > 0 else 0.0, "drops": 0, "order_errors": 0}
    return {
        "rates": rates,
        "target_to_solved_error": {
            "samples": len(position_errors),
            "max_position_error_m": max(position_errors) if position_errors else "unavailable",
            "max_orientation_error_rad": max(orientation_errors) if orientation_errors else "unavailable",
            "note": "protocol target_sequence association",
        },
        "protocol_order_errors": ordering["order_errors"],
        "protocol_drops": ordering["drops"],
        "command_feedback_tracking": {
            "samples": len(tracking_values),
            "max_error_rad": max(tracking_values) if tracking_values else "unavailable",
        },
        "tracking_threshold_rad": _tracking_threshold_rad(manifest),
        "joint_step_rad": protocol_step,
        "hard_limit_violations": protocol_limits,
        "phase_velocity_rad_s": protocol_phase_velocity,
        "hand_zero_feedback_ok": bool(hand_zero_ok) and all(hand_zero_ok.values()),
    }


def _authority_statuses(statuses: Iterable[Mapping[str, Any]], manifest: Mapping[str, Any]) -> tuple[set[str], list[str]]:
    expected = list(manifest.get("authority_contract", []))
    matched: dict[str, list[Mapping[str, Any]]] = defaultdict(list)
    for row in statuses:
        topic = _topic(row)
        side_match = re.search(r"/(left|right)(?:/|$)", topic)
        diagnostics = row.get("diagnostics")
        diagnostic_side = diagnostics.get("side") if isinstance(diagnostics, Mapping) else None
        side = row.get("side") or diagnostic_side or (side_match.group(1) if side_match else "")
        role = row.get("component_role")
        logical = row.get("component_id") or row.get("logical_id")
        instance = str(row.get("publisher_instance_id", ""))
        router = str(row.get("router_zid", ""))
        if role is None and topic.startswith("tianji/executor/hand/"):
            role, logical = "executor_hand", f"wuji_{side}"
        sequence = row.get("sequence")
        if isinstance(sequence, bool) or not isinstance(sequence, int):
            continue
        for authority in expected:
            if authority["publisher_instance_id"] != instance or authority["router_zid"] != router or authority["component_role"] != role or authority["logical_id"] != logical:
                continue
            expected_side = authority.get("side")
            # ComponentStatus is intentionally side-less.  A shared hand
            # producer identity may therefore prove every authorized active
            # side; typed executor status still carries its side.
            if expected_side is not None and side not in {"", None, expected_side}:
                continue
            matched.setdefault(json.dumps(authority, sort_keys=True), []).append(row)
    found: set[str] = set()
    unhealthy: list[str] = []
    for authority in expected:
        key = json.dumps(authority, sort_keys=True)
        rows = matched.get(key, [])
        if not rows:
            continue
        rows = sorted(rows, key=lambda row: int(row["sequence"]))
        latest = rows[-1]
        found.add(key)
        # Startup/armed ready=false is a valid lifecycle state.  Only the
        # latest monotonic status decides health, and a recorder's normal
        # close phase is healthy when it carries no error.
        if latest.get("healthy") is not True or (
            latest.get("phase") == "fault" and latest.get("error")
        ):
            unhealthy.append(f"{authority['component_role']}:{authority['logical_id']}:{authority.get('side') or ''}")
    return found, unhealthy


def _authority_capture(
    protocol: Iterable[Mapping[str, Any]],
    liveliness: Iterable[Mapping[str, Any]],
    manifest: Mapping[str, Any],
    statuses: Iterable[Mapping[str, Any]] = (),
) -> tuple[set[str], set[str]]:
    expected = list(manifest.get("authority_contract", []))
    protocol_keys: set[str] = set()
    live_keys: set[str] = set()

    def key(value: Mapping[str, Any]) -> str:
        return json.dumps(value, sort_keys=True)

    for row in list(protocol) + list(statuses):
        payload = _payload(row)
        topic = _topic(row)
        instance = str(payload.get("publisher_instance_id", row.get("publisher_instance_id", "")))
        router = str(payload.get("router_zid", row.get("router_zid", "")))
        side_match = re.search(r"/(left|right)(?:/|$)", topic)
        side = str(payload.get("side") or (side_match.group(1) if side_match else ""))
        if not router:
            continue
        for authority in expected:
            if authority["publisher_instance_id"] != instance or authority["router_zid"] != router:
                continue
            expected_side = authority.get("side")
            if expected_side is not None and side not in {"", expected_side}:
                continue
            topics = []
            for template in authority.get("topics", []):
                if "{side}" in template:
                    topics.append(template.replace("{side}", expected_side or side))
                else:
                    topics.append(template)
            if topic in topics:
                protocol_keys.add(key(authority))
    for row in liveliness:
        value = str(row.get("key_expr") or row.get("topic") or "")
        for authority in expected:
            if value == authority["liveliness"]:
                live_keys.add(key(authority))
    return protocol_keys, live_keys


def _child_log_labels(bundle: Path, manifest: Mapping[str, Any]) -> set[str]:
    names = {path.name.lower() for path in (bundle / "logs").glob("*") if path.is_file() and path.name != "validation.log"}; labels: set[str] = set()
    for authority in manifest.get("authority_contract", []):
        role = authority["component_role"]; side = authority.get("side"); logical = str(authority.get("logical_id", ""))
        patterns = [role.replace("_", "-"), role.replace("_", ""), role.split("_")[-1], role.split("_")[0], logical]
        if side: patterns += [f"{role}_{side}", f"{role}-{side}", f"{role.split('_')[-1]}_{side}", f"{logical}_{side}"]
        if any(any(pattern and pattern in name for pattern in patterns) for name in names):
            labels.add(json.dumps(authority, sort_keys=True))
    return labels


def _verify_operator_events(bundle: Path, manifest: Mapping[str, Any]) -> list[dict[str, Any]]:
    values = _json_lines(bundle / "operator_events.jsonl")
    for row in values:
        if row.get("schema_version") != 1 or row.get("run_id") != manifest["run_id"] or not isinstance(row.get("time_ns"), int) or row["time_ns"] < 0 or not isinstance(row.get("event"), str) or not row["event"]:
            raise AnalysisError("invalid operator event")
    times = [int(row["time_ns"]) for row in values]
    if times != sorted(times) or any(left == right for left, right in zip(times, times[1:])):
        raise AnalysisError("operator event time is not strictly monotonic")
    return values

def _validate_pass_gate(bundle: Path, manifest: Mapping[str, Any], case: Mapping[str, Any], statuses: list[Mapping[str, Any]], protocol: list[Mapping[str, Any]], liveliness: list[Mapping[str, Any]], metrics: Mapping[str, Any], events: list[Mapping[str, Any]], operator: Mapping[str, Any]) -> None:
    if manifest.get("fake"):
        raise AnalysisError("fake/headless bundle cannot be pass")
    if manifest.get("case_id") == "acquisition_live":
        aligned = [row for row in protocol if _topic(row) == "mocap/aligned/hands"]
        observed = [row for row in statuses if row.get("event") == "acquisition_observation" and row.get("healthy") is True and row.get("complete") is True]
        if not aligned or not observed:
            raise AnalysisError("acquisition pass requires real aligned-hands samples and healthy capture status")
        instances = {str(row.get("stream_instance_id", "")) for row in aligned}
        routers = {str(row.get("router_zid", "")) for row in aligned}
        if not any(str(row.get("stream_instance_id", "")) in instances and str(row.get("router_zid", "")) == manifest["router_zid"] and int(row.get("samples", 0)) >= 3 for row in observed):
            raise AnalysisError("acquisition observation status does not match captured stream")
        sequences = [row.get("stream_sequence") for row in sorted(aligned, key=lambda row: int(row.get("time_ns", 0)))]
        times = [int(row.get("time_ns", 0)) for row in sorted(aligned, key=lambda row: int(row.get("time_ns", 0)))]
        if len(instances) != 1 or "" in instances or routers != {manifest["router_zid"]} or len(aligned) < 3:
            raise AnalysisError("acquisition stream instance/router evidence is incomplete")
        if any(isinstance(value, bool) or not isinstance(value, int) for value in sequences) or any(b != a + 1 for a, b in zip(sequences, sequences[1:])):
            raise AnalysisError("acquisition stream sequence is not continuous")
        duration = (times[-1] - times[0]) / 1e9 if len(times) > 1 else 0.0
        if duration <= 0 or (len(times) - 1) / duration < 50.0:
            raise AnalysisError("acquisition stream is below the required 60Hz observation rate")
        if not any(path.name != "validation.log" for path in (bundle / "logs").glob("*") if path.is_file()):
            raise AnalysisError("acquisition pass requires capture log evidence")
        return
    expected = list(manifest.get("authority_contract", []))
    found_status, unhealthy = _authority_statuses(statuses, manifest)
    found_protocol, found_live = _authority_capture(protocol, liveliness, manifest, statuses)
    expected_keys = {json.dumps(item, sort_keys=True) for item in expected}
    if found_status != expected_keys:
        raise AnalysisError("pass requires formal healthy/ready status for every authority")
    fault_case = str(manifest.get("case_id", "")).startswith("fault_recovery")
    if unhealthy and not fault_case:
        raise AnalysisError("unhealthy/fault authority status prevents pass: " + ",".join(sorted(set(unhealthy))))
    required_protocol = {
        json.dumps(item, sort_keys=True)
        for item in expected
        if item["component_role"] != "recorder"
        and not (manifest.get("profile") == "wuji_direct_real" and item["component_role"] == "source")
        and not (manifest.get("profile") == "joint_replay_sim" and item["component_role"] == "source")
    }
    if not required_protocol.issubset(found_protocol):
        raise AnalysisError("pass requires protocol evidence for every active authority")
    if found_live != expected_keys:
        raise AnalysisError("pass requires exact liveliness token for every authority")
    if _child_log_labels(bundle, manifest) != expected_keys:
        raise AnalysisError("pass requires a child log for every authority")
    evidence = REQUIRED_EVIDENCE.get(manifest["case_id"], {"streams": set(), "checks": set()})
    rates = dict(metrics.get("rates", {}))
    for stream in evidence["streams"]:
        if not isinstance(rates.get(stream), Mapping) or rates[stream].get("samples", 0) <= 0:
            raise AnalysisError(f"pass requires samples for {stream}")
    if any(int(value.get("order_errors", 0)) or int(value.get("drops", 0)) for value in rates.values() if isinstance(value, Mapping)) or metrics.get("protocol_order_errors", 0):
        raise AnalysisError("pass cannot contain sequence duplicate/rollback/drop errors")
    for side, value in metrics.get("hard_limit_violations", {}).items():
        if value != 0:
            raise AnalysisError(f"hard joint limit violation on {side}")
    coordinator = yaml.safe_load((canonical_config_root() / "coordinator" / "arm.yaml").read_text(encoding="utf-8")) or {}
    max_step = float(coordinator["maximum_command_step_rad"])
    for side, value in metrics.get("joint_step_rad", {}).items():
        if value is not None and float(value) > max_step:
            raise AnalysisError(f"joint step exceeds configured limit for {side}")
    marvin = yaml.safe_load((canonical_config_root() / "executors" / "marvin.yaml").read_text(encoding="utf-8")) or {}
    max_teleop = math.radians(float(marvin.get("maximum_teleop_speed_deg_s", 0.0))); max_home = float(coordinator["home_max_speed_rad_s"])
    for side, phases in metrics.get("phase_velocity_rad_s", {}).items():
        for phase, value in phases.items():
            if value is not None and float(value) > (max_home if phase in {"idle", "returning"} else max_teleop) * float(case["velocity_ratio"]):
                raise AnalysisError(f"{phase} velocity exceeds configured limit for {side}")
    target = metrics.get("target_to_solved_error", {}); ik = yaml.safe_load((canonical_config_root() / "producers" / "ik.yaml").read_text(encoding="utf-8")) or {}
    if "target_to_solved" in evidence["checks"] and (target.get("max_position_error_m") == "unavailable" or float(target.get("max_position_error_m", math.inf)) > float(ik.get("position_tolerance_m", 0.001)) or target.get("max_orientation_error_rad") == "unavailable" or float(target.get("max_orientation_error_rad", math.inf)) > float(ik.get("orientation_tolerance_rad", 0.01))):
        raise AnalysisError("target-to-solved error is missing or exceeds configured tolerance")
    tracking = metrics.get("command_feedback_tracking", {}); threshold = float(metrics.get("tracking_threshold_rad", math.inf))
    if tracking.get("max_error_rad") == "unavailable" or float(tracking.get("max_error_rad", math.inf)) > threshold:
        raise AnalysisError("command-to-feedback tracking evidence is missing or exceeds configured threshold")
    if "home_feedback" in evidence["checks"] and metrics.get("home_feedback_ok") is not True:
        raise AnalysisError("pass requires final arm feedback in Home tolerance after return")
    if "hand_zero" in evidence["checks"] and metrics.get("hand_zero_feedback_ok") is not True:
        raise AnalysisError("pass requires every enabled hand feedback in zero tolerance after return")
    if not fault_case and (metrics.get("fault_reasons") or metrics.get("soft_stop_reasons")):
        raise AnalysisError("fault or soft-stop prevents pass")
    if fault_case:
        stop_rows = [row for row in statuses if row.get("event") == "safety_stop"]
        if not stop_rows:
            raise AnalysisError("fault recovery requires a latched SafetyStop evidence row")
        for row in stop_rows:
            expected_ids = {str(item) for item in row.get("expected_executor_ids") or []}
            acked_ids = {str(item) for item in row.get("acked_executor_ids") or []}
            stop_evidence = row.get("executor_safety_evidence")
            if not expected_ids or acked_ids != expected_ids or row.get("ack_complete") is not True or row.get("lockout") is not True or row.get("new_motion_commands_after_stop") is not False:
                raise AnalysisError("fault recovery stop lacks complete matching ack/no-motion/lockout evidence")
            if not isinstance(stop_evidence, Mapping) or stop_evidence.get("same_tick_ack") is not True or stop_evidence.get("unhealthy") is not True or stop_evidence.get("no_motion_commands") is not True:
                raise AnalysisError("fault recovery stop lacks executor unhealthy/no-motion evidence")
    dangerous_names = {"wrong_direction_or_side", "physical_limit", "collision_risk", "feedback_stale", "tracking_threshold", "device_or_servo_error", "duplicate_authority", "router_zid_change", "emergency_stop"}
    if dangerous_names.intersection(str(item.get("event")) for item in events) or any(operator.get(key) is True for key in ("emergency_stop", "abnormal_direction", "collision_risk")):
        raise AnalysisError("dangerous operator event prevents pass")
    for row in statuses:
        if row.get("event") == "safety_stop" and (row.get("new_motion_commands_after_stop") is not False or row.get("ack_complete") is not True or row.get("lockout") is not True or not (row.get("executor_safety_evidence") or row.get("sdk_no_motion_evidence"))):
            raise AnalysisError("SafetyStop lacks same-tick no-motion/lockout evidence")


def analyze_bundle(bundle: Path, matrix: Mapping[str, Any]) -> dict[str, Any]:
    manifest_preview = _yaml(bundle / "manifest.yaml"); session_required = manifest_preview.get("profile") not in SESSION_OPTIONAL_PROFILES; required = set(REQUIRED_FILES)
    if session_required: required.add("session.h5")
    missing = [name for name in required if not (bundle / name).is_file()]
    if missing: raise AnalysisError(f"bundle missing required files: {sorted(missing)}")
    _verify_checksums(bundle, session_required=session_required); manifest = _verify_manifest(bundle, matrix); statuses = _verify_status(bundle, manifest); protocol, liveliness = _verify_capture(bundle, manifest); operator = _verify_operator_result(bundle); events = _verify_operator_events(bundle, manifest)
    if session_required or (bundle / "session.h5").is_file(): metrics = _verify_h5(bundle, manifest)
    else: metrics = {"rates": {}, "target_to_solved_error": {"samples": 0, "max_position_error_m": "unavailable", "max_orientation_error_rad": "unavailable"}, "joint_step_rad": {}, "joint_velocity_rad_s": {}, "phase_velocity_rad_s": {}, "saturation_count": {}, "hard_limit_violations": {}, "proposal_rejections": "unavailable", "command_feedback_tracking": {"samples": 0, "max_error_rad": "unavailable"}, "tracking_threshold_rad": _tracking_threshold_rad(manifest), "home_time_s": "unavailable", "home_feedback_ok": False, "hand_zero_time_s": "unavailable", "hand_zero_feedback_ok": False, "fault_reasons": [], "soft_stop_reasons": [], "session_event_states": {}}
    protocol_metrics = _protocol_metrics(protocol, manifest, statuses)
    metrics.setdefault("rates", {}).update({key: value for key, value in protocol_metrics["rates"].items() if key not in metrics["rates"]})
    if protocol_metrics["target_to_solved_error"]["samples"]:
        metrics["target_to_solved_error"] = protocol_metrics["target_to_solved_error"]
    if protocol_metrics["command_feedback_tracking"]["samples"]:
        metrics["command_feedback_tracking"] = protocol_metrics["command_feedback_tracking"]
    if protocol_metrics["joint_step_rad"]:
        metrics["joint_step_rad"].update(protocol_metrics["joint_step_rad"])
    if protocol_metrics["hard_limit_violations"]:
        metrics["hard_limit_violations"].update(protocol_metrics["hard_limit_violations"])
    if protocol_metrics["phase_velocity_rad_s"]:
        metrics["phase_velocity_rad_s"].update(protocol_metrics["phase_velocity_rad_s"])
    if protocol_metrics["hand_zero_feedback_ok"]:
        metrics["hand_zero_feedback_ok"] = True
    metrics["protocol_order_errors"] = protocol_metrics["protocol_order_errors"]
    metrics["protocol_drops"] = protocol_metrics["protocol_drops"]
    metrics["proposal_rejections"] = len([row for row in statuses if row.get("event") in {"proposal_rejected", "producer_unhealthy", "action_rejected"}]) or metrics.get("proposal_rejections", "unavailable")
    metrics["fault_reasons"] = [str(row.get("reason") or row.get("error")) for row in statuses if row.get("event") in {"fault", "fault_latched"} and (row.get("reason") or row.get("error"))]
    metrics["soft_stop_reasons"] = [str(row.get("reason") or row.get("error")) for row in statuses if row.get("event") in {"soft_stop", "safety_stop"} and (row.get("reason") or row.get("error"))]
    if operator["outcome"] == "pass": _validate_pass_gate(bundle, manifest, matrix["cases"][manifest["case_id"]], statuses, protocol, liveliness, metrics, events, operator)
    return {"schema_name": "tianji-validation-analysis", "schema_version": BUNDLE_VERSION, "run_id": manifest["run_id"], "case_id": manifest["case_id"], "profile": manifest["profile"], "operator_outcome": operator["outcome"], "physical_validation": bool(manifest["required_capability"] == "real" and not manifest.get("fake", False)), "status_records": len(statuses), "operator_events": events, "stop_criteria": matrix["cases"][manifest["case_id"]]["stop_criteria"], "metrics": metrics, "config": {"robot": "robot/arm.yaml", "coordinator": "coordinator/arm.yaml", "hashes": manifest["hashes"]}}


def _markdown(report: Mapping[str, Any]) -> str:
    metrics = report["metrics"]; lines = [f"# Validation analysis: `{report['case_id']}`", "", f"- Run: `{report['run_id']}`", f"- Profile: `{report['profile']}`", f"- Operator outcome: **{report['operator_outcome']}**", "", "## Stream rates and drops", "", "| Stream | Samples | Rate (Hz) | Drops | Order errors |", "|---|---:|---:|---:|---:|"]
    for name, value in metrics["rates"].items(): lines.append(f"| {name} | {value['samples']} | {value['rate_hz']:.3f} | {value.get('drops', 0)} | {value.get('order_errors', 0)} |")
    lines += ["", "## Metrics", "", f"- Joint step: `{metrics.get('joint_step_rad')}`", f"- Joint velocity: `{metrics.get('joint_velocity_rad_s')}`", f"- Hard-limit violations: `{metrics.get('hard_limit_violations')}`", f"- Target → solved: `{metrics.get('target_to_solved_error')}`", f"- Command → feedback: `{metrics.get('command_feedback_tracking')}`", f"- Home: `{metrics.get('home_time_s')}`", f"- Hand zero: `{metrics.get('hand_zero_time_s')}`", f"- Faults: `{metrics.get('fault_reasons')}`", f"- Soft stops: `{metrics.get('soft_stop_reasons')}`", "", "## Operator events", ""]
    lines.extend(f"- `{event['time_ns']}` **{event['event']}**: {event.get('details', '')}" for event in report["operator_events"]) if report["operator_events"] else lines.append("- none")
    return "\n".join(lines) + "\n"


def analyze(root: Path) -> list[dict[str, Any]]:
    matrix = load_matrix(MATRIX_PATH); reports = []
    for bundle in _bundle_paths(root):
        report = analyze_bundle(bundle, matrix); (bundle / "analysis.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, default=lambda value: dict(value)), encoding="utf-8"); (bundle / "analysis.md").write_text(_markdown(report), encoding="utf-8"); reports.append(report)
    return reports


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument("root", type=Path)
    try: reports = analyze(parser.parse_args(argv).root.expanduser())
    except (AnalysisError, ValueError, OSError, yaml.YAMLError) as exc:
        print(f"validation-analyze failed: {exc}", file=sys.stderr); return 1
    print(json.dumps({"bundles": len(reports), "run_ids": [item["run_id"] for item in reports]}, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
