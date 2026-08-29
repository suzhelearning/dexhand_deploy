#!/usr/bin/env python3
"""Validate and summarize Tianji validation result bundles.

The analyzer is intentionally fail-closed: malformed artifacts, changed
configuration, a missing danger-stop acknowledgement, or an incomplete HDF5
record prevent analysis output from being accepted.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable, Mapping

import h5py
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC_ROOT = ROOT / "src" / "pico_body_tianji"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))
from pico_body_tianji.config_loader import canonical_config_root
from scripts.validation.run_case import BUNDLE_SCHEMA, BUNDLE_VERSION, MATRIX_PATH, load_matrix, sha256_file, sha256_tree

REQUIRED_FILES = frozenset({
    "manifest.yaml", "status.jsonl", "operator_events.jsonl",
    "liveliness.jsonl", "protocol.jsonl", "operator_result.yaml", "checksums.sha256",
})
SESSION_OPTIONAL_PROFILES = frozenset({"target_replay_sim", "joint_replay_sim", "acquisition_live"})
OUTCOMES = frozenset({"pass", "fail", "aborted"})


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
    checksum = bundle / "checksums.sha256"
    try:
        lines = checksum.read_text(encoding="utf-8").splitlines()
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
        actual = sha256_file(bundle / name)
        if actual != expected_hash:
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
    for name in ("run_id", "router_zid", "started_at", "ended_at", "exit_reason"):
        if not manifest.get(name):
            raise AnalysisError(f"manifest field is missing: {name}")
    hashes = manifest.get("hashes")
    if not isinstance(hashes, dict):
        raise AnalysisError("manifest hashes are missing")
    for name in ("config_sha256", "runtime_sha256", "acl_sha256"):
        if manifest.get(name) != hashes.get(name):
            raise AnalysisError(f"manifest {name} alias mismatch")
    current_config = sha256_tree(canonical_config_root(), suffixes={".yaml", ".yml"})
    if hashes.get("config_sha256") != current_config:
        raise AnalysisError("canonical config hash mismatch")
    current_runtime = sha256_tree(ROOT / "runtime")
    if hashes.get("runtime_sha256") != current_runtime:
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
    safety = [row for row in statuses if row.get("event") == "safety_stop"]
    for row in safety:
        expected = set(row.get("expected_executor_ids") or [])
        acked = set(row.get("acked_executor_ids") or [])
        if not expected or expected != acked or row.get("ack_complete") is not True or row.get("new_motion_commands_after_stop") is not False or row.get("lockout") is not True:
            raise AnalysisError("danger stop did not receive every matching ack or remained unlocked")
    return statuses


def _verify_capture(bundle: Path, manifest: Mapping[str, Any]) -> None:
    for name in ("liveliness.jsonl", "protocol.jsonl"):
        rows = _json_lines(bundle / name)
        for row in rows:
            if row.get("schema_version") != 1 or row.get("run_id") != manifest["run_id"]:
                raise AnalysisError(f"{name} schema or run_id mismatch")


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
    # Importing recording.session_h5 imports the optional Zenoh-backed
    # recorder package. Keep list/preflight/analyze usable in a plain Python
    # environment while using the canonical strict reader whenever available.
    try:
        from pico_body_tianji.recording.session_h5 import IncompleteSessionError, SessionH5Reader
    except ModuleNotFoundError as exc:
        if exc.name != "zenoh":
            raise
        with h5py.File(bundle / "session.h5", "r") as file:
            attrs = {str(key): value.decode() if isinstance(value, bytes) else value for key, value in file.attrs.items()}
            if attrs.get("complete") is not True or attrs.get("schema_name") != "tianji-teleop-session" or attrs.get("schema_version") != "1.0":
                raise AnalysisError("unsupported or incomplete session HDF5 schema")
            if attrs.get("router_zid") != manifest["router_zid"]:
                raise AnalysisError("session router_zid differs from manifest")
            return _metrics_from_h5(file, manifest)
    try:
        reader = SessionH5Reader(bundle / "session.h5")
    except IncompleteSessionError as exc:
        raise AnalysisError(str(exc)) from exc
    try:
        attrs = reader.attrs
        if attrs.get("router_zid") != manifest["router_zid"]:
            raise AnalysisError("session router_zid differs from manifest")
        if attrs.get("schema_name") != "tianji-teleop-session" or attrs.get("schema_version") != "1.0":
            raise AnalysisError("unsupported session HDF5 schema")
        with h5py.File(bundle / "session.h5", "r") as file:
            return _metrics_from_h5(file, manifest)
    finally:
        reader.close()


def _stream_metric(file: h5py.File, path: str, sequence_name: str | None = None) -> dict[str, Any]:
    if path not in file:
        return {"samples": 0, "rate_hz": 0.0, "drops": 0}
    group = file[path]
    if "time_ns" not in group:
        return {"samples": 0, "rate_hz": 0.0, "drops": 0}
    times = np.asarray(group["time_ns"][:], dtype=np.int64)
    count = int(len(times))
    duration = float(times[-1] - times[0]) / 1e9 if count > 1 and times[-1] > times[0] else 0.0
    drops = 0
    if sequence_name and sequence_name in group and count > 1:
        seq = np.asarray(group[sequence_name][:], dtype=np.int64)
        drops = int(np.maximum(np.diff(seq) - 1, 0).sum())
    return {"samples": count, "rate_hz": count / duration if duration > 0 else 0.0, "drops": drops, "duration_s": duration}


def _finite_max(dataset: h5py.Dataset | None, *, absolute: bool = True) -> float | None:
    if dataset is None:
        return None
    values = np.asarray(dataset[:], dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    if absolute:
        values = np.abs(values)
    return float(np.max(values))


def _metrics_from_h5(file: h5py.File, manifest: Mapping[str, Any]) -> dict[str, Any]:
    streams = {
        "raw_pico_controller": _stream_metric(file, "raw/pico_controller", "sequence"),
        "raw_mocap_live": _stream_metric(file, "raw/mocap_live", "stream_sequence"),
        "raw_h5_replay": _stream_metric(file, "raw/h5_replay", "sequence"),
        "target_arm_left": _stream_metric(file, "target/arm/left", "sequence"),
        "target_arm_right": _stream_metric(file, "target/arm/right", "sequence"),
        "target_hand_left": _stream_metric(file, "target/hand/left", "sequence"),
        "target_hand_right": _stream_metric(file, "target/hand/right", "sequence"),
        "command_arm_left": _stream_metric(file, "joint/command/arm/left", "sequence"),
        "command_arm_right": _stream_metric(file, "joint/command/arm/right", "sequence"),
        "command_hand_left": _stream_metric(file, "joint/command/hand/left", "sequence"),
        "command_hand_right": _stream_metric(file, "joint/command/hand/right", "sequence"),
        "state_arm": _stream_metric(file, "joint/state/arm", "sequence"),
        "state_hand_left": _stream_metric(file, "joint/state/hand/left", "sequence"),
        "state_hand_right": _stream_metric(file, "joint/state/hand/right", "sequence"),
    }
    command_step: dict[str, float | None] = {}
    command_velocity: dict[str, float | None] = {}
    saturation: dict[str, int] = {}
    arm_config = yaml.safe_load((canonical_config_root() / "robot" / "arm.yaml").read_text(encoding="utf-8")) or {}
    lower = np.asarray(arm_config.get("lower_limits_rad", []), dtype=float)
    upper = np.asarray(arm_config.get("upper_limits_rad", []), dtype=float)
    for side in ("left", "right"):
        path = f"joint/command/arm/{side}"
        if path not in file:
            continue
        group = file[path]
        positions = np.asarray(group["position_rad"][:], dtype=float)
        command_step[side] = float(np.max(np.abs(np.diff(positions, axis=0)))) if len(positions) > 1 else None
        times = np.asarray(group["time_ns"][:], dtype=np.int64)
        if len(positions) > 1 and np.all(np.diff(times) > 0):
            command_velocity[side] = float(np.max(np.abs(np.diff(positions, axis=0) / (np.diff(times)[:, None] / 1e9))))
        else:
            command_velocity[side] = None
        saturation[side] = int(np.sum((positions <= lower + 1e-9) | (positions >= upper - 1e-9))) if len(lower) == positions.shape[1] else 0
    target_solved = {"samples": 0, "max_position_error_m": "unavailable", "max_orientation_error_rad": "unavailable", "note": "session-v1 stores no solved-pose stream"}
    events = _stream_metric(file, "meta/session_events", None)
    event_states: list[str] = []
    event_times: list[int] = []
    if "meta/session_events/state" in file:
        event_states = [value.decode() if isinstance(value, bytes) else str(value) for value in file["meta/session_events/state"][:]]
        event_times = [int(value) for value in file["meta/session_events/time_ns"][:]]
    home_time = None
    if "teleop" in event_states:
        start_index = event_states.index("teleop")
        for index in range(start_index + 1, len(event_states)):
            if event_states[index] == "idle":
                home_time = max(0.0, (event_times[index] - event_times[start_index]) / 1e9)
                break
    tracking_values: list[float] = []
    if "joint/state/arm" in file:
        state_group = file["joint/state/arm"]
        state_times = np.asarray(state_group["time_ns"][:], dtype=np.int64)
        state_positions = np.asarray(state_group["position_rad"][:], dtype=float)
        for side_index, side in enumerate(("left", "right")):
            command_path = f"joint/command/arm/{side}"
            if command_path not in file or not len(state_times):
                continue
            command_group = file[command_path]
            command_times = np.asarray(command_group["time_ns"][:], dtype=np.int64)
            command_positions = np.asarray(command_group["position_rad"][:], dtype=float)
            for timestamp, position in zip(command_times, command_positions):
                state_index = int(np.searchsorted(state_times, timestamp, side="left"))
                state_index = min(state_index, len(state_times) - 1)
                state_position = state_positions[state_index, side_index * 7:(side_index + 1) * 7]
                error = np.asarray(position) - state_position
                if np.isfinite(error).all():
                    tracking_values.append(float(np.max(np.abs(error))))
    hand_zero_times: list[float] = []
    hand_config_path = canonical_config_root() / "robot" / "wuji_hand2.yaml"
    hand_config = yaml.safe_load(hand_config_path.read_text(encoding="utf-8")) if hand_config_path.is_file() else {}
    zero = np.asarray(hand_config.get("zero_position_rad", []), dtype=float)
    tolerance = np.asarray(hand_config.get("zero_tolerance_rad", []), dtype=float)
    for side in ("left", "right"):
        path = f"joint/state/hand/{side}"
        if path not in file or not len(zero) or not len(tolerance):
            continue
        group = file[path]
        positions = np.asarray(group["position_rad"][:], dtype=float)
        times = np.asarray(group["time_ns"][:], dtype=np.int64)
        for index, position in enumerate(positions):
            if len(position) == len(zero) and np.isfinite(position).all() and np.all(np.abs(position - zero) <= tolerance):
                hand_zero_times.append(float(times[index]) / 1e9)
                break
    tracking = {
        "samples": len(tracking_values),
        "max_error_rad": max(tracking_values) if tracking_values else "unavailable",
    }
    return {
        "rates": streams,
        "target_to_solved_error": target_solved,
        "joint_step_rad": command_step,
        "joint_velocity_rad_s": command_velocity,
        "saturation_count": saturation,
        "proposal_rejections": None,
        "command_feedback_tracking": tracking,
        "home_time_s": home_time,
        "hand_zero_time_s": max(hand_zero_times) if hand_zero_times else "unavailable",
        "soft_stop_reasons": [],
        "session_event_states": Counter(event_states),
    }


def _verify_operator_events(bundle: Path, manifest: Mapping[str, Any]) -> list[dict[str, Any]]:
    values = _json_lines(bundle / "operator_events.jsonl")
    for row in values:
        if row.get("schema_version") != 1 or row.get("run_id") != manifest["run_id"]:
            raise AnalysisError("operator event schema or run_id mismatch")
        if not isinstance(row.get("time_ns"), int) or row["time_ns"] < 0 or not isinstance(row.get("event"), str) or not row["event"]:
            raise AnalysisError("invalid operator event")
    times = [int(row["time_ns"]) for row in values]
    if times != sorted(times):
        raise AnalysisError("operator event monotonic time rollback")
    return values


def analyze_bundle(bundle: Path, matrix: Mapping[str, Any]) -> dict[str, Any]:
    manifest_preview = _yaml(bundle / "manifest.yaml")
    session_required = manifest_preview.get("profile") not in SESSION_OPTIONAL_PROFILES
    required = set(REQUIRED_FILES)
    if session_required:
        required.add("session.h5")
    missing = [name for name in required if not (bundle / name).is_file()]
    if missing:
        raise AnalysisError(f"bundle missing required files: {sorted(missing)}")
    _verify_checksums(bundle, session_required=session_required)
    manifest = _verify_manifest(bundle, matrix)
    statuses = _verify_status(bundle, manifest)
    _verify_capture(bundle, manifest)
    operator = _verify_operator_result(bundle)
    events = _verify_operator_events(bundle, manifest)
    if session_required or (bundle / "session.h5").is_file():
        metrics = _verify_h5(bundle, manifest)
    else:
        metrics = {
            "rates": {},
            "target_to_solved_error": {"samples": 0, "max_position_error_m": "unavailable", "max_orientation_error_rad": "unavailable", "note": "replay/acquisition profile is not session-recordable in this run"},
            "joint_step_rad": {}, "joint_velocity_rad_s": {}, "saturation_count": {},
            "proposal_rejections": "unavailable",
            "command_feedback_tracking": {"samples": 0, "max_error_rad": "unavailable", "note": "no session HDF5"},
            "home_time_s": "unavailable", "hand_zero_time_s": "unavailable",
            "fault_reasons": [], "soft_stop_reasons": [], "session_event_states": {},
        }
    rejection_events = [row for row in statuses if row.get("event") in {"proposal_rejected", "producer_unhealthy", "action_rejected"}]
    metrics["proposal_rejections"] = len(rejection_events) if rejection_events else "unavailable"
    metrics["fault_reasons"] = [str(row.get("reason") or row.get("error")) for row in statuses if row.get("event") in {"fault", "fault_latched"} and (row.get("reason") or row.get("error"))]
    metrics["soft_stop_reasons"] = [str(row.get("reason") or row.get("error")) for row in statuses if row.get("event") in {"soft_stop", "safety_stop"} and (row.get("reason") or row.get("error"))]
    solved_errors = []
    for row in _json_lines(bundle / "protocol.jsonl"):
        target = row.get("target_position_m")
        solved = row.get("solved_position_m")
        if isinstance(target, list) and isinstance(solved, list) and len(target) == len(solved):
            try:
                error = np.asarray(target, dtype=float) - np.asarray(solved, dtype=float)
                if np.isfinite(error).all():
                    solved_errors.append(float(np.linalg.norm(error)))
            except (TypeError, ValueError):
                continue
    if solved_errors:
        metrics["target_to_solved_error"] = {
            "samples": len(solved_errors),
            "max_position_error_m": max(solved_errors),
            "max_orientation_error_rad": "unavailable",
            "note": "protocol target/solved position pairing",
        }
    if operator["outcome"] == "pass":
        if manifest.get("fake"):
            raise AnalysisError("fake/headless bundle cannot be pass")
        rates = metrics.get("rates", {})
        if not any(value.get("samples", 0) for value in rates.values() if isinstance(value, Mapping)):
            raise AnalysisError("pass requires recorded authority samples")
    return {
        "schema_name": "tianji-validation-analysis",
        "schema_version": BUNDLE_VERSION,
        "run_id": manifest["run_id"],
        "case_id": manifest["case_id"],
        "profile": manifest["profile"],
        "operator_outcome": operator["outcome"],
        "physical_validation": bool(manifest["required_capability"] == "real" and not manifest.get("fake", False)),
        "status_records": len(statuses),
        "operator_events": events,
        "stop_criteria": matrix["cases"][manifest["case_id"]]["stop_criteria"],
        "metrics": metrics,
        "config": {"robot": "robot/arm.yaml", "coordinator": "coordinator/arm.yaml", "hashes": manifest["hashes"]},
    }


def _markdown(report: Mapping[str, Any]) -> str:
    metrics = report["metrics"]
    lines = [
        f"# Validation analysis: `{report['case_id']}`",
        "",
        f"- Run: `{report['run_id']}`",
        f"- Profile: `{report['profile']}`",
        f"- Operator outcome: **{report['operator_outcome']}**",
        f"- Status records: {report['status_records']}",
        "",
        "## Stream rates and drops",
        "",
        "| Stream | Samples | Rate (Hz) | Drops |",
        "|---|---:|---:|---:|",
    ]
    for name, value in metrics["rates"].items():
        lines.append(f"| {name} | {value['samples']} | {value['rate_hz']:.3f} | {value['drops']} |")
    lines += [
        "",
        "## Joint and safety metrics",
        "",
        f"- Maximum command step (rad): `{metrics['joint_step_rad']}`",
        f"- Maximum command velocity (rad/s): `{metrics['joint_velocity_rad_s']}`",
        f"- Saturation count: `{metrics['saturation_count']}`",
        f"- Proposal rejections: `{metrics['proposal_rejections']}`",
        f"- Target → solved: `{metrics['target_to_solved_error']}`",
        f"- Command → feedback: `{metrics['command_feedback_tracking']}`",
        f"- Home time (s): `{metrics['home_time_s']}`",
        f"- Hand zero time (s): `{metrics['hand_zero_time_s']}`",
        f"- Fault reasons: `{metrics['fault_reasons']}`",
        f"- Soft-stop reasons: `{metrics['soft_stop_reasons']}`",
        "",
        "## Operator events",
        "",
    ]
    if report["operator_events"]:
        for event in report["operator_events"]:
            lines.append(f"- `{event['time_ns']}` **{event['event']}**: {event.get('details', '')}")
    else:
        lines.append("- none")
    lines.append("")
    return "\n".join(lines)


def analyze(root: Path) -> list[dict[str, Any]]:
    matrix = load_matrix(MATRIX_PATH)
    reports: list[dict[str, Any]] = []
    for bundle in _bundle_paths(root):
        report = analyze_bundle(bundle, matrix)
        (bundle / "analysis.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, default=lambda value: dict(value)), encoding="utf-8")
        (bundle / "analysis.md").write_text(_markdown(report), encoding="utf-8")
        reports.append(report)
    return reports


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    args = parser.parse_args(argv)
    try:
        reports = analyze(args.root.expanduser())
    except (AnalysisError, ValueError, OSError, yaml.YAMLError) as exc:
        print(f"validation-analyze failed: {exc}", file=sys.stderr)
        return 1
    print(json.dumps({"bundles": len(reports), "run_ids": [item["run_id"] for item in reports]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
