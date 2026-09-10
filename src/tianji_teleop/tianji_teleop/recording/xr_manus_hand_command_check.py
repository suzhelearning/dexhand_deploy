"""Replay the XR/Manus target-to-Wuji2 hand boundary offline.

The XR/Manus path has three independent sequence domains: rawviz callbacks,
hand targets, and executor commands.  This checker therefore uses the
executor's passive target-to-command audit instead of guessing an association
from any one of those sequence numbers.
"""
from contextlib import ExitStack
from pathlib import Path
from typing import Any

import numpy as np

from ..producers.hand_retarget import OfficialHandClient
from ..protocol.messages import HandJointCommand, HandTargetCommand
from .hand_command_check import hand_replay_asset_hashes, _asset_matches
from .manus_check import check_manus_recording
from .session_h5 import SessionH5Reader


def _audit_record(value: Any, *, router_zid: str) -> tuple[HandTargetCommand, HandJointCommand, str]:
    if not isinstance(value, dict):
        raise ValueError("hand output audit payload must be an object")
    expected = {
        "schema_version", "kind", "router_zid", "run_id",
        "executor_instance_id", "side", "target", "command",
    }
    if set(value) != expected:
        raise ValueError("hand output audit has an invalid field set")
    if value["schema_version"] != 1 or value["kind"] != "hand_output":
        raise ValueError("unsupported hand output audit schema")
    if value["router_zid"] != router_zid:
        raise ValueError("hand output audit router does not match recording")
    for field in ("run_id", "executor_instance_id"):
        if not isinstance(value[field], str) or not value[field].strip():
            raise ValueError(f"hand output audit {field} is required")
    side = value["side"]
    if side not in ("left", "right"):
        raise ValueError("hand output audit side must be left/right")
    try:
        target = HandTargetCommand.from_dict(value["target"])
        command = HandJointCommand.from_dict(value["command"])
    except (TypeError, ValueError) as exc:
        raise ValueError("hand output audit contains an invalid target or command") from exc
    if target.side != side or command.side != side:
        raise ValueError("hand output audit side does not match target/command")
    if target.router_zid != router_zid or command.router_zid != router_zid:
        raise ValueError("hand output target/command router does not match recording")
    if command.sequence <= 0 or target.sequence <= 0:
        raise ValueError("hand output target/command sequences must be positive")
    return target, command, side


def _compare(report: dict[str, Any], expected: dict[str, Any], actual: HandJointCommand,
             *, side: str, target_sequence: int) -> None:
    actual_names = list(actual.names)
    if expected.get("joint_names") != actual_names:
        report["passed"] = False
        if report["first_difference"] is None:
            report["first_difference"] = {
                "stage": "hand_command",
                "side": side,
                "command_sequence": actual.sequence,
                "target_sequence": target_sequence,
                "field": "joint_names",
            }
    expected_values = np.asarray(expected.get("position_rad"), dtype=np.float64)
    actual_values = np.asarray(actual.position_rad, dtype=np.float64)
    if expected_values.shape != (20,) or actual_values.shape != (20,):
        raise ValueError("invalid hand command shape during XR/Manus replay")
    error = np.abs(expected_values - actual_values)
    if not np.isfinite(error).all():
        raise ValueError("nonfinite hand command difference during XR/Manus replay")
    report["max_error_rad"] = max(report["max_error_rad"], float(error.max(initial=0.0)))
    bad = np.flatnonzero(error > report["tolerance_rad"])
    if len(bad):
        report["passed"] = False
        if report["first_difference"] is None:
            index = int(bad[0])
            report["first_difference"] = {
                "stage": "hand_command",
                "side": side,
                "command_sequence": actual.sequence,
                "target_sequence": target_sequence,
                "field": "position_rad",
                "joint_name": actual_names[index],
                "error_rad": float(error[index]),
            }


def check_xr_manus_hand_commands(path, *, root, backend_factory=None, input_atol=1e-9):
    """Validate and replay one recorded XR/Manus hand control stream."""
    reconstruction = check_manus_recording(path, atol=input_atol)
    report = dict(
        passed=reconstruction["passed"],
        scope="xr_manus_hand_target_to_recorded_hand_command",
        solver="injected_test_backend" if backend_factory is not None else "official_wuji_hand2",
        input_reconstruction=reconstruction,
        replayed_callbacks=0,
        replayed_targets=0,
        audit_commands=0,
        matched_commands=0,
        max_error_rad=0.0,
        tolerance_rad=1e-5,
        first_difference=reconstruction["first_difference"],
        operator_events_executed=0,
        limitations=[
            "command numerical consistency only; no authorization/execution replay",
            "target-to-command association comes from the executor audit",
            "asset hashes are snapshots, not runtime dependency attestation",
        ],
    )
    if not reconstruction["passed"]:
        return report

    root = Path(root).resolve(strict=True)
    with SessionH5Reader(path) as reader:
        source_type = str(reader.attrs.get("source_type", ""))
        if source_type != "vr_manus_xr_sim":
            raise ValueError("XR/Manus hand replay requires source_type=vr_manus_xr_sim")
        configuration = reader.read_hand_tracking_metadata()["resolved_configuration"]
        recorded_assets = configuration.get("asset_sha256", {})
        for asset, digest in hand_replay_asset_hashes(root).items():
            if not _asset_matches(recorded_assets, root, asset, digest):
                raise ValueError(f"hand replay asset provenance mismatch: {asset}")
        audits = [row for row in reader.read_dual_audit() if row["kind"] == "hand_output"]
        recorded_commands = {
            side: reader.read_hand_command(side) for side in ("left", "right")
        }
        router_zid = str(reader.attrs["router_zid"])

    if not audits:
        raise ValueError(
            "XR/Manus hand replay requires hand_output audit; recapture with the updated runtime"
        )

    audit_rows: list[tuple[dict[str, Any], HandTargetCommand, HandJointCommand, str]] = []
    audit_index: dict[tuple[str, int], HandJointCommand] = {}
    previous_command: dict[str, int] = {}
    previous_target: dict[str, int] = {}
    for row in audits:
        target, command, side = _audit_record(row["payload"], router_zid=router_zid)
        if command.sequence <= previous_command.get(side, 0):
            raise ValueError(f"XR/Manus hand command sequence rollback for {side}")
        if target.sequence < previous_target.get(side, 0):
            raise ValueError(f"XR/Manus hand target sequence rollback for {side}")
        previous_command[side] = command.sequence
        previous_target[side] = target.sequence
        key = (side, command.sequence)
        if key in audit_index:
            raise ValueError(f"duplicate XR/Manus hand output audit: {key}")
        audit_index[key] = command
        audit_rows.append((row, target, command, side))

    recorded_index: dict[tuple[str, int], dict[str, Any]] = {}
    for side, rows in recorded_commands.items():
        for command in rows:
            key = (side, int(command["sequence"]))
            if key in recorded_index:
                raise ValueError(f"duplicate recorded XR/Manus hand command: {key}")
            recorded_index[key] = command
    if set(recorded_index) != set(audit_index):
        missing = sorted(set(audit_index) - set(recorded_index))
        extra = sorted(set(recorded_index) - set(audit_index))
        raise ValueError(
            f"XR/Manus hand command/audit association mismatch: missing={missing[:3]}, extra={extra[:3]}"
        )

    for row, target, command, side in audit_rows:
        recorded = recorded_index[(side, command.sequence)]
        if recorded["names"] != command.names:
            report["passed"] = False
            if report["first_difference"] is None:
                report["first_difference"] = {
                    "stage": "recording",
                    "side": side,
                    "command_sequence": command.sequence,
                    "field": "joint_names",
                }
        actual_values = np.asarray(recorded["position_rad"], dtype=np.float64)
        audit_values = np.asarray(command.position_rad, dtype=np.float64)
        if actual_values.shape != (20,) or not np.isfinite(actual_values).all():
            raise ValueError("recorded XR/Manus hand command is invalid")
        error = np.abs(actual_values - audit_values)
        if not np.isfinite(error).all():
            raise ValueError("nonfinite recorded/audit XR/Manus hand command difference")
        report["max_error_rad"] = max(report["max_error_rad"], float(error.max(initial=0.0)))
        if np.any(error > report["tolerance_rad"]):
            report["passed"] = False
            if report["first_difference"] is None:
                index = int(np.flatnonzero(error > report["tolerance_rad"])[0])
                report["first_difference"] = {
                    "stage": "recording",
                    "side": side,
                    "command_sequence": command.sequence,
                    "target_sequence": target.sequence,
                    "field": "position_rad",
                    "joint_name": command.names[index],
                    "error_rad": float(error[index]),
                }

    if not report["passed"]:
        return report

    sides = configuration["manus_input_contract"]["sides"]
    factory = backend_factory if backend_factory is not None else OfficialHandClient
    with ExitStack() as stack:
        clients = {}
        for side in sides:
            client = factory(
                python=root / "tools/wuji_hand_native/.pixi/envs/default/bin/python",
                script=root / "scripts/wuji_hand_worker.py",
                single_hand_side=side,
                startup_handshake=True,
            )
            stack.callback(client.close)
            clients[side] = client
        cached: dict[str, tuple[int, HandTargetCommand, dict[str, Any]]] = {}
        for _row, target, command, side in audit_rows:
            previous = cached.get(side)
            if previous is None or target.sequence != previous[0]:
                if previous is not None and target.sequence <= previous[0]:
                    raise ValueError(f"XR/Manus hand target sequence did not increase for {side}")
                result = clients[side].retarget(
                    np.asarray(target.keypoints_m, dtype=np.float64).reshape(-1).tolist(),
                    sequence=target.sequence,
                    timestamp_ns=target.timestamp_ns,
                )
                expected = result.get(side) if isinstance(result, dict) else None
                if not isinstance(expected, dict) or expected.get("valid") is not True:
                    raise ValueError(f"official hand replay returned no valid {side} result")
                cached[side] = (target.sequence, target, expected)
                report["replayed_targets"] += 1
            else:
                previous_target = previous[1]
                if target.to_dict() != previous_target.to_dict():
                    raise ValueError(f"XR/Manus repeated target changed for {side}")
            _compare(
                report,
                cached[side][2],
                command,
                side=side,
                target_sequence=target.sequence,
            )
            report["matched_commands"] += 1
    report["audit_commands"] = len(audit_rows)
    # The callback count is already exposed by the nested reconstruction; keep
    # a scalar here for consumers that aggregate all checker reports.
    report["replayed_callbacks"] = reconstruction["rebuilt_callbacks"]
    return report


__all__ = ["check_xr_manus_hand_commands"]
