"""Offline raw XR recording consistency checks; never starts a device.

The checker validates the denormalized HDF5 columns against the complete
``XrFrame`` JSON stored in the same row.  It also checks connection-generation
and sequence ordering.  It deliberately stops at the raw acquisition
boundary: no pose mapping, operator action, IK, hand retarget or actuator
replay is performed here.
"""
from __future__ import annotations

import math
from typing import Any

import numpy as np

from ..hand_tracking.xr_input import XrFrame
from ..hand_tracking.xr_operator import decode_xr_operator_observation
from .session_h5 import SessionH5Reader


_SIDES = ("left", "right")


def _same_array(expected: Any, actual: Any, *, atol: float) -> tuple[bool, float]:
    if expected is None:
        return actual is None, 0.0
    if actual is None:
        return False, math.inf
    left = np.asarray(expected, dtype=np.float64)
    right = np.asarray(actual, dtype=np.float64)
    if left.shape != right.shape or not np.isfinite(right).all():
        return False, math.inf
    error = float(np.max(np.abs(left - right), initial=0.0))
    return bool(np.isfinite(error) and error <= atol), error


def check_xr_recording(path: str, *, atol: float = 1.0e-9) -> dict[str, Any]:
    """Check one schema-1.2 XR recording and return a diagnostic report."""
    if isinstance(atol, bool) or not math.isfinite(atol) or atol < 0.0:
        raise ValueError("atol must be finite and nonnegative")
    with SessionH5Reader(path) as reader:
        source_type = str(reader.attrs.get("source_type", ""))
        router_zid = str(reader.attrs.get("router_zid", ""))
        rows = reader.read_raw_xr(preserve_source_clock=True)
        audits = reader.read_dual_audit()
    if source_type != "vr_manus_xr_sim":
        raise ValueError("XR recording check requires source_type=vr_manus_xr_sim")
    if not rows:
        raise ValueError("recording has no raw XR frames to check")

    report: dict[str, Any] = {
        "passed": True,
        "scope": "xr_raw_columns_to_frame_consistency",
        "raw_frames": len(rows),
        "receivers": [],
        "connection_generations": [],
        "matched_frames": 0,
        "duplicate_frames": 0,
        "operator_observations": 0,
        "valid_operator_observations": 0,
        "duplicate_operator_observations": 0,
        "first_difference": None,
        "max_absolute_error": 0.0,
        "atol": atol,
        "operator_events_executed": 0,
        "limitations": [
            "raw acquisition consistency only; not an independent SDK oracle",
            "no XR pose mapping, controller event, IK, hand retarget or actuator replay",
        ],
    }

    def difference(row: dict[str, Any], field: str) -> None:
        report["passed"] = False
        if report["first_difference"] is None:
            report["first_difference"] = {
                "receiver_instance_id": row.get("receiver_instance_id"),
                "connection_generation": row.get("connection_generation"),
                "receiver_frame_sequence": row.get("receiver_frame_sequence"),
                "field": field,
            }

    seen: set[tuple[str, int, int]] = set()
    last_by_receiver: dict[str, tuple[int, int]] = {}
    generations: set[int] = set()
    receivers: set[str] = set()
    sequences_by_generation: dict[int, set[int]] = {}
    operator_rows = [row for row in audits if row["kind"] == "operator_observation"]
    report["operator_observations"] = len(operator_rows)

    def compare_array(row: dict[str, Any], field: str, expected: Any, actual: Any) -> None:
        equal, error = _same_array(expected, actual, atol=atol)
        if math.isfinite(error):
            report["max_absolute_error"] = max(report["max_absolute_error"], error)
        if not equal:
            difference(row, field)

    for row in rows:
        try:
            frame = XrFrame.from_dict(row["frame"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("invalid decoded XR frame in recording") from exc

        key = (frame.receiver_instance_id, frame.connection_generation, frame.sequence)
        if key in seen:
            report["duplicate_frames"] += 1
            difference(row, "duplicate_frame")
        seen.add(key)
        receivers.add(frame.receiver_instance_id)
        generations.add(frame.connection_generation)
        sequences_by_generation.setdefault(frame.connection_generation, set()).add(frame.sequence)
        previous = last_by_receiver.get(frame.receiver_instance_id)
        if previous is not None:
            previous_generation, previous_sequence = previous
            if frame.connection_generation < previous_generation:
                raise ValueError("XR connection generation rolled back")
            if (frame.connection_generation == previous_generation and
                    frame.sequence <= previous_sequence):
                raise ValueError("XR frame sequence is not strictly increasing")
        last_by_receiver[frame.receiver_instance_id] = (
            frame.connection_generation, frame.sequence
        )

        scalar_fields = (
            ("receiver_instance_id", frame.receiver_instance_id),
            ("connection_generation", frame.connection_generation),
            ("receiver_frame_sequence", frame.sequence),
            ("received_timestamp_ns", frame.received_timestamp_ns),
        )
        for field, expected in scalar_fields:
            if row.get(field) != expected:
                difference(row, field)
        expected_source_time = (
            0 if frame.source_timestamp_ns is None else frame.source_timestamp_ns
        )
        if bool(row.get("source_time_valid")) != (frame.source_timestamp_ns is not None):
            difference(row, "source_time_valid")
        if row.get("source_time_ns") != expected_source_time:
            difference(row, "source_time_ns")
        if row.get("source_timestamp_ns") != frame.source_timestamp_ns:
            difference(row, "source_timestamp_ns")
        if bool(row.get("hmd_valid")) != (frame.hmd_pose is not None):
            difference(row, "hmd_valid")
        compare_array(row, "hmd_pose", frame.hmd_pose, row.get("hmd_pose"))

        for side in _SIDES:
            controller = frame.controller(side)
            for field, expected in (
                (f"{side}_controller_valid", bool(controller.valid)),
                (f"{side}_controller_available", bool(controller.available)),
                (f"{side}_trigger", controller.trigger),
                (f"{side}_grip", controller.grip),
            ):
                actual = row.get(field)
                if isinstance(expected, bool):
                    equal = bool(actual) == expected
                else:
                    equal = isinstance(actual, (int, float, np.number)) and math.isfinite(float(actual)) and abs(float(actual) - expected) <= atol
                    if equal:
                        report["max_absolute_error"] = max(
                            report["max_absolute_error"], abs(float(actual) - expected)
                        )
                if not equal:
                    difference(row, field)
            compare_array(
                row,
                f"{side}_controller_pose",
                controller.pose,
                row.get(f"{side}_controller_pose"),
            )
            compare_array(
                row,
                f"{side}_axis",
                controller.axis,
                row.get(f"{side}_axis"),
            )

        if row.get("tracker_count") != len(frame.trackers):
            difference(row, "tracker_count")
        if row.get("trackers") != row["frame"].get("trackers"):
            difference(row, "trackers_json")
        if "connection_generation" not in row.get("frame", {}):
            difference(row, "frame_json.connection_generation")
        elif row["frame"].get("connection_generation") != frame.connection_generation:
            difference(row, "frame_json.connection_generation")
        report["matched_frames"] += 1

    operator_seen: set[tuple[str, str, int, int, str]] = set()
    last_operator: dict[tuple[str, str, str], tuple[int, int]] = {}
    allowed_actions = {
        "start_request", "home_request", "clutch_press", "clutch_release",
    }
    for row in operator_rows:
        try:
            payload = row["payload"]
            observation = decode_xr_operator_observation(
                payload,
                expected_router_zid=router_zid,
            )
        except (KeyError, TypeError, ValueError):
            difference(row, "operator_observation")
            continue
        publisher = payload.get("publisher_instance_id") if isinstance(payload, dict) else None
        if publisher != observation.source or observation.action not in allowed_actions:
            difference(row, "operator_observation")
            continue
        if observation.epoch not in generations:
            difference(row, "operator_observation")
            continue
        if observation.sequence not in sequences_by_generation[observation.epoch]:
            difference(row, "operator_observation")
            continue
        key = (
            observation.source, observation.side, observation.epoch,
            observation.sequence, observation.action,
        )
        if key in operator_seen:
            report["duplicate_operator_observations"] += 1
            difference(row, "operator_observation")
            continue
        operator_seen.add(key)
        stream = (observation.source, observation.side, observation.action)
        previous = last_operator.get(stream)
        if previous is not None:
            previous_epoch, previous_sequence = previous
            if (observation.epoch < previous_epoch or
                    (observation.epoch == previous_epoch and
                     observation.sequence <= previous_sequence)):
                difference(row, "operator_observation")
                continue
        last_operator[stream] = (observation.epoch, observation.sequence)
        report["valid_operator_observations"] += 1

    report["receivers"] = sorted(receivers)
    report["connection_generations"] = sorted(generations)
    if report["duplicate_frames"]:
        report["passed"] = False
    return report


__all__ = ["check_xr_recording"]
