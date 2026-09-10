"""Composition helpers for one XR input stream and one Manus rawviz stream."""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np

from .models import ArmInputObservation, HandObservation
from .reference_manus_process import ManusCallback
from .xr_input import XrBindingConfig, XrFrame


_MANUS_CALLBACK_KEYS = frozenset({
    "schema_version", "kind", "router_zid", "receiver_instance_id",
    "callback_sequence", "received_timestamp_ns", "points",
    "source_sequences", "source_timestamps_ns",
})


def _manus_points(value: Any) -> np.ndarray:
    """Normalize the reference assembler's flat or row-shaped point array."""
    try:
        points = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError("callback points must be numeric") from exc
    if points.ndim == 1 and points.size in (63, 126):
        points = points.reshape((-1, 3))
    elif points.ndim == 2 and points.shape[1] == 3 and points.shape[0] in (21, 42):
        points = points.reshape((-1, 3))
    else:
        raise ValueError("callback points must contain 21 or 42 xyz points")
    if not np.isfinite(points).all():
        raise ValueError("callback points must be finite")
    return points


@dataclass(frozen=True)
class ManusCallbackRaw:
    callback: ManusCallback
    router_zid: str


def _callback_metadata(value: Any, field: str) -> dict[str, int]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be an object")
    result: dict[str, int] = {}
    for side, item in value.items():
        if side not in {"left", "right"} or type(item) is not int or not 0 <= item < 2**63:
            raise ValueError(f"{field} contains an invalid side/value")
        result[side] = item
    return result


def encode_manus_callback(callback: ManusCallback, router_zid: str) -> dict[str, Any]:
    """Encode one complete rawviz callback without changing its point order."""
    if not isinstance(callback, ManusCallback):
        raise TypeError("callback must be ManusCallback")
    if not isinstance(router_zid, str) or not router_zid.strip():
        raise ValueError("router_zid is required")
    if type(callback.sequence) is not int or not 0 < callback.sequence < 2**63:
        raise ValueError("callback sequence must be a positive int64")
    if type(callback.received_timestamp_ns) is not int or not 0 < callback.received_timestamp_ns < 2**63:
        raise ValueError("callback receive timestamp must be a positive int64")
    points = _manus_points(callback.points).reshape(-1).tolist()
    sequences = _callback_metadata(callback.source_sequences, "source_sequences")
    timestamps = _callback_metadata(callback.source_timestamps_ns, "source_timestamps_ns")
    return {
        "schema_version": 1,
        "kind": "manus_callback",
        "router_zid": router_zid,
        "receiver_instance_id": callback.receiver_instance_id,
        "callback_sequence": callback.sequence,
        "received_timestamp_ns": callback.received_timestamp_ns,
        "points": points,
        "source_sequences": sequences,
        "source_timestamps_ns": timestamps,
    }


def decode_manus_callback(value: Mapping[str, Any]) -> ManusCallbackRaw:
    """Strictly decode a rawviz callback envelope."""
    if not isinstance(value, Mapping) or set(value) != _MANUS_CALLBACK_KEYS:
        raise ValueError("invalid Manus callback envelope")
    if value["schema_version"] != 1 or value["kind"] != "manus_callback":
        raise ValueError("unsupported Manus callback envelope")
    router_zid = value["router_zid"]
    receiver_instance_id = value["receiver_instance_id"]
    if not isinstance(router_zid, str) or not router_zid.strip():
        raise ValueError("callback router_zid is required")
    if not isinstance(receiver_instance_id, str) or not receiver_instance_id.strip():
        raise ValueError("callback receiver_instance_id is required")
    points = value["points"]
    if not isinstance(points, (list, tuple)) or len(points) not in (63, 126):
        raise ValueError("callback points must contain 63 or 126 values")
    if any(type(item) not in (int, float) for item in points) or not np.isfinite(points).all():
        raise ValueError("callback points must be finite numbers")
    callback = ManusCallback(
        receiver_instance_id=receiver_instance_id,
        sequence=value["callback_sequence"],
        received_timestamp_ns=value["received_timestamp_ns"],
        points=tuple(float(item) for item in points),
        source_sequences=_callback_metadata(value["source_sequences"], "source_sequences"),
        source_timestamps_ns=_callback_metadata(value["source_timestamps_ns"], "source_timestamps_ns"),
    )
    if type(callback.sequence) is not int or not 0 < callback.sequence < 2**63:
        raise ValueError("callback sequence must be a positive int64")
    if type(callback.received_timestamp_ns) is not int or not 0 < callback.received_timestamp_ns < 2**63:
        raise ValueError("callback receive timestamp must be a positive int64")
    return ManusCallbackRaw(callback=callback, router_zid=router_zid)


def manus_callback_observations(
    callback: ManusCallback, *, sides: tuple[str, ...] = ("right", "left")
) -> dict[str, HandObservation]:
    """Convert the reference rawviz callback into canonical 21-point hands.

    ``HandInputAssembler`` deliberately preserves its historical concatenation
    order (right then left); callers must pass that order explicitly.  The
    conversion subtracts the wrist once because rawviz positions are absolute
    in the Manus local frame while the canonical hand contract is wrist-relative.
    """
    if not isinstance(callback, ManusCallback):
        raise TypeError("callback must be ManusCallback")
    if not sides or len(set(sides)) != len(sides) or set(sides) - {"left", "right"}:
        raise ValueError("sides must contain distinct left/right values")
    values = _manus_points(callback.points)
    expected = (len(sides) * 21, 3)
    if values.shape != expected or not np.isfinite(values).all():
        raise ValueError(f"Manus callback points must have shape {expected}")
    result: dict[str, HandObservation] = {}
    for index, side in enumerate(sides):
        points = values[index * 21:(index + 1) * 21].copy()
        points -= points[0]
        source_sequence = callback.source_sequences.get(side)
        source_timestamp_ns = callback.source_timestamps_ns.get(side)
        result[side] = HandObservation(
            source="manus",
            side=side,
            source_instance_id=f"{callback.receiver_instance_id}:{side}",
            source_sequence=source_sequence,
            source_timestamp_ns=source_timestamp_ns,
            received_timestamp_ns=callback.received_timestamp_ns,
            receiver_instance_id=callback.receiver_instance_id,
            receiver_frame_sequence=callback.sequence,
            coordinate_frame="manus_local_vuh_y_flipped_wrist_relative",
            mapping_version="manus25_to_mediapipe21_v1",
            keypoints_m=points,
            joint_valid=np.ones(21, dtype=np.bool_),
            valid=True,
            wrist_pose=None,
            frame_association_id=f"manus:{callback.receiver_instance_id}:{callback.sequence}",
        )
    return result


def xr_frame_arm_observations(
    frame: XrFrame, *, binding: XrBindingConfig, tracked_frame: str,
    reference_frame: str, receiver_instance_id: str = "xr",
    source_instance_id: str = "xr",
) -> dict[str, ArmInputObservation]:
    """Convert one raw XR frame to the canonical pre-mapping arm inputs."""
    if not isinstance(frame, XrFrame) or not isinstance(binding, XrBindingConfig):
        raise TypeError("frame and binding are required")
    if not isinstance(tracked_frame, str) or not tracked_frame:
        raise ValueError("tracked_frame is required")
    if not isinstance(reference_frame, str) or not reference_frame:
        raise ValueError("reference_frame is required")
    result: dict[str, ArmInputObservation] = {}
    if receiver_instance_id == "xr" and source_instance_id == "xr":
        receiver_instance_id = frame.receiver_instance_id
    base_source_id = receiver_instance_id if source_instance_id == "xr" else source_instance_id
    generation_source_id = f"{base_source_id}:{frame.connection_generation}"
    for side in ("left", "right"):
        pose = binding.pose_for_arm(frame, side)
        elbow_pose = binding.elbow_pose_for_arm(frame, side)
        result[side] = ArmInputObservation(
            source="xr",
            side=side,
            tracked_frame=tracked_frame,
            reference_frame=reference_frame,
            pose=pose,
            valid=pose is not None,
            source_timestamp_ns=frame.source_timestamp_ns,
            received_timestamp_ns=frame.received_timestamp_ns,
            receiver_instance_id=receiver_instance_id,
            receiver_frame_sequence=frame.sequence,
            mapping_version=("xr_controller_v1" if binding.arm_input == "xr_controller"
                             else "xr_tracker_v1"),
            frame_association_id=(
                f"xr:{receiver_instance_id}:{frame.connection_generation}:{frame.sequence}"
            ),
            source_sequence=frame.sequence,
            source_instance_id=generation_source_id,
            elbow_pose=elbow_pose,
        )
    return result


__all__ = [
    "ManusCallbackRaw", "decode_manus_callback", "encode_manus_callback",
    "manus_callback_observations", "xr_frame_arm_observations",
]
