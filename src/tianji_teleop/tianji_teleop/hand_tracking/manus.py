"""Strict Manus JSON parsing and semantic 25-node to 21-point conversion."""
from __future__ import annotations

from typing import Any, Iterable, Mapping

import numpy as np

from .models import HandObservation, ManusRawFrame


class ManusMappingError(ValueError):
    """Raised when Manus metadata cannot identify the required points."""


MEDIAPIPE_SEMANTIC_ORDER = (
    ("hand", "wrist"),
    ("thumb", "mcp"), ("thumb", "pip"), ("thumb", "dip"), ("thumb", "tip"),
    ("index", "pip"), ("index", "ip"), ("index", "dip"), ("index", "tip"),
    ("middle", "pip"), ("middle", "ip"), ("middle", "dip"), ("middle", "tip"),
    ("ring", "pip"), ("ring", "ip"), ("ring", "dip"), ("ring", "tip"),
    ("pinky", "pip"), ("pinky", "ip"), ("pinky", "dip"), ("pinky", "tip"),
)

_REQUIRED_SEMANTICS = frozenset(MEDIAPIPE_SEMANTIC_ORDER)
_CHAIN_ALIASES = {
    "hand": "hand", "fingerthumb": "thumb", "thumb": "thumb",
    "fingerindex": "index", "index": "index", "fingermiddle": "middle", "middle": "middle",
    "fingerring": "ring", "ring": "ring", "fingerpinky": "pinky", "pinky": "pinky",
}
_JOINT_ALIASES = {
    "mcp": "mcp", "metacarpal": "mcp", "pip": "pip", "proximal": "pip",
    "ip": "ip", "intermediate": "ip", "dip": "dip", "distal": "dip", "tip": "tip",
}
_CHAIN_NUMERIC_ALIASES = {5: "thumb", 6: "index", 7: "middle", 8: "ring", 9: "pinky", 13: "hand"}
_JOINT_NUMERIC_ALIASES = {1: "mcp", 2: "pip", 3: "ip", 4: "dip", 5: "tip"}


def _normalise_token(value: Any) -> str:
    return str(value).strip().lower().replace("_", "").replace("-", "").replace(" ", "")


def _normalise_chain(value: Any) -> str | None:
    if isinstance(value, (int, np.integer)):
        return _CHAIN_NUMERIC_ALIASES.get(int(value))
    token = _normalise_token(value)
    if token.startswith("chaintype"):
        token = token[len("chaintype") :]
    return _CHAIN_ALIASES.get(token)


def _normalise_joint(value: Any) -> str | None:
    if isinstance(value, (int, np.integer)):
        return _JOINT_NUMERIC_ALIASES.get(int(value))
    token = _normalise_token(value)
    if token.startswith("fingerjointtype"):
        token = token[len("fingerjointtype") :]
    return _JOINT_ALIASES.get(token)


def node_semantic_key(node: Mapping[str, Any]) -> tuple[str, str] | None:
    chain = _normalise_chain(node.get("chain_type"))
    if chain is None:
        return None
    if chain == "hand":
        return ("hand", "wrist")
    joint = _normalise_joint(node.get("finger_joint_type"))
    return None if joint is None else (chain, joint)


def _normalise_side(value: Any) -> str:
    side = str(value).strip().lower()
    if side in {"left", "left_hand"}:
        return "left"
    if side in {"right", "right_hand"}:
        return "right"
    raise ValueError("Manus side must be left/right or left_hand/right_hand")


def parse_manus_payload(
    payload: Mapping[str, Any],
    *,
    receiver_instance_id: str,
    receiver_frame_sequence: int,
    received_timestamp_ns: int,
) -> ManusRawFrame:
    if not isinstance(payload, Mapping):
        raise ValueError("Manus payload must be an object")
    expected = {
        "glove_id", "side", "seq", "source_monotonic_ns", "sdk_publish_time",
        "nodes", "node_quaternions_wxyz", "node_semantics",
    }
    if set(payload) != expected:
        missing = expected - set(payload)
        extra = set(payload) - expected
        raise ValueError(f"invalid Manus payload fields; missing={sorted(missing)}, extra={sorted(extra)}")
    positions = np.asarray(payload["nodes"], dtype=np.float64)
    quaternions = np.asarray(payload["node_quaternions_wxyz"], dtype=np.float64)
    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError("Manus nodes must have shape (node_count, 3)")
    if quaternions.shape != (positions.shape[0], 4):
        raise ValueError("Manus node_quaternions_wxyz must match node count")
    semantics = payload["node_semantics"]
    if not isinstance(semantics, (list, tuple)):
        raise ValueError("Manus node_semantics must be an array")
    if not isinstance(payload["glove_id"], str) or not payload["glove_id"]:
        raise ValueError("Manus glove_id must be a non-empty string")
    return ManusRawFrame(
        glove_id=payload["glove_id"],
        side=_normalise_side(payload["side"]),
        source_sequence=payload["seq"],
        source_monotonic_ns=payload["source_monotonic_ns"],
        sdk_publish_time=payload["sdk_publish_time"],
        node_positions=positions,
        node_quaternions_wxyz=quaternions,
        node_semantics=tuple(semantics),
        received_timestamp_ns=received_timestamp_ns,
        receiver_instance_id=receiver_instance_id,
        receiver_frame_sequence=receiver_frame_sequence,
    )


def resolve_manus_keypoints(raw: ManusRawFrame) -> np.ndarray:
    resolved: dict[tuple[str, str], int] = {}
    for node in raw.node_semantics:
        key = node_semantic_key(node)
        if key not in _REQUIRED_SEMANTICS:
            continue
        index = int(node["array_index"])
        if key in resolved:
            raise ManusMappingError(f"duplicate semantic node: {key[0]}/{key[1]}")
        if index in resolved.values():
            raise ManusMappingError(f"semantic nodes share array_index: {index}")
        resolved[key] = index
    missing = [key for key in MEDIAPIPE_SEMANTIC_ORDER if key not in resolved]
    if missing:
        text = ", ".join(f"{chain}/{joint}" for chain, joint in missing)
        raise ManusMappingError(f"missing semantic nodes: {text}")
    result = raw.node_positions[[resolved[key] for key in MEDIAPIPE_SEMANTIC_ORDER]].copy()
    result[:, 1] *= -1.0
    result -= result[0:1]
    return result


def manus_to_mediapipe(raw: ManusRawFrame) -> HandObservation:
    points = resolve_manus_keypoints(raw)
    return HandObservation(
        source="manus",
        side=raw.side,
        source_instance_id=raw.glove_id,
        source_sequence=raw.source_sequence,
        source_timestamp_ns=raw.source_monotonic_ns,
        received_timestamp_ns=raw.received_timestamp_ns,
        receiver_instance_id=raw.receiver_instance_id,
        receiver_frame_sequence=raw.receiver_frame_sequence,
        coordinate_frame="manus_local_vuh_y_flipped_wrist_relative",
        mapping_version="manus25_to_mediapipe21_v1",
        keypoints_m=points,
        joint_valid=np.ones(21, dtype=np.bool_),
        valid=True,
        wrist_pose=None,
        frame_association_id=raw.association_id,
    )


__all__ = [
    "MEDIAPIPE_SEMANTIC_ORDER",
    "ManusMappingError",
    "manus_to_mediapipe",
    "node_semantic_key",
    "parse_manus_payload",
    "resolve_manus_keypoints",
]
