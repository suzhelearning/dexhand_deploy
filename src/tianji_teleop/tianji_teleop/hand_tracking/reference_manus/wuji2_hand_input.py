#!/usr/bin/env python3
"""Semantic Manus-to-MediaPipe conversion for Wuji Hand 2 teleoperation.

The Manus SDK raw-skeleton node IDs and array order are not a stable public
interface.  This module therefore resolves nodes by the SDK's chain and finger
joint semantics, matching the mapping used by ``wuji_teleop``.
"""

from __future__ import annotations

from typing import Any, Iterable

import numpy as np


class Wuji2MappingError(ValueError):
    """Raised when a raw Manus frame cannot be mapped safely."""


MEDIAPIPE_SEMANTIC_ORDER = (
    ("hand", "wrist"),
    ("thumb", "mcp"),
    ("thumb", "pip"),
    ("thumb", "dip"),
    ("thumb", "tip"),
    ("index", "pip"),
    ("index", "ip"),
    ("index", "dip"),
    ("index", "tip"),
    ("middle", "pip"),
    ("middle", "ip"),
    ("middle", "dip"),
    ("middle", "tip"),
    ("ring", "pip"),
    ("ring", "ip"),
    ("ring", "dip"),
    ("ring", "tip"),
    ("pinky", "pip"),
    ("pinky", "ip"),
    ("pinky", "dip"),
    ("pinky", "tip"),
)

_REQUIRED_SEMANTICS = frozenset(MEDIAPIPE_SEMANTIC_ORDER)

_CHAIN_ALIASES = {
    "hand": "hand",
    "fingerthumb": "thumb",
    "thumb": "thumb",
    "fingerindex": "index",
    "index": "index",
    "fingermiddle": "middle",
    "middle": "middle",
    "fingerring": "ring",
    "ring": "ring",
    "fingerpinky": "pinky",
    "pinky": "pinky",
}

_JOINT_ALIASES = {
    "mcp": "mcp",
    "metacarpal": "mcp",
    "pip": "pip",
    "proximal": "pip",
    "ip": "ip",
    "intermediate": "ip",
    "dip": "dip",
    "distal": "dip",
    "tip": "tip",
}

# Values from ManusSDKTypes.h.  Accepting integers lets the adapter consume
# rawviz's compact protocol without making the protocol dependent on SDK enum
# spelling.
_CHAIN_NUMERIC_ALIASES = {
    5: "thumb",
    6: "index",
    7: "middle",
    8: "ring",
    9: "pinky",
    13: "hand",
}
_JOINT_NUMERIC_ALIASES = {
    1: "mcp",
    2: "pip",
    3: "ip",
    4: "dip",
    5: "tip",
}


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


def node_semantic_key(node: dict[str, Any]) -> tuple[str, str] | None:
    """Return the canonical semantic key for one raw Manus node."""

    chain = _normalise_chain(node.get("chain_type"))
    if chain is None:
        return None
    if chain == "hand":
        return ("hand", "wrist")
    joint = _normalise_joint(node.get("finger_joint_type"))
    if joint is None:
        return None
    return (chain, joint)


def resolve_wuji2_keypoints(
    nodes: Iterable[dict[str, Any]],
    positions: np.ndarray,
) -> np.ndarray:
    """Resolve raw Manus positions to the official 21-point input order.

    Args:
        nodes: Node metadata containing ``array_index``, ``chain_type`` and
            ``finger_joint_type``.
        positions: Raw positions in the same array order as the POSE record,
            with shape ``(node_count, 3)`` and meters as units.

    Returns:
        A float32 ``(21, 3)`` array.  The existing Manus VUH-to-retarget input
        conversion is applied once by negating the Y coordinate.
    """

    raw_positions = np.asarray(positions, dtype=np.float32)
    if raw_positions.ndim != 2 or raw_positions.shape[1] != 3:
        raise ValueError(
            f"positions must have shape (node_count, 3), got {raw_positions.shape}"
        )
    if not np.isfinite(raw_positions).all():
        raise ValueError("positions contain NaN or infinity")

    resolved: dict[tuple[str, str], int] = {}
    duplicate_keys: list[tuple[str, str]] = []
    for node in nodes:
        key = node_semantic_key(node)
        if key not in _REQUIRED_SEMANTICS:
            continue
        try:
            array_index = int(node["array_index"])
        except (KeyError, TypeError, ValueError) as exc:
            raise Wuji2MappingError("semantic node has no valid array_index") from exc
        if not 0 <= array_index < raw_positions.shape[0]:
            raise Wuji2MappingError(
                f"semantic node {key} has out-of-range array_index {array_index}"
            )
        if key in resolved:
            duplicate_keys.append(key)
        else:
            resolved[key] = array_index

    if duplicate_keys:
        duplicate_text = ", ".join(
            f"{chain}/{joint}" for chain, joint in sorted(set(duplicate_keys))
        )
        raise Wuji2MappingError(f"duplicate semantic nodes: {duplicate_text}")

    missing = [key for key in MEDIAPIPE_SEMANTIC_ORDER if key not in resolved]
    if missing:
        missing_text = ", ".join(f"{chain}/{joint}" for chain, joint in missing)
        raise Wuji2MappingError(f"missing semantic nodes: {missing_text}")

    indices = [resolved[key] for key in MEDIAPIPE_SEMANTIC_ORDER]
    result = raw_positions[indices].copy()
    result[:, 1] *= -1.0
    return result


def _parse_int(value: str, field: str) -> int:
    try:
        return int(value, 0)
    except ValueError as exc:
        raise ValueError(f"invalid {field}: {value!r}") from exc


def parse_rawviz_line(line: str) -> dict[str, Any] | None:
    """Parse one rawviz protocol line.

    Unknown/blank lines return ``None`` so SDK diagnostics accidentally written
    to stdout cannot be mistaken for a hand frame.
    """

    fields = line.strip().split()
    if not fields:
        return None
    tag = fields[0].upper()
    if tag == "HAND":
        if len(fields) != 4:
            raise ValueError("HAND requires glove_id, side, and node_count")
        return {
            "kind": "hand",
            "glove_id": fields[1],
            "side": fields[2].lower(),
            "node_count": _parse_int(fields[3], "node_count"),
        }
    if tag == "NODE":
        if len(fields) != 8:
            raise ValueError(
                "NODE requires glove_id, array_index, node_id, parent_id, "
                "chain_type, side, and finger_joint_type"
            )
        return {
            "kind": "node",
            "glove_id": fields[1],
            "array_index": _parse_int(fields[2], "array_index"),
            "node_id": _parse_int(fields[3], "node_id"),
            "parent_id": _parse_int(fields[4], "parent_id"),
            "chain_type": _parse_int(fields[5], "chain_type"),
            "side": _parse_int(fields[6], "side"),
            "finger_joint_type": _parse_int(fields[7], "finger_joint_type"),
        }
    if tag == "POSE":
        if len(fields) < 5:
            raise ValueError("POSE requires glove_id, seq, and timestamps")
        glove_id = fields[1]
        try:
            values = np.asarray([float(value) for value in fields[5:]], dtype=np.float32)
        except ValueError as exc:
            raise ValueError("POSE contains a non-numeric value") from exc
        if values.size % 7 != 0:
            raise ValueError("POSE payload must contain 7 values per node")
        rows = values.reshape((-1, 7))
        return {
            "kind": "pose",
            "glove_id": glove_id,
            "sequence": _parse_int(fields[2], "sequence"),
            "source_monotonic_ns": _parse_int(fields[3], "source_monotonic_ns"),
            "sdk_publish_time": _parse_int(fields[4], "sdk_publish_time"),
            "positions": rows[:, :3],
            "quaternions_wxyz": rows[:, 3:],
        }
    if tag == "EDGE":
        return {"kind": "edge", "fields": fields[1:]}
    return None


__all__ = [
    "MEDIAPIPE_SEMANTIC_ORDER",
    "Wuji2MappingError",
    "node_semantic_key",
    "parse_rawviz_line",
    "resolve_wuji2_keypoints",
]
