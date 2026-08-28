"""Versioned, strict JSON wire messages for Tianji teleoperation.

The module deliberately has no Zenoh dependency.  It is the single Python-side
schema implementation used by publishers, subscribers, recorders, and tests.
"""
from __future__ import annotations
from dataclasses import dataclass
import json
import math
from typing import Any, Mapping, Sequence
SCHEMA_VERSION = 1
SIDES = ("left", "right")
ARM_FRAMES = {"left": "Base_L", "right": "Base_R"}
ARM_MODES = ("idle", "teleop", "returning")
SESSION_ACTIONS = ("start", "return", "shutdown")
SESSION_STATES = ("idle", "teleop", "returning", "fault")
COMPONENT_ROLES = (
    "source", "producer_arm", "producer_hand", "coordinator_arm",
    "executor_arm", "executor_hand", "recorder",
)
HAND_FRAME = "wrist_relative_mediapipe"
DIAGNOSTIC_FRAME = "motive_world"
ARM_JOINT_NAMES = {
    side: tuple(f"Joint{i}_{'L' if side == 'left' else 'R'}" for i in range(1, 8))
    for side in SIDES
}
_HAND_BASE_NAMES = (
    "thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp", "thumb_ip",
    "index_mcp_flex", "index_mcp_abd", "index_pip", "index_dip",
    "middle_mcp_flex", "middle_mcp_abd", "middle_pip", "middle_dip",
    "ring_mcp_flex", "ring_mcp_abd", "ring_pip", "ring_dip",
    "pinky_mcp_flex", "pinky_mcp_abd", "pinky_pip", "pinky_dip",
)
HAND_JOINT_NAMES = {
    side: tuple(f"{'l' if side == 'left' else 'r'}_{name}" for name in _HAND_BASE_NAMES)
    for side in SIDES
}
ALL_ARM_JOINT_NAMES = ARM_JOINT_NAMES["left"] + ARM_JOINT_NAMES["right"]


class ProtocolError(ValueError):
    """Raised when a message is missing, malformed, or unsupported."""

def strict_loads(payload: str | bytes | bytearray) -> dict[str, Any]:
    """Decode a protocol JSON object without accepting ambiguous wire data.

    ``json.loads`` accepts duplicate object members and non-standard constants
    by default.  Both are unsafe for schema messages because a producer could
    hide one value behind another or silently turn NaN into a valid-looking
    array element.  Keep this decoder at the protocol boundary so every
    consumer can share the same rejection behavior.
    """
    def reject_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        value: dict[str, Any] = {}
        for key, item in pairs:
            if key in value:
                raise ProtocolError(f"duplicate field: {key}")
            value[key] = item
        return value

    def reject_constant(value: str) -> Any:
        raise ProtocolError(f"non-finite JSON constant: {value}")

    try:
        value = json.loads(
            payload,
            object_pairs_hook=reject_duplicates,
            parse_constant=reject_constant,
        )
    except ProtocolError:
        raise
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"invalid JSON payload: {exc}") from exc
    if not isinstance(value, dict):
        raise ProtocolError("protocol payload must be a JSON object")
    return value


def parse_message(payload: str | bytes | bytearray, message_type: Any) -> Any:
    """Strictly decode ``payload`` and pass it through a typed parser."""
    return message_type.from_dict(strict_loads(payload))


def _mapping(value: Any) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ProtocolError("message must be a JSON object")
    return value


def _keys(value: Mapping[str, Any], expected: set[str]) -> None:
    missing = expected - set(value)
    extra = set(value) - expected
    if missing:
        raise ProtocolError(f"missing fields: {', '.join(sorted(missing))}")
    if extra:
        raise ProtocolError(f"unknown fields: {', '.join(sorted(extra))}")


def _string(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value:
        raise ProtocolError(f"{field} must be a non-empty string")
    return value


def _integer(value: Any, field: str, *, nonnegative: bool = True) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProtocolError(f"{field} must be an integer")
    if nonnegative and value < 0:
        raise ProtocolError(f"{field} must be non-negative")
    return value


def _nullable_integer(value: Any, field: str) -> int | None:
    return None if value is None else _integer(value, field)


def _finite(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ProtocolError(f"{field} must be finite")
    return float(value)


def _vector(value: Any, size: int, field: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ProtocolError(f"{field} must have shape [{size}]")
    return [_finite(item, f"{field}[{index}]") for index, item in enumerate(value)]


def _matrix(value: Any, rows: int, cols: int, field: str) -> list[list[float]]:
    if not isinstance(value, (list, tuple)) or len(value) != rows:
        raise ProtocolError(f"{field} must have shape [{rows},{cols}]")
    return [_vector(row, cols, f"{field}[{index}]") for index, row in enumerate(value)]


def _edges(value: Any) -> list[list[int]]:
    if not isinstance(value, (list, tuple)) or len(value) != 20:
        raise ProtocolError("edges must have shape [20,2]")
    result: list[list[int]] = []
    for index, edge in enumerate(value):
        if not isinstance(edge, (list, tuple)) or len(edge) != 2:
            raise ProtocolError(f"edges[{index}] must have shape [2]")
        a, b = edge
        if isinstance(a, bool) or isinstance(b, bool) or not isinstance(a, int) or not isinstance(b, int):
            raise ProtocolError("edge indices must be integers")
        if not (0 <= a < 21 and 0 <= b < 21):
            raise ProtocolError("edge indices must reference keypoints 0..20")
        result.append([a, b])
    return result


def _side(value: Any) -> str:
    value = _string(value, "side")
    if value not in SIDES:
        raise ProtocolError("side must be left or right")
    return value


def _schema(value: Any) -> int:
    if _integer(value, "schema_version") != SCHEMA_VERSION:
        raise ProtocolError(f"unsupported schema_version (expected {SCHEMA_VERSION})")
    return SCHEMA_VERSION

def _identity(value: Any, field: str) -> str:
    return _string(value, field)


def _quaternion(value: Any, field: str, *, parse: bool = False) -> list[float]:
    result = _vector(value, 4, field)
    norm = math.sqrt(sum(component * component for component in result))
    if norm == 0.0:
        raise ProtocolError(f"{field} must not be zero")
    if parse and not 0.999 <= norm <= 1.001:
        raise ProtocolError(f"{field} norm must be in [0.999, 1.001]")
    return [component / norm for component in result]


def _elbow(value: Any, field: str) -> list[float]:
    result = _vector(value, 3, field)
    norm = math.sqrt(sum(component * component for component in result))
    if norm < 1e-8:
        raise ProtocolError(f"{field} norm must be at least 1e-8")
    return [component / norm for component in result]


def _pose(value: Any, field: str, *, parse: bool = False) -> list[float]:
    result = _vector(value, 7, field)
    result[3:] = _quaternion(result[3:], f"{field}[3:7]", parse=parse)
    return result


def _names(value: Any, expected: Sequence[str], field: str) -> list[str]:
    if not isinstance(value, (list, tuple)) or len(value) != len(expected):
        raise ProtocolError(f"{field} must contain {len(expected)} names")
    result = [_string(item, f"{field}[{index}]") for index, item in enumerate(value)]
    if tuple(result) != tuple(expected):
        raise ProtocolError(f"{field} has invalid joint order")
    return result


def _json_data(value: Any, field: str) -> Any:
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ProtocolError(f"{field} must contain only finite JSON values")
        return value
    if isinstance(value, (list, tuple)):
        return [_json_data(item, f"{field}[{index}]") for index, item in enumerate(value)]
    if isinstance(value, Mapping):
        result: dict[str, Any] = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise ProtocolError(f"{field} object keys must be strings")
            result[key] = _json_data(item, f"{field}.{key}")
        return result
    raise ProtocolError(f"{field} contains a non-JSON value")


def _dict(value: Any, field: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ProtocolError(f"{field} must be an object")
    return _json_data(value, field)

def _capabilities(value: Any) -> list[str]:
    if not isinstance(value, (list, tuple, set)):
        raise ProtocolError("capabilities must be an array")
    result = [_string(item, "capabilities[]") for item in value]
    if not {"simulation", "real"}.intersection(result):
        raise ProtocolError("capabilities must include simulation or real")
    return result


def _wire_fields(schema: int, publisher_instance_id: str, router_zid: str, sequence: int, timestamp_ns: int) -> dict[str, Any]:
    return {
        "schema_version": schema,
        "publisher_instance_id": publisher_instance_id,
        "router_zid": router_zid,
        "sequence": sequence,
        "timestamp_ns": timestamp_ns,
    }


def _parse_wire(data: Any, payload_fields: set[str]) -> tuple[int, str, str, int, int, Mapping[str, Any]]:
    value = _mapping(data)
    expected = {"schema_version", "publisher_instance_id", "router_zid", "sequence", "timestamp_ns"} | payload_fields
    _keys(value, expected)
    schema = _schema(value["schema_version"])
    instance = _identity(value["publisher_instance_id"], "publisher_instance_id")
    router = _identity(value["router_zid"], "router_zid")
    sequence = _integer(value["sequence"], "sequence")
    timestamp = _integer(value["timestamp_ns"], "timestamp_ns")
    return schema, instance, router, sequence, timestamp, value


@dataclass(eq=True)
class ProtocolEnvelope:
    schema_version: int
    publisher_instance_id: str
    router_zid: str
    sequence: int
    timestamp_ns: int

    def __post_init__(self) -> None:
        _schema(self.schema_version)
        _identity(self.publisher_instance_id, "publisher_instance_id")
        _identity(self.router_zid, "router_zid")
        _integer(self.sequence, "sequence")
        _integer(self.timestamp_ns, "timestamp_ns")

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "publisher_instance_id": self.publisher_instance_id,
            "router_zid": self.router_zid,
            "sequence": self.sequence,
            "timestamp_ns": self.timestamp_ns,
        }

    @classmethod
    def from_dict(cls, data: Any) -> "ProtocolEnvelope":
        value = _mapping(data)
        _keys(value, {"schema_version", "publisher_instance_id", "router_zid", "sequence", "timestamp_ns"})
        return cls(**dict(value))


@dataclass(eq=True)
class ArmTargetCommand:
    envelope: ProtocolEnvelope
    source_timestamp_ns: int | None
    source: str
    side: str
    frame_id: str
    position_m: list[float]
    orientation_xyzw: list[float]
    elbow_reference_direction: list[float]

    def __post_init__(self) -> None:
        if not isinstance(self.envelope, ProtocolEnvelope):
            raise ProtocolError("envelope must be ProtocolEnvelope")
        _nullable_integer(self.source_timestamp_ns, "source_timestamp_ns")
        _string(self.source, "source")
        side = _side(self.side)
        if self.frame_id != ARM_FRAMES[side]:
            raise ProtocolError(f"{side} arm must use frame_id={ARM_FRAMES[side]}")
        self.position_m = _vector(self.position_m, 3, "position_m")
        self.orientation_xyzw = _quaternion(self.orientation_xyzw, "orientation_xyzw")
        self.elbow_reference_direction = _elbow(self.elbow_reference_direction, "elbow_reference_direction")

    def to_dict(self) -> dict[str, Any]:
        return {**self.envelope.to_dict(), "source_timestamp_ns": self.source_timestamp_ns, "source": self.source,
                "side": self.side, "frame_id": self.frame_id, "position_m": self.position_m,
                "orientation_xyzw": self.orientation_xyzw, "elbow_reference_direction": self.elbow_reference_direction}

    @classmethod
    def from_dict(cls, data: Any) -> "ArmTargetCommand":
        value = _mapping(data)
        payload = {"source_timestamp_ns", "source", "side", "frame_id", "position_m", "orientation_xyzw", "elbow_reference_direction"}
        _keys(value, {"schema_version", "publisher_instance_id", "router_zid", "sequence", "timestamp_ns"} | payload)
        envelope = ProtocolEnvelope.from_dict({key: value[key] for key in ("schema_version", "publisher_instance_id", "router_zid", "sequence", "timestamp_ns")})
        _vector(value["position_m"], 3, "position_m")
        _quaternion(value["orientation_xyzw"], "orientation_xyzw", parse=True)
        _elbow(value["elbow_reference_direction"], "elbow_reference_direction")
        return cls(envelope=envelope, **{key: value[key] for key in payload})

@dataclass(eq=True)
class ArmSolvedPose:
    envelope: ProtocolEnvelope
    producer: str
    side: str
    frame_id: str
    target_sequence: int | None
    position_m: list[float]
    orientation_xyzw: list[float]

    def __post_init__(self) -> None:
        if not isinstance(self.envelope, ProtocolEnvelope):
            raise ProtocolError("envelope must be ProtocolEnvelope")
        _string(self.producer, "producer")
        side = _side(self.side)
        if self.frame_id != ARM_FRAMES[side]:
            raise ProtocolError(f"{side} arm must use frame_id={ARM_FRAMES[side]}")
        _nullable_integer(self.target_sequence, "target_sequence")
        self.position_m = _vector(self.position_m, 3, "position_m")
        self.orientation_xyzw = _quaternion(self.orientation_xyzw, "orientation_xyzw")

    def to_dict(self) -> dict[str, Any]:
        return {**self.envelope.to_dict(), "producer": self.producer, "side": self.side, "frame_id": self.frame_id,
                "target_sequence": self.target_sequence, "position_m": self.position_m, "orientation_xyzw": self.orientation_xyzw}

    @classmethod
    def from_dict(cls, data: Any) -> "ArmSolvedPose":
        value = _mapping(data)
        payload = {"producer", "side", "frame_id", "target_sequence", "position_m", "orientation_xyzw"}
        _, _, _, _, _, _ = _parse_wire(value, payload)
        _vector(value["position_m"], 3, "position_m")
        _quaternion(value["orientation_xyzw"], "orientation_xyzw", parse=True)
        envelope = ProtocolEnvelope.from_dict({key: value[key] for key in ("schema_version", "publisher_instance_id", "router_zid", "sequence", "timestamp_ns")})
        return cls(envelope=envelope, **{key: value[key] for key in payload})


@dataclass(eq=True)
class ArmJointProposal:
    schema_version: int
    sequence: int
    timestamp_ns: int
    producer: str
    side: str
    target_sequence: int | None
    names: list[str]
    position_rad: list[float]
    diagnostics: dict[str, Any]
    publisher_instance_id: str
    router_zid: str

    def __post_init__(self) -> None:
        _schema(self.schema_version); _integer(self.sequence, "sequence"); _integer(self.timestamp_ns, "timestamp_ns")
        _string(self.producer, "producer"); side = _side(self.side); _nullable_integer(self.target_sequence, "target_sequence")
        self.names = _names(self.names, ARM_JOINT_NAMES[side], "names")
        self.position_rad = _vector(self.position_rad, 7, "position_rad")
        self.diagnostics = _dict(self.diagnostics, "diagnostics")
        _identity(self.publisher_instance_id, "publisher_instance_id"); _identity(self.router_zid, "router_zid")

    def to_dict(self) -> dict[str, Any]:
        return {**_wire_fields(self.schema_version, self.publisher_instance_id, self.router_zid, self.sequence, self.timestamp_ns),
                "producer": self.producer, "side": self.side, "target_sequence": self.target_sequence,
                "names": self.names, "position_rad": self.position_rad, "diagnostics": self.diagnostics}

    @classmethod
    def from_dict(cls, data: Any) -> "ArmJointProposal":
        schema, instance, router, sequence, timestamp, value = _parse_wire(data, {"producer", "side", "target_sequence", "names", "position_rad", "diagnostics"})
        return cls(schema, sequence, timestamp, value["producer"], value["side"], value["target_sequence"], value["names"], value["position_rad"], value["diagnostics"], instance, router)


@dataclass(eq=True)
class ArmJointCommand:
    schema_version: int
    sequence: int
    timestamp_ns: int
    producer: str
    side: str
    mode: str
    proposal_sequence: int | None
    target_sequence: int | None
    names: list[str]
    position_rad: list[float]
    publisher_instance_id: str
    router_zid: str

    def __post_init__(self) -> None:
        _schema(self.schema_version); _integer(self.sequence, "sequence"); _integer(self.timestamp_ns, "timestamp_ns")
        _string(self.producer, "producer"); side = _side(self.side)
        if self.mode not in ARM_MODES: raise ProtocolError("mode must be idle, teleop, or returning")
        _nullable_integer(self.proposal_sequence, "proposal_sequence"); _nullable_integer(self.target_sequence, "target_sequence")
        self.names = _names(self.names, ARM_JOINT_NAMES[side], "names"); self.position_rad = _vector(self.position_rad, 7, "position_rad")
        _identity(self.publisher_instance_id, "publisher_instance_id"); _identity(self.router_zid, "router_zid")

    def to_dict(self) -> dict[str, Any]:
        return {**_wire_fields(self.schema_version, self.publisher_instance_id, self.router_zid, self.sequence, self.timestamp_ns),
                "producer": self.producer, "side": self.side, "mode": self.mode, "proposal_sequence": self.proposal_sequence,
                "target_sequence": self.target_sequence, "names": self.names, "position_rad": self.position_rad}

    @classmethod
    def from_dict(cls, data: Any) -> "ArmJointCommand":
        schema, instance, router, sequence, timestamp, value = _parse_wire(data, {"producer", "side", "mode", "proposal_sequence", "target_sequence", "names", "position_rad"})
        return cls(schema, sequence, timestamp, value["producer"], value["side"], value["mode"], value["proposal_sequence"], value["target_sequence"], value["names"], value["position_rad"], instance, router)


@dataclass(eq=True)
class ArmJointState:
    schema_version: int
    sequence: int
    timestamp_ns: int
    executor: str
    names: list[str]
    position_rad: list[float]
    velocity_rad_s: list[float] | None
    publisher_instance_id: str
    router_zid: str

    def __post_init__(self) -> None:
        _schema(self.schema_version); _integer(self.sequence, "sequence"); _integer(self.timestamp_ns, "timestamp_ns"); _string(self.executor, "executor")
        self.names = _names(self.names, ALL_ARM_JOINT_NAMES, "names"); self.position_rad = _vector(self.position_rad, 14, "position_rad")
        self.velocity_rad_s = None if self.velocity_rad_s is None else _vector(self.velocity_rad_s, 14, "velocity_rad_s")
        _identity(self.publisher_instance_id, "publisher_instance_id"); _identity(self.router_zid, "router_zid")

    def to_dict(self) -> dict[str, Any]:
        return {**_wire_fields(self.schema_version, self.publisher_instance_id, self.router_zid, self.sequence, self.timestamp_ns),
                "executor": self.executor, "names": self.names, "position_rad": self.position_rad, "velocity_rad_s": self.velocity_rad_s}

    @classmethod
    def from_dict(cls, data: Any) -> "ArmJointState":
        schema, instance, router, sequence, timestamp, value = _parse_wire(data, {"executor", "names", "position_rad", "velocity_rad_s"})
        return cls(schema, sequence, timestamp, value["executor"], value["names"], value["position_rad"], value["velocity_rad_s"], instance, router)


@dataclass(eq=True)
class HandTargetCommand:
    schema_version: int
    sequence: int
    timestamp_ns: int
    source_timestamp_ns: int | None
    source: str
    side: str
    frame_id: str
    keypoints_m: list[list[float]]
    publisher_instance_id: str
    router_zid: str

    def __post_init__(self) -> None:
        _schema(self.schema_version); _integer(self.sequence, "sequence"); _integer(self.timestamp_ns, "timestamp_ns"); _nullable_integer(self.source_timestamp_ns, "source_timestamp_ns")
        _string(self.source, "source"); _side(self.side)
        if self.frame_id != HAND_FRAME: raise ProtocolError(f"hand target must use frame_id={HAND_FRAME}")
        self.keypoints_m = _matrix(self.keypoints_m, 21, 3, "keypoints_m")
        if any(component != 0.0 for component in self.keypoints_m[0]): raise ProtocolError("keypoints_m[0] wrist must be exactly [0, 0, 0]")
        _identity(self.publisher_instance_id, "publisher_instance_id"); _identity(self.router_zid, "router_zid")

    def to_dict(self) -> dict[str, Any]:
        return {**_wire_fields(self.schema_version, self.publisher_instance_id, self.router_zid, self.sequence, self.timestamp_ns),
                "source_timestamp_ns": self.source_timestamp_ns, "source": self.source, "side": self.side,
                "frame_id": self.frame_id, "keypoints_m": self.keypoints_m}

    @classmethod
    def from_dict(cls, data: Any) -> "HandTargetCommand":
        schema, instance, router, sequence, timestamp, value = _parse_wire(data, {"source_timestamp_ns", "source", "side", "frame_id", "keypoints_m"})
        return cls(schema, sequence, timestamp, value["source_timestamp_ns"], value["source"], value["side"], value["frame_id"], value["keypoints_m"], instance, router)


@dataclass(eq=True)
class HandJointCommand:
    schema_version: int
    sequence: int
    timestamp_ns: int
    producer: str
    side: str
    names: list[str]
    position_rad: list[float]
    publisher_instance_id: str
    router_zid: str

    def __post_init__(self) -> None:
        _schema(self.schema_version); _integer(self.sequence, "sequence"); _integer(self.timestamp_ns, "timestamp_ns"); _string(self.producer, "producer"); side = _side(self.side)
        self.names = _names(self.names, HAND_JOINT_NAMES[side], "names"); self.position_rad = _vector(self.position_rad, 20, "position_rad")
        _identity(self.publisher_instance_id, "publisher_instance_id"); _identity(self.router_zid, "router_zid")

    def to_dict(self) -> dict[str, Any]:
        return {**_wire_fields(self.schema_version, self.publisher_instance_id, self.router_zid, self.sequence, self.timestamp_ns), "producer": self.producer, "side": self.side, "names": self.names, "position_rad": self.position_rad}

    @classmethod
    def from_dict(cls, data: Any) -> "HandJointCommand":
        schema, instance, router, sequence, timestamp, value = _parse_wire(data, {"producer", "side", "names", "position_rad"})
        return cls(schema, sequence, timestamp, value["producer"], value["side"], value["names"], value["position_rad"], instance, router)


@dataclass(eq=True)
class HandJointState:
    schema_version: int
    sequence: int
    timestamp_ns: int
    executor: str
    side: str
    names: list[str]
    position_rad: list[float]
    velocity_rad_s: list[float] | None
    publisher_instance_id: str
    router_zid: str

    def __post_init__(self) -> None:
        _schema(self.schema_version); _integer(self.sequence, "sequence"); _integer(self.timestamp_ns, "timestamp_ns"); _string(self.executor, "executor"); side = _side(self.side)
        self.names = _names(self.names, HAND_JOINT_NAMES[side], "names"); self.position_rad = _vector(self.position_rad, 20, "position_rad"); self.velocity_rad_s = None if self.velocity_rad_s is None else _vector(self.velocity_rad_s, 20, "velocity_rad_s")
        _identity(self.publisher_instance_id, "publisher_instance_id"); _identity(self.router_zid, "router_zid")

    def to_dict(self) -> dict[str, Any]:
        return {**_wire_fields(self.schema_version, self.publisher_instance_id, self.router_zid, self.sequence, self.timestamp_ns), "executor": self.executor, "side": self.side, "names": self.names, "position_rad": self.position_rad, "velocity_rad_s": self.velocity_rad_s}

    @classmethod
    def from_dict(cls, data: Any) -> "HandJointState":
        schema, instance, router, sequence, timestamp, value = _parse_wire(data, {"executor", "side", "names", "position_rad", "velocity_rad_s"})
        return cls(schema, sequence, timestamp, value["executor"], value["side"], value["names"], value["position_rad"], value["velocity_rad_s"], instance, router)


@dataclass(eq=True)
class SessionIntent:
    schema_version: int
    sequence: int
    timestamp_ns: int
    source: str
    action: str
    reason: str
    publisher_instance_id: str
    router_zid: str

    def __post_init__(self) -> None:
        _schema(self.schema_version); _integer(self.sequence, "sequence"); _integer(self.timestamp_ns, "timestamp_ns"); _string(self.source, "source"); _string(self.reason, "reason")
        if self.action not in SESSION_ACTIONS: raise ProtocolError("action must be start, return, or shutdown")
        _identity(self.publisher_instance_id, "publisher_instance_id"); _identity(self.router_zid, "router_zid")

    def to_dict(self) -> dict[str, Any]:
        return {**_wire_fields(self.schema_version, self.publisher_instance_id, self.router_zid, self.sequence, self.timestamp_ns), "source": self.source, "action": self.action, "reason": self.reason}

    @classmethod
    def from_dict(cls, data: Any) -> "SessionIntent":
        schema, instance, router, sequence, timestamp, value = _parse_wire(data, {"source", "action", "reason"})
        return cls(schema, sequence, timestamp, value["source"], value["action"], value["reason"], instance, router)


@dataclass(eq=True)
class SessionState:
    schema_version: int
    sequence: int
    timestamp_ns: int
    state: str
    reason: str
    source: str
    intent_sequence: int | None
    publisher_instance_id: str
    router_zid: str

    def __post_init__(self) -> None:
        _schema(self.schema_version); _integer(self.sequence, "sequence"); _integer(self.timestamp_ns, "timestamp_ns"); _string(self.reason, "reason"); _string(self.source, "source"); _nullable_integer(self.intent_sequence, "intent_sequence")
        if self.state not in SESSION_STATES: raise ProtocolError("state must be idle, teleop, returning, or fault")
        _identity(self.publisher_instance_id, "publisher_instance_id"); _identity(self.router_zid, "router_zid")

    def to_dict(self) -> dict[str, Any]:
        return {**_wire_fields(self.schema_version, self.publisher_instance_id, self.router_zid, self.sequence, self.timestamp_ns), "state": self.state, "reason": self.reason, "source": self.source, "intent_sequence": self.intent_sequence}

    @classmethod
    def from_dict(cls, data: Any) -> "SessionState":
        schema, instance, router, sequence, timestamp, value = _parse_wire(data, {"state", "reason", "source", "intent_sequence"})
        return cls(schema, sequence, timestamp, value["state"], value["reason"], value["source"], value["intent_sequence"], instance, router)


@dataclass(eq=True)
class LatchedBool:
    schema_version: int
    sequence: int
    timestamp_ns: int
    value: bool
    publisher_instance_id: str
    router_zid: str

    def __post_init__(self) -> None:
        _schema(self.schema_version); _integer(self.sequence, "sequence"); _integer(self.timestamp_ns, "timestamp_ns")
        if not isinstance(self.value, bool): raise ProtocolError("value must be boolean")
        _identity(self.publisher_instance_id, "publisher_instance_id"); _identity(self.router_zid, "router_zid")

    def to_dict(self) -> dict[str, Any]: return {**_wire_fields(self.schema_version, self.publisher_instance_id, self.router_zid, self.sequence, self.timestamp_ns), "value": self.value}

    @classmethod
    def from_dict(cls, data: Any) -> "LatchedBool":
        schema, instance, router, sequence, timestamp, value = _parse_wire(data, {"value"})
        return cls(schema, sequence, timestamp, value["value"], instance, router)


@dataclass(eq=True)
class ComponentStatus:
    schema_version: int
    sequence: int
    timestamp_ns: int
    component_role: str
    component_id: str
    phase: str
    ready: bool
    healthy: bool
    capabilities: list[str]
    error: str | None
    diagnostics: dict[str, Any]
    publisher_instance_id: str
    router_zid: str

    def __post_init__(self) -> None:
        _schema(self.schema_version); _integer(self.sequence, "sequence"); _integer(self.timestamp_ns, "timestamp_ns")
        if self.component_role not in COMPONENT_ROLES: raise ProtocolError("invalid component_role")
        _string(self.component_id, "component_id"); _string(self.phase, "phase")
        if not isinstance(self.ready, bool) or not isinstance(self.healthy, bool): raise ProtocolError("ready and healthy must be boolean")
        self.capabilities = _capabilities(self.capabilities)
        if self.error is not None: _string(self.error, "error")
        self.diagnostics = _dict(self.diagnostics, "diagnostics")
        _identity(self.publisher_instance_id, "publisher_instance_id"); _identity(self.router_zid, "router_zid")

    def to_dict(self) -> dict[str, Any]:
        return {"schema_version": self.schema_version, "publisher_instance_id": self.publisher_instance_id, "router_zid": self.router_zid, "sequence": self.sequence, "timestamp_ns": self.timestamp_ns, "component_role": self.component_role, "component_id": self.component_id, "phase": self.phase, "ready": self.ready, "healthy": self.healthy, "capabilities": self.capabilities, "error": self.error, "diagnostics": self.diagnostics}

    @classmethod
    def from_dict(cls, data: Any) -> "ComponentStatus":
        value = _mapping(data); payload = {"sequence", "timestamp_ns", "component_role", "component_id", "phase", "ready", "healthy", "capabilities", "error", "diagnostics"}
        _keys(value, {"schema_version", "publisher_instance_id", "router_zid"} | payload)
        return cls(value["schema_version"], value["sequence"], value["timestamp_ns"], value["component_role"], value["component_id"], value["phase"], value["ready"], value["healthy"], value["capabilities"], value["error"], value["diagnostics"], value["publisher_instance_id"], value["router_zid"])


@dataclass(eq=True)
class HandExecutorStatus:
    schema_version: int
    sequence: int
    timestamp_ns: int
    side: str
    ready: bool
    healthy: bool
    at_zero: bool
    tracking_allowed: bool
    error: str | None
    publisher_instance_id: str
    router_zid: str

    def __post_init__(self) -> None:
        _schema(self.schema_version); _integer(self.sequence, "sequence"); _integer(self.timestamp_ns, "timestamp_ns"); _side(self.side)
        for field in ("ready", "healthy", "at_zero", "tracking_allowed"):
            if not isinstance(getattr(self, field), bool): raise ProtocolError(f"{field} must be boolean")
        if self.error is not None: _string(self.error, "error")
        _identity(self.publisher_instance_id, "publisher_instance_id"); _identity(self.router_zid, "router_zid")

    def to_dict(self) -> dict[str, Any]:
        return {**_wire_fields(self.schema_version, self.publisher_instance_id, self.router_zid, self.sequence, self.timestamp_ns), "side": self.side, "ready": self.ready, "healthy": self.healthy, "at_zero": self.at_zero, "tracking_allowed": self.tracking_allowed, "error": self.error}

    @classmethod
    def from_dict(cls, data: Any) -> "HandExecutorStatus":
        schema, instance, router, sequence, timestamp, value = _parse_wire(data, {"side", "ready", "healthy", "at_zero", "tracking_allowed", "error"})
        return cls(schema, sequence, timestamp, value["side"], value["ready"], value["healthy"], value["at_zero"], value["tracking_allowed"], value["error"], instance, router)


@dataclass(eq=True)
class SafetyStopRequest:
    envelope: ProtocolEnvelope
    run_id: str
    reason: str
    latch: bool = True

    def __post_init__(self) -> None:
        if not isinstance(self.envelope, ProtocolEnvelope): raise ProtocolError("envelope must be ProtocolEnvelope")
        _string(self.run_id, "run_id"); _string(self.reason, "reason")
        if not isinstance(self.latch, bool): raise ProtocolError("latch must be boolean")
        if not self.latch: raise ProtocolError("safety stop requests must be latched")

    def to_dict(self) -> dict[str, Any]: return {**self.envelope.to_dict(), "run_id": self.run_id, "reason": self.reason, "latch": self.latch}

    @classmethod
    def from_dict(cls, data: Any) -> "SafetyStopRequest":
        value = _mapping(data); _keys(value, {"schema_version", "publisher_instance_id", "router_zid", "sequence", "timestamp_ns", "run_id", "reason", "latch"})
        env = ProtocolEnvelope.from_dict({key: value[key] for key in ("schema_version", "publisher_instance_id", "router_zid", "sequence", "timestamp_ns")})
        return cls(env, value["run_id"], value["reason"], value["latch"])

    def validate_authority(self, expected_supervisor_instance_id: str, expected_run_id: str) -> None:
        """Validate launcher authorization and active-run binding before consuming a stop."""
        _identity(expected_supervisor_instance_id, "expected_supervisor_instance_id")
        _identity(expected_run_id, "expected_run_id")
        if self.envelope.publisher_instance_id != expected_supervisor_instance_id:
            raise ProtocolError("safety stop publisher is not the authorized supervisor")
        if self.run_id != expected_run_id:
            raise ProtocolError("safety stop run_id does not match active run")


@dataclass(eq=True)
class SafetyStopAck:
    envelope: ProtocolEnvelope
    executor_id: str
    run_id: str
    latched: bool
    reason: str

    def __post_init__(self) -> None:
        if not isinstance(self.envelope, ProtocolEnvelope): raise ProtocolError("envelope must be ProtocolEnvelope")
        _string(self.executor_id, "executor_id"); _string(self.run_id, "run_id"); _string(self.reason, "reason")
        if not isinstance(self.latched, bool): raise ProtocolError("latched must be boolean")

    def to_dict(self) -> dict[str, Any]: return {**self.envelope.to_dict(), "executor_id": self.executor_id, "run_id": self.run_id, "latched": self.latched, "reason": self.reason}

    @classmethod
    def from_dict(cls, data: Any) -> "SafetyStopAck":
        value = _mapping(data); _keys(value, {"schema_version", "publisher_instance_id", "router_zid", "sequence", "timestamp_ns", "executor_id", "run_id", "latched", "reason"})
        env = ProtocolEnvelope.from_dict({key: value[key] for key in ("schema_version", "publisher_instance_id", "router_zid", "sequence", "timestamp_ns")})
        return cls(env, value["executor_id"], value["run_id"], value["latched"], value["reason"])

    def validate_for(self, expected_executor_id: str, expected_run_id: str) -> None:
        """Validate that an acknowledgement belongs to this executor and run."""
        _identity(expected_executor_id, "expected_executor_id")
        _identity(expected_run_id, "expected_run_id")
        if self.executor_id != expected_executor_id:
            raise ProtocolError("safety stop ack executor_id mismatch")
        if self.run_id != expected_run_id:
            raise ProtocolError("safety stop ack run_id mismatch")
        if not self.latched:
            raise ProtocolError("safety stop ack must be latched")


@dataclass(eq=True)
class RawPicoControllerSample:
    envelope: ProtocolEnvelope
    source_timestamp_ns: int | None
    left_pose: list[float]
    right_pose: list[float]
    right_a_pressed: bool
    source_type: str = "pico_controller"

    def __post_init__(self) -> None:
        if not isinstance(self.envelope, ProtocolEnvelope): raise ProtocolError("envelope must be ProtocolEnvelope")
        if self.source_type != "pico_controller": raise ProtocolError("source_type is fixed to pico_controller")
        _nullable_integer(self.source_timestamp_ns, "source_timestamp_ns"); self.left_pose = _pose(self.left_pose, "left_pose"); self.right_pose = _pose(self.right_pose, "right_pose")
        if not isinstance(self.right_a_pressed, bool): raise ProtocolError("right_a_pressed must be boolean")

    def to_dict(self) -> dict[str, Any]: return {**self.envelope.to_dict(), "source_timestamp_ns": self.source_timestamp_ns, "source_type": self.source_type, "left_pose": self.left_pose, "right_pose": self.right_pose, "right_a_pressed": self.right_a_pressed}

    @classmethod
    def from_dict(cls, data: Any) -> "RawPicoControllerSample":
        value = _mapping(data)
        payload = {"source_timestamp_ns", "source_type", "left_pose", "right_pose", "right_a_pressed"}
        _keys(value, {"schema_version", "publisher_instance_id", "router_zid", "sequence", "timestamp_ns"} | payload)
        _pose(value["left_pose"], "left_pose", parse=True)
        _pose(value["right_pose"], "right_pose", parse=True)
        env = ProtocolEnvelope.from_dict({key: value[key] for key in ("schema_version", "publisher_instance_id", "router_zid", "sequence", "timestamp_ns")})
        return cls(env, value["source_timestamp_ns"], value["left_pose"], value["right_pose"], value["right_a_pressed"], value["source_type"])


def _validate_hand_record(value: Any, side: str, *, h5: bool) -> dict[str, Any]:
    value = _mapping(value); expected = {"valid", "wrist_pose", "keypoints_world_m"} | ({"wuji2_joints_rad"} if h5 else set()); _keys(value, expected)
    if not isinstance(value["valid"], bool): raise ProtocolError("hand valid must be boolean")
    if value["valid"]:
        if value["wrist_pose"] is None or value["keypoints_world_m"] is None: raise ProtocolError("valid hand requires pose and keypoints")
        wrist = _pose(value["wrist_pose"], f"hands.{side}.wrist_pose", parse=True); points = _matrix(value["keypoints_world_m"], 21, 3, f"hands.{side}.keypoints_world_m")
    else:
        if value["wrist_pose"] is not None or value["keypoints_world_m"] is not None: raise ProtocolError("invalid hand fields must be null")
        wrist = points = None
    joints = value.get("wuji2_joints_rad")
    if h5 and joints is not None: joints = _vector(joints, 20, f"hands.{side}.wuji2_joints_rad")
    if h5 and not value["valid"] and joints is not None: raise ProtocolError("invalid hand wuji2_joints_rad must be null")
    result = {"valid": value["valid"], "wrist_pose": wrist, "keypoints_world_m": points}
    if h5: result["wuji2_joints_rad"] = joints
    return result


@dataclass(eq=True)
class RawMocapLiveSample:
    envelope: ProtocolEnvelope
    source_timestamp_ns: int | None
    stream_instance_id: str
    stream_sequence: int
    frame_index: int
    hands: dict[str, dict[str, Any]]
    source_type: str = "mocap_live"

    def __post_init__(self) -> None:
        if not isinstance(self.envelope, ProtocolEnvelope): raise ProtocolError("envelope must be ProtocolEnvelope")
        if self.source_type != "mocap_live": raise ProtocolError("source_type is fixed to mocap_live")
        _nullable_integer(self.source_timestamp_ns, "source_timestamp_ns"); _identity(self.stream_instance_id, "stream_instance_id"); _integer(self.stream_sequence, "stream_sequence"); _integer(self.frame_index, "frame_index")
        if set(self.hands) != set(SIDES): raise ProtocolError("hands must contain exactly left and right")
        self.hands = {side: _validate_hand_record(self.hands[side], side, h5=False) for side in SIDES}

    def to_dict(self) -> dict[str, Any]: return {**self.envelope.to_dict(), "source_timestamp_ns": self.source_timestamp_ns, "source_type": self.source_type, "stream_instance_id": self.stream_instance_id, "stream_sequence": self.stream_sequence, "frame_index": self.frame_index, "hands": self.hands}

    @classmethod
    def from_dict(cls, data: Any) -> "RawMocapLiveSample":
        value = _mapping(data); payload = {"source_timestamp_ns", "source_type", "stream_instance_id", "stream_sequence", "frame_index", "hands"}; _keys(value, {"schema_version", "publisher_instance_id", "router_zid", "sequence", "timestamp_ns"} | payload); env = ProtocolEnvelope.from_dict({key: value[key] for key in ("schema_version", "publisher_instance_id", "router_zid", "sequence", "timestamp_ns")}); return cls(env, value["source_timestamp_ns"], value["stream_instance_id"], value["stream_sequence"], value["frame_index"], value["hands"], value["source_type"])


@dataclass(eq=True)
class RawH5ReplaySample:
    envelope: ProtocolEnvelope
    source_timestamp_ns: int | None
    hands: dict[str, dict[str, Any]]
    source_type: str = "h5_replay"

    def __post_init__(self) -> None:
        if not isinstance(self.envelope, ProtocolEnvelope): raise ProtocolError("envelope must be ProtocolEnvelope")
        if self.source_type != "h5_replay": raise ProtocolError("source_type is fixed to h5_replay")
        _nullable_integer(self.source_timestamp_ns, "source_timestamp_ns")
        if set(self.hands) != set(SIDES): raise ProtocolError("hands must contain exactly left and right")
        self.hands = {side: _validate_hand_record(self.hands[side], side, h5=True) for side in SIDES}

    def to_dict(self) -> dict[str, Any]: return {**self.envelope.to_dict(), "source_timestamp_ns": self.source_timestamp_ns, "source_type": self.source_type, "hands": self.hands}

    @classmethod
    def from_dict(cls, data: Any) -> "RawH5ReplaySample":
        value = _mapping(data); payload = {"source_timestamp_ns", "source_type", "hands"}; _keys(value, {"schema_version", "publisher_instance_id", "router_zid", "sequence", "timestamp_ns"} | payload); env = ProtocolEnvelope.from_dict({key: value[key] for key in ("schema_version", "publisher_instance_id", "router_zid", "sequence", "timestamp_ns")}); return cls(env, value["source_timestamp_ns"], value["hands"], value["source_type"])


@dataclass(eq=True)
class Frame0HandSkeleton:
    schema_version: int
    timestamp_ns: int
    side: str
    frame_id: str
    keypoints_world_m: list[list[float]]
    edges: list[list[int]]
    manus_wrist_pose: list[float]
    robot_wrist_home_pose: list[float]
    target_wrist_pose: list[float]
    tcp_to_wrist_pose: list[float]
    sequence: int
    publisher_instance_id: str
    router_zid: str

    def __post_init__(self) -> None:
        _schema(self.schema_version); _integer(self.timestamp_ns, "timestamp_ns"); _side(self.side)
        if self.frame_id != DIAGNOSTIC_FRAME: raise ProtocolError(f"frame_id must be {DIAGNOSTIC_FRAME}")
        self.keypoints_world_m = _matrix(self.keypoints_world_m, 21, 3, "keypoints_world_m"); self.edges = _edges(self.edges)
        self.manus_wrist_pose = _pose(self.manus_wrist_pose, "manus_wrist_pose"); self.robot_wrist_home_pose = _pose(self.robot_wrist_home_pose, "robot_wrist_home_pose"); self.target_wrist_pose = _pose(self.target_wrist_pose, "target_wrist_pose"); self.tcp_to_wrist_pose = _pose(self.tcp_to_wrist_pose, "tcp_to_wrist_pose")
        _identity(self.publisher_instance_id, "publisher_instance_id"); _identity(self.router_zid, "router_zid"); _integer(self.sequence, "sequence")

    def to_dict(self) -> dict[str, Any]: return {"schema_version": self.schema_version, "publisher_instance_id": self.publisher_instance_id, "router_zid": self.router_zid, "sequence": self.sequence, "timestamp_ns": self.timestamp_ns, "side": self.side, "frame_id": self.frame_id, "keypoints_world_m": self.keypoints_world_m, "edges": self.edges, "manus_wrist_pose": self.manus_wrist_pose, "robot_wrist_home_pose": self.robot_wrist_home_pose, "target_wrist_pose": self.target_wrist_pose, "tcp_to_wrist_pose": self.tcp_to_wrist_pose}

    @classmethod
    def from_dict(cls, data: Any) -> "Frame0HandSkeleton":
        value = _mapping(data)
        payload = {"timestamp_ns", "side", "frame_id", "keypoints_world_m", "edges", "manus_wrist_pose", "robot_wrist_home_pose", "target_wrist_pose", "tcp_to_wrist_pose"}
        _keys(value, {"schema_version", "publisher_instance_id", "router_zid", "sequence"} | payload)
        _matrix(value["keypoints_world_m"], 21, 3, "keypoints_world_m")
        _edges(value["edges"])
        for field in ("manus_wrist_pose", "robot_wrist_home_pose", "target_wrist_pose", "tcp_to_wrist_pose"):
            _pose(value[field], field, parse=True)
        return cls(value["schema_version"], value["timestamp_ns"], value["side"], value["frame_id"], value["keypoints_world_m"], value["edges"], value["manus_wrist_pose"], value["robot_wrist_home_pose"], value["target_wrist_pose"], value["tcp_to_wrist_pose"], value["sequence"], value["publisher_instance_id"], value["router_zid"])


__all__ = [
    "SCHEMA_VERSION", "SIDES", "ARM_FRAMES", "ARM_MODES", "SESSION_ACTIONS",
    "SESSION_STATES", "COMPONENT_ROLES", "HAND_FRAME", "DIAGNOSTIC_FRAME",
    "ARM_JOINT_NAMES", "HAND_JOINT_NAMES", "ALL_ARM_JOINT_NAMES",
    "ProtocolError", "strict_loads", "parse_message", "ProtocolEnvelope", "ArmTargetCommand", "ArmJointProposal",
    "ArmSolvedPose", "ArmJointCommand", "ArmJointState", "HandTargetCommand",
    "HandJointCommand", "HandJointState", "SessionIntent", "SessionState",
    "LatchedBool", "ComponentStatus", "HandExecutorStatus", "SafetyStopRequest",
    "SafetyStopAck", "RawPicoControllerSample", "RawMocapLiveSample",
    "RawH5ReplaySample", "Frame0HandSkeleton",
]
