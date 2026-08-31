"""Append-only Tianji session-v1 HDF5 storage."""
from __future__ import annotations

from collections.abc import Iterable
import json
from pathlib import Path
import time
from typing import Any

import h5py
import numpy as np

from ..protocol.messages import (
    ArmJointCommand, ArmJointState, ArmTargetCommand, HandJointCommand,
    HandJointState, HandTargetCommand, RawH5ReplaySample, RawMocapLiveSample,
    SessionState, ARM_JOINT_NAMES, HAND_JOINT_NAMES,
    ALL_ARM_JOINT_NAMES,
)

SCHEMA_NAME = "tianji-teleop-session"
SCHEMA_VERSION = "1.0"
SOURCE_TYPES = frozenset({"mocap_live", "h5_replay", "target_replay", "joint_replay"})
_LEGACY_CONTROLLER_GROUP = "pico_controller"
SIDES = ("left", "right")
_STRING = h5py.string_dtype(encoding="utf-8")


class SessionH5Error(ValueError):
    """Invalid session-v1 file or record."""


class IncompleteSessionError(SessionH5Error):
    """A session was interrupted before normal close."""


class UnsafeSessionLinkError(SessionH5Error):
    """Soft and external links are never accepted in a session file."""


def _text(value: Any) -> str:
    if isinstance(value, (bytes, np.bytes_)):
        return bytes(value).decode("utf-8")
    return str(value)


def _json_text(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def _empty_dataset(group: h5py.Group, name: str, shape: tuple[int, ...] = (), *, dtype: Any = np.float64) -> h5py.Dataset:
    return group.create_dataset(name, shape=(0,) + shape, maxshape=(None,) + shape, chunks=(256,) + shape, dtype=dtype)


def _append(dataset: h5py.Dataset, value: Any) -> None:
    index = dataset.shape[0]
    dataset.resize(index + 1, axis=0)
    dataset[index] = value


def _source_time(value: int | None) -> tuple[int, bool]:
    if value is None:
        return 0, False
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or int(value) < 0:
        raise SessionH5Error("source_timestamp_ns must be a non-negative integer or None")
    return int(value), True


def _time_value(value: int | None, clock: Any) -> int:
    if value is None:
        return int(clock())
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise SessionH5Error("received_time_ns must be an integer")
    return int(value)


def _finite(value: Any, field: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if not np.isfinite(result).all():
        raise SessionH5Error(f"{field} contains non-finite values")
    return result


class SessionH5Writer:
    """Create and append one session-v1 recording."""

    def __init__(
        self,
        path: str | Path,
        *,
        source_type: str,
        robot_model: str,
        router_zid: str,
        flush_interval_s: float = 1.0,
        schema_name: str = SCHEMA_NAME,
        schema_version: str = SCHEMA_VERSION,
        overwrite: bool = False,
        clock: Any = time.monotonic_ns,
    ) -> None:
        if source_type not in SOURCE_TYPES:
            raise SessionH5Error(f"unsupported source_type: {source_type}")
        if not robot_model or not router_zid:
            raise SessionH5Error("robot_model and router_zid are required")
        if flush_interval_s <= 0:
            raise SessionH5Error("flush_interval_s must be positive")
        if schema_name != SCHEMA_NAME or schema_version != SCHEMA_VERSION:
            raise SessionH5Error("unsupported session HDF5 schema")
        self.path = Path(path)
        if self.path.exists() and not overwrite:
            raise FileExistsError(f"refusing to overwrite existing recording: {self.path}")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = h5py.File(self.path, "w")
        self._closed = False
        self._complete = False
        self._clock = clock
        self._flush_interval_s = float(flush_interval_s)
        self._last_flush = time.monotonic()
        self._timeline_start: int | None = None
        self._schema_name = schema_name
        self._schema_version = schema_version
        self._initialize_layout(source_type, robot_model, router_zid)
    def _initialize_layout(self, source_type: str, robot_model: str, router_zid: str) -> None:
        self._file.attrs.update(
            schema_name=self._schema_name,
            schema_version=self._schema_version,
            source_type=source_type,
            robot_model=robot_model,
            router_zid=router_zid,
            complete=False,
        )
        raw = self._file.create_group("raw")
        live = raw.create_group("mocap_live")
        for name, shape, dtype in (("time_ns", (), np.int64), ("source_time_ns", (), np.int64), ("source_time_valid", (), np.bool_), ("publisher_instance_id", (), _STRING), ("stream_instance_id", (), _STRING), ("stream_sequence", (), np.int64), ("frame_index", (), np.int64), ("left_valid", (), np.bool_), ("right_valid", (), np.bool_), ("left_wrist_pose", (7,), np.float64), ("right_wrist_pose", (7,), np.float64), ("left_keypoints_world", (21, 3), np.float64), ("right_keypoints_world", (21, 3), np.float64)): _empty_dataset(live, name, shape, dtype=dtype)
        h5raw = raw.create_group("h5_replay")
        for name, shape, dtype in (("time_ns", (), np.int64), ("source_time_ns", (), np.int64), ("source_time_valid", (), np.bool_), ("publisher_instance_id", (), _STRING), ("sequence", (), np.int64)): _empty_dataset(h5raw, name, shape, dtype=dtype)
        hands = h5raw.create_group("hands")
        for side in SIDES:
            hand = hands.create_group(side); hand.attrs["side"] = side
            for name, shape, dtype in (("valid", (), np.bool_), ("wrist", (7,), np.float64), ("keypoints_world", (21, 3), np.float64)): _empty_dataset(hand, name, shape, dtype=dtype)
        target = self._file.create_group("target"); arm = target.create_group("arm"); hand_target = target.create_group("hand")
        for side in SIDES:
            ag = arm.create_group(side); ag.attrs.update(frame_id="Base_L" if side == "left" else "Base_R", side=side)
            for name, shape, dtype in (("time_ns", (), np.int64), ("source_time_ns", (), np.int64), ("source_time_valid", (), np.bool_), ("publisher_instance_id", (), _STRING), ("sequence", (), np.int64), ("pose", (7,), np.float64), ("elbow_reference_direction", (3,), np.float64)): _empty_dataset(ag, name, shape, dtype=dtype)
            hg = hand_target.create_group(side); hg.attrs.update(frame_id="wrist_relative_mediapipe", side=side)
            for name, shape, dtype in (("time_ns", (), np.int64), ("source_time_ns", (), np.int64), ("source_time_valid", (), np.bool_), ("publisher_instance_id", (), _STRING), ("sequence", (), np.int64), ("keypoints_m", (21, 3), np.float64)): _empty_dataset(hg, name, shape, dtype=dtype)
        joint = self._file.create_group("joint"); command = joint.create_group("command"); arm_cmd = command.create_group("arm"); hand_cmd = command.create_group("hand"); state = joint.create_group("state"); arm_state = state.create_group("arm"); hand_state = state.create_group("hand")
        for side in SIDES:
            ag = arm_cmd.create_group(side); ag.attrs["side"] = side
            for name, shape, dtype in (("time_ns", (), np.int64), ("publisher_instance_id", (), _STRING), ("sequence", (), np.int64), ("proposal_sequence", (), np.int64), ("proposal_sequence_valid", (), np.bool_), ("target_sequence", (), np.int64), ("target_sequence_valid", (), np.bool_), ("position_rad", (7,), np.float64), ("mode", (), _STRING)): _empty_dataset(ag, name, shape, dtype=dtype)
            hg = hand_cmd.create_group(side); hg.attrs["side"] = side
            for name, shape, dtype in (("time_ns", (), np.int64), ("publisher_instance_id", (), _STRING), ("sequence", (), np.int64), ("position_rad", (20,), np.float64)): _empty_dataset(hg, name, shape, dtype=dtype)
        for name, shape, dtype in (("time_ns", (), np.int64), ("publisher_instance_id", (), _STRING), ("sequence", (), np.int64), ("position_rad", (14,), np.float64), ("velocity_rad_s", (14,), np.float64), ("velocity_valid", (), np.bool_)): _empty_dataset(arm_state, name, shape, dtype=dtype)
        for side in SIDES:
            hg = hand_state.create_group(side); hg.attrs["side"] = side
            for name, shape, dtype in (("time_ns", (), np.int64), ("publisher_instance_id", (), _STRING), ("sequence", (), np.int64), ("position_rad", (20,), np.float64), ("velocity_rad_s", (20,), np.float64), ("velocity_valid", (), np.bool_)): _empty_dataset(hg, name, shape, dtype=dtype)
        events = self._file.create_group("meta").create_group("session_events")
        for name, shape, dtype in (("time_ns", (), np.int64), ("publisher_instance_id", (), _STRING), ("state", (), _STRING), ("reason", (), _STRING), ("source", (), _STRING), ("intent_sequence", (), np.int64), ("intent_sequence_valid", (), np.bool_)): _empty_dataset(events, name, shape, dtype=dtype)

    def _record_time(self, received_time_ns: int | None) -> int:
        value = _time_value(received_time_ns, self._clock)
        if self._timeline_start is None: self._timeline_start = value
        return value - self._timeline_start

    def flush(self) -> None:
        if not self._closed: self._file.flush(); self._last_flush = time.monotonic()

    def _maybe_flush(self) -> None:
        if time.monotonic() - self._last_flush >= self._flush_interval_s: self.flush()

    def _common(self, group: h5py.Group, receive: int, source_time: int | None, instance: str, sequence: int) -> None:
        source_value, valid = _source_time(source_time); _append(group["time_ns"], receive); _append(group["source_time_ns"], source_value); _append(group["source_time_valid"], valid); _append(group["publisher_instance_id"], instance)
        if "sequence" in group: _append(group["sequence"], sequence)

    def append_raw_mocap(self, sample: RawMocapLiveSample, received_time_ns: int | None = None) -> None:
        if not isinstance(sample, RawMocapLiveSample): raise TypeError("sample must be RawMocapLiveSample")
        group = self._file["raw/mocap_live"]; self._common(group, self._record_time(received_time_ns), sample.source_timestamp_ns, sample.envelope.publisher_instance_id, sample.envelope.sequence); _append(group["stream_instance_id"], sample.stream_instance_id); _append(group["stream_sequence"], sample.stream_sequence); _append(group["frame_index"], sample.frame_index)
        for side in SIDES:
            item = sample.hands[side]; valid = bool(item["valid"]); _append(group[f"{side}_valid"], valid); pose = np.full(7, np.nan); points = np.full((21, 3), np.nan)
            if valid: pose = _finite(item["wrist_pose"], f"{side}_wrist_pose"); points = _finite(item["keypoints_world_m"], f"{side}_keypoints_world")
            _append(group[f"{side}_wrist_pose"], pose); _append(group[f"{side}_keypoints_world"], points)
        self._maybe_flush()

    def append_raw_h5(self, sample: RawH5ReplaySample, received_time_ns: int | None = None) -> None:
        if not isinstance(sample, RawH5ReplaySample): raise TypeError("sample must be RawH5ReplaySample")
        group = self._file["raw/h5_replay"]; self._common(group, self._record_time(received_time_ns), sample.source_timestamp_ns, sample.envelope.publisher_instance_id, sample.envelope.sequence)
        for side in SIDES:
            item = sample.hands[side]; hand = group["hands"][side]; valid = bool(item["valid"]); _append(hand["valid"], valid); pose = np.full(7, np.nan); points = np.full((21, 3), np.nan)
            if valid: pose = _finite(item["wrist_pose"], f"{side}_wrist_pose"); points = _finite(item["keypoints_world_m"], f"{side}_keypoints_world")
            _append(hand["wrist"], pose); _append(hand["keypoints_world"], points); joints = item.get("wuji2_joints_rad")
            if joints is not None:
                if "wuji2_joints" not in hand:
                    dataset = _empty_dataset(hand, "wuji2_joints", (20,), dtype=np.float64)
                    for _ in range(hand["valid"].shape[0] - 1): _append(dataset, np.full(20, np.nan))
                _append(hand["wuji2_joints"], _finite(joints, f"{side}_wuji2_joints_rad"))
            elif "wuji2_joints" in hand: _append(hand["wuji2_joints"], np.full(20, np.nan))
        self._maybe_flush()

    def append_arm_target(self, target: ArmTargetCommand, received_time_ns: int | None = None) -> None:
        if not isinstance(target, ArmTargetCommand): raise TypeError("target must be ArmTargetCommand")
        group = self._file["target/arm"][target.side]; self._common(group, self._record_time(received_time_ns), target.source_timestamp_ns, target.envelope.publisher_instance_id, target.envelope.sequence); group.attrs.update(frame_id=target.frame_id, source=target.source); _append(group["pose"], target.position_m + target.orientation_xyzw); _append(group["elbow_reference_direction"], target.elbow_reference_direction); self._maybe_flush()

    def append_hand_target(self, target: HandTargetCommand, received_time_ns: int | None = None) -> None:
        if not isinstance(target, HandTargetCommand): raise TypeError("target must be HandTargetCommand")
        group = self._file["target/hand"][target.side]; self._common(group, self._record_time(received_time_ns), target.source_timestamp_ns, target.publisher_instance_id, target.sequence); group.attrs.update(frame_id=target.frame_id, source=target.source); _append(group["keypoints_m"], target.keypoints_m); self._maybe_flush()

    @staticmethod
    def _set_names(group: h5py.Group, names: Iterable[str], logical_id: str) -> None:
        value = _json_text(list(names)); group.attrs["joint_names"] = value; group.attrs["names"] = value; group.attrs["logical_id"] = logical_id

    def append_arm_command(self, command: ArmJointCommand, received_time_ns: int | None = None) -> None:
        group = self._file["joint/command/arm"][command.side]; receive = self._record_time(received_time_ns); _append(group["time_ns"], receive); _append(group["publisher_instance_id"], command.publisher_instance_id); _append(group["sequence"], command.sequence); _append(group["proposal_sequence"], -1 if command.proposal_sequence is None else command.proposal_sequence); _append(group["proposal_sequence_valid"], command.proposal_sequence is not None); _append(group["target_sequence"], -1 if command.target_sequence is None else command.target_sequence); _append(group["target_sequence_valid"], command.target_sequence is not None); _append(group["position_rad"], command.position_rad); _append(group["mode"], command.mode); self._set_names(group, command.names, command.producer); self._maybe_flush()

    def append_arm_state(self, state: ArmJointState, received_time_ns: int | None = None) -> None:
        group = self._file["joint/state/arm"]; receive = self._record_time(received_time_ns); _append(group["time_ns"], receive); _append(group["publisher_instance_id"], state.publisher_instance_id); _append(group["sequence"], state.sequence); _append(group["position_rad"], state.position_rad); valid = state.velocity_rad_s is not None; _append(group["velocity_valid"], valid); _append(group["velocity_rad_s"], state.velocity_rad_s if valid else np.full(14, np.nan)); self._set_names(group, state.names, state.executor); self._maybe_flush()

    def append_hand_command(self, command: HandJointCommand, received_time_ns: int | None = None) -> None:
        group = self._file["joint/command/hand"][command.side]; receive = self._record_time(received_time_ns); _append(group["time_ns"], receive); _append(group["publisher_instance_id"], command.publisher_instance_id); _append(group["sequence"], command.sequence); _append(group["position_rad"], command.position_rad); self._set_names(group, command.names, command.producer); self._maybe_flush()

    def append_hand_state(self, state: HandJointState, received_time_ns: int | None = None) -> None:
        group = self._file["joint/state/hand"][state.side]; receive = self._record_time(received_time_ns); _append(group["time_ns"], receive); _append(group["publisher_instance_id"], state.publisher_instance_id); _append(group["sequence"], state.sequence); _append(group["position_rad"], state.position_rad); valid = state.velocity_rad_s is not None; _append(group["velocity_valid"], valid); _append(group["velocity_rad_s"], state.velocity_rad_s if valid else np.full(20, np.nan)); self._set_names(group, state.names, state.executor); self._maybe_flush()

    def append_session_state(self, state: SessionState, received_time_ns: int | None = None) -> None:
        group = self._file["meta/session_events"]; receive = self._record_time(received_time_ns); _append(group["time_ns"], receive); _append(group["publisher_instance_id"], state.publisher_instance_id); _append(group["state"], state.state); _append(group["reason"], state.reason); _append(group["source"], state.source); _append(group["intent_sequence"], -1 if state.intent_sequence is None else state.intent_sequence); _append(group["intent_sequence_valid"], state.intent_sequence is not None); self._maybe_flush()

    append_raw_mocap_live = append_raw_mocap
    append_raw_h5_replay = append_raw_h5

    def append(self, value: Any, received_time_ns: int | None = None) -> None:
        dispatch = ((RawMocapLiveSample, self.append_raw_mocap), (RawH5ReplaySample, self.append_raw_h5), (ArmTargetCommand, self.append_arm_target), (HandTargetCommand, self.append_hand_target), (ArmJointCommand, self.append_arm_command), (ArmJointState, self.append_arm_state), (HandJointCommand, self.append_hand_command), (HandJointState, self.append_hand_state), (SessionState, self.append_session_state))
        for cls, method in dispatch:
            if isinstance(value, cls): method(value, received_time_ns=received_time_ns); return
        raise TypeError(f"unsupported session message: {type(value).__name__}")

    def close(self) -> None:
        if self._closed: return
        self._file.attrs["complete"] = True; self._complete = True; self.flush(); self._file.close(); self._closed = True

    def abort(self) -> None:
        if self._closed: return
        self._file.attrs["complete"] = False; self._complete = False; self.flush(); self._file.close(); self._closed = True

    def __enter__(self) -> "SessionH5Writer": return self
    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None: self.abort() if exc_type is not None else self.close()


class SessionH5Reader:
    """Validated read-only session-v1 loader."""
    def __init__(self, path: str | Path, *, allow_incomplete: bool = False) -> None:
        self.path = Path(path); self._file = h5py.File(self.path, "r"); self._closed = False
        try: self._validate(allow_incomplete)
        except Exception: self._file.close(); self._closed = True; raise

    @property
    def file(self) -> h5py.File:
        if self._closed: raise RuntimeError("session reader is closed")
        return self._file

    @property
    def attrs(self) -> dict[str, Any]: return {str(k): (_text(v) if isinstance(v, (bytes, np.bytes_)) else v) for k, v in self.file.attrs.items()}

    @staticmethod
    def _reject_links(group: h5py.Group, prefix: str = "") -> None:
        for name in group.keys():
            path = f"{prefix}/{name}" if prefix else name; link = group.get(name, getlink=True)
            if isinstance(link, (h5py.SoftLink, h5py.ExternalLink)): raise UnsafeSessionLinkError(f"linked objects are not allowed: {path}")
            item = group[name]
            if isinstance(item, h5py.Group): SessionH5Reader._reject_links(item, path)

    @staticmethod
    def _dtype_ok(dataset: h5py.Dataset, dtype: Any) -> bool:
        if dtype == _STRING: return dataset.dtype.kind == "O" and h5py.check_dtype(vlen=dataset.dtype) is str
        return dataset.dtype == np.dtype(dtype)

    @staticmethod
    def _validate_dataset(dataset: h5py.Dataset, shape: tuple[int, ...], dtype: Any, rows: int | None = None) -> int:
        if dataset.shape[1:] != shape or dataset.maxshape[0] is not None or dataset.maxshape[1:] != shape or dataset.chunks is None or dataset.chunks[0] <= 0 or dataset.chunks[1:] != shape or not SessionH5Reader._dtype_ok(dataset, dtype): raise SessionH5Error(f"invalid dataset layout: {dataset.name}")
        if rows is not None and dataset.shape[0] != rows: raise SessionH5Error(f"dataset row count mismatch: {dataset.name}")
        return int(dataset.shape[0])

    def _validate_group(self, path: str, specs: tuple[tuple[str, tuple[int, ...], Any], ...], *, children: tuple[str, ...] = ()) -> int:
        group = self.file[path]
        expected = {name for name, _, _ in specs} | set(children)
        if set(group.keys()) != expected: raise SessionH5Error(f"invalid dataset set: {path}")
        rows: int | None = None
        for name, shape, dtype in specs:
            count = self._validate_dataset(group[name], shape, dtype, rows)
            rows = count if rows is None else rows
        return rows or 0
    @staticmethod
    def _validate_joint_attrs(group: h5py.Group, expected: tuple[str, ...], side: str | None = None) -> None:
        if side is not None and _text(group.attrs.get("side", "")) != side: raise SessionH5Error(f"invalid side attr: {group.name}")
        names = group.attrs.get("joint_names"); logical = group.attrs.get("logical_id")
        if names is None or logical is None or not _text(logical): raise SessionH5Error(f"missing joint attrs: {group.name}")
        try: parsed = tuple(json.loads(_text(names)))
        except (TypeError, ValueError, json.JSONDecodeError): raise SessionH5Error(f"invalid joint_names attr: {group.name}")
        if parsed != expected: raise SessionH5Error(f"invalid canonical joint order: {group.name}")

    def _validate(self, allow_incomplete: bool) -> None:
        self._reject_links(self._file); attrs = self.attrs
        if set(attrs) != {"schema_name", "schema_version", "source_type", "robot_model", "router_zid", "complete"}: raise SessionH5Error("invalid root attrs")
        if attrs.get("schema_name") != SCHEMA_NAME or attrs.get("schema_version") != SCHEMA_VERSION: raise SessionH5Error("unsupported session HDF5 schema")
        if attrs.get("source_type") not in SOURCE_TYPES | {_LEGACY_CONTROLLER_GROUP}: raise SessionH5Error("invalid source_type")
        if not isinstance(attrs["complete"], (bool, np.bool_)): raise SessionH5Error("complete attr must be boolean")
        for key in ("robot_model", "router_zid"):
            if not isinstance(attrs[key], str) or not attrs[key]: raise SessionH5Error(f"invalid root attr: {key}")
        if not bool(attrs["complete"]) and not allow_incomplete: raise IncompleteSessionError(f"session is incomplete: {self.path}")
        if set(self.file.keys()) != {"raw", "target", "joint", "meta"}: raise SessionH5Error("invalid root group set")
        raw_groups = set(self.file["raw"].keys())
        if raw_groups not in ({"mocap_live", "h5_replay"}, {"mocap_live", "h5_replay", _LEGACY_CONTROLLER_GROUP}) or set(self.file["target"].keys()) != {"arm", "hand"} or set(self.file["joint"].keys()) != {"command", "state"} or set(self.file["meta"].keys()) != {"session_events"}: raise SessionH5Error("invalid fixed group set")
        scalar_i = (("time_ns", (), np.int64), ("source_time_ns", (), np.int64)); scalar_b = (("source_time_valid", (), np.bool_),); instance = (("publisher_instance_id", (), _STRING),)
        if _LEGACY_CONTROLLER_GROUP in raw_groups:
            self._validate_group(f"raw/{_LEGACY_CONTROLLER_GROUP}", scalar_i + scalar_b + instance + (("sequence", (), np.int64), ("left_pose", (7,), np.float64), ("right_pose", (7,), np.float64), ("right_a_pressed", (), np.bool_)))
        self._validate_group("raw/mocap_live", scalar_i + scalar_b + instance + (("stream_instance_id", (), _STRING), ("stream_sequence", (), np.int64), ("frame_index", (), np.int64), ("left_valid", (), np.bool_), ("right_valid", (), np.bool_), ("left_wrist_pose", (7,), np.float64), ("right_wrist_pose", (7,), np.float64), ("left_keypoints_world", (21, 3), np.float64), ("right_keypoints_world", (21, 3), np.float64)))
        parent_rows = self._validate_group("raw/h5_replay", scalar_i + scalar_b + instance + (("sequence", (), np.int64),), children=("hands",))
        if set(self.file["raw/h5_replay/hands"].keys()) != set(SIDES): raise SessionH5Error("invalid raw H5 hand group set")
        for side in SIDES:
            hand_path = f"raw/h5_replay/hands/{side}"
            hand_group = self.file[hand_path]
            if _text(hand_group.attrs.get("side", "")) != side: raise SessionH5Error(f"invalid hand side attr: {hand_path}")
            hand_rows = self._validate_group(hand_path, (("valid", (), np.bool_), ("wrist", (7,), np.float64), ("keypoints_world", (21, 3), np.float64)) + ((("wuji2_joints", (20,), np.float64),) if "wuji2_joints" in hand_group else ()))
            if hand_rows != parent_rows: raise SessionH5Error(f"raw H5 parent/hands row mismatch: {hand_path}")
        if set(self.file["target/arm"].keys()) != set(SIDES) or set(self.file["target/hand"].keys()) != set(SIDES): raise SessionH5Error("invalid target side group set")
        if set(self.file["joint/command"].keys()) != {"arm", "hand"} or set(self.file["joint/state"].keys()) != {"arm", "hand"}: raise SessionH5Error("invalid joint domain group set")
        for side in SIDES:
            ag = self.file[f"target/arm/{side}"]; hg = self.file[f"target/hand/{side}"]
            if _text(ag.attrs.get("side", "")) != side or _text(hg.attrs.get("side", "")) != side: raise SessionH5Error("invalid target side attr")
            if _text(ag.attrs.get("frame_id", "")) != ("Base_L" if side == "left" else "Base_R") or _text(hg.attrs.get("frame_id", "")) != "wrist_relative_mediapipe": raise SessionH5Error("invalid target frame_id")
            arm_rows = self._validate_group(f"target/arm/{side}", scalar_i + scalar_b + instance + (("sequence", (), np.int64), ("pose", (7,), np.float64), ("elbow_reference_direction", (3,), np.float64)))
            hand_rows = self._validate_group(f"target/hand/{side}", scalar_i + scalar_b + instance + (("sequence", (), np.int64), ("keypoints_m", (21, 3), np.float64)))
            if arm_rows and not _text(ag.attrs.get("source", "")): raise SessionH5Error("missing non-empty arm target source attr")
            if hand_rows and not _text(hg.attrs.get("source", "")): raise SessionH5Error("missing non-empty hand target source attr")
            for path, group, specs in ((f"joint/command/arm/{side}", self.file[f"joint/command/arm/{side}"], (("time_ns", (), np.int64), ("publisher_instance_id", (), _STRING), ("sequence", (), np.int64), ("proposal_sequence", (), np.int64), ("proposal_sequence_valid", (), np.bool_), ("target_sequence", (), np.int64), ("target_sequence_valid", (), np.bool_), ("position_rad", (7,), np.float64), ("mode", (), _STRING))), (f"joint/command/hand/{side}", self.file[f"joint/command/hand/{side}"], (("time_ns", (), np.int64), ("publisher_instance_id", (), _STRING), ("sequence", (), np.int64), ("position_rad", (20,), np.float64))), (f"joint/state/hand/{side}", self.file[f"joint/state/hand/{side}"], (("time_ns", (), np.int64), ("publisher_instance_id", (), _STRING), ("sequence", (), np.int64), ("position_rad", (20,), np.float64), ("velocity_rad_s", (20,), np.float64), ("velocity_valid", (), np.bool_)))):
                if _text(group.attrs.get("side", "")) != side: raise SessionH5Error(f"invalid side attr: {path}")
                rows = self._validate_group(path, specs)
                if rows:
                    expected_names = HAND_JOINT_NAMES[side] if "hand" in path else ARM_JOINT_NAMES[side]
                    self._validate_joint_attrs(group, expected_names, side)
        arm_state_group = self.file["joint/state/arm"]
        arm_state_rows = self._validate_group("joint/state/arm", (("time_ns", (), np.int64), ("publisher_instance_id", (), _STRING), ("sequence", (), np.int64), ("position_rad", (14,), np.float64), ("velocity_rad_s", (14,), np.float64), ("velocity_valid", (), np.bool_)))
        if arm_state_rows: self._validate_joint_attrs(arm_state_group, ALL_ARM_JOINT_NAMES)
        self._validate_group("meta/session_events", (("time_ns", (), np.int64), ("publisher_instance_id", (), _STRING), ("state", (), _STRING), ("reason", (), _STRING), ("source", (), _STRING), ("intent_sequence", (), np.int64), ("intent_sequence_valid", (), np.bool_)))

    @staticmethod
    def _row_group(group: h5py.Group) -> list[dict[str, Any]]:
        count = int(group["time_ns"].shape[0]); rows: list[dict[str, Any]] = []
        for index in range(count):
            row: dict[str, Any] = {}
            for name, dataset in group.items():
                if isinstance(dataset, h5py.Dataset):
                    value = dataset[index]; row[name] = _text(value) if isinstance(value, (bytes, np.bytes_)) else value.tolist() if isinstance(value, np.ndarray) else value.item() if isinstance(value, np.generic) else value
            rows.append(row)
        return rows

    @staticmethod
    def _source(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
        for row in rows:
            row["source_timestamp_ns"] = int(row["source_time_ns"]) if bool(row.pop("source_time_valid", False)) else None; row.pop("source_time_ns", None)
        return rows

    def read_legacy_controller(self) -> list[dict[str, Any]]:
        if _LEGACY_CONTROLLER_GROUP not in self.file["raw"]:
            return []
        rows = self._source(self._row_group(self.file[f"raw/{_LEGACY_CONTROLLER_GROUP}"]))
        [row.update(source_type=_LEGACY_CONTROLLER_GROUP) for row in rows]
        return rows
    def read_raw_mocap(self) -> list[dict[str, Any]]:
        rows = self._source(self._row_group(self.file["raw/mocap_live"])); group = self.file["raw/mocap_live"]
        for index, row in enumerate(rows):
            row["source_type"] = "mocap_live"; row["hands"] = {side: {"valid": bool(group[f"{side}_valid"][index]), "wrist_pose": group[f"{side}_wrist_pose"][index].tolist() if bool(group[f"{side}_valid"][index]) else None, "keypoints_world_m": group[f"{side}_keypoints_world"][index].tolist() if bool(group[f"{side}_valid"][index]) else None} for side in SIDES}
        return rows
    def read_raw_h5(self) -> list[dict[str, Any]]:
        rows = self._source(self._row_group(self.file["raw/h5_replay"])); groups = self.file["raw/h5_replay/hands"]
        for index, row in enumerate(rows):
            row["source_type"] = "h5_replay"; row["hands"] = {}
            for side in SIDES:
                hand = groups[side]; valid = bool(hand["valid"][index]); item = {"valid": valid, "wrist_pose": hand["wrist"][index].tolist() if valid else None, "keypoints_world_m": hand["keypoints_world"][index].tolist() if valid else None}; item["wuji2_joints_rad"] = None if "wuji2_joints" not in hand or not np.isfinite(hand["wuji2_joints"][index]).all() else hand["wuji2_joints"][index].tolist(); row["hands"][side] = item
        return rows

    def _target(self, side: str, hand: bool) -> list[dict[str, Any]]:
        if side not in SIDES: raise ValueError("side must be left or right")
        group = self.file[f"target/{'hand' if hand else 'arm'}/{side}"]; rows = self._source(self._row_group(group))
        for row in rows:
            row["frame_id"] = _text(group.attrs["frame_id"]); row["source"] = _text(group.attrs.get("source", ""))
            if not hand: row["position_m"], row["orientation_xyzw"] = row["pose"][:3], row["pose"][3:]; row.pop("pose", None)
        return rows
    def read_arm_target(self, side: str) -> list[dict[str, Any]]: return self._target(side, False)
    def read_hand_target(self, side: str) -> list[dict[str, Any]]: return self._target(side, True)

    def _joint(self, path: str, side: str | None = None) -> list[dict[str, Any]]:
        if side is not None and side not in SIDES: raise ValueError("side must be left or right")
        group = self.file[path if side is None else f"{path}/{side}"]; rows = self._row_group(group); names = group.attrs.get("joint_names", "[]"); logical = _text(group.attrs.get("logical_id", ""))
        try: names = json.loads(_text(names))
        except (TypeError, ValueError, json.JSONDecodeError): names = []
        for row in rows: row["names"] = list(names); row["producer"] = logical; row["executor"] = logical
        return rows
    def read_arm_command(self, side: str) -> list[dict[str, Any]]:
        rows = self._joint("joint/command/arm", side)
        for row in rows:
            valid = bool(row.pop(f"proposal_sequence_valid", False)); row["proposal_sequence"] = int(row["proposal_sequence"]) if valid else None; valid = bool(row.pop("target_sequence_valid", False)); row["target_sequence"] = int(row["target_sequence"]) if valid else None
        return rows
    def read_hand_command(self, side: str) -> list[dict[str, Any]]: return self._joint("joint/command/hand", side)
    def read_arm_state(self) -> list[dict[str, Any]]: return self._velocity(self._joint("joint/state/arm"))
    def read_hand_state(self, side: str) -> list[dict[str, Any]]: return self._velocity(self._joint("joint/state/hand", side))
    @staticmethod
    def _velocity(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
        for row in rows:
            if not bool(row.pop("velocity_valid", False)): row["velocity_rad_s"] = None
        return rows
    def read_session_events(self) -> list[dict[str, Any]]:
        rows = self._row_group(self.file["meta/session_events"])
        for row in rows: valid = bool(row.pop("intent_sequence_valid", False)); row["intent_sequence"] = int(row["intent_sequence"]) if valid else None
        return rows
    read_raw_mocap_live = read_raw_mocap
    read_raw_h5_replay = read_raw_h5
    def stream(self, kind: str, side: str | None = None) -> list[dict[str, Any]]:
        methods = {"raw_mocap": self.read_raw_mocap, "raw_h5": self.read_raw_h5, "legacy_controller": self.read_legacy_controller, "arm_state": self.read_arm_state, "session_state": self.read_session_events}; side_methods = {"arm_target": self.read_arm_target, "hand_target": self.read_hand_target, "arm_command": self.read_arm_command, "hand_command": self.read_hand_command, "hand_state": self.read_hand_state}
        if kind in methods: return methods[kind]()
        if side is None or kind not in side_methods: raise ValueError(f"unknown stream or missing side: {kind}")
        return side_methods[kind](side)
    def close(self) -> None:
        if not self._closed: self._file.close(); self._closed = True
    def __enter__(self) -> "SessionH5Reader": return self
    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None: self.close()


SessionH5Loader = SessionH5Reader

def load_session_h5(path: str | Path, *, allow_incomplete: bool = False) -> SessionH5Reader: return SessionH5Reader(path, allow_incomplete=allow_incomplete)

__all__ = ["SCHEMA_NAME", "SCHEMA_VERSION", "SOURCE_TYPES", "SessionH5Error", "IncompleteSessionError", "UnsafeSessionLinkError", "SessionH5Writer", "SessionH5Reader", "SessionH5Loader", "load_session_h5"]
