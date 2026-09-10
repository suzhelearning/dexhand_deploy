"""Append-only Tianji session-v1 HDF5 storage."""
from __future__ import annotations

from collections.abc import Iterable, Mapping
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
from ..hand_tracking.models import (
    ArmInputObservation as ArmInputObservationModel,
    HandObservation,
    LegacyPicoPalmFrame,
    ManusRawFrame,
    PICO_JOINT_NAMES,
    PicoRawFrame,
    SIDES as HAND_TRACKING_SIDES,
)
from ..hand_tracking.xr_input import XrFrame

SCHEMA_NAME = "tianji-teleop-session"
SCHEMA_VERSION = "1.0"
EXTENDED_SCHEMA_VERSION = "1.1"
DUAL_SCHEMA_VERSION = "1.2"
_EXTENDED_SCHEMA_VERSIONS = (EXTENDED_SCHEMA_VERSION, DUAL_SCHEMA_VERSION)
DUAL_SOURCE_TYPES = frozenset({
    'pico2_hands_sim', 'vr_manus_sim', 'vr_manus_xr_sim',
    'pico2_hands_real', 'vr_manus_real',
})
SOURCE_TYPES = frozenset({"mocap_live", "h5_replay", "target_replay", "joint_replay"})
EXTENDED_SOURCE_TYPES = frozenset({
    "hand_tracking_observation",
    "hand_tracking_sim",
    "hand_tracking_sim_manus",
})
_LEGACY_CONTROLLER_GROUP = "pico_controller"
SIDES = ("left", "right")
_STRING = h5py.string_dtype(encoding="utf-8")
_UINT8_VECTOR = h5py.vlen_dtype(np.dtype("uint8"))
_TJVR_RAW_SPECS = (("time_ns", (), np.int64), ("received_timestamp_ns", (), np.int64),
    ("receiver_instance_id", (), _STRING), ("receiver_frame_sequence", (), np.int64),
    ("raw_packet", (), _UINT8_VECTOR))
_MANUS_CALLBACK_SPECS = (("time_ns", (), np.int64), ("received_timestamp_ns", (), np.int64),
    ("receiver_instance_id", (), _STRING), ("callback_sequence", (), np.int64),
    ("point_count", (), np.int64), ("single_hand_side", (), _STRING), ("points", (126,), np.float64),
    ("source_sequences_json", (), _STRING), ("source_timestamps_ns_json", (), _STRING))
_MANUS_CALLBACK_LEGACY_SPECS = (("time_ns", (), np.int64), ("received_timestamp_ns", (), np.int64),
    ("receiver_instance_id", (), _STRING), ("callback_sequence", (), np.int64),
    ("point_count", (), np.int64), ("single_hand_side", (), _STRING), ("points", (126,), np.float64))
_XR_RAW_SPECS = (
    ("time_ns", (), np.int64),
    ("source_time_ns", (), np.int64),
    ("source_time_valid", (), np.bool_),
    ("received_timestamp_ns", (), np.int64),
    ("receiver_instance_id", (), _STRING),
    ("connection_generation", (), np.int64),
    ("receiver_frame_sequence", (), np.int64),
    ("hmd_valid", (), np.bool_),
    ("hmd_pose", (7,), np.float64),
    ("left_controller_valid", (), np.bool_),
    ("left_controller_available", (), np.bool_),
    ("left_controller_pose", (7,), np.float64),
    ("left_trigger", (), np.float64),
    ("left_grip", (), np.float64),
    ("left_axis", (2,), np.float64),
    ("right_controller_valid", (), np.bool_),
    ("right_controller_available", (), np.bool_),
    ("right_controller_pose", (7,), np.float64),
    ("right_trigger", (), np.float64),
    ("right_grip", (), np.float64),
    ("right_axis", (2,), np.float64),
    ("tracker_count", (), np.int64),
    ("trackers_json", (), _STRING),
    ("frame_json", (), _STRING),
)
_XR_RAW_LEGACY_SPECS = tuple(
    item for item in _XR_RAW_SPECS if item[0] != "connection_generation"
)
_DUAL_AUDIT_SPECS = (("time_ns", (), np.int64), ("received_timestamp_ns", (), np.int64),
    ("kind", (), _STRING), ("payload_json", (), _STRING))
_DUAL_AUDIT_KINDS = frozenset({'lifecycle', 'native_cycle', 'operator_result', 'operator_observation',
    'manus_rawviz_line', 'manus_callback_metadata', 'hand_output', 'component_status', 'hand_retarget_input',
    'tjvr_stream_decision'})


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
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"), allow_nan=False)


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

    # Existing writers keep exactly the same immediate row append operation.
    # A new-only disk owner may override this seam for bounded row batching.
    _append = staticmethod(_append)

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
        metadata: Mapping[str, Any] | None = None,
    ) -> None:
        if source_type not in SOURCE_TYPES | EXTENDED_SOURCE_TYPES | DUAL_SOURCE_TYPES:
            raise SessionH5Error(f"unsupported source_type: {source_type}")
        if not robot_model or not router_zid:
            raise SessionH5Error("robot_model and router_zid are required")
        if flush_interval_s <= 0:
            raise SessionH5Error("flush_interval_s must be positive")
        if schema_name != SCHEMA_NAME or schema_version not in (SCHEMA_VERSION, *_EXTENDED_SCHEMA_VERSIONS):
            raise SessionH5Error("unsupported session HDF5 schema")
        if schema_version == SCHEMA_VERSION and source_type not in SOURCE_TYPES:
            raise SessionH5Error("schema 1.0 only supports legacy session source types")
        if schema_version == EXTENDED_SCHEMA_VERSION and source_type not in EXTENDED_SOURCE_TYPES:
            raise SessionH5Error("schema 1.1 requires an extended hand-tracking source type")
        if schema_version == DUAL_SCHEMA_VERSION and source_type not in DUAL_SOURCE_TYPES:
            raise SessionH5Error('schema 1.2 requires an explicit dual-input source type')
        if metadata is not None and not isinstance(metadata, Mapping):
            raise SessionH5Error("session metadata must be a mapping")
        metadata_value = dict(metadata or {})
        try:
            _json_text(metadata_value)
        except (TypeError, ValueError) as exc:
            raise SessionH5Error("session metadata must be JSON serializable") from exc
        self.path = Path(path)
        if self.path.exists() and not overwrite:
            raise FileExistsError(f"refusing to overwrite existing recording: {self.path}")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = h5py.File(self.path, "w" if overwrite else "x")
        self._closed = False
        self._complete = False
        self._clock = clock
        self._flush_interval_s = float(flush_interval_s)
        self._last_flush = time.monotonic()
        self._timeline_start: int | None = None
        self._schema_name = schema_name
        self._schema_version = schema_version
        self._metadata = metadata_value
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
        if self._schema_version in _EXTENDED_SCHEMA_VERSIONS:
            self._initialize_hand_tracking_layout()
            metadata_group = self._file["meta"].create_group("hand_tracking")
            metadata_group.attrs["metadata_json"] = _json_text(self._metadata)
        if self._schema_version == DUAL_SCHEMA_VERSION:
            group = self._file['raw'].create_group('tjvr_upper_limb')
            group.attrs['input_stage'] = 'decoded_before_stream_gate'
            for name, shape, dtype in _TJVR_RAW_SPECS:
                _empty_dataset(group, name, shape, dtype=dtype)
            callbacks = self._file['raw'].create_group('manus_callbacks')
            callbacks.attrs['input_stage'] = 'mediapipe21_callback_not_raw25'
            callbacks.attrs['order'] = 'right21_left21'
            for name, shape, dtype in _MANUS_CALLBACK_SPECS:
                _empty_dataset(callbacks, name, shape, dtype=dtype)
            audit = self._file['meta'].create_group('dual_audit')
            for name, shape, dtype in _DUAL_AUDIT_SPECS:
                _empty_dataset(audit, name, shape, dtype=dtype)

    def _initialize_hand_tracking_layout(self) -> None:
        """Add the receive-only hand-tracking streams used by schema 1.1.

        The legacy groups remain present in a 1.1 file so existing session
        tooling can still inspect the common target/joint streams.  The new
        groups deliberately keep source frames, converted observations, and
        arm-input poses separate; no target or command dataset is populated by
        this layer.
        """
        for side in SIDES:
            _empty_dataset(self._file[f"target/arm/{side}"], "tracking_valid", (), dtype=np.bool_)
        raw = self._file["raw"]

        pico = raw.create_group("pico_hand_tracking")
        for name, shape, dtype in (
            ("time_ns", (), np.int64),
            ("source_time_ns", (), np.int64),
            ("source_time_valid", (), np.bool_),
            ("source_timestamp_ms", (), np.int64),
            ("protocol_version", (), np.uint8),
            ("flags", (), np.uint8),
            ("joint_count", (), np.uint8),
            ("head_valid", (), np.bool_),
            ("head_pose", (7,), np.float64),
            ("receiver_instance_id", (), _STRING),
            ("connection_generation", (), np.int64),
            ("receiver_frame_sequence", (), np.int64),
            ("association_id", (), _STRING),
            ("raw_packet", (), _UINT8_VECTOR),
        ):
            _empty_dataset(pico, name, shape, dtype=dtype)
        if self._schema_version == DUAL_SCHEMA_VERSION:
            # Source receive clock, not recorder receipt/timeline. Optional
            # on read for earlier 1.2 captures; do not change the 1.1 layout.
            _empty_dataset(pico, 'received_timestamp_ns', (), dtype=np.int64)
        pico.attrs["source"] = "pico2"
        pico_hands = pico.create_group("hands")
        for side in SIDES:
            hand = pico_hands.create_group(side)
            hand.attrs.update(side=side, joint_names=_json_text(list(PICO_JOINT_NAMES)))
            for name, shape, dtype in (
                ("valid", (), np.bool_),
                ("wrist_valid", (), np.bool_),
                ("wrist_pose", (7,), np.float64),
                ("joint_valid", (26,), np.bool_),
                ("joint_poses", (26, 7), np.float64),
                ("joint_radii_m", (26,), np.float64),
            ):
                _empty_dataset(hand, name, shape, dtype=dtype)

        manus = raw.create_group("manus_hand_tracking")
        for name, shape, dtype in (
            ("time_ns", (), np.int64),
            ("source_time_ns", (), np.int64),
            ("source_time_valid", (), np.bool_),
            ("source_sequence", (), np.int64),
            ("sdk_publish_time", (), np.int64),
            ("glove_id", (), _STRING),
            ("side", (), _STRING),
            ("receiver_instance_id", (), _STRING),
            ("receiver_frame_sequence", (), np.int64),
            ("association_id", (), _STRING),
            ("node_count", (), np.int64),
            ("node_valid", (64,), np.bool_),
            ("node_positions", (64, 3), np.float64),
            ("node_quaternions_wxyz", (64, 4), np.float64),
            ("node_semantics_json", (), _STRING),
            ("payload_json", (), _STRING),
        ):
            _empty_dataset(manus, name, shape, dtype=dtype)
        manus.attrs["source"] = "manus"

        legacy = raw.create_group("legacy_pico_palm")
        for name, shape, dtype in (
            ("time_ns", (), np.int64),
            ("source_time_ns", (), np.int64),
            ("source_time_valid", (), np.bool_),
            ("protocol_version", (), np.uint8),
            ("packet_size", (), np.int64),
            ("flags", (), np.int64),
            ("source_sequence", (), np.int64),
            ("tracking_epoch", (), np.int64),
            ("bridge_send_monotonic_ns", (), np.int64),
            ("receiver_instance_id", (), _STRING),
            ("receiver_frame_sequence", (), np.int64),
            ("association_id", (), _STRING),
            ("raw_packet", (), _UINT8_VECTOR),
            ("left_valid", (), np.bool_),
            ("right_valid", (), np.bool_),
            ("left_pose", (7,), np.float64),
            ("right_pose", (7,), np.float64),
            ("upper_limb_skeleton_valid", (), np.bool_),
            ("upper_limb_rotations_valid", (), np.bool_),
            ("upper_limb_points", (8, 3), np.float64),
            ("upper_limb_rotations_xyzw", (8, 4), np.float64),
        ):
            _empty_dataset(legacy, name, shape, dtype=dtype)
        legacy.attrs["source"] = "pico_manus_teleop_experiments"

        if self._schema_version == DUAL_SCHEMA_VERSION:
            xr = raw.create_group("xr_input")
            xr.attrs["source"] = "xr_observation_before_mapping"
            for name, shape, dtype in _XR_RAW_SPECS:
                _empty_dataset(xr, name, shape, dtype=dtype)

        observation = self._file.create_group("observation")
        hand_observation = observation.create_group("hand_tracking")
        arm_observation = observation.create_group("arm_input")
        hand_specs = (
            ("time_ns", (), np.int64),
            ("source_time_ns", (), np.int64),
            ("source_time_valid", (), np.bool_),
            ("received_timestamp_ns", (), np.int64),
            ("source", (), _STRING),
            ("source_instance_id", (), _STRING),
            ("source_sequence", (), np.int64),
            ("source_sequence_valid", (), np.bool_),
            ("receiver_instance_id", (), _STRING),
            ("receiver_frame_sequence", (), np.int64),
            ("coordinate_frame", (), _STRING),
            ("mapping_version", (), _STRING),
            ("valid", (), np.bool_),
            ("joint_valid", (21,), np.bool_),
            ("keypoints_m", (21, 3), np.float64),
            ("wrist_pose", (7,), np.float64),
            ("wrist_pose_valid", (), np.bool_),
            ("frame_association_id", (), _STRING),
        )
        arm_specs = (
            ("time_ns", (), np.int64),
            ("source_time_ns", (), np.int64),
            ("source_time_valid", (), np.bool_),
            ("received_timestamp_ns", (), np.int64),
            ("source", (), _STRING),
            ("tracked_frame", (), _STRING),
            ("reference_frame", (), _STRING),
            ("source_instance_id", (), _STRING),
            ("source_sequence", (), np.int64),
            ("source_sequence_valid", (), np.bool_),
            ("receiver_instance_id", (), _STRING),
            ("receiver_frame_sequence", (), np.int64),
            ("mapping_version", (), _STRING),
            ("pose", (7,), np.float64),
            ("pose_valid", (), np.bool_),
            ("elbow_pose", (7,), np.float64),
            ("elbow_pose_valid", (), np.bool_),
            ("valid", (), np.bool_),
            ("frame_association_id", (), _STRING),
        )
        for side in SIDES:
            hand_group = hand_observation.create_group(side)
            arm_group = arm_observation.create_group(side)
            hand_group.attrs["side"] = side
            arm_group.attrs["side"] = side
            for name, shape, dtype in hand_specs:
                _empty_dataset(hand_group, name, shape, dtype=dtype)
            for name, shape, dtype in arm_specs:
                _empty_dataset(arm_group, name, shape, dtype=dtype)

    def _record_time(self, received_time_ns: int | None) -> int:
        value = _time_value(received_time_ns, self._clock)
        if self._timeline_start is None: self._timeline_start = value
        return value - self._timeline_start

    def flush(self) -> None:
        if not self._closed: self._file.flush(); self._last_flush = time.monotonic()

    def _maybe_flush(self) -> None:
        if time.monotonic() - self._last_flush >= self._flush_interval_s: self.flush()

    def _common(self, group: h5py.Group, receive: int, source_time: int | None, instance: str, sequence: int) -> None:
        source_value, valid = _source_time(source_time); self._append(group["time_ns"], receive); self._append(group["source_time_ns"], source_value); self._append(group["source_time_valid"], valid); self._append(group["publisher_instance_id"], instance)
        if "sequence" in group: self._append(group["sequence"], sequence)

    def append_raw_mocap(self, sample: RawMocapLiveSample, received_time_ns: int | None = None) -> None:
        if not isinstance(sample, RawMocapLiveSample): raise TypeError("sample must be RawMocapLiveSample")
        group = self._file["raw/mocap_live"]; self._common(group, self._record_time(received_time_ns), sample.source_timestamp_ns, sample.envelope.publisher_instance_id, sample.envelope.sequence); self._append(group["stream_instance_id"], sample.stream_instance_id); self._append(group["stream_sequence"], sample.stream_sequence); self._append(group["frame_index"], sample.frame_index)
        for side in SIDES:
            item = sample.hands[side]; valid = bool(item["valid"]); self._append(group[f"{side}_valid"], valid); pose = np.full(7, np.nan); points = np.full((21, 3), np.nan)
            if valid: pose = _finite(item["wrist_pose"], f"{side}_wrist_pose"); points = _finite(item["keypoints_world_m"], f"{side}_keypoints_world")
            self._append(group[f"{side}_wrist_pose"], pose); self._append(group[f"{side}_keypoints_world"], points)
        self._maybe_flush()

    def append_raw_h5(self, sample: RawH5ReplaySample, received_time_ns: int | None = None) -> None:
        if not isinstance(sample, RawH5ReplaySample): raise TypeError("sample must be RawH5ReplaySample")
        group = self._file["raw/h5_replay"]; self._common(group, self._record_time(received_time_ns), sample.source_timestamp_ns, sample.envelope.publisher_instance_id, sample.envelope.sequence)
        for side in SIDES:
            item = sample.hands[side]; hand = group["hands"][side]; valid = bool(item["valid"]); self._append(hand["valid"], valid); pose = np.full(7, np.nan); points = np.full((21, 3), np.nan)
            if valid: pose = _finite(item["wrist_pose"], f"{side}_wrist_pose"); points = _finite(item["keypoints_world_m"], f"{side}_keypoints_world")
            self._append(hand["wrist"], pose); self._append(hand["keypoints_world"], points); joints = item.get("wuji2_joints_rad")
            if joints is not None:
                if "wuji2_joints" not in hand:
                    dataset = _empty_dataset(hand, "wuji2_joints", (20,), dtype=np.float64)
                    for _ in range(hand["valid"].shape[0] - 1): self._append(dataset, np.full(20, np.nan))
                self._append(hand["wuji2_joints"], _finite(joints, f"{side}_wuji2_joints_rad"))
            elif "wuji2_joints" in hand: self._append(hand["wuji2_joints"], np.full(20, np.nan))
        self._maybe_flush()

    def _require_extended(self) -> None:
        if self._schema_version not in _EXTENDED_SCHEMA_VERSIONS:
            raise SessionH5Error("hand-tracking records require session HDF5 schema 1.1 or 1.2")

    def append_dual_audit(self, kind, payload, *, received_timestamp_ns) -> None:
        """Passive diagnostics only: these rows never grant replay authority."""
        self.append_dual_audit_batch([(kind, payload, received_timestamp_ns)])

    def append_dual_audit_batch(self, rows) -> None:
        """Bounded consecutive rows, validated together and written without resampling.

        Validation is atomic, disk writes are not: an I/O failure still requires
        aborting the recording. No buffering or scheduling changes in old paths.
        """
        if not isinstance(rows, (list, tuple)) or not 0 < len(rows) <= 256:
            raise SessionH5Error('dual audit batch must contain 1..256 rows')
        encoded_rows = []
        for row in rows:
            if not isinstance(row, (list, tuple)) or len(row) != 3:
                raise SessionH5Error('dual audit row requires kind, payload and timestamp')
            kind, payload, stamp = row
            encoded_rows.append((kind, self._encode_dual_audit(kind, payload, stamp), stamp))
        # Do not establish the timeline until the complete batch validates.
        values = dict(time_ns=[self._record_time(row[2]) for row in encoded_rows],
                      received_timestamp_ns=[row[2] for row in encoded_rows],
                      kind=[row[0] for row in encoded_rows], payload_json=[row[1] for row in encoded_rows])
        group = self._file['meta/dual_audit']
        for name, column in values.items():
            dataset = group[name]
            offset = dataset.shape[0]
            dataset.resize(offset + len(column), axis=0)
            dataset[offset:] = column
        self._maybe_flush()

    def _encode_dual_audit(self, kind, payload, received_timestamp_ns):
        if (self._schema_version != DUAL_SCHEMA_VERSION or not isinstance(kind, str) or kind not in _DUAL_AUDIT_KINDS or
                type(received_timestamp_ns) is not int or not 0 < received_timestamp_ns < 2**63 or
                not isinstance(payload, dict)):
            raise SessionH5Error('invalid dual audit kind, payload, timestamp or schema')
        try:
            encoded = _json_text(payload)
        except (TypeError, ValueError) as exc:
            raise SessionH5Error('dual audit payload must be finite JSON') from exc
        if len(encoded.encode('utf-8')) > 1_048_576:
            raise SessionH5Error('dual audit payload exceeds 1 MiB')
        return encoded

    def append_raw_reference_tjvr(self, observation) -> None:
        """Complete original datagram, including rejected/duplicate gate input.

        Independent validity bits and redundant directions remain in raw bytes;
        never serialize normalized derived geometry as a substitute for raw.
        """
        from ..hand_tracking.reference_tjvr import ReferenceTjvrFrame, parse_reference_tjvr_packet
        if self._schema_version != DUAL_SCHEMA_VERSION:
            raise SessionH5Error('reference TJVR requires session HDF5 schema 1.2')
        if not isinstance(observation, ReferenceTjvrFrame):
            raise SessionH5Error('expected reference TJVR observation before gate')
        frame = observation.frame
        # Revalidate raw instead of trusting mutable redundant arrays.
        parse_reference_tjvr_packet(frame.raw_packet, receiver_instance_id=frame.receiver_instance_id,
            receiver_frame_sequence=frame.receiver_frame_sequence,
            received_timestamp_ns=frame.received_timestamp_ns)
        values = dict(time_ns=self._record_time(frame.received_timestamp_ns),
            received_timestamp_ns=frame.received_timestamp_ns, receiver_instance_id=frame.receiver_instance_id,
            receiver_frame_sequence=frame.receiver_frame_sequence,
            raw_packet=np.frombuffer(frame.raw_packet, dtype=np.uint8))
        group = self._file['raw/tjvr_upper_limb']
        for name, value in values.items():
            self._append(group[name], value)
        self._maybe_flush()

    def append_raw_xr(self, frame: XrFrame, received_time_ns: int | None = None) -> None:
        """Append a complete XR snapshot before controller/tracker mapping."""
        if self._schema_version != DUAL_SCHEMA_VERSION:
            raise SessionH5Error('raw XR input requires session HDF5 schema 1.2')
        if not isinstance(frame, XrFrame):
            raise TypeError('frame must be XrFrame')
        group = self._file['raw/xr_input']
        receive = self._record_time(
            frame.received_timestamp_ns if received_time_ns is None else received_time_ns
        )
        source_time, source_valid = _source_time(frame.source_timestamp_ns)
        frame_dict = frame.to_dict()

        def pose(value: Any, field: str) -> np.ndarray:
            return _finite(value, field) if value is not None else np.full(7, np.nan)

        values: dict[str, Any] = {
            'time_ns': receive,
            'source_time_ns': source_time,
            'source_time_valid': source_valid,
            'received_timestamp_ns': frame.received_timestamp_ns,
            'receiver_instance_id': frame.receiver_instance_id,
            'connection_generation': frame.connection_generation,
            'receiver_frame_sequence': frame.sequence,
            'hmd_valid': frame.hmd_pose is not None,
            'hmd_pose': pose(frame.hmd_pose, 'hmd_pose'),
            'tracker_count': len(frame.trackers),
            'trackers_json': _json_text(frame_dict['trackers']),
            'frame_json': _json_text(frame_dict),
        }
        for side in SIDES:
            controller = frame.controller(side)
            values.update({
                f'{side}_controller_valid': bool(controller.valid),
                f'{side}_controller_available': bool(controller.available),
                f'{side}_controller_pose': pose(controller.pose, f'{side}_controller_pose'),
                f'{side}_trigger': controller.trigger,
                f'{side}_grip': controller.grip,
                f'{side}_axis': _finite(controller.axis, f'{side}_controller_axis'),
            })
        for name, value in values.items():
            self._append(group[name], value)
        self._maybe_flush()

    def append_raw_xr_input(self, frame: XrFrame, received_time_ns: int | None = None) -> None:
        """Compatibility alias for callers that use the topic name."""
        self.append_raw_xr(frame, received_time_ns=received_time_ns)

    def append_manus_callback(self, points, *, callback_sequence, received_timestamp_ns,
                              receiver_instance_id, single_hand_side='right',
                              source_sequences=None, source_timestamps_ns=None) -> None:
        if self._schema_version != DUAL_SCHEMA_VERSION:
            raise SessionH5Error('Manus callbacks require session HDF5 schema 1.2')
        for value in (callback_sequence, received_timestamp_ns):
            if type(value) is not int or not 0 < value < 2**63:
                raise SessionH5Error('callback sequence and receive time must be positive int64')
        if not isinstance(receiver_instance_id, str) or not receiver_instance_id.strip():
            raise SessionH5Error('callback receiver identity required')
        if single_hand_side not in SIDES:
            raise SessionH5Error('single callback side must be left/right')
        if (not isinstance(points, (list, tuple)) or len(points) not in (63, 126) or
                any(type(value) not in (float, int) for value in points)):
            raise SessionH5Error('callback requires 63/126 finite numbers')
        values = _finite(points, 'callback points')
        stored = np.full(126, np.nan)
        offset = 63 if len(points) == 63 and single_hand_side == 'left' else 0
        stored[offset:offset + len(points)] = values
        def metadata_json(value, field):
            if value is None:
                value = {}
            if not isinstance(value, Mapping):
                raise SessionH5Error(f'{field} must be a mapping')
            normalized = {}
            for side, sequence in value.items():
                if side not in SIDES or type(sequence) is not int or not 0 <= sequence < 2**63:
                    raise SessionH5Error(f'{field} contains an invalid side/value')
                normalized[side] = sequence
            return _json_text(normalized)

        row = dict(time_ns=self._record_time(received_timestamp_ns),
            received_timestamp_ns=received_timestamp_ns, receiver_instance_id=receiver_instance_id,
            callback_sequence=callback_sequence, point_count=len(points),
            single_hand_side=single_hand_side, points=stored,
            source_sequences_json=metadata_json(source_sequences, 'source_sequences'),
            source_timestamps_ns_json=metadata_json(source_timestamps_ns, 'source_timestamps_ns'))
        for name, value in row.items():
            self._append(self._file['raw/manus_callbacks'][name], value)
        self._maybe_flush()

    def append_raw_pico(self, frame: PicoRawFrame, received_time_ns: int | None = None) -> None:
        """Append the complete decoded PICO frame, including its wire bytes."""
        self._require_extended()
        if not isinstance(frame, PicoRawFrame):
            raise TypeError("frame must be PicoRawFrame")
        group = self._file["raw/pico_hand_tracking"]
        receive = self._record_time(
            frame.received_timestamp_ns if received_time_ns is None else received_time_ns
        )
        source_time, source_valid = _source_time(frame.source_timestamp_ns)
        self._append(group["time_ns"], receive)
        if 'received_timestamp_ns' in group:
            self._append(group['received_timestamp_ns'], frame.received_timestamp_ns)
        self._append(group["source_time_ns"], source_time)
        self._append(group["source_time_valid"], source_valid)
        self._append(group["source_timestamp_ms"], frame.source_timestamp_ms)
        self._append(group["protocol_version"], frame.protocol_version)
        self._append(group["flags"], frame.flags)
        self._append(group["joint_count"], frame.joint_count)
        self._append(group["head_valid"], bool(frame.head_valid))
        self._append(group["head_pose"], _finite(frame.head_pose, "head_pose"))
        self._append(group["receiver_instance_id"], frame.receiver_instance_id)
        self._append(group["connection_generation"], frame.connection_generation)
        self._append(group["receiver_frame_sequence"], frame.receiver_frame_sequence)
        self._append(group["association_id"], frame.association_id)
        self._append(group["raw_packet"], np.frombuffer(frame.raw_packet, dtype=np.uint8))
        for side in SIDES:
            source_hand = frame.hands[side]
            hand = group["hands"][side]
            self._append(hand["valid"], bool(source_hand.valid))
            self._append(hand["wrist_valid"], bool(source_hand.wrist_valid))
            self._append(hand["wrist_pose"], _finite(source_hand.wrist_pose, f"{side}_wrist_pose"))
            self._append(hand["joint_valid"], [bool(joint.valid) for joint in source_hand.joints])
            self._append(
                hand["joint_poses"],
                np.asarray([_finite(joint.pose, f"{side}_joint_pose") for joint in source_hand.joints]),
            )
            self._append(
                hand["joint_radii_m"],
                np.asarray([joint.radius_m for joint in source_hand.joints], dtype=np.float64),
            )
        self._maybe_flush()

    def append_raw_manus(self, frame: ManusRawFrame, received_time_ns: int | None = None) -> None:
        """Append the complete normalized Manus source frame."""
        self._require_extended()
        if not isinstance(frame, ManusRawFrame):
            raise TypeError("frame must be ManusRawFrame")
        group = self._file["raw/manus_hand_tracking"]
        receive = self._record_time(
            frame.received_timestamp_ns if received_time_ns is None else received_time_ns
        )
        source_time, source_valid = _source_time(frame.source_monotonic_ns)
        count = frame.node_positions.shape[0]
        positions = np.full((64, 3), np.nan, dtype=np.float64)
        quaternions = np.full((64, 4), np.nan, dtype=np.float64)
        valid = np.zeros(64, dtype=np.bool_)
        positions[:count] = _finite(frame.node_positions, "node_positions")
        quaternions[:count] = _finite(frame.node_quaternions_wxyz, "node_quaternions_wxyz")
        valid[:count] = True
        for name, value in (
            ("time_ns", receive),
            ("source_time_ns", source_time),
            ("source_time_valid", source_valid),
            ("source_sequence", frame.source_sequence),
            ("sdk_publish_time", frame.sdk_publish_time),
            ("glove_id", frame.glove_id),
            ("side", frame.side),
            ("receiver_instance_id", frame.receiver_instance_id),
            ("receiver_frame_sequence", frame.receiver_frame_sequence),
            ("association_id", frame.association_id),
            ("node_count", count),
            ("node_valid", valid),
            ("node_positions", positions),
            ("node_quaternions_wxyz", quaternions),
            ("node_semantics_json", _json_text([dict(item) for item in frame.node_semantics])),
            ("payload_json", _json_text(frame.to_dict())),
        ):
            self._append(group[name], value)
        self._maybe_flush()

    def append_raw_legacy_palm(self, frame: LegacyPicoPalmFrame, received_time_ns: int | None = None) -> None:
        """Append one historical TJVR arm/palm datagram without control use."""
        self._require_extended()
        if not isinstance(frame, LegacyPicoPalmFrame):
            raise TypeError("frame must be LegacyPicoPalmFrame")
        group = self._file["raw/legacy_pico_palm"]
        receive = self._record_time(
            frame.received_timestamp_ns if received_time_ns is None else received_time_ns
        )
        source_time, source_valid = _source_time(frame.source_timestamp_ns)
        values = (
            ("time_ns", receive),
            ("source_time_ns", source_time),
            ("source_time_valid", source_valid),
            ("protocol_version", frame.protocol_version),
            ("packet_size", frame.packet_size),
            ("flags", frame.flags),
            ("source_sequence", frame.sequence),
            ("tracking_epoch", frame.tracking_epoch),
            ("bridge_send_monotonic_ns", frame.bridge_send_monotonic_ns),
            ("receiver_instance_id", frame.receiver_instance_id),
            ("receiver_frame_sequence", frame.receiver_frame_sequence),
            ("association_id", frame.association_id),
            ("raw_packet", np.frombuffer(frame.raw_packet, dtype=np.uint8)),
            ("left_valid", True),
            ("right_valid", True),
            ("left_pose", _finite(frame.left_pose, "left_pose")),
            ("right_pose", _finite(frame.right_pose, "right_pose")),
            ("upper_limb_skeleton_valid", bool(frame.upper_limb_skeleton_valid)),
            ("upper_limb_rotations_valid", bool(frame.upper_limb_rotations_valid)),
            ("upper_limb_points", _finite(frame.upper_limb_points, "upper_limb_points")),
            ("upper_limb_rotations_xyzw", _finite(frame.upper_limb_rotations_xyzw, "upper_limb_rotations_xyzw")),
        )
        for name, value in values:
            self._append(group[name], value)
        self._maybe_flush()

    def _append_nullable_sequence(self, group: h5py.Group, sequence: int | None) -> None:
        self._append(group["source_sequence"], -1 if sequence is None else sequence)
        self._append(group["source_sequence_valid"], sequence is not None)

    def append_hand_observation(
        self,
        observation: HandObservation,
        received_time_ns: int | None = None,
    ) -> None:
        self._require_extended()
        if not isinstance(observation, HandObservation):
            raise TypeError("observation must be hand_tracking.models.HandObservation")
        group = self._file["observation/hand_tracking"][observation.side]
        receive = self._record_time(
            observation.received_timestamp_ns if received_time_ns is None else received_time_ns
        )
        source_time, source_valid = _source_time(observation.source_timestamp_ns)
        self._append(group["time_ns"], receive)
        self._append(group["source_time_ns"], source_time)
        self._append(group["source_time_valid"], source_valid)
        self._append(group["received_timestamp_ns"], observation.received_timestamp_ns)
        self._append(group["source"], observation.source)
        self._append(group["source_instance_id"], observation.source_instance_id)
        self._append_nullable_sequence(group, observation.source_sequence)
        self._append(group["receiver_instance_id"], observation.receiver_instance_id)
        self._append(group["receiver_frame_sequence"], observation.receiver_frame_sequence)
        self._append(group["coordinate_frame"], observation.coordinate_frame)
        self._append(group["mapping_version"], observation.mapping_version)
        self._append(group["valid"], bool(observation.valid))
        self._append(group["joint_valid"], observation.joint_valid)
        self._append(group["keypoints_m"], _finite(observation.keypoints_m, "keypoints_m"))
        wrist_valid = observation.wrist_pose is not None
        self._append(
            group["wrist_pose"],
            _finite(observation.wrist_pose, "wrist_pose") if wrist_valid else np.full(7, np.nan),
        )
        self._append(group["wrist_pose_valid"], wrist_valid)
        self._append(group["frame_association_id"], observation.frame_association_id)
        self._maybe_flush()

    def append_arm_input_observation(
        self,
        observation: ArmInputObservationModel,
        received_time_ns: int | None = None,
    ) -> None:
        self._require_extended()
        if not isinstance(observation, ArmInputObservationModel):
            raise TypeError("observation must hand_tracking.models.ArmInputObservation")
        group = self._file["observation/arm_input"][observation.side]
        receive = self._record_time(
            observation.received_timestamp_ns if received_time_ns is None else received_time_ns
        )
        source_time, source_valid = _source_time(observation.source_timestamp_ns)
        self._append(group["time_ns"], receive)
        self._append(group["source_time_ns"], source_time)
        self._append(group["source_time_valid"], source_valid)
        self._append(group["received_timestamp_ns"], observation.received_timestamp_ns)
        self._append(group["source"], observation.source)
        self._append(group["tracked_frame"], observation.tracked_frame)
        self._append(group["reference_frame"], observation.reference_frame)
        self._append(group["source_instance_id"], observation.source_instance_id)
        self._append_nullable_sequence(group, observation.source_sequence)
        self._append(group["receiver_instance_id"], observation.receiver_instance_id)
        self._append(group["receiver_frame_sequence"], observation.receiver_frame_sequence)
        self._append(group["mapping_version"], observation.mapping_version)
        pose_valid = observation.pose is not None
        self._append(group["pose"], _finite(observation.pose, "pose") if pose_valid else np.full(7, np.nan))
        self._append(group["pose_valid"], pose_valid)
        elbow_valid = observation.elbow_pose is not None
        if "elbow_pose" in group:
            self._append(
                group["elbow_pose"],
                _finite(observation.elbow_pose, "elbow_pose") if elbow_valid else np.full(7, np.nan),
            )
            self._append(group["elbow_pose_valid"], elbow_valid)
        self._append(group["valid"], bool(observation.valid))
        self._append(group["frame_association_id"], observation.frame_association_id)
        self._maybe_flush()

    def append_arm_target(self, target: ArmTargetCommand, received_time_ns: int | None = None) -> None:
        if not isinstance(target, ArmTargetCommand): raise TypeError("target must be ArmTargetCommand")
        if not target.tracking_valid and self._schema_version not in _EXTENDED_SCHEMA_VERSIONS:
            raise SessionH5Error("tracking hold targets require session HDF5 schema 1.1")
        group = self._file["target/arm"][target.side]
        self._common(group, self._record_time(received_time_ns), target.source_timestamp_ns, target.envelope.publisher_instance_id, target.envelope.sequence)
        group.attrs.update(frame_id=target.frame_id, source=target.source)
        self._append(group["pose"], target.position_m + target.orientation_xyzw)
        self._append(group["elbow_reference_direction"], target.elbow_reference_direction)
        if "tracking_valid" in group:
            self._append(group["tracking_valid"], target.tracking_valid)
        self._maybe_flush()

    def append_hand_target(self, target: HandTargetCommand, received_time_ns: int | None = None) -> None:
        if not isinstance(target, HandTargetCommand): raise TypeError("target must be HandTargetCommand")
        group = self._file["target/hand"][target.side]; self._common(group, self._record_time(received_time_ns), target.source_timestamp_ns, target.publisher_instance_id, target.sequence); group.attrs.update(frame_id=target.frame_id, source=target.source); self._append(group["keypoints_m"], target.keypoints_m); self._maybe_flush()

    @staticmethod
    def _set_names(group: h5py.Group, names: Iterable[str], logical_id: str) -> None:
        value = _json_text(list(names)); group.attrs["joint_names"] = value; group.attrs["names"] = value; group.attrs["logical_id"] = logical_id

    def append_arm_command(self, command: ArmJointCommand, received_time_ns: int | None = None) -> None:
        group = self._file["joint/command/arm"][command.side]; receive = self._record_time(received_time_ns); self._append(group["time_ns"], receive); self._append(group["publisher_instance_id"], command.publisher_instance_id); self._append(group["sequence"], command.sequence); self._append(group["proposal_sequence"], -1 if command.proposal_sequence is None else command.proposal_sequence); self._append(group["proposal_sequence_valid"], command.proposal_sequence is not None); self._append(group["target_sequence"], -1 if command.target_sequence is None else command.target_sequence); self._append(group["target_sequence_valid"], command.target_sequence is not None); self._append(group["position_rad"], command.position_rad); self._append(group["mode"], command.mode); self._set_names(group, command.names, command.producer); self._maybe_flush()

    def append_arm_state(self, state: ArmJointState, received_time_ns: int | None = None) -> None:
        group = self._file["joint/state/arm"]; receive = self._record_time(received_time_ns); self._append(group["time_ns"], receive); self._append(group["publisher_instance_id"], state.publisher_instance_id); self._append(group["sequence"], state.sequence); self._append(group["position_rad"], state.position_rad); valid = state.velocity_rad_s is not None; self._append(group["velocity_valid"], valid); self._append(group["velocity_rad_s"], state.velocity_rad_s if valid else np.full(14, np.nan)); self._set_names(group, state.names, state.executor); self._maybe_flush()

    def append_hand_command(self, command: HandJointCommand, received_time_ns: int | None = None) -> None:
        group = self._file["joint/command/hand"][command.side]; receive = self._record_time(received_time_ns); self._append(group["time_ns"], receive); self._append(group["publisher_instance_id"], command.publisher_instance_id); self._append(group["sequence"], command.sequence); self._append(group["position_rad"], command.position_rad); self._set_names(group, command.names, command.producer); self._maybe_flush()

    def append_hand_state(self, state: HandJointState, received_time_ns: int | None = None) -> None:
        group = self._file["joint/state/hand"][state.side]; receive = self._record_time(received_time_ns); self._append(group["time_ns"], receive); self._append(group["publisher_instance_id"], state.publisher_instance_id); self._append(group["sequence"], state.sequence); self._append(group["position_rad"], state.position_rad); valid = state.velocity_rad_s is not None; self._append(group["velocity_valid"], valid); self._append(group["velocity_rad_s"], state.velocity_rad_s if valid else np.full(20, np.nan)); self._set_names(group, state.names, state.executor); self._maybe_flush()

    def append_session_state(self, state: SessionState, received_time_ns: int | None = None) -> None:
        group = self._file["meta/session_events"]; receive = self._record_time(received_time_ns); self._append(group["time_ns"], receive); self._append(group["publisher_instance_id"], state.publisher_instance_id); self._append(group["state"], state.state); self._append(group["reason"], state.reason); self._append(group["source"], state.source); self._append(group["intent_sequence"], -1 if state.intent_sequence is None else state.intent_sequence); self._append(group["intent_sequence_valid"], state.intent_sequence is not None); self._maybe_flush()

    append_raw_mocap_live = append_raw_mocap
    append_raw_h5_replay = append_raw_h5
    append_raw_pico_hand_tracking = append_raw_pico
    append_raw_manus_hand_tracking = append_raw_manus
    append_raw_legacy_pico_palm = append_raw_legacy_palm
    append_raw_xr_input = append_raw_xr

    def append(self, value: Any, received_time_ns: int | None = None) -> None:
        dispatch = ((RawMocapLiveSample, self.append_raw_mocap), (RawH5ReplaySample, self.append_raw_h5), (PicoRawFrame, self.append_raw_pico), (ManusRawFrame, self.append_raw_manus), (LegacyPicoPalmFrame, self.append_raw_legacy_palm), (XrFrame, self.append_raw_xr), (HandObservation, self.append_hand_observation), (ArmInputObservationModel, self.append_arm_input_observation), (ArmTargetCommand, self.append_arm_target), (HandTargetCommand, self.append_hand_target), (ArmJointCommand, self.append_arm_command), (ArmJointState, self.append_arm_state), (HandJointCommand, self.append_hand_command), (HandJointState, self.append_hand_state), (SessionState, self.append_session_state))
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
        self._schema_version = _text(self._file.attrs.get("schema_version", ""))
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
        expected_vlen = h5py.check_dtype(vlen=np.dtype(dtype))
        if expected_vlen is str:
            return dataset.dtype.kind == "O" and h5py.check_dtype(vlen=dataset.dtype) is str
        if expected_vlen == np.dtype("uint8"):
            return dataset.dtype.kind == "O" and h5py.check_dtype(vlen=dataset.dtype) == np.dtype("uint8")
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
        schema_version = attrs.get("schema_version")
        if attrs.get("schema_name") != SCHEMA_NAME or schema_version not in (SCHEMA_VERSION, *_EXTENDED_SCHEMA_VERSIONS): raise SessionH5Error("unsupported session HDF5 schema")
        if schema_version == SCHEMA_VERSION and attrs.get("source_type") not in SOURCE_TYPES | {_LEGACY_CONTROLLER_GROUP}: raise SessionH5Error("invalid source_type")
        if schema_version == EXTENDED_SCHEMA_VERSION and attrs.get("source_type") not in EXTENDED_SOURCE_TYPES: raise SessionH5Error("invalid schema 1.1 source_type")
        if schema_version == DUAL_SCHEMA_VERSION and attrs.get('source_type') not in DUAL_SOURCE_TYPES:
            raise SessionH5Error('invalid schema 1.2 source_type')
        if not isinstance(attrs["complete"], (bool, np.bool_)): raise SessionH5Error("complete attr must be boolean")
        for key in ("robot_model", "router_zid"):
            if not isinstance(attrs[key], str) or not attrs[key]: raise SessionH5Error(f"invalid root attr: {key}")
        if not bool(attrs["complete"]) and not allow_incomplete: raise IncompleteSessionError(f"session is incomplete: {self.path}")
        expected_root_groups = {"raw", "target", "joint", "meta"} | ({"observation"} if schema_version in _EXTENDED_SCHEMA_VERSIONS else set())
        if set(self.file.keys()) != expected_root_groups: raise SessionH5Error("invalid root group set")
        raw_groups = set(self.file["raw"].keys())
        expected_raw_groups = (
            {"mocap_live", "h5_replay", "pico_hand_tracking", "manus_hand_tracking", "legacy_pico_palm"}
            if schema_version in _EXTENDED_SCHEMA_VERSIONS
            else None
        )
        if schema_version == DUAL_SCHEMA_VERSION:
            expected_raw_groups.add('tjvr_upper_limb')
            expected_raw_groups.add('manus_callbacks')
        if schema_version == DUAL_SCHEMA_VERSION:
            # Schema 1.2 predates the dedicated XR snapshot group.  Keep those
            # older PICO/TJVR captures readable, while requiring the group for
            # the new XR/Manus source type.
            legacy_dual_groups = expected_raw_groups
            xr_dual_groups = expected_raw_groups | {'xr_input'}
            raw_groups_ok = raw_groups in (legacy_dual_groups, xr_dual_groups)
            if attrs.get('source_type') == 'vr_manus_xr_sim' and raw_groups != xr_dual_groups:
                raise SessionH5Error('XR/Manus schema 1.2 recording is missing raw/xr_input')
        else:
            raw_groups_ok = raw_groups == expected_raw_groups if expected_raw_groups is not None else raw_groups in ({"mocap_live", "h5_replay"}, {"mocap_live", "h5_replay", _LEGACY_CONTROLLER_GROUP})
        expected_meta_groups = {"session_events"} | ({"hand_tracking"} if schema_version in _EXTENDED_SCHEMA_VERSIONS else set())
        if schema_version == DUAL_SCHEMA_VERSION and 'dual_audit' in self.file['meta']:
            expected_meta_groups.add('dual_audit')
        if not raw_groups_ok or set(self.file["target"].keys()) != {"arm", "hand"} or set(self.file["joint"].keys()) != {"command", "state"} or set(self.file["meta"].keys()) != expected_meta_groups: raise SessionH5Error("invalid fixed group set")
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
            # Older 1.1 files predate explicit tracking-loss holds. Accept the
            # missing field, but validate dtype and row count when present.
            tracking_specs = (("tracking_valid", (), np.bool_),) if schema_version in _EXTENDED_SCHEMA_VERSIONS and "tracking_valid" in ag else ()
            arm_rows = self._validate_group(f"target/arm/{side}", scalar_i + scalar_b + instance + (("sequence", (), np.int64), ("pose", (7,), np.float64), ("elbow_reference_direction", (3,), np.float64)) + tracking_specs)
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
        if schema_version in _EXTENDED_SCHEMA_VERSIONS:
            self._validate_extended_layout()
        if schema_version == DUAL_SCHEMA_VERSION:
            if 'dual_audit' in self.file['meta']:
                self._validate_group('meta/dual_audit', _DUAL_AUDIT_SPECS)
                self.read_dual_audit()  # validate payloads, not only HDF5 dtypes
            self._validate_group('raw/tjvr_upper_limb', _TJVR_RAW_SPECS)
            if _text(self.file['raw/tjvr_upper_limb'].attrs.get('input_stage', '')) != 'decoded_before_stream_gate':
                raise SessionH5Error('invalid TJVR recording boundary')
            callback_group = self.file['raw/manus_callbacks']
            callback_specs = _MANUS_CALLBACK_SPECS
            if set(callback_group.keys()) == {name for name, _, _ in _MANUS_CALLBACK_LEGACY_SPECS}:
                callback_specs = _MANUS_CALLBACK_LEGACY_SPECS
            self._validate_group('raw/manus_callbacks', callback_specs)
            callbacks = self.file['raw/manus_callbacks']
            if (_text(callbacks.attrs.get('input_stage', '')) != 'mediapipe21_callback_not_raw25' or
                    _text(callbacks.attrs.get('order', '')) != 'right21_left21'):
                raise SessionH5Error('invalid Manus callback recording boundary')
            if 'xr_input' in self.file['raw']:
                xr_group = self.file['raw/xr_input']
                if _text(xr_group.attrs.get('source', '')) != 'xr_observation_before_mapping':
                    raise SessionH5Error('invalid raw XR recording boundary')
                xr_specs = (_XR_RAW_SPECS if 'connection_generation' in xr_group
                            else _XR_RAW_LEGACY_SPECS)
                self._validate_group('raw/xr_input', xr_specs)

    def _validate_extended_layout(self) -> None:
        """Validate schema 1.1 raw and receive-only observation groups."""
        pico_specs = (
            ("time_ns", (), np.int64),
            ("source_time_ns", (), np.int64),
            ("source_time_valid", (), np.bool_),
            ("source_timestamp_ms", (), np.int64),
            ("protocol_version", (), np.uint8),
            ("flags", (), np.uint8),
            ("joint_count", (), np.uint8),
            ("head_valid", (), np.bool_),
            ("head_pose", (7,), np.float64),
            ("receiver_instance_id", (), _STRING),
            ("connection_generation", (), np.int64),
            ("receiver_frame_sequence", (), np.int64),
            ("association_id", (), _STRING),
            ("raw_packet", (), _UINT8_VECTOR),
        )
        pico_group = self.file["raw/pico_hand_tracking"]
        if self._schema_version == DUAL_SCHEMA_VERSION and 'received_timestamp_ns' in pico_group:
            pico_specs += (('received_timestamp_ns', (), np.int64),)
        pico_rows = self._validate_group("raw/pico_hand_tracking", pico_specs, children=("hands",))
        if 'received_timestamp_ns' in pico_group and np.any(pico_group['received_timestamp_ns'][:] < 0):
            raise SessionH5Error('invalid PICO receiver timestamp')
        if _text(pico_group.attrs.get("source", "")) != "pico2":
            raise SessionH5Error("invalid PICO raw source attr")
        if set(pico_group["hands"].keys()) != set(SIDES):
            raise SessionH5Error("invalid PICO raw hand group set")
        pico_hand_specs = (
            ("valid", (), np.bool_),
            ("wrist_valid", (), np.bool_),
            ("wrist_pose", (7,), np.float64),
            ("joint_valid", (26,), np.bool_),
            ("joint_poses", (26, 7), np.float64),
            ("joint_radii_m", (26,), np.float64),
        )
        for side in SIDES:
            group = pico_group["hands"][side]
            if _text(group.attrs.get("side", "")) != side:
                raise SessionH5Error(f"invalid PICO hand side attr: {side}")
            if _text(group.attrs.get("joint_names", "")) != _json_text(list(PICO_JOINT_NAMES)):
                raise SessionH5Error(f"invalid PICO joint names: {side}")
            if self._validate_group(f"raw/pico_hand_tracking/hands/{side}", pico_hand_specs) != pico_rows:
                raise SessionH5Error(f"PICO parent/hand row mismatch: {side}")

        manus_specs = (
            ("time_ns", (), np.int64),
            ("source_time_ns", (), np.int64),
            ("source_time_valid", (), np.bool_),
            ("source_sequence", (), np.int64),
            ("sdk_publish_time", (), np.int64),
            ("glove_id", (), _STRING),
            ("side", (), _STRING),
            ("receiver_instance_id", (), _STRING),
            ("receiver_frame_sequence", (), np.int64),
            ("association_id", (), _STRING),
            ("node_count", (), np.int64),
            ("node_valid", (64,), np.bool_),
            ("node_positions", (64, 3), np.float64),
            ("node_quaternions_wxyz", (64, 4), np.float64),
            ("node_semantics_json", (), _STRING),
            ("payload_json", (), _STRING),
        )
        manus_group = self.file["raw/manus_hand_tracking"]
        if _text(manus_group.attrs.get("source", "")) != "manus":
            raise SessionH5Error("invalid Manus raw source attr")
        self._validate_group("raw/manus_hand_tracking", manus_specs)

        legacy_specs = (
            ("time_ns", (), np.int64),
            ("source_time_ns", (), np.int64),
            ("source_time_valid", (), np.bool_),
            ("protocol_version", (), np.uint8),
            ("packet_size", (), np.int64),
            ("flags", (), np.int64),
            ("source_sequence", (), np.int64),
            ("tracking_epoch", (), np.int64),
            ("bridge_send_monotonic_ns", (), np.int64),
            ("receiver_instance_id", (), _STRING),
            ("receiver_frame_sequence", (), np.int64),
            ("association_id", (), _STRING),
            ("raw_packet", (), _UINT8_VECTOR),
            ("left_valid", (), np.bool_),
            ("right_valid", (), np.bool_),
            ("left_pose", (7,), np.float64),
            ("right_pose", (7,), np.float64),
            ("upper_limb_skeleton_valid", (), np.bool_),
            ("upper_limb_rotations_valid", (), np.bool_),
            ("upper_limb_points", (8, 3), np.float64),
            ("upper_limb_rotations_xyzw", (8, 4), np.float64),
        )
        legacy_group = self.file["raw/legacy_pico_palm"]
        if _text(legacy_group.attrs.get("source", "")) != "pico_manus_teleop_experiments":
            raise SessionH5Error("invalid legacy palm source attr")
        self._validate_group("raw/legacy_pico_palm", legacy_specs)

        if set(self.file["observation"].keys()) != {"hand_tracking", "arm_input"}:
            raise SessionH5Error("invalid observation domain group set")
        if set(self.file["observation/hand_tracking"].keys()) != set(SIDES) or set(self.file["observation/arm_input"].keys()) != set(SIDES):
            raise SessionH5Error("invalid observation side group set")
        hand_specs = (
            ("time_ns", (), np.int64),
            ("source_time_ns", (), np.int64),
            ("source_time_valid", (), np.bool_),
            ("received_timestamp_ns", (), np.int64),
            ("source", (), _STRING),
            ("source_instance_id", (), _STRING),
            ("source_sequence", (), np.int64),
            ("source_sequence_valid", (), np.bool_),
            ("receiver_instance_id", (), _STRING),
            ("receiver_frame_sequence", (), np.int64),
            ("coordinate_frame", (), _STRING),
            ("mapping_version", (), _STRING),
            ("valid", (), np.bool_),
            ("joint_valid", (21,), np.bool_),
            ("keypoints_m", (21, 3), np.float64),
            ("wrist_pose", (7,), np.float64),
            ("wrist_pose_valid", (), np.bool_),
            ("frame_association_id", (), _STRING),
        )
        arm_specs = (
            ("time_ns", (), np.int64),
            ("source_time_ns", (), np.int64),
            ("source_time_valid", (), np.bool_),
            ("received_timestamp_ns", (), np.int64),
            ("source", (), _STRING),
            ("tracked_frame", (), _STRING),
            ("reference_frame", (), _STRING),
            ("source_instance_id", (), _STRING),
            ("source_sequence", (), np.int64),
            ("source_sequence_valid", (), np.bool_),
            ("receiver_instance_id", (), _STRING),
            ("receiver_frame_sequence", (), np.int64),
            ("mapping_version", (), _STRING),
            ("pose", (7,), np.float64),
            ("pose_valid", (), np.bool_),
            ("elbow_pose", (7,), np.float64),
            ("elbow_pose_valid", (), np.bool_),
            ("valid", (), np.bool_),
            ("frame_association_id", (), _STRING),
        )
        for side in SIDES:
            hand_group = self.file[f"observation/hand_tracking/{side}"]
            arm_group = self.file[f"observation/arm_input/{side}"]
            if _text(hand_group.attrs.get("side", "")) != side or _text(arm_group.attrs.get("side", "")) != side:
                raise SessionH5Error(f"invalid observation side attr: {side}")
            self._validate_group(f"observation/hand_tracking/{side}", hand_specs)
            arm_group = self.file[f"observation/arm_input/{side}"]
            # The forearm pose is an additive extension.  Older 1.1/1.2
            # captures remain readable and expose ``elbow_pose=None``.
            if "elbow_pose" in arm_group or "elbow_pose_valid" in arm_group:
                if {"elbow_pose", "elbow_pose_valid"} - set(arm_group):
                    raise SessionH5Error(f"incomplete elbow pose datasets: {arm_group.name}")
                self._validate_group(f"observation/arm_input/{side}", arm_specs)
            else:
                self._validate_group(
                    f"observation/arm_input/{side}",
                    tuple(item for item in arm_specs if item[0] not in {"elbow_pose", "elbow_pose_valid"}),
                )
        metadata_group = self.file["meta/hand_tracking"]
        if set(metadata_group.keys()) != set():
            raise SessionH5Error("invalid hand-tracking metadata group")
        try:
            metadata = json.loads(_text(metadata_group.attrs["metadata_json"]))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise SessionH5Error("invalid hand-tracking metadata JSON") from exc
        if not isinstance(metadata, dict):
            raise SessionH5Error("hand-tracking metadata must be a JSON object")

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

    def read_raw_reference_tjvr(self) -> list[dict[str, Any]]:
        if self._schema_version != DUAL_SCHEMA_VERSION:
            return []
        from ..hand_tracking.reference_tjvr import parse_reference_tjvr_packet
        rows = self._row_group(self.file['raw/tjvr_upper_limb'])
        for row in rows:
            row['raw_packet'] = bytes(row['raw_packet'])
            parse_reference_tjvr_packet(row['raw_packet'], receiver_instance_id=row['receiver_instance_id'],
                receiver_frame_sequence=row['receiver_frame_sequence'],
                received_timestamp_ns=row['received_timestamp_ns'])
        return rows

    def read_manus_callbacks(self) -> list[dict[str, Any]]:
        if self._schema_version != DUAL_SCHEMA_VERSION:
            return []
        group = self.file['raw/manus_callbacks']
        rows = self._row_group(group)
        for row in rows:
            count, side = row['point_count'], row['single_hand_side']
            if count not in (63, 126) or side not in SIDES:
                raise SessionH5Error('invalid recorded callback shape/side')
            offset = 63 if count == 63 and side == 'left' else 0
            row['points'] = row['points'][offset:offset + count]
            _finite(row['points'], 'recorded callback points')
            if 'source_sequences_json' in group:
                for json_name, output_name in (
                    ('source_sequences_json', 'source_sequences'),
                    ('source_timestamps_ns_json', 'source_timestamps_ns'),
                ):
                    try:
                        value = json.loads(row.pop(json_name))
                    except (TypeError, ValueError, json.JSONDecodeError) as exc:
                        raise SessionH5Error('invalid recorded Manus callback metadata') from exc
                    if not isinstance(value, dict):
                        raise SessionH5Error('recorded Manus callback metadata must be an object')
                    row[output_name] = value
            else:
                row['source_sequences'] = {}
                row['source_timestamps_ns'] = {}
        return rows

    def read_raw_xr(self, *, preserve_source_clock: bool = False) -> list[dict[str, Any]]:
        if self._schema_version != DUAL_SCHEMA_VERSION or 'xr_input' not in self.file['raw']:
            return []
        group = self.file['raw/xr_input']
        rows = self._row_group(group)
        if not isinstance(preserve_source_clock, bool):
            raise TypeError('preserve_source_clock must be boolean')
        if not preserve_source_clock:
            rows = self._source(rows)
        else:
            for row in rows:
                row['source_timestamp_ns'] = (
                    int(row['source_time_ns'])
                    if bool(row['source_time_valid']) else None
                )
        for row in rows:
            try:
                frame = json.loads(row.pop('frame_json'))
                decoded = XrFrame.from_dict(frame)
                trackers = json.loads(row.pop('trackers_json'))
            except (TypeError, ValueError, json.JSONDecodeError) as exc:
                raise SessionH5Error('invalid recorded XR frame JSON') from exc
            if not isinstance(trackers, list) or row['tracker_count'] != len(trackers):
                raise SessionH5Error('recorded XR tracker count does not match payload')
            if ('connection_generation' in row and
                    row['connection_generation'] != decoded.connection_generation):
                raise SessionH5Error('recorded XR connection generation does not match payload')
            row['frame'] = frame
            row['trackers'] = trackers
            if not bool(row['hmd_valid']):
                row['hmd_pose'] = None
            for side in SIDES:
                if not bool(row[f'{side}_controller_valid']) and not bool(
                    row[f'{side}_controller_available']
                ):
                    row[f'{side}_controller_pose'] = None
        return rows
    def read_raw_h5(self) -> list[dict[str, Any]]:
        rows = self._source(self._row_group(self.file["raw/h5_replay"])); groups = self.file["raw/h5_replay/hands"]
        for index, row in enumerate(rows):
            row["source_type"] = "h5_replay"; row["hands"] = {}
            for side in SIDES:
                hand = groups[side]; valid = bool(hand["valid"][index]); item = {"valid": valid, "wrist_pose": hand["wrist"][index].tolist() if valid else None, "keypoints_world_m": hand["keypoints_world"][index].tolist() if valid else None}; item["wuji2_joints_rad"] = None if "wuji2_joints" not in hand or not np.isfinite(hand["wuji2_joints"][index]).all() else hand["wuji2_joints"][index].tolist(); row["hands"][side] = item
        return rows

    def read_raw_pico(self) -> list[dict[str, Any]]:
        if self._schema_version not in _EXTENDED_SCHEMA_VERSIONS:
            return []
        group = self.file["raw/pico_hand_tracking"]
        rows = self._source(self._row_group(group))
        for index, row in enumerate(rows):
            row["source_type"] = "pico2"
            row["raw_packet"] = bytes(np.asarray(group["raw_packet"][index], dtype=np.uint8).tolist())
            row["hands"] = {}
            for side in SIDES:
                hand = group["hands"][side]
                row["hands"][side] = {
                    "valid": bool(hand["valid"][index]),
                    "wrist_valid": bool(hand["wrist_valid"][index]),
                    "wrist_pose": hand["wrist_pose"][index].tolist(),
                    "joint_valid": hand["joint_valid"][index].tolist(),
                    "joint_poses": hand["joint_poses"][index].tolist(),
                    "joint_radii_m": hand["joint_radii_m"][index].tolist(),
                }
        return rows

    def read_raw_manus(self) -> list[dict[str, Any]]:
        if self._schema_version not in _EXTENDED_SCHEMA_VERSIONS:
            return []
        group = self.file["raw/manus_hand_tracking"]
        rows = self._source(self._row_group(group))
        for index, row in enumerate(rows):
            row["source_type"] = "manus"
            row["node_positions"] = group["node_positions"][index].tolist()
            row["node_quaternions_wxyz"] = group["node_quaternions_wxyz"][index].tolist()
            row["node_valid"] = group["node_valid"][index].tolist()
            row["node_semantics"] = json.loads(_text(group["node_semantics_json"][index]))
            row["payload"] = json.loads(_text(group["payload_json"][index]))
        return rows

    def read_raw_legacy_palm(self) -> list[dict[str, Any]]:
        if self._schema_version not in _EXTENDED_SCHEMA_VERSIONS:
            return []
        group = self.file["raw/legacy_pico_palm"]
        rows = self._source(self._row_group(group))
        for index, row in enumerate(rows):
            row["source_type"] = "legacy_pico_palm"
            row["raw_packet"] = bytes(np.asarray(group["raw_packet"][index], dtype=np.uint8).tolist())
            row["upper_limb_points"] = group["upper_limb_points"][index].tolist()
            row["upper_limb_rotations_xyzw"] = group["upper_limb_rotations_xyzw"][index].tolist()
        return rows

    @staticmethod
    def _nullable_sequence(row: dict[str, Any]) -> None:
        valid = bool(row.pop("source_sequence_valid", False))
        row["source_sequence"] = int(row["source_sequence"]) if valid else None

    def read_hand_observation(self, side: str) -> list[dict[str, Any]]:
        if side not in SIDES:
            raise ValueError("side must be left or right")
        if self._schema_version not in _EXTENDED_SCHEMA_VERSIONS:
            return []
        group = self.file[f"observation/hand_tracking/{side}"]
        rows = self._source(self._row_group(group))
        for row in rows:
            self._nullable_sequence(row)
            if not bool(row.pop("wrist_pose_valid", False)):
                row["wrist_pose"] = None
        return rows

    def read_arm_input_observation(self, side: str) -> list[dict[str, Any]]:
        if side not in SIDES:
            raise ValueError("side must be left or right")
        if self._schema_version not in _EXTENDED_SCHEMA_VERSIONS:
            return []
        group = self.file[f"observation/arm_input/{side}"]
        rows = self._source(self._row_group(group))
        for row in rows:
            self._nullable_sequence(row)
            if not bool(row.pop("pose_valid", False)):
                row["pose"] = None
            if "elbow_pose_valid" in group:
                if not bool(row.pop("elbow_pose_valid", False)):
                    row["elbow_pose"] = None
            else:
                row["elbow_pose"] = None
        return rows

    def read_dual_audit(self) -> list[dict[str, Any]]:
        if self._schema_version != DUAL_SCHEMA_VERSION or 'dual_audit' not in self.file['meta']:
            return []
        rows = self._row_group(self.file['meta/dual_audit'])
        for row in rows:
            try:
                encoded = row.pop('payload_json')
                if len(encoded.encode('utf-8')) > 1_048_576:
                    raise ValueError('oversized payload')
                payload = json.loads(encoded)
                _json_text(payload)  # rejects NaN/Infinity accepted by json.loads
                if (row['kind'] not in _DUAL_AUDIT_KINDS or not isinstance(payload, dict) or
                        not 0 < row['received_timestamp_ns'] < 2**63):
                    raise ValueError('invalid audit record')
                row['payload'] = payload
            except (TypeError, ValueError) as exc:
                raise SessionH5Error('invalid dual audit payload') from exc
        return rows

    def read_hand_tracking_metadata(self) -> dict[str, Any]:
        if self._schema_version not in _EXTENDED_SCHEMA_VERSIONS:
            return {}
        try:
            value = json.loads(_text(self.file["meta/hand_tracking"].attrs["metadata_json"]))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise SessionH5Error("invalid hand-tracking metadata JSON") from exc
        if not isinstance(value, dict):
            raise SessionH5Error("hand-tracking metadata must be a JSON object")
        return value

    def _target(self, side: str, hand: bool) -> list[dict[str, Any]]:
        if side not in SIDES: raise ValueError("side must be left or right")
        group = self.file[f"target/{'hand' if hand else 'arm'}/{side}"]; rows = self._source(self._row_group(group))
        for row in rows:
            row["frame_id"] = _text(group.attrs["frame_id"]); row["source"] = _text(group.attrs.get("source", ""))
            if not hand:
                row["tracking_valid"] = bool(row.get("tracking_valid", True))
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
        methods = {"raw_mocap": self.read_raw_mocap, "raw_h5": self.read_raw_h5, "raw_pico": self.read_raw_pico, "raw_manus": self.read_raw_manus, "raw_legacy_palm": self.read_raw_legacy_palm, "legacy_controller": self.read_legacy_controller, "arm_state": self.read_arm_state, "session_state": self.read_session_events}; side_methods = {"arm_target": self.read_arm_target, "hand_target": self.read_hand_target, "arm_command": self.read_arm_command, "hand_command": self.read_hand_command, "hand_state": self.read_hand_state, "hand_observation": self.read_hand_observation, "arm_input_observation": self.read_arm_input_observation}
        methods.update(
            raw_reference_tjvr=self.read_raw_reference_tjvr,
            manus_callbacks=self.read_manus_callbacks,
            raw_xr=self.read_raw_xr,
        )
        if kind in methods: return methods[kind]()
        if side is None or kind not in side_methods: raise ValueError(f"unknown stream or missing side: {kind}")
        return side_methods[kind](side)
    def close(self) -> None:
        if not self._closed: self._file.close(); self._closed = True
    def __enter__(self) -> "SessionH5Reader": return self
    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None: self.close()


SessionH5Loader = SessionH5Reader

def load_session_h5(path: str | Path, *, allow_incomplete: bool = False) -> SessionH5Reader: return SessionH5Reader(path, allow_incomplete=allow_incomplete)

__all__ = ["SCHEMA_NAME", "SCHEMA_VERSION", "EXTENDED_SCHEMA_VERSION", "DUAL_SCHEMA_VERSION", "SOURCE_TYPES", "EXTENDED_SOURCE_TYPES", "DUAL_SOURCE_TYPES", "SessionH5Error", "IncompleteSessionError", "UnsafeSessionLinkError", "SessionH5Writer", "SessionH5Reader", "SessionH5Loader", "load_session_h5"]
