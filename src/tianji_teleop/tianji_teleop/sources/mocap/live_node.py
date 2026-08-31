"""Acquisition ``mocap/aligned/hands`` live source.

Product live deliberately has no Motive rigid-body or keyboard-step path.  The
acquisition stream is the sole live hand/wrist truth; robot markers remain an
H5/diagnostic concern.
"""
from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np
from scipy.spatial.transform import Rotation

from ...protocol import topics
from ...protocol.messages import ProtocolError
from ...sources.common.freshness import FreshnessGate
from ...sources.common.real_admission import RealCapabilityInput, parse_real_capability
from ...sources.common.session_client import SessionClient
from ...sources.common.target_conditioner import TargetConditioningSettings
from ...sources.common.target_mapper import ArmTargetBatch, EndEffectorTargetMapper
from ...sources.common.target_publisher import SequenceAllocator, TargetPublisher
from ..common.wrist_pose_frame import WristPoseFrame
from ...zenoh_util import (
    ZenohJsonSub,
    load_tianji_config,
    load_node_config,
    open_session,
    parse_cli_args,
    parse_param_override,
    require_single_router,
)
from ..common.keyboard import raw_keyboard

_LOG = logging.getLogger("mocap_live")

DEFAULT_PARAMETERS = {
    "rate": 60.0,
    "stale_timeout": 0.5,
    "translation_gain": [1.0, 1.0, 1.0],
    "rotation_gain": 1.0,
    "workspace_relative_radii_m": [0.42, 0.38, 0.38],
    "workspace_soft_zone_ratio": 0.90,
    "maximum_linear_speed_m_s": 0.36,
    "maximum_angular_speed_rad_s": 1.55,
    "maximum_linear_acceleration_m_s2": 3.5,
    "maximum_angular_acceleration_rad_s2": 9.0,
    "left_default_elbow_direction": [0.45638698, -0.74604902, -0.48489358],
    "right_default_elbow_direction": [0.45638698, 0.74604902, -0.48489358],
    "real_preflight_passed": False,
    "real_mode": False,
    "speed": 1.0,
    "yaw_deg": 0.0,
}

_FRAME_STALE_S = 0.5


@dataclass(frozen=True)
class AlignedHandFrame:
    stream_instance_id: str
    stream_sequence: int
    frame_index: int
    source_timestamp_ns: int
    frame_valid: bool
    hands: dict[str, dict[str, Any]]
    router_zid: str


def _finite_pose(value: Any, field: str) -> list[float]:
    values = np.asarray(value, dtype=np.float64)
    if values.shape != (7,) or not np.isfinite(values).all():
        raise ValueError(f"{field} must be a finite 7-vector")
    norm = float(np.linalg.norm(values[3:]))
    if not 0.999 <= norm <= 1.001:
        raise ValueError(f"{field} quaternion must be normalized")
    values[3:] /= norm
    return values.tolist()


def _hand(value: Any, side: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"hands.{side} must be an object")
    valid = value.get("valid")
    if not isinstance(valid, bool):
        raise ValueError(f"hands.{side}.valid must be boolean")
    if not valid:
        if value.get("wrist_pose") is not None or value.get("keypoints_world_m") is not None:
            raise ValueError(f"invalid {side} hand must use null fields")
        return {"valid": False, "wrist_pose": None, "keypoints_world_m": None}
    wrist = _finite_pose(value.get("wrist_pose"), f"hands.{side}.wrist_pose")
    points = np.asarray(value.get("keypoints_world_m"), dtype=np.float64)
    if points.shape != (21, 3) or not np.isfinite(points).all():
        raise ValueError(f"hands.{side}.keypoints_world_m must be finite (21,3)")
    return {"valid": True, "wrist_pose": wrist, "keypoints_world_m": points.tolist()}


def parse_aligned_hands(payload: bytes | str | Mapping[str, Any]) -> AlignedHandFrame:
    """Parse and validate the external acquisition envelope without mutation."""
    if isinstance(payload, (bytes, bytearray, memoryview)):
        payload = json.loads(bytes(payload).decode("utf-8"))
    elif isinstance(payload, str):
        payload = json.loads(payload)
    if not isinstance(payload, Mapping):
        raise ValueError("aligned hands payload must be an object")
    required = {"stream_instance_id", "stream_sequence", "router_zid", "time_ns", "frame_index", "frame_valid", "hands"}
    missing = required - set(payload)
    if missing:
        raise ValueError(f"aligned hands missing fields: {', '.join(sorted(missing))}")
    if (
        not isinstance(payload["stream_instance_id"], str)
        or not payload["stream_instance_id"]
        or not isinstance(payload["router_zid"], str)
        or not payload["router_zid"]
        or isinstance(payload["stream_sequence"], bool)
        or not isinstance(payload["stream_sequence"], int)
        or isinstance(payload["time_ns"], bool)
        or not isinstance(payload["time_ns"], int)
        or isinstance(payload["frame_index"], bool)
        or not isinstance(payload["frame_index"], int)
        or not isinstance(payload["frame_valid"], bool)
    ):
        raise ValueError("aligned hands envelope has invalid field types")
    hands = payload["hands"]
    if not isinstance(hands, Mapping) or set(hands) != {"left", "right"}:
        raise ValueError("aligned hands must contain exactly left and right")
    return AlignedHandFrame(
        stream_instance_id=payload["stream_instance_id"],
        stream_sequence=payload["stream_sequence"],
        frame_index=payload["frame_index"],
        source_timestamp_ns=payload["time_ns"],
        frame_valid=payload["frame_valid"],
        hands={side: _hand(hands[side], side) for side in ("left", "right")},
        router_zid=payload["router_zid"],
    )


class MocapLiveNode:
    """Freeze aligned wrist references on ``s`` and publish canonical targets."""

    def __init__(
        self,
        session: Any,
        params: dict[str, Any] | None = None,
        *,
        publisher_instance_id: str,
        router_zid: str,
        coordinator_instance_id: str | None = None,
        active_sides: tuple[str, ...] = ("right",),
        clock: Any = time.monotonic,
        real_capability: RealCapabilityInput | Mapping[str, Any] | Any | None = None,
    ) -> None:
        params = {**DEFAULT_PARAMETERS, **(params or {})}
        for field in ("real_preflight_passed", "real_mode"):
            if not isinstance(params[field], bool):
                raise ValueError(f"{field} must be a YAML boolean")
        if params["real_preflight_passed"]:
            raise ValueError(
                "real_preflight_passed cannot be supplied by YAML; "
                "use typed runtime preflight"
            )
        self._real_mode = params["real_mode"]
        self._speed = float(params["speed"])
        self._yaw_deg = float(params["yaw_deg"])
        if not np.isfinite(self._speed) or self._speed <= 0.0:
            raise ValueError("speed must be positive and finite")
        if not np.isfinite(self._yaw_deg):
            raise ValueError("yaw_deg must be finite")
        if self._real_mode and real_capability is None:
            raise ValueError("real mode requires typed real_capability input")
        if real_capability is not None and not (
            isinstance(real_capability, RealCapabilityInput)
            or callable(real_capability)
        ):
            raise ValueError(
                "real_capability must be typed runtime input, not YAML mapping"
            )
        self._real_capability = real_capability
        self._real_capability_error: str | None = None
        if set(active_sides) not in ({"left"}, {"right"}, {"left", "right"}):
            raise ValueError("active_sides must contain left/right")
        self._active_sides = tuple(active_sides)
        self._clock = clock
        self._rate = float(params["rate"])
        if not np.isfinite(self._rate) or self._rate <= 0.0:
            raise ValueError("rate must be positive")
        self._stale_timeout_s = float(params["stale_timeout"])
        if not np.isfinite(self._stale_timeout_s) or self._stale_timeout_s <= 0.0:
            raise ValueError("stale_timeout must be positive")
        self._session = session
        allocator = SequenceAllocator()
        self._session_client = SessionClient(
            session,
            source="mocap_live",
            publisher_instance_id=publisher_instance_id,
            router_zid=router_zid,
            expected_coordinator_instance_id=coordinator_instance_id,
            allocator=allocator,
        )
        self._publisher = TargetPublisher(
            session,
            source="mocap_live",
            publisher_instance_id=publisher_instance_id,
            router_zid=router_zid,
            allocator=allocator,
        )
        config = load_tianji_config()
        self._config = config
        settings = TargetConditioningSettings(
            rate_hz=self._rate,
            translation_gain=params["translation_gain"],
            rotation_gain=float(params["rotation_gain"]),
            workspace_relative_radii_m=params["workspace_relative_radii_m"],
            workspace_soft_zone_ratio=float(params["workspace_soft_zone_ratio"]),
            maximum_linear_speed_m_s=float(params["maximum_linear_speed_m_s"]),
            maximum_angular_speed_rad_s=float(params["maximum_angular_speed_rad_s"]),
            maximum_linear_acceleration_m_s2=float(params["maximum_linear_acceleration_m_s2"]),
            maximum_angular_acceleration_rad_s2=float(params["maximum_angular_acceleration_rad_s2"]),
        )
        self._mapper = EndEffectorTargetMapper(
            config,
            rate=self._rate,
            conditioning_settings=settings,
            default_zsp_directions={
                side: params[f"{side}_default_elbow_direction"]
                for side in ("left", "right")
            },
        )
        self._session_client.start()
        self._frame_sub = ZenohJsonSub(
            session, topics.MOCAP_ALIGNED_HANDS, self._on_aligned_payload
        )
        self._lock = threading.RLock()
        self._latest: AlignedHandFrame | None = None
        self._received_at = 0.0
        self._stop_event = threading.Event()
        self._stream_instance_id: str | None = None
        self._stream_sequence = -1
        self._references: dict[str, np.ndarray] = {}
        self._phase = "armed"
        self._last_error: str | None = None
        self._real_capability_error: str | None = None
        self._return_deadline = 0.0
        self._return_timed_out = False
        self._closed = False

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def latest_frame(self) -> AlignedHandFrame | None:
        with self._lock:
            return self._latest

    @property
    def stream_instance_id(self) -> str | None:
        return self._stream_instance_id
    def _on_aligned_payload(self, payload: Mapping[str, Any] | bytes) -> None:
        try:
            frame = parse_aligned_hands(payload)
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self._last_error = str(exc)
            return
        with self._lock:
            if frame.router_zid != self._session_client.router_zid:
                self._last_error = "aligned hands router_zid mismatch"
                self._request_return_locked("aligned_router_mismatch")
                return
            if self._stream_instance_id is None:
                self._stream_instance_id = frame.stream_instance_id
                self._stream_sequence = -1
            elif frame.stream_instance_id != self._stream_instance_id:
                was_active = self._phase not in {"armed", "returning", "fault"}
                self._stream_instance_id = frame.stream_instance_id
                self._stream_sequence = -1
                self._references.clear()
                if was_active:
                    self._request_return_locked("aligned_stream_instance_changed")
            if frame.stream_sequence <= self._stream_sequence:
                return
            self._stream_sequence = frame.stream_sequence
            self._latest = frame
            self._received_at = self._clock()
            self._publisher.publish_raw_mocap_live(_to_raw_payload(frame))
    def _request_return_locked(self, reason: str) -> None:
        was_active = self._phase not in {"armed", "returning", "fault"}
        try:
            self._session_client.request_return(reason)
        except (RuntimeError, ValueError) as exc:
            self._last_error = str(exc)
        self._references.clear()
        self._return_timed_out = False
        self._return_deadline = time.monotonic() + 5.0
        if was_active:
            self._phase = "returning"
    def _complete_return_locked(self) -> None:
        self._references.clear()
        self._phase = "armed"
        self._return_timed_out = False
        self._return_deadline = 0.0

    def request_start(self) -> bool:
        with self._lock:
            frame = self._latest
            if self._phase != "armed" or frame is None:
                return False
            if self._real_mode and not self._real_capability_snapshot()[0]:
                self._real_capability_error = self._real_capability_snapshot()[1]
                return False
            if any(not frame.hands[side]["valid"] for side in self._active_sides):
                return False
            self._references = {
                side: np.asarray(frame.hands[side]["wrist_pose"], dtype=np.float64).copy()
                for side in self._active_sides
            }
            if not self._session_client.startup_ready:
                self._references.clear()
                return False
            try:
                self._session_client.request_start("mocap_s")
            except (RuntimeError, ValueError) as exc:
                self._last_error = str(exc)
                self._references.clear()
                return False
            self._phase = "start_pending"
            return True

    def _on_key(self, value: str) -> None:
        if value == "s":
            if self._phase == "armed":
                self.request_start()
            elif self._phase in {"start_pending", "teleop"}:
                with self._lock:
                    self._request_return_locked("mocap_s_return")
        elif value in {"q", "\x03"}:
            with self._lock:
                if self._phase not in {"armed", "returning", "fault"}:
                    self._request_return_locked("mocap_quit")
                elif self._phase == "returning":
                    self._return_deadline = time.monotonic() + 5.0
                else:
                    self._closed = True

    def _build_targets(self, frame: AlignedHandFrame) -> ArmTargetBatch:
        poses: dict[str, np.ndarray] = {}
        for side in ("left", "right"):
            home = np.concatenate(
                (self._config.init_pos[side], self._config.init_quat[side])
            )
            if side not in self._references:
                poses[side] = home
                continue
            current = np.asarray(frame.hands[side]["wrist_pose"], dtype=np.float64)
            reference = self._references[side]
            # Both poses are source/world rotations.  The frozen-reference
            # delta is current * reference^-1; transform that delta into the
            # arm Base frame before applying it to the robot Home rotation.
            delta_world = (
                Rotation.from_quat(current[3:])
                * Rotation.from_quat(reference[3:]).inv()
            )
            world_to_base = (
                self._config.get_world_to_chest_rotation(side)
                @ self._config.mocap_to_robot
            )
            delta_base = Rotation.from_matrix(
                world_to_base
                @ delta_world.as_matrix()
                @ world_to_base.T
            )
            position = home[:3] + world_to_base @ (current[:3] - reference[:3])
            orientation = (
                delta_base * Rotation.from_quat(home[3:])
            ).as_quat()
            poses[side] = np.concatenate((position, orientation))
        return self._mapper.map_absolute_tcp_poses(poses["left"], poses["right"])

    def _real_capability_snapshot(self) -> tuple[bool, str | None]:
        if not self._real_mode:
            return False, "real mode not requested"
        if self._real_capability is None:
            return False, "typed real capability input missing"
        try:
            capability = parse_real_capability(self._real_capability)
        except Exception as exc:
            return False, str(exc)
        if not capability.admitted:
            return False, "real capability predicates are not admitted"
        if float(capability.speed) != self._speed:
            return False, "real capability speed does not match configured speed"
        if float(capability.yaw_deg) != self._yaw_deg:
            return False, "real capability yaw does not match configured yaw"
        return True, None

    def _tick(self, now: float | None = None) -> None:
        now = self._clock() if now is None else float(now)
        self._session_client.poll()
        with self._lock:
            frame = self._latest
            fresh = frame is not None and now - self._received_at <= self._stale_timeout_s
            if self._phase == "returning":
                if self._session_client.return_completion_fresh:
                    self._complete_return_locked()
                elif now >= self._return_deadline:
                    self._return_timed_out = True
                    self._last_error = "coordinator return completion timeout"
                    self._phase = "fault"
                self._publish_status_locked()
                return
            if self._phase == "start_pending":
                if self._real_mode and not self._real_capability_snapshot()[0]:
                    self._request_return_locked("mocap_real_capability_lost")
                elif not fresh:
                    self._request_return_locked("mocap_stale_before_start")
                elif self._session_client.start_authorized:
                    self._mapper.initialize(
                        self._frame_from_references(frame)
                    )
                    self._phase = "teleop"
                elif self._session_client.pending_intent_sequence is None:
                    self._references.clear()
                    self._phase = "armed"
                self._publish_status_locked()
                return
            if self._phase != "teleop":
                self._publish_status_locked()
                return
            # Re-evaluate all real-admission inputs on every active tick,
            # including a provider that changes speed/yaw/deadman state.
            if self._real_mode and not self._real_capability_snapshot()[0]:
                self._request_return_locked("mocap_real_capability_lost")
                self._publish_status_locked()
                return
            if not fresh or any(not frame.hands[side]["valid"] for side in self._active_sides):
                self._request_return_locked("mocap_stale_or_invalid_side")
                self._publish_status_locked()
                return
            try:
                targets = self._build_targets(frame)
                for side in self._active_sides:
                    pose = targets.left_pose if side == "left" else targets.right_pose
                    elbow = (
                        targets.left_default_elbow_direction
                        if side == "left"
                        else targets.right_default_elbow_direction
                    )
                    self._publisher.publish_arm_target(
                        side=side,
                        position_m=pose[:3],
                        orientation_xyzw=pose[3:],
                        elbow_reference_direction=elbow,
                        source_timestamp_ns=frame.source_timestamp_ns,
                    )
                    points = TargetPublisher.relative_hand_keypoints(
                        frame.hands[side]["keypoints_world_m"]
                    )
                    self._publisher.publish_hand_target(
                        side=side,
                        keypoints_m=points,
                        source_timestamp_ns=frame.source_timestamp_ns,
                    )
            except (ValueError, ProtocolError) as exc:
                self._last_error = str(exc)
                self._request_return_locked("mocap_target_validation_error")
            self._publish_status_locked()

    def _frame_from_references(self, frame: AlignedHandFrame) -> WristPoseFrame:
        poses = {
            side: self._references.get(
                side,
                np.concatenate(
                    (self._config.init_pos[side], self._config.init_quat[side])
                ),
            )
            for side in ("left", "right")
        }
        return WristPoseFrame.from_poses(poses["left"], poses["right"])

    def _publish_status_locked(self) -> None:
        real_ok, real_reason = self._real_capability_snapshot()
        self._real_capability_error = real_reason
        self._publisher.publish_source_status(
            component_id="mocap_live",
            phase=self._phase,
            ready=self._session_client.startup_ready and self._last_error is None,
            healthy=self._last_error is None and self._phase != "fault",
            capabilities=["simulation"] + (["real"] if real_ok else []),
            error=self._last_error,
            diagnostics={
                "stream_instance_id": self._stream_instance_id,
                "stream_sequence": self._stream_sequence,
                "frame_valid": None if self._latest is None else self._latest.frame_valid,
                "active_sides": list(self._active_sides),
                "watchdog_s": self._stale_timeout_s,
                "real_capability_error": real_reason,
            },
        )
    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop_event.set()
        self._frame_sub.close()
        self._publisher.close()
        self._session_client.close()
        self._session.close()


def _to_raw_payload(frame: AlignedHandFrame) -> dict[str, Any]:
    return {
        "stream_instance_id": frame.stream_instance_id,
        "stream_sequence": frame.stream_sequence,
        "router_zid": frame.router_zid,
        "time_ns": frame.source_timestamp_ns,
        "frame_index": frame.frame_index,
        "frame_valid": frame.frame_valid,
        "hands": frame.hands,
    }


__all__ = ["AlignedHandFrame", "MocapLiveNode", "parse_aligned_hands", "main"]
def main(argv: list[str] | None = None) -> int:
    import os

    args = parse_cli_args(argv)
    overrides = dict(parse_param_override(item) for item in args.param)
    params = load_node_config(args.config, "mocap_live", {"rate": 60.0, "stale_timeout": _FRAME_STALE_S}, overrides)
    instance_id = os.environ.get("TIANJI_COMPONENT_INSTANCE_ID")
    real_mode = os.environ.get("TIANJI_REQUIRED_CAPABILITY", "simulation") == "real"
    if real_mode:
        params["real_mode"] = True
    real_capability = None
    if real_mode:
        from ...executors.marvin.preflight import trusted_real_capability
        real_capability = trusted_real_capability
    router_zid = os.environ.get("TIANJI_ROUTER_ZID")
    coordinator_id = os.environ.get("TIANJI_COORDINATOR_INSTANCE_ID")
    endpoint = os.environ.get("TIANJI_ROUTER_ENDPOINT", "tcp/127.0.0.1:7447")
    if not instance_id or not router_zid or not coordinator_id:
        raise RuntimeError(
            "TIANJI_COMPONENT_INSTANCE_ID, TIANJI_ROUTER_ZID and "
            "TIANJI_COORDINATOR_INSTANCE_ID are required"
        )
    session = open_session(endpoint)
    require_single_router(session, router_zid)
    node = MocapLiveNode(
        session,
        params,
        publisher_instance_id=instance_id,
        router_zid=router_zid,
        coordinator_instance_id=coordinator_id,
        real_capability=real_capability,
    )
    keyboard_thread = threading.Thread(
        target=raw_keyboard,
        args=(node._on_key, node._stop_event),
        daemon=True,
    )
    keyboard_thread.start()
    try:
        while not node._closed:
            node._tick()
            time.sleep(1.0 / node._rate)
    except KeyboardInterrupt:
        return 0
    finally:
        node.close()
    return 0
