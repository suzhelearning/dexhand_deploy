"""Real Regrind policy source: Motive/Wuji feedback to Tianji TCP and Wuji targets."""
from __future__ import annotations

import argparse
import hashlib
import json
import logging
import os
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation
import torch

from ..executors.wuji_hand2.config import WujiHandConfig
from ..protocol import topics
from ..protocol.messages import ComponentStatus, HAND_JOINT_NAMES, HandJointState
from ..regrind_policy import action_to_targets, build_observation, infer, load_actor, load_reference
from ..zenoh_util import (
    ZenohJsonSub,
    ZenohPub,
    declare_component_liveliness,
    load_node_config,
    load_tianji_config,
    open_session,
    require_single_router,
)
from .common.keyboard import X11KeyState, raw_keyboard
from .common.real_admission import parse_real_capability
from .common.session_client import SessionClient
from .common.target_conditioner import TargetConditioner, TargetConditioningSettings
from .common.target_publisher import SequenceAllocator, TargetPublisher
from .mocap.h5 import compose_pose, invert_pose
from .mocap.regrind import (
    HAMMER_RIGID_TO_OBJECT,
    MARKER_TO_MOUNT,
    MOUNT_TO_WRIST,
    WRIST_RIGID_TO_MARKER,
    RegrindMotiveSample,
    RegrindMotiveTracker,
)


_LOG = logging.getLogger("regrind_policy")
DEFAULT_PARAMETERS = {
    "checkpoint_sha256": "c2373a187e823209204abfd2a7c07987aa7cd3c674029c9ca0c20dfbac13e211",
    "reference_sha256": "aa73644d97d7de2a8f7a7453bec4a5f103ea7a1003f8bf16f13906fcc8e4f5ad",
    "rate": 50.0,
    "motive_stale_s": 0.04,
    "hand_stale_s": 0.04,
    "maximum_input_skew_s": 0.02,
    "hammer_start_position_tolerance_m": 0.01,
    "hammer_start_orientation_tolerance_deg": 5.0,
    "hand_maximum_step_rad": 0.01,
    "hand_tracking_error_rad": 0.25,
    "hand_tracking_error_duration_s": 0.1,
    "hand_instant_error_rad": 0.5,
    "return_timeout_s": 5.0,
    "hammer_rigid_to_object_translation_m": HAMMER_RIGID_TO_OBJECT[:3].tolist(),
    "hammer_rigid_to_object_quaternion_xyzw": HAMMER_RIGID_TO_OBJECT[3:].tolist(),
    "right_rigid_to_marker_mocap_translation_m": WRIST_RIGID_TO_MARKER[:3].tolist(),
    "right_rigid_to_marker_mocap_quaternion_xyzw": WRIST_RIGID_TO_MARKER[3:].tolist(),
    "right_marker_to_mount_translation_m": MARKER_TO_MOUNT[:3].tolist(),
    "right_marker_to_mount_quaternion_xyzw": MARKER_TO_MOUNT[3:].tolist(),
    "right_tcp_to_mount_translation_m": [0.0, 0.0, 0.008],
    "right_tcp_to_mount_quaternion_xyzw": [0.7071067811865476, 0.7071067811865476, 0.0, 0.0],
    # First real pass: 10% of the established H5 workspace dynamics.
    "workspace_relative_radii_m": [0.42, 0.38, 0.38],
    "workspace_soft_zone_ratio": 0.90,
    "maximum_linear_speed_m_s": 0.036,
    "maximum_angular_speed_rad_s": 0.155,
    "maximum_linear_acceleration_m_s2": 0.35,
    "maximum_angular_acceleration_rad_s2": 0.9,
    "right_default_elbow_direction": [0.45638698, 0.74604902, -0.48489358],
}


def _configured_pose(params: dict[str, Any], prefix: str) -> np.ndarray:
    pose = np.asarray(
        params[f"{prefix}_translation_m"] + params[f"{prefix}_quaternion_xyzw"],
        dtype=np.float64,
    )
    if pose.shape != (7,) or not np.isfinite(pose).all() or np.linalg.norm(pose[3:]) < 1e-8:
        raise ValueError(f"{prefix} must be a finite xyz+xyzw pose")
    pose[3:] /= np.linalg.norm(pose[3:])
    return pose


def _pose_error(actual_xyzw: np.ndarray, expected_xyzw: np.ndarray) -> tuple[float, float]:
    position = float(np.linalg.norm(actual_xyzw[:3] - expected_xyzw[:3]))
    rotation = Rotation.from_quat(expected_xyzw[3:]).inv() * Rotation.from_quat(actual_xyzw[3:])
    return position, float(np.rad2deg(np.linalg.norm(rotation.as_rotvec())))


class RegrindPolicyNode:
    def __init__(
        self,
        session: Any,
        params: dict[str, Any],
        *,
        model: Path,
        reference: Path,
        publisher_instance_id: str,
        router_zid: str,
        coordinator_instance_id: str,
        hand_executor_instance_id: str,
        real_capability: Any,
    ) -> None:
        self._params = params
        self._rate = float(params["rate"])
        if not np.isclose(self._rate, 50.0):
            raise ValueError("Regrind policy rate must remain training-exact at 50 Hz")
        capability = parse_real_capability(real_capability)
        if not capability.admitted or float(capability.speed) != 1.0 or float(capability.yaw_deg) != 0.0:
            raise ValueError("real Regrind requires admitted --speed 1 and yaw 0")
        for path, key in ((model, "checkpoint_sha256"), (reference, "reference_sha256")):
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            if digest != str(params[key]).lower():
                raise ValueError(f"{path.name} SHA256 does not match the authorized Regrind artifact")
        torch.set_num_threads(1)
        self._actor, self._mean, self._variance, iteration = load_actor(model)
        self._reference = load_reference(reference)
        self._router_zid = router_zid
        self._hand_executor_instance_id = hand_executor_instance_id
        self._real_capability = real_capability

        rigid_to_wrist = compose_pose(
            compose_pose(
                _configured_pose(params, "right_rigid_to_marker_mocap"),
                _configured_pose(params, "right_marker_to_mount"),
            ),
            MOUNT_TO_WRIST,
        )
        self._motive = RegrindMotiveTracker(
            session,
            rigid_to_wrist=rigid_to_wrist,
            rigid_to_object=_configured_pose(params, "hammer_rigid_to_object"),
        )

        config = load_tianji_config()
        self._home_tcp = np.concatenate((config.init_pos["right"], config.init_quat["right"]))
        tcp_to_wrist = compose_pose(_configured_pose(params, "right_tcp_to_mount"), MOUNT_TO_WRIST)
        home_wrist = compose_pose(self._home_tcp, tcp_to_wrist)
        reference_wrist_zero = np.concatenate(
            (self._reference.wrist_pos[0], np.roll(self._reference.wrist_quat_wxyz[0], -1))
        )
        self._robot_from_training = compose_pose(home_wrist, invert_pose(reference_wrist_zero))
        self._wrist_to_tcp = invert_pose(tcp_to_wrist)
        self._conditioner = TargetConditioner(
            self._home_tcp[:3],
            self._home_tcp[3:],
            TargetConditioningSettings(
                rate_hz=self._rate,
                translation_gain=np.ones(3),
                rotation_gain=1.0,
                workspace_relative_radii_m=params["workspace_relative_radii_m"],
                workspace_soft_zone_ratio=float(params["workspace_soft_zone_ratio"]),
                maximum_linear_speed_m_s=float(params["maximum_linear_speed_m_s"]),
                maximum_angular_speed_rad_s=float(params["maximum_angular_speed_rad_s"]),
                maximum_linear_acceleration_m_s2=float(params["maximum_linear_acceleration_m_s2"]),
                maximum_angular_acceleration_rad_s2=float(params["maximum_angular_acceleration_rad_s2"]),
            ),
        )
        elbow = np.asarray(params["right_default_elbow_direction"], dtype=np.float64)
        self._elbow = elbow / np.linalg.norm(elbow)
        self._hand_config = WujiHandConfig.load()

        allocator = SequenceAllocator()
        self._publisher = TargetPublisher(
            session,
            source="regrind_policy",
            publisher_instance_id=publisher_instance_id,
            router_zid=router_zid,
            allocator=allocator,
        )
        self._hand_token = declare_component_liveliness(
            session,
            role="producer/hand",
            logical_id="regrind_policy",
            instance_id=publisher_instance_id,
        )
        self._hand_status_pub = ZenohPub(session, topics.PRODUCER_STATUS)
        self._session_client = SessionClient(
            session,
            source="regrind_policy",
            publisher_instance_id=publisher_instance_id,
            router_zid=router_zid,
            expected_coordinator_instance_id=coordinator_instance_id,
            allocator=allocator,
        )
        self._session_client.start()
        self._lock = threading.RLock()
        self._hand_state: HandJointState | None = None
        self._hand_received_at = 0.0
        self._hand_sequence = -1
        self._input_error: str | None = None
        self._hand_state_sub = ZenohJsonSub(session, topics.hand_state("right"), self._on_hand_state)
        self._phase = "armed"
        self._phase_started = time.monotonic()
        self._return_deadline = 0.0
        self._exit_after_return = False
        self._quit = False
        self._last_error: str | None = None
        self._frame_index = 0
        self._training_from_motive: np.ndarray | None = None
        self._previous_wrist_pos: np.ndarray | None = None
        self._previous_wrist_quat: np.ndarray | None = None
        self._previous_joints: np.ndarray | None = None
        self._last_action = np.zeros(26, dtype=np.float64)
        self._cached_tcp: np.ndarray | None = None
        self._cached_joints: np.ndarray | None = None
        self._last_hand_target: np.ndarray | None = None
        self._tracking_error_since: float | None = None
        self._deadman_pressed = False
        self._deadman_error: str | None = None
        try:
            self._deadman: X11KeyState | None = X11KeyState(("Return", "KP_Enter"))
        except RuntimeError as exc:
            self._deadman = None
            self._deadman_error = str(exc)
        self._stop_event = threading.Event()
        self._keyboard_thread = threading.Thread(
            target=raw_keyboard,
            args=(self._on_key, self._stop_event),
            daemon=True,
        )
        self._keyboard_thread.start()
        _LOG.warning(
            "REAL Regrind loaded: checkpoint iteration=%s, frames=%s. "
            "Place wrist/hammer at take001 start, release Enter, then press s; hold Enter to execute; s returns Home; q returns and exits.",
            iteration,
            self._reference.frame_count,
        )

    def _on_hand_state(self, payload: Any) -> None:
        try:
            state = HandJointState.from_dict(payload)
            if (
                state.side != "right"
                or state.executor != "wuji_hand2"
                or state.router_zid != self._router_zid
                or state.publisher_instance_id != self._hand_executor_instance_id
            ):
                raise ValueError("right Wuji hand state authority mismatch")
            positions = np.asarray(state.position_rad, dtype=np.float64)
            self._hand_config.validate_positions(positions)
        except (TypeError, ValueError) as exc:
            with self._lock:
                self._input_error = str(exc)
            return
        with self._lock:
            if state.sequence <= self._hand_sequence:
                self._input_error = "right Wuji hand state sequence rollback"
                return
            self._hand_state = state
            self._hand_received_at = time.monotonic()
            self._hand_sequence = state.sequence

    def _read_deadman(self) -> bool:
        if self._deadman is None:
            return False
        try:
            pressed = bool(self._deadman.is_pressed())
        except Exception as exc:
            self._deadman_error = str(exc)
            return False
        self._deadman_pressed = pressed
        return pressed

    def _real_admitted(self) -> tuple[bool, str | None]:
        try:
            capability = parse_real_capability(self._real_capability)
        except Exception as exc:
            return False, str(exc)
        if not capability.admitted or float(capability.speed) != 1.0 or float(capability.yaw_deg) != 0.0:
            return False, "real capability no longer admits speed=1/yaw=0"
        if self._deadman is None or self._deadman_error:
            return False, self._deadman_error or "deadman unavailable"
        return True, None

    def _fresh_inputs(self, now: float) -> tuple[RegrindMotiveSample | None, np.ndarray | None, str | None]:
        sample = self._motive.latest()
        with self._lock:
            hand, hand_received, input_error = self._hand_state, self._hand_received_at, self._input_error
        if self._motive.error:
            return None, None, self._motive.error
        if input_error:
            return None, None, input_error
        if sample is None:
            return None, None, "waiting for valid Motive wrist+hammer"
        if now - sample.received_at > float(self._params["motive_stale_s"]):
            return None, None, "Motive wrist+hammer stale"
        if hand is None or now - hand_received > float(self._params["hand_stale_s"]):
            return None, None, "Wuji hand feedback stale"
        if abs(sample.received_at - hand_received) > float(self._params["maximum_input_skew_s"]):
            return None, None, "Motive/Wuji feedback skew too large"
        return sample, np.asarray(hand.position_rad, dtype=np.float64), None

    def _on_key(self, key: str) -> None:
        if key in ("q", "\x03"):
            with self._lock:
                if self._phase == "armed" and self._session_client.at_home:
                    self._quit = True
                    self._stop_event.set()
                else:
                    self._begin_return("operator quit", exit_after_return=True)
            return
        if key != "s":
            return
        with self._lock:
            if self._phase == "armed":
                self._request_start()
            elif self._phase in {"start_pending", "running"}:
                self._begin_return("operator return", exit_after_return=False)

    def _request_start(self) -> None:
        now = time.monotonic()
        admitted, reason = self._real_admitted()
        sample, joints, input_reason = self._fresh_inputs(now)
        if not admitted or not self._session_client.startup_ready or not self._session_client.at_home:
            _LOG.error("start rejected: %s", reason or "coordinator/Home snapshot not ready")
            return
        if self._read_deadman():
            _LOG.error("start rejected: release Enter before pressing s")
            return
        if sample is None or joints is None:
            _LOG.error("start rejected: %s", input_reason)
            return
        if not self._hand_config.at_zero(joints):
            _LOG.error("start rejected: Wuji hand is not at zero")
            return
        reference_wrist = np.concatenate(
            (self._reference.wrist_pos[0], np.roll(self._reference.wrist_quat_wxyz[0], -1))
        )
        training_from_motive = compose_pose(reference_wrist, invert_pose(sample.wrist_xyzw))
        aligned_hammer = compose_pose(training_from_motive, sample.hammer_xyzw)
        reference_hammer = np.concatenate(
            (self._reference.object_pos[0], np.roll(self._reference.object_quat_wxyz[0], -1))
        )
        position_error, orientation_error = _pose_error(aligned_hammer, reference_hammer)
        if (
            position_error > float(self._params["hammer_start_position_tolerance_m"])
            or orientation_error > float(self._params["hammer_start_orientation_tolerance_deg"])
        ):
            _LOG.error(
                "start rejected: hammer relative to wrist differs from take001 (%.1f mm, %.1f deg)",
                position_error * 1000.0,
                orientation_error,
            )
            return
        self._training_from_motive = training_from_motive
        self._previous_wrist_pos = self._reference.wrist_pos[0].copy()
        self._previous_wrist_quat = self._reference.wrist_quat_wxyz[0].copy()
        self._previous_joints = joints.copy()
        self._last_action.fill(0.0)
        self._cached_tcp = None
        self._cached_joints = None
        self._last_hand_target = None
        self._tracking_error_since = None
        self._frame_index = 0
        self._last_error = None
        self._conditioner.reset()
        self._session_client.request_start("regrind_real_s")
        self._phase = "start_pending"
        self._phase_started = now
        _LOG.warning("start requested; hammer preflight %.1f mm / %.1f deg passed", position_error * 1000.0, orientation_error)

    def _begin_return(self, reason: str, *, exit_after_return: bool) -> None:
        if self._phase == "returning":
            self._exit_after_return = self._exit_after_return or exit_after_return
            return
        self._cached_tcp = None
        self._cached_joints = None
        self._exit_after_return = exit_after_return
        self._return_deadline = time.monotonic() + float(self._params["return_timeout_s"])
        try:
            self._session_client.request_return(reason, timeout_s=1.0)
        except (RuntimeError, ValueError) as exc:
            self._last_error = str(exc)
        self._phase = "returning"
        self._phase_started = time.monotonic()
        _LOG.warning("bounded return requested: %s", reason)

    def _check_hand_tracking(self, joints: np.ndarray, now: float) -> str | None:
        if self._last_hand_target is None:
            return None
        error = float(np.max(np.abs(joints - self._last_hand_target)))
        if error > float(self._params["hand_instant_error_rad"]):
            return f"Wuji instantaneous tracking error {error:.3f} rad"
        if error > float(self._params["hand_tracking_error_rad"]):
            self._tracking_error_since = self._tracking_error_since or now
            if now - self._tracking_error_since > float(self._params["hand_tracking_error_duration_s"]):
                return f"Wuji sustained tracking error {error:.3f} rad"
        else:
            self._tracking_error_since = None
        return None

    def _infer_target(self, sample: RegrindMotiveSample, joints: np.ndarray) -> None:
        assert self._training_from_motive is not None
        assert self._previous_wrist_pos is not None
        assert self._previous_wrist_quat is not None
        assert self._previous_joints is not None
        index = self._frame_index
        wrist = compose_pose(self._training_from_motive, sample.wrist_xyzw)
        hammer = compose_pose(self._training_from_motive, sample.hammer_xyzw)
        wrist_quat = np.roll(wrist[3:], 1)
        hammer_quat = np.roll(hammer[3:], 1)
        observation = build_observation(
            object_pos=hammer[:3],
            object_quat_wxyz=hammer_quat,
            previous_wrist_pos=self._previous_wrist_pos,
            wrist_pos=wrist[:3],
            previous_wrist_quat_wxyz=self._previous_wrist_quat,
            wrist_quat_wxyz=wrist_quat,
            previous_joints=self._previous_joints,
            joints=joints,
            last_action=self._last_action,
            phase=index / (self._reference.frame_count - 1),
            base_wrist_pos=self._reference.wrist_pos[index],
            base_wrist_quat_wxyz=self._reference.wrist_quat_wxyz[index],
            base_joints=self._reference.joints[index],
        )
        raw_action = infer(self._actor, self._mean, self._variance, observation)
        target_pos, target_quat, target_joints = action_to_targets(
            raw_action,
            self._reference.wrist_pos[index],
            self._reference.wrist_quat_wxyz[index],
            self._reference.joints[index],
        )
        target_joints = np.clip(target_joints, self._hand_config.lower_limits_rad, self._hand_config.upper_limits_rad)
        maximum_step = float(self._params["hand_maximum_step_rad"])
        target_joints = joints + np.clip(target_joints - joints, -maximum_step, maximum_step)
        wrist_training = np.concatenate((target_pos, np.roll(target_quat, -1)))
        tcp_robot = compose_pose(compose_pose(self._robot_from_training, wrist_training), self._wrist_to_tcp)
        tcp_position, tcp_quaternion, _ = self._conditioner.condition(tcp_robot[:3], tcp_robot[3:])
        self._cached_tcp = np.concatenate((tcp_position, tcp_quaternion))
        self._cached_joints = target_joints
        self._last_hand_target = target_joints.copy()
        self._last_action = np.clip(raw_action, -1.0, 1.0)
        self._previous_wrist_pos = wrist[:3].copy()
        self._previous_wrist_quat = wrist_quat.copy()
        self._previous_joints = joints.copy()
        self._frame_index += 1

    def _publish_cached(self) -> None:
        if self._cached_tcp is None or self._cached_joints is None:
            return
        self._publisher.publish_arm_target(
            side="right",
            position_m=self._cached_tcp[:3],
            orientation_xyzw=self._cached_tcp[3:],
            elbow_reference_direction=self._elbow,
        )
        self._publisher.publish_hand_joint_command(
            side="right",
            names=HAND_JOINT_NAMES["right"],
            position_rad=self._cached_joints,
            producer="regrind_policy",
        )

    def _publish_status(self, now: float) -> None:
        admitted, admission_error = self._real_admitted()
        sample, _joints, input_error = self._fresh_inputs(now)
        healthy = self._last_error is None and self._phase != "fault" and admitted
        ready = (
            healthy
            and self._phase in {"armed", "start_pending", "running"}
            and sample is not None
            and input_error is None
            and self._session_client.startup_ready
        )
        diagnostics = {
            "mode": "real_closed_loop",
            "frame_index": self._frame_index,
            "frame_count": self._reference.frame_count,
            "deadman_pressed": self._deadman_pressed,
            "motive_age_ms": None if sample is None else (now - sample.received_at) * 1000.0,
            "input_error": input_error,
            "admission_error": admission_error,
        }
        self._publisher.publish_source_status(
            component_id="regrind_policy",
            phase=self._phase,
            ready=ready,
            healthy=healthy,
            capabilities=["real"] if admitted else [],
            error=self._last_error or admission_error,
            diagnostics=diagnostics,
        )
        hand_status = ComponentStatus(
            1,
            self._publisher.sequence,
            time.monotonic_ns(),
            "producer_hand",
            "regrind_policy",
            self._phase,
            ready,
            healthy,
            ["real"] if admitted else [],
            self._last_error or admission_error,
            diagnostics,
            self._publisher.publisher_instance_id,
            self._publisher.router_zid,
        )
        self._hand_status_pub.put_json(hand_status.to_dict())

    def _tick(self, now: float) -> bool:
        self._session_client.poll()
        if self._phase == "armed":
            self._read_deadman()
            return not self._quit
        if self._phase == "start_pending":
            if self._session_client.start_authorized:
                self._phase = "running"
                self._phase_started = now
                _LOG.warning("start authorized; hold Enter to execute policy")
            elif self._session_client.pending_intent_sequence is None:
                self._phase = "armed"
                _LOG.error("coordinator rejected start: %s", self._session_client.state.reason if self._session_client.state else "unknown")
            return True
        if self._phase == "returning":
            if self._session_client.return_completion_fresh:
                if self._exit_after_return:
                    return False
                self._phase = "armed"
                self._last_error = None
                self._training_from_motive = None
                _LOG.warning("arm Home and Wuji zero confirmed")
            elif now >= self._return_deadline:
                self._last_error = "return completion timeout"
                self._phase = "fault"
            return True
        if self._phase == "fault":
            return True

        admitted, reason = self._real_admitted()
        sample, joints, input_error = self._fresh_inputs(now)
        if not admitted or sample is None or joints is None:
            self._last_error = reason or input_error
            self._begin_return(self._last_error or "real input lost", exit_after_return=True)
            return True
        tracking_error = self._check_hand_tracking(joints, now)
        if tracking_error:
            self._last_error = tracking_error
            self._begin_return(tracking_error, exit_after_return=True)
            return True
        if not self._session_client.start_authorized:
            self._begin_return("coordinator left teleop", exit_after_return=True)
            return True
        if self._read_deadman():
            if self._frame_index >= self._reference.frame_count - 1:
                self._begin_return("Regrind reference complete", exit_after_return=True)
                return True
            self._infer_target(sample, joints)
        elif self._deadman_error:
            self._last_error = self._deadman_error
            self._begin_return("deadman read failed", exit_after_return=True)
            return True
        self._publish_cached()
        return True

    def run(self) -> int:
        period = 1.0 / self._rate
        next_tick = time.monotonic()
        next_status = next_tick
        while True:
            now = time.monotonic()
            if now >= next_tick:
                if not self._tick(now):
                    return 0
                next_tick += period
            if now >= next_status:
                self._publish_status(now)
                next_status += 0.2
            time.sleep(max(0.001, min(next_tick, next_status) - time.monotonic()))

    def close(self) -> None:
        self._stop_event.set()
        self._hand_state_sub.close()
        self._motive.close()
        self._hand_status_pub.close()
        if self._hand_token is not None:
            self._hand_token.undeclare()
        if self._deadman is not None:
            self._deadman.close()
        self._publisher.close()
        self._session_client.close()


def _hand_executor_instance() -> str:
    try:
        authorities = json.loads(os.environ["TIANJI_AUTHORITIES"])
        value = authorities["executor_hand"]["right"]["publisher_instance_id"]
    except (KeyError, TypeError, json.JSONDecodeError) as exc:
        raise RuntimeError("launcher did not provide right hand executor authority") from exc
    if not isinstance(value, str) or not value:
        raise RuntimeError("right hand executor authority is invalid")
    return value


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--config", default="")
    args = parser.parse_args(argv)
    if not args.model.is_file() or not args.reference.is_file():
        parser.error("--model and --reference must be existing files")
    if os.environ.get("TIANJI_REQUIRED_CAPABILITY") != "real":
        parser.error("regrind_policy source is real-only")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    params = load_node_config(args.config, "regrind_policy", DEFAULT_PARAMETERS, {})
    from ..executors.marvin.preflight import trusted_real_capability

    session = open_session(os.environ.get("TIANJI_ROUTER_ENDPOINT"))
    node = None
    try:
        instance = os.environ.get("TIANJI_COMPONENT_INSTANCE_ID")
        coordinator = os.environ.get("TIANJI_COORDINATOR_INSTANCE_ID")
        router = require_single_router(session, os.environ.get("TIANJI_ROUTER_ZID"))
        if not instance or not coordinator:
            raise RuntimeError("launcher must provide component and coordinator identities")
        node = RegrindPolicyNode(
            session,
            params,
            model=args.model,
            reference=args.reference,
            publisher_instance_id=instance,
            router_zid=router,
            coordinator_instance_id=coordinator,
            hand_executor_instance_id=_hand_executor_instance(),
            real_capability=trusted_real_capability,
        )
        return node.run()
    except KeyboardInterrupt:
        return 130
    finally:
        if node is not None:
            node.close()
        session.close()


if __name__ == "__main__":
    raise SystemExit(main())
