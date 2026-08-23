#!/usr/bin/env python3
"""Manus HDF5 右手腕轨迹的 Enter 保压安全回放节点。

端点契约：输入是 H5 Manus wrist；机器人端对应 wuji2 ``r_base``，
不是固定旋转 180° 后的 ``r_wrist``。Motive ``right_arm`` 经 GL/GO
到 marker_mocap，再由机械外参和 ``r_wrist→r_base`` 固定变换推导
``r_base`` Home。H5 wrist 同样转换到 ``r_base``，随后由
``r_base→TCP`` 固定外参送入 IK。

状态机：

    armed -> approaching（s 读取实时 marker；Enter 保压接近 frame0）
          -> ready（保持绝对 frame0，等待 r）
          -> replaying（r 后 Enter 保压推进后续帧）
          -> completed -> returning

HDF5 路径由 CLI 位置参数选择；左臂不发布目标，保持 Home。
"""

from __future__ import annotations

import argparse
import json
import logging
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from .controller_only_mapper import (
    ControllerOnlyTargets,
    ControllerOnlyTeleopMapper,
)
from .controller_only_trace import _assert_replay_graph_is_safe
from .mocap_h5 import (
    HAND_KEYPOINT_EDGES,
    HandPoseTrajectory,
    MocapRecording,
    compose_pose,
    invert_pose,
    load_mocap_h5,
    synthetic_reference_pose,
)
from .mocap_keyboard_step import HoldToRunClock
from .raw_keyboard import X11KeyState, raw_keyboard
from .target_conditioner import TargetConditioningSettings
from ..controller_frame import ControllerFrame
from ..zenoh_util import (
    LiveToken,
    ZenohJsonSub,
    ZenohPub,
    ZenohTextSub,
    key,
    load_node_config,
    load_tianji_config,
    open_session,
    parse_param_override,
    stamp_now,
)

_LOG = logging.getLogger("mocap_h5_replay")

AT_HOME_KEY = "pico_body_sim/at_home"
RETURN_COMPLETE_KEY = "pico_body_sim/return_complete"
SOLVED_POSE_KEY = "pico_body_sim/right_arm/solved_pose"
MOCAP_FRAME_KEY = "mocap/hands/frame"
RIGID_BODY_NAMES_KEY = "mocap/rigid_body_names"
FRAME_ZERO_SKELETON_KEY = "pico_body_sim/frame0_hand_skeleton"

_S_DEBOUNCE_S = 0.5
_SOLVED_STALE_S = 0.5
_MOTIVE_STALE_S = 0.5
_SKELETON_PREVIEW_INTERVAL_S = 0.2
_CREATE_DEADMAN = object()

# wuji2 URDF 固定关节：r_base -> r_wrist，原点重合、绕 X 旋转 180°。
# 其逆相同。H5/IK 回放的语义端点是 r_base，不是 r_wrist。
_WUJI2_WRIST_TO_BASE_POSE = np.array(
    [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    dtype=np.float64,
)

DEFAULT_PARAMETERS = {
    "min_cutoff": 1.2,
    "beta": 0.45,
    "translation_gain": [1.0, 1.0, 1.0],
    "rotation_gain": 1.0,
    "workspace_relative_radii_m": [0.42, 0.38, 0.38],
    "workspace_soft_zone_ratio": 0.90,
    "maximum_linear_speed_m_s": 0.36,
    "maximum_angular_speed_rad_s": 1.55,
    "maximum_linear_acceleration_m_s2": 3.5,
    "maximum_angular_acceleration_rad_s2": 9.0,
    # Motive raw right_arm rigid frame -> marker URDF marker_mocap frame。
    # Motive Visuals: GL [-3,-4,0] mm；GO Pitch/Yaw/Roll [2,-90,0] deg。
    "right_rigid_to_marker_mocap_translation_m": [-0.003, -0.004, 0.0],
    "right_rigid_to_marker_mocap_quaternion_xyzw": [
        0.0123407149398269, -0.7069990853988243,
        0.0123407149398269, 0.7069990853988243
    ],
    # right_arm marker 中心(G) -> wuji2 r_wrist(B)。SolidWorks
    # marker URDF 为 8mm 刚体；安装轴关系为 marker
    # +x→mount +z、+y→mount -y、+z→mount +x。
    "right_marker_to_wrist_translation_m": [0.0325, 0.00025, 0.003],
    "right_marker_to_wrist_quaternion_xyzw": [
        0.0, -0.7071067811865476, 0.0, 0.7071067811865476
    ],
    # Tianji TCP(T) -> wuji2 r_wrist(B)，含 8mm marker 刚体。
    "right_tcp_to_wrist_translation_m": [0.00025, 0.003, 0.0365],
    "right_tcp_to_wrist_quaternion_xyzw": [
        0.7071067811865476, 0.7071067811865476, 0.0, 0.0
    ],
    # H5 Manus wrist(H) -> wuji2 r_wrist(B)，两者原点都是人手 wrist。
    # 回放语义端点是 r_base。组合 r_wrist→r_base=Rx(π) 后，再绕 base
    # z +90° 补偿数据与 wuji2 手的朝向（实测数据目标绕 base z 偏 -90°）。
    # 最终 H→r_wrist 四元数（xyzw）：
    #   [0.10031515, 0.11726593, 0.70127022, 0.69599256]
    "right_h5_wrist_to_wuji2_wrist_translation_m": [0.0, 0.0, 0.0],
    "right_h5_wrist_to_wuji2_wrist_quaternion_xyzw": [
        0.10031515, 0.11726593, 0.70127022, 0.69599256
    ],
    "approach_position_tolerance_m": 0.002,
    "approach_orientation_tolerance_deg": 1.0,
    "approach_solved_position_tolerance_m": 0.005,
    "approach_solved_orientation_tolerance_deg": 2.0,
    "approach_stable_seconds": 0.25,
    "left_default_zsp_direction": [
        0.45638698,
        -0.74604902,
        -0.48489358,
    ],
    "right_default_zsp_direction": [
        0.45638698,
        0.74604902,
        -0.48489358,
    ],
}

_ACTIVE_PHASES = {"approaching", "ready", "replaying", "completed"}

def _pose_from_payload(payload: dict[str, Any]) -> np.ndarray | None:
    position = payload.get("position")
    orientation = payload.get("orientation")
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        return None
    try:
        values = np.asarray(
            [position[axis] for axis in ("x", "y", "z")]
            + [orientation[axis] for axis in ("x", "y", "z", "w")],
            dtype=np.float64,
        )
    except (KeyError, TypeError, ValueError):
        return None
    if values.shape != (7,) or not np.isfinite(values).all():
        return None
    quaternion_norm = float(np.linalg.norm(values[3:7]))
    if quaternion_norm < 1.0e-8:
        return None
    values[3:7] /= quaternion_norm
    return values


def _rotation_error_rad(first_xyzw: np.ndarray, second_xyzw: np.ndarray) -> float:
    delta = Rotation.from_quat(first_xyzw).inv() * Rotation.from_quat(
        second_xyzw
    )
    return float(np.linalg.norm(delta.as_rotvec()))

def _rotate_points_yaw(points: np.ndarray, yaw_deg: float) -> np.ndarray:
    """绕 Motive +Y 旋转世界系点，与手腕轨迹 yaw 标定一致。"""
    values = np.asarray(points, dtype=np.float64)
    if values.shape != (21, 3) or not np.isfinite(values).all():
        raise ValueError("frame0 keypoints 必须是有限 (21,3) 数组")
    rotation = Rotation.from_rotvec(
        np.array([0.0, np.deg2rad(float(yaw_deg)), 0.0])
    ).as_matrix()
    return values @ rotation.T

def _configured_pose(params: dict[str, Any], prefix: str) -> np.ndarray:
    position = np.asarray(
        params[f"{prefix}_translation_m"], dtype=np.float64
    )
    quaternion = np.asarray(
        params[f"{prefix}_quaternion_xyzw"], dtype=np.float64
    )
    if position.shape != (3,) or quaternion.shape != (4,):
        raise ValueError(f"{prefix} 必须包含 3 维平移和 4 维 xyzw 四元数")
    return compose_pose(
        np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
        np.concatenate((position, quaternion)),
    )


class MocapH5ReplayNode:
    """右臂 H5 轨迹状态机与 Zenoh 发布驱动。"""

    def __init__(
        self,
        session,
        params: dict[str, Any],
        recording: MocapRecording,
        *,
        right_rigid_id: int | str = "right_arm",
        speed: float = 1.0,
        yaw_deg: float = 0.0,
        rate: float = 60.0,
        deadman: X11KeyState | None | object = _CREATE_DEADMAN,
        start_keyboard: bool = True,
    ) -> None:
        if not np.isfinite(speed) or speed <= 0.0:
            raise ValueError("speed 必须为正有限数值")
        if not np.isfinite(rate) or rate <= 0.0:
            raise ValueError("rate 必须为正有限数值")
        if isinstance(right_rigid_id, int):
            if right_rigid_id <= 0:
                raise ValueError("right_rigid_id 必须是正整数或刚体名")
        elif not isinstance(right_rigid_id, str) or not right_rigid_id.strip():
            raise ValueError("right_rigid_id 必须是正整数或刚体名")
        self._session = session
        self._recording = recording
        self._speed = float(speed)
        self._yaw_deg = float(yaw_deg)
        self._rate = float(rate)
        self._right_rigid_id = (
            int(right_rigid_id)
            if isinstance(right_rigid_id, int)
            else str(right_rigid_id).strip()
        )
        self._trajectory = HandPoseTrajectory(
            recording,
            side="right",
            yaw_deg=yaw_deg,
        )
        self._frame_zero_pose = self._trajectory.pose_at_frame(0)
        self._frame_zero_source_index = self._trajectory.start_frame_index
        self._frame_zero_keypoints = _rotate_points_yaw(
            recording.hands["right"].keypoints_world[
                self._frame_zero_source_index
            ],
            yaw_deg,
        )
        h5_wrist_to_wuji2_wrist = _configured_pose(
            params, "right_h5_wrist_to_wuji2_wrist"
        )
        marker_to_wrist = _configured_pose(
            params, "right_marker_to_wrist"
        )
        tcp_to_wrist = _configured_pose(
            params, "right_tcp_to_wrist"
        )
        self._h5_wrist_to_wuji2_base_pose = compose_pose(
            h5_wrist_to_wuji2_wrist,
            _WUJI2_WRIST_TO_BASE_POSE,
        )
        self._marker_to_base_pose = compose_pose(
            marker_to_wrist,
            _WUJI2_WRIST_TO_BASE_POSE,
        )
        self._tcp_to_base_pose = compose_pose(
            tcp_to_wrist,
            _WUJI2_WRIST_TO_BASE_POSE,
        )
        self._base_to_tcp_pose = invert_pose(self._tcp_to_base_pose)
        self._left_reference_pose = synthetic_reference_pose()
        self._rigid_to_marker_mocap_pose = _configured_pose(
            params, "right_rigid_to_marker_mocap"
        )

        settings = TargetConditioningSettings(
            rate_hz=rate,
            translation_gain=params["translation_gain"],
            rotation_gain=float(params["rotation_gain"]),
            workspace_relative_radii_m=params[
                "workspace_relative_radii_m"
            ],
            workspace_soft_zone_ratio=float(
                params["workspace_soft_zone_ratio"]
            ),
            maximum_linear_speed_m_s=float(
                params["maximum_linear_speed_m_s"]
            ),
            maximum_angular_speed_rad_s=float(
                params["maximum_angular_speed_rad_s"]
            ),
            maximum_linear_acceleration_m_s2=float(
                params["maximum_linear_acceleration_m_s2"]
            ),
            maximum_angular_acceleration_rad_s2=float(
                params["maximum_angular_acceleration_rad_s2"]
            ),
        )
        tianji_config = load_tianji_config()
        self._mapper = ControllerOnlyTeleopMapper(
            tianji_config,
            rate=rate,
            min_cutoff=float(params["min_cutoff"]),
            beta=float(params["beta"]),
            conditioning_settings=settings,
            default_zsp_directions={
                side: params[f"{side}_default_zsp_direction"]
                for side in ("left", "right")
            },
            # Motive 系(+X 左, +Z 前)与 PICO 系(+X 右, +Z 后)水平轴
            # 相差 180°，必须用独立的动捕同向映射，不能复用 pico_to_robot。
            input_to_robot=tianji_config.mocap_to_robot,
        )
        self._approach_position_tolerance_m = float(
            params["approach_position_tolerance_m"]
        )
        self._approach_orientation_tolerance_rad = float(
            np.deg2rad(params["approach_orientation_tolerance_deg"])
        )
        self._solved_position_tolerance_m = float(
            params["approach_solved_position_tolerance_m"]
        )
        self._solved_orientation_tolerance_rad = float(
            np.deg2rad(
                params["approach_solved_orientation_tolerance_deg"]
            )
        )
        stable_seconds = float(params["approach_stable_seconds"])
        for label, value in (
            ("approach_position_tolerance_m", self._approach_position_tolerance_m),
            (
                "approach_orientation_tolerance_deg",
                self._approach_orientation_tolerance_rad,
            ),
            (
                "approach_solved_position_tolerance_m",
                self._solved_position_tolerance_m,
            ),
            (
                "approach_solved_orientation_tolerance_deg",
                self._solved_orientation_tolerance_rad,
            ),
            ("approach_stable_seconds", stable_seconds),
        ):
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{label} 必须为正有限数值")
        self._required_stable_ticks = max(1, round(stable_seconds * rate))

        self._pose_pub = ZenohPub(
            session, key("/pico_body/right_arm_target_pose")
        )
        self._elbow_pub = ZenohPub(
            session, key("/pico_body/right_arm_elbow_direction")
        )
        self._state_pub = ZenohPub(session, key("/pico_body/teleop_state"))
        self._status_pub = ZenohPub(session, key("/pico_body/status"))
        self._frame_zero_skeleton_pub = ZenohPub(
            session, FRAME_ZERO_SKELETON_KEY
        )
        self._live = LiveToken(session, "mocap_h5_replay")

        self._lock = threading.RLock()
        self._latest_motive_frame: dict[str, Any] | None = None
        self._motive_received_at = 0.0
        self._rigid_body_names: dict[int, str] = {}
        self._right_rigid_home_pose: np.ndarray | None = None
        self._right_marker_home_pose: np.ndarray | None = None
        self._right_base_home_pose: np.ndarray | None = None
        self._virtual_tcp_home_pose: np.ndarray | None = None
        self._current_motive_base_target_pose: np.ndarray | None = None
        self._motive_frame_sub = ZenohJsonSub(
            session, MOCAP_FRAME_KEY, self._on_motive_frame
        )
        self._rigid_names_sub = ZenohJsonSub(
            session, RIGID_BODY_NAMES_KEY, self._on_rigid_body_names
        )
        self._at_home = False
        self._return_complete = False
        self._solved_pose: np.ndarray | None = None
        self._solved_received_at = 0.0
        self._at_home_sub = ZenohTextSub(
            session, AT_HOME_KEY, self._on_at_home_text
        )
        self._return_complete_sub = ZenohTextSub(
            session,
            RETURN_COMPLETE_KEY,
            self._on_return_complete_text,
        )
        self._solved_sub = ZenohJsonSub(
            session, SOLVED_POSE_KEY, self._on_solved_pose
        )
        self._session.get(
            AT_HOME_KEY,
            self._on_at_home_query,
            timeout=1.0,
        )

        self._deadman_error: str | None = None
        if deadman is _CREATE_DEADMAN:
            try:
                self._deadman: X11KeyState | None = X11KeyState(
                    ("Return", "KP_Enter")
                )
            except RuntimeError as exc:
                self._deadman = None
                self._deadman_error = str(exc)
                _LOG.error(
                    "H5 自动运动已禁用：无法可靠读取 Enter 按下/松开：%s",
                    exc,
                )
        else:
            self._deadman = deadman  # type: ignore[assignment]

        self._phase = "armed"
        self._phase_started = time.monotonic()
        self._exit_after_return = False
        self._quit = False
        self._last_error: str | None = None
        self._last_s_at = -float("inf")
        self._cached_targets: ControllerOnlyTargets | None = None
        self._approach_stable_ticks = 0
        self._source_complete = False
        self._final_stable_ticks = 0
        self._replay_clock: HoldToRunClock | None = None
        self._last_mapped_elapsed_s = -1.0
        self._current_source_frame = 0
        self._current_source_elapsed_s = 0.0
        self._deadman_pressed = False
        self._stop_event = threading.Event()
        self._last_skeleton_preview_at = -float("inf")
        self._last_skeleton_error: str | None = None
        self._keyboard_thread: threading.Thread | None = None
        if start_keyboard:
            self._keyboard_thread = threading.Thread(
                target=raw_keyboard,
                args=(self._on_key, self._stop_event),
                daemon=True,
            )
            self._keyboard_thread.start()

        _LOG.info(
            "H5 Manus wrist 相对轨迹已加载：%s；right 有效=%d/%d，"
            "插值丢帧=%d，时长=%.3fs，speed=%g，yaw=%g°；"
            "等待 Motive 刚体 %s 推导 wuji2 r_base Home",
            recording.path,
            int(recording.hands["right"].valid.sum()),
            recording.frame_count,
            self._trajectory.interpolated_frame_count,
            self._trajectory.duration_s,
            self._speed,
            self._yaw_deg,
            self._right_rigid_id,
        )
        _LOG.warning(
            "等待 IK Home、有效 right_arm marker 后按 s；节点推导 "
            "wuji2 r_base Home，并把 H5 wrist frame0 转换到 r_base。"
            "随后 Enter 保压接近，按 r 装载后续轨迹；活动阶段按 s "
            "回 Home，按 q 回 Home 后退出。"
        )

    def _frame(self, right_pose: np.ndarray) -> ControllerFrame:
        return ControllerFrame.from_poses(
            self._left_reference_pose,
            right_pose,
        )

    def _on_motive_frame(self, frame: dict[str, Any]) -> None:
        with self._lock:
            self._latest_motive_frame = frame
            self._motive_received_at = time.monotonic()

    def _on_rigid_body_names(self, mapping: dict[str, Any]) -> None:
        payload = mapping.get("names", mapping)
        if not isinstance(payload, dict):
            return
        try:
            names = {
                int(rigid_id): str(name)
                for rigid_id, name in payload.items()
            }
        except (TypeError, ValueError):
            return
        with self._lock:
            changed = names != self._rigid_body_names
            self._rigid_body_names = names
        # 发布端周期性重发名称映射（约 5s/次）；仅在实际变化时记录，
        # 避免刷屏。
        if changed:
            _LOG.info("Motive 刚体名映射已更新：%s", names)

    def _resolved_right_rigid_id(self) -> int | None:
        if isinstance(self._right_rigid_id, int):
            return self._right_rigid_id
        for rigid_id, name in self._rigid_body_names.items():
            if name == self._right_rigid_id:
                return rigid_id
        return None

    def _right_arm_pose(
        self, frame: dict[str, Any] | None
    ) -> np.ndarray | None:
        if not isinstance(frame, dict):
            return None
        rigid_id = self._resolved_right_rigid_id()
        if rigid_id is None:
            return None
        bodies = frame.get("rigid_bodies")
        if not isinstance(bodies, list):
            return None
        for body in bodies:
            if not isinstance(body, dict) or body.get("id") != rigid_id:
                continue
            if not body.get("tracking_valid", False):
                return None
            position = body.get("position")
            quaternion = body.get("quaternion_xyzw")
            if (
                not isinstance(position, (list, tuple))
                or len(position) != 3
                or not isinstance(quaternion, (list, tuple))
                or len(quaternion) != 4
            ):
                return None
            pose = np.asarray(
                list(position) + list(quaternion), dtype=np.float64
            )
            if not np.isfinite(pose).all():
                return None
            quaternion_norm = float(np.linalg.norm(pose[3:7]))
            if quaternion_norm < 1.0e-8:
                return None
            pose[3:7] /= quaternion_norm
            return pose
        return None

    def _fresh_right_arm_pose(
        self, now: float | None = None
    ) -> tuple[np.ndarray | None, str | None]:
        now = time.monotonic() if now is None else float(now)
        if self._latest_motive_frame is None:
            return None, f"尚未收到 {MOCAP_FRAME_KEY}"
        age_s = max(0.0, now - self._motive_received_at)
        if age_s > _MOTIVE_STALE_S:
            return None, f"Motive 帧已超时 {age_s:.2f}s"
        rigid_id = self._resolved_right_rigid_id()
        if rigid_id is None:
            return (
                None,
                f"刚体名 {self._right_rigid_id!r} 尚未从 "
                f"{RIGID_BODY_NAMES_KEY} 解析",
            )
        pose = self._right_arm_pose(self._latest_motive_frame)
        if pose is None:
            return None, f"right_arm 刚体 id={rigid_id} 无效或缺失"
        return pose, None

    def _base_pose_from_rigid(self, rigid_pose: np.ndarray) -> np.ndarray:
        marker_pose = compose_pose(
            rigid_pose, self._rigid_to_marker_mocap_pose
        )
        return compose_pose(marker_pose, self._marker_to_base_pose)

    def _frame_zero_skeleton_payload(
        self, base_home_pose: np.ndarray
    ) -> dict[str, Any]:
        target_base_motive = compose_pose(
            self._frame_zero_pose,
            self._h5_wrist_to_wuji2_base_pose,
        )
        return {
            "frame_id": "motive_world",
            "side": "right",
            "phase": self._phase,
            "frozen": self._phase != "armed",
            "source_frame_index": self._frame_zero_source_index,
            "h5_path": str(self._recording.path),
            "point_order": (
                "mediapipe: wrist, thumb1-4, index5-8, "
                "middle9-12, ring13-16, pinky17-20"
            ),
            "edges": [list(edge) for edge in HAND_KEYPOINT_EDGES],
            "points_motive_world": self._frame_zero_keypoints.tolist(),
            "frame0_manus_quat_xyzw": (
                self._frame_zero_pose[3:7].tolist()
            ),
            "home_wuji2_base_pose_motive": base_home_pose.tolist(),
            "frame0_wuji2_base_pose_motive": (
                target_base_motive.tolist()
            ),
            "transform_contract": (
                "world axes use mocap_to_robot; right_arm locates "
                "the shared r_base/r_wrist origin only"
            ),
        }

    def _publish_frame_zero_skeleton(
        self, base_home_pose: np.ndarray
    ) -> bool:
        try:
            payload = self._frame_zero_skeleton_payload(base_home_pose)
            self._frame_zero_skeleton_pub.put_json(payload)
        except Exception as exc:
            message = str(exc)
            if message != self._last_skeleton_error:
                _LOG.warning("frame0 手部关键点预览失败：%s", exc)
                self._last_skeleton_error = message
            return False
        self._last_skeleton_error = None
        return True

    def _publish_latest_frame_zero_skeleton(
        self, now: float | None = None
    ) -> bool:
        rigid_pose, _reason = self._fresh_right_arm_pose(now)
        if rigid_pose is None:
            return False
        return self._publish_frame_zero_skeleton(
            self._base_pose_from_rigid(rigid_pose)
        )

    def _on_at_home_query(self, reply) -> None:
        if reply.ok and reply.result.payload:
            self._on_at_home_text(bytes(reply.result.payload).decode("utf-8"))

    def _on_at_home_text(self, payload: str) -> None:
        with self._lock:
            self._at_home = payload.strip() == "true"

    def _on_return_complete_text(self, payload: str) -> None:
        if payload.strip() == "true":
            with self._lock:
                self._return_complete = True

    def _on_solved_pose(self, payload: dict[str, Any]) -> None:
        pose = _pose_from_payload(payload)
        if pose is None:
            return
        with self._lock:
            self._solved_pose = pose
            self._solved_received_at = time.monotonic()

    def _read_deadman(self) -> bool:
        if self._deadman is None:
            return False
        try:
            pressed = bool(self._deadman.is_pressed())
        except RuntimeError as exc:
            message = str(exc)
            if message != self._deadman_error:
                self._deadman_error = message
                _LOG.error("Enter 状态读取失败，轨迹已安全暂停：%s", exc)
            return False
        if pressed != self._deadman_pressed:
            _LOG.info(
                "Enter %s：phase=%s，%s",
                "按下" if pressed else "松开",
                self._phase,
                "目标继续推进" if pressed else "目标保持",
            )
        self._deadman_pressed = pressed
        return pressed

    def _on_key(self, value: str) -> None:
        if value in ("q", "\x03"):
            self._request_quit()
            return
        if value not in ("s", "r"):
            return
        now = time.monotonic()
        with self._lock:
            if value == "s":
                if now - self._last_s_at < _S_DEBOUNCE_S:
                    _LOG.info("键盘 s 的 key-repeat 连击已忽略")
                    return
                self._last_s_at = now
                if self._phase == "armed":
                    self._start_approach()
                elif self._phase in _ACTIVE_PHASES:
                    self._begin_return(exit_after_return=False)
                    _LOG.warning("键盘 s：立即取消当前流程并回 Home")
                else:
                    _LOG.info("正在回 Home；完成后可再次按 s")
                return

            if self._phase != "ready":
                _LOG.info(
                    "键盘 r：尚未稳定到达 H5 wrist→r_base frame0，phase=%s",
                    self._phase,
                )
                return
            if self._read_deadman():
                _LOG.warning("键盘 r：请先完全松开 Enter，再装载后续轨迹")
                return
            self._replay_clock = HoldToRunClock(
                maximum_step_s=1.0 / self._rate
            )
            self._last_mapped_elapsed_s = 0.0
            self._current_source_elapsed_s = 0.0
            self._current_source_frame = 0
            self._source_complete = False
            self._final_stable_ticks = 0
            self._phase = "replaying"
            self._phase_started = now
            _LOG.warning(
                "键盘 r：后续 wrist→r_base 轨迹已装载；持续按住 Enter "
                "从 frame0 推进，松开立即保持。"
            )

    def _start_approach(self) -> None:
        if not self._at_home:
            _LOG.warning("键盘 s：IK 尚未确认安全 Home，拒绝开始")
            return
        if self._deadman is None:
            _LOG.error(
                "键盘 s：Enter 松开检测不可用，拒绝自动运动：%s",
                self._deadman_error,
            )
            return
        if self._read_deadman():
            _LOG.warning("键盘 s：请先完全松开 Enter，再记录实时 r_base Home")
            return
        rigid_home_pose, reason = self._fresh_right_arm_pose()
        if rigid_home_pose is None:
            _LOG.warning("键盘 s：%s，拒绝记录 r_base Home", reason)
            return
        try:
            marker_home_pose = compose_pose(
                rigid_home_pose,
                self._rigid_to_marker_mocap_pose,
            )
            base_home_pose = compose_pose(
                marker_home_pose,
                self._marker_to_base_pose,
            )
            virtual_tcp_home_pose = compose_pose(
                base_home_pose,
                self._base_to_tcp_pose,
            )
            self._mapper.initialize(self._frame(virtual_tcp_home_pose))
        except ValueError as exc:
            _LOG.warning(
                "键盘 s：r_base/TCP Home 外参无效，拒绝开始：%s",
                exc,
            )
            return

        self._right_rigid_home_pose = rigid_home_pose.copy()
        self._right_marker_home_pose = marker_home_pose.copy()
        self._right_base_home_pose = base_home_pose
        self._virtual_tcp_home_pose = virtual_tcp_home_pose
        self._current_motive_base_target_pose = None
        self._cached_targets = None
        self._approach_stable_ticks = 0
        self._source_complete = False
        self._final_stable_ticks = 0
        self._replay_clock = None
        self._return_complete = False
        self._exit_after_return = False
        self._last_error = None
        self._last_mapped_elapsed_s = -1.0
        self._current_source_elapsed_s = 0.0
        self._current_source_frame = 0
        self._phase = "approaching"
        self._phase_started = time.monotonic()
        self._publish_frame_zero_skeleton(base_home_pose)
        _LOG.warning(
            "键盘 s：已读取 right_arm marker 并推导 wuji2 r_base Home；"
            "持续按住 Enter，使 r_base 接近 H5 wrist frame0，松开保持。"
        )

    def _request_quit(self) -> None:
        with self._lock:
            if self._phase == "armed" and self._at_home:
                _LOG.info("键盘 q：已在 Home，退出")
                self._quit = True
                self._stop_event.set()
                return
            if self._phase == "returning":
                self._exit_after_return = True
                _LOG.info("键盘 q：等待回 Home 完成后退出")
                return
            self._begin_return(exit_after_return=True)
            _LOG.warning("键盘 q：请求回 Home，完成后退出")

    def _begin_return(self, *, exit_after_return: bool) -> None:
        self._replay_clock = None
        self._return_complete = False
        self._exit_after_return = exit_after_return
        self._deadman_pressed = False
        self._phase = "returning"
        self._phase_started = time.monotonic()

    def _complete_return(self) -> None:
        self._cached_targets = None
        self._approach_stable_ticks = 0
        self._source_complete = False
        self._final_stable_ticks = 0
        self._replay_clock = None
        self._last_mapped_elapsed_s = -1.0
        self._current_source_frame = 0
        self._current_source_elapsed_s = 0.0
        self._deadman_pressed = False
        self._right_rigid_home_pose = None
        self._right_marker_home_pose = None
        self._right_base_home_pose = None
        self._virtual_tcp_home_pose = None
        self._current_motive_base_target_pose = None
        self._phase = "armed"
        self._phase_started = time.monotonic()
        self._last_skeleton_preview_at = -float("inf")
        self._publish_state("idle")

    def _map_right_pose(self, h5_wrist_pose: np.ndarray) -> bool:
        if (
            self._right_base_home_pose is None
            or self._virtual_tcp_home_pose is None
        ):
            self._last_error = "wuji2 r_base Home reference missing"
            _LOG.error("wuji2 r_base Home 起点丢失，立即回 Home")
            self._begin_return(exit_after_return=False)
            return False
        try:
            desired_base_pose = compose_pose(
                np.asarray(h5_wrist_pose, dtype=np.float64),
                self._h5_wrist_to_wuji2_base_pose,
            )
            virtual_tcp_pose = compose_pose(
                desired_base_pose,
                self._base_to_tcp_pose,
            )
            targets = self._mapper.map_frame(
                self._frame(virtual_tcp_pose)
            )
        except Exception as exc:
            self._last_error = str(exc)
            _LOG.error(
                "H5 wrist→r_base/TCP 目标映射失败，立即回 Home：%s",
                exc,
            )
            self._begin_return(exit_after_return=False)
            return False
        self._current_motive_base_target_pose = desired_base_pose.copy()
        self._cached_targets = targets
        return True

    def _target_is_stable(self, now: float) -> bool:
        targets = self._cached_targets
        solved = self._solved_pose
        if targets is None or solved is None:
            return False
        if now - self._solved_received_at > _SOLVED_STALE_S:
            return False
        diagnostics = targets.right_conditioning
        remaining_position_m = (
            diagnostics.requested_linear_speed_m_s / self._rate
        )
        remaining_orientation_rad = (
            diagnostics.requested_angular_speed_rad_s / self._rate
        )
        solved_position_error = float(
            np.linalg.norm(targets.right_pose[:3] - solved[:3])
        )
        solved_orientation_error = _rotation_error_rad(
            targets.right_pose[3:7], solved[3:7]
        )
        return (
            remaining_position_m <= self._approach_position_tolerance_m
            and remaining_orientation_rad
            <= self._approach_orientation_tolerance_rad
            and solved_position_error <= self._solved_position_tolerance_m
            and solved_orientation_error
            <= self._solved_orientation_tolerance_rad
        )

    @staticmethod
    def _pose_message(pose: np.ndarray, stamp: dict[str, int]) -> dict:
        return {
            "stamp": stamp,
            "frame_id": "right_chest",
            "position": {
                "x": float(pose[0]),
                "y": float(pose[1]),
                "z": float(pose[2]),
            },
            "orientation": {
                "x": float(pose[3]),
                "y": float(pose[4]),
                "z": float(pose[5]),
                "w": float(pose[6]),
            },
        }

    @staticmethod
    def _vector_message(direction: np.ndarray, stamp: dict[str, int]) -> dict:
        return {
            "stamp": stamp,
            "frame_id": "right_chest",
            "vector": {
                "x": float(direction[0]),
                "y": float(direction[1]),
                "z": float(direction[2]),
            },
        }

    def _publish_cached_targets(self) -> None:
        targets = self._cached_targets
        if targets is None:
            return
        stamp = stamp_now()
        self._pose_pub.put_json(
            self._pose_message(targets.right_pose, stamp)
        )
        self._elbow_pub.put_json(
            self._vector_message(
                targets.right_default_elbow_direction,
                stamp,
            )
        )

    def _motive_tracking_status(self, now: float) -> dict[str, Any]:
        frame = self._latest_motive_frame
        age_s = (
            None
            if frame is None
            else max(0.0, now - self._motive_received_at)
        )
        actual_rigid_pose = self._right_arm_pose(frame)
        actual_marker_pose = (
            None
            if actual_rigid_pose is None
            else compose_pose(
                actual_rigid_pose,
                self._rigid_to_marker_mocap_pose,
            )
        )
        actual_base_pose = (
            None
            if actual_marker_pose is None
            else compose_pose(
                actual_marker_pose, self._marker_to_base_pose
            )
        )
        fresh = (
            age_s is not None
            and age_s <= _MOTIVE_STALE_S
            and actual_base_pose is not None
        )
        desired_pose = self._current_motive_base_target_pose
        position_error_m = None
        orientation_error_deg = None
        if fresh and desired_pose is not None:
            position_error_m = float(
                np.linalg.norm(actual_base_pose[:3] - desired_pose[:3])
            )
            orientation_error_deg = float(
                np.rad2deg(
                    _rotation_error_rad(
                        actual_base_pose[3:7], desired_pose[3:7]
                    )
                )
            )
        return {
            "frame_key": MOCAP_FRAME_KEY,
            "names_key": RIGID_BODY_NAMES_KEY,
            "rigid_spec": self._right_rigid_id,
            "resolved_id": self._resolved_right_rigid_id(),
            "frame_number": (
                frame.get("frame_number")
                if isinstance(frame, dict)
                else None
            ),
            "age_ms": None if age_s is None else age_s * 1000.0,
            "tracking_valid": fresh,
            "rigid_home_pose_xyzw": (
                None
                if self._right_rigid_home_pose is None
                else self._right_rigid_home_pose.tolist()
            ),
            "marker_home_pose_xyzw": (
                None
                if self._right_marker_home_pose is None
                else self._right_marker_home_pose.tolist()
            ),
            "base_home_pose_xyzw": (
                None
                if self._right_base_home_pose is None
                else self._right_base_home_pose.tolist()
            ),
            "desired_base_pose_xyzw": (
                None if desired_pose is None else desired_pose.tolist()
            ),
            "actual_rigid_pose_xyzw": (
                None
                if actual_rigid_pose is None
                else actual_rigid_pose.tolist()
            ),
            "actual_marker_pose_xyzw": (
                None
                if actual_marker_pose is None
                else actual_marker_pose.tolist()
            ),
            "actual_base_pose_xyzw": (
                None
                if actual_base_pose is None
                else actual_base_pose.tolist()
            ),
            "position_error_m": position_error_m,
            "orientation_error_deg": orientation_error_deg,
        }

    def _base_status(self, state: str) -> dict[str, Any]:
        return {
            "state": state,
            "source": "offline_replay",
            "input": "mocap_h5_replay",
            "scope": "mocap_replay",
            "mapping": "motive_absolute_wrist_to_wuji2_base_tcp_v6",
            "body_tracking": "disabled",
            "motion_trackers_required": True,
            "elbow_constraint": "published_default_zsp_backend_selected",
            "smpl_used": False,
            "at_safe_home": state == "idle" and self._at_home,
            "control_mode": "h5_right_wrist_to_wuji2_base_hold_to_run",
            "side": "right",
            "right_rigid_id": self._right_rigid_id,
            "recording": self._recording.summary(),
            "yaw_deg": self._yaw_deg,
            "speed": self._speed,
            # 这些字段必须出现在每个高频状态样本中；真机桥不能依赖
            # 0.5s 诊断快照恰好覆盖 60Hz 的基础状态。
            "phase": self._phase,
            "source_complete": self._source_complete,
            "deadman_available": self._deadman is not None,
            "deadman_pressed": self._deadman_pressed,
            "deadman_error": self._deadman_error,
            "motive_right_arm": self._motive_tracking_status(
                time.monotonic()
            ),
            "coordinate_contract": (
                "raw right_arm rigid * Motive GL/GO -> marker_mocap; "
                "marker_mocap -> current wuji2 r_base; "
                "H5 Manus wrist * T_manus_base -> wuji2 r_base; "
                "wuji2 r_base * inverse(T_tcp_to_base) -> Tianji TCP IK"
            ),
            "frame_zero_target": (
                "absolute_h5_manus_wrist_pose_converted_to_wuji2_r_base"
            ),
            "position_delta": "p_h5_base - p_current_robot_base",
            "orientation_delta": (
                "R_h5_manus * R_manus_base * "
                "inverse(R_current_robot_base)"
            ),
            "rigid_to_marker_mocap_pose_xyzw": (
                self._rigid_to_marker_mocap_pose.tolist()
            ),
            "marker_to_base_pose_xyzw": (
                self._marker_to_base_pose.tolist()
            ),
            "tcp_to_base_pose_xyzw": self._tcp_to_base_pose.tolist(),
            "h5_wrist_to_wuji2_base_pose_xyzw": (
                self._h5_wrist_to_wuji2_base_pose.tolist()
            ),
            "base_to_tcp_pose_xyzw": self._base_to_tcp_pose.tolist(),
            "endpoint": "wuji2_r_base",
            "error": self._last_error,
        }

    def _publish_state(self, state: str) -> None:
        self._state_pub.put_text(state)
        self._status_pub.put_json(self._base_status(state))

    def _tick(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else float(now)
        with self._lock:
            if self._phase == "armed":
                self._read_deadman()
                if (
                    self._at_home
                    and now - self._last_skeleton_preview_at
                    >= _SKELETON_PREVIEW_INTERVAL_S
                ):
                    self._last_skeleton_preview_at = now
                    self._publish_latest_frame_zero_skeleton(now)
                self._publish_state("idle")
                return True
            if self._phase == "returning":
                self._publish_state("returning")
                if not (self._return_complete and self._at_home):
                    return True
                if self._exit_after_return:
                    _LOG.info("已确认 IK 回到安全 Home，退出")
                    return False
                self._complete_return()
                _LOG.info("已确认 IK 回到安全 Home；按 s 可再次加载")
                return True

            self._publish_state("teleop")


            if self._phase == "approaching":
                pressed = self._read_deadman()
                if pressed:
                    if not self._map_right_pose(self._frame_zero_pose):
                        return True
                    if self._target_is_stable(now):
                        self._approach_stable_ticks += 1
                    else:
                        self._approach_stable_ticks = 0
                    if (
                        self._approach_stable_ticks
                        >= self._required_stable_ticks
                    ):
                        self._phase = "ready"
                        self._phase_started = now
                        _LOG.warning(
                            "已到达并稳定保持 H5 wrist frame0；完全松开 "
                            "Enter，确认安全后按 r 装载后续轨迹。"
                        )
                self._publish_cached_targets()
                return True

            if self._phase == "ready":
                self._deadman_pressed = self._read_deadman()
                self._publish_cached_targets()
                return True

            if self._phase == "replaying":
                clock = self._replay_clock
                if clock is None:
                    self._last_error = "replay clock missing"
                    self._begin_return(exit_after_return=False)
                    return True
                pressed = self._read_deadman()
                elapsed_hold_s = clock.update(now, pressed)
                source_elapsed_s = min(
                    self._trajectory.duration_s,
                    elapsed_hold_s * self._speed,
                )
                if (
                    pressed
                    or source_elapsed_s
                    > self._last_mapped_elapsed_s + 1.0e-12
                ):
                    sample = self._trajectory.sample(source_elapsed_s)
                    if not self._map_right_pose(sample.pose):
                        return True
                    self._last_mapped_elapsed_s = source_elapsed_s
                    self._current_source_elapsed_s = sample.elapsed_s
                    self._current_source_frame = sample.source_frame_index
                    if sample.complete:
                        self._source_complete = True
                        if pressed and self._target_is_stable(now):
                            self._final_stable_ticks += 1
                        else:
                            self._final_stable_ticks = 0
                        if (
                            self._final_stable_ticks
                            >= self._required_stable_ticks
                        ):
                            self._phase = "completed"
                            self._replay_clock = None
                            _LOG.warning(
                                "H5 右腕轨迹回放完成并稳定到达末帧，"
                                "当前保持末帧；按 s 回 Home，或按 q "
                                "回 Home 后退出。"
                            )
                    else:
                        self._source_complete = False
                        self._final_stable_ticks = 0
                elif self._source_complete:
                    self._final_stable_ticks = 0
                self._publish_cached_targets()
                return True

            if self._phase != "completed":
                raise RuntimeError(f"未知 H5 回放 phase={self._phase!r}")
            self._read_deadman()
            self._publish_cached_targets()
            return True

    def _publish_status(self) -> None:
        with self._lock:
            state = {
                "armed": "idle",
                "approaching": "teleop",
                "ready": "teleop",
                "replaying": "teleop",
                "completed": "teleop",
                "returning": "returning",
            }[self._phase]
            status = self._base_status(state)
            status.update(
                {
                    "frame_index": self._current_source_frame,
                    "frame_count": self._recording.frame_count,
                    "source_elapsed_s": self._current_source_elapsed_s,
                    "source_duration_s": self._trajectory.duration_s,
                    "source_progress": (
                        1.0
                        if self._trajectory.duration_s <= 0.0
                        else self._current_source_elapsed_s
                        / self._trajectory.duration_s
                    ),
                    "interpolated_invalid_frames": (
                        self._trajectory.interpolated_frame_count
                    ),
                    "final_stable_ticks": self._final_stable_ticks,
                    "approach_stable_ticks": self._approach_stable_ticks,
                    "approach_required_stable_ticks": self._required_stable_ticks,
                    "ready_for_replay_key": self._phase == "ready",
                    "target_conditioning": (
                        None
                        if self._cached_targets is None
                        else self._cached_targets.right_conditioning.as_dict()
                    ),
                }
            )
            self._status_pub.put_json(status)

    def run(self) -> int:
        tick_interval = 1.0 / self._rate
        status_interval = 0.5
        next_tick = time.monotonic() + tick_interval
        next_status = next_tick + status_interval
        while True:
            if self._quit:
                return 0
            now = time.monotonic()
            if now >= next_tick:
                if not self._tick(now):
                    return 0
                next_tick += tick_interval
            if now >= next_status:
                self._publish_status()
                next_status += status_interval
            time.sleep(
                max(0.001, min(next_tick, next_status) - time.monotonic())
            )

    def close(self) -> None:
        self._stop_event.set()
        try:
            for resource in (
                self._motive_frame_sub,
                self._rigid_names_sub,
                self._at_home_sub,
                self._return_complete_sub,
                self._solved_sub,
                self._pose_pub,
                self._elbow_pub,
                self._state_pub,
                self._status_pub,
                self._frame_zero_skeleton_pub,
            ):
                resource.close()
        finally:
            try:
                self._live.close()
            finally:
                try:
                    if self._deadman is not None:
                        self._deadman.close()
                finally:
                    self._session.close()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "可选 Manus/Motive HDF5 hands/right 手腕轨迹的天机右臂 "
            "Enter 保压安全回放"
        )
    )
    parser.add_argument("h5", type=Path, help="mocap-acquisition v4.0 HDF5")
    parser.add_argument("--config", default="", help="节点参数 YAML")
    parser.add_argument(
        "--param", action="append", default=[], metavar="key:=value"
    )
    parser.add_argument("--speed", type=float, default=1.0, help="回放倍速")
    parser.add_argument(
        "--yaw-deg",
        type=float,
        default=0.0,
        help="绕 Motive +Y 旋转整条轨迹的角度",
    )
    parser.add_argument(
        "--right-rigid-id",
        default="right_arm",
        help="天机右末端 Motive 刚体：数字 id 或刚体名（默认 right_arm）",
    )
    parser.add_argument(
        "--rate", type=float, default=60.0, help="目标映射与发布频率 Hz"
    )
    parser.add_argument(
        "--connect-endpoint",
        default="",
        help="可选 Zenoh Router 端点；默认本机 scouting",
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="只校验和汇总 H5，不连接 Zenoh、不运动",
    )
    return parser


def _open_session(endpoint: str):
    if not endpoint:
        return open_session()
    import zenoh

    config = zenoh.Config.from_json5(
        json.dumps(
            {"mode": "client", "connect": {"endpoints": [endpoint]}}
        )
    )
    return zenoh.open(config)


def _configure_logging() -> None:
    """每条交互日志后留一空行，避免 raw 键盘提示挤在一起。"""
    handler = logging.StreamHandler()
    handler.terminator = "\n\n"
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        handlers=[handler],
        force=True,
    )


def main(argv=None) -> int:
    _configure_logging()
    args = _parser().parse_args(argv)
    recording = load_mocap_h5(args.h5)
    trajectory = HandPoseTrajectory(
        recording,
        side="right",
        yaw_deg=args.yaw_deg,
    )
    summary = {
        **recording.summary(),
        "selected_hand": "right",
        "trajectory_start_frame": trajectory.start_frame_index,
        "trajectory_end_frame": trajectory.end_frame_index,
        "trajectory_duration_s": trajectory.duration_s,
        "interpolated_invalid_frames": trajectory.interpolated_frame_count,
        "frame_zero_pose_xyzw": trajectory.pose_at_frame(0).tolist(),
    }
    if args.validate_only:
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return 0

    overrides: dict[str, str] = {}
    for spec in args.param:
        name, value = parse_param_override(spec)
        overrides[name] = value
    params = load_node_config(
        args.config,
        "mocap_h5_replay",
        DEFAULT_PARAMETERS,
        overrides,
    )
    session = _open_session(args.connect_endpoint)
    node: MocapH5ReplayNode | None = None
    try:
        _assert_replay_graph_is_safe(session)
        node = MocapH5ReplayNode(
            session,
            params,
            recording,
            right_rigid_id=(
                int(args.right_rigid_id)
                if args.right_rigid_id.isdecimal()
                else args.right_rigid_id
            ),
            speed=args.speed,
            yaw_deg=args.yaw_deg,
            rate=args.rate,
        )
        try:
            return node.run()
        except KeyboardInterrupt:
            node._request_quit()
            return node.run()
    finally:
        if node is not None:
            node.close()
        else:
            session.close()


if __name__ == "__main__":
    raise SystemExit(main())
