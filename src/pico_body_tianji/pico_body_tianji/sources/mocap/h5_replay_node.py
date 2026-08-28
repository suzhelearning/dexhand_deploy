#!/usr/bin/env python3
"""Manus HDF5 右手腕轨迹的 Enter 保压安全回放节点。

端点契约：输入是 H5 Manus wrist；机器人端直接对应新版 wuji hand2
``r_wrist``。Motive ``tianji_wrist`` 经 GL/GO 到 marker_mocap，再由
机械安装外参推导 ``r_wrist`` Home。H5 wrist 同样转换到 ``r_wrist``，
随后由 ``r_wrist→TCP`` 固定外参送入 IK。厂商 URDF 的 ``r_mount``
保留为刚体安装位置，不参与 Manus wrist 端点定义。

状态机：

    armed -> approaching（s 读取实时 marker；Enter 保压接近 frame0）
          -> ready（保持绝对 frame0，等待 r）
          -> replaying（r 后 Enter 保压推进后续帧）
          -> completed -> returning

HDF5 路径由 CLI 位置参数选择；左臂不发布目标，保持 Home。
"""

from __future__ import annotations
import os
import argparse
import json
import logging
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from ...protocol import topics
from ...protocol.messages import ArmSolvedPose
from ..common.replay_clock import HoldToRunClock
from ..common.session_client import SessionClient
from ..common.target_conditioner import TargetConditioningSettings
from ..common.target_mapper import ArmTargetBatch, EndEffectorTargetMapper
from ..common.target_publisher import SequenceAllocator, TargetPublisher
from ..pico_controller.controller_frame import ControllerFrame
from .h5 import (
    HAND_KEYPOINT_EDGES,
    HandPoseTrajectory,
    MocapRecording,
    compose_pose,
    invert_pose,
    load_mocap_h5,
    synthetic_reference_pose,
)
from .motive import MotiveFrameSource
from ...zenoh_util import (
    ZenohJsonSub,
    load_node_config,
    load_tianji_config,
    open_session,
    parse_param_override,
    require_single_router,
)
from ..common.keyboard import X11KeyState, raw_keyboard

_LOG = logging.getLogger("mocap_h5_replay")

FRAME_ZERO_SKELETON_KEY = topics.FRAME0_HAND_SKELETON
MOCAP_FRAME_KEY = topics.MOCAP_HANDS_FRAME
RIGID_BODY_NAMES_KEY = topics.MOCAP_RIGID_BODY_NAMES

_S_DEBOUNCE_S = 0.5
_SOLVED_STALE_S = 0.5
_MOTIVE_STALE_S = 0.5
_SKELETON_PREVIEW_INTERVAL_S = 0.2
_CREATE_DEADMAN = object()
# 厂商 beta1 URDF 固定关节 r_mount -> r_wrist。
_WUJI2_MOUNT_TO_WRIST_POSE = np.array(
    [
        0.003,
        0.00025016,
        -0.0285,
        0.0,
        0.0,
        0.0000081994999999,
        0.9999999999663841,
    ],
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
    # Motive raw tianji_wrist rigid frame -> marker URDF marker_mocap frame。
    # Motive Visuals: GL [1,-4,2] mm；GO Pitch/Yaw/Roll [-1,10,0] deg。
    "right_rigid_to_marker_mocap_translation_m": [0.001, -0.004, 0.002],
    "right_rigid_to_marker_mocap_quaternion_xyzw": [
        -0.0086933284, 0.0871524241, 0.0007605677, 0.9961567661
    ],
    # marker_mocap(G) -> beta1 r_mount(M)：marker 中心到手侧面 4mm，
    # marker_wuji2 与 r_mount 的安装旋转为 Ry(-90°)。
    "right_marker_to_mount_translation_m": [0.004, 0.0, 0.0],
    "right_marker_to_mount_quaternion_xyzw": [
        0.0, -0.7071067811865476, 0.0, 0.7071067811865476
    ],
    # Tianji TCP(T) -> beta1 r_mount(M)，含 8mm marker 刚体。
    "right_tcp_to_mount_translation_m": [0.0, 0.0, 0.008],
    "right_tcp_to_mount_quaternion_xyzw": [
        0.7071067811865476, 0.7071067811865476, 0.0, 0.0
    ],
    # H5 wrist quaternion 是采集端已标定的 wrist 局部系 W；21 点几何验证：
    # W +x=指尖，W +y=手背，W +z=小指。axis_transform 只把 Manus
    # 节点偏移表达进 W，不能再次乘进 wrist pose，否则会多转 90°。
    # wuji B: -z=指尖，-y=手背，+x=拇指，因此 W->B xyzw 如下。
    "right_h5_wrist_to_wuji2_wrist_translation_m": [0.0, 0.0, 0.0],
    "right_h5_wrist_to_wuji2_wrist_quaternion_xyzw": [
        0.7071067811865476, 0.0, -0.7071067811865476, 0.0
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
    """绕 Motive 竖直轴（+Z）旋转世界系点，与手腕轨迹 yaw 标定一致。"""
    values = np.asarray(points, dtype=np.float64)
    if values.shape != (21, 3) or not np.isfinite(values).all():
        raise ValueError("frame0 keypoints 必须是有限 (21,3) 数组")
    rotation = Rotation.from_rotvec(
        np.array([0.0, 0.0, np.deg2rad(float(yaw_deg))])
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
        publisher_instance_id: str,
        router_zid: str,
        coordinator_instance_id: str | None = None,
        expected_producer_instance_id: str | None = None,
        right_rigid_id: int | str = "tianji_wrist",
        speed: float = 1.0,
        yaw_deg: float = 0.0,
        rate: float = 60.0,
        deadman: X11KeyState | None | object = _CREATE_DEADMAN,
        start_keyboard: bool = True,
    ) -> None:
        if not np.isfinite(speed) or speed <= 0.0:
            raise ValueError("speed 必须为正有限数值")
        if not np.isfinite(yaw_deg):
            raise ValueError("yaw_deg 必须为有限数值")
        if not np.isfinite(rate) or rate <= 0.0:
            raise ValueError("rate 必须为正有限数值")
        if isinstance(right_rigid_id, int):
            if right_rigid_id <= 0:
                raise ValueError("right_rigid_id 必须为正整数或刚体名")
        elif not isinstance(right_rigid_id, str) or not right_rigid_id.strip():
            raise ValueError("right_rigid_id 必须为正整数或刚体名")
        self._expected_producer_instance_id = expected_producer_instance_id
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
        # v5 可选字段：离线 retarget 的 20 关节角。存在则回放直通
        # 驱动手（跳过 keypoints → 运行时 retarget 桥），不存在照旧。
        offline_joints = recording.hands["right"].wuji2_joints
        self._hand_joint_commands_payload = (
            self._build_hand_joint_commands_payload(offline_joints)
        )
        self._hand_keypoints_payload = (
            None
            if self._hand_joint_commands_payload is not None
            else self._build_hand_keypoints_payload()
        )
        h5_wrist_to_wuji2_wrist = _configured_pose(
            params, "right_h5_wrist_to_wuji2_wrist"
        )
        marker_to_mount = _configured_pose(
            params, "right_marker_to_mount"
        )
        tcp_to_mount = _configured_pose(
            params, "right_tcp_to_mount"
        )
        self._h5_wrist_to_wuji2_wrist_pose = (
            h5_wrist_to_wuji2_wrist.copy()
        )
        self._marker_to_mount_pose = marker_to_mount.copy()
        self._tcp_to_mount_pose = tcp_to_mount.copy()
        self._marker_to_wrist_pose = compose_pose(
            self._marker_to_mount_pose,
            _WUJI2_MOUNT_TO_WRIST_POSE,
        )
        self._tcp_to_wrist_pose = compose_pose(
            self._tcp_to_mount_pose,
            _WUJI2_MOUNT_TO_WRIST_POSE,
        )
        self._wrist_to_tcp_pose = invert_pose(self._tcp_to_wrist_pose)
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
        self._tianji_config = tianji_config
        self._mocap_to_robot = np.asarray(
            tianji_config.mocap_to_robot, dtype=np.float64
        )
        self._world_to_right_chest = np.asarray(
            tianji_config.get_world_to_chest_rotation("right"),
            dtype=np.float64,
        )
        self._left_robot_home_tcp_pose = np.concatenate((
            np.asarray(tianji_config.init_pos["left"], dtype=np.float64),
            np.asarray(tianji_config.init_quat["left"], dtype=np.float64),
        ))
        self._right_robot_home_tcp_pose = np.concatenate((
            np.asarray(tianji_config.init_pos["right"], dtype=np.float64),
            np.asarray(tianji_config.init_quat["right"], dtype=np.float64),
        ))
        self._right_robot_home_wrist_pose = compose_pose(
            self._right_robot_home_tcp_pose,
            self._tcp_to_wrist_pose,
        )
        self._mapper = EndEffectorTargetMapper(
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

        allocator = SequenceAllocator()
        self._publisher = TargetPublisher(
            session,
            source="h5_replay",
            publisher_instance_id=publisher_instance_id,
            router_zid=router_zid,
            allocator=allocator,
        )
        self._session_client = SessionClient(
            session,
            source="h5_replay",
            publisher_instance_id=publisher_instance_id,
            router_zid=router_zid,
            expected_coordinator_instance_id=coordinator_instance_id,
            allocator=allocator,
        )
        # Coordinator state/latches are subscribed and queried before any
        # keyboard event can enter a moving phase.
        self._session_client.start()

        self._lock = threading.RLock()
        self._latest_motive_frame: dict[str, Any] | None = None
        self._motive_received_at = 0.0
        self._rigid_body_names: dict[int, str] = {}
        self._right_rigid_home_pose: np.ndarray | None = None
        self._right_marker_home_pose: np.ndarray | None = None
        self._right_mount_home_pose: np.ndarray | None = None
        self._right_wrist_home_pose: np.ndarray | None = None
        self._virtual_tcp_home_pose: np.ndarray | None = None
        self._current_motive_wrist_target_pose: np.ndarray | None = None
        self._motive_frame_sub = ZenohJsonSub(
            session, MOCAP_FRAME_KEY, self._on_motive_frame
        )
        self._motive_source = MotiveFrameSource()
        self._rigid_names_sub = ZenohJsonSub(
            session, RIGID_BODY_NAMES_KEY, self._on_rigid_body_names
        )
        self._solved_sub = ZenohJsonSub(
            session, topics.arm_solved_pose("right"), self._on_solved_pose
        )
        self._solved_target_sequence: int | None = None
        self._current_target_sequence: int | None = None

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
        self._real_preflight_ok = bool(
            params.get("h5_real_preflight_passed", False)
            and params.get("hand_real_preflight_passed", False)
        )

        self._at_home = False
        self._return_complete = False
        self._phase = "armed"
        self._exit_after_return = False
        self._quit = False
        self._last_error: str | None = None
        self._last_s_at = -float("inf")
        self._cached_targets: ArmTargetBatch | None = None
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
            "等待 Motive 刚体 %s 推导 wuji2 r_mount/r_wrist Home",
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
            "等待 IK Home、有效 tianji_wrist marker 后按 s；节点推导 "
            "wuji2 r_mount/r_wrist Home，并把 H5 wrist frame0 转换到 "
            "r_wrist。随后 Enter 保压接近，按 r 装载后续轨迹；"
            "活动阶段按 s 回 Home，按 q 回 Home 后退出。"
        )

    def _frame(self, right_pose: np.ndarray) -> ControllerFrame:
        return ControllerFrame.from_poses(
            self._left_reference_pose,
            right_pose,
        )

    def _on_motive_frame(self, frame: dict[str, Any]) -> None:
        try:
            self._motive_source.parse(frame)
        except (TypeError, ValueError) as exc:
            self._last_error = str(exc)
            return
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
            return None, f"tianji_wrist 刚体 id={rigid_id} 无效或缺失"
        return pose, None

    def _mount_pose_from_rigid(self, rigid_pose: np.ndarray) -> np.ndarray:
        marker_pose = compose_pose(
            rigid_pose, self._rigid_to_marker_mocap_pose
        )
        return compose_pose(marker_pose, self._marker_to_mount_pose)

    def _wrist_pose_from_rigid(self, rigid_pose: np.ndarray) -> np.ndarray:
        return compose_pose(
            self._mount_pose_from_rigid(rigid_pose),
            _WUJI2_MOUNT_TO_WRIST_POSE,
        )

    def _frame_zero_skeleton_payload(
        self,
        wrist_home_pose: np.ndarray,
        mount_home_pose: np.ndarray,
    ) -> dict[str, Any]:
        target_wrist_motive = compose_pose(
            self._frame_zero_pose,
            self._h5_wrist_to_wuji2_wrist_pose,
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
            "home_wuji2_mount_pose_motive": mount_home_pose.tolist(),
            "home_wuji2_wrist_pose_motive": wrist_home_pose.tolist(),
            "frame0_wuji2_wrist_pose_motive": (
                target_wrist_motive.tolist()
            ),
            "tcp_to_wrist_pose_xyzw": self._tcp_to_wrist_pose.tolist(),
            "transform_contract": (
                "world axes use mocap_to_robot; tianji_wrist locates "
                "r_mount; Manus wrist maps directly to r_wrist"
            ),
        }

    def _publish_frame_zero_skeleton(
        self,
        wrist_home_pose: np.ndarray,
        mount_home_pose: np.ndarray,
    ) -> bool:
        try:
            target_wrist_motive = compose_pose(
                self._frame_zero_pose,
                self._h5_wrist_to_wuji2_wrist_pose,
            )
            self._publisher.publish_frame0_skeleton_data(
                side="right",
                keypoints_world_m=self._frame_zero_keypoints,
                manus_wrist_pose=self._frame_zero_pose,
                robot_wrist_home_pose=wrist_home_pose,
                target_wrist_pose=target_wrist_motive,
                tcp_to_wrist_pose=self._tcp_to_wrist_pose,
                edges=HAND_KEYPOINT_EDGES,
            )
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
        mount_pose = self._mount_pose_from_rigid(rigid_pose)
        wrist_pose = compose_pose(
            mount_pose, _WUJI2_MOUNT_TO_WRIST_POSE
        )
        return self._publish_frame_zero_skeleton(wrist_pose, mount_pose)

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
        try:
            solved = ArmSolvedPose.from_dict(payload)
        except (TypeError, ValueError):
            return
        if solved.side != "right" or solved.frame_id != "Base_R":
            return
        if solved.envelope.router_zid != self._session_client.router_zid:
            return
        expected = getattr(self, "_expected_producer_instance_id", None)
        if expected is not None and solved.envelope.publisher_instance_id != expected:
            return
        pose = np.concatenate(
            (np.asarray(solved.position_m), np.asarray(solved.orientation_xyzw))
        )
        with self._lock:
            self._solved_pose = pose
            self._solved_target_sequence = solved.target_sequence
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
                    "键盘 r：尚未稳定到达 H5 wrist→r_wrist frame0，phase=%s",
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
                "键盘 r：后续 wrist→r_wrist 轨迹已装载；持续按住 Enter "
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
            _LOG.warning("键盘 s：请先完全松开 Enter，再记录实时 wrist Home")
            return
        rigid_home_pose, reason = self._fresh_right_arm_pose()
        if rigid_home_pose is None:
            _LOG.warning("键盘 s：%s，拒绝记录 wrist Home", reason)
            return
        try:
            marker_home_pose = compose_pose(
                rigid_home_pose,
                self._rigid_to_marker_mocap_pose,
            )
            mount_home_pose = compose_pose(
                marker_home_pose,
                self._marker_to_mount_pose,
            )
            wrist_home_pose = compose_pose(
                mount_home_pose,
                _WUJI2_MOUNT_TO_WRIST_POSE,
            )
            virtual_tcp_home_pose = compose_pose(
                wrist_home_pose,
                self._wrist_to_tcp_pose,
            )
        except ValueError as exc:
            _LOG.warning(
                "键盘 s：r_mount/r_wrist/TCP Home 外参无效，拒绝开始：%s",
                exc,
            )
            return

        self._right_rigid_home_pose = rigid_home_pose.copy()
        self._right_marker_home_pose = marker_home_pose.copy()
        self._right_mount_home_pose = mount_home_pose
        self._right_wrist_home_pose = wrist_home_pose
        self._virtual_tcp_home_pose = virtual_tcp_home_pose
        self._current_motive_wrist_target_pose = None
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
        if not self._session_client.startup_ready:
            _LOG.warning("键盘 s：coordinator snapshot 未就绪，拒绝开始")
            return
        try:
            self._session_client.request_start("h5_s")
        except RuntimeError as exc:
            self._last_error = str(exc)
            return
        self._phase = "start_pending"
        self._phase_started = time.monotonic()
        self._publish_frame_zero_skeleton(
            wrist_home_pose, mount_home_pose
        )
        _LOG.warning(
            "键盘 s：已读取 tianji_wrist marker 并推导 r_mount/r_wrist "
            "Home；持续按住 Enter，使 r_wrist 接近 H5 wrist frame0，"
            "松开保持。"
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
        try:
            self._session_client.request_return("h5_return")
        except (RuntimeError, ValueError) as exc:
            self._last_error = str(exc)
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
        self._right_mount_home_pose = None
        self._right_wrist_home_pose = None
        self._virtual_tcp_home_pose = None
        self._current_motive_wrist_target_pose = None
        self._phase = "armed"
        self._phase_started = time.monotonic()
        self._last_skeleton_preview_at = -float("inf")
        self._publish_state("idle")

    def _map_right_pose(self, h5_wrist_pose: np.ndarray) -> bool:
        if (
            self._right_wrist_home_pose is None
            or self._virtual_tcp_home_pose is None
        ):
            self._last_error = "wuji2 r_wrist Home reference missing"
            _LOG.error("wuji2 r_wrist Home 起点丢失，立即回 Home")
            self._begin_return(exit_after_return=False)
            return False
        try:
            desired_wrist_pose = compose_pose(
                np.asarray(h5_wrist_pose, dtype=np.float64),
                self._h5_wrist_to_wuji2_wrist_pose,
            )
            # 平移以 s 时 live wrist Home 为原点；姿态使用绝对
            # Motive→robot world→right_chest 映射，不再使用 Home 相对旋转。
            delta_motive = (
                desired_wrist_pose[:3]
                - self._right_wrist_home_pose[:3]
            )
            target_wrist_position_chest = (
                self._right_robot_home_wrist_pose[:3]
                + self._world_to_right_chest
                @ (self._mocap_to_robot @ delta_motive)
            )
            target_wrist_rotation_world = (
                self._mocap_to_robot
                @ Rotation.from_quat(
                    desired_wrist_pose[3:]
                ).as_matrix()
            )
            target_wrist_rotation_chest = (
                self._world_to_right_chest
                @ target_wrist_rotation_world
            )
            target_wrist_pose_chest = np.concatenate((
                target_wrist_position_chest,
                Rotation.from_matrix(
                    target_wrist_rotation_chest
                ).as_quat(),
            ))
            target_tcp_pose_chest = compose_pose(
                target_wrist_pose_chest,
                self._wrist_to_tcp_pose,
            )
            targets = self._mapper.map_absolute_tcp_poses(
                self._left_robot_home_tcp_pose,
                target_tcp_pose_chest,
            )
        except Exception as exc:
            self._last_error = str(exc)
            _LOG.error(
                "H5 wrist→r_wrist/TCP 目标映射失败，立即回 Home：%s",
                exc,
            )
            self._begin_return(exit_after_return=False)
            return False
        self._current_motive_wrist_target_pose = desired_wrist_pose.copy()
        self._cached_targets = targets
        return True

    def _target_is_stable(self, now: float) -> bool:
        targets = self._cached_targets
        solved = self._solved_pose
        if targets is None or solved is None:
            return False
        if self._current_target_sequence is None or self._solved_target_sequence != self._current_target_sequence:
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


    def _build_hand_joint_commands_payload(
        self, offline_joints: np.ndarray | None
    ) -> np.ndarray | None:
        """预计算离线 retarget 的 20 关节角负载（float32 LE）。

        关节序与 wuji2 组合 URDF 一致（docs/mocap_h5_v40_format.md
        §3.5）。非有限帧用最近有效帧前向填充；全部无效返回 None
        （调用方跳过发布，与键点负载策略一致）。
        """
        if offline_joints is None:
            return None
        finite = np.isfinite(offline_joints).all(axis=1)
        filled = offline_joints.copy()
        last: np.ndarray | None = None
        for index, is_valid in enumerate(finite):
            if is_valid:
                last = offline_joints[index]
            elif last is not None:
                filled[index] = last
        if last is None:
            return None
        return filled.astype("<f4")

    def _build_hand_keypoints_payload(self) -> np.ndarray | None:
        """预计算右手 21×3 键点负载（float32 LE，腕部相对，Motive 系 + yaw）。

        无效帧（NaN）用最近有效帧前向填充；全部无效返回 None（调用方跳过发布）。
        负载直接喂给 wuji_hand2_bridge 的 retarget 会话。
        """
        keypoints = self._recording.hands["right"].keypoints_world
        finite = np.isfinite(keypoints).all(axis=(1, 2))
        filled = keypoints.copy()
        last: np.ndarray | None = None
        for index, is_valid in enumerate(finite):
            if is_valid:
                last = keypoints[index]
            elif last is not None:
                filled[index] = last
        if last is None:
            return None
        rotation = Rotation.from_rotvec(
            np.array([0.0, 0.0, np.deg2rad(float(self._yaw_deg))])
        ).as_matrix()
        rotated = filled @ rotation.T
        relative = rotated - rotated[:, 0:1, :]
        return relative.astype("<f4")

    def _publish_hand_joint_commands(self) -> None:
        """Direct mode emits the typed 20-joint command, never a raw byte topic."""
        payload = getattr(self, "_hand_joint_commands_payload", None)
        if payload is None:
            return
        trajectory = getattr(self, "_trajectory", None)
        start_frame = trajectory.start_frame_index if trajectory is not None else 0
        frame_index = max(self._current_source_frame, start_frame)
        from ...protocol.messages import HAND_JOINT_NAMES

        self._publisher.publish_hand_joint_command(
            side="right",
            names=HAND_JOINT_NAMES["right"],
            position_rad=payload[frame_index],
            producer="h5_direct",
        )

    def _publish_hand_keypoints(self) -> None:
        payload = getattr(self, "_hand_keypoints_payload", None)
        if payload is None:
            return
        trajectory = getattr(self, "_trajectory", None)
        start_frame = trajectory.start_frame_index if trajectory is not None else 0
        frame_index = max(self._current_source_frame, start_frame)
        self._publisher.publish_hand_target(
            side="right",
            keypoints_m=payload[frame_index],
            source_timestamp_ns=int(self._recording.time_ns[frame_index]),
        )

    def _publish_cached_targets(self) -> None:
        targets = self._cached_targets
        if targets is None:
            return
        command = self._publisher.publish_arm_target(
            side="right",
            position_m=targets.right_pose[:3],
            orientation_xyzw=targets.right_pose[3:],
            elbow_reference_direction=targets.right_default_elbow_direction,
            source_timestamp_ns=int(self._recording.time_ns[self._current_source_frame]),
        )
        self._current_target_sequence = command.envelope.sequence
        index = self._current_source_frame
        right = self._recording.hands["right"]
        right_valid = bool(right.valid[index])
        left = self._recording.hands["left"]
        left_valid = bool(left.valid[index])
        self._publisher.publish_raw_h5_replay(
            source_timestamp_ns=int(self._recording.time_ns[index]),
            hands={
                "left": {
                    "valid": left_valid,
                    "wrist_pose": left.wrist[index].tolist() if left_valid else None,
                    "keypoints_world_m": left.keypoints_world[index].tolist() if left_valid else None,
                    "wuji2_joints_rad": None,
                },
                "right": {
                    "valid": right_valid,
                    "wrist_pose": right.wrist[index].tolist() if right_valid else None,
                    "keypoints_world_m": right.keypoints_world[index].tolist() if right_valid else None,
                    "wuji2_joints_rad": (
                        right.wuji2_joints[index].tolist()
                        if right_valid and right.wuji2_joints is not None
                        else None
                    ),
                },
            },
        )
        if self._hand_joint_commands_payload is not None:
            self._publish_hand_joint_commands()
        else:
            self._publish_hand_keypoints()

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
        actual_mount_pose = (
            None
            if actual_marker_pose is None
            else compose_pose(
                actual_marker_pose, self._marker_to_mount_pose
            )
        )
        actual_wrist_pose = (
            None
            if actual_mount_pose is None
            else compose_pose(
                actual_mount_pose, _WUJI2_MOUNT_TO_WRIST_POSE
            )
        )
        fresh = (
            age_s is not None
            and age_s <= _MOTIVE_STALE_S
            and actual_wrist_pose is not None
        )
        desired_pose = self._current_motive_wrist_target_pose
        position_error_m = None
        orientation_error_deg = None
        if fresh and desired_pose is not None:
            position_error_m = float(
                np.linalg.norm(actual_wrist_pose[:3] - desired_pose[:3])
            )
            orientation_error_deg = float(
                np.rad2deg(
                    _rotation_error_rad(
                        actual_wrist_pose[3:7], desired_pose[3:7]
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
            "mount_home_pose_xyzw": (
                None
                if self._right_mount_home_pose is None
                else self._right_mount_home_pose.tolist()
            ),
            "wrist_home_pose_xyzw": (
                None
                if self._right_wrist_home_pose is None
                else self._right_wrist_home_pose.tolist()
            ),
            "desired_wrist_pose_xyzw": (
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
            "actual_mount_pose_xyzw": (
                None
                if actual_mount_pose is None
                else actual_mount_pose.tolist()
            ),
            "actual_wrist_pose_xyzw": (
                None
                if actual_wrist_pose is None
                else actual_wrist_pose.tolist()
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
            "mapping": "motive_r_mount_h5_wrist_to_wuji2_r_wrist_beta1",
            "body_tracking": "disabled",
            "motion_trackers_required": True,
            "elbow_constraint": "published_default_zsp_backend_selected",
            "at_safe_home": state == "idle" and self._at_home,
            "control_mode": "h5_right_wrist_to_wuji2_wrist_hold_to_run",
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
                "raw tianji_wrist rigid * Motive GL/GO -> marker_mocap; "
                "marker_mocap -> wuji2 r_mount -> r_wrist; "
                "H5 Manus wrist * T_manus_wrist -> wuji2 r_wrist; "
                "wuji2 r_wrist * inverse(T_tcp_to_wrist) -> Tianji TCP IK"
            ),
            "frame_zero_target": (
                "absolute_h5_manus_wrist_pose_converted_to_wuji2_r_wrist"
            ),
            "position_delta": "p_h5_wrist - p_current_robot_wrist",
            "orientation_delta": (
                "R_h5_manus * R_manus_wrist * "
                "inverse(R_current_robot_wrist)"
            ),
            "rigid_to_marker_mocap_pose_xyzw": (
                self._rigid_to_marker_mocap_pose.tolist()
            ),
            "marker_to_mount_pose_xyzw": (
                self._marker_to_mount_pose.tolist()
            ),
            "mount_to_wrist_pose_xyzw": (
                _WUJI2_MOUNT_TO_WRIST_POSE.tolist()
            ),
            "tcp_to_mount_pose_xyzw": self._tcp_to_mount_pose.tolist(),
            "tcp_to_wrist_pose_xyzw": self._tcp_to_wrist_pose.tolist(),
            "h5_wrist_to_wuji2_wrist_pose_xyzw": (
                self._h5_wrist_to_wuji2_wrist_pose.tolist()
            ),
            "wrist_to_tcp_pose_xyzw": self._wrist_to_tcp_pose.tolist(),
            "endpoint": "wuji2_r_wrist",
            "error": self._last_error,
        }

    def _publish_state(self, state: str) -> None:
        self._publish_status()

    def _tick(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else float(now)
        self._session_client.poll()
        if self._session_client.at_home is not None:
            self._at_home = bool(self._session_client.at_home)
        if self._session_client.return_complete is not None:
            self._return_complete = bool(self._session_client.return_complete)
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
                return True
            if self._phase == "start_pending":
                if self._session_client.start_authorized:
                    try:
                        self._mapper.initialize(self._frame(self._virtual_tcp_home_pose))
                    except (TypeError, ValueError) as exc:
                        self._last_error = str(exc)
                        self._begin_return(exit_after_return=False)
                    else:
                        self._phase = "approaching"
                        self._phase_started = now
                elif self._session_client.pending_intent_sequence is None:
                    # Coordinator reject/timeout is atomic: no stale reference
                    # may survive into the next start request.
                    self._phase = "armed"
                    self._right_rigid_home_pose = None
                    self._right_marker_home_pose = None
                    self._right_mount_home_pose = None
                    self._right_wrist_home_pose = None
                    self._virtual_tcp_home_pose = None
                self._publish_state("idle")
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
            if (
                self._phase in {"approaching", "ready", "replaying", "completed"}
                and not self._real_preflight_ok
                and getattr(self, "_real_mode", False)
            ):
                self._begin_return(exit_after_return=False)
                self._publish_state("returning")
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
                        if self._final_stable_ticks >= self._required_stable_ticks:
                            self._phase = "completed"
                            self._replay_clock = None
                            _LOG.warning(
                                "H5 右腕轨迹回放完成并稳定到达末帧，开始请求 return"
                            )
                            self._begin_return(exit_after_return=False)
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
                "armed": "armed",
                "start_pending": "start_pending",
                "approaching": "approaching",
                "ready": "ready",
                "replaying": "replaying",
                "completed": "completed",
                "returning": "returning",
            }.get(self._phase, self._phase)
            diagnostics = self._base_status(state)
            diagnostics.update(
                {
                    "frame_index": self._current_source_frame,
                    "frame_count": self._recording.frame_count,
                    "source_elapsed_s": self._current_source_elapsed_s,
                    "source_duration_s": self._trajectory.duration_s,
                    "source_progress": (
                        1.0
                        if self._trajectory.duration_s <= 0.0
                        else self._current_source_elapsed_s / self._trajectory.duration_s
                    ),
                    "interpolated_invalid_frames": self._trajectory.interpolated_frame_count,
                    "final_stable_ticks": self._final_stable_ticks,
                    "approach_stable_ticks": self._approach_stable_ticks,
                    "approach_required_stable_ticks": self._required_stable_ticks,
                    "ready_for_replay_key": self._phase == "ready",
                    "target_conditioning": (
                        None
                        if self._cached_targets is None
                        else self._cached_targets.right_conditioning.as_dict()
                    ),
                    "startup_snapshot_ready": self._session_client.startup_ready,
                }
            )
            self._publisher.publish_source_status(
                component_id="h5_replay",
                phase=self._phase,
                ready=(
                    self._session_client.startup_ready
                    and self._deadman is not None
                    and self._last_error is None
                ),
                healthy=self._last_error is None,
                capabilities=(
                    ["simulation", "real"]
                    if self._real_preflight_ok
                    and self._speed <= 0.25
                    and abs(self._yaw_deg) <= 1.0e-12
                    and self._deadman is not None
                    else ["simulation"]
                ),
                error=self._last_error,
                diagnostics=diagnostics,
            )

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
        for resource in (self._motive_frame_sub, self._rigid_names_sub, self._solved_sub):
            try:
                resource.close()
            except Exception:
                pass
        try:
            self._publisher.close()
        finally:
            try:
                self._session_client.close()
            finally:
                if self._deadman is not None:
                    self._deadman.close()
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
        help="绕 Motive 竖直轴(+Z)旋转整条轨迹的角度",
    )
    parser.add_argument(
        "--right-rigid-id",
        default="tianji_wrist",
        help="天机右末端 Motive 刚体：数字 id 或刚体名（默认 tianji_wrist）",
    )
    parser.add_argument(
        "--rate", type=float, default=60.0, help="目标映射与发布频率 Hz"
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
        json.dumps({"mode": "client", "connect": {"endpoints": [endpoint]}})
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
    session = _open_session(os.environ.get("TIANJI_ROUTER_ENDPOINT", "tcp/127.0.0.1:7447"))
    instance_id = os.environ.get("TIANJI_COMPONENT_INSTANCE_ID")
    router_zid = os.environ.get("TIANJI_ROUTER_ZID")
    coordinator_id = os.environ.get("TIANJI_COORDINATOR_INSTANCE_ID")
    if not instance_id or not router_zid or not coordinator_id:
        session.close()
        raise RuntimeError(
            "TIANJI_COMPONENT_INSTANCE_ID, TIANJI_ROUTER_ZID and "
            "TIANJI_COORDINATOR_INSTANCE_ID are required"
        )
    require_single_router(session, router_zid)
    node: MocapH5ReplayNode | None = None
    try:
        node = MocapH5ReplayNode(
            session,
            params,
            recording,
            publisher_instance_id=instance_id,
            router_zid=router_zid,
            coordinator_instance_id=coordinator_id,
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
