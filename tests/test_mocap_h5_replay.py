from __future__ import annotations

import importlib.util
import os
import select
import termios
from pathlib import Path
import threading
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from pico_body_tianji.controller_only.mocap_h5 import (
    HAND_KEYPOINT_EDGES,
    HandTrajectorySample,
    compose_pose,
    invert_pose,
)
from pico_body_tianji.mujoco_urdf import portable_mujoco_urdf
from pico_body_tianji.controller_only.mocap_h5_replay_node import (
    DEFAULT_PARAMETERS,
    MocapH5ReplayNode,
    _WUJI2_MOUNT_TO_WRIST_POSE,
    _configure_logging,
    _configured_pose,
)
from pico_body_tianji.controller_only.raw_keyboard import raw_keyboard
from pico_body_tianji.joint_state_model import urdf_joint_names
from pico_body_tianji.mujoco_urdf import portable_mujoco_urdf


class _FakePub:
    def __init__(self) -> None:
        self.json_values: list[dict] = []
        self.text_values: list[str] = []
        self.bytes_values: list[bytes] = []

    def put_json(self, value: dict) -> None:
        self.json_values.append(value)

    def put_text(self, value: str) -> None:
        self.text_values.append(value)

    def put_bytes(self, value: bytes) -> None:
        self.bytes_values.append(value)


class _FakeDeadman:
    def __init__(self) -> None:
        self.pressed = False

    def is_pressed(self) -> bool:
        return self.pressed


class _FakeDiagnostics:
    requested_linear_speed_m_s = 0.0
    requested_angular_speed_rad_s = 0.0

    @staticmethod
    def as_dict() -> dict:
        return {"requested_linear_speed_m_s": 0.0}


class _FakeMapper:
    def __init__(self, target_pose: np.ndarray) -> None:
        self.target_pose = target_pose
        self.initialized = False
        self.initialized_frame = None
        self.mapped_frames = []
        self.map_count = 0

    def initialize(self, frame) -> None:
        self.initialized = True
        self.initialized_frame = frame

    def map_frame(self, frame):
        self.map_count += 1
        self.mapped_frames.append(frame)
        return SimpleNamespace(
            right_pose=self.target_pose.copy(),
            right_conditioning=_FakeDiagnostics(),
            right_default_elbow_direction=np.array([0.0, 1.0, 0.0]),
        )

    def map_absolute_poses(self, left_pose, right_pose):
        self.map_count += 1
        frame = SimpleNamespace(
            left_pose=np.asarray(left_pose, dtype=np.float64).copy(),
            right_pose=np.asarray(right_pose, dtype=np.float64).copy(),
        )
        self.mapped_frames.append(frame)
        return SimpleNamespace(
            left_pose=frame.left_pose,
            right_pose=frame.right_pose,
            left_conditioning=_FakeDiagnostics(),
            right_conditioning=_FakeDiagnostics(),
            left_default_elbow_direction=np.array([0.0, -1.0, 0.0]),
            right_default_elbow_direction=np.array([0.0, 1.0, 0.0]),
        )


class _FakeTrajectory:
    duration_s = 0.5
    interpolated_frame_count = 1
    start_frame_index = 0

    def sample(self, elapsed_s: float) -> HandTrajectorySample:
        elapsed = min(float(elapsed_s), self.duration_s)
        return HandTrajectorySample(
            pose=np.array(
                [0.2 + elapsed, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
            ),
            elapsed_s=elapsed,
            source_frame_index=min(2, int(elapsed * 4.0)),
            complete=elapsed >= self.duration_s,
        )


class MocapH5ReplayStateMachineTest(unittest.TestCase):
    @staticmethod
    def _node() -> MocapH5ReplayNode:
        node = object.__new__(MocapH5ReplayNode)
        pose = np.array([0.5, 0.1, -0.2, 0.0, 0.0, 0.0, 1.0])
        node._lock = threading.RLock()
        node._phase = "armed"
        node._phase_started = time.monotonic()
        node._at_home = True
        node._return_complete = False
        node._exit_after_return = False
        node._quit = False
        node._last_error = None
        node._last_s_at = -float("inf")
        node._deadman = _FakeDeadman()
        node._deadman_error = None
        node._deadman_pressed = False
        node._mapper = _FakeMapper(pose)
        node._frame_zero_pose = np.array(
            [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        )
        node._h5_wrist_to_wuji2_wrist_pose = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        )
        node._frame_zero_source_index = 0
        node._frame_zero_keypoints = np.column_stack(
            (
                np.linspace(0.2, 0.3, 21),
                np.zeros(21),
                np.zeros(21),
            )
        )
        node._left_reference_pose = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        )
        node._rigid_to_marker_mocap_pose = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        )
        desired_marker_to_wrist = np.array(
            [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        )
        node._marker_to_mount_pose = compose_pose(
            desired_marker_to_wrist,
            invert_pose(_WUJI2_MOUNT_TO_WRIST_POSE),
        )
        node._marker_to_wrist_pose = desired_marker_to_wrist
        desired_tcp_to_wrist = np.array(
            [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        )
        node._tcp_to_mount_pose = compose_pose(
            desired_tcp_to_wrist,
            invert_pose(_WUJI2_MOUNT_TO_WRIST_POSE),
        )
        node._tcp_to_wrist_pose = desired_tcp_to_wrist
        node._wrist_to_tcp_pose = invert_pose(desired_tcp_to_wrist)
        node._mocap_to_robot = np.eye(3)
        node._world_to_right_chest = np.eye(3)
        node._left_robot_home_tcp_pose = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        )
        node._right_robot_home_tcp_pose = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        )
        node._right_robot_home_wrist_pose = np.array(
            [1.1, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]
        )
        node._right_rigid_id = "tianji_wrist"
        node._rigid_body_names = {7: "tianji_wrist"}
        node._latest_motive_frame = {
            "frame_number": 42,
            "rigid_bodies": [
                {
                    "id": 7,
                    "tracking_valid": True,
                    "position": [1.0, 2.0, 3.0],
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                }
            ],
        }
        node._motive_received_at = time.monotonic()
        node._right_rigid_home_pose = None
        node._right_marker_home_pose = None
        node._right_mount_home_pose = None
        node._right_wrist_home_pose = None
        node._virtual_tcp_home_pose = None
        node._current_motive_wrist_target_pose = None
        node._approach_stable_ticks = 0
        node._cached_targets = None
        node._source_complete = False
        node._final_stable_ticks = 0
        node._required_stable_ticks = 2
        node._approach_position_tolerance_m = 0.002
        node._approach_orientation_tolerance_rad = 0.02
        node._solved_position_tolerance_m = 0.005
        node._solved_orientation_tolerance_rad = 0.04
        node._solved_pose = pose.copy()
        node._solved_received_at = 10.0
        node._trajectory = _FakeTrajectory()
        node._speed = 1.0
        node._rate = 4.0
        node._replay_clock = None
        node._last_mapped_elapsed_s = -1.0
        node._current_source_frame = 0
        node._current_source_elapsed_s = 0.0
        node._last_skeleton_preview_at = -float("inf")
        node._last_skeleton_error = None
        node._pose_pub = _FakePub()
        node._elbow_pub = _FakePub()
        node._state_pub = _FakePub()
        node._status_pub = _FakePub()
        node._frame_zero_skeleton_pub = _FakePub()
        node._keypoints_pub = _FakePub()
        node._hand_keypoints_payload = np.zeros((4, 21, 3), dtype=np.float32)
        node._stop_event = threading.Event()
        node._recording = SimpleNamespace(
            path="/chosen/trajectory.h5",
            frame_count=3,
            summary=lambda: {"path": "/chosen/trajectory.h5"},
        )
        node._yaw_deg = 0.0
        return node

    def test_build_hand_joint_commands_payload(self) -> None:
        node = object.__new__(MocapH5ReplayNode)
        # 无离线关节 → None
        self.assertIsNone(node._build_hand_joint_commands_payload(None))
        # 正常数据 → float32 LE (N,20)
        joints = np.linspace(0.0, 1.0, 200).reshape(10, 20)
        payload = node._build_hand_joint_commands_payload(joints)
        self.assertEqual(payload.shape, (10, 20))
        self.assertEqual(payload.dtype, np.dtype("<f4"))
        np.testing.assert_allclose(payload, joints, atol=1e-6)
        # 非有限帧用最近有效帧前向填充
        broken = joints.copy()
        broken[3] = np.nan
        payload = node._build_hand_joint_commands_payload(broken)
        np.testing.assert_allclose(payload[3], payload[2])
        # 全部无效 → None
        self.assertIsNone(
            node._build_hand_joint_commands_payload(
                np.full((10, 20), np.nan)
            )
        )

    def test_default_rigid_pose_derives_mount_and_wrist(self) -> None:
        rigid_to_marker = _configured_pose(
            DEFAULT_PARAMETERS, "right_rigid_to_marker_mocap"
        )
        marker_to_mount = _configured_pose(
            DEFAULT_PARAMETERS, "right_marker_to_mount"
        )
        rigid_to_mount = compose_pose(
            rigid_to_marker, marker_to_mount
        )
        rigid_to_wrist = compose_pose(
            rigid_to_mount, _WUJI2_MOUNT_TO_WRIST_POSE
        )
        tcp_to_wrist = compose_pose(
            _configured_pose(DEFAULT_PARAMETERS, "right_tcp_to_mount"),
            _WUJI2_MOUNT_TO_WRIST_POSE,
        )

        np.testing.assert_allclose(
            rigid_to_marker,
            [
                0.001, -0.004, 0.002,
                -0.0086933284, 0.0871524241,
                0.0007605677, 0.9961567661,
            ],
            atol=1e-8,
        )
        np.testing.assert_allclose(
            rigid_to_mount,
            [
                0.0049392310, -0.004, 0.0013054073,
                -0.0056093089, -0.6427631343, 0.0066849140,
                0.7660152745,
            ],
            atol=1e-8,
        )
        np.testing.assert_allclose(
            rigid_to_wrist,
            [
                0.0335263590, -0.0036975209, -0.0006938921,
                -0.0056145792, -0.6427630883, 0.0066911950,
                0.7660152196,
            ],
            atol=1e-8,
        )
        np.testing.assert_allclose(
            tcp_to_wrist[:3],
            [0.00025016, 0.003, 0.0365],
            atol=1e-9,
        )

    def test_h5_manus_wrist_axes_map_to_wuji2_r_wrist(self) -> None:
        wrist_transform = _configured_pose(
            DEFAULT_PARAMETERS, "right_h5_wrist_to_wuji2_wrist"
        )
        rotation_wrist_from_h5 = Rotation.from_quat(
            wrist_transform[3:]
        ).as_matrix()
        np.testing.assert_allclose(
            rotation_wrist_from_h5,
            [
                [0.0, 0.0, -1.0],
                [0.0, -1.0, 0.0],
                [-1.0, 0.0, 0.0],
            ],
            atol=1e-9,
        )
        self.assertAlmostEqual(
            float(np.linalg.det(rotation_wrist_from_h5)), 1.0
        )


        node = self._node()
        node._h5_wrist_to_wuji2_wrist_pose = wrist_transform
        node._right_wrist_home_pose = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        )
        node._right_robot_home_wrist_pose = node._right_wrist_home_pose.copy()
        node._virtual_tcp_home_pose = node._right_wrist_home_pose.copy()
        self.assertTrue(node._map_right_pose(node._frame_zero_pose))
        desired_wrist = compose_pose(
            node._frame_zero_pose, wrist_transform
        )
        expected_tcp = compose_pose(
            desired_wrist, node._wrist_to_tcp_pose
        )
        np.testing.assert_allclose(
            node._mapper.mapped_frames[-1].right_pose,
            expected_tcp,
            atol=1e-9,
        )
        reconstructed_wrist = compose_pose(
            expected_tcp, node._tcp_to_wrist_pose
        )
        np.testing.assert_allclose(
            reconstructed_wrist, desired_wrist, atol=1e-9
        )


    def test_s_approaches_absolute_wrist_frame_zero_then_r_replays(self) -> None:
        node = self._node()

        node._on_key("s")
        self.assertEqual(node._phase, "approaching")
        self.assertTrue(node._mapper.initialized)
        # raw rigid [1,2,3]，fixture 的 rigid→marker 为 I，
        # marker→wrist +0.1，wrist→TCP -0.05。
        np.testing.assert_allclose(
            node._mapper.initialized_frame.right_pose,
            [1.05, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0],
        )
        np.testing.assert_allclose(
            node._right_wrist_home_pose,
            [1.1, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.assertEqual(node._mapper.map_count, 0)
        self.assertEqual(len(node._frame_zero_skeleton_pub.json_values), 1)
        skeleton = node._frame_zero_skeleton_pub.json_values[-1]
        self.assertTrue(skeleton["frozen"])
        self.assertEqual(len(skeleton["points_motive_world"]), 21)
        self.assertEqual(
            tuple(tuple(edge) for edge in skeleton["edges"]),
            HAND_KEYPOINT_EDGES,
        )
        np.testing.assert_allclose(
            skeleton["points_motive_world"][0],
            [0.2, 0.0, 0.0],
        )
        np.testing.assert_allclose(
            skeleton["points_motive_world"][-1],
            [0.3, 0.0, 0.0],
        )

        # Enter 保压接近 H5 绝对 wrist frame0；虚拟 TCP=0.2-0.05。
        node._deadman.pressed = True
        node._tick(10.0)
        node._solved_pose = node._cached_targets.right_pose.copy()
        node._solved_received_at = 10.1
        node._tick(10.1)
        node._solved_received_at = 10.2
        node._tick(10.2)
        self.assertEqual(node._phase, "ready")
        self.assertEqual(node._mapper.map_count, 3)
        np.testing.assert_allclose(
            node._mapper.mapped_frames[-1].right_pose,
            [0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.assertEqual(len(node._pose_pub.json_values), 3)

        # r 前必须松开 Enter。
        node._on_key("r")
        self.assertEqual(node._phase, "ready")
        node._deadman.pressed = False
        node._on_key("r")
        self.assertEqual(node._phase, "replaying")

        start = time.monotonic() + 1.0
        node._deadman.pressed = True
        node._tick(start)
        node._tick(start + 0.25)
        self.assertAlmostEqual(node._current_source_elapsed_s, 0.25)
        # H5 wrist 0.45 经 wrist→TCP -0.05，虚拟 TCP=0.40。
        np.testing.assert_allclose(
            node._mapper.mapped_frames[-1].right_pose[:3],
            [0.40, 0.0, 0.0],
        )

        node._deadman.pressed = False
        node._tick(start + 0.25)
        node._tick(start + 20.0)
        self.assertAlmostEqual(node._current_source_elapsed_s, 0.25)
        self.assertEqual(node._phase, "replaying")

        node._deadman.pressed = True
        node._solved_received_at = start + 20.0
        node._tick(start + 20.0)
        # 第一帧到末点后 target 已更新，solved 下一周期才跟上。
        node._tick(start + 20.25)
        self.assertEqual(node._phase, "replaying")
        self.assertTrue(node._source_complete)
        self.assertEqual(node._final_stable_ticks, 0)
        node._solved_pose = node._cached_targets.right_pose.copy()
        node._solved_received_at = start + 20.5
        node._tick(start + 20.5)
        self.assertEqual(node._final_stable_ticks, 1)
        node._solved_received_at = start + 20.75
        node._tick(start + 20.75)
        self.assertEqual(node._phase, "completed")
        self.assertAlmostEqual(node._current_source_elapsed_s, 0.5)
        map_count_at_completion = node._mapper.map_count
        node._tick(start + 21.0)
        self.assertEqual(node._mapper.map_count, map_count_at_completion)

        node._last_s_at = -float("inf")
        node._on_key("s")
        self.assertEqual(node._phase, "returning")
        node._tick(start + 21.25)
        self.assertEqual(node._phase, "returning")
        node._return_complete = True
        node._at_home = True
        node._tick(start + 21.35)
        self.assertEqual(node._phase, "armed")

    def test_s_requires_released_enter_before_alignment(self) -> None:
        node = self._node()
        node._deadman.pressed = True

        node._on_key("s")

        self.assertEqual(node._phase, "armed")
        self.assertFalse(node._mapper.initialized)
        self.assertEqual(node._mapper.map_count, 0)

    def test_s_refuses_to_leave_home_until_ik_confirms_home(self) -> None:
        node = self._node()
        node._at_home = False
        node._on_key("s")
        self.assertEqual(node._phase, "armed")
        self.assertFalse(node._mapper.initialized)

    def test_s_requires_fresh_valid_right_arm_rigid_body(self) -> None:
        node = self._node()
        node._latest_motive_frame = None
        node._on_key("s")
        self.assertEqual(node._phase, "armed")
        self.assertFalse(node._mapper.initialized)

        node = self._node()
        node._motive_received_at = time.monotonic() - 1.0
        node._on_key("s")
        self.assertEqual(node._phase, "armed")
        self.assertFalse(node._mapper.initialized)

        node = self._node()
        node._latest_motive_frame["rigid_bodies"][0]["tracking_valid"] = False
        node._on_key("s")
        self.assertEqual(node._phase, "armed")
        self.assertFalse(node._mapper.initialized)

    def test_tracking_status_reports_actual_wrist_to_target_error(
        self,
    ) -> None:
        node = self._node()
        node._right_marker_home_pose = np.array(
            [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]
        )
        node._right_wrist_home_pose = np.array(
            [1.1, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]
        )
        node._current_motive_wrist_target_pose = np.array(
            [1.11, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]
        )
        status = node._motive_tracking_status(time.monotonic())
        self.assertTrue(status["tracking_valid"])
        self.assertEqual(status["resolved_id"], 7)
        self.assertAlmostEqual(status["position_error_m"], 0.01)
        self.assertAlmostEqual(status["orientation_error_deg"], 0.0)
        np.testing.assert_allclose(
            status["actual_marker_pose_xyzw"],
            [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0],
        )
        np.testing.assert_allclose(
            status["actual_wrist_pose_xyzw"],
            [1.1, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0],
        )

    def test_armed_tick_samples_deadman_without_moving(self) -> None:
        node = self._node()
        node._deadman.pressed = True

        self.assertTrue(node._tick(time.monotonic()))

        self.assertEqual(node._phase, "armed")
        self.assertTrue(node._deadman_pressed)
        self.assertEqual(node._mapper.map_count, 0)
        self.assertEqual(node._state_pub.text_values[-1], "idle")
        self.assertTrue(node._status_pub.json_values[-1]["deadman_pressed"])
        self.assertEqual(len(node._frame_zero_skeleton_pub.json_values), 1)
        self.assertFalse(
            node._frame_zero_skeleton_pub.json_values[-1]["frozen"]
        )

    def test_every_base_status_carries_real_readiness_fields(self) -> None:
        node = self._node()
        node._speed = 0.1

        status = node._base_status("idle")

        self.assertEqual(status["phase"], "armed")
        self.assertTrue(status["at_safe_home"])
        self.assertTrue(status["deadman_available"])
        self.assertFalse(status["deadman_pressed"])
        self.assertIsNone(status["deadman_error"])
        self.assertFalse(status["source_complete"])
        self.assertTrue(status["motive_right_arm"]["tracking_valid"])
        self.assertEqual(status["motive_right_arm"]["resolved_id"], 7)
        self.assertEqual(
            status["mapping"],
            "motive_r_mount_h5_wrist_to_wuji2_r_wrist_beta1",
        )
        self.assertEqual(status["endpoint"], "wuji2_r_wrist")
        self.assertEqual(
            status["control_mode"],
            "h5_right_wrist_to_wuji2_wrist_hold_to_run",
        )

    def test_q_returns_home_before_exit(self) -> None:
        node = self._node()
        node._phase = "replaying"
        node._request_quit()
        self.assertEqual(node._phase, "returning")
        self.assertTrue(node._exit_after_return)
        self.assertFalse(node._quit)

        node._return_complete = True
        node._at_home = True
        self.assertFalse(node._tick(time.monotonic()))

    def test_hand_keypoints_payload_wrist_relative_and_forward_fill(self) -> None:
        node = object.__new__(MocapH5ReplayNode)
        keypoints = np.zeros((5, 21, 3), dtype=np.float64)
        keypoints[0] = np.ones((21, 3)) * 1.0
        keypoints[1] = np.ones((21, 3)) * 2.0
        keypoints[2, :, :] = np.nan
        keypoints[3] = np.ones((21, 3)) * 3.0
        keypoints[4, :, :] = np.nan
        node._recording = SimpleNamespace(
            hands={"right": SimpleNamespace(keypoints_world=keypoints)}
        )
        node._yaw_deg = 0.0
        payload = node._build_hand_keypoints_payload()
        self.assertIsNotNone(payload)
        self.assertEqual(payload.shape, (5, 21, 3))
        self.assertEqual(payload.dtype, np.dtype("<f4"))
        np.testing.assert_allclose(payload[:, 0, :], 0.0, atol=1.0e-7)
        np.testing.assert_allclose(payload[2], payload[1], atol=1.0e-6)
        np.testing.assert_allclose(payload[4], payload[3], atol=1.0e-6)

    def test_teleop_tick_publishes_hand_keypoints(self) -> None:
        node = self._node()
        node._hand_keypoints_payload = np.zeros((4, 21, 3), dtype=np.float32)
        node._hand_keypoints_payload[1, 1:, :] = 0.5
        node._current_source_frame = 1
        node._cached_targets = SimpleNamespace(
            right_pose=np.array([0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
            right_default_elbow_direction=np.array([0.0, 1.0, 0.0]),
            right_conditioning=_FakeDiagnostics(),
        )
        node._publish_cached_targets()
        self.assertEqual(len(node._keypoints_pub.bytes_values), 1)
        payload = np.frombuffer(
            node._keypoints_pub.bytes_values[-1], dtype=np.float32
        ).reshape(21, 3)
        np.testing.assert_allclose(payload[0, :], 0.0, atol=1.0e-7)
        np.testing.assert_allclose(payload[1:, :], 0.5, atol=1.0e-6)


class TianjiWuji2AssetTest(unittest.TestCase):
    """tianji_wuji2.urdf（本机新组合资产）的 MuJoCo 加载契约。"""

    def test_tianji_wuji2_urdf_loads_and_keeps_visual(self) -> None:
        root = Path(__file__).resolve().parents[1]
        urdf = (
            root
            / "src/pico_body_tianji/assets/tianji_wuji2"
            / "tianji_wuji2.urdf"
        )
        self.assertTrue(urdf.is_file())
        xml, assets = portable_mujoco_urdf(urdf)
        # 薄壳 TCP 轴 mesh 被替换为球，不再进入 assets。
        self.assertNotIn("TCP_Link_L.STL", assets)
        self.assertGreaterEqual(len(assets), 60)
        model = mujoco.MjModel.from_xml_string(xml, assets)
        # 双臂 14 关节。
        for name in urdf_joint_names():
            self.assertGreaterEqual(
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name), 0
            )
        # wuji2 双手 40 关节（tianji_wuji2 命名，小指无 _finger_ 后缀）。
        hand_joints = []
        for side in ("r", "l"):
            for finger in (
                "thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp", "thumb_ip",
                "index_finger_mcp_flex", "index_finger_mcp_abd",
                "index_finger_pip", "index_finger_dip",
                "middle_finger_mcp_flex", "middle_finger_mcp_abd",
                "middle_finger_pip", "middle_finger_dip",
                "ring_finger_mcp_flex", "ring_finger_mcp_abd",
                "ring_finger_pip", "ring_finger_dip",
                "pinky_mcp_flex", "pinky_mcp_abd", "pinky_pip", "pinky_dip",
            ):
                hand_joints.append(f"{side}_{finger}")
        for name in hand_joints:
            self.assertGreaterEqual(
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name), 0
            )
        # 视觉保留（axis 几何 + 骨架注入前的原生视觉）。
        self.assertGreater(model.ngeom, 100)
        # wrist replay 依赖的轴几何存在。
        for geom in (
            "r_wrist_axis_0", "r_wrist_axis_2",
            "TCP_Link_R_axis_0", "TCP_Link_R_axis_2",
            "marker_mocap_r_axis_0", "l_wrist_axis_0",
        ):
            self.assertGreaterEqual(
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom), 0
            )


class _FakeSubscriberHandle:
    def undeclare(self) -> None:
        pass


class _FakeZenohSession:
    def __init__(self) -> None:
        self.handler = None

    def declare_subscriber(self, _topic, handler):
        self.handler = handler
        return _FakeSubscriberHandle()


class FrameZeroHandSkeletonViewerTest(unittest.TestCase):
    @staticmethod
    def _viewer_module():
        path = (
            Path(__file__).resolve().parents[1]
            / "src/pico_body_tianji/scripts/mujoco_joint_viewer.py"
        )
        spec = importlib.util.spec_from_file_location(
            "mujoco_joint_viewer_test_module", path
        )
        if spec is None or spec.loader is None:
            raise RuntimeError(f"无法加载 viewer：{path}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_wuji_hand_command_mirror_updates_new_urdf(self) -> None:
        viewer = self._viewer_module()
        root = Path(__file__).resolve().parents[1]
        urdf = (
            root
            / "src/pico_body_tianji/assets/tianji_wuji2"
            / "tianji_wuji2.urdf"
        )
        xml, assets = portable_mujoco_urdf(urdf)
        model = mujoco.MjModel.from_xml_string(xml, assets)
        data = mujoco.MjData(model)
        session = _FakeZenohSession()
        mirror = viewer.WujiHandCommandMirror(
            session, model, "/pico_body_sim/right_hand/joint_commands"
        )
        qpos = np.linspace(-0.4, 0.7, 20, dtype=np.float32)
        session.handler(SimpleNamespace(payload=qpos.tobytes()))
        self.assertEqual(mirror.apply_latest(data), 20)
        self.assertTrue(mirror.received_once)
        for index, name in enumerate(viewer._WUJI2_RIGHT_JOINT_NAMES):
            joint_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            self.assertGreaterEqual(joint_id, 0)
            address = model.jnt_qposadr[joint_id]
            self.assertAlmostEqual(float(data.qpos[address]), float(qpos[index]))
        mirror.close()

    def test_motive_frame_zero_uses_fixed_world_axes_and_home_origin(self) -> None:
        viewer = self._viewer_module()
        root = Path(__file__).resolve().parents[1]
        urdf = (
            root
            / "src/pico_body_tianji/assets/tianji_wuji2"
            / "tianji_wuji2.urdf"
        )
        xml, assets = portable_mujoco_urdf(urdf)
        model = mujoco.MjModel.from_xml_string(
            viewer._add_frame_zero_skeleton(xml),
            assets,
        )
        data = mujoco.MjData(model)
        session = _FakeZenohSession()
        skeleton = viewer.FrameZeroHandSkeleton(
            session,
            model,
            "/pico_body_sim/frame0_hand_skeleton",
        )
        # 最新双手 URDF 的静态调试轴全部隐藏，只保留动态 Manus/r_wrist。
        for prefix in (
            "TCP_Link_L_axis_", "TCP_Link_R_axis_",
            "marker_mocap_l_axis_", "marker_mocap_r_axis_",
            "l_mount_axis_", "r_mount_axis_",
            "l_wrist_axis_", "r_wrist_axis_",
        ):
            for index in range(3):
                geom_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_GEOM, f"{prefix}{index}"
                )
                self.assertGreaterEqual(geom_id, 0)
                self.assertEqual(float(model.geom_rgba[geom_id, 3]), 0.0)
        config = skeleton._tianji_config
        home = np.concatenate(
            (
                np.asarray(config.init_joints["left"]),
                np.asarray(config.init_joints["right"]),
            )
        )
        for name, angle_deg in zip(urdf_joint_names(), home):
            joint_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            data.qpos[model.jnt_qposadr[joint_id]] = np.deg2rad(
                float(angle_deg)
            )
        mujoco.mj_forward(model, data)

        wrist_position_sim, wrist_rotation_sim = (
            viewer._frame_from_axis_geoms(
                data,
                skeleton._wrist_axis_x_geom_id,
                skeleton._wrist_axis_z_geom_id,
                0.045,
            )
        )
        _tcp_position_sim, tcp_rotation_sim = (
            viewer._frame_from_axis_geoms(
                data,
                skeleton._tcp_axis_x_geom_id,
                skeleton._tcp_axis_z_geom_id,
                0.025,
            )
        )
        wrist_position_motive = np.array([1.0, 2.0, 3.0])
        # 故意使用任意 marker/wrist 局部姿态；世界轴映射不得依赖它。
        wrist_quaternion_motive = Rotation.from_euler(
            "xyz", [0.35, -0.2, 0.65]
        ).as_quat()
        points_motive = np.empty((21, 3), dtype=np.float64)
        points_motive[0] = wrist_position_motive
        for finger, start in enumerate((1, 5, 9, 13, 17)):
            lateral = (finger - 2) * 0.012
            for joint in range(4):
                points_motive[start + joint] = (
                    wrist_position_motive
                    + np.array(
                        [lateral, -0.025 * (joint + 1), 0.004 * finger]
                    )
                )
        rotation_sim_from_motive = viewer._sim_from_motive_rotation(
            tcp_rotation_sim, config
        )
        points_sim = (
            (points_motive - wrist_position_motive)
            @ rotation_sim_from_motive.T
            + wrist_position_sim
        )
        # 构造一个映射后恰好等于模型 FK r_wrist 的 frame0 目标姿态。
        target_rotation_motive = (
            rotation_sim_from_motive.T @ wrist_rotation_sim
        )
        target_quaternion_motive = Rotation.from_matrix(
            target_rotation_motive
        ).as_quat()
        payload = {
            "frozen": False,
            "source_frame_index": 0,
            "edges": [list(edge) for edge in HAND_KEYPOINT_EDGES],
            "points_motive_world": points_motive.tolist(),
            "frame0_manus_quat_xyzw": (
                wrist_quaternion_motive.tolist()
            ),
            "home_wuji2_wrist_pose_motive": [
                *wrist_position_motive.tolist(),
                *wrist_quaternion_motive.tolist(),
            ],
            "frame0_wuji2_wrist_pose_motive": [
                *wrist_position_motive.tolist(),
                *target_quaternion_motive.tolist(),
            ],
            "tcp_to_wrist_pose_xyzw": [
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0
            ],
        }
        skeleton._on_skeleton(payload)
        self.assertTrue(skeleton.apply_latest(model, data))
        mujoco.mj_forward(model, data)

        # 动态 r_wrist 三轴共用 FK 原点，方向与 X/Z 恢复的右手系一致。
        dynamic_directions = []
        for axis_index, geom_id in enumerate(skeleton._wrist_axis_ids):
            direction = data.geom_xmat[geom_id].reshape(3, 3)[:, 2]
            dynamic_directions.append(direction)
            origin = (
                data.geom_xpos[geom_id]
                - model.geom_size[geom_id, 1] * direction
            )
            np.testing.assert_allclose(
                origin, wrist_position_sim, atol=1.0e-8
            )
            np.testing.assert_allclose(
                direction, wrist_rotation_sim[:, axis_index], atol=1.0e-8
            )
        np.testing.assert_allclose(
            np.cross(dynamic_directions[0], dynamic_directions[1]),
            dynamic_directions[2],
            atol=1.0e-8,
        )

        # 原始 Manus W 轴单独显示：方向来自 H5 wrist quaternion。
        manus_rotation_sim = (
            rotation_sim_from_motive
            @ Rotation.from_quat(wrist_quaternion_motive).as_matrix()
        )
        for axis_index, geom_id in enumerate(skeleton._manus_axis_ids):
            direction = data.geom_xmat[geom_id].reshape(3, 3)[:, 2]
            origin = (
                data.geom_xpos[geom_id]
                - model.geom_size[geom_id, 1] * direction
            )
            np.testing.assert_allclose(
                origin, wrist_position_sim, atol=1.0e-8
            )
            np.testing.assert_allclose(
                direction, manus_rotation_sim[:, axis_index], atol=1.0e-8
            )

        # 转换后的目标 Wuji B 不再画第三套轴，只用于数值误差。
        np.testing.assert_allclose(
            skeleton._target_origin_mj, wrist_position_sim, atol=1.0e-8
        )
        np.testing.assert_allclose(
            skeleton._target_rotation_mj, wrist_rotation_sim, atol=1.0e-8
        )

        self.assertEqual(len(skeleton._point_geom_ids), 21)
        self.assertEqual(len(skeleton._bone_geom_ids), 20)
        for index, geom_id in enumerate(skeleton._point_geom_ids):
            np.testing.assert_allclose(
                data.geom_xpos[geom_id], points_sim[index], atol=1.0e-8
            )
            self.assertGreater(model.geom_rgba[geom_id, 3], 0.9)

        first_parent, first_child = HAND_KEYPOINT_EDGES[0]
        first_bone = skeleton._bone_geom_ids[0]
        delta = points_sim[first_child] - points_sim[first_parent]
        np.testing.assert_allclose(
            data.geom_xpos[first_bone],
            0.5 * (points_sim[first_parent] + points_sim[first_child]),
            atol=1.0e-8,
        )
        self.assertAlmostEqual(
            model.geom_size[first_bone, 1],
            0.5 * np.linalg.norm(delta),
        )
        np.testing.assert_allclose(
            data.geom_xmat[first_bone].reshape(3, 3)[:, 2],
            delta / np.linalg.norm(delta),
            atol=1.0e-8,
        )
        # 收到真实 IK target/solved 后，目标轴改为控制器实际目标，
        # 不再使用独立 Motive 映射。10mm + 0.1rad 差应被准确报告。
        skeleton._solved_tcp_pose_chest = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        )
        skeleton._target_tcp_pose_chest = np.array([
            0.01, 0.0, 0.0,
            *Rotation.from_rotvec([0.0, 0.0, 0.1]).as_quat(),
        ])
        skeleton._tcp_to_wrist_pose = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        )
        self.assertTrue(skeleton.update_fk_axes(model, data))
        mujoco.mj_forward(model, data)
        self.assertAlmostEqual(skeleton.last_position_error_mm, 10.0, places=6)
        self.assertAlmostEqual(
            skeleton.last_rotation_error_deg,
            np.rad2deg(0.1),
            places=6,
        )

        # 后续单独验证 FK 刷新：冻结当前目标，避免使用刻意构造的
        # stale solved pose 重算 chest→MuJoCo 对齐。
        skeleton._target_tcp_pose_chest = None
        skeleton._solved_tcp_pose_chest = None
        # 手臂关节改变后，当前 FK 轴必须逐帧刷新；目标轴保持不动，
        # 且误差诊断从 0 增大。
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "Joint1_R"
        )
        data.qpos[model.jnt_qposadr[joint_id]] += 0.05
        mujoco.mj_forward(model, data)
        self.assertTrue(skeleton.update_fk_axes(model, data))
        mujoco.mj_forward(model, data)
        self.assertGreater(skeleton.last_position_error_mm, 0.1)
        self.assertGreater(skeleton.last_rotation_error_deg, 0.1)
        new_wrist_origin, new_wrist_rotation = viewer._frame_from_axis_geoms(
            data,
            skeleton._wrist_axis_x_geom_id,
            skeleton._wrist_axis_z_geom_id,
            0.045,
        )
        for axis_index, geom_id in enumerate(skeleton._wrist_axis_ids):
            direction = data.geom_xmat[geom_id].reshape(3, 3)[:, 2]
            origin = (
                data.geom_xpos[geom_id]
                - model.geom_size[geom_id, 1] * direction
            )
            np.testing.assert_allclose(origin, new_wrist_origin, atol=1.0e-8)
            np.testing.assert_allclose(
                direction, new_wrist_rotation[:, axis_index], atol=1.0e-8
            )

        skeleton.close()


class MocapH5ReplayTerminalTest(unittest.TestCase):
    def test_raw_keyboard_preserves_terminal_output_newline_mode(self) -> None:
        master_fd, slave_fd = os.openpty()
        before = termios.tcgetattr(slave_fd)
        stop = threading.Event()
        thread = threading.Thread(
            target=raw_keyboard,
            args=(lambda _value: None, stop),
            kwargs={"fd": slave_fd},
        )
        try:
            thread.start()
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                active = termios.tcgetattr(slave_fd)
                if (
                    not active[3] & termios.ICANON
                    and active[1] == before[1]
                ):
                    break
                time.sleep(0.01)
            else:
                self.fail("raw_keyboard 未进入 raw 输入模式")
            self.assertEqual(active[1], before[1])
            os.write(slave_fd, b"first\n\nsecond\n\n")
            readable, _, _ = select.select([master_fd], [], [], 1.0)
            self.assertEqual(readable, [master_fd])
            rendered = os.read(master_fd, 1024)
            self.assertIn(
                b"first\r\n\r\nsecond\r\n\r\n",
                rendered,
            )
        finally:
            stop.set()
            thread.join(timeout=1.0)
            os.close(master_fd)
            os.close(slave_fd)
        self.assertFalse(thread.is_alive())

    def test_h5_logging_uses_blank_line_message_separator(self) -> None:
        with patch(
            "pico_body_tianji.controller_only."
            "mocap_h5_replay_node.logging.basicConfig"
        ) as configure:
            _configure_logging()
        options = configure.call_args.kwargs
        self.assertTrue(options["force"])
        self.assertEqual(options["handlers"][0].terminator, "\n\n")


if __name__ == "__main__":
    unittest.main()
