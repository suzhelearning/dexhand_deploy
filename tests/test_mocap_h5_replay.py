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
)
from pico_body_tianji.controller_only.mocap_h5_replay_node import (
    DEFAULT_PARAMETERS,
    MocapH5ReplayNode,
    _configure_logging,
    _configured_pose,
)
from pico_body_tianji.controller_only.raw_keyboard import raw_keyboard
from pico_body_tianji.mujoco_urdf import portable_mujoco_urdf


class _FakePub:
    def __init__(self) -> None:
        self.json_values: list[dict] = []
        self.text_values: list[str] = []

    def put_json(self, value: dict) -> None:
        self.json_values.append(value)

    def put_text(self, value: str) -> None:
        self.text_values.append(value)


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


class _FakeTrajectory:
    duration_s = 0.5
    interpolated_frame_count = 1

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
        node._marker_to_wrist_pose = np.array(
            [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        )
        node._tcp_to_wrist_pose = np.array(
            [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        )
        node._wrist_to_tcp_pose = np.array(
            [-0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        )
        node._right_rigid_id = "right_arm"
        node._rigid_body_names = {7: "right_arm"}
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
        node._stop_event = threading.Event()
        node._recording = SimpleNamespace(
            path="/chosen/trajectory.h5",
            frame_count=3,
            summary=lambda: {"path": "/chosen/trajectory.h5"},
        )
        node._yaw_deg = 0.0
        return node

    def test_default_right_arm_visual_offset_matches_gl_go(self) -> None:
        rigid_to_marker = _configured_pose(
            DEFAULT_PARAMETERS, "right_rigid_to_marker_mocap"
        )
        marker_to_wrist = _configured_pose(
            DEFAULT_PARAMETERS, "right_marker_to_wrist"
        )
        rigid_to_wrist = compose_pose(
            rigid_to_marker, marker_to_wrist
        )

        np.testing.assert_allclose(
            rigid_to_marker,
            [
                -0.003, -0.004, 0.0,
                0.0123407149398269, -0.7069990853988243,
                0.0123407149398269, 0.7069990853988243,
            ],
            atol=1e-9,
        )
        np.testing.assert_allclose(
            rigid_to_wrist,
            [
                -0.0060068974, -0.0038548508, 0.0325,
                0.0174524064, -0.9998476952, 0.0, 0.0,
            ],
            atol=1e-9,
        )

    def test_default_h5_manus_wrist_axes_map_to_wuji2_anatomy(self) -> None:
        transform = _configured_pose(
            DEFAULT_PARAMETERS, "right_h5_wrist_to_wuji2_wrist"
        )
        rotation = Rotation.from_quat(transform[3:7]).as_matrix()
        np.testing.assert_allclose(
            rotation,
            [
                [0.0, 0.0, -1.0],
                [0.0, -1.0, 0.0],
                [-1.0, 0.0, 0.0],
            ],
            atol=1e-9,
        )
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0)

        node = self._node()
        node._h5_wrist_to_wuji2_wrist_pose = transform
        node._right_wrist_home_pose = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        )
        node._virtual_tcp_home_pose = node._right_wrist_home_pose.copy()
        self.assertTrue(node._map_right_pose(node._frame_zero_pose))
        expected_tcp = compose_pose(
            compose_pose(node._frame_zero_pose, transform),
            node._wrist_to_tcp_pose,
        )
        np.testing.assert_allclose(
            node._mapper.mapped_frames[-1].right_pose,
            expected_tcp,
            atol=1e-9,
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
        node._tick(10.1)
        self.assertEqual(node._phase, "ready")
        self.assertEqual(node._mapper.map_count, 2)
        np.testing.assert_allclose(
            node._mapper.mapped_frames[-1].right_pose,
            [0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.assertEqual(len(node._pose_pub.json_values), 2)

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
        node._tick(start + 20.25)
        self.assertEqual(node._phase, "replaying")
        self.assertTrue(node._source_complete)
        self.assertEqual(node._final_stable_ticks, 1)
        node._tick(start + 20.5)
        self.assertEqual(node._phase, "completed")
        self.assertAlmostEqual(node._current_source_elapsed_s, 0.5)
        map_count_at_completion = node._mapper.map_count
        node._tick(start + 20.75)
        self.assertEqual(node._mapper.map_count, map_count_at_completion)

        node._last_s_at = -float("inf")
        node._on_key("s")
        self.assertEqual(node._phase, "returning")
        node._tick(start + 21.0)
        self.assertEqual(node._phase, "returning")
        node._return_complete = True
        node._at_home = True
        node._tick(start + 21.1)
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
            "motive_rigid_offset_absolute_wrist_tcp_v5",
        )
        self.assertEqual(status["endpoint"], "wuji2_r_wrist")

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

    def test_motive_frame_zero_uses_home_calibrated_world_transform(self) -> None:
        viewer = self._viewer_module()
        root = Path(__file__).resolve().parents[1]
        urdf = (
            root
            / "src/pico_body_tianji/assets/marvin_m6_ccs/urdf"
            / "marvin_m6_s_ccs_696_v4_wuji2.urdf"
        )
        xml, assets = portable_mujoco_urdf(urdf)
        model = mujoco.MjModel.from_xml_string(
            viewer._add_frame_zero_skeleton(xml),
            assets,
        )
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        session = _FakeZenohSession()
        skeleton = viewer.FrameZeroHandSkeleton(
            session,
            model,
            "/pico_body_sim/frame0_hand_skeleton",
        )

        wrist_position_sim, wrist_rotation_sim = skeleton._wrist_frame_mj(
            data
        )
        wrist_position_motive = np.array([1.0, 2.0, 3.0])
        wrist_rotation_motive = Rotation.from_euler(
            "xyz", [0.35, -0.2, 0.65]
        ).as_matrix()
        wrist_quaternion_motive = Rotation.from_matrix(
            wrist_rotation_motive
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
        rotation_sim_from_motive = (
            wrist_rotation_sim @ wrist_rotation_motive.T
        )
        points_sim = (
            (points_motive - wrist_position_motive)
            @ rotation_sim_from_motive.T
            + wrist_position_sim
        )
        payload = {
            "frozen": False,
            "source_frame_index": 0,
            "edges": [list(edge) for edge in HAND_KEYPOINT_EDGES],
            "points_motive_world": points_motive.tolist(),
            "home_wuji2_wrist_pose_motive": [
                *wrist_position_motive.tolist(),
                *wrist_quaternion_motive.tolist(),
            ],
        }
        skeleton._on_skeleton(payload)
        self.assertTrue(skeleton.apply_latest(model, data))
        mujoco.mj_forward(model, data)

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
