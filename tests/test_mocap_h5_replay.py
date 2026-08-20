from __future__ import annotations

import os
import select
import termios
import threading
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np

from pico_body_tianji.controller_only.mocap_h5 import HandTrajectorySample
from pico_body_tianji.controller_only.mocap_h5_replay_node import (
    MocapH5ReplayNode,
    _configure_logging,
)
from pico_body_tianji.controller_only.raw_keyboard import raw_keyboard


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
        node._left_reference_pose = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
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
        node._right_arm_home_pose = None
        node._current_motive_target_pose = None
        node._cached_targets = None
        node._approach_stable_ticks = 0
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
        node._pose_pub = _FakePub()
        node._elbow_pub = _FakePub()
        node._state_pub = _FakePub()
        node._status_pub = _FakePub()
        node._stop_event = threading.Event()
        node._recording = SimpleNamespace(
            frame_count=3,
            summary=lambda: {"path": "/chosen/trajectory.h5"},
        )
        node._yaw_deg = 0.0
        return node

    def test_enter_gates_approach_and_replay_then_s_returns_home(self) -> None:
        node = self._node()

        node._on_key("s")
        self.assertEqual(node._phase, "approaching")
        self.assertTrue(node._mapper.initialized)
        np.testing.assert_allclose(
            node._mapper.initialized_frame.right_pose,
            [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0],
        )

        # 未按 Enter：目标不前进，也没有目标可发布。
        node._tick(10.0)
        self.assertEqual(node._mapper.map_count, 0)
        self.assertEqual(node._approach_stable_ticks, 0)

        # 按住 Enter 两个稳定周期，到达轨迹 0 帧；只发布右臂目标。
        node._deadman.pressed = True
        node._tick(10.0)
        node._tick(10.1)
        self.assertEqual(node._phase, "ready")
        self.assertEqual(node._mapper.map_count, 2)
        np.testing.assert_allclose(
            node._mapper.mapped_frames[-1].right_pose,
            [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.assertEqual(len(node._pose_pub.json_values), 2)

        # r 前必须先松开 Enter。
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
        np.testing.assert_allclose(
            node._mapper.mapped_frames[-1].right_pose[:3],
            [0.45, 0.0, 0.0],
        )

        # 松开后等待任意时长，源轨迹时间不增加。
        node._deadman.pressed = False
        node._tick(start + 0.25)
        node._tick(start + 20.0)
        self.assertAlmostEqual(node._current_source_elapsed_s, 0.25)
        self.assertEqual(node._phase, "replaying")

        # 再按继续；源时间到末帧后仍须在 Enter 保压下稳定收敛，
        # 然后进入 completed 并保持，不自动回 Home。
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

    def test_tracking_status_reports_actual_to_absolute_target_error(
        self,
    ) -> None:
        node = self._node()
        node._right_arm_home_pose = np.array(
            [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]
        )
        node._current_motive_target_pose = np.array(
            [1.01, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]
        )
        status = node._motive_tracking_status(time.monotonic())
        self.assertTrue(status["tracking_valid"])
        self.assertEqual(status["resolved_id"], 7)
        self.assertAlmostEqual(status["position_error_m"], 0.01)
        self.assertAlmostEqual(status["orientation_error_deg"], 0.0)

    def test_armed_tick_samples_deadman_without_moving(self) -> None:
        node = self._node()
        node._deadman.pressed = True

        self.assertTrue(node._tick(time.monotonic()))

        self.assertEqual(node._phase, "armed")
        self.assertTrue(node._deadman_pressed)
        self.assertEqual(node._mapper.map_count, 0)
        self.assertEqual(node._state_pub.text_values[-1], "idle")
        self.assertTrue(node._status_pub.json_values[-1]["deadman_pressed"])

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
