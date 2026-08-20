from __future__ import annotations

import threading
import time
import unittest

import numpy as np

from pico_body_tianji.controller_frame import ControllerFrame
from pico_body_tianji.controller_only.controller_only_mapper import (
    ControllerOnlyTeleopMapper,
)
from pico_body_tianji.controller_only.mocap_keyboard_step import (
    AXIS_STEPS,
    ArrowKeyParser,
    HoldToRunClock,
    MotiveFrontCircleTrajectory,
    StepAccumulator,
)
from pico_body_tianji.controller_only.mocap_live_node import MocapLiveNode
from pico_body_tianji.controller_only.target_conditioner import (
    TargetConditioningSettings,
)
from tianji_world_output.config_loader import TianjiConfig

_REFERENCE_POSE = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])


class ArrowKeyParserTest(unittest.TestCase):
    def _feed_sequence(self, parser: ArrowKeyParser, sequence: str) -> list:
        events = []
        for byte in sequence:
            event = parser.feed(byte)
            if event is not None:
                events.append(event)
        return events

    def test_escape_sequences_map_to_directions(self) -> None:
        parser = ArrowKeyParser()
        self.assertEqual(self._feed_sequence(parser, "\x1b[A"), ["up"])
        self.assertEqual(self._feed_sequence(parser, "\x1b[B"), ["down"])
        self.assertEqual(self._feed_sequence(parser, "\x1b[C"), ["right"])
        self.assertEqual(self._feed_sequence(parser, "\x1b[D"), ["left"])

    def test_single_bytes_pass_through(self) -> None:
        parser = ArrowKeyParser()
        self.assertEqual(parser.feed("s"), "s")
        self.assertEqual(parser.feed("c"), "c")
        self.assertEqual(parser.feed("1"), "1")
        self.assertEqual(parser.feed("0"), "0")

    def test_interleaved_sequences(self) -> None:
        parser = ArrowKeyParser()
        events = []
        for byte in "\x1b[A1\x1b[D":
            event = parser.feed(byte)
            if event is not None:
                events.append(event)
        self.assertEqual(events, ["up", "1", "left"])

    def test_broken_escape_is_dropped(self) -> None:
        parser = ArrowKeyParser()
        self.assertIsNone(parser.feed("\x1b"))
        self.assertIsNone(parser.feed("X"))  # 非法转义丢弃
        self.assertEqual(parser.feed("s"), "s")  # 状态已复位


class StepAccumulatorTest(unittest.TestCase):
    def test_each_key_steps_10mm_along_axis(self) -> None:
        accumulator = StepAccumulator(
            reference_pose=_REFERENCE_POSE, step_mm=10.0
        )
        pose = accumulator.step("up")
        np.testing.assert_allclose(pose[:3], [0.0, 0.0, 0.01])
        pose = accumulator.step("up")
        np.testing.assert_allclose(pose[:3], [0.0, 0.0, 0.02])
        pose = accumulator.step("left")
        np.testing.assert_allclose(pose[:3], [0.01, 0.0, 0.02])

    def test_all_axis_mappings(self) -> None:
        accumulator = StepAccumulator(
            reference_pose=_REFERENCE_POSE, step_mm=10.0
        )
        expected = {
            "up": [0, 0, 1],
            "down": [0, 0, -1],
            "left": [1, 0, 0],
            "right": [-1, 0, 0],
            "1": [0, 1, 0],
            "0": [0, -1, 0],
        }
        self.assertEqual(set(AXIS_STEPS), set(expected))
        for event, direction in expected.items():
            accumulator.reset()
            pose = accumulator.step(event)
            np.testing.assert_allclose(
                pose[:3], np.asarray(direction) * 0.01, atol=1e-12
            )

    def test_reset_returns_to_reference(self) -> None:
        accumulator = StepAccumulator(
            reference_pose=_REFERENCE_POSE, step_mm=10.0
        )
        accumulator.step("up")
        accumulator.reset()
        np.testing.assert_array_equal(
            accumulator.pose(), _REFERENCE_POSE
        )

    def test_invalid_event_raises(self) -> None:
        accumulator = StepAccumulator(
            reference_pose=_REFERENCE_POSE, step_mm=10.0
        )
        with self.assertRaisesRegex(ValueError, "未知按键"):
            accumulator.step("s")

    def test_custom_step_mm(self) -> None:
        accumulator = StepAccumulator(
            reference_pose=_REFERENCE_POSE, step_mm=25.0
        )
        pose = accumulator.step("1")
        np.testing.assert_allclose(pose[:3], [0.0, 0.025, 0.0])


class MotiveFrontCircleTrajectoryTest(unittest.TestCase):
    def setUp(self) -> None:
        self.trajectory = MotiveFrontCircleTrajectory(
            radius_mm=100.0,
            maximum_speed_mm_s=50.0,
        )

    def test_required_waypoints_and_clockwise_direction(self) -> None:
        start = self.trajectory.sample(0.0)
        np.testing.assert_allclose(start.delta_m, [0.0, 0.0, 0.0])
        self.assertEqual(start.segment, "rise")

        top = self.trajectory.sample(
            self.trajectory.rise_duration_s
        )
        np.testing.assert_allclose(top.delta_m, [0.0, 0.2, 0.0])
        self.assertEqual(top.segment, "circle")

        clockwise = self.trajectory.sample(
            self.trajectory.rise_duration_s
            + 0.25 * self.trajectory.circle_duration_s
        )
        self.assertGreater(clockwise.delta_m[0], 0.0)
        self.assertLess(clockwise.delta_m[1], 0.2)
        self.assertEqual(clockwise.delta_m[2], 0.0)

        origin = self.trajectory.sample(
            self.trajectory.rise_duration_s
            + 0.5 * self.trajectory.circle_duration_s
        )
        np.testing.assert_allclose(
            origin.delta_m, [0.0, 0.0, 0.0], atol=1.0e-12
        )
        self.assertFalse(origin.complete)

        end = self.trajectory.sample(
            self.trajectory.total_duration_s
        )
        np.testing.assert_allclose(
            end.delta_m, [0.0, 0.2, 0.0], atol=1.0e-12
        )
        self.assertEqual(end.segment, "complete")
        self.assertTrue(end.complete)

    def test_path_is_continuous_at_rise_to_circle_transition(self) -> None:
        before = self.trajectory.sample(
            self.trajectory.rise_duration_s - 1.0e-6
        )
        after = self.trajectory.sample(
            self.trajectory.rise_duration_s + 1.0e-6
        )
        self.assertLess(
            float(np.linalg.norm(after.delta_m - before.delta_m)),
            1.0e-9,
        )

    def test_invalid_geometry_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "radius_mm"):
            MotiveFrontCircleTrajectory(radius_mm=0.0)
        with self.assertRaisesRegex(ValueError, "maximum_speed_mm_s"):
            MotiveFrontCircleTrajectory(maximum_speed_mm_s=float("nan"))


class HoldToRunClockTest(unittest.TestCase):
    def test_only_pressed_intervals_advance_and_resume(self) -> None:
        clock = HoldToRunClock()
        self.assertEqual(clock.update(10.0, False), 0.0)
        self.assertEqual(clock.update(11.0, True), 0.0)
        self.assertEqual(clock.update(12.5, True), 1.5)
        self.assertEqual(clock.update(12.5, False), 1.5)
        self.assertEqual(clock.update(20.0, False), 1.5)
        self.assertEqual(clock.update(21.0, True), 1.5)
        self.assertEqual(clock.update(23.0, True), 3.5)

    def test_rejects_non_monotonic_time(self) -> None:
        clock = HoldToRunClock()
        clock.update(10.0, False)
        with self.assertRaisesRegex(ValueError, "不能倒退"):
            clock.update(9.0, True)

    def test_stalled_poll_cannot_advance_more_than_one_cycle(self) -> None:
        clock = HoldToRunClock(maximum_step_s=1.0 / 60.0)
        clock.update(10.0, True)
        elapsed = clock.update(20.0, False)
        self.assertAlmostEqual(elapsed, 1.0 / 60.0)
        self.assertFalse(clock.running)


class _FakeDeadman:
    def __init__(self) -> None:
        self.pressed = False
        self.error: RuntimeError | None = None

    def is_pressed(self) -> bool:
        if self.error is not None:
            raise self.error
        return self.pressed

    def close(self) -> None:
        pass


class _InitializeCapture:
    def __init__(self) -> None:
        self.frame: ControllerFrame | None = None

    def initialize(self, frame: ControllerFrame) -> set[str]:
        self.frame = frame
        return {"pico_left_wrist", "pico_right_wrist"}

class _CapturePublisher:
    def __init__(self) -> None:
        self.values: list[dict] = []

    def put_json(self, value: dict) -> None:
        self.values.append(value)


class MocapLiveReferenceKeyboardTest(unittest.TestCase):
    """机器人末端刚体只用于定零，后续实测随动不得改变虚拟目标。"""

    @staticmethod
    def _node() -> MocapLiveNode:
        node = MocapLiveNode.__new__(MocapLiveNode)
        node._phase_lock = threading.RLock()
        node._rate = 60.0
        node._circle_plan = MotiveFrontCircleTrajectory()
        node._circle_clock = None
        node._circle_sample = None
        node._circle_deadman = _FakeDeadman()
        node._circle_deadman_error = None
        return node

    def test_right_arm_reference_freezes_then_keyboard_steps(self) -> None:
        node = self._node()
        node._parser = ArrowKeyParser()
        node._phase = "armed"
        node._frame_lock = threading.Lock()
        right_reference = np.array(
            [0.41, -0.12, 0.28, 0.0, 0.0, 0.0, 1.0]
        )
        node._latest_frame = {
            "left": _REFERENCE_POSE.copy(),
            "right": right_reference.copy(),
        }
        node._latest_received_monotonic = time.monotonic()
        node._side_pose = lambda frame, side: frame[side].copy()
        node._active_sides = ("right",)
        node._rigid_ids = {
            "left": "left_wrist",
            "right": "right_arm",
        }
        node._step_mm = 10.0
        node._command_lock = threading.Lock()
        node._accumulators = None
        node._mapper = _InitializeCapture()
        node._publish_state = lambda state: None
        node._echo = lambda event: None

        node._on_key("s")
        self.assertEqual(node._phase, "stepping")
        frozen = node._command_frame()
        self.assertIsNotNone(frozen)
        np.testing.assert_array_equal(
            frozen.right_pose, right_reference
        )

        # 模拟机器人运动后，贴在末端的 right_arm 刚体随动 300mm；
        # 虚拟命令必须仍停在按 s 时冻结的参考。
        node._latest_frame["right"][0] += 0.3
        after_feedback = node._command_frame()
        np.testing.assert_array_equal(
            after_feedback.right_pose, right_reference
        )

        for byte in "\x1b[A":
            node._on_key(byte)
        stepped = node._command_frame()
        np.testing.assert_allclose(
            stepped.right_pose[:3],
            right_reference[:3] + np.array([0.0, 0.0, 0.01]),
        )
        np.testing.assert_array_equal(
            stepped.right_pose[3:], right_reference[3:]
        )
        np.testing.assert_array_equal(
            stepped.left_pose, _REFERENCE_POSE
        )

        for sequence in ("\x1b[A", "\x1b[A", "1", "\x1b[D"):
            for byte in sequence:
                node._on_key(byte)
        continuous = node._command_frame()
        self.assertEqual(node._phase, "stepping")
        np.testing.assert_allclose(
            continuous.right_pose[:3],
            right_reference[:3] + np.array([0.01, 0.01, 0.03]),
        )

    def test_c_requires_hold_and_resumes_without_jump(self) -> None:
        node = self._node()
        node._parser = ArrowKeyParser()
        node._echo = lambda event: None
        node._phase = "stepping"
        node._side = "right"
        node._active_sides = ("right",)
        node._command_lock = threading.Lock()
        node._accumulators = {
            side: StepAccumulator(_REFERENCE_POSE, 10.0)
            for side in ("left", "right")
        }

        node._on_key("c")
        self.assertIsNotNone(node._circle_clock)
        self.assertAlmostEqual(
            node._circle_clock.maximum_step_s, 1.0 / 60.0
        )
        # 后续用稀疏关键时刻验证几何；单周期上限由上面的断言及纯时钟
        # 测试覆盖，避免在本测试机械循环 1800 个 60Hz tick。
        node._circle_clock.maximum_step_s = None

        # 未按 Enter 时，无论墙钟过去多久都保持起点。
        self.assertFalse(node._advance_circle(100.0))
        self.assertFalse(node._advance_circle(110.0))
        np.testing.assert_array_equal(
            node._accumulators["right"].delta_m(),
            [0.0, 0.0, 0.0],
        )

        # 第一次按住：推进到 +y 200mm 最高点。
        node._circle_deadman.pressed = True
        self.assertFalse(node._advance_circle(110.0))
        top_at = 110.0 + node._circle_plan.rise_duration_s
        self.assertFalse(node._advance_circle(top_at))
        np.testing.assert_allclose(
            node._accumulators["right"].delta_m(),
            [0.0, 0.2, 0.0],
        )

        # 第一次松开：保持最高点，暂停时间不计入轨迹时钟。
        node._circle_deadman.pressed = False
        self.assertFalse(node._advance_circle(top_at))
        self.assertFalse(node._advance_circle(top_at + 5.0))
        np.testing.assert_allclose(
            node._accumulators["right"].delta_m(),
            [0.0, 0.2, 0.0],
        )

        # 再次按住：从最高点继续半圈到参考零点。
        node._circle_deadman.pressed = True
        resumed_at = top_at + 5.0
        self.assertFalse(node._advance_circle(resumed_at))
        origin_at = (
            resumed_at + 0.5 * node._circle_plan.circle_duration_s
        )
        self.assertFalse(node._advance_circle(origin_at))
        np.testing.assert_allclose(
            node._accumulators["right"].delta_m(),
            [0.0, 0.0, 0.0],
            atol=1.0e-12,
        )

        # 再次松开仍保持零点；第三次按住完成剩余半圈。
        node._circle_deadman.pressed = False
        self.assertFalse(node._advance_circle(origin_at))
        self.assertFalse(node._advance_circle(origin_at + 4.0))
        np.testing.assert_allclose(
            node._accumulators["right"].delta_m(),
            [0.0, 0.0, 0.0],
            atol=1.0e-12,
        )
        node._circle_deadman.pressed = True
        final_resume_at = origin_at + 4.0
        self.assertFalse(node._advance_circle(final_resume_at))
        self.assertTrue(
            node._advance_circle(
                final_resume_at
                + 0.5 * node._circle_plan.circle_duration_s
            )
        )
        self.assertIsNone(node._circle_clock)
        np.testing.assert_allclose(
            node._accumulators["right"].delta_m(),
            [0.0, 0.2, 0.0],
            atol=1.0e-12,
        )
        self.assertTrue(node._circle_sample.complete)

    def test_deadman_read_failure_pauses_without_progress(self) -> None:
        node = self._node()
        node._command_lock = threading.Lock()
        node._accumulators = {
            side: StepAccumulator(_REFERENCE_POSE, 10.0)
            for side in ("left", "right")
        }
        node._circle_clock = HoldToRunClock()
        node._circle_sample = node._circle_plan.sample(0.0)
        node._circle_deadman.error = RuntimeError("X11 disconnected")

        self.assertFalse(node._advance_circle(100.0))
        self.assertFalse(node._advance_circle(120.0))

        np.testing.assert_array_equal(
            node._accumulators["right"].delta_m(),
            [0.0, 0.0, 0.0],
        )
        self.assertEqual(node._circle_clock.elapsed_s, 0.0)
        self.assertEqual(node._circle_deadman_error, "X11 disconnected")

    def test_c_rejects_nonzero_manual_offset(self) -> None:
        node = self._node()
        node._parser = ArrowKeyParser()
        node._echo = lambda event: None
        node._phase = "stepping"
        node._side = "right"
        node._active_sides = ("right",)
        node._command_lock = threading.Lock()
        node._accumulators = {
            side: StepAccumulator(_REFERENCE_POSE, 10.0)
            for side in ("left", "right")
        }
        node._accumulators["right"].step("1")

        node._on_key("c")

        self.assertIsNone(node._circle_clock)
        np.testing.assert_allclose(
            node._accumulators["right"].delta_m(),
            [0.0, 0.01, 0.0],
        )


    def test_teleop_transition_cannot_be_followed_by_stale_idle(self) -> None:
        node = self._node()
        node._parser = ArrowKeyParser()
        node._phase = "armed"
        node._frame_lock = threading.Lock()
        node._latest_frame = {
            "left": _REFERENCE_POSE.copy(),
            "right": _REFERENCE_POSE.copy(),
        }
        node._latest_received_monotonic = time.monotonic()
        node._side_pose = lambda frame, side: frame[side].copy()
        node._active_sides = ("right",)
        node._rigid_ids = {
            "left": "left_wrist",
            "right": "right_arm",
        }
        node._step_mm = 10.0
        node._command_lock = threading.Lock()
        node._accumulators = None
        node._mapper = _InitializeCapture()
        node._echo = lambda event: None

        idle_publish_started = threading.Event()
        release_idle_publish = threading.Event()
        states: list[str] = []
        thread_errors: list[BaseException] = []

        def publish_state(state: str) -> None:
            if state == "idle":
                idle_publish_started.set()
                release_idle_publish.wait(timeout=1.0)
            states.append(state)

        def run_tick() -> None:
            try:
                node._tick()
            except BaseException as exc:
                thread_errors.append(exc)

        def press_start() -> None:
            try:
                node._on_key("s")
            except BaseException as exc:
                thread_errors.append(exc)

        node._publish_state = publish_state
        tick_thread = threading.Thread(target=run_tick, daemon=True)
        tick_thread.start()
        self.assertTrue(idle_publish_started.wait(timeout=1.0))

        key_thread = threading.Thread(target=press_start, daemon=True)
        key_thread.start()
        time.sleep(0.05)
        self.assertTrue(key_thread.is_alive())

        release_idle_publish.set()
        tick_thread.join(timeout=1.0)
        key_thread.join(timeout=1.0)
        self.assertFalse(tick_thread.is_alive())
        self.assertFalse(key_thread.is_alive())
        self.assertEqual(thread_errors, [])
        self.assertEqual(states, ["idle", "teleop"])
        self.assertEqual(node._phase, "stepping")

    def test_s_within_debounce_window_ignored(self) -> None:
        node = self._node()
        node._parser = ArrowKeyParser()
        node._echo = lambda event: None
        node._phase = "stepping"
        node._phase_started = time.monotonic()
        node._return_complete = False
        node._at_home = False
        node._exit_after_return = False
        node._command_lock = threading.Lock()
        node._accumulators = {"left": object(), "right": object()}
        node._last_conditioning = {"left": {}, "right": {}}
        node._publish_state = lambda state: None

        node._on_key("s")
        self.assertEqual(node._phase, "stepping")
        self.assertFalse(node._exit_after_return)

    def test_s_after_debounce_window_returns(self) -> None:
        node = self._node()
        node._parser = ArrowKeyParser()
        node._echo = lambda event: None
        node._phase = "stepping"
        node._phase_started = time.monotonic() - 0.6
        node._return_complete = False
        node._at_home = False
        node._exit_after_return = False
        node._command_lock = threading.Lock()
        node._accumulators = {"left": object(), "right": object()}
        node._last_conditioning = {"left": {}, "right": {}}
        node._publish_state = lambda state: None

        node._on_key("s")
        self.assertEqual(node._phase, "returning")
        self.assertFalse(node._exit_after_return)

    def test_s_rearms_after_home_and_q_exits_after_home(self) -> None:
        node = self._node()
        node._parser = ArrowKeyParser()
        node._echo = lambda event: None
        node._phase = "stepping"
        node._phase_started = time.monotonic() - 0.6
        node._return_complete = False
        node._at_home = False
        node._exit_after_return = False
        node._command_lock = threading.Lock()
        node._accumulators = {"left": object(), "right": object()}
        node._last_conditioning = {"left": {}, "right": {}}
        states = []
        node._publish_state = states.append

        node._on_key("s")
        self.assertEqual(node._phase, "returning")
        self.assertFalse(node._exit_after_return)
        node._return_complete = True
        node._at_home = True
        self.assertTrue(node._tick())
        self.assertEqual(node._phase, "armed")
        self.assertIsNone(node._accumulators)
        self.assertEqual(states[-1], "idle")

        node._phase = "stepping"
        node._accumulators = {"left": object(), "right": object()}
        node._on_key("q")
        self.assertEqual(node._phase, "returning")
        self.assertTrue(node._exit_after_return)
        node._return_complete = True
        node._at_home = True
        self.assertFalse(node._tick())

    def test_status_contains_live_motive_pose_and_valid_idle_state(self) -> None:
        node = self._node()
        node._frame_lock = threading.Lock()
        node._latest_frame = {
            "frame_number": 42,
            "rigid_bodies": [
                {
                    "id": 10,
                    "position": [0.41, -0.12, 0.28],
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                    "tracking_valid": True,
                }
            ],
        }
        node._latest_received_monotonic = time.monotonic()
        node._rigid_body_names = {}
        node._missing_rigid_warned = set()
        node._rigid_ids = {"left": 1, "right": 10}
        node._active_sides = ("right",)
        node._command_lock = threading.Lock()
        node._accumulators = None
        node._phase = "armed"
        node._at_home = True
        node._side = "right"
        node._step_mm = 10.0
        node._last_conditioning = {"left": None, "right": None}
        node._status_pub = _CapturePublisher()

        node._publish_status()

        status = node._status_pub.values[-1]
        self.assertEqual(status["state"], "idle")
        self.assertTrue(status["at_safe_home"])
        observed = status["motive_pose"]["right"]
        self.assertEqual(observed["frame_number"], 42)
        self.assertTrue(observed["tracking_valid"])
        self.assertEqual(observed["position_m"], [0.41, -0.12, 0.28])
        circle = status["circle_trajectory"]
        self.assertFalse(circle["active"])
        self.assertEqual(circle["plane"], "motive_xy")
        self.assertEqual(circle["clockwise_view"], "motive_positive_z")
        self.assertEqual(circle["radius_mm"], 100.0)
        self.assertEqual(circle["top_offset_mm"], 200.0)


class MocapKeyboardStepMappingTest(unittest.TestCase):
    """验收：按一次键（动捕系 10mm）→ 目标位移 10mm（1:1）。"""

    def _mapper(self) -> ControllerOnlyTeleopMapper:
        config = TianjiConfig.load()
        return ControllerOnlyTeleopMapper(
            config,
            rate=60.0,
            min_cutoff=1.2,
            beta=0.45,
            conditioning_settings=TargetConditioningSettings(
                rate_hz=60.0,
                translation_gain=np.ones(3),
                rotation_gain=1.0,
                workspace_relative_radii_m=np.full(3, 10.0),
                workspace_soft_zone_ratio=0.99,
                maximum_linear_speed_m_s=100.0,
                maximum_angular_speed_rad_s=100.0,
                maximum_linear_acceleration_m_s2=10000.0,
                maximum_angular_acceleration_rad_s2=10000.0,
            ),
            default_zsp_directions={
                side: config.get_default_zsp_direction(side)
                for side in ("left", "right")
            },
        )

    def test_one_key_press_maps_to_exactly_step_mm(self) -> None:
        config = TianjiConfig.load()
        mapper = self._mapper()
        accumulator = StepAccumulator(
            reference_pose=_REFERENCE_POSE, step_mm=10.0
        )
        mapper.initialize(
            ControllerFrame.from_poses(
                accumulator.pose(), accumulator.pose()
            )
        )

        def settle(pose) -> None:
            # One-Euro 滤波对 10mm 台阶渐近收敛（时间常数 ~0.13s）：
            # 每次按键后保持 0.5s（30 帧）再记录目标。
            for _ in range(30):
                mapper.map_frame(
                    ControllerFrame.from_poses(pose, pose)
                )

        settled = []
        for _ in range(3):
            pose = accumulator.step("up")
            settle(pose)
            targets = mapper.map_frame(
                ControllerFrame.from_poses(pose, pose)
            )
            settled.append(targets)
        for side in ("left", "right"):
            positions = [
                (targets.left_pose[:3] if side == "left"
                 else targets.right_pose[:3])
                for targets in settled
            ]
            # 动捕 +z → 机器人世界 -x（pico_to_robot）；每次按键
            # 收敛后目标位移恰为 10mm。
            step_distances = [
                float(np.linalg.norm(positions[i + 1] - positions[i]))
                for i in range(len(positions) - 1)
            ]
            for index, distance in enumerate(step_distances):
                self.assertAlmostEqual(
                    distance, 0.010, delta=0.001,
                    msg=f"{side} 第 {index + 1} 次按键目标位移 "
                        f"{distance*1000:.1f}mm ≠ 10mm",
                )


if __name__ == "__main__":
    unittest.main()
