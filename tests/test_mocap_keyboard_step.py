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

    def test_right_arm_reference_freezes_then_keyboard_steps(self) -> None:
        node = MocapLiveNode.__new__(MocapLiveNode)
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

    def test_s_rearms_after_home_and_q_exits_after_home(self) -> None:
        node = MocapLiveNode.__new__(MocapLiveNode)
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
        node = MocapLiveNode.__new__(MocapLiveNode)
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
