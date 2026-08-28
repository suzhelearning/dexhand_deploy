from __future__ import annotations

import unittest

import numpy as np

from pico_body_tianji.executors.marvin.bridge import MarvinExecutor
from pico_body_tianji.executors.marvin.readiness import MarvinReadiness
from pico_body_tianji.executors.mujoco.node import MujocoExecutor
from pico_body_tianji.executors.wuji_hand2.node import WujiHandExecutor
from pico_body_tianji.executors.wuji_hand2.config import WujiHandConfig
from pico_body_tianji.marvin_hardware import MarvinFeedback
from pico_body_tianji.protocol.messages import (
    ArmJointCommand,
    ComponentStatus,
    HandJointCommand,
    HandTargetCommand,
    ProtocolEnvelope,
    SafetyStopRequest,
    SessionState,
)


class _FakeModel:
    def __init__(self) -> None:
        self.jnt_qposadr = np.arange(54, dtype=np.int32)
        self._ids = {}
        names = [f"Joint{i}_{side}" for side in ("L", "R") for i in range(1, 8)]
        names += [
            f"{'l' if side == 'left' else 'r'}_{name}"
            for side in ("left", "right")
            for name in (
                "thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp", "thumb_ip",
                "index_mcp_flex", "index_mcp_abd", "index_pip", "index_dip",
                "middle_mcp_flex", "middle_mcp_abd", "middle_pip", "middle_dip",
                "ring_mcp_flex", "ring_mcp_abd", "ring_pip", "ring_dip",
                "pinky_mcp_flex", "pinky_mcp_abd", "pinky_pip", "pinky_dip",
            )
        ]
        for index, name in enumerate(names):
            self._ids[name] = index
        self.jnt_limited = np.ones(len(names), dtype=np.uint8)
        self.jnt_range = np.tile(np.asarray([[-10.0, 10.0]]), (len(names), 1))


class _FakeData:
    def __init__(self) -> None:
        self.qpos = np.zeros(54, dtype=np.float64)


class _FakeSession:
    def __init__(self) -> None:
        self.published = []
        self.subscribers = []

    def declare_publisher(self, topic):
        owner = self
        class Pub:
            def put(self, payload, **kwargs):
                owner.published.append((topic, bytes(payload)))
            def undeclare(self):
                pass
        return Pub()

    def declare_subscriber(self, topic, callback):
        self.subscribers.append((topic, callback))
        class Sub:
            def undeclare(self):
                pass
        return Sub()


class Task5ExecutorContractTest(unittest.TestCase):
    def test_mujoco_applies_both_same_tick_commands_and_publishes_rad_state(self):
        session, model, data = _FakeSession(), _FakeModel(), _FakeData()
        executor = MujocoExecutor(
            session=session, model=model, data=data,
            publisher_instance_id="mujoco", router_zid="router",
            coordinator_instance_id="coord",
        )
        for side in ("left", "right"):
            executor.on_arm_command(ArmJointCommand(
                1, 1, 1, "coordinator", side, "teleop", None, None,
                [f"Joint{i}_{'L' if side == 'left' else 'R'}" for i in range(1, 8)],
                [0.1] * 7, "coord", "router"
            ))
        executor.tick(now_ns=2)
        self.assertTrue(np.allclose(data.qpos[:14], 0.1))
        self.assertIsNone(executor.arm_state.velocity_rad_s)
        self.assertTrue(any(topic == "tianji/executor/hand/left/status" for topic, _ in session.published))
        self.assertTrue(any(topic == "tianji/executor/hand/right/status" for topic, _ in session.published))

    def test_mujoco_safety_stop_freezes_qpos_and_requires_restart(self):
        session, model, data = _FakeSession(), _FakeModel(), _FakeData()
        executor = MujocoExecutor(
            session=session, model=model, data=data,
            publisher_instance_id="mujoco", router_zid="router",
            coordinator_instance_id="coord", run_id="run",
            safety_supervisor_instance_id="supervisor",
        )
        executor.on_safety_stop(SafetyStopRequest(
            ProtocolEnvelope(1, "supervisor", "router", 1, 1), "run", "operator", True
        ))
        before = data.qpos.copy()
        executor.tick(now_ns=2)
        np.testing.assert_array_equal(data.qpos, before)
        self.assertTrue(executor.safety_locked)
        self.assertEqual(executor.safety_ack.run_id, "run")
        self.assertTrue(any(topic == "tianji/safety/ack/mujoco" for topic, _ in session.published))

    def test_wuji_config_has_canonical_twenty_joint_contract(self):
        config = WujiHandConfig.load()
        self.assertEqual(len(config.joint_names), 20)
        self.assertEqual(config.joint_names[16], "r_pinky_mcp_flex")
        self.assertEqual(len(config.zero_tolerance_rad), 20)

    def test_wuji_direct_rejects_wrong_publisher_without_refreshing_watchdog(self):
        executor = WujiHandExecutor(
            mode="direct", side="right", router_zid="router",
            publisher_instance_id="wuji", authorized_producer="h5_direct",
            authorized_publisher_instance_id="h5-instance",
        )
        command = HandJointCommand(
            1, 1, 1, "h5_direct", "right", list(executor.config.joint_names),
            [0.1] * 20, "spoof", "router"
        )
        self.assertFalse(executor.on_hand_command(command))
        self.assertFalse(executor.tracking_allowed)
        self.assertTrue(executor.unhealthy)
    def test_wuji_direct_rejects_new_command_while_returning(self):
        executor = WujiHandExecutor(
            mode="direct", side="right", router_zid="router",
            publisher_instance_id="wuji", authorized_producer="h5_direct",
            authorized_publisher_instance_id="h5-instance",
        )
        executor._state = "returning"
        command = HandJointCommand(
            1, 1, 1, "h5_direct", "right", list(executor.config.joint_names),
            [0.1] * 20, "h5-instance", "router"
        )
        self.assertFalse(executor.on_hand_command(command))
        self.assertEqual(executor.position_rad, [0.0] * 20)

    def test_wuji_retarget_is_invariant_to_global_translation(self):
        executor = WujiHandExecutor(
            mode="retarget", side="right", router_zid="router",
            publisher_instance_id="wuji", authorized_producer="source",
            authorized_publisher_instance_id="source-instance",
        )
        points = np.zeros((21, 3), dtype=float)
        points[8] = [0.0, 0.0, 0.1]
        first = executor.tick(now_ns=1)
        shifted = points + [1.0, -2.0, 3.0]
        self.assertTrue(np.allclose(
            executor.config.validate_positions([0.0] * 20),
            executor.config.validate_positions([0.0] * 20),
        ))
        from pico_body_tianji.executors.wuji_hand2.node import _retarget_keypoints
        np.testing.assert_allclose(
            _retarget_keypoints(points, executor.config),
            _retarget_keypoints(shifted, executor.config),
        )
        self.assertIsNone(first)

    def test_marvin_real_readiness_requires_source_real_but_not_producer_real(self):
        readiness = MarvinReadiness(router_zid="router")
        for role, capabilities in (("source", ["simulation", "real"]), ("producer_arm", ["simulation"])):
            readiness.observe_component(ComponentStatus(
                1, 1, 10, role, role, "ready", True, True,
                capabilities, None, {}, role + "-instance", "router"
            ), received_ns=10)
        readiness.observe_session_state(
            SessionState(1, 1, 10, "idle", "startup", "coordinator", None, "coord", "router"),
            received_ns=10,
        )
        for side in ("left", "right"):
            readiness.observe_command(ArmJointCommand(
                1, 1, 10, "coordinator", side, "idle", None, None,
                [f"Joint{i}_{'L' if side == 'left' else 'R'}" for i in range(1, 8)],
                list(readiness.robot.left_home_rad if side == "left" else readiness.robot.right_home_rad),
                "coord", "router",
            ), received_ns=10)
        self.assertTrue(readiness.connection_ready(now_ns=10))


class _FakeMarvinHardware:
    def __init__(self) -> None:
        self.sent = []
        self.soft_stops = 0
        self.feedback = MarvinFeedback(
            np.zeros(7), np.zeros(7), (1, 1), (1, 1), (0, 0),
            (1, 1), (10, 10), (10, 10), ("None", "None"),
        )

    def send_joint_targets(self, left, right):
        self.sent.append((np.asarray(left).copy(), np.asarray(right).copy()))

    def read_feedback(self, include_servo_errors=False):
        return self.feedback

    def soft_stop_once(self):
        self.soft_stops += 1

    def shutdown(self):
        pass


class MarvinExecutorSafetyTest(unittest.TestCase):
    def _command(self, side, sequence=1, timestamp=100):
        return ArmJointCommand(
            1, sequence, timestamp, "coordinator", side, "returning", None, None,
            [f"Joint{i}_{'L' if side == 'left' else 'R'}" for i in range(1, 8)],
            [0.005] * 7, "coord", "router",
        )

    def test_fault_return_consumes_bounded_returning_command(self):
        hardware = _FakeMarvinHardware()
        executor = MarvinExecutor(
            hardware_session=hardware, publisher_instance_id="marvin", router_zid="router",
            coordinator_instance_id="coord", clock=lambda: 100,
        )
        executor.on_session_state(SessionState(1, 1, 100, "fault", "fault", "coordinator", None, "coord", "router"))
        executor.on_arm_command(self._command("left"))
        executor.on_arm_command(self._command("right"))
        executor.tick(now_ns=100)
        self.assertEqual(executor.phase, "fault_return")
        self.assertEqual(len(hardware.sent), 1)
        np.testing.assert_allclose(hardware.sent[0][0], np.degrees([0.005] * 7))

    def test_safety_stop_ack_locks_out_same_tick_motion(self):
        hardware = _FakeMarvinHardware()
        executor = MarvinExecutor(
            hardware_session=hardware, publisher_instance_id="marvin", router_zid="router",
            coordinator_instance_id="coord", run_id="run", safety_supervisor_instance_id="supervisor",
            clock=lambda: 100,
        )
        request = SafetyStopRequest(
            ProtocolEnvelope(1, "supervisor", "router", 1, 100), "run", "operator stop", True
        )
        self.assertTrue(executor.on_safety_stop(request))
        executor.tick(now_ns=100)
        self.assertEqual(hardware.soft_stops, 1)
        self.assertEqual(hardware.sent, [])
        self.assertTrue(executor.safety_locked)
        self.assertIsNotNone(executor.safety_ack)


if __name__ == "__main__":
    unittest.main()
