from __future__ import annotations

import json
import time
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
    LatchedBool,
    ProtocolEnvelope,
    SafetyStopRequest,
    SessionState,
)
from pico_body_tianji.sources.common.real_admission import RealCapabilityInput


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


class _Clock:
    def __init__(self, value):
        self.value = int(value)

    def __call__(self):
        return self.value


class _FakeLiveSession(_FakeSession):
    def __init__(self):
        super().__init__()
        self.tokens = []

    def liveliness(self):
        owner = self

        class Live:
            def declare_token(self, key):
                token = type("Token", (), {"key": key, "undeclare": lambda self: None})()
                owner.tokens.append(token)
                return token

        return Live()


class _ReplySample:
    def __init__(self, payload):
        self.payload = json.dumps(payload, separators=(",", ":")).encode("utf-8")


class _Reply:
    """Shape of eclipse-zenoh 1.10 successful query replies."""

    def __init__(self, payload, ok=True):
        self.ok = ok
        self.result = _ReplySample(payload) if ok else None


class _SnapshotSession(_FakeSession):
    def __init__(self, replies):
        super().__init__()
        self.replies = dict(replies)
        self.queries = []

    def get(self, key, callback, **kwargs):
        self.queries.append(key)
        if key in self.replies:
            reply = self.replies[key]
            if isinstance(reply, list):
                for value in reply:
                    callback(_Reply(value))
            else:
                callback(_Reply(reply))


class _DeferredSnapshotSession(_SnapshotSession):
    def __init__(self, replies):
        super().__init__(replies)
        self.pending = {}

    def get(self, key, callback, **kwargs):
        self.queries.append(key)
        self.pending[key] = callback

    def deliver(self, key):
        reply = self.replies[key]
        self.pending[key](_Reply(reply))

class Task5ExecutorContractTest(unittest.TestCase):
    def test_mujoco_snapshot_barrier_requires_typed_replies(self):
        clock = _Clock(time.monotonic_ns())
        now = clock()
        valid_state = SessionState(1, 1, now, "idle", "startup", "coordinator", None, "coord", "router").to_dict()
        valid_latch = LatchedBool(1, 1, now, True, "coord", "router").to_dict()
        session = _SnapshotSession({
            "tianji/session/state": {"not": "a typed state"},
            "tianji/coordinator/at_home": valid_latch,
            "tianji/coordinator/return_complete": valid_latch | {"value": False},
        })
        executor = MujocoExecutor(
            session=session, model=_FakeModel(), data=_FakeData(),
            publisher_instance_id="mujoco", router_zid="router",
            coordinator_instance_id="coord", clock=clock,
        )
        self.assertFalse(executor._snapshot_ready)
        self.assertFalse(executor.status.ready)

    def test_mujoco_snapshot_barrier_retries_missing_key(self):
        clock = _Clock(time.monotonic_ns())
        now = clock()
        session = _SnapshotSession({
            "tianji/session/state": SessionState(
                1, 1, now, "idle", "startup", "coordinator", None, "coord", "router"
            ).to_dict(),
            "tianji/coordinator/at_home": LatchedBool(
                1, 1, now, True, "coord", "router"
            ).to_dict(),
        })
        executor = MujocoExecutor(
            session=session, model=_FakeModel(), data=_FakeData(),
            publisher_instance_id="mujoco", router_zid="router",
            coordinator_instance_id="coord", clock=clock,
        )
        self.assertFalse(executor._snapshot_ready)
        initial_queries = len(session.queries)
        clock.value += 2_000_000_000
        executor.tick(now_ns=clock())
        self.assertGreater(len(session.queries), initial_queries)
        self.assertFalse(executor._snapshot_ready)
    def test_mujoco_snapshot_barrier_unlocks_only_after_three_valid_replies(self):
        clock = _Clock(time.monotonic_ns())
        now = clock()
        session = _SnapshotSession({
            "tianji/session/state": SessionState(
                1, 1, now, "idle", "startup", "coordinator", None, "coord", "router"
            ).to_dict(),
            "tianji/coordinator/at_home": LatchedBool(
                1, 1, now, True, "coord", "router"
            ).to_dict(),
            "tianji/coordinator/return_complete": LatchedBool(
                1, 1, now, False, "coord", "router"
            ).to_dict(),
        })
        executor = MujocoExecutor(
            session=session, model=_FakeModel(), data=_FakeData(),
            publisher_instance_id="mujoco", router_zid="router",
            coordinator_instance_id="coord", clock=clock,
        )
        self.assertTrue(executor._snapshot_ready)
        self.assertTrue(executor.status.ready)
    def test_mujoco_snapshot_does_not_overwrite_newer_subscriber_state(self):
        clock = _Clock(time.monotonic_ns())
        now = clock()
        state = SessionState(
            1, 1, now, "idle", "startup", "coordinator", None, "coord", "router"
        ).to_dict()
        newer = SessionState(
            1, 2, now, "teleop", "start", "coordinator", 1, "coord", "router"
        )
        session = _DeferredSnapshotSession({
            "tianji/session/state": state,
            "tianji/coordinator/at_home": LatchedBool(
                1, 1, now, True, "coord", "router"
            ).to_dict(),
            "tianji/coordinator/return_complete": LatchedBool(
                1, 1, now, False, "coord", "router"
            ).to_dict(),
        })
        executor = MujocoExecutor(
            session=session, model=_FakeModel(), data=_FakeData(),
            publisher_instance_id="mujoco", router_zid="router",
            coordinator_instance_id="coord", clock=clock,
        )
        state_callback = next(
            callback for topic, callback in session.subscribers
            if topic == "tianji/session/state"
        )
        state_callback(newer)
        for key in (
            "tianji/session/state",
            "tianji/coordinator/at_home",
            "tianji/coordinator/return_complete",
        ):
            session.deliver(key)
        self.assertTrue(executor._snapshot_ready)
        self.assertEqual(executor._session_state.sequence, 2)
        self.assertEqual(executor._session_state.state, "teleop")
    def test_mujoco_retry_keeps_newer_subscriber_baseline(self):
        clock = _Clock(time.monotonic_ns())
        now = clock()
        old_state = SessionState(
            1, 1, now, "idle", "startup", "coordinator", None, "coord", "router"
        ).to_dict()
        session = _DeferredSnapshotSession({
            "tianji/session/state": old_state,
            "tianji/coordinator/at_home": LatchedBool(
                1, 1, now, True, "coord", "router"
            ).to_dict(),
            "tianji/coordinator/return_complete": LatchedBool(
                1, 1, now, False, "coord", "router"
            ).to_dict(),
        })
        executor = MujocoExecutor(
            session=session, model=_FakeModel(), data=_FakeData(),
            publisher_instance_id="mujoco", router_zid="router",
            coordinator_instance_id="coord", clock=clock,
        )
        state_callback = next(
            callback for topic, callback in session.subscribers
            if topic == "tianji/session/state"
        )
        state_callback(SessionState(
            1, 2, now, "teleop", "start", "coordinator", 1, "coord", "router"
        ))
        clock.value += executor._snapshot_timeout_ns
        executor.tick(now_ns=clock())
        session.deliver("tianji/session/state")
        self.assertEqual(executor._session_state.sequence, 2)
        self.assertEqual(executor._session_state.state, "teleop")



    def test_wuji_role_status_ids_match_liveliness_logical_ids(self):
        session = _FakeLiveSession()
        WujiHandExecutor(
            mode="retarget", side="right", session=session,
            router_zid="router", publisher_instance_id="wuji",
            authorized_producer="h5-hand", authorized_publisher_instance_id="h5-instance",
            coordinator_instance_id="coord",
        )
        self.assertEqual(
            {token.key for token in session.tokens},
            {
                "tj/live/executor/hand/wuji_right/wuji",
                "tj/live/producer/hand/h5-hand/wuji",
            },
        )
        component_statuses = [
            json.loads(payload)
            for topic, payload in session.published
            if topic == "tianji/executor/status"
        ]
        self.assertEqual(
            {(value["component_role"], value["component_id"]) for value in component_statuses},
            {("producer_hand", "h5-hand"), ("executor_hand", "wuji_right")},
        )

    def test_mujoco_snapshot_barrier_rejects_duplicate_key_reply(self):
        clock = _Clock(time.monotonic_ns())
        now = clock()
        state = SessionState(
            1, 1, now, "idle", "startup", "coordinator", None, "coord", "router"
        ).to_dict()
        latch = LatchedBool(1, 1, now, True, "coord", "router").to_dict()
        session = _SnapshotSession({
            "tianji/session/state": [state, state],
            "tianji/coordinator/at_home": latch,
            "tianji/coordinator/return_complete": latch | {"value": False},
        })
        executor = MujocoExecutor(
            session=session, model=_FakeModel(), data=_FakeData(),
            publisher_instance_id="mujoco", router_zid="router",
            coordinator_instance_id="coord", clock=clock,
        )
        self.assertFalse(executor._snapshot_ready)

    def test_wuji_real_session_constructor_keeps_subscriptions_and_tokens(self):
        session = _FakeLiveSession()
        executor = WujiHandExecutor(
            mode="retarget", side="right", session=session,
            router_zid="router", publisher_instance_id="wuji",
            authorized_producer="source",
            authorized_publisher_instance_id="source-instance",
            coordinator_instance_id="coord",
        )
        self.assertEqual(len(executor._subscriptions), 3)
        self.assertEqual(len(executor._live_tokens), 2)
        executor.close()
        self.assertEqual(executor._subscriptions, [])

    def test_wuji_accepts_input_only_during_fresh_teleop(self):
        clock = _Clock(1_000_000_000)
        executor = WujiHandExecutor(
            mode="direct", side="right", router_zid="router",
            publisher_instance_id="wuji", authorized_producer="h5_direct",
            authorized_publisher_instance_id="h5-instance",
            coordinator_instance_id="coord", clock=clock,
        )
        state = SessionState(
            1, 1, clock(), "teleop", "start", "coordinator", 1, "coord", "router"
        )
        executor.on_session_state(state)
        command = HandJointCommand(
            1, 1, clock(), "h5_direct", "right",
            list(executor.config.joint_names), [0.1] * 20, "h5-instance", "router"
        )
        self.assertTrue(executor.on_hand_command(command))
        executor.tick(now_ns=clock())
        self.assertNotEqual(executor.position_rad, [0.0] * 20)
        clock.value += executor.command_timeout_ns + 1
        executor.tick(now_ns=clock())
        self.assertFalse(executor.tracking_allowed)
        self.assertTrue(executor.unhealthy)
        self.assertEqual(executor._state, "returning")


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
    def test_mujoco_hand_overlay_consumes_commands_without_hand_authority(self):
        session, model, data = _FakeSession(), _FakeModel(), _FakeData()
        executor = MujocoExecutor(
            session=session, model=model, data=data,
            publisher_instance_id="mujoco", router_zid="router",
            coordinator_instance_id="coord", hand_sides=("right",),
            hand_overlay=True,
        )
        command = HandJointCommand(
            1, 1, 1, "wuji_retarget_right", "right",
            list(executor.hand_config.joint_names), [0.1] * 20, "wuji-instance", "router"
        )
        self.assertTrue(executor.on_hand_command(command))
        executor.tick(now_ns=2)
        self.assertTrue(np.allclose(data.qpos[34:54], 0.1))
        published_topics = {topic for topic, _ in session.published}
        self.assertNotIn("tianji/state/hand/right", published_topics)
        self.assertNotIn("tianji/executor/hand/right/status", published_topics)
        self.assertIn("tianji/executor/status", published_topics)


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
    def test_marvin_readiness_allows_bounded_reconnect_in_returning(self):
        readiness = MarvinReadiness(router_zid="router")
        readiness.observe_session_state(
            SessionState(1, 1, 10, "returning", "disconnect", "coordinator", None, "coord", "router"),
            received_ns=10,
        )
        for side in ("left", "right"):
            readiness.observe_command(ArmJointCommand(
                1, 1, 10, "coordinator", side, "returning", None, None,
                [f"Joint{i}_{'L' if side == 'left' else 'R'}" for i in range(1, 8)],
                [0.0] * 7, "coord", "router",
            ), received_ns=10)
        self.assertTrue(readiness.fault_return_ready(now_ns=10))


class _FakeMarvinHardware:
    def __init__(self) -> None:
        self.sent = []
        self.soft_stops = 0
        self.connect_calls = 0
        self.feedback = MarvinFeedback(
            np.zeros(7), np.zeros(7), (1, 1), (1, 1), (0, 0),
            (1, 1), (10, 10), (10, 10), ("None", "None"),
        )
    def connect_and_prepare(self, *args, **kwargs):
        self.connect_calls += 1
        raise AssertionError("SDK connect must be blocked after SafetyStop")

    def send_joint_targets(self, left, right):
        self.sent.append((np.asarray(left).copy(), np.asarray(right).copy()))

    def read_feedback(self, include_servo_errors=False):
        return self.feedback

    def soft_stop_once(self):
        self.soft_stops += 1

    def shutdown(self):
        pass

class _ConnectableMarvinHardware(_FakeMarvinHardware):
    def __init__(self) -> None:
        super().__init__()
        self.home_calls = 0

    def connect_and_prepare(self, *args, **kwargs):
        self.connect_calls += 1
        return self.feedback

    def move_to_home(self, *args, **kwargs):
        self.home_calls += 1



class MarvinExecutorSafetyTest(unittest.TestCase):
    def _command(self, side, sequence=1, timestamp=100, value=0.005, mode="returning"):
        return ArmJointCommand(
            1, sequence, timestamp, "coordinator", side, mode, None, None,
            [f"Joint{i}_{'L' if side == 'left' else 'R'}" for i in range(1, 8)],
            [value] * 7, "coord", "router",
        )
    def test_admitted_real_capability_allows_normal_connect(self):
        hardware = _ConnectableMarvinHardware()
        executor = MarvinExecutor(
            hardware_session=hardware, publisher_instance_id="marvin",
            router_zid="router", coordinator_instance_id="coord",
            real_capability=RealCapabilityInput(0.1, 0.0, True, True),
            clock=lambda: 100, params={"connection_wait_s": 0.01},
        )
        executor._readiness.connection_ready = lambda now_ns: True
        self.assertTrue(executor._admission_ok())
        self.assertTrue(executor.connect())
        self.assertEqual(executor.phase, "armed_idle")
        self.assertEqual(hardware.connect_calls, 1)
        self.assertEqual(hardware.home_calls, 1)

    def test_returning_reconnect_can_rearm_but_fault_reconnect_stays_latched(self):
        returning_hardware = _ConnectableMarvinHardware()
        returning = MarvinExecutor(
            hardware_session=returning_hardware, publisher_instance_id="marvin",
            router_zid="router", coordinator_instance_id="coord",
            clock=lambda: 100,
        )
        returning.on_session_state(SessionState(
            1, 1, 100, "returning", "disconnect", "coordinator", None, "coord", "router"
        ))
        returning.on_arm_command(self._command("left"))
        returning.on_arm_command(self._command("right"))
        self.assertTrue(returning.connect())
        self.assertEqual(returning.phase, "returning")
        returning.on_session_state(SessionState(
            1, 2, 100, "idle", "returned", "coordinator", None, "coord", "router"
        ))
        self.assertEqual(returning.phase, "armed_idle")

        fault_hardware = _ConnectableMarvinHardware()
        fault = MarvinExecutor(
            hardware_session=fault_hardware, publisher_instance_id="marvin",
            router_zid="router", coordinator_instance_id="coord",
            clock=lambda: 100,
        )
        fault.on_session_state(SessionState(
            1, 1, 100, "fault", "fault", "coordinator", None, "coord", "router"
        ))
        fault.on_arm_command(self._command("left"))
        fault.on_arm_command(self._command("right"))
        self.assertTrue(fault.connect())
        self.assertEqual(fault.phase, "fault_return")
        fault.on_session_state(SessionState(
            1, 2, 100, "idle", "reboot-required", "coordinator", None, "coord", "router"
        ))
        self.assertEqual(fault.phase, "fault_return")


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
    def test_safety_stop_rejects_same_process_reconnect_before_sdk(self):
        hardware = _FakeMarvinHardware()
        executor = MarvinExecutor(
            hardware_session=hardware, publisher_instance_id="marvin", router_zid="router",
            coordinator_instance_id="coord", run_id="run",
            safety_supervisor_instance_id="supervisor", clock=lambda: 100,
        )
        executor._readiness.fault_return_ready = lambda now_ns: True
        self.assertTrue(executor.on_safety_stop(SafetyStopRequest(
            ProtocolEnvelope(1, "supervisor", "router", 1, 100),
            "run", "operator stop", True,
        )))
        self.assertFalse(executor.connect())
        self.assertEqual(hardware.connect_calls, 0)
        self.assertEqual(hardware.sent, [])

    def test_marvin_controller_slews_large_bounded_return_target(self):
        hardware = _FakeMarvinHardware()
        executor = MarvinExecutor(
            hardware_session=hardware, publisher_instance_id="marvin", router_zid="router",
            coordinator_instance_id="coord", clock=lambda: 100,
        )
        executor.on_session_state(SessionState(
            1, 1, 100, "fault", "fault", "coordinator", None, "coord", "router"
        ))
        executor.on_arm_command(self._command("left", value=0.2))
        executor.on_arm_command(self._command("right", value=0.2))
        executor.tick(now_ns=100)
        self.assertEqual(len(hardware.sent), 1)
        np.testing.assert_allclose(
            hardware.sent[0][0],
            np.full(7, executor.params["maximum_output_step_deg"]),
        )
        self.assertFalse(executor.safety_locked)

    def test_duplicate_feedback_frame_waits_for_controller_timeout(self):
        base = time.monotonic_ns()
        clock = _Clock(base)
        hardware = _FakeMarvinHardware()
        executor = MarvinExecutor(
            hardware_session=hardware, publisher_instance_id="marvin", router_zid="router",
            coordinator_instance_id="coord", clock=clock,
            params={"feedback_timeout_s": 0.5, "state_timeout_s": 0.1},
        )
        self.assertIsNone(executor._check_feedback(hardware.feedback, base))
        self.assertIsNone(executor._check_feedback(hardware.feedback, base + 1_000_000))

    def test_feedback_does_not_refresh_stale_session_authority(self):
        base = time.monotonic_ns()
        clock = _Clock(base)
        hardware = _FakeMarvinHardware()
        executor = MarvinExecutor(
            hardware_session=hardware, publisher_instance_id="marvin", router_zid="router",
            coordinator_instance_id="coord", clock=clock,
            params={"feedback_timeout_s": 10.0, "state_timeout_s": 0.1},
        )
        executor.on_session_state(SessionState(
            1, 1, base, "teleop", "start", "coordinator", 1, "coord", "router"
        ))
        self.assertIsNone(executor._check_feedback(hardware.feedback, base))
        received_at = executor._hardware_safety._teleop_state[1]
        clock.value = base + 200_000_000
        hardware.feedback = MarvinFeedback(
            np.zeros(7), np.zeros(7), (1, 1), (1, 1), (0, 0),
            (2, 2), (10, 10), (10, 10), ("None", "None"),
        )
        self.assertIsNone(executor._check_feedback(hardware.feedback, clock(),))
        self.assertEqual(executor._hardware_safety._teleop_state[1], received_at)
        decision = executor._hardware_safety.decide(now=clock() / 1e9)
        self.assertEqual(decision.action, "return_home")


if __name__ == "__main__":
    unittest.main()
