from __future__ import annotations
import json
import signal
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import yaml

import tianji_teleop.executors.marvin.bridge as marvin_bridge
import tianji_teleop.executors.mujoco.node as mujoco_node
import tianji_teleop.executors.wuji_hand2.main as wuji_main_module

from tianji_teleop.executors.marvin.bridge import MarvinExecutor, _apply_speed_overrides
from tianji_teleop.executors.marvin.readiness import MarvinReadiness
from tianji_teleop.executors.mujoco.node import MujocoExecutor
from tianji_teleop.executors.wuji_hand2.node import WujiHandExecutor
from tianji_teleop.executors.wuji_hand2.config import WujiHandConfig
from tianji_teleop.hardware_safety import HardwareSafetyController, HardwareSafetySettings
from tianji_teleop.marvin_hardware import MarvinFeedback, MarvinHardwareError, MarvinHardwareSession
from tianji_teleop.protocol.messages import (
    ARM_JOINT_NAMES,
    ArmJointCommand,
    ArmJointProposal,
    ComponentStatus,
    HandJointCommand,
    HandTargetCommand,
    LatchedBool,
    ProtocolEnvelope,
    SafetyStopRequest,
    SessionState,
)
from tianji_teleop.sources.common.real_admission import RealCapabilityInput


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
    def test_mujoco_configured_urdf_resolves_from_package_root(self):
        config_path = (
            Path(__file__).parents[1]
            / "src"
            / "tianji_teleop"
            / "config"
            / "executors"
            / "mujoco.yaml"
        )
        captured = {}

        def capture_urdf(path):
            captured["path"] = Path(path)
            raise RuntimeError("captured URDF path")

        with patch.object(mujoco_node, "portable_mujoco_urdf", side_effect=capture_urdf):
            with self.assertRaisesRegex(RuntimeError, "captured URDF path"):
                mujoco_node.main(["--config", str(config_path), "--headless"])

        self.assertEqual(
            captured["path"],
            config_path.parents[2]
            / "assets"
            / "tianji_wuji2"
            / "tianji_wuji2.urdf",
        )

    def test_wuji_dry_run_accepts_configured_rate(self):
        captured = {}

        class FakeSession:
            def close(self):
                captured["session_closed"] = True

        class FakeExecutor:
            def __init__(self, **kwargs):
                captured["executor_kwargs"] = kwargs

            def run(self, *, rate_hz):
                captured["rate_hz"] = rate_hz

            def close(self):
                captured["executor_closed"] = True

        with (
            patch.object(wuji_main_module, "open_session", return_value=FakeSession()),
            patch.object(wuji_main_module, "require_single_router", return_value="router"),
            patch.object(wuji_main_module, "WujiHandExecutor", FakeExecutor),
        ):
            try:
                result = wuji_main_module.main(
                    [
                        "--mode",
                        "retarget",
                        "--side",
                        "right",
                        "--dry-run",
                        "--rate",
                        "60",
                    ]
                )
            except SystemExit as exc:
                result = exc.code

        self.assertEqual(result, 0)
        self.assertEqual(captured["rate_hz"], 60.0)
        self.assertTrue(captured["executor_closed"])
        self.assertTrue(captured["session_closed"])

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

    def test_wuji_waits_at_zero_for_first_deadman_input(self):
        clock = _Clock(1_000_000_000)
        executor = WujiHandExecutor(
            mode="retarget", side="right", router_zid="router",
            publisher_instance_id="wuji", authorized_producer="retarget",
            authorized_publisher_instance_id="source",
            coordinator_instance_id="coord", clock=clock,
        )
        executor.on_session_state(SessionState(
            1, 1, clock(), "teleop", "start", "coordinator", 1, "coord", "router"
        ))
        executor.tick(now_ns=clock())
        self.assertEqual(executor._state, "teleop")
        self.assertFalse(executor.unhealthy)
        self.assertFalse(executor.tracking_allowed)


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
        self.assertEqual(set(config.zero_tolerance_rad), {0.1})

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
        from tianji_teleop.executors.wuji_hand2.node import _retarget_keypoints
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
        self.feedback_requests = []
        self.soft_stops = 0
        self.connect_calls = 0
        self.shutdown_calls = 0
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
        self.feedback_requests.append(include_servo_errors)
        return self.feedback

    def soft_stop_once(self):
        self.soft_stops += 1

    def shutdown(self):
        self.shutdown_calls += 1

class _ConnectableMarvinHardware(_FakeMarvinHardware):
    def __init__(self) -> None:
        super().__init__()
        self.home_calls = 0
        self.home_args = None

    def connect_and_prepare(self, *args, **kwargs):
        self.connect_calls += 1
        return self.feedback

    def move_to_home(self, *args, **kwargs):
        self.home_calls += 1
        self.home_args = args
        self.feedback = MarvinFeedback(
            np.asarray(args[0]),
            np.asarray(args[1]),
            self.feedback.arm_states,
            self.feedback.command_states,
            self.feedback.error_codes,
            self.feedback.frame_serials,
            self.feedback.velocity_ratios,
            self.feedback.acceleration_ratios,
            self.feedback.servo_error_reports,
        )
        return self.feedback

class _ReturnHomeRecoveryHardware(_ConnectableMarvinHardware):
    def __init__(self) -> None:
        super().__init__()
        self.connect_kwargs = None
        self.home_kwargs = None
        self.feedback = MarvinFeedback(
            np.asarray([55, -65, -70, -60, 60, 0, 0], dtype=np.float64),
            np.asarray([-55, -65, -11.002, -60, -60, 0, 0], dtype=np.float64),
            (1, 1), (1, 1), (0, 0), (1, 1),
            (10, 10), (10, 10), ("None", "None"),
        )

    def connect_and_prepare(self, *args, **kwargs):
        self.connect_calls += 1
        self.connect_kwargs = kwargs
        return self.feedback

    def move_to_home(self, *args, **kwargs):
        self.home_calls += 1
        self.home_args = args
        self.home_kwargs = kwargs
        self.feedback = MarvinFeedback(
            np.asarray(args[0]), np.asarray(args[1]), (1, 1), (1, 1),
            (0, 0), (2, 2), (1, 1), (1, 1), ("None", "None"),
        )
        return self.feedback



class _DelayedMarvinHardware(MarvinHardwareSession):
    """Native-style one-sample feedback lag for the bounded home path."""

    def __init__(self) -> None:
        self._connected = True
        self._soft_stopped = False
        self._now = 0.0
        self._pending = None
        self._feedback = MarvinFeedback(
            np.zeros(7), np.zeros(7), (3, 3), (3, 3), (0, 0),
            (0, 0), (100, 100), (100, 100), ("None", "None"),
        )
        self._sleep = lambda seconds: setattr(self, "_now", self._now + seconds)
        self._monotonic = lambda: self._now

    def send_joint_targets(self, left, right):
        self._pending = (np.asarray(left).copy(), np.asarray(right).copy())

    def read_feedback(self, include_servo_errors=False):
        del include_servo_errors
        returned = self._feedback
        if self._pending is not None:
            left, right = self._pending
            self._pending = None
            self._feedback = MarvinFeedback(
                left, right, returned.arm_states, returned.command_states,
                returned.error_codes,
                tuple(serial + 1 for serial in returned.frame_serials),
                returned.velocity_ratios, returned.acceleration_ratios,
                returned.servo_error_reports,
            )
        return returned

    def soft_stop_once(self):
        self._soft_stopped = True

class _WrongDirectionRecoveryHardware(MarvinHardwareSession):
    def __init__(self) -> None:
        self._connected = True
        self._soft_stopped = False
        self._now = 0.0
        self._feedback = MarvinFeedback(
            np.zeros(7),
            np.asarray([0, 0, -11.002, 0, 0, 0, 0], dtype=np.float64),
            (1, 1), (1, 1), (0, 0), (1, 1),
            (1, 1), (1, 1), ("None", "None"),
        )
        self._sleep = lambda seconds: setattr(self, "_now", self._now + seconds)
        self._monotonic = lambda: self._now

    def send_joint_targets(self, left, right):
        del left, right
        joints = self._feedback.right_joints_deg.copy()
        joints[2] -= 0.5
        self._feedback = MarvinFeedback(
            self._feedback.left_joints_deg,
            joints,
            self._feedback.arm_states,
            self._feedback.command_states,
            self._feedback.error_codes,
            tuple(serial + 1 for serial in self._feedback.frame_serials),
            self._feedback.velocity_ratios,
            self._feedback.acceleration_ratios,
            self._feedback.servo_error_reports,
        )

    def read_feedback(self, include_servo_errors=False):
        del include_servo_errors
        return self._feedback

    def soft_stop_once(self):
        self._soft_stopped = True




class MarvinExecutorSafetyTest(unittest.TestCase):
    def test_return_home_recovery_moves_inward_with_conservative_limits(self):
        try:
            from tianji_teleop.executors.marvin.return_home import run_return_home
        except ImportError as exc:
            self.fail(f"missing return-home recovery runner: {exc}")

        hardware = _ReturnHomeRecoveryHardware()
        settings = yaml.safe_load(
            (
                Path(__file__).resolve().parents[1]
                / "src/tianji_teleop/config/executors/marvin.yaml"
            ).read_text()
        )
        feedback = run_return_home(
            {"marvin": {"ip": "192.168.1.190"}},
            settings,
            "right",
            recover_outside_limits=True,
            hardware=hardware,
        )

        self.assertEqual(hardware.connect_kwargs["velocity_ratio"], 5)
        self.assertEqual(hardware.connect_kwargs["acceleration_ratio"], 5)
        self.assertEqual(hardware.connect_kwargs["hard_limit_padding_deg"], 0.0)
        self.assertAlmostEqual(hardware.connect_kwargs["lower_limits_deg"][9], -165.0)
        self.assertAlmostEqual(hardware.connect_kwargs["upper_limits_deg"][9], 165.0)
        self.assertEqual(hardware.home_kwargs["max_speed_deg_s"], 10.0)
        self.assertEqual(hardware.home_kwargs["maximum_tracking_error_deg"], 3.0)
        self.assertTrue(hardware.home_kwargs["require_monotonic_home_progress"])
        np.testing.assert_allclose(
            hardware.home_args[0],
            [55, -65, -70, -60, 60, 0, 0],
        )
        np.testing.assert_allclose(
            hardware.home_args[1],
            [-55, -65, 70, -60, -60, 0, 0],
        )
        np.testing.assert_allclose(
            feedback.right_joints_deg,
            [-55, -65, 70, -60, -60, 0, 0],
        )
        self.assertEqual(hardware.shutdown_calls, 1)


    def test_recovery_home_from_normal_range_still_moves_both_arms(self):
        from tianji_teleop.executors.marvin.return_home import run_return_home

        hardware = _ReturnHomeRecoveryHardware()
        hardware.feedback = MarvinFeedback(
            np.asarray([50, -65, -70, -60, 60, 0, 0], dtype=np.float64),
            np.asarray([-50, -65, 60, -60, -60, 0, 0], dtype=np.float64),
            (1, 1), (1, 1), (0, 0), (1, 1),
            (10, 10), (10, 10), ("None", "None"),
        )
        settings = yaml.safe_load(
            (
                Path(__file__).resolve().parents[1]
                / "src/tianji_teleop/config/executors/marvin.yaml"
            ).read_text()
        )
        try:
            run_return_home(
                {"marvin": {"ip": "192.168.1.190"}},
                settings,
                "both",
                recover_outside_limits=True,
                hardware=hardware,
            )
        except MarvinHardwareError as exc:
            self.fail(f"in-range Home was rejected: {exc}")

        self.assertEqual(hardware.home_calls, 1)
        self.assertEqual(hardware.home_kwargs["lower_limits_deg"][9], 0.0)
        self.assertEqual(hardware.home_kwargs["hard_limit_padding_deg"], 1.0)



    def test_marvin_feedback_hard_limits_are_side_specific(self):
        lower = np.asarray(
            [-90, -120, -178, -145, -178, -60, -90,
             -178, -120, 0, -145, -178, -60, -90],
            dtype=np.float64,
        )
        upper = np.asarray(
            [178, 120, 0, 0, 178, 60, 90,
             90, 120, 178, 0, 178, 60, 90],
            dtype=np.float64,
        )
        bounds = MarvinHardwareSession._hard_limit_bounds(
            lower, upper, 0.0
        )
        feedback = MarvinFeedback(
            np.asarray([175, 0, -70, -60, 60, 0, 0]),
            np.asarray([-175, 0, 70, -60, -60, 0, 0]),
            (1, 1), (1, 1), (0, 0), (1, 1),
            (10, 10), (10, 10), ("None", "None"),
        )
        MarvinHardwareSession._require_feedback_within_hard_limits(
            feedback, bounds
        )

        config_path = (
            Path(__file__).resolve().parents[1]
            / "src/tianji_teleop/config/executors/marvin.yaml"
        )
        recovery = yaml.safe_load(config_path.read_text())
        recovery_bounds = MarvinHardwareSession._hard_limit_bounds(
            recovery["feedback_hard_lower_limits_deg"],
            recovery["feedback_hard_upper_limits_deg"],
            recovery["feedback_hard_limit_padding_deg"],
        )
        edge_feedback = MarvinFeedback(
            np.asarray([55, -65, -70, -145.506, 60, 0, 0]),
            np.asarray([-55, -65, 70, -60, -60, 0, 0]),
            (1, 1), (1, 1), (0, 0), (1, 1),
            (10, 10), (10, 10), ("None", "None"),
        )
        MarvinHardwareSession._require_feedback_within_hard_limits(
            edge_feedback, recovery_bounds
        )

    def test_explicit_marvin_speed_overrides_are_applied(self):
        params = {"velocity_ratio": 10, "acceleration_ratio": 10}
        with patch.dict(
            "os.environ",
            {
                "TIANJI_MARVIN_VELOCITY_RATIO": "100",
                "TIANJI_MARVIN_ACCELERATION_RATIO": "80",
            },
            clear=False,
        ):
            _apply_speed_overrides(params)
        self.assertEqual(params["velocity_ratio"], 100)
        self.assertEqual(params["acceleration_ratio"], 80)

    def _command(self, side, sequence=1, timestamp=100, value=0.005, mode="returning"):
        return ArmJointCommand(
            1, sequence, timestamp, "coordinator", side, mode, None, None,
            [f"Joint{i}_{'L' if side == 'left' else 'R'}" for i in range(1, 8)],
            [value] * 7, "coord", "router",
        )

    def test_canonical_rate_hz_drives_real_executor_loop(self):
        executor = MarvinExecutor(
            hardware_session=_FakeMarvinHardware(), publisher_instance_id="marvin",
            router_zid="router", coordinator_instance_id="coord",
            params={"rate_hz": 200.0},
        )
        self.assertEqual(executor._rate_hz, 200.0)


    def test_direct_path_subscribes_only_to_configured_ik_sides(self):
        session = _FakeLiveSession()
        MarvinExecutor(
            session=session,
            hardware_session=_FakeMarvinHardware(),
            publisher_instance_id="marvin",
            router_zid="router",
            coordinator_instance_id="coord",
            params={
                "arm_command_path": "direct",
                "direct_producer_id": "ik",
                "direct_producer_instance_id": "ik-instance",
                "direct_sides": ["right"],
            },
        )
        topics_seen = {topic for topic, _callback in session.subscribers}
        self.assertIn("tianji/proposal/arm/right", topics_seen)
        self.assertNotIn("tianji/proposal/arm/left", topics_seen)

    def test_direct_ik_proposal_is_accepted_only_from_authorized_producer(self):
        executor = MarvinExecutor(
            hardware_session=_FakeMarvinHardware(),
            publisher_instance_id="marvin",
            router_zid="router",
            coordinator_instance_id="coord",
            clock=lambda: 100,
            params={
                "arm_command_path": "direct",
                "direct_producer_id": "ik",
                "direct_producer_instance_id": "ik-instance",
                "direct_sides": ["right"],
            },
        )
        executor.on_session_state(
            SessionState(1, 1, 100, "teleop", "accepted", "coordinator", 1, "coord", "router")
        )
        accepted = ArmJointProposal(
            1, 1, 100, "ik", "right", 7, list(ARM_JOINT_NAMES["right"]),
            [0.01] * 7, {"accepted": True}, "ik-instance", "router",
        )
        self.assertTrue(executor.on_arm_proposal(accepted))
        self.assertEqual(executor._commands["right"].producer, "ik")

        rejected = ArmJointProposal(
            1, 2, 100, "ik", "right", 8, list(ARM_JOINT_NAMES["right"]),
            [0.02] * 7, {"accepted": True}, "other-instance", "router",
        )
        self.assertFalse(executor.on_arm_proposal(rejected))
        self.assertEqual(executor._commands["right"].sequence, 1)

    def test_direct_path_does_not_allow_coordinator_teleop_to_replace_ik_output(self):
        executor = MarvinExecutor(
            hardware_session=_FakeMarvinHardware(),
            publisher_instance_id="marvin",
            router_zid="router",
            coordinator_instance_id="coord",
            clock=lambda: 100,
            params={
                "arm_command_path": "direct",
                "direct_producer_id": "ik",
                "direct_producer_instance_id": "ik-instance",
                "direct_sides": ["right"],
            },
        )
        executor.on_session_state(
            SessionState(1, 1, 100, "teleop", "accepted", "coordinator", 1, "coord", "router")
        )
        proposal = ArmJointProposal(
            1, 1, 100, "ik", "right", 7, list(ARM_JOINT_NAMES["right"]),
            [0.01] * 7, {"accepted": True}, "ik-instance", "router",
        )
        self.assertTrue(executor.on_arm_proposal(proposal))
        coordinator_command = ArmJointCommand(
            1, 1, 100, "coordinator", "right", "teleop", 1, 7,
            list(ARM_JOINT_NAMES["right"]), [0.02] * 7, "coord", "router",
        )
        self.assertTrue(executor.on_arm_command(coordinator_command))
        self.assertEqual(executor._commands["right"].producer, "ik")
        self.assertEqual(executor._commands["right"].position_rad, [0.01] * 7)
        coordinator_return = ArmJointCommand(
            1, 2, 100, "coordinator", "right", "returning", None, None,
            list(ARM_JOINT_NAMES["right"]), [0.0] * 7, "coord", "router",
        )
        self.assertTrue(executor.on_arm_command(coordinator_return))
        self.assertEqual(executor._commands["right"].producer, "coordinator")

    def test_direct_proposal_stays_locked_until_new_teleop_state(self):
        executor = MarvinExecutor(
            hardware_session=_FakeMarvinHardware(),
            publisher_instance_id="marvin",
            router_zid="router",
            coordinator_instance_id="coord",
            clock=lambda: 100,
            params={
                "arm_command_path": "direct",
                "direct_producer_id": "ik",
                "direct_producer_instance_id": "ik-instance",
                "direct_sides": ["right"],
            },
        )
        executor.on_session_state(
            SessionState(1, 1, 100, "teleop", "accepted", "coordinator", 1, "coord", "router")
        )
        proposal = ArmJointProposal(
            1, 1, 100, "ik", "right", 7, list(ARM_JOINT_NAMES["right"]),
            [0.01] * 7, {}, "ik-instance", "router",
        )
        self.assertTrue(executor.on_arm_proposal(proposal))

        returning = ArmJointCommand(
            1, 2, 100, "coordinator", "right", "returning", None, None,
            list(ARM_JOINT_NAMES["right"]), [0.0] * 7, "coord", "router",
        )
        self.assertTrue(executor.on_arm_command(returning))
        late = ArmJointProposal(
            1, 2, 100, "ik", "right", 8, list(ARM_JOINT_NAMES["right"]),
            [0.02] * 7, {}, "ik-instance", "router",
        )
        self.assertTrue(executor.on_arm_proposal(late))
        self.assertEqual(executor._commands["right"].producer, "coordinator")

        executor.on_session_state(
            SessionState(1, 2, 100, "returning", "return", "coordinator", 1, "coord", "router")
        )
        executor.on_session_state(
            SessionState(1, 3, 100, "teleop", "accepted", "coordinator", 2, "coord", "router")
        )
        resumed = ArmJointProposal(
            1, 3, 100, "ik", "right", 9, list(ARM_JOINT_NAMES["right"]),
            [0.03] * 7, {}, "ik-instance", "router",
        )
        self.assertTrue(executor.on_arm_proposal(resumed))
        self.assertEqual(executor._commands["right"].producer, "ik")

    def test_close_does_not_home_without_coordinator_return_authority(self):
        hardware = _ConnectableMarvinHardware()
        executor = MarvinExecutor(
            hardware_session=hardware,
            publisher_instance_id="marvin",
            router_zid="router",
            coordinator_instance_id="coord",
            params={"return_home_on_exit": True},
        )
        executor._phase = "armed_idle"

        executor.close()

        self.assertEqual(hardware.home_calls, 0)
        self.assertEqual(hardware.shutdown_calls, 1)
        self.assertEqual(hardware.sent, [])

    def test_direct_proposal_waits_for_teleop_and_rejects_stale_updates(self):
        now = [100]
        executor = MarvinExecutor(
            hardware_session=_FakeMarvinHardware(),
            publisher_instance_id="marvin",
            router_zid="router",
            coordinator_instance_id="coord",
            clock=lambda: now[0],
            params={
                "arm_command_path": "direct",
                "direct_producer_id": "ik",
                "direct_producer_instance_id": "ik-instance",
                "direct_sides": ["right"],
            },
        )
        proposal = ArmJointProposal(
            1, 1, 100, "ik", "right", 7, list(ARM_JOINT_NAMES["right"]),
            [0.01] * 7, {}, "ik-instance", "router",
        )
        executor.on_session_state(
            SessionState(1, 1, 100, "idle", "accepted", "coordinator", 1, "coord", "router")
        )
        self.assertTrue(executor.on_arm_proposal(proposal))
        self.assertNotIn("right", executor._commands)

        executor.on_session_state(
            SessionState(1, 2, 100, "teleop", "accepted", "coordinator", 1, "coord", "router")
        )
        self.assertTrue(executor.on_arm_proposal(proposal))
        self.assertEqual(executor._commands["right"].sequence, 1)

        now[0] = 200_000_101
        stale = ArmJointProposal(
            1, 2, 100, "ik", "right", 8, list(ARM_JOINT_NAMES["right"]),
            [0.02] * 7, {}, "ik-instance", "router",
        )
        self.assertFalse(executor.on_arm_proposal(stale))
        self.assertEqual(executor._commands["right"].sequence, 1)

    def test_marvin_real_uses_native_200hz_joint_impedance_driver(self):
        config_root = Path(__file__).resolve().parents[1] / "src/tianji_teleop/config"
        profile = yaml.safe_load((config_root / "sessions/regrind_real.yaml").read_text())
        config = yaml.safe_load((config_root / profile["arm_executor_config"]).read_text())
        self.assertEqual(config["hardware_driver"], "native_cpp")
        self.assertEqual(float(config["rate_hz"]), 200.0)
        self.assertEqual(config["control_mode"], "joint_impedance")
        self.assertEqual(config["velocity_ratio"], 50)
        self.assertEqual(config["acceleration_ratio"], 50)
        self.assertEqual(config["velocity_estimation_step_ms"], 5)
        self.assertEqual(config["joint_stiffness"], [10, 10, 10, 1.6, 1, 1, 1])
        self.assertEqual(config["joint_damping"], [0.8, 0.8, 0.8, 0.4, 0.4, 0.4, 0.4])
        self.assertEqual(config["tool_kinematics"], [0, 0, 0, 0, 0, 0])
        self.assertEqual(config["tool_dynamics"]["left"], [0] * 10)
        self.assertEqual(config["tool_dynamics"]["right"], [0.95, 0, 0, 90, 0, 0, 0, 0, 0, 0])
        self.assertEqual(config["controlled_sides"], ["right"])
        self.assertEqual(float(config["maximum_tracking_error_deg"]), 8.0)

        executor = MarvinExecutor(
            hardware_session=_FakeMarvinHardware(), publisher_instance_id="marvin",
            router_zid="router", coordinator_instance_id="coord",
            params={**config, "hardware_factory": _FakeMarvinHardware},
        )
        self.assertEqual(executor._required_arm_state, 3)
        self.assertEqual(executor._hardware_safety.settings.required_arm_state, 3)

    def test_tracking_error_guard_is_independent_of_output_step_bypass(self):
        controller = HardwareSafetyController(
            left_home_deg=np.zeros(7),
            right_home_deg=np.zeros(7),
            lower_limits_deg=np.full(7, -180.0),
            upper_limits_deg=np.full(7, 180.0),
            settings=HardwareSafetySettings(
                maximum_output_step_deg=100000.0,
                maximum_tracking_error_deg=8.0,
            ),
        )
        controller.observe_feedback(
            left_joints_deg=np.zeros(7),
            right_joints_deg=np.zeros(7),
            arm_states=(1, 1),
            error_codes=(0, 0),
            received_at=0.0,
        )
        controller.observe_teleop_state("teleop", received_at=0.0)
        controller.observe_command(
            "left", np.full(7, 20.0), received_at=0.0,
            frame_id="left_base_marvin_degrees",
        )
        controller.observe_command(
            "right", np.zeros(7), received_at=0.0,
            frame_id="right_base_marvin_degrees",
        )

        first = controller.decide(now=0.0)
        self.assertEqual(first.action, "send")
        np.testing.assert_allclose(first.left_joints_deg, np.full(7, 20.0))
        second = controller.decide(now=0.0)
        self.assertEqual(second.action, "soft_stop")
        self.assertEqual(second.reason, "tracking_error")

    def test_right_only_control_returns_both_arms_home_before_start(self):
        hardware = _ConnectableMarvinHardware()
        robot = MarvinExecutor(
            hardware_session=hardware,
            publisher_instance_id="marvin",
            router_zid="router",
            coordinator_instance_id="coord",
        ).robot
        home = np.asarray(robot.home_all)
        left_start = np.degrees(home[:7])
        left_start[0] += 2.0
        hardware.feedback = MarvinFeedback(
            left_start, np.degrees(home[7:]), (1, 1), (1, 1),
            (0, 0), (1, 1), (10, 10), (10, 10), ("None", "None"),
        )
        executor = MarvinExecutor(
            hardware_session=hardware,
            publisher_instance_id="marvin",
            router_zid="router",
            coordinator_instance_id="coord",
            real_capability=RealCapabilityInput(0.1, 0.0, True, True),
            clock=lambda: 100,
            params={"controlled_sides": ["right"], "connection_wait_s": 0.01},
        )
        executor._readiness.connection_ready = lambda now_ns: True
        self.assertTrue(executor.connect())
        np.testing.assert_allclose(hardware.home_args[0], np.degrees(home[:7]))
        np.testing.assert_allclose(hardware.home_args[1], np.degrees(home[7:]))

        executor.on_session_state(SessionState(
            1, 1, 100, "teleop", "accepted", "coordinator", 1,
            "coord", "router",
        ))
        executor._check_feedback(hardware.feedback, 100)
        executor._send_commands(
            self._command("left", value=0.1, mode="teleop"),
            self._command("right", value=0.001, mode="teleop"),
        )
        np.testing.assert_allclose(hardware.sent[-1][0], np.degrees(home[:7]))
        self.assertGreater(np.max(np.abs(hardware.sent[-1][1])), 0.0)

    def test_move_to_home_waits_for_delayed_native_feedback(self):
        hardware = _DelayedMarvinHardware()

        final = hardware.move_to_home(
            np.ones(7), np.ones(7), rate_hz=10.0,
            minimum_duration_s=0.1, max_speed_deg_s=100.0,
            maximum_tracking_error_deg=8.0, home_tolerance_deg=0.01,
            required_state=3,
        )

        np.testing.assert_allclose(final.left_joints_deg, np.ones(7))
        np.testing.assert_allclose(final.right_joints_deg, np.ones(7))

    def test_recovery_home_soft_stops_motion_away_from_home(self):
        hardware = _WrongDirectionRecoveryHardware()
        try:
            with self.assertRaisesRegex(MarvinHardwareError, "moved away from Home"):
                hardware.move_to_home(
                    np.zeros(7),
                    np.asarray([0, 0, 70, 0, 0, 0, 0], dtype=np.float64),
                    rate_hz=10.0,
                    minimum_duration_s=0.1,
                    max_speed_deg_s=2.0,
                    maximum_tracking_error_deg=1000.0,
                    home_tolerance_deg=1.0,
                    lower_limits_deg=np.full(14, -165.0),
                    upper_limits_deg=np.full(14, 165.0),
                    hard_limit_padding_deg=0.0,
                    require_monotonic_home_progress=True,
                )
        except TypeError as exc:
            self.fail(f"missing monotonic recovery guard: {exc}")
        self.assertTrue(hardware._soft_stopped)


    def test_move_to_home_stops_when_return_authority_is_revoked(self):
        hardware = _DelayedMarvinHardware()

        with self.assertRaisesRegex(MarvinHardwareError, "authorization revoked"):
            hardware.move_to_home(
                np.ones(7), np.ones(7), rate_hz=10.0,
                minimum_duration_s=0.1, max_speed_deg_s=100.0,
                maximum_tracking_error_deg=8.0, home_tolerance_deg=0.01,
                required_state=3, return_authority=lambda: False,
            )

        self.assertTrue(hardware._soft_stopped)

    def test_real_output_slew_does_not_underrun_coordinator_step(self):
        config_root = Path(__file__).resolve().parents[1] / "src" / "tianji_teleop" / "config"
        coordinator = yaml.safe_load((config_root / "coordinator" / "arm.yaml").read_text())
        marvin = yaml.safe_load((config_root / "executors" / "marvin.yaml").read_text())
        self.assertGreaterEqual(
            float(marvin["maximum_output_step_deg"]),
            np.degrees(float(coordinator["maximum_command_step_rad"])),
        )

    def test_healthy_tick_does_not_poll_slow_servo_diagnostics(self):
        hardware = _FakeMarvinHardware()
        executor = MarvinExecutor(
            hardware_session=hardware, publisher_instance_id="marvin",
            router_zid="router", coordinator_instance_id="coord",
        )
        executor.tick(now_ns=100)
        self.assertEqual(hardware.feedback_requests, [False])

    def test_tracking_error_soft_stop_logs_once_and_preserves_status_error(self):
        hardware = _FakeMarvinHardware()
        executor = MarvinExecutor(
            hardware_session=hardware, publisher_instance_id="marvin",
            router_zid="router", coordinator_instance_id="coord",
        )
        executor._phase = "teleop"
        executor._last_output_deg = np.full(14, 9.0)

        with self.assertLogs(marvin_bridge._LOG, level="ERROR") as captured:
            executor.tick(now_ns=100)
            executor.tick(now_ns=100)

        self.assertEqual(len(captured.records), 1)
        self.assertEqual(
            captured.records[0].getMessage(),
            "Marvin soft stop: reason=tracking_error phase=teleop "
            "arm_command_path=coordinator",
        )
        self.assertEqual(executor.status.error, "tracking_error")
        self.assertEqual(hardware.soft_stops, 1)

    def test_runtime_error_soft_stop_logs_once_and_preserves_status_error(self):
        hardware = _FakeMarvinHardware()

        def fail_feedback(*_args, **_kwargs):
            raise RuntimeError("feedback exploded")

        hardware.read_feedback = fail_feedback
        executor = MarvinExecutor(
            hardware_session=hardware, publisher_instance_id="marvin",
            router_zid="router", coordinator_instance_id="coord",
        )
        executor._phase = "armed_idle"

        with self.assertLogs(marvin_bridge._LOG, level="ERROR") as captured:
            executor.tick(now_ns=100)
            executor.tick(now_ns=100)

        self.assertEqual(len(captured.records), 1)
        self.assertEqual(
            captured.records[0].getMessage(),
            "Marvin soft stop: reason=runtime_error: feedback exploded "
            "phase=armed_idle arm_command_path=coordinator",
        )
        self.assertEqual(
            executor.status.error, "runtime_error: feedback exploded"
        )
        self.assertEqual(hardware.soft_stops, 1)

    def test_shutdown_signal_is_not_swallowed_by_executor_tick(self):
        hardware = _FakeMarvinHardware()
        executor = MarvinExecutor(
            hardware_session=hardware, publisher_instance_id="marvin",
            router_zid="router", coordinator_instance_id="coord",
        )

        def raise_shutdown(*args, **kwargs):
            raise marvin_bridge._ShutdownRequested()

        hardware.read_feedback = raise_shutdown
        with self.assertRaises(marvin_bridge._ShutdownRequested):
            executor.tick(now_ns=100)

        with self.assertRaises(marvin_bridge._ShutdownRequested):
            marvin_bridge._handle_shutdown(signal.SIGTERM, None)

    def test_fresh_teleop_commands_recover_from_transient_pair_gap(self):
        hardware = _FakeMarvinHardware()
        executor = MarvinExecutor(
            hardware_session=hardware, publisher_instance_id="marvin",
            router_zid="router", coordinator_instance_id="coord",
            clock=lambda: 100,
        )
        home = np.asarray(executor.robot.home_all)
        hardware.feedback = MarvinFeedback(
            np.degrees(home[:7]), np.degrees(home[7:]), (1, 1), (1, 1),
            (0, 0), (1, 1), (10, 10), (10, 10), ("None", "None"),
        )
        executor._phase = "armed_idle"
        executor.on_session_state(SessionState(
            1, 1, 100, "teleop", "accepted", "coordinator", 1,
            "coord", "router",
        ))
        executor.tick(now_ns=100)
        self.assertEqual(executor.phase, "returning")

        hardware.sent.clear()
        for side, values in (("left", home[:7]), ("right", home[7:] + 0.001)):
            executor.on_arm_command(ArmJointCommand(
                1, 2, 100, "coordinator", side, "teleop", 1, 1,
                list(ARM_JOINT_NAMES[side]), values.tolist(), "coord", "router",
            ))
        executor.tick(now_ns=100)
        self.assertEqual(executor.phase, "teleop")
        self.assertGreater(
            np.max(np.abs(hardware.sent[-1][1] - np.degrees(home[7:]))),
            0.0,
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
        self.assertEqual(len(hardware.sent), 1)

    def test_normal_close_smoothly_returns_home_before_release(self):
        hardware = _ConnectableMarvinHardware()
        now = time.monotonic_ns()
        executor = MarvinExecutor(
            hardware_session=hardware,
            publisher_instance_id="marvin",
            router_zid="router",
            coordinator_instance_id="coord",
            clock=lambda: now,
            params={"return_home_on_exit": True},
        )
        executor.on_session_state(
            SessionState(1, 1, now, "returning", "return", "coordinator", None, "coord", "router")
        )
        executor.on_arm_command(self._command("left", timestamp=now))
        executor.on_arm_command(self._command("right", timestamp=now))
        initial_left = hardware.feedback.left_joints_deg.copy()
        initial_right = hardware.feedback.right_joints_deg.copy()

        executor.close()

        self.assertEqual(hardware.home_calls, 1)
        self.assertEqual(hardware.shutdown_calls, 1)
        self.assertEqual(len(hardware.sent), 1)
        np.testing.assert_allclose(hardware.sent[0][0], initial_left)
        np.testing.assert_allclose(hardware.sent[0][1], initial_right)
        np.testing.assert_allclose(
            hardware.home_args[0], np.degrees(executor.robot.left_home_rad)
        )
        np.testing.assert_allclose(
            hardware.home_args[1], np.degrees(executor.robot.right_home_rad)
        )

    def test_safety_locked_close_does_not_move(self):
        hardware = _ConnectableMarvinHardware()
        executor = MarvinExecutor(
            hardware_session=hardware,
            publisher_instance_id="marvin",
            router_zid="router",
            coordinator_instance_id="coord",
            params={"return_home_on_exit": True},
        )
        executor._phase = "soft_stopped"
        executor._safety_locked = True

        executor.close()

        self.assertEqual(hardware.home_calls, 0)
        self.assertEqual(hardware.shutdown_calls, 1)

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
