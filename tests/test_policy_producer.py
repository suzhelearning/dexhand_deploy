import math
import unittest
from types import SimpleNamespace

from pico_body_tianji.coordination.arm_command_coordinator import ArmRobotConfig, ArmCommandCoordinator
from pico_body_tianji.protocol.messages import (
    ALL_ARM_JOINT_NAMES,
    ARM_JOINT_NAMES,
    ArmJointState,
    ArmTargetCommand,
    ComponentStatus,
    ProtocolEnvelope,
    SessionState,
)
from pico_body_tianji.producers.policy.contracts import (
    ActionAdapter,
    ActionValidationError,
    HoldPolicyRunner,
    ObservationBuilder,
    PolicyAction,
    PolicyObservation,
)
from pico_body_tianji.producers.policy.node import PolicyProducerNode


ROUTER = "router-1"
INSTANCE = "policy-instance"


def state(timestamp, positions=None, velocity=None):
    return ArmJointState(
        1, timestamp // 1_000_000, timestamp, "mujoco", list(ALL_ARM_JOINT_NAMES),
        list(positions or [0.0] * 14), velocity, "mujoco-instance", ROUTER,
    )


def session_state(state_name="teleop", timestamp=1_000_000_000):
    return SessionState(1, 1, timestamp, state_name, "test", "coordinator", 1, "coordinator-instance", ROUTER)


class ActionAdapterTest(unittest.TestCase):
    def setUp(self):
        self.robot = ArmRobotConfig.load()
        self.adapter = ActionAdapter(
            self.robot,
            publisher_instance_id=INSTANCE,
            router_zid=ROUTER,
            maximum_step_rad=10.0,
            control_period_s=0.5,
        )

    def test_absolute_position_is_split_in_canonical_order(self):
        values = [0.0] * 14
        values[0] = 0.1
        values[7] = -0.1
        proposals = self.adapter.adapt(PolicyAction("absolute_position_rad", values), [0.0] * 14, sequence=4, timestamp_ns=10)
        self.assertEqual(proposals["left"].names, list(ARM_JOINT_NAMES["left"]))
        self.assertEqual(proposals["right"].names, list(ARM_JOINT_NAMES["right"]))
        self.assertEqual(proposals["left"].position_rad[0], 0.1)
        self.assertEqual(proposals["right"].position_rad[0], -0.1)

    def test_delta_and_velocity_modes_use_current_position(self):
        current = [0.1] * 14
        delta = self.adapter.adapt(PolicyAction("delta_position_rad", [0.2] * 14), current, sequence=1, timestamp_ns=1)
        velocity = self.adapter.adapt(PolicyAction("velocity_rad_s", [0.2] * 14), current, sequence=2, timestamp_ns=2)
        self.assertAlmostEqual(delta["left"].position_rad[0], 0.3)
        self.assertEqual(velocity["right"].position_rad[0], 0.2)

    def test_nonfinite_shape_and_limits_are_rejected(self):
        with self.assertRaises(ActionValidationError):
            self.adapter.adapt(PolicyAction("absolute_position_rad", [0.0] * 13), [0.0] * 14, sequence=1, timestamp_ns=1)
        with self.assertRaises(ActionValidationError):
            self.adapter.adapt(PolicyAction("absolute_position_rad", [math.nan] * 14), [0.0] * 14, sequence=1, timestamp_ns=1)
        with self.assertRaises(ActionValidationError):
            self.adapter.adapt(PolicyAction("absolute_position_rad", [3.1] * 14), [0.0] * 14, sequence=1, timestamp_ns=1)
        small_step = ActionAdapter(self.robot, publisher_instance_id=INSTANCE, router_zid=ROUTER, maximum_step_rad=0.05)
        with self.assertRaises(ActionValidationError):
            small_step.adapt(PolicyAction("delta_position_rad", [0.1] * 14), [0.0] * 14, sequence=1, timestamp_ns=1)


class ObservationBuilderTest(unittest.TestCase):
    def test_missing_velocity_is_estimated_from_adjacent_states(self):
        builder = ObservationBuilder(stale_timeout_s=0.5, clock=lambda: 2_000_000_000)
        self.assertIsNone(builder.build(state(1_000_000_000, [0.0] * 14)))
        observation = builder.build(state(1_500_000_000, [0.5] * 14))
        self.assertIsNotNone(observation)
        self.assertEqual(observation.joint_state.velocity_rad_s, [1.0] * 14)
    def test_repeated_executor_frame_reuses_fresh_observation(self):
        builder = ObservationBuilder(stale_timeout_s=0.5, clock=lambda: 1_100_000_000)
        current = state(1_000_000_000, [0.5] * 14)
        self.assertIsNone(builder.build(current))
        observation = builder.build(state(1_050_000_000, [0.6] * 14))
        repeated = builder.build(state(1_050_000_000, [0.6] * 14), now_ns=1_100_000_000)
        self.assertIsNotNone(observation)
        self.assertIs(repeated, observation)

    def test_state_stale_or_velocity_gap_is_not_ready(self):
        builder = ObservationBuilder(stale_timeout_s=0.2, clock=lambda: 2_000_000_000)
        stale = builder.build(state(1_000_000_000, [0.0] * 14, [0.0] * 14))
        self.assertIsNone(stale)
        builder = ObservationBuilder(stale_timeout_s=0.2, clock=lambda: 1_500_000_000)
        self.assertIsNone(builder.build(state(1_000_000_000, [0.0] * 14)))
        self.assertIsNone(builder.build(state(1_400_000_000, [0.2] * 14)))
        self.assertIn("velocity", builder.last_reason)


class HoldPolicyRunnerTest(unittest.TestCase):
    def test_hold_returns_current_14_position(self):
        joint_state = state(1_000_000_000, list(range(14)), [0.0] * 14)
        observation = PolicyObservation(joint_state, session_state=session_state())
        action = HoldPolicyRunner().step(observation)
        self.assertEqual(action.mode, "absolute_position_rad")
        self.assertEqual(action.values, list(range(14)))


class _FakePublisher:
    def __init__(self):
        self.values = []
    def put(self, payload, **kwargs):
        self.values.append(payload)
    def undeclare(self):
        pass


class _FakeSession:
    def __init__(self):
        self.publishers = {}
        self.subscribers = []
    def declare_publisher(self, key, **kwargs):
        pub = _FakePublisher()
        self.publishers[key] = pub
        return pub
    def declare_subscriber(self, key, callback):
        self.subscribers.append((key, callback))
        return SimpleNamespace(undeclare=lambda: None)


class _InvalidRunner:
    loaded = True
    healthy = True
    def step(self, observation):
        return PolicyAction("absolute_position_rad", [math.nan] * 14)


class PolicyProducerNodeTest(unittest.TestCase):
    def _node(self, runner=None):
        session = _FakeSession()
        node = PolicyProducerNode(
            session,
            publisher_instance_id=INSTANCE,
            router_zid=ROUTER,
            runner=runner,
            observation_builder=ObservationBuilder(stale_timeout_s=0.5, clock=lambda: 1_000_000_000),
            maximum_step_rad=10.0,
            clock=lambda: 1_000_000_000,
        )
        node.start()
        node.on_session_state(session_state())
        node.on_arm_state(state(1_000_000_000, [0.0] * 14, [0.0] * 14))
        return node, session

    def test_hold_publishes_only_typed_proposals_and_no_final_command(self):
        node, session = self._node()
        node.tick(now_ns=1_000_000_000)
        self.assertIn("tianji/proposal/arm/left", session.publishers)
        self.assertIn("tianji/proposal/arm/right", session.publishers)
        self.assertNotIn("tianji/command/arm/left", session.publishers)
        self.assertNotIn("tianji/command/arm/right", session.publishers)
        self.assertTrue(node.status.healthy)
        proposal_sequences = [proposal.sequence for proposal in node.tick(now_ns=1_000_000_000).values()]
        self.assertTrue(all(sequence > node.status.sequence - 1 for sequence in proposal_sequences))
        self.assertTrue(node.status.ready)

    def test_invalid_action_marks_unhealthy_and_drops_proposal(self):
        node, session = self._node(_InvalidRunner())
        node.tick(now_ns=1_000_000_000)
        self.assertFalse(node.status.healthy)
        self.assertFalse(node.status.ready)
        self.assertEqual(len(session.publishers["tianji/proposal/arm/left"].values), 0)

    def test_target_identity_uses_envelope_and_propagates_sequence(self):
        node, _ = self._node()
        target = ArmTargetCommand(
            ProtocolEnvelope(1, "source-instance", ROUTER, 77, 1_000_000_000),
            None, "mocap_live", "right", "Base_R", [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0], [1.0, 0.0, 0.0],
        )
        node.on_arm_target("right", target)
        proposals = node.tick(now_ns=1_000_000_000)
        self.assertEqual(proposals["right"].target_sequence, 77)

    def test_invalid_session_snapshot_clears_teleop_and_stops_proposals(self):
        node, session = self._node()
        node.on_session_state({"not": "a SessionState"})
        self.assertEqual(node.tick(now_ns=1_000_000_000), {})
        self.assertFalse(node.status.healthy)
        self.assertEqual(len(session.publishers["tianji/proposal/arm/right"].values), 0)
        self.assertEqual(len(session.publishers["tianji/proposal/arm/left"].values), 0)


class PolicyCoordinatorPathTest(unittest.TestCase):
    def test_hold_proposals_are_consumed_as_final_commands(self):
        now = 1_000_000_000
        robot = ArmRobotConfig.load()
        coordinator = ArmCommandCoordinator(
            session=None,
            publisher_instance_id="coordinator-instance",
            router_zid=ROUTER,
            profile={"active_sides": ["right"], "required_capability": "simulation"},
            clock=lambda: now,
        )
        for role, component_id, instance in (
            ("source", "mocap_live", "source-instance"),
            ("producer_arm", "policy_hold", INSTANCE),
            ("executor_arm", "mujoco", "mujoco-instance"),
        ):
            coordinator.update_component(
                ComponentStatus(
                    1, 1, now, role, component_id, "ready", True, True,
                    ["simulation"], None, {}, instance, ROUTER,
                )
            )
        coordinator.update_arm_state(state(now, list(robot.home_all), [0.0] * 14))
        self.assertTrue(
            coordinator.handle_intent(
                SimpleNamespace(action="start", sequence=1, source="mocap_live", reason="test")
            ).accepted
        )
        node = PolicyProducerNode(
            None,
            publisher_instance_id=INSTANCE,
            router_zid=ROUTER,
            observation_builder=ObservationBuilder(stale_timeout_s=0.5, clock=lambda: now),
            maximum_step_rad=10.0,
            clock=lambda: now,
        )
        node.on_session_state(coordinator.state)
        node.on_arm_state(state(now, list(robot.home_all), [0.0] * 14))
        proposals = node.tick(now_ns=now)
        for proposal in proposals.values():
            coordinator.update_proposal(proposal, received_ns=now)
        commands = coordinator.tick(now_ns=now)
        self.assertEqual(coordinator.state.state, "teleop")
        self.assertEqual(commands["right"].mode, "teleop")
        self.assertEqual(commands["right"].position_rad, list(robot.right_home_rad))
        self.assertEqual(commands["left"].position_rad, list(robot.left_home_rad))


if __name__ == "__main__":
    unittest.main()
