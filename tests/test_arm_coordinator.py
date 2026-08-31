import json
import math
import unittest
from types import SimpleNamespace

from pico_body_tianji.protocol.messages import (
    ALL_ARM_JOINT_NAMES,
    ARM_JOINT_NAMES,
    ArmJointProposal,
    ArmJointState,
    ComponentStatus,
    LatchedBool,
    SessionIntent,
    SessionState,
)
from pico_body_tianji.coordination.arm_command_coordinator import ArmCommandCoordinator
from pico_body_tianji.sources.common.session_client import SessionClient


class _QueryableSession:
    def __init__(self):
        self.get_callbacks = {}

    def declare_subscriber(self, key, callback):
        return SimpleNamespace(undeclare=lambda: None)

    def declare_publisher(self, key, **kwargs):
        return SimpleNamespace(put=lambda *args, **kwargs: None, undeclare=lambda: None)

    def get(self, key, callback, **kwargs):
        self.get_callbacks[key] = callback


class SessionClientQueryBarrierTest(unittest.TestCase):
    def test_old_cross_channel_reply_cannot_complete_missing_channel(self):
        client = SessionClient(
            _QueryableSession(),
            source="source",
            publisher_instance_id="source-instance",
            router_zid="router-1",
            expected_coordinator_instance_id="coord-1",
        )
        client.start()
        state = SessionState(1, 10, 10, "idle", "ready", "coordinator", None, "coord-1", "router-1")
        home = LatchedBool(1, 11, 11, True, "coord-1", "router-1")
        old_completion = LatchedBool(1, 9, 9, False, "coord-1", "router-1")
        client._on_state_payload(json.dumps(state.to_dict()).encode(), query_channel="state")
        client._on_latched_payload(json.dumps(home.to_dict()).encode(), is_home=True, query_channel="at_home")
        client._on_latched_payload(json.dumps(old_completion.to_dict()).encode(), is_home=False, query_channel="return_complete")
        self.assertFalse(client.snapshot_complete)




class ArmCommandCoordinatorTest(unittest.TestCase):
    def setUp(self):
        self.coordinator = ArmCommandCoordinator(
            session=None,
            publisher_instance_id="coord-1",
            router_zid="router-1",
            profile={"active_sides": ["right"], "required_capability": "simulation"},
            clock=lambda: 1_000_000_000,
        )

    def test_home_config_is_rad_and_exact_robot_order(self):
        self.assertEqual(len(self.coordinator.robot.left_joint_names), 7)
        self.assertEqual(self.coordinator.robot.right_joint_names[0], "Joint1_R")
        self.assertAlmostEqual(self.coordinator.robot.left_home_rad[0], math.radians(55.0))
        self.assertAlmostEqual(self.coordinator.robot.right_home_rad[1], math.radians(-65.0))
        self.assertEqual(len(self.coordinator.robot.lower_limits_rad), 7)
        self.assertEqual(len(self.coordinator.robot.upper_limits_rad), 7)

    def test_idle_tick_publishes_same_sequence_timestamp_home_commands(self):
        outputs = self.coordinator.tick()
        self.assertEqual(self.coordinator.state.state, "idle")
        self.assertTrue(self.coordinator.at_home.value)
        self.assertFalse(self.coordinator.return_complete.value)
        self.assertEqual(outputs["left"].sequence, outputs["right"].sequence)
        self.assertEqual(outputs["left"].timestamp_ns, outputs["right"].timestamp_ns)
        self.assertEqual(outputs["right"].position_rad, list(self.coordinator.robot.right_home_rad))

    def test_start_requires_exactly_one_fresh_domain_and_returns_teleop(self):
        now = 1_000_000_000
        self.coordinator.update_component(_status("source", "src", now))
        self.coordinator.update_component(_status("producer_arm", "ik", now))
        self.coordinator.update_component(_status("executor_arm", "mujoco", now))
        self.coordinator.update_arm_state(_arm_state(now, self.coordinator.robot.home_all))
        self.coordinator._proposals["right"] = SimpleNamespace(received_ns=0)
        intent = self.coordinator.handle_intent(SimpleNamespace(action="start", sequence=7, source="src", reason="run"))
        self.assertTrue(intent.accepted)
        self.assertEqual(self.coordinator.state.state, "teleop")
        self.assertEqual(self.coordinator.state.intent_sequence, 7)
        self.assertFalse(self.coordinator.at_home.value)
        self.assertEqual(self.coordinator._proposals, {})

    def test_launcher_authority_mapping_rejects_foreign_component_identity(self):
        disabled = {
            "logical_id": "disabled",
            "publisher_instance_id": "disabled",
            "router_zid": "router-1",
            "enabled": False,
        }
        authorities = {
            "source": {"logical_id": "src", "publisher_instance_id": "src-instance", "router_zid": "router-1"},
            "producer_arm": {"logical_id": "ik", "publisher_instance_id": "ik-instance", "router_zid": "router-1"},
            "producer_hand": {"left": disabled, "right": disabled},
            "coordinator_arm": {"logical_id": "arm", "publisher_instance_id": "coord-1", "router_zid": "router-1"},
            "executor_arm": {"logical_id": "mujoco", "publisher_instance_id": "mujoco-instance", "router_zid": "router-1"},
            "executor_hand": {"left": disabled, "right": disabled},
        }
        coordinator = ArmCommandCoordinator(
            session=None,
            publisher_instance_id="coord-1",
            router_zid="router-1",
            profile={"active_sides": ["right"], "required_capability": "simulation", "authorities": authorities},
            clock=lambda: 1_000_000_000,
        )
        coordinator.update_component(_status("source", "unexpected", 1_000_000_000))
        self.assertEqual(coordinator.state.state, "fault")

    def test_reject_does_not_mutate_state_and_requires_new_intent(self):
        before = self.coordinator.state
        result = self.coordinator.handle_intent(SimpleNamespace(action="start", sequence=9, source="src", reason="run"))
        self.assertFalse(result.accepted)
        self.assertEqual(self.coordinator.state, before)
        self.assertEqual(result.state.intent_sequence, 9)

    def test_invalid_proposal_latches_fault_and_returns_bounded_home(self):
        self.coordinator._state = self.coordinator._make_state("teleop", "active", 1)
        self.coordinator._safe_command["right"] = [x + 0.4 for x in self.coordinator.robot.right_home_rad]
        # Protocol rejects NaN before coordinator; direct malformed ingress is faulted.
        self.coordinator.handle_proposal_dict({"schema_version": 1, "sequence": 3,
            "timestamp_ns": 1_000_000_000, "producer": "ik", "side": "right",
            "target_sequence": 2, "names": list(ARM_JOINT_NAMES["right"]),
            "position_rad": [9.0] * 7, "diagnostics": {},
            "publisher_instance_id": "ik-1", "router_zid": "router-1"})
        self.assertEqual(self.coordinator.state.state, "fault")
        command = self.coordinator.tick()["right"]
        self.assertEqual(command.mode, "returning")
        self.assertNotEqual(command.position_rad, list(self.coordinator.robot.right_home_rad))

    def test_stale_proposal_enters_bounded_returning(self):
        self.coordinator._state = self.coordinator._make_state("teleop", "active", 2)
        self.coordinator._safe_command["right"] = [x + 0.4 for x in self.coordinator.robot.right_home_rad]
        now = 2_000_000_001
        self.coordinator._proposals["right"] = SimpleNamespace(received_ns=1_000_000_000)
        self.coordinator.update_component(_status("source", "src", now), received_ns=now)
        self.coordinator.update_component(_status("producer_arm", "ik", now), received_ns=now)
        self.coordinator.update_component(_status("executor_arm", "mujoco", now), received_ns=now)
        self.coordinator.update_arm_state(
            _arm_state(now, self.coordinator.robot.home_all), received_ns=now
        )
        commands = self.coordinator.tick(now_ns=now)
        self.assertEqual(self.coordinator.state.state, "returning")
        self.assertTrue(all(command.mode == "returning" for command in commands.values()))

    def test_teleop_waits_at_home_for_first_proposal(self):
        now = 2_000_000_001
        self.coordinator._state = self.coordinator._make_state("teleop", "active", 2)
        self.coordinator.update_component(_status("source", "src", now), received_ns=now)
        self.coordinator.update_component(_status("producer_arm", "ik", now), received_ns=now)
        self.coordinator.update_component(_status("executor_arm", "mujoco", now), received_ns=now)
        self.coordinator.update_arm_state(
            _arm_state(now, self.coordinator.robot.home_all), received_ns=now
        )
        commands = self.coordinator.tick(now_ns=now)
        self.assertEqual(self.coordinator.state.state, "teleop")
        self.assertEqual(commands["right"].position_rad, list(self.coordinator.robot.right_home_rad))

    def test_one_tick_proposal_lag_is_clipped_not_faulted(self):
        now = 1_000_000_000
        self.coordinator._state = self.coordinator._make_state("teleop", "active", 2)
        for role, component in (("source", "src"), ("producer_arm", "ik"), ("executor_arm", "mujoco")):
            self.coordinator.update_component(_status(role, component, now), received_ns=now)
        self.coordinator.update_arm_state(
            _arm_state(now, self.coordinator.robot.home_all), received_ns=now
        )
        old = list(self.coordinator.robot.right_home_rad)
        maximum_step = self.coordinator.config["maximum_command_step_rad"]
        candidate = old.copy()
        candidate[0] += 1.5 * maximum_step
        self.coordinator._proposals["right"] = SimpleNamespace(
            value=ArmJointProposal(
                1, 2, now, "ik", "right", 1, list(ARM_JOINT_NAMES["right"]),
                candidate, {}, "ik-instance", "router-1"
            ),
            received_ns=now,
        )
        command = self.coordinator.tick(now_ns=now)["right"]
        self.assertEqual(self.coordinator.state.state, "teleop")
        self.assertAlmostEqual(command.position_rad[0] - old[0], maximum_step)

    def test_return_waits_for_fresh_arm_home_then_latches_once(self):
        self.coordinator._state = self.coordinator._make_state("returning", "return", 4)
        self.coordinator.tick()
        self.assertFalse(self.coordinator.return_complete.value)
        self.coordinator.update_arm_state(_arm_state(1_000_000_100, self.coordinator.robot.home_all))
        self.coordinator.tick(now_ns=1_000_000_200)
        self.assertEqual(self.coordinator.state.state, "idle")
        self.assertTrue(self.coordinator.return_complete.value)
        first_sequence = self.coordinator.return_complete.sequence
        self.coordinator.tick(now_ns=1_000_000_300)
        self.assertGreater(self.coordinator.return_complete.sequence, first_sequence)


class ArmCommandCoordinatorAuthorityRound5Test(unittest.TestCase):
    def setUp(self):
        disabled = {
            "logical_id": "disabled",
            "publisher_instance_id": "disabled",
            "router_zid": "router-1",
            "enabled": False,
        }
        self.authorities = {
            "source": {"logical_id": "src", "publisher_instance_id": "src-instance", "router_zid": "router-1"},
            "producer_arm": {"logical_id": "ik", "publisher_instance_id": "ik-instance", "router_zid": "router-1"},
            "producer_hand": {
                "left": {"logical_id": "h5_direct", "publisher_instance_id": "hand-instance", "router_zid": "router-1"},
                "right": {"logical_id": "h5_direct", "publisher_instance_id": "hand-instance", "router_zid": "router-1"},
            },
            "coordinator_arm": {"logical_id": "arm", "publisher_instance_id": "coord-1", "router_zid": "router-1"},
            "executor_arm": {"logical_id": "mujoco", "publisher_instance_id": "mujoco-instance", "router_zid": "router-1"},
            "executor_hand": {"left": disabled, "right": disabled},
        }
        self.coordinator = ArmCommandCoordinator(
            session=None,
            publisher_instance_id="coord-1",
            router_zid="router-1",
            profile={
                "active_sides": ["right"],
                "active_hand_sides": ["left", "right"],
                "hand_sides": ["left", "right"],
                "required_capability": "simulation",
                "authorities": self.authorities,
            },
            clock=lambda: 1_000_000_000,
        )

    def test_shared_side_less_hand_producer_matches_only_active_profile_sides(self):
        self.assertTrue(self.coordinator._matches_authority(
            "producer_hand", "h5_direct", "hand-instance", "router-1"
        ))
        self.assertTrue(self.coordinator._matches_authority(
            "producer_hand", "h5_direct", "hand-instance", "router-1", side="right"
        ))
        self.assertFalse(self.coordinator._matches_authority(
            "producer_hand", "h5_direct", "other-instance", "router-1"
        ))

    def test_foreign_same_router_intent_and_arm_state_are_rejected(self):
        intent = SessionIntent(1, 1, 1_000_000_000, "src", "start", "forged", "foreign", "router-1")
        result = self.coordinator.handle_intent(intent)
        self.assertFalse(result.accepted)
        self.assertNotEqual(self.coordinator.state.state, "teleop")
        state = _arm_state(1_000_000_000, self.coordinator.robot.home_all)
        forged_state = ArmJointState(
            state.schema_version, state.sequence, state.timestamp_ns, state.executor,
            state.names, state.position_rad, state.velocity_rad_s, "foreign", "router-1"
        )
        self.coordinator.update_arm_state(forged_state)
        self.assertIsNone(self.coordinator._arm_state)
from pico_body_tianji.connection_readiness import evaluate_connection, evaluate_fault_return, evaluate_start


class MarvinReadinessSplitTest(unittest.TestCase):
    def test_connection_does_not_require_policy_observation(self):
        status = SimpleNamespace(timestamp_ns=100, ready=True, healthy=True, capabilities=["real"])
        producer = SimpleNamespace(timestamp_ns=100, ready=True, healthy=True)
        coordinator = SimpleNamespace(timestamp_ns=100, state="idle")
        command = SimpleNamespace(timestamp_ns=100, mode="idle", names=["Joint1_R"], position_rad=[0.0])
        self.assertTrue(evaluate_connection(source_status=status, arm_producer_status=producer, coordinator_state=coordinator, arm_command=command, now_ns=100).ready)

    def test_fault_return_requires_fresh_bounded_returning_command(self):
        state = SimpleNamespace(state="fault")
        command = SimpleNamespace(timestamp_ns=100, mode="returning")
        self.assertTrue(evaluate_fault_return(coordinator_state=state, arm_command=command, now_ns=100).allowed)
        self.assertFalse(evaluate_fault_return(coordinator_state=state, arm_command=command, now_ns=2_000_000_000).allowed)
        self.assertTrue(evaluate_start(executor_status=SimpleNamespace(timestamp_ns=100, ready=True, healthy=True), arm_state=SimpleNamespace(timestamp_ns=100), coordinator_state=SimpleNamespace(timestamp_ns=100, state="idle"), now_ns=100).ready)

def _status(role, component_id, timestamp):
    return ComponentStatus(1, 1, timestamp, role, component_id, "ready", True, True,
                           ["simulation"], None, {}, component_id + "-instance", "router-1")


def _arm_state(timestamp, positions=None):
    positions = list(positions or [0.0] * 14)
    return ArmJointState(1, 1, timestamp, "mujoco", list(ALL_ARM_JOINT_NAMES), positions,
                         None, "executor-instance", "router-1")


if __name__ == "__main__":
    unittest.main()
