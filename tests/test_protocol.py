from __future__ import annotations

import math
import unittest

from pico_body_tianji.protocol.messages import (
    ArmJointCommand,
    ArmJointProposal,
    ArmJointState,
    ArmSolvedPose,
    ArmTargetCommand,
    ComponentStatus,
    Frame0HandSkeleton,
    HandExecutorStatus,
    HandJointCommand,
    HandJointState,
    HandTargetCommand,
    LatchedBool,
    ProtocolEnvelope,
    RawH5ReplaySample,
    RawMocapLiveSample,
    RawPicoControllerSample,
    SafetyStopAck,
    SafetyStopRequest,
    SessionIntent,
    SessionState,
)
from pico_body_tianji.protocol import topics


class ProtocolTopicsTest(unittest.TestCase):
    def test_topics_are_canonical_and_parameterized(self) -> None:
        expected = {
            "SESSION_INTENT": "tianji/session/intent", "SESSION_STATE": "tianji/session/state",
            "SOURCE_STATUS": "tianji/source/status", "ARM_TARGET": "tianji/target/arm/{side}",
            "HAND_TARGET": "tianji/target/hand/{side}", "PRODUCER_STATUS": "tianji/producer/status",
            "ARM_PROPOSAL": "tianji/proposal/arm/{side}", "ARM_SOLVED_POSE": "tianji/producer/arm/{side}/solved_pose",
            "COORDINATOR_STATUS": "tianji/coordinator/status", "AT_HOME": "tianji/coordinator/at_home",
            "RETURN_COMPLETE": "tianji/coordinator/return_complete", "ARM_COMMAND": "tianji/command/arm/{side}",
            "HAND_COMMAND": "tianji/command/hand/{side}", "ARM_STATE": "tianji/state/arm",
            "HAND_STATE": "tianji/state/hand/{side}", "EXECUTOR_STATUS": "tianji/executor/status",
            "HAND_EXECUTOR_STATUS": "tianji/executor/hand/{side}/status", "SAFETY_STOP": "tianji/safety/stop",
            "SAFETY_ACK": "tianji/safety/ack/{executor_id}", "RAW_PICO_CONTROLLER": "tianji/raw/pico_controller",
            "RAW_MOCAP_LIVE": "tianji/raw/mocap_live", "RAW_H5_REPLAY": "tianji/raw/h5_replay",
            "FRAME0_HAND_SKELETON": "tianji/diagnostics/h5/frame0_hand_skeleton",
            "MOCAP_ALIGNED_HANDS": "mocap/aligned/hands", "MOCAP_HANDS_FRAME": "mocap/hands/frame",
            "MOCAP_RIGID_BODY_NAMES": "mocap/rigid_body_names",
        }
        for name, value in expected.items():
            self.assertEqual(getattr(topics, name), value)
        self.assertEqual(topics.hand_target("right"), "tianji/target/hand/right")
        self.assertEqual(topics.arm_proposal("left"), "tianji/proposal/arm/left")


class ProtocolMessagesTest(unittest.TestCase):
    def setUp(self) -> None:
        self.envelope = ProtocolEnvelope(
            schema_version=1,
            publisher_instance_id="pub-1",
            router_zid="router-1",
            sequence=7,
            timestamp_ns=123456,
        )
        self.arm_names = [f"Joint{i}_L" for i in range(1, 8)]
        self.hand_names = [
            "l_thumb_cmc_flex", "l_thumb_cmc_abd", "l_thumb_mcp", "l_thumb_ip",
            "l_index_mcp_flex", "l_index_mcp_abd", "l_index_pip", "l_index_dip",
            "l_middle_mcp_flex", "l_middle_mcp_abd", "l_middle_pip", "l_middle_dip",
            "l_ring_mcp_flex", "l_ring_mcp_abd", "l_ring_pip", "l_ring_dip",
            "l_pinky_mcp_flex", "l_pinky_mcp_abd", "l_pinky_pip", "l_pinky_dip",
        ]
        self.pose = [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]
        self.keypoints = [[0.0, 0.0, 0.0] for _ in range(21)]

    def assert_round_trip(self, message) -> None:
        encoded = message.to_dict()
        self.assertEqual(type(encoded), dict)
        self.assertEqual(message, type(message).from_dict(encoded))

    def test_envelope_and_arm_target_round_trip(self) -> None:
        self.assert_round_trip(
            ArmTargetCommand(
                envelope=self.envelope,
                source_timestamp_ns=None,
                source="pico_controller",
                side="left",
                frame_id="Base_L",
                position_m=[0.1, 0.2, 0.3],
                orientation_xyzw=[0.0, 0.0, 0.0, 1.0],
                elbow_reference_direction=[1.0, 0.0, 0.0],
            )
        )

    def test_arm_messages_round_trip(self) -> None:
        self.assert_round_trip(ArmJointProposal(
            schema_version=1, sequence=8, timestamp_ns=9, producer="ik",
            side="left", target_sequence=None, names=self.arm_names,
            position_rad=[0.0] * 7, diagnostics={"iterations": 1},
            publisher_instance_id="ik-1", router_zid="router-1",
        ))
        self.assert_round_trip(ArmSolvedPose(
            envelope=self.envelope, producer="ik", side="right", frame_id="Base_R",
            target_sequence=7, position_m=[0.1, 0.2, 0.3],
            orientation_xyzw=[0.0, 0.0, 0.0, 1.0],
        ))
        self.assert_round_trip(ArmJointCommand(
            schema_version=1, sequence=10, timestamp_ns=11, producer="coordinator",
            side="left", mode="teleop", proposal_sequence=8, target_sequence=7,
            names=self.arm_names, position_rad=[0.0] * 7,
            publisher_instance_id="coord-1", router_zid="router-1",
        ))
        self.assert_round_trip(ArmJointState(
            schema_version=1, sequence=12, timestamp_ns=13, executor="mujoco",
            names=self.arm_names + [f"Joint{i}_R" for i in range(1, 8)],
            position_rad=[0.0] * 14, velocity_rad_s=None,
            publisher_instance_id="exec-1", router_zid="router-1",
        ))

    def test_hand_messages_round_trip(self) -> None:
        self.assert_round_trip(HandTargetCommand(
            schema_version=1, sequence=1, timestamp_ns=2, source_timestamp_ns=None,
            source="mocap_live", side="left", frame_id="wrist_relative_mediapipe",
            keypoints_m=self.keypoints, publisher_instance_id="source-1", router_zid="router-1",
        ))
        self.assert_round_trip(HandJointCommand(
            schema_version=1, sequence=3, timestamp_ns=4, producer="retarget",
            side="left", names=self.hand_names, position_rad=[0.0] * 20,
            publisher_instance_id="retarget-1", router_zid="router-1",
        ))
        self.assert_round_trip(HandJointState(
            schema_version=1, sequence=5, timestamp_ns=6, executor="wuji",
            side="left", names=self.hand_names, position_rad=[0.0] * 20,
            velocity_rad_s=None, publisher_instance_id="wuji-1", router_zid="router-1",
        ))

    def test_session_status_and_safety_round_trip(self) -> None:
        self.assert_round_trip(SessionIntent(
            schema_version=1, sequence=1, timestamp_ns=2, source="user",
            action="start", reason="button", publisher_instance_id="source-1", router_zid="router-1",
        ))
        self.assert_round_trip(SessionState(
            schema_version=1, sequence=3, timestamp_ns=4, state="teleop",
            reason="ready", source="coordinator", intent_sequence=1,
            publisher_instance_id="coord-1", router_zid="router-1",
        ))
        self.assert_round_trip(LatchedBool(schema_version=1, sequence=5, timestamp_ns=6, value=True, publisher_instance_id="coord-1", router_zid="router-1"))
        self.assert_round_trip(ComponentStatus(
            schema_version=1, sequence=7, timestamp_ns=7, component_role="producer_arm",
            component_id="ik-1", phase="ready", ready=True, healthy=True,
            capabilities=["simulation"], error=None, diagnostics={"backend": "pinocchio_cpp"},
            publisher_instance_id="ik-1", router_zid="router-1",
        ))
        self.assert_round_trip(HandExecutorStatus(
            schema_version=1, publisher_instance_id="hand-1", router_zid="router-1",
            sequence=8, timestamp_ns=9, side="left", ready=True, healthy=True,
            at_zero=True, tracking_allowed=False, error=None,
        ))
        self.assert_round_trip(SafetyStopRequest(
            envelope=self.envelope, run_id="run-1", reason="operator", latch=True,
        ))
        self.assert_round_trip(SafetyStopAck(
            envelope=self.envelope, executor_id="arm-1", run_id="run-1",
            latched=True, reason="operator",
        ))

    def test_raw_and_diagnostic_messages_round_trip(self) -> None:
        self.assert_round_trip(RawPicoControllerSample(
            envelope=self.envelope, source_timestamp_ns=None,
            left_pose=self.pose, right_pose=self.pose, right_a_pressed=False,
        ))
        hands = {
            "left": {"valid": True, "wrist_pose": self.pose, "keypoints_world_m": self.keypoints},
            "right": {"valid": False, "wrist_pose": None, "keypoints_world_m": None},
        }
        self.assert_round_trip(RawMocapLiveSample(
            envelope=self.envelope, source_timestamp_ns=88, stream_instance_id="stream-1",
            stream_sequence=4, frame_index=12, hands=hands,
        ))
        self.assert_round_trip(RawH5ReplaySample(
            envelope=self.envelope, source_timestamp_ns=89, hands={
                "left": {"valid": True, "wrist_pose": self.pose, "keypoints_world_m": self.keypoints, "wuji2_joints_rad": [0.0] * 20},
                "right": {"valid": False, "wrist_pose": None, "keypoints_world_m": None, "wuji2_joints_rad": None},
            },
        ))
        self.assert_round_trip(Frame0HandSkeleton(
            schema_version=1, sequence=20, timestamp_ns=20, side="right", frame_id="motive_world",
            keypoints_world_m=self.keypoints, edges=[[i, i + 1] for i in range(20)],
            manus_wrist_pose=self.pose, robot_wrist_home_pose=self.pose,
            target_wrist_pose=self.pose, tcp_to_wrist_pose=self.pose,
            publisher_instance_id="diag-1", router_zid="router-1",
        ))

    def test_every_message_parser_rejects_unknown_field(self) -> None:
        hand_records = {
            "left": {"valid": False, "wrist_pose": None, "keypoints_world_m": None},
            "right": {"valid": False, "wrist_pose": None, "keypoints_world_m": None},
        }
        messages = [
            self.envelope,
            ArmTargetCommand(self.envelope, None, "source", "left", "Base_L", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
            ArmJointProposal(1, 1, 1, "ik", "left", None, self.arm_names, [0.0] * 7, {}, "ik-1", "router-1"),
            ArmSolvedPose(self.envelope, "ik", "right", "Base_R", None, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
            ArmJointCommand(1, 1, 1, "coordinator", "left", "idle", None, None, self.arm_names, [0.0] * 7, "coord-1", "router-1"),
            ArmJointState(1, 1, 1, "mujoco", self.arm_names + [f"Joint{i}_R" for i in range(1, 8)], [0.0] * 14, None, "exec-1", "router-1"),
            HandTargetCommand(1, 1, 1, None, "source", "left", "wrist_relative_mediapipe", self.keypoints, "source-1", "router-1"),
            HandJointCommand(1, 1, 1, "retarget", "left", self.hand_names, [0.0] * 20, "retarget-1", "router-1"),
            HandJointState(1, 1, 1, "wuji", "left", self.hand_names, [0.0] * 20, None, "wuji-1", "router-1"),
            SessionIntent(1, 1, 1, "source", "start", "reason", "source-1", "router-1"),
            SessionState(1, 1, 1, "idle", "reason", "coordinator", None, "coord-1", "router-1"),
            LatchedBool(1, 1, 1, True, "coord-1", "router-1"),
            ComponentStatus(1, 1, 1, "source", "source-1", "ready", True, True, ["simulation"], None, {}, "source-1", "router-1"),
            HandExecutorStatus(1, 1, 1, "left", True, True, True, False, None, "hand-1", "router-1"),
            SafetyStopRequest(self.envelope, "run-1", "stop"),
            SafetyStopAck(self.envelope, "arm-1", "run-1", True, "stop"),
            RawPicoControllerSample(self.envelope, None, self.pose, self.pose, False),
            RawMocapLiveSample(self.envelope, None, "stream-1", 1, 1, hand_records),
            RawH5ReplaySample(self.envelope, None, {"left": {**hand_records["left"], "wuji2_joints_rad": None}, "right": {**hand_records["right"], "wuji2_joints_rad": None}}),
            Frame0HandSkeleton(1, 1, "right", "motive_world", self.keypoints, [[i, i + 1] for i in range(20)], self.pose, self.pose, self.pose, self.pose, 1, "diag-1", "router-1"),
        ]
        for message in messages:
            payload = message.to_dict()
            payload["unknown_field"] = True
            with self.assertRaises(ValueError, msg=type(message).__name__):
                type(message).from_dict(payload)
    def test_rejects_unknown_schema_missing_fields_and_bad_shapes(self) -> None:
        payload = self.envelope.to_dict()
        payload["schema_version"] = 2
        with self.assertRaises(ValueError):
            ProtocolEnvelope.from_dict(payload)
        with self.assertRaises(ValueError):
            ProtocolEnvelope.from_dict({"schema_version": 1})
        with self.assertRaises(ValueError):
            ArmTargetCommand.from_dict({**ArmTargetCommand(
                envelope=self.envelope, source_timestamp_ns=None, source="x", side="left",
                frame_id="Base_L", position_m=[0.1, 0.2, 0.3], orientation_xyzw=[0, 0, 0, 1],
                elbow_reference_direction=[1, 0, 0],
            ).to_dict(), "position_m": [0.1, 0.2]})
        unknown = self.envelope.to_dict()
        unknown["unexpected"] = 1
        with self.assertRaises(ValueError):
            ProtocolEnvelope.from_dict(unknown)
        target_unknown = ArmTargetCommand(
            envelope=self.envelope, source_timestamp_ns=None, source="x", side="left",
            frame_id="Base_L", position_m=[0.1, 0.2, 0.3], orientation_xyzw=[0, 0, 0, 1],
            elbow_reference_direction=[1, 0, 0],
        ).to_dict()
        target_unknown["unexpected"] = True
        with self.assertRaises(ValueError):
            ArmTargetCommand.from_dict(target_unknown)
        with self.assertRaises(ValueError):
            ArmTargetCommand(
                envelope=self.envelope, source_timestamp_ns=None, source="x", side="left",
                frame_id="Base_L", position_m=[0.1, 0.2, 0.3], orientation_xyzw=[0, 0, 0, 0],
                elbow_reference_direction=[1, 0, 0],
            )
        with self.assertRaises(ValueError):
            ArmTargetCommand(
                envelope=self.envelope, source_timestamp_ns=None, source="x", side="right",
                frame_id="Base_L", position_m=[0.1, 0.2, 0.3], orientation_xyzw=[0, 0, 0, 1],
                elbow_reference_direction=[1, 0, 0],
            )
        with self.assertRaises(ValueError):
            HandTargetCommand(
                schema_version=1, sequence=1, timestamp_ns=2, source_timestamp_ns=None,
                source="x", side="left", frame_id="wrist_relative_mediapipe",
                keypoints_m=[[1.0, 0.0, 0.0]] + self.keypoints[1:],
                publisher_instance_id="source-1", router_zid="router-1",
            )
        with self.assertRaises(ValueError):
            ArmTargetCommand(
                envelope=self.envelope, source_timestamp_ns=None, source="x", side="left",
                frame_id="Base_L", position_m=[0.1, 0.2, 0.3], orientation_xyzw=[0, 0, 0, 1],
                elbow_reference_direction=[0, 0, 0],
            )

    def test_rejects_nonfinite_envelope_geometry_and_wrong_orders(self) -> None:
        with self.assertRaises(ValueError):
            ProtocolEnvelope(1, "pub", "router", 1, math.nan)
        with self.assertRaises(ValueError):
            ArmJointProposal(1, 1, 1, "ik", "left", None, self.arm_names, [math.inf] * 7, {}, "ik-1", "router-1")
        with self.assertRaises(ValueError):
            HandJointCommand(1, 1, 1, "retarget", "left", self.hand_names[::-1], [0.0] * 20, "hand-1", "router-1")
        target = ArmTargetCommand(
            envelope=self.envelope, source_timestamp_ns=None, source="x", side="left",
            frame_id="Base_L", position_m=[0.1, 0.2, 0.3], orientation_xyzw=[0, 0, 0, 2],
            elbow_reference_direction=[2, 0, 0],
        )
        self.assertEqual(target.orientation_xyzw, [0.0, 0.0, 0.0, 1.0])
        self.assertEqual(target.elbow_reference_direction, [1.0, 0.0, 0.0])
        malformed = target.to_dict()
        malformed["orientation_xyzw"] = [0.0, 0.0, 0.0, 2.0]
        with self.assertRaises(ValueError):
            ArmTargetCommand.from_dict(malformed)
        with self.assertRaises(ValueError):
            Frame0HandSkeleton(1, 1, "left", "motive_world", self.keypoints, [[0, 1]] * 19,
                               self.pose, self.pose, self.pose, self.pose, 1, "diag-1", "router-1")

    def test_safety_authority_and_run_validation(self) -> None:
        request = SafetyStopRequest(self.envelope, "run-1", "operator")
        request.validate_authority("pub-1", "run-1")
        with self.assertRaises(ValueError):
            request.validate_authority("other-supervisor", "run-1")
        with self.assertRaises(ValueError):
            request.validate_authority("pub-1", "other-run")
        with self.assertRaises(ValueError):
            SafetyStopRequest(self.envelope, "run-1", "operator", latch=False)
        ack = SafetyStopAck(self.envelope, "arm-1", "run-1", True, "operator")
        ack.validate_for("arm-1", "run-1")
        with self.assertRaises(ValueError):
            ack.validate_for("arm-2", "run-1")
        with self.assertRaises(ValueError):
            ack.validate_for("arm-1", "other-run")
        with self.assertRaises(ValueError):
            SafetyStopAck(self.envelope, "arm-1", "run-1", False, "operator").validate_for("arm-1", "run-1")

    def test_rejects_nested_non_json_diagnostics_and_topic_side(self) -> None:
        for invalid in (math.nan, math.inf):
            with self.assertRaises(ValueError):
                ComponentStatus(1, 1, 2, "producer_arm", "ik", "ready", True, True, ["simulation"], None, {"nested": {"bad": invalid}}, "ik-1", "router-1")
        with self.assertRaises(ValueError):
            ComponentStatus(1, 1, 2, "producer_arm", "ik", "ready", True, True, ["simulation"], None, {"nested": {"bad": object()}}, "ik-1", "router-1")
        for helper in (topics.arm_target, topics.hand_target, topics.arm_proposal, topics.arm_solved_pose, topics.arm_command, topics.hand_command, topics.hand_state, topics.hand_executor_status):
            with self.assertRaises(ValueError):
                helper("bad")

    def test_direct_wire_constructor_requires_identity(self) -> None:
        with self.assertRaises(TypeError):
            ArmJointProposal(1, 1, 1, "ik", "left", None, self.arm_names, [0.0] * 7, {})



    def test_raw_discriminators_and_invalid_hand_fields(self) -> None:
        live = RawMocapLiveSample(
            envelope=self.envelope, source_timestamp_ns=None, stream_instance_id="stream-1",
            stream_sequence=1, frame_index=1, hands={
                "left": {"valid": False, "wrist_pose": None, "keypoints_world_m": None},
                "right": {"valid": False, "wrist_pose": None, "keypoints_world_m": None},
            },
        ).to_dict()
        live["source_type"] = "h5_replay"
        with self.assertRaises(ValueError):
            RawMocapLiveSample.from_dict(live)
        h5 = RawH5ReplaySample(
            envelope=self.envelope, source_timestamp_ns=None, hands={
                "left": {"valid": False, "wrist_pose": None, "keypoints_world_m": None, "wuji2_joints_rad": None},
                "right": {"valid": False, "wrist_pose": None, "keypoints_world_m": None, "wuji2_joints_rad": None},
            },
        ).to_dict()
        h5["source_type"] = "mocap_live"
        with self.assertRaises(ValueError):
            RawH5ReplaySample.from_dict(h5)
        bad_hand = dict(live)
        bad_hand["source_type"] = "mocap_live"
        bad_hand["hands"] = dict(bad_hand["hands"])
        bad_hand["hands"]["left"] = {"valid": False, "wrist_pose": self.pose, "keypoints_world_m": None}
        with self.assertRaises(ValueError):
            RawMocapLiveSample.from_dict(bad_hand)

    def test_all_direct_wire_constructors_require_identity(self) -> None:
        missing = [
            lambda: ArmJointCommand(1, 1, 1, "coordinator", "left", "idle", None, None, self.arm_names, [0.0] * 7),
            lambda: ArmJointState(1, 1, 1, "mujoco", self.arm_names + [f"Joint{i}_R" for i in range(1, 8)], [0.0] * 14, None),
            lambda: HandTargetCommand(1, 1, 1, None, "source", "left", "wrist_relative_mediapipe", self.keypoints),
            lambda: HandJointCommand(1, 1, 1, "retarget", "left", self.hand_names, [0.0] * 20),
            lambda: HandJointState(1, 1, 1, "wuji", "left", self.hand_names, [0.0] * 20, None),
            lambda: SessionIntent(1, 1, 1, "source", "start", "reason"),
            lambda: SessionState(1, 1, 1, "idle", "reason", "coordinator", None),
            lambda: LatchedBool(1, 1, 1, True),
            lambda: ComponentStatus(1, 1, 2, "source", "id", "ready", True, True, ["simulation"], None, {}),
            lambda: HandExecutorStatus(1, 1, 1, "left", True, True, True, False, None),
            lambda: Frame0HandSkeleton(1, 1, "left", "motive_world", self.keypoints, [[0, 1]] * 20, self.pose, self.pose, self.pose, self.pose, 1),
        ]
        for constructor in missing:
            with self.assertRaises(TypeError):
                constructor()
    def test_wire_boundaries_and_discriminators(self) -> None:
        target = ArmTargetCommand(
            envelope=self.envelope, source_timestamp_ns=None, source="x", side="left",
            frame_id="Base_L", position_m=[0.1, 0.2, 0.3], orientation_xyzw=[0, 0, 0, 1],
            elbow_reference_direction=[1, 0, 0],
        ).to_dict()
        for norm in (0.999, 1.001):
            bounded = dict(target)
            bounded["orientation_xyzw"] = [0.0, 0.0, 0.0, norm]
            ArmTargetCommand.from_dict(bounded)
        for norm in (0.998, 1.002):
            outside = dict(target)
            outside["orientation_xyzw"] = [0.0, 0.0, 0.0, norm]
            with self.assertRaises(ValueError):
                ArmTargetCommand.from_dict(outside)
        threshold = ArmTargetCommand(
            envelope=self.envelope, source_timestamp_ns=None, source="x", side="left",
            frame_id="Base_L", position_m=[0.1, 0.2, 0.3], orientation_xyzw=[0, 0, 0, 1],
            elbow_reference_direction=[1e-8, 0, 0],
        )
        self.assertEqual(threshold.elbow_reference_direction, [1.0, 0.0, 0.0])
        with self.assertRaises(ValueError):
            ArmTargetCommand(
                envelope=self.envelope, source_timestamp_ns=None, source="x", side="left",
                frame_id="Base_L", position_m=[0.1, 0.2, 0.3], orientation_xyzw=[0, 0, 0, 1],
                elbow_reference_direction=[1e-9, 0, 0],
            )
        with self.assertRaises(ValueError):
            ArmJointCommand(
                schema_version=1, sequence=1, timestamp_ns=2, producer="coordinator",
                side="left", mode="teleop", proposal_sequence=None, target_sequence=None,
                names=[f"Joint{i}_R" for i in range(1, 8)], position_rad=[0.0] * 7,
                publisher_instance_id="coord-1", router_zid="router-1",
            )
        with self.assertRaises(ValueError):
            RawPicoControllerSample(
                envelope=self.envelope, source_timestamp_ns=None, left_pose=self.pose,
                right_pose=self.pose, right_a_pressed=False, source_type="bad",
            )
        with self.assertRaises(ValueError):
            SessionIntent(1, 1, 1, "source", "bad", "reason", "source-1", "router-1")
        with self.assertRaises(ValueError):
            SessionState(1, 1, 1, "bad", "reason", "source", None, "coord-1", "router-1")
        with self.assertRaises(ValueError):
            ComponentStatus(1, 1, 2, "bad", "id", "ready", True, True, ["simulation"], None, {}, "id", "router-1")

if __name__ == "__main__":
    unittest.main()
