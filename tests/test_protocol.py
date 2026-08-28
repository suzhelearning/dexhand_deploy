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
        self.assertEqual(topics.SESSION_INTENT, "tianji/session/intent")
        self.assertEqual(topics.SOURCE_STATUS, "tianji/source/status")
        self.assertEqual(topics.MOCAP_ALIGNED_HANDS, "mocap/aligned/hands")
        self.assertEqual(topics.ARM_TARGET.format(side="left"), "tianji/target/arm/left")
        self.assertEqual(topics.ARM_COMMAND.format(side="right"), "tianji/command/arm/right")
        self.assertEqual(topics.SAFETY_ACK.format(executor_id="arm-1"), "tianji/safety/ack/arm-1")


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
        ))
        self.assert_round_trip(ArmJointState(
            schema_version=1, sequence=12, timestamp_ns=13, executor="mujoco",
            names=self.arm_names + [f"Joint{i}_R" for i in range(1, 8)],
            position_rad=[0.0] * 14, velocity_rad_s=None,
        ))

    def test_hand_messages_round_trip(self) -> None:
        self.assert_round_trip(HandTargetCommand(
            schema_version=1, sequence=1, timestamp_ns=2, source_timestamp_ns=None,
            source="mocap_live", side="left", frame_id="wrist_relative_mediapipe",
            keypoints_m=self.keypoints,
        ))
        self.assert_round_trip(HandJointCommand(
            schema_version=1, sequence=3, timestamp_ns=4, producer="retarget",
            side="left", names=self.hand_names, position_rad=[0.0] * 20,
        ))
        self.assert_round_trip(HandJointState(
            schema_version=1, sequence=5, timestamp_ns=6, executor="wuji",
            side="left", names=self.hand_names, position_rad=[0.0] * 20,
            velocity_rad_s=None,
        ))

    def test_session_status_and_safety_round_trip(self) -> None:
        self.assert_round_trip(SessionIntent(
            schema_version=1, sequence=1, timestamp_ns=2, source="user",
            action="start", reason="button",
        ))
        self.assert_round_trip(SessionState(
            schema_version=1, sequence=3, timestamp_ns=4, state="teleop",
            reason="ready", source="coordinator", intent_sequence=1,
        ))
        self.assert_round_trip(LatchedBool(schema_version=1, sequence=5, timestamp_ns=6, value=True))
        self.assert_round_trip(ComponentStatus(
            schema_version=1, timestamp_ns=7, component_role="producer_arm",
            component_id="ik-1", phase="ready", ready=True, healthy=True,
            capabilities=["simulation"], error=None, diagnostics={"backend": "pinocchio_cpp"},
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
            schema_version=1, timestamp_ns=20, side="right", frame_id="motive_world",
            keypoints_world_m=self.keypoints, edges=[[i, i + 1] for i in range(20)],
            manus_wrist_pose=self.pose, robot_wrist_home_pose=self.pose,
            target_wrist_pose=self.pose, tcp_to_wrist_pose=self.pose,
        ))

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
            )

    def test_rejects_nonfinite_envelope_geometry_and_wrong_orders(self) -> None:
        with self.assertRaises(ValueError):
            ProtocolEnvelope(1, "pub", "router", 1, math.nan)
        with self.assertRaises(ValueError):
            ArmJointProposal(1, 1, 1, "ik", "left", None, self.arm_names, [math.inf] * 7, {})
        with self.assertRaises(ValueError):
            HandJointCommand(1, 1, 1, "retarget", "left", self.hand_names[::-1], [0.0] * 20)
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
                               self.pose, self.pose, self.pose, self.pose)


if __name__ == "__main__":
    unittest.main()
