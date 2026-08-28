from __future__ import annotations

import json
import unittest
from types import SimpleNamespace

import numpy as np

from pico_body_tianji.protocol.messages import (
    ArmTargetCommand,
    HandTargetCommand,
    LatchedBool,
    ProtocolError,
    ProtocolEnvelope,
    SessionState,
)
from pico_body_tianji.protocol import topics
from pico_body_tianji.sources.common.session_client import SessionClient
from pico_body_tianji.sources.common.target_mapper import (
    ArmTargetBatch,
    EndEffectorTargetMapper,
)
from pico_body_tianji.sources.mocap.live_node import MocapLiveNode, parse_aligned_hands
from pico_body_tianji.sources.common.target_publisher import TargetPublisher
from pico_body_tianji.sources.pico_controller.controller_frame import ControllerFrame
from pico_body_tianji.sources.pico_controller.source import XRoboControllerOnlySource
from tianji_world_output.config_loader import TianjiConfig


class _Publisher:
    def __init__(self, key: str) -> None:
        self.key = key
        self.payloads: list[bytes] = []

    def put(self, payload, **kwargs) -> None:
        self.payloads.append(bytes(payload))

    def undeclare(self) -> None:
        pass


class _Session:
    def __init__(self) -> None:
        self.publishers: dict[str, _Publisher] = {}
        self.subscribers: list[tuple[str, object]] = []
        self.queryables: list[tuple[str, object]] = []
        self.puts: list[tuple[str, bytes]] = []
        self.get_callbacks: dict[str, object] = {}

    def declare_publisher(self, key: str, **kwargs):
        publisher = _Publisher(key)
        self.publishers[key] = publisher
        return publisher

    def declare_subscriber(self, key: str, callback):
        self.subscribers.append((key, callback))
        return SimpleNamespace(undeclare=lambda: None)

    def declare_queryable(self, key: str, callback):
        self.queryables.append((key, callback))
        return SimpleNamespace(undeclare=lambda: None)

    def put(self, key: str, payload, **kwargs) -> None:
        self.puts.append((key, bytes(payload)))
    def get(self, key: str, callback, **kwargs) -> None:
        # Query replies are supplied explicitly by each behavior test.
        self.get_callbacks[key] = callback
        return None
    def close(self) -> None:
        pass


class CanonicalMapperTest(unittest.TestCase):
    def test_mapper_uses_canonical_names_and_preserves_geometry(self) -> None:
        config = TianjiConfig.load()
        mapper = EndEffectorTargetMapper(config, rate=90.0)
        frame = ControllerFrame.from_poses(
            [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
            [-0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
        )
        self.assertEqual(mapper.initialize(frame), {"pico_left_wrist", "pico_right_wrist"})
        targets = mapper.map_relative_controller_frame(frame)
        self.assertIsInstance(targets, ArmTargetBatch)
        np.testing.assert_allclose(targets.left_pose[:3], config.init_pos["left"])
        np.testing.assert_allclose(targets.right_pose[:3], config.init_pos["right"])
        self.assertFalse(hasattr(mapper, "map_frame"))
        self.assertFalse(hasattr(mapper, "map_absolute_poses"))


class TargetPublisherTest(unittest.TestCase):
    def test_publisher_emits_typed_arm_and_wrist_relative_hand(self) -> None:
        session = _Session()
        publisher = TargetPublisher(
            session,
            source="unit-test",
            publisher_instance_id="source-instance",
            router_zid="router-zid",
        )
        arm = publisher.publish_arm_target(
            side="right",
            position_m=[0.1, 0.2, 0.3],
            orientation_xyzw=[0.0, 0.0, 0.0, 1.0],
            elbow_reference_direction=[0.0, 1.0, 0.0],
            source_timestamp_ns=123,
        )
        points = np.arange(63, dtype=float).reshape(21, 3).tolist()
        hand = publisher.publish_hand_target(
            side="right", keypoints_m=(np.asarray(points) - points[0]).tolist(), source_timestamp_ns=123
        )
        self.assertIsInstance(arm, ArmTargetCommand)
        self.assertIsInstance(hand, HandTargetCommand)
        self.assertEqual(arm.frame_id, "Base_R")
        self.assertEqual(hand.keypoints_m[0], [0.0, 0.0, 0.0])
        arm_wire = ArmTargetCommand.from_dict(
            json.loads(session.publishers[topics.arm_target("right")].payloads[-1])
        )
        hand_wire = HandTargetCommand.from_dict(
            json.loads(session.publishers[topics.hand_target("right")].payloads[-1])
        )
        self.assertEqual(arm_wire.envelope.publisher_instance_id, "source-instance")
        self.assertEqual(hand_wire.router_zid, "router-zid")
        with self.assertRaises(ProtocolError):
            publisher.publish_hand_target(side="right", keypoints_m=points)


class SessionClientTest(unittest.TestCase):
    def test_start_intent_waits_for_matching_authoritative_teleop_state(self) -> None:
        session = _Session()
        client = SessionClient(
            session,
            source="unit-test",
            publisher_instance_id="source-instance",
            router_zid="router-zid",
            expected_coordinator_instance_id="coordinator-instance",
        )
        client.start()
        intent_sequence = client.request_start("operator")
        intent = json.loads(session.publishers[topics.SESSION_INTENT].payloads[-1])
        self.assertEqual(intent["publisher_instance_id"], "source-instance")
        self.assertFalse(client.start_authorized)
        state = SessionState(
            schema_version=1,
            sequence=8,
            timestamp_ns=20,
            state="teleop",
            reason="accepted",
            source="coordinator",
            intent_sequence=intent_sequence,
            publisher_instance_id="coordinator-instance",
            router_zid="router-zid",
        )
        client._on_state_payload(json.dumps(state.to_dict()).encode())
        self.assertTrue(client.start_authorized)
        client.close()


class MocapLiveTest(unittest.TestCase):
    @staticmethod
    def _payload(instance: str = "stream-1", sequence: int = 1, *, left_valid: bool = False) -> dict:
        right_points = np.arange(63, dtype=float).reshape(21, 3).tolist()
        right_points[0] = [0.1, 0.2, 0.3]
        return {
            "stream_instance_id": instance,
            "stream_sequence": sequence,
            "router_zid": "router-zid",
            "time_ns": 900 + sequence,
            "frame_index": sequence,
            "frame_valid": False,
            "hands": {
                "left": {"valid": left_valid, "wrist_pose": None, "keypoints_world_m": None},
                "right": {
                    "valid": True,
                    "wrist_pose": [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
                    "keypoints_world_m": right_points,
                },
            },
        }

    def test_live_uses_aligned_key_only_and_waits_for_authority(self) -> None:
        session = _Session()
        node = MocapLiveNode(
            session,
            publisher_instance_id="live-instance",
            router_zid="router-zid",
            coordinator_instance_id="coordinator-instance",
        )
        declared_keys = [key for key, _callback in session.subscribers]
        self.assertIn(topics.MOCAP_ALIGNED_HANDS, declared_keys)
        self.assertNotIn(topics.MOCAP_HANDS_FRAME, declared_keys)
        node._on_aligned_payload(self._payload())
        self.assertFalse(node.request_start())
        snapshot_state = SessionState(
            schema_version=1,
            sequence=1,
            timestamp_ns=10,
            state="idle",
            reason="ready",
            source="coordinator",
            intent_sequence=None,
            publisher_instance_id="coordinator-instance",
            router_zid="router-zid",
        )
        session.get_callbacks[topics.SESSION_STATE](snapshot_state.to_dict())
        session.get_callbacks[topics.AT_HOME](
            LatchedBool(1, 1, 10, True, "coordinator-instance", "router-zid").to_dict()
        )
        session.get_callbacks[topics.RETURN_COMPLETE](
            LatchedBool(1, 1, 10, False, "coordinator-instance", "router-zid").to_dict()
        )
        state = SessionState(
            schema_version=1,
            sequence=1,
            timestamp_ns=10,
            state="idle",
            reason="ready",
            source="coordinator",
            intent_sequence=None,
            publisher_instance_id="coordinator-instance",
            router_zid="router-zid",
        )
        node._session_client._on_state_payload(json.dumps(state.to_dict()).encode())
        self.assertTrue(node.request_start())
        self.assertEqual(node.phase, "start_pending")
        node._session_client._on_state_payload(
            json.dumps(
                SessionState(
                    schema_version=1,
                    sequence=2,
                    timestamp_ns=11,
                    state="teleop",
                    reason="accepted",
                    source="coordinator",
                    intent_sequence=node._session_client.pending_intent_sequence,
                    publisher_instance_id="coordinator-instance",
                    router_zid="router-zid",
                ).to_dict()
            ).encode()
        )
        node._tick(now=node._received_at)
        self.assertEqual(node.phase, "teleop")
        node._tick(now=node._received_at)
        self.assertIn(topics.arm_target("right"), session.publishers)
        hand = HandTargetCommand.from_dict(
            json.loads(session.publishers[topics.hand_target("right")].payloads[-1])
        )
        self.assertEqual(hand.keypoints_m[0], [0.0, 0.0, 0.0])
        node._on_aligned_payload(self._payload("stream-2", 1))
        self.assertEqual(node.phase, "returning")
        node.close()


class PicoSourceTest(unittest.TestCase):
    def test_source_stays_controller_only_and_does_not_read_body(self) -> None:
        class SDK:
            def init(self):
                pass
            def close(self):
                pass
            def get_left_controller_pose(self):
                return [0, 0, 0, 0, 0, 0, 1]
            def get_right_controller_pose(self):
                return [0, 0, 0, 0, 0, 0, 1]
            def get_time_stamp_ns(self):
                return 321
            def get_A_button(self):
                return False
            def is_body_data_available(self):
                raise AssertionError("body API must not be accessed")

        source = XRoboControllerOnlySource(sdk=SDK())
        source.open()
        try:
            sample = source.read()
        finally:
            source.close()
        self.assertEqual(sample.source_timestamp_ns, 321)
        self.assertFalse(hasattr(sample, "body_frame"))


if __name__ == "__main__":
    unittest.main()
