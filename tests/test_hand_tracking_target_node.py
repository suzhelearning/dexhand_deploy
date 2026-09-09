from __future__ import annotations

import ast
import sys
import types
import unittest
from pathlib import Path

import numpy as np

if "zenoh" not in sys.modules:
    sys.modules["zenoh"] = types.SimpleNamespace(
        Reliability=types.SimpleNamespace(RELIABLE="reliable"),
    )

from tianji_teleop.hand_tracking.target_bridge import (
    BridgeTargets,
    PreparedArmTarget,
)
from tianji_teleop.hand_tracking.hand_target_adapter import PreparedHandTarget
from tianji_teleop.hand_tracking.target_node import _load_config
from tianji_teleop.hand_tracking.target_node import HandTrackingTargetNode
from tianji_teleop.hand_tracking.target_bridge import TargetBridgeInputRejected


class _FakeSessionClient:
    def __init__(self) -> None:
        self.startup_ready = True
        self.start_authorized = False
        self.return_completion_fresh = False
        self.pending_intent_sequence: int | None = None
        self.start_requests: list[str] = []
        self.return_requests: list[str] = []
        self.poll_count = 0

    def poll(self) -> None:
        self.poll_count += 1

    def request_start(self, reason: str) -> int:
        self.start_requests.append(reason)
        self.pending_intent_sequence = 1
        return 1

    def request_return(self, reason: str) -> int:
        self.return_requests.append(reason)
        self.pending_intent_sequence = 2
        return 2

    def close(self) -> None:
        return None


class _FakeBridge:
    def __init__(self, result: BridgeTargets | None = None) -> None:
        self.result = result or BridgeTargets()
        self.started = False
        self.start_count = 0
        self.reset_count = 0
        self.hand_inputs: list[object] = []
        self.arm_inputs: list[object] = []
        self.error: Exception | None = None

    def ingest_hand_observation(self, value: object, *, now_ns: int | None = None) -> object:
        if self.error is not None:
            raise self.error
        self.hand_inputs.append(value)
        return value

    def ingest_arm_observation(self, value: object) -> object:
        if self.error is not None:
            raise self.error
        self.arm_inputs.append(value)
        return value

    def start(self, *, now_ns: int | None = None) -> None:
        self.start_count += 1
        self.started = True

    def reset(self) -> None:
        self.reset_count += 1
        self.started = False

    def tick(self, *, now_ns: int | None = None) -> BridgeTargets:
        if self.error is not None:
            raise self.error
        return self.result


class _FakeTargetPublisher:
    def __init__(self) -> None:
        self.hand: list[dict] = []
        self.arm: list[dict] = []
        self.status: list[dict] = []
        self.closed = False

    def publish_hand_target(self, **kwargs):
        self.hand.append(kwargs)
        return kwargs

    def publish_arm_target(self, **kwargs):
        self.arm.append(kwargs)
        return kwargs

    def publish_source_status(self, **kwargs):
        self.status.append(kwargs)
        return kwargs

    def close(self) -> None:
        self.closed = True


def _targets() -> BridgeTargets:
    hand = PreparedHandTarget(
        side="right",
        keypoints_m=np.zeros((21, 3)),
        source_timestamp_ns=10,
        observation_sequence=2,
        observation_publisher_instance_id="obs",
        frame_association_id="frame-2",
        adapter_version="fixed_rotation_v1",
    )
    arm = PreparedArmTarget(
        side="right",
        pose=np.array([0.5, -0.2, 0.3, 0.0, 0.0, 0.0, 1.0]),
        elbow_reference_direction=(0.0, 1.0, 0.0),
        source_timestamp_ns=10,
        observation_sequence=2,
        observation_publisher_instance_id="obs",
        frame_association_id="frame-2",
        mapping_backend="relative_home",
        processor_backend="passthrough",
    )
    return BridgeTargets(hand=(hand,), arm=(arm,))


class HandTrackingTargetNodeTest(unittest.TestCase):
    def _node(self, result: BridgeTargets | None = None):
        client = _FakeSessionClient()
        publisher = _FakeTargetPublisher()
        bridge = _FakeBridge(result)
        node = HandTrackingTargetNode(
            bridge=bridge,
            session_client=client,
            target_publisher=publisher,
            active_sides=("right",),
            active_hand_sides=("right",),
            rate_hz=60.0,
            clock=lambda: 1_000_000_000,
        )
        return node, bridge, client, publisher

    def test_targets_are_never_published_before_authorized_teleop(self) -> None:
        node, bridge, client, publisher = self._node(_targets())

        node.tick(now_ns=1_000_000_000)
        self.assertEqual(publisher.hand, [])
        self.assertEqual(publisher.arm, [])
        self.assertEqual(bridge.reset_count, 0)

        self.assertTrue(node.request_start())
        self.assertEqual(bridge.start_count, 1)
        node.tick(now_ns=1_000_000_000)
        self.assertEqual(publisher.hand, [])
        client.start_authorized = True
        node.tick(now_ns=1_000_000_000)
        self.assertEqual(node.phase, "teleop")
        self.assertEqual(publisher.hand, [])
        node.tick(now_ns=1_000_000_000)
        self.assertEqual(len(publisher.hand), 1)
        self.assertEqual(len(publisher.arm), 1)
        self.assertEqual(publisher.hand[0]["source_timestamp_ns"], 10)
        self.assertEqual(publisher.arm[0]["source_timestamp_ns"], 10)

    def test_revoked_teleop_authorization_stops_both_target_streams(self) -> None:
        node, bridge, client, publisher = self._node(_targets())
        self.assertTrue(node.request_start())
        client.start_authorized = True
        node.tick()
        node.tick()
        self.assertEqual(len(publisher.hand), 1)
        self.assertEqual(len(publisher.arm), 1)
        client.start_authorized = False
        result = node.tick()
        self.assertEqual(result, BridgeTargets())
        self.assertEqual(len(publisher.hand), 1)
        self.assertEqual(len(publisher.arm), 1)
        self.assertEqual(node.phase, "returning")
        self.assertEqual(client.return_requests, ["coordinator_authorization_lost"])

    def test_bridge_rejection_requests_return_and_drops_outputs(self) -> None:
        node, bridge, client, publisher = self._node(_targets())
        self.assertTrue(node.request_start())
        client.start_authorized = True
        node.tick(now_ns=1_000_000_000)
        bridge.error = TargetBridgeInputRejected("stale")

        node.tick(now_ns=1_000_000_000)

        self.assertEqual(node.phase, "returning")
        self.assertEqual(client.return_requests, ["target_bridge_rejected"])
        self.assertEqual(publisher.hand, [])
        self.assertEqual(publisher.arm, [])

        client.return_completion_fresh = True
        node.tick(now_ns=1_000_000_000)
        self.assertEqual(node.phase, "armed")
        self.assertGreaterEqual(bridge.reset_count, 1)

    def test_observation_input_is_forwarded_and_invalid_input_does_not_publish(self) -> None:
        node, bridge, client, publisher = self._node()
        node.on_hand_observation({"sequence": 1})
        node.on_arm_observation({"sequence": 1})
        self.assertEqual(len(bridge.hand_inputs), 1)
        self.assertEqual(len(bridge.arm_inputs), 1)
        node.tick(now_ns=1_000_000_000)
        self.assertEqual(publisher.hand, [])
        self.assertEqual(publisher.arm, [])
        self.assertEqual(client.return_requests, [])

    def test_target_node_has_no_robot_control_or_ik_dependency(self) -> None:
        tree = ast.parse(
            Path("src/tianji_teleop/tianji_teleop/hand_tracking/target_node.py").read_text(
                encoding="utf-8"
            )
        )
        imports = []
        for item in ast.walk(tree):
            if isinstance(item, ast.Import):
                imports.extend(alias.name for alias in item.names)
            elif isinstance(item, ast.ImportFrom):
                imports.append(item.module or "")
        joined = "\n".join(imports)
        self.assertNotIn("executors", joined)
        self.assertNotIn("producers", joined)
        self.assertNotIn("coordinator", joined)
        self.assertNotIn("ik", joined.lower())


class HandTrackingTargetConfigTest(unittest.TestCase):
    def test_config_requires_simulation_only_and_profile_specific_sources(self) -> None:
        import tempfile
        from pathlib import Path

        good = """
input_profile: pico
simulation_only: true
active_sides: [right]
active_hand_sides: [right]
arm_input_source: pico
hand_adapter: fixed_rotation
hand_adapter_config:
  source: pico
  coordinate_frame: pico_tracking_initial_wrist_relative
  max_age_s: 0.5
arm_pose_mapper: relative_home
arm_pose_mapper_config: {}
arm_target_processor: passthrough
arm_target_processor_config: {}
elbow_reference_direction:
  left: [0, -1, 0]
  right: [0, 1, 0]
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "target.yaml"
            path.write_text(good, encoding="utf-8")
            config = _load_config(path)
        self.assertEqual(config["input_profile"], "pico")
        self.assertTrue(config["simulation_only"])

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "target.yaml"
            path.write_text(good.replace("simulation_only: true", "simulation_only: false"), encoding="utf-8")
            with self.assertRaises(ValueError):
                _load_config(path)


if __name__ == "__main__":
    unittest.main()
