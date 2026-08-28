from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from pico_body_tianji.protocol import topics
from pico_body_tianji.protocol.messages import (
    ArmJointCommand,
    HandJointCommand,
    ProtocolEnvelope,
    SessionState,
)
from pico_body_tianji.recording.replay import JointReplayNode, TargetReplaySource
from pico_body_tianji.recording.session_h5 import SessionH5Writer


class _Pub:
    def __init__(self, key): self.key, self.payloads = key, []
    def put(self, payload, **kwargs): self.payloads.append(bytes(payload))
    def undeclare(self): pass


class _Session:
    def __init__(self): self.publishers = {}; self.tokens = []
    def declare_publisher(self, key, **kwargs):
        pub = _Pub(key); self.publishers[key] = pub; return pub
    def declare_subscriber(self, key, callback): return SimpleNamespace(undeclare=lambda: None)
    def declare_queryable(self, key, callback): return SimpleNamespace(undeclare=lambda: None)
    def get(self, key, callback, **kwargs): pass
    def liveliness(self):
        return SimpleNamespace(declare_token=lambda key: self.tokens.append(key) or SimpleNamespace(undeclare=lambda: None))


def _arm_command(sequence, timestamp, side="right"):
    names = [f"Joint{i}_{'R' if side == 'right' else 'L'}" for i in range(1, 8)]
    return ArmJointCommand(1, sequence, timestamp, "ik", side, "teleop", sequence, sequence, names, [0.1] * 7, "recorded", "router")


def _hand_command(sequence, timestamp):
    names = ["r_" + x for x in ("thumb_cmc_flex", "thumb_cmc_abd", "thumb_mcp", "thumb_ip", "index_mcp_flex", "index_mcp_abd", "index_pip", "index_dip", "middle_mcp_flex", "middle_mcp_abd", "middle_pip", "middle_dip", "ring_mcp_flex", "ring_mcp_abd", "ring_pip", "ring_dip", "pinky_mcp_flex", "pinky_mcp_abd", "pinky_pip", "pinky_dip")]
    return HandJointCommand(1, sequence, timestamp, "retarget", "right", names, [0.2] * 20, "recorded", "router")


class ReplayLifecycleTest(unittest.TestCase):
    def _record(self, path):
        with SessionH5Writer(path, source_type="joint_replay", robot_model="marvin", router_zid="router") as writer:
            writer.append_arm_command(_arm_command(1, 10), received_time_ns=100)
            writer.append_arm_command(_arm_command(2, 20), received_time_ns=200)
            writer.append_hand_command(_hand_command(1, 10), received_time_ns=100)

    def test_joint_pause_keeps_recorded_frame_but_refreshes_wire(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "session.h5"; self._record(path); session = _Session()
            node = JointReplayNode(path, session=session, source_publisher_instance_id="source", producer_publisher_instance_id="producer", router_zid="router", active_sides=("right",), inactive_sides=("left",), active_hand_sides=("right",), inactive_hand_sides=("left",))
            start_sequence = node.request_start()
            node.on_session_state(SessionState(1, 1, 1, "teleop", "accepted", "coordinator", start_sequence, "coordinator", "router"))
            node.tick(now_ns=0); node.pause(); node.tick(now_ns=1); node.tick(now_ns=2)
            payloads = session.publishers[topics.arm_proposal("right")].payloads
            self.assertGreaterEqual(len(session.publishers[topics.PRODUCER_STATUS].payloads), 2)
            values = [json.loads(value) for value in payloads]
            self.assertEqual(values[-1]["target_sequence"], 1)
            self.assertGreater(values[-1]["sequence"], values[0]["sequence"])
            self.assertEqual(values[-1]["timestamp_ns"] > values[0]["timestamp_ns"], True)
            node.close()

    def test_target_replay_registers_source_token_and_requires_explicit_inactive_side(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "session.h5"
            with SessionH5Writer(path, source_type="target_replay", robot_model="marvin", router_zid="router") as writer:
                from pico_body_tianji.protocol.messages import ArmTargetCommand
                writer.append_arm_target(ArmTargetCommand(ProtocolEnvelope(1, "saved", "router", 1, 10), 7, "saved", "right", "Base_R", [1, 2, 3], [0, 0, 0, 1], [1, 0, 0]), received_time_ns=100)
            session = _Session(); node = TargetReplaySource(path, session=session, publisher_instance_id="source", router_zid="router", active_sides=("right",), inactive_sides=("left",), active_hand_sides=(), inactive_hand_sides=("left", "right"))
            self.assertIn("tj/live/source/target_replay/source", session.tokens)
            intent = node.request_start()
            node.on_session_state(SessionState(1, 1, 1, "teleop", "accepted", "coordinator", intent, "coordinator", "router"))
            node.tick(now_ns=0)
            self.assertGreaterEqual(len(session.publishers[topics.SOURCE_STATUS].payloads), 2)
            self.assertIn(topics.arm_target("right"), session.publishers)
            node.close()
    def test_joint_replay_rejects_missing_active_hand_command_stream(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "session.h5"
            self._record(path)
            session = _Session()
            with self.assertRaisesRegex(ValueError, "active hand command stream: left"):
                JointReplayNode(
                    path,
                    session=session,
                    source_publisher_instance_id="source",
                    producer_publisher_instance_id="producer",
                    router_zid="router",
                    active_sides=("right",),
                    inactive_sides=("left",),
                    active_hand_sides=("left",),
                    inactive_hand_sides=("right",),
                )
            self.assertEqual(session.tokens, [])
            for key in (
                topics.SOURCE_STATUS,
                topics.PRODUCER_STATUS,
                topics.arm_proposal("left"),
                topics.hand_command("left"),
                topics.hand_command("right"),
            ):
                self.assertNotIn(key, session.publishers)
                self.assertFalse(
                    getattr(session.publishers.get(key), "payloads", [])
                )
            self.assertFalse(
                any(publisher.payloads for publisher in session.publishers.values())
            )

    def test_fault_stays_locked_after_return_and_shutdown_intents(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "session.h5"
            self._record(path)
            session = _Session()
            node = JointReplayNode(
                path,
                session=session,
                source_publisher_instance_id="source",
                producer_publisher_instance_id="producer",
                router_zid="router",
                active_sides=("right",),
                inactive_sides=("left",),
                active_hand_sides=("right",),
                inactive_hand_sides=("left",),
            )
            completed = []
            node.on_return_complete = lambda: completed.append(True)
            node.on_session_state(
                SessionState(1, 1, 1, "fault", "unsafe", "coordinator", None, "coordinator", "router")
            )
            self.assertEqual(node.phase, "fault")

            return_sequence = node.request_return()
            self.assertEqual(node.phase, "fault")
            self._assert_fault_heartbeats(session)
            node.tick(now_ns=2)
            self._assert_fault_heartbeats(session)
            return_intent = json.loads(
                session.publishers[topics.SESSION_INTENT].payloads[-1]
            )
            self.assertEqual(return_intent["action"], "return")
            self.assertEqual(return_intent["sequence"], return_sequence)
            node.on_session_state(
                SessionState(
                    1,
                    2,
                    2,
                    "idle",
                    "returned",
                    "coordinator",
                    return_sequence,
                    "coordinator",
                    "router",
                )
            )
            node.on_latched(
                {
                    "schema_version": 1,
                    "sequence": 3,
                    "timestamp_ns": 3,
                    "value": True,
                    "publisher_instance_id": "coordinator",
                    "router_zid": "router",
                },
                kind="at_home",
            )
            node.on_latched(
                {
                    "schema_version": 1,
                    "sequence": 4,
                    "timestamp_ns": 4,
                    "value": True,
                    "publisher_instance_id": "coordinator",
                    "router_zid": "router",
                },
                kind="return_complete",
            )
            self.assertEqual(node.phase, "fault")
            self.assertEqual(completed, [])

            shutdown_sequence = node.request_shutdown()
            self.assertEqual(node.phase, "fault")
            node.tick(now_ns=5)
            self._assert_fault_heartbeats(session)
            shutdown_intent = json.loads(
                session.publishers[topics.SESSION_INTENT].payloads[-1]
            )
            self.assertEqual(shutdown_intent["action"], "shutdown")
            self.assertEqual(shutdown_intent["sequence"], shutdown_sequence)
            node.on_session_state(
                SessionState(
                    1,
                    5,
                    5,
                    "idle",
                    "shutdown",
                    "coordinator",
                    shutdown_sequence,
                    "coordinator",
                    "router",
                )
            )
            node.on_latched(
                {
                    "schema_version": 1,
                    "sequence": 6,
                    "timestamp_ns": 6,
                    "value": True,
                    "publisher_instance_id": "coordinator",
                    "router_zid": "router",
                },
                kind="at_home",
            )
            node.on_latched(
                {
                    "schema_version": 1,
                    "sequence": 7,
                    "timestamp_ns": 7,
                    "value": True,
                    "publisher_instance_id": "coordinator",
                    "router_zid": "router",
                },
                kind="return_complete",
            )
            self.assertEqual(node.phase, "fault")
            self.assertEqual(completed, [])
            node.close()

    def _assert_fault_heartbeats(self, session):
        source_status = session.publishers[topics.SOURCE_STATUS].payloads
        producer_status = session.publishers[topics.PRODUCER_STATUS].payloads
        self.assertTrue(source_status)
        self.assertTrue(producer_status)
        for payload in (*source_status, *producer_status):
            status = json.loads(payload)
            self.assertEqual(status["phase"], "fault")
            self.assertFalse(status["ready"])
            self.assertFalse(status["healthy"])




if __name__ == "__main__": unittest.main()
