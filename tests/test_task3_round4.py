from __future__ import annotations

import json
import unittest
from types import SimpleNamespace

import numpy as np

from pico_body_tianji.protocol.messages import LatchedBool, SessionState
from pico_body_tianji.sources.common.real_admission import RealCapabilityInput
from pico_body_tianji.sources.common.session_client import SessionClient
from pico_body_tianji.sources.mocap.h5_replay_node import validate_h5_hand_real_preflight
from pico_body_tianji.sources.mocap.live_node import AlignedHandFrame, MocapLiveNode
from pico_body_tianji.sources.mocap.motive import MotiveFrameSource


class _Publisher:
    def __init__(self, key: str) -> None:
        self.key = key
        self.payloads: list[bytes] = []

    def put(self, payload, **kwargs) -> None:
        self.payloads.append(bytes(payload))

    def undeclare(self) -> None:
        pass


class _QueryableSession:
    def __init__(self) -> None:
        self.publishers = {}
        self.subscribers = []
        self.get_callbacks = {}

    def declare_publisher(self, key: str, **kwargs):
        publisher = _Publisher(key)
        self.publishers[key] = publisher
        return publisher

    def declare_subscriber(self, key: str, callback):
        self.subscribers.append((key, callback))
        return SimpleNamespace(undeclare=lambda: None)

    def get(self, key: str, callback, **kwargs) -> None:
        self.get_callbacks.setdefault(key, []).append(callback)

    def close(self) -> None:
        pass


class RealAdmissionTest(unittest.TestCase):
    def test_capability_input_rejects_string_booleans(self) -> None:
        with self.assertRaises(ValueError):
            RealCapabilityInput.from_mapping(
                {
                    "speed": 0.1,
                    "yaw_deg": 0.0,
                    "deadman_available": "false",
                    "preflight_passed": True,
                }
            )
        capability = RealCapabilityInput.from_mapping(
            {
                "speed": 0.1,
                "yaw_deg": 0.0,
                "deadman_available": True,
                "preflight_passed": True,
            }
        )
        self.assertTrue(capability.admitted)

    def test_h5_direct_frames_are_scanned_without_forward_fill(self) -> None:
        good = np.zeros((3, 20), dtype=np.float64)
        ok, reason = validate_h5_hand_real_preflight(good)
        self.assertTrue(ok, reason)
        bad = good.copy()
        bad[1, 4] = np.nan
        ok, reason = validate_h5_hand_real_preflight(bad)
        self.assertFalse(ok)
        self.assertIn("finite", reason)


class SessionClientRound4Test(unittest.TestCase):
    @staticmethod
    def _state(sequence: int, intent: int | None, state: str = "idle") -> dict:
        return SessionState(
            schema_version=1,
            sequence=sequence,
            timestamp_ns=sequence,
            state=state,
            reason="test",
            source="coordinator",
            intent_sequence=intent,
            publisher_instance_id="coord",
            router_zid="router",
        ).to_dict()

    @staticmethod
    def _latch(sequence: int, value: bool) -> dict:
        return LatchedBool(
            schema_version=1,
            sequence=sequence,
            timestamp_ns=sequence,
            value=value,
            publisher_instance_id="coord",
            router_zid="router",
        ).to_dict()

    def test_three_query_completions_are_independent_and_reconnect_resets(self) -> None:
        session = _QueryableSession()
        client = SessionClient(
            session,
            source="test",
            publisher_instance_id="source",
            router_zid="router",
            expected_coordinator_instance_id="coord",
        )
        client.start()
        self.assertFalse(client.startup_ready)
        callbacks = session.get_callbacks
        callbacks["tianji/session/state"][0](self._state(1, None))
        callbacks["tianji/coordinator/at_home"][0](self._latch(2, True))
        self.assertFalse(client.startup_ready)
        callbacks["tianji/coordinator/return_complete"][0](self._latch(3, False))
        self.assertTrue(client.startup_ready)
        # A second reply on one query is an authority conflict, not a harmless event.
        callbacks["tianji/session/state"][0](self._state(4, None))
        self.assertFalse(client.startup_ready)
        client.reconnect()
        self.assertFalse(client.startup_ready)
        self.assertFalse(client._invalid_coordinator)
        self.assertIsNone(client.coordinator_instance_id)
        client.close()


class MotiveStrictParserTest(unittest.TestCase):
    def test_names_are_top_level_and_ids_are_canonical(self) -> None:
        source = MotiveFrameSource()
        payload = {
            "frame_number": 1,
            "names": {"7": "tianji_wrist"},
            "rigid_bodies": [
                {
                    "id": 7,
                    "tracking_valid": True,
                    "position": [0.0, 0.0, 0.0],
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                }
            ],
        }
        frame = source.parse(payload)
        self.assertEqual(frame.names, {7: "tianji_wrist"})
        with self.assertRaises(ValueError):
            source.parse({**payload, "names": {"07": "tianji_wrist"}})
        with self.assertRaises(ValueError):
            source.parse({**payload, "rigid_bodies": [{**payload["rigid_bodies"][0], "id": "7"}]})


class LiveOrientationRound4Test(unittest.TestCase):
    def test_frozen_orientation_uses_current_times_reference_inverse_in_base(self) -> None:
        node = MocapLiveNode.__new__(MocapLiveNode)
        node._config = SimpleNamespace(
            init_pos={"left": np.zeros(3), "right": np.zeros(3)},
            init_quat={"left": np.array([0.0, 0.0, 0.0, 1.0]), "right": np.array([0.0, 0.0, 0.0, 1.0])},
            mocap_to_robot=np.eye(3),
            get_world_to_chest_rotation=lambda side: np.eye(3),
        )
        node._references = {
            "right": np.array([0.0, 0.0, 0.0, 0.0, np.sin(np.pi / 8), 0.0, np.cos(np.pi / 8)]),
        }
        node._mapper = SimpleNamespace(
            map_absolute_tcp_poses=lambda left, right: SimpleNamespace(right_pose=right)
        )
        frame = AlignedHandFrame(
            "stream", 1, 1, 1, True,
            {
                "left": {"valid": False, "wrist_pose": None, "keypoints_world_m": None},
                "right": {
                    "valid": True,
                    "wrist_pose": [0.0, 0.0, 0.0, np.sin(np.pi / 8), 0.0, 0.0, np.cos(np.pi / 8)],
                    "keypoints_world_m": np.zeros((21, 3)).tolist(),
                },
            },
            "router",
        )
        result = node._build_targets(frame)
        # Non-commuting X/Y rotations prove this is R_current * R_reference^-1.
        from scipy.spatial.transform import Rotation
        expected = Rotation.from_quat(frame.hands["right"]["wrist_pose"][3:]) * Rotation.from_quat(
            node._references["right"][3:]
        ).inv()
        np.testing.assert_allclose(
            Rotation.from_quat(result.right_pose[3:]).as_matrix(),
            expected.as_matrix(),
            atol=1e-9,
        )


if __name__ == "__main__":
    unittest.main()
