from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import h5py
import numpy as np

from tianji_teleop.protocol.messages import LatchedBool, SessionState
from tianji_teleop.sources.common.real_admission import RealCapabilityInput
from tianji_teleop.sources.common.session_client import SessionClient
from tianji_teleop.sources.mocap.h5 import load_mocap_h5
from tianji_teleop.sources.mocap.h5_replay_node import (
    DEFAULT_PARAMETERS,
    MocapH5ReplayNode,
    main as h5_main,
    validate_h5_hand_real_preflight,
)
from tianji_teleop.sources.mocap.live_node import AlignedHandFrame, MocapLiveNode
from tianji_teleop.sources.mocap.motive import MotiveFrameSource
from tianji_teleop.diagnostics.mocap_calibration_node import (
    MocapLiveNode as CalibrationNode,
)


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


def _motive_payload(*, body_id=7, quaternion=None) -> dict:
    return {
        "schema_version": 1,
        "frame_number": 1,
        "motive_timestamp": 1.0,
        "publisher_received_time_ns": 10,
        "coordinate_system": "motive_x_forward_z_up_right_handed",
        "unit": "meter",
        "publisher_dropped_frames": 0,
        "markers": [],
        "rigid_bodies": [
            {
                "id": body_id,
                "position": [0.0, 0.0, 0.0],
                "quaternion_xyzw": (
                    [0.0, 0.0, 0.0, 1.0]
                    if quaternion is None
                    else quaternion
                ),
                "mean_error": 0.0,
                "tracking_valid": True,
            }
        ],
    }


def _write_cli_h5(path: Path) -> None:
    time_ns = np.arange(2, dtype=np.int64)
    points = np.zeros((2, 21, 3), dtype=np.float32)
    quat = np.zeros((2, 4), dtype=np.float32)
    quat[:, 3] = 1.0
    with h5py.File(path, "w") as handle:
        handle.attrs["h5_version"] = "4.0"
        handle.attrs["schema_layout"] = "compact-aligned-60hz-v1"
        handle.create_dataset("time_ns", data=time_ns)
        for side in ("left", "right"):
            group = handle.create_group(f"hands/{side}")
            group.create_dataset("valid", data=np.ones(2, dtype=np.uint8))
            group.create_dataset("keypoints_world", data=points)
            group.create_dataset("wrist_position", data=points[:, 0])
            group.create_dataset("wrist_quaternion_xyzw", data=quat)
def _dummy_recording() -> SimpleNamespace:
    time_ns = np.array([0, 1], dtype=np.int64)
    wrist = np.zeros((2, 7), dtype=np.float64)
    wrist[:, 6] = 1.0
    hand = SimpleNamespace(
        valid=np.ones(2, dtype=bool),
        wrist=wrist,
        keypoints_world=np.zeros((2, 21, 3), dtype=np.float64),
        wuji2_joints=np.zeros((2, 20), dtype=np.float64),
    )
    return SimpleNamespace(
        path=Path("dummy.h5"),
        time_ns=time_ns,
        frame_count=2,
        duration_s=1.0e-9,
        hands={"left": hand, "right": hand},
        summary=lambda: {},
    )


class _FailingDeadman:
    def is_pressed(self) -> bool:
        raise RuntimeError("deadman disconnected")


class _FakeSessionClient:
    def __init__(self) -> None:
        self.started = 0
        self.startup_ready = False
        self.return_intent_baseline = None

    def start(self) -> None:
        self.started += 1


class _FakeSource:
    def close(self) -> None:
        pass


class _FakePublisher:
    def publish_source_status(self, **kwargs) -> None:
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
    def test_h5_yaml_cannot_self_report_real_preflight(self) -> None:
        params = {**DEFAULT_PARAMETERS, "real_mode": True}
        params["h5_real_preflight_passed"] = True
        params["hand_real_preflight_passed"] = True
        with self.assertRaisesRegex(ValueError, "typed"):
            MocapH5ReplayNode(
                _QueryableSession(),
                params,
                _dummy_recording(),
                publisher_instance_id="h5",
                router_zid="router",
                coordinator_instance_id="coord",
                expected_producer_logical_id="ik",
                expected_producer_instance_id="ik-instance",
                deadman=SimpleNamespace(is_pressed=lambda: False, close=lambda: None),
                start_keyboard=False,
                real_capability=RealCapabilityInput(0.1, 0.0, True, True),
            )

    def test_h5_limits_cannot_be_overridden(self) -> None:
        joints = np.zeros((2, 20), dtype=np.float64)
        ok, reason = validate_h5_hand_real_preflight(
            joints,
            lower_limits_rad=np.full(20, -100.0),
            upper_limits_rad=np.full(20, 100.0),
        )
        self.assertFalse(ok)
        self.assertIn("override", reason)

    def test_live_yaml_capability_is_not_an_admission_input(self) -> None:
        params = {"real_mode": True, "real_capability": {
            "speed": 0.1,
            "yaw_deg": 0.0,
            "deadman_available": True,
            "preflight_passed": True,
        }}
        with self.assertRaisesRegex(ValueError, "typed"):
            MocapLiveNode(
                _QueryableSession(),
                params,
                publisher_instance_id="live",
                router_zid="router",
                coordinator_instance_id="coord",
            )

    def test_deadman_runtime_failure_removes_real_capability(self) -> None:
        node = MocapH5ReplayNode.__new__(MocapH5ReplayNode)
        node._real_mode = True
        node._real_capability = RealCapabilityInput(0.1, 0.0, True, True)
        node._speed = 0.1
        node._yaw_deg = 0.0
        node._real_preflight_ok = True
        node._deadman = object()
        node._deadman_error = "deadman disconnected"
        ok, reason = node._real_capability_snapshot()
        self.assertFalse(ok)
        self.assertIn("deadman", reason)


class SessionClientRound5Test(unittest.TestCase):
    @staticmethod
    def _state(sequence: int) -> dict:
        return SessionState(
            1, sequence, sequence, "idle", "ready", "coordinator", None,
            "coord", "router",
        ).to_dict()

    @staticmethod
    def _latch(sequence: int, value: bool) -> dict:
        return LatchedBool(
            1, sequence, sequence, value, "coord", "router"
        ).to_dict()

    def test_stale_query_after_newer_subscriber_completes_without_invalidating(self) -> None:
        session = _QueryableSession()
        client = SessionClient(
            session,
            source="test",
            publisher_instance_id="source",
            router_zid="router",
            expected_coordinator_instance_id="coord",
        )
        client.start()
        callbacks = session.get_callbacks
        client._on_state_payload(
            json.dumps(self._state(2)).encode("utf-8")
        )
        callbacks["tianji/session/state"][0](self._state(1))
        callbacks["tianji/coordinator/at_home"][0](self._latch(3, True))
        callbacks["tianji/coordinator/return_complete"][0](self._latch(4, False))
        self.assertTrue(client.startup_ready)
        self.assertFalse(client._invalid_coordinator)
        client.close()


class MotiveEnvelopeRound5Test(unittest.TestCase):
    def test_actual_acquisition_envelope_is_accepted(self) -> None:
        frame = MotiveFrameSource().parse(_motive_payload())
        self.assertEqual(frame.frame_number, 1)
        self.assertEqual(frame.rigid_pose(7).tolist(), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

    def test_standard_fields_and_id_geometry_are_strict(self) -> None:
        source = MotiveFrameSource()
        payload = _motive_payload()
        for field in ("schema_version", "motive_timestamp", "publisher_received_time_ns",
                      "coordinate_system", "unit", "publisher_dropped_frames", "markers"):
            broken = dict(payload)
            broken.pop(field)
            with self.assertRaises(ValueError, msg=field):
                source.parse(broken)
        for bad_id in (True, "7", 7.0):
            with self.assertRaises(ValueError):
                source.parse(_motive_payload(body_id=bad_id))
        with self.assertRaises(ValueError):
            source.parse(_motive_payload(quaternion=[0.0, 0.0, 0.0, 0.0]))
        with self.assertRaises(ValueError):
            source.parse({**payload, "unexpected": 1})

    def test_duplicate_canonical_names_are_rejected(self) -> None:
        source = MotiveFrameSource()
        with self.assertRaises(ValueError):
            source.parse_names({"names": {"7": "wrist", "07": "other"}})
        with self.assertRaises(ValueError):
            source.parse_names({"names": {"7": "wrist", "8": "wrist"}})


class LiveOrientationRound5Test(unittest.TestCase):
    def test_non_identity_home_and_world_base_use_left_multiplication(self) -> None:
        node = MocapLiveNode.__new__(MocapLiveNode)
        from scipy.spatial.transform import Rotation
        home = Rotation.from_euler("x", 0.7)
        world_to_base = Rotation.from_euler("z", -0.4)
        reference = Rotation.from_euler("y", 0.3)
        current = Rotation.from_euler("x", 0.2) * Rotation.from_euler("y", -0.6)
        node._config = SimpleNamespace(
            init_pos={"left": np.zeros(3), "right": np.zeros(3)},
            init_quat={"left": [0.0, 0.0, 0.0, 1.0], "right": home.as_quat()},
            mocap_to_robot=np.eye(3),
            get_world_to_chest_rotation=lambda side: world_to_base.as_matrix(),
        )
        node._references = {
            "right": np.r_[np.zeros(3), reference.as_quat()]
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
                    "wrist_pose": np.r_[np.zeros(3), current.as_quat()].tolist(),
                    "keypoints_world_m": np.zeros((21, 3)).tolist(),
                },
            },
            "router",
        )
        result = node._build_targets(frame)
        delta_base = world_to_base * current * reference.inv() * world_to_base.inv()
        expected = delta_base * home
        np.testing.assert_allclose(
            Rotation.from_quat(result.right_pose[3:]).as_matrix(),
            expected.as_matrix(),
            atol=1e-9,
        )


class EntryAndDiagnosticsRound5Test(unittest.TestCase):
    def test_h5_validate_only_cli_loads_recording(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "take.h5"
            _write_cli_h5(path)
            self.assertEqual(h5_main([str(path), "--validate-only"]), 0)

    def test_diagnostic_authoritative_state_callback_uses_typed_state(self) -> None:
        node = CalibrationNode.__new__(CalibrationNode)
        node._phase_lock = __import__("threading").RLock()
        payload = SessionState(
            1, 1, 1, "idle", "ready", "coordinator", None, "coord", "router"
        ).to_dict()
        node._on_authoritative_state(payload)
        self.assertTrue(node._at_home)


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
        frame = source.parse(_motive_payload())
        names = source.parse_names({"names": {"7": "tianji_wrist"}})
        self.assertEqual(names, {7: "tianji_wrist"})
        self.assertEqual(frame.names, {})
        with self.assertRaises(ValueError):
            source.parse_names({"names": {"07": "tianji_wrist"}})
        with self.assertRaises(ValueError):
            source.parse({**_motive_payload(body_id="7")})


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
