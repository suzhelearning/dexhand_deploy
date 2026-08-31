from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import h5py
import numpy as np
from pico_body_tianji.protocol import topics
from pico_body_tianji.protocol.messages import LatchedBool, SessionState
from pico_body_tianji.sources.mocap.h5 import load_mocap_h5
from pico_body_tianji.sources.mocap.h5_replay_node import (
    DEFAULT_PARAMETERS,
    MocapH5ReplayNode,
)


class _Publisher:
    def __init__(self, key):
        self.key = key
        self.payloads = []
    def put(self, payload, **kwargs):
        self.payloads.append(bytes(payload))
    def undeclare(self):
        pass


class _Session:
    def __init__(self):
        self.publishers = {}
        self.subscribers = []
        self.queryables = []
        self.get_callbacks = {}
    def declare_publisher(self, key, **kwargs):
        pub = _Publisher(key)
        self.publishers[key] = pub
        return pub
    def declare_subscriber(self, key, callback):
        self.subscribers.append((key, callback))
        return SimpleNamespace(undeclare=lambda: None)
    def declare_queryable(self, key, callback):
        self.queryables.append((key, callback))
        return SimpleNamespace(undeclare=lambda: None)
    def get(self, key, callback, **kwargs):
        self.get_callbacks[key] = callback
    def close(self):
        pass


class _Deadman:
    pressed = False
    def is_pressed(self):
        return self.pressed
    def close(self):
        pass


def _write_h5(path: Path) -> None:
    time_ns = np.arange(3, dtype=np.int64) * 10_000_000
    points = np.zeros((3, 21, 3), dtype=np.float32)
    points[:, :, 0] = (
        np.arange(3, dtype=np.float32)[:, None]
        + np.arange(21, dtype=np.float32)[None, :]
    )
    wrist = np.zeros((3, 3), dtype=np.float32)
    wrist[:, 0] = np.arange(3, dtype=np.float32)
    quat = np.zeros((3, 4), dtype=np.float32)
    quat[:, 3] = 1.0
    with h5py.File(path, "w") as f:
        f.attrs["h5_version"] = "4.0"
        f.attrs["schema_layout"] = "compact-aligned-60hz-v1"
        f.create_dataset("time_ns", data=time_ns)
        for side in ("left", "right"):
            group = f.create_group(f"hands/{side}")
            group.create_dataset("valid", data=np.ones(3, dtype=np.uint8))
            group.create_dataset("keypoints_world", data=points)
            group.create_dataset("wrist_position", data=wrist)
            group.create_dataset("wrist_quaternion_xyzw", data=quat)


class H5CanonicalLifecycleTest(unittest.TestCase):
    def test_replay_skeleton_tracks_the_current_wrist_pose(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "take.h5"
            _write_h5(path)
            recording = load_mocap_h5(path)
            session = _Session()
            node = MocapH5ReplayNode(
                session,
                DEFAULT_PARAMETERS,
                recording,
                publisher_instance_id="h5-instance",
                router_zid="router-zid",
                coordinator_instance_id="coordinator-instance",
                expected_producer_logical_id="ik",
                expected_producer_instance_id="ik-instance",
                deadman=_Deadman(),
                start_keyboard=False,
            )
            try:
                node._cached_targets = SimpleNamespace(
                    right_pose=np.asarray(
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
                    ),
                    right_default_elbow_direction=np.asarray([1.0, 0.0, 0.0]),
                )
                node._current_source_frame = 0
                node._current_source_elapsed_s = 0.005
                node._right_wrist_home_pose = np.asarray(
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
                )
                node._publish_cached_targets()
                payload = json.loads(
                    session.publishers[topics.FRAME0_HAND_SKELETON].payloads[-1]
                )
                np.testing.assert_allclose(
                    payload["keypoints_world_m"][0], [0.5, 0.0, 0.0]
                )
                np.testing.assert_allclose(
                    payload["keypoints_world_m"][1], [1.5, 0.0, 0.0]
                )
                np.testing.assert_allclose(
                    payload["manus_wrist_pose"],
                    [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                )
            finally:
                node.close()

    def test_s_freezes_reference_and_waits_for_authoritative_teleop(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "take.h5"
            _write_h5(path)
            recording = load_mocap_h5(path)
            session = _Session()
            node = MocapH5ReplayNode(
                session,
                DEFAULT_PARAMETERS,
                recording,
                publisher_instance_id="h5-instance",
                router_zid="router-zid",
                coordinator_instance_id="coordinator-instance",
                expected_producer_logical_id="ik",
                expected_producer_instance_id="ik-instance",
                deadman=_Deadman(),
                start_keyboard=False,
            )
            try:
                self.assertIsNone(node._solved_pose)
                self.assertFalse(node._target_is_stable(0.0))
                node._session_client._on_state_payload(
                    json.dumps(SessionState(
                        1, 1, 10, "idle", "ready", "coordinator", None,
                        "coordinator-instance", "router-zid",
                    ).to_dict()).encode(),
                    query_channel="state",
                )
                node._session_client._on_latched_payload(
                    json.dumps(LatchedBool(
                        1, 1, 10, True, "coordinator-instance", "router-zid"
                    ).to_dict()).encode(),
                    is_home=True,
                    query_channel="at_home",
                )
                node._session_client._on_latched_payload(
                    json.dumps(LatchedBool(
                        1, 1, 10, False, "coordinator-instance", "router-zid"
                    ).to_dict()).encode(),
                    is_home=False,
                    query_channel="return_complete",
                )
                node._at_home = True
                node._session_client._on_state_payload(json.dumps(SessionState(
                    1, 1, 10, "idle", "ready", "coordinator", None,
                    "coordinator-instance", "router-zid",
                ).to_dict()).encode())
                node._on_motive_frame({
                    "schema_version": 1,
                    "frame_number": 1,
                    "motive_timestamp": 1.0,
                    "publisher_received_time_ns": 1,
                    "coordinate_system": "motive_x_forward_z_up_right_handed",
                    "unit": "meter",
                    "publisher_dropped_frames": 0,
                    "markers": [],
                    "rigid_bodies": [{
                        "id": 7,
                        "tracking_valid": True,
                        "position": [0.0, 0.0, 0.0],
                        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                        "mean_error": 0.0,
                    }],
                })
                node._on_rigid_body_names({"names": {"7": "tianji_wrist"}})
                node._on_key("s")
                self.assertEqual(node._phase, "start_pending")
                self.assertNotIn(topics.arm_target("right"), session.publishers)
                intent = json.loads(session.publishers[topics.SESSION_INTENT].payloads[-1])
                node._session_client._on_state_payload(json.dumps(SessionState(
                    1, 2, 11, "teleop", "accepted", "coordinator",
                    intent["sequence"], "coordinator-instance", "router-zid",
                ).to_dict()).encode())
                node._tick(now=0.0)
                self.assertEqual(node._phase, "approaching")
            finally:
                node.close()


if __name__ == "__main__":
    unittest.main()
