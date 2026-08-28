from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import h5py
import numpy as np

from pico_body_tianji.protocol import topics
from pico_body_tianji.protocol.messages import SessionState
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
        return None
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
    points[:, :, 0] = np.arange(21, dtype=np.float32)
    wrist = np.zeros((3, 3), dtype=np.float32)
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
                deadman=_Deadman(),
                start_keyboard=False,
            )
            try:
                node._at_home = True
                node._session_client._on_state_payload(json.dumps(SessionState(
                    1, 1, 10, "idle", "ready", "coordinator", None,
                    "coordinator-instance", "router-zid",
                ).to_dict()).encode())
                node._on_motive_frame({
                    "frame_number": 1,
                    "rigid_bodies": [{
                        "id": 7, "tracking_valid": True,
                        "position": [0.0, 0.0, 0.0],
                        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                    }],
                })
                node._on_rigid_body_names({"7": "tianji_wrist"})
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
