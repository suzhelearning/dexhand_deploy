import unittest
from pathlib import Path
import json
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import yaml

from tianji_teleop.executors.wuji_hand2.node import WujiHandExecutor
from tianji_teleop.protocol import topics
from tianji_teleop.protocol.messages import HandTargetCommand, SessionState


ROOT = Path(__file__).resolve().parents[1]


class _Clock:
    def __init__(self, value=1_000_000_000):
        self.value = int(value)

    def __call__(self):
        return self.value


class _FakeOfficialRetargeter:
    def __init__(self):
        self.calls = []
        self.closed = False

    def retarget(self, points, *, sequence, timestamp_ns):
        self.calls.append((np.asarray(points, dtype=np.float64).copy(), sequence, timestamp_ns))
        return {
            "right": {"valid": True, "position_rad": [0.05] * 20},
            "left": {"valid": False, "position_rad": [0.0] * 20},
        }

    def close(self):
        self.closed = True


class _Publisher:
    def __init__(self):
        self.values = []

    def put(self, payload, **_kwargs):
        self.values.append(json.loads(payload))

    def undeclare(self):
        return None


class _Session:
    def __init__(self):
        self.publishers = {}

    def declare_subscriber(self, _key, _callback):
        return SimpleNamespace(undeclare=lambda: None)

    def declare_publisher(self, key):
        publisher = self.publishers.setdefault(key, _Publisher())
        return publisher


def _target(sequence, timestamp_ns, points=None):
    points = np.zeros((21, 3), dtype=np.float64) if points is None else points
    return HandTargetCommand(
        1,
        sequence,
        timestamp_ns,
        None,
        "hand_tracking_target",
        "right",
        "wrist_relative_mediapipe",
        points.tolist(),
        "target",
        "router",
    )


class WujiHand2OfficialSimulationTest(unittest.TestCase):
    def test_xr_manus_profile_selects_official_hand2_executor_config(self):
        profile = yaml.safe_load(
            (ROOT / "src/tianji_teleop/config/sessions/vr_manus_xr_runtime.yaml").read_text()
        )
        executor = yaml.safe_load(
            (ROOT / "src/tianji_teleop/config/executors/wuji_hand2_official.yaml").read_text()
        )
        self.assertEqual(profile["hand_executor"], "wuji_hand2")
        self.assertEqual(profile["hand_executor_config"], "executors/wuji_hand2_official.yaml")
        self.assertEqual(profile["coordinator_config"], "coordinator/arm_v131_wuji2.yaml")
        self.assertEqual(
            profile["hand_overlay"],
            "mujoco",
            "XR+Manus simulation must apply hand commands to the MuJoCo hand overlay",
        )
        self.assertEqual(executor["retarget_backend"], "official_wuji_hand2")
        self.assertEqual(executor["hand_config"], "robot/wuji_hand2.yaml")

        coordinator = yaml.safe_load(
            (ROOT / "src/tianji_teleop/config/coordinator/arm_v131_wuji2.yaml").read_text()
        )
        self.assertGreater(float(coordinator["hand_return_timeout_s"]), 3.0)

    def test_official_backend_retargets_new_target_once_and_reuses_cached_result(self):
        clock = _Clock()
        backend = _FakeOfficialRetargeter()
        executor = WujiHandExecutor(
            mode="retarget",
            side="right",
            router_zid="router",
            publisher_instance_id="executor",
            authorized_producer="wuji_retarget_right",
            authorized_publisher_instance_id="target",
            producer_publisher_instance_id="producer",
            coordinator_instance_id="coord",
            retarget_backend="official_wuji_hand2",
            retargeter=backend,
            clock=clock,
        )
        self.addCleanup(executor.close)
        executor.on_session_state(
            SessionState(1, 1, clock(), "teleop", "start", "coordinator", 1, "coord", "router")
        )

        self.assertTrue(executor.on_hand_target(_target(1, clock())))
        first = executor.tick(now_ns=clock())
        self.assertIsNotNone(first)
        self.assertEqual(first.position_rad, [0.05] * 20)
        self.assertEqual(len(backend.calls), 1)

        # The target publisher can run faster than the hand executor.  A
        # repeated executor tick must not feed the same callback sequence into
        # the official stateful retargeter twice.
        repeated = executor.tick(now_ns=clock())
        self.assertIsNotNone(repeated)
        self.assertEqual(len(backend.calls), 1)

        clock.value += 20_000_000
        self.assertTrue(executor.on_hand_target(_target(2, clock())))
        executor.tick(now_ns=clock())
        self.assertEqual(len(backend.calls), 2)
        self.assertEqual([call[1] for call in backend.calls], [1, 2])

    def test_official_backend_is_closed_with_executor(self):
        backend = _FakeOfficialRetargeter()
        executor = WujiHandExecutor(
            mode="retarget",
            side="left",
            router_zid="router",
            publisher_instance_id="executor",
            authorized_producer="wuji_retarget_left",
            authorized_publisher_instance_id="target",
            coordinator_instance_id="coord",
            retarget_backend="official_wuji_hand2",
            retargeter=backend,
        )
        executor.close()
        self.assertTrue(backend.closed)

    def test_xr_output_audit_associates_exact_target_and_command(self):
        clock = _Clock()
        backend = _FakeOfficialRetargeter()
        session = _Session()
        with patch.dict("os.environ", {"TIANJI_HAND_OUTPUT_AUDIT": "1"}, clear=False):
            executor = WujiHandExecutor(
                mode="retarget",
                side="right",
                router_zid="router",
                publisher_instance_id="executor-right",
                authorized_producer="wuji_retarget_right",
                authorized_publisher_instance_id="target",
                producer_publisher_instance_id="hand-producer",
                coordinator_instance_id="coord",
                run_id="run-1",
                retarget_backend="official_wuji_hand2",
                retargeter=backend,
                session=session,
                clock=clock,
            )
            self.addCleanup(executor.close)
            executor.on_session_state(
                SessionState(1, 1, clock(), "teleop", "start", "coordinator", 1, "coord", "router")
            )
            self.assertTrue(executor.on_hand_target(_target(1, clock())))
            command = executor.tick(now_ns=clock())
            self.assertIsNotNone(command)
            rows = session.publishers[topics.HAND_OUTPUT_AUDIT].values
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["kind"], "hand_output")
            self.assertEqual(rows[0]["target"]["sequence"], 1)
            self.assertEqual(rows[0]["command"]["sequence"], command.sequence)

    def test_xr_output_audit_uses_target_snapshot_when_new_state_arrives_during_tick(self):
        clock = _Clock()
        backend = _FakeOfficialRetargeter()
        session = _Session()
        executor = WujiHandExecutor(
            mode="retarget",
            side="right",
            router_zid="router",
            publisher_instance_id="executor-right",
            authorized_producer="wuji_retarget_right",
            authorized_publisher_instance_id="target",
            producer_publisher_instance_id="hand-producer",
            coordinator_instance_id="coord",
            run_id="run-1",
            retarget_backend="official_wuji_hand2",
            retargeter=backend,
            session=session,
            clock=clock,
        )
        self.addCleanup(executor.close)
        executor._hand_output_audit_enabled = True
        executor._publishers["audit"] = session.declare_publisher(topics.HAND_OUTPUT_AUDIT)
        executor.on_session_state(
            SessionState(1, 1, clock(), "teleop", "start", "coordinator", 1, "coord", "router")
        )
        target = _target(1, clock())
        self.assertTrue(executor.on_hand_target(target))

        def retarget_and_clear(_target):
            executor._latest_target = None
            return [0.05] * 20

        with patch.object(executor, "_retarget_values", side_effect=retarget_and_clear):
            command = executor.tick(now_ns=clock())

        self.assertIsNotNone(command)
        rows = session.publishers[topics.HAND_OUTPUT_AUDIT].values
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["target"], target.to_dict())

    def test_unknown_retarget_backend_is_rejected(self):
        with self.assertRaises(ValueError):
            WujiHandExecutor(
                mode="retarget",
                side="right",
                router_zid="router",
                publisher_instance_id="executor",
                authorized_producer="producer",
                authorized_publisher_instance_id="target",
                coordinator_instance_id="coord",
                retarget_backend="unknown",
            )


if __name__ == "__main__":
    unittest.main()
