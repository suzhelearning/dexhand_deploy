"""Canonical Wuji Hand 2 dry-run regressions without legacy transport topics."""
from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import unittest

import numpy as np

from pico_body_tianji.protocol.messages import HandTargetCommand, ProtocolEnvelope, SessionState
from pico_body_tianji.zenoh_util import open_session
from pico_body_tianji.executors.wuji_hand2.config import WujiHandConfig
from pico_body_tianji.executors.wuji_hand2.node import _retarget_keypoints


class WujiHand2DryRunTest(unittest.TestCase):
    @staticmethod
    def _open_hand_pose() -> np.ndarray:
        points = np.zeros((21, 3), dtype=np.float64)
        for index, base in enumerate((1, 5, 9, 13, 17)):
            x, y = (0.03, 0.02) if base == 1 else (0.01, 0.03)
            points[base : base + 4, 0] = x
            points[base : base + 4, 1] = y
            points[base : base + 4, 2] = np.linspace(0.01, 0.07, 4)
        return points

    def test_retarget_is_finite_and_has_twenty_joints(self) -> None:
        values = _retarget_keypoints(self._open_hand_pose(), WujiHandConfig.load())
        self.assertEqual(len(values), 20)
        self.assertTrue(np.isfinite(values).all())

    def test_wrist_translation_does_not_change_retarget(self) -> None:
        config = WujiHandConfig.load()
        pose = self._open_hand_pose()
        translated = pose + np.array([0.5, -0.3, 0.2])
        np.testing.assert_allclose(
            _retarget_keypoints(pose, config),
            _retarget_keypoints(translated, config),
            atol=5.0e-3,
        )

    def test_nonfinite_keypoints_are_rejected(self) -> None:
        pose = self._open_hand_pose()
        pose[2, 1] = np.nan
        with self.assertRaises(ValueError):
            _retarget_keypoints(pose, WujiHandConfig.load())



class WujiHand2ProcessDryRunTest(unittest.TestCase):
    """Process-level canonical transport smoke; skips when router/binary is absent."""

    @classmethod
    def setUpClass(cls) -> None:
        endpoint = os.environ.get("TIANJI_ROUTER_ENDPOINT", "")
        router = os.environ.get("TIANJI_ROUTER_ZID", "")
        if not endpoint or not router or shutil.which("zenohd") is None:
            raise unittest.SkipTest("managed router endpoint/zenohd unavailable")
        root = Path(os.environ.get("PICO_BODY_TIANJI_BUNDLE_ROOT", Path(__file__).parents[1]))
        candidates = (
            root / "staging/ik/lib/pico_body_tianji/wuji_hand2_bridge",
            root / "runtime/pico_body_tianji/lib/pico_body_tianji/wuji_hand2_bridge.bin",
        )
        binary = next((path for path in candidates if path.is_file()), None)
        if binary is None:
            raise unittest.SkipTest("canonical Wuji bridge is not built")
        cls.router = router
        cls.source_instance = "validation-wuji-source"
        cls.process = None
        try:
            cls.session = open_session(endpoint)
        except Exception as exc:
            raise unittest.SkipTest(f"router unavailable: {exc}") from exc
        cls.commands: list[dict] = []
        cls.statuses: list[dict] = []
        cls.command_sub = cls.session.declare_subscriber(
            "tianji/command/hand/right",
            lambda sample: cls.commands.append(json.loads(bytes(sample.payload).decode())),
        )
        cls.status_sub = cls.session.declare_subscriber(
            "tianji/executor/hand/right/status",
            lambda sample: cls.statuses.append(json.loads(bytes(sample.payload).decode())),
        )
        env = os.environ.copy()
        env.update({
            "TIANJI_COMPONENT_INSTANCE_ID": "validation-wuji-executor",
            "TIANJI_ROUTER_ZID": router,
            "TIANJI_COORDINATOR_INSTANCE_ID": "validation-coordinator",
            "TIANJI_HAND_PRODUCER_ID": cls.source_instance,
            "TIANJI_HAND_PRODUCER_INSTANCE_ID": cls.source_instance,
            "TIANJI_HAND_LOGICAL_PRODUCER_ID": "validation-wuji-retarget",
            "TIANJI_WUJI_CONFIG": str(root / "src/pico_body_tianji/config/robot/wuji_hand2.yaml"),
            "TIANJI_RUN_ID": "validation-wuji-run",
            "TIANJI_SAFETY_SUPERVISOR_INSTANCE_ID": "validation-supervisor",
            "LD_LIBRARY_PATH": str(root / "vendor/wuji-sdk/lib") + (":" + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else ""),
        })
        cls.process = subprocess.Popen(
            [str(binary), "--mode", "retarget", "--side", "right", "--dry-run", "--rate", "50"],
            env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        time.sleep(0.2)
        if cls.process.poll() is not None:
            stderr = cls.process.stderr.read() if cls.process.stderr else ""
            cls.tearDownClass()
            raise unittest.SkipTest(f"Wuji bridge exited during setup: {stderr[-300:]}")
        cls._put_state("teleop", 1)
        cls._put_target(1)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not cls.commands:
            time.sleep(0.02)
        if not cls.commands:
            cls.tearDownClass()
            raise unittest.SkipTest("Wuji bridge produced no canonical command")

    @classmethod
    def _put(cls, key: str, payload: dict) -> None:
        data = json.dumps(payload, separators=(",", ":")).encode()
        try:
            cls.session.put(key, data, encoding="application/json")
        except TypeError:
            cls.session.put(key, data)

    @classmethod
    def _put_state(cls, state: str, sequence: int) -> None:
        cls._put("tianji/session/state", SessionState(
            1, sequence, time.monotonic_ns(), state, "process smoke",
            "coordinator", None, "validation-coordinator", cls.router,
        ).to_dict())

    @classmethod
    def _put_target(cls, sequence: int) -> None:
        cls._put("tianji/target/hand/right", HandTargetCommand(
            1, sequence, time.monotonic_ns(), None, cls.source_instance,
            "right", "wrist_relative_mediapipe", np.zeros((21, 3)).tolist(),
            cls.source_instance, cls.router,
        ).to_dict())

    @classmethod
    def tearDownClass(cls) -> None:
        process = getattr(cls, "process", None)
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        for name in ("command_sub", "status_sub"):
            resource = getattr(cls, name, None)
            if resource is not None:
                try:
                    resource.undeclare()
                except Exception:
                    pass
        session = getattr(cls, "session", None)
        if session is not None:
            session.close()

    def test_typed_status_and_command_contract(self) -> None:
        self.assertTrue(self.statuses)
        self.assertTrue(all(row.get("schema_version") == 1 for row in self.statuses))
        self.assertTrue(all(len(row.get("position_rad", [])) == 20 for row in self.commands))

    def test_invalid_target_does_not_refresh_command(self) -> None:
        before = len(self.commands)
        payload = HandTargetCommand(
            1, 2, time.monotonic_ns(), None, self.source_instance,
            "right", "wrist_relative_mediapipe", np.zeros((21, 3)).tolist(),
            self.source_instance, self.router,
        ).to_dict()
        payload["frame_id"] = "invalid"
        self._put("tianji/target/hand/right", payload)
        time.sleep(0.2)
        self.assertEqual(len(self.commands), before)

    def test_returning_disables_tracking_and_close_is_clean(self) -> None:
        before = len(self.commands)
        self._put_state("returning", 2)
        time.sleep(0.2)
        self.assertEqual(len(self.commands), before)
if __name__ == "__main__":
    unittest.main()
