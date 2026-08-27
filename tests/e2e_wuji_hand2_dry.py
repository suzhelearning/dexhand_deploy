"""wuji_hand2_bridge dry-run 端到端测试（无硬件）。

契约：
- 订阅 pico_body_sim/right_hand/keypoints（63×float32 LE，腕部相对）；
- 发布 pico_body_sim/right_hand/joint_commands（20×float32，firmware 序）；
- 负载尺寸不符的键点帧被丢弃；
- 腕部相对化：整体平移键点不改变 retarget 输出（容差内）；
- status 含 phase/dry_run/rotation 字段（JSON 文本）。
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import threading
import time
import unittest
from pathlib import Path

import numpy as np
import zenoh

KEYPOINTS_KEY = "pico_body_sim/right_hand/keypoints"
COMMANDS_KEY = "pico_body_sim/right_hand/joint_commands"
STATUS_KEY = "pico_body_real/right_hand/status"
TELEOP_STATE_KEY = "pico_body/teleop_state"


def _open_hand_pose() -> np.ndarray:
    """张开右手 21×3 键点（米，腕部相对，任一参考系）。"""
    kp = np.zeros((21, 3), dtype=np.float32)
    kp[1] = [0.015, -0.012, 0.0]
    kp[2] = [0.040, -0.022, -0.012]
    kp[3] = [0.058, -0.026, -0.030]
    kp[4] = [0.068, -0.029, -0.048]
    index = [(5, [-0.022, 0.020]), (6, [-0.024, 0.055]),
             (7, [-0.025, 0.082]), (8, [-0.026, 0.103])]
    middle = [(9, [-0.005, 0.025]), (10, [-0.005, 0.062]),
              (11, [-0.005, 0.090]), (12, [-0.005, 0.112])]
    ring = [(13, [0.012, 0.024]), (14, [0.012, 0.060]),
            (15, [0.012, 0.086]), (16, [0.013, 0.107])]
    pinky = [(17, [0.028, 0.020]), (18, [0.029, 0.055]),
             (19, [0.030, 0.080]), (20, [0.031, 0.098])]
    for i, (y, z) in index + middle + ring + pinky:
        kp[i] = [0.0, y, z]
    return kp


def _curl_hand_pose() -> np.ndarray:
    """闭合右手：指尖沿腕部卷曲。"""
    kp = _open_hand_pose()
    for i in range(5, 21):
        z = kp[i, 2]
        kp[i, 2] = z * 0.35
        kp[i, 1] = kp[i, 1] * 1.4
        kp[i, 0] = -(z * 0.9)
    return kp


class WujiHand2DryRunTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        root = Path(os.environ["PICO_BODY_TIANJI_BUNDLE_ROOT"])
        bridge = root / "staging/ik/lib/pico_body_tianji/wuji_hand2_bridge"
        if not bridge.is_file():
            bridge = (
                root
                / "runtime/pico_body_tianji/lib/pico_body_tianji"
                / "wuji_hand2_bridge.bin"
            )
        if not bridge.is_file():
            raise unittest.SkipTest("wuji_hand2_bridge 未编译/未部署")

        env = os.environ.copy()
        wuji_lib = str(root / "vendor/wuji-sdk/lib")
        env["LD_LIBRARY_PATH"] = (
            wuji_lib + ":" + env["LD_LIBRARY_PATH"]
            if env.get("LD_LIBRARY_PATH")
            else wuji_lib
        )
        cls.proc = subprocess.Popen(
            [str(bridge), "--dry-run", "--rate", "50", "--side", "right"],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        cls.session = zenoh.open(zenoh.Config())
        cls.commands: list[np.ndarray] = []
        cls.statuses: list[dict] = []

        def on_commands(sample) -> None:
            data = bytes(sample.payload)
            if len(data) == 20 * 4:
                cls.commands.append(
                    np.frombuffer(data, dtype=np.float32).copy()
                )

        def on_status(sample) -> None:
            try:
                cls.statuses.append(json.loads(bytes(sample.payload)))
            except Exception:
                pass

        cls.raw_sub = cls.session.declare_subscriber(COMMANDS_KEY, on_commands)
        cls.status_sub = cls.session.declare_subscriber(STATUS_KEY, on_status)

        # 等桥发布第一条 status（phase=dry_run）。
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if cls.statuses:
                break
            time.sleep(0.1)
        if not cls.statuses:
            cls.tearDownClass()
            raise unittest.SkipTest("桥未在 15s 内就绪（检查 zenoh scouting）")
        cls.session.put(TELEOP_STATE_KEY, b"idle")
        time.sleep(0.2)

    @classmethod
    def tearDownClass(cls) -> None:
        for attr in ("raw_sub", "status_sub"):
            sub = getattr(cls, attr, None)
            if sub is not None:
                try:
                    sub.undeclare()
                except Exception:
                    pass
        cls.session.close()
        proc = getattr(cls, "proc", None)
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)

    @staticmethod
    def _set_state(session, state: str) -> None:
        for _ in range(3):
            session.put(TELEOP_STATE_KEY, state.encode("utf-8"))
            time.sleep(0.02)

    @staticmethod
    def _publish(session, keypoints: np.ndarray, count: int = 25) -> None:
        for _ in range(count):
            session.put(KEYPOINTS_KEY, np.asarray(
                keypoints, dtype="<f4").tobytes())
            time.sleep(0.02)

    def _latest(self) -> np.ndarray | None:
        return self.commands[-1] if self.commands else None

    def test_wrong_size_keypoints_are_ignored(self) -> None:
        self._set_state(self.session, "teleop")
        # 桥在获得首个有效键点后持续保持上帧命令（50 Hz）。
        # 错误尺寸的键点帧应被丢弃：命令值保持最后有效 qpos 不变。
        self._publish(self.session, _open_hand_pose(), count=80)
        held = self._latest().copy()
        self.assertIsNotNone(held)
        self.session.put(KEYPOINTS_KEY, b"\x00" * 100)
        self.session.put(KEYPOINTS_KEY, b"\x00" * 19)
        time.sleep(0.2)
        np.testing.assert_allclose(self._latest(), held, atol=2.0e-2)

    def test_keypoints_produce_finite_commands(self) -> None:
        self._set_state(self.session, "teleop")
        self._publish(self.session, _open_hand_pose())
        deadline = time.monotonic() + 3.0
        while not self.commands and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertTrue(self.commands, "桥未发布任何 retarget 命令")
        qpos = self._latest()
        self.assertIsNotNone(qpos)
        self.assertEqual(qpos.shape, (20,))
        self.assertTrue(np.isfinite(qpos).all())
        self.assertLess(np.abs(qpos).max(), 2.0)

    def test_wrist_translation_is_invariant(self) -> None:
        self._set_state(self.session, "teleop")
        open_pose = _open_hand_pose()
        translated = open_pose + np.array([0.5, -0.3, 0.2])
        self._publish(self.session, open_pose, count=60)
        q_a = self._latest().copy()
        self._publish(self.session, translated, count=60)
        q_b = self._latest()
        self.assertIsNotNone(q_a)
        self.assertIsNotNone(q_b)
        np.testing.assert_allclose(q_a, q_b, atol=5.0e-3)

    def test_curl_differs_from_open(self) -> None:
        self._set_state(self.session, "teleop")
        self._publish(self.session, _open_hand_pose(), count=60)
        q_open = self._latest().copy()
        self._publish(self.session, _curl_hand_pose(), count=60)
        q_curl = self._latest()
        self.assertIsNotNone(q_open)
        self.assertIsNotNone(q_curl)
        self.assertGreater(
            float(np.abs(q_curl - q_open).max()), 1.0e-3
        )

    def test_returning_and_timeout_ramp_to_zero(self) -> None:
        self._set_state(self.session, "teleop")
        self._publish(self.session, _curl_hand_pose(), count=80)
        active = self._latest().copy()
        self.assertGreater(float(np.abs(active).max()), 0.05)

        self._set_state(self.session, "returning")
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            if self._latest() is not None and np.abs(self._latest()).max() < 0.01:
                break
            time.sleep(0.05)
        self.assertLess(float(np.abs(self._latest()).max()), 0.01)

        # teleop 恢复后重新运动；停止键点超过 0.5s 也应自动回零。
        self._set_state(self.session, "teleop")
        self._publish(self.session, _curl_hand_pose(), count=60)
        self.assertGreater(float(np.abs(self._latest()).max()), 0.05)
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            if np.abs(self._latest()).max() < 0.01:
                break
            time.sleep(0.05)
        self.assertLess(float(np.abs(self._latest()).max()), 0.01)

    def test_status_reports_dry_run_and_rotation(self) -> None:
        self.assertTrue(self.statuses)
        status = self.statuses[-1]
        self.assertIn(
            status["phase"], {"tracking", "returning_zero", "zero_hold"}
        )
        self.assertTrue(status["dry_run"])
        self.assertEqual(status["side"], "right")
        self.assertEqual(status["rotation_deg"], [0.0, 0.0, -15.0])
        self.assertEqual(status["rate_hz"], 50)


if __name__ == "__main__":
    unittest.main()
