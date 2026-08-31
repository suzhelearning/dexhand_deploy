from __future__ import annotations

import fcntl
import json
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import tempfile
import termios
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from pico_body_tianji.executors.mujoco.node import (
    MujocoExecutor,
    _configure_viewer_platform,
)
from pico_body_tianji.protocol import topics
from pico_body_tianji.protocol.messages import Frame0HandSkeleton


ROOT = Path(__file__).parents[1]
SCRIPTS = ROOT / "scripts"
CONFIG = ROOT / "src" / "pico_body_tianji" / "config"


class _FakeModel:
    def __init__(self) -> None:
        joint_names = [
            f"Joint{index}_{side}"
            for side in ("L", "R")
            for index in range(1, 8)
        ]
        self._ids = {name: index for index, name in enumerate(joint_names)}
        self.jnt_qposadr = np.arange(len(joint_names), dtype=np.int32)
        self.jnt_limited = np.ones(len(joint_names), dtype=np.uint8)
        self.jnt_range = np.tile(
            np.asarray([[-10.0, 10.0]], dtype=np.float64),
            (len(joint_names), 1),
        )
        self.geom_names = {
            f"r_wrist_axis_{index}": index for index in range(3)
        }


class _FakeData:
    def __init__(self) -> None:
        self.qpos = np.zeros(14, dtype=np.float64)
        self.geom_xmat = np.zeros((3, 9), dtype=np.float64)
        rotations = (
            np.asarray(
                [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]]
            ),
            np.asarray(
                [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]
            ),
            np.eye(3),
        )
        for index, rotation in enumerate(rotations):
            self.geom_xmat[index] = rotation.reshape(-1)
        self.geom_xpos = np.asarray(
            [
                [1.045, 2.0, 3.0],
                [1.0, 2.045, 3.0],
                [1.0, 2.0, 3.045],
            ],
            dtype=np.float64,
        )


class _FakeSession:
    def __init__(self) -> None:
        self.subscribers: list[tuple[str, object]] = []
        self.published: list[tuple[str, bytes]] = []

    def declare_subscriber(self, topic: str, callback: object) -> object:
        self.subscribers.append((topic, callback))
        return SimpleNamespace(undeclare=lambda: None)

    def declare_publisher(self, topic: str) -> object:
        owner = self

        class _Publisher:
            def put(self, payload: bytes, **_: object) -> None:
                owner.published.append((topic, bytes(payload)))

            def undeclare(self) -> None:
                return None

        return _Publisher()


class _FakeScene:
    def __init__(self, maxgeom: int = 64) -> None:
        self.maxgeom = maxgeom
        self.ngeom = 0
        self.geoms = [SimpleNamespace() for _ in range(maxgeom)]


class _FakeViewer:
    def __init__(self, maxgeom: int = 64) -> None:
        self.user_scn = _FakeScene(maxgeom)
        self.locked = False

    def lock(self) -> object:
        owner = self

        class _Lock:
            def __enter__(self) -> None:
                owner.locked = True

            def __exit__(self, *_: object) -> None:
                owner.locked = False

        return _Lock()


class _FakeMujoco:
    class mjtGeom:
        mjGEOM_SPHERE = 2
        mjGEOM_CAPSULE = 3

    def __init__(self, viewer: _FakeViewer) -> None:
        self.viewer = viewer

    def mjv_initGeom(
        self,
        geom: object,
        geom_type: int,
        size: object,
        pos: object,
        mat: object,
        rgba: object,
    ) -> None:
        if not self.viewer.locked:
            raise AssertionError("user scene geometry updated outside viewer lock")
        geom.type = geom_type
        geom.size = np.asarray(size, dtype=np.float64).copy()
        geom.pos = np.asarray(pos, dtype=np.float64).copy()
        geom.mat = np.asarray(mat, dtype=np.float64).copy()
        geom.rgba = np.asarray(rgba, dtype=np.float64).copy()

    def mjv_connector(
        self,
        geom: object,
        geom_type: int,
        width: float,
        start: object,
        end: object,
    ) -> None:
        if not self.viewer.locked:
            raise AssertionError("user scene connector updated outside viewer lock")
        geom.type = geom_type
        geom.width = float(width)
        geom.start = np.asarray(start, dtype=np.float64).copy()
        geom.end = np.asarray(end, dtype=np.float64).copy()


class ViewerPlatformTest(unittest.TestCase):
    def test_h5_viewer_uses_x11_for_deadman_key_state(self) -> None:
        glfw = SimpleNamespace(PLATFORM=1, PLATFORM_X11=2, init_hint=Mock())
        with patch.dict(os.environ, {"TIANJI_SOURCE_LOGICAL_ID": "h5_replay"}), \
             patch.dict(sys.modules, {"glfw": glfw}):
            _configure_viewer_platform()
        glfw.init_hint.assert_called_once_with(glfw.PLATFORM, glfw.PLATFORM_X11)


def _frame0_message(**updates: object) -> dict[str, object]:
    wrist = np.asarray([10.0, 20.0, 30.0], dtype=np.float64)
    points = np.repeat(wrist[None, :], 21, axis=0)
    points[:, 1] += np.arange(21, dtype=np.float64) * 0.01
    payload: dict[str, object] = Frame0HandSkeleton(
        schema_version=1,
        timestamp_ns=10,
        side="right",
        frame_id="motive_world",
        keypoints_world_m=points.tolist(),
        edges=[[index, index + 1] for index in range(20)],
        manus_wrist_pose=[*wrist.tolist(), 0.0, 0.0, 0.0, 1.0],
        robot_wrist_home_pose=[
            *wrist.tolist(),
            0.0,
            0.0,
            np.sin(np.pi / 4.0),
            np.cos(np.pi / 4.0),
        ],
        target_wrist_pose=[*wrist.tolist(), 0.0, 0.0, 0.0, 1.0],
        tcp_to_wrist_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        sequence=1,
        publisher_instance_id="h5-source-instance",
        router_zid="router-zid",
    ).to_dict()
    payload.update(updates)
    return payload


class Frame0OverlayTest(unittest.TestCase):
    def setUp(self) -> None:
        self.session = _FakeSession()
        self.data = _FakeData()
        self.executor = MujocoExecutor(
            session=self.session,
            model=_FakeModel(),
            data=self.data,
            publisher_instance_id="mujoco-instance",
            router_zid="router-zid",
            coordinator_instance_id="coordinator-instance",
            source_instance_id="h5-source-instance",
            hand_sides=(),
        )

    def tearDown(self) -> None:
        self.executor.close()

    def test_subscribes_and_strictly_rejects_wrong_diagnostic_authority(self) -> None:
        subscriptions = {topic for topic, _ in self.session.subscribers}
        self.assertIn(topics.FRAME0_HAND_SKELETON, subscriptions)
        status_before = self.executor.status
        published_before = list(self.session.published)

        invalid_messages = (
            _frame0_message(router_zid="other-router"),
            _frame0_message(publisher_instance_id="other-source"),
            _frame0_message(side="left"),
            _frame0_message(frame_id="Base_R"),
            _frame0_message(unexpected=True),
        )
        for payload in invalid_messages:
            with self.subTest(payload=payload):
                self.assertFalse(self.executor.on_frame0_hand_skeleton(payload))
                self.assertIsNone(self.executor.frame0_overlay)

        self.assertEqual(self.executor.status, status_before)
        self.assertEqual(self.session.published, published_before)

    def test_valid_message_builds_fixed_home_aligned_points_and_edges(self) -> None:
        published_before = list(self.session.published)
        self.assertTrue(self.executor.on_frame0_hand_skeleton(_frame0_message()))
        overlay = self.executor.frame0_overlay
        self.assertIsNotNone(overlay)
        assert overlay is not None
        self.assertEqual(overlay.points_mujoco.shape, (21, 3))
        self.assertEqual(overlay.edges.shape, (20, 2))
        np.testing.assert_allclose(overlay.points_mujoco[0], [1.0, 2.0, 3.0])
        np.testing.assert_allclose(overlay.points_mujoco[20], [1.2, 2.0, 3.0])
        self.assertTrue(np.isfinite(overlay.points_mujoco).all())

        fixed_points = overlay.points_mujoco.copy()
        self.data.geom_xpos += 50.0
        self.executor.tick(now_ns=20)
        np.testing.assert_allclose(
            self.executor.frame0_overlay.points_mujoco, fixed_points
        )
        self.assertEqual(self.session.published[: len(published_before)], published_before)

    def test_viewer_draws_clear_bounded_geometry_under_lock(self) -> None:
        self.assertTrue(self.executor.on_frame0_hand_skeleton(_frame0_message()))
        viewer = _FakeViewer(maxgeom=64)
        mujoco = _FakeMujoco(viewer)
        self.executor.update_frame0_viewer_overlay(viewer, mujoco_module=mujoco)

        self.assertEqual(viewer.user_scn.ngeom, 41)
        for geom in viewer.user_scn.geoms[: viewer.user_scn.ngeom]:
            for field in ("pos", "rgba"):
                if hasattr(geom, field):
                    self.assertTrue(np.isfinite(getattr(geom, field)).all())
            for field in ("start", "end"):
                if hasattr(geom, field):
                    self.assertTrue(np.isfinite(getattr(geom, field)).all())
        self.assertFalse(viewer.locked)

        bounded_viewer = _FakeViewer(maxgeom=24)
        bounded_mujoco = _FakeMujoco(bounded_viewer)
        self.executor.update_frame0_viewer_overlay(
            bounded_viewer, mujoco_module=bounded_mujoco
        )
        self.assertEqual(bounded_viewer.user_scn.ngeom, 24)


class ActualMujocoOverlaySmokeTest(unittest.TestCase):
    def test_real_urdf_populates_actual_user_scene(self) -> None:
        import contextlib
        import mujoco

        from pico_body_tianji.mujoco_urdf import portable_mujoco_urdf

        urdf = (
            ROOT
            / "src"
            / "pico_body_tianji"
            / "assets"
            / "tianji_wuji2"
            / "tianji_wuji2.urdf"
        )
        xml, assets = portable_mujoco_urdf(urdf)
        model = mujoco.MjModel.from_xml_string(xml, assets)
        data = mujoco.MjData(model)
        executor = MujocoExecutor(
            session=None,
            model=model,
            data=data,
            publisher_instance_id="mujoco-instance",
            router_zid="router-zid",
            coordinator_instance_id="coordinator-instance",
            source_instance_id="h5-source-instance",
            hand_sides=(),
        )
        try:
            self.assertTrue(
                executor.on_frame0_hand_skeleton(_frame0_message())
            )
            scene = mujoco.MjvScene(model, maxgeom=64)
            viewer = SimpleNamespace(
                user_scn=scene, lock=lambda: contextlib.nullcontext()
            )
            executor.update_frame0_viewer_overlay(
                viewer, mujoco_module=mujoco
            )
            self.assertEqual(scene.ngeom, 41)
        finally:
            executor.close()


class ManagedSourceTerminalTest(unittest.TestCase):
    def test_interactive_managed_source_receives_byte_and_mirrors_log(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bin_dir = root / "bin"
            runtime_dir = root / "runtime"
            bin_dir.mkdir()
            source_ready = root / "source-ready"
            source_byte = root / "source-byte"
            source_reader = root / "source_reader.py"
            source_reader.write_text(
                "import os, pathlib, sys, time, tty\n"
                "tty.setraw(sys.stdin.fileno())\n"
                "pathlib.Path(os.environ['SOURCE_READY']).touch()\n"
                "value = os.read(sys.stdin.fileno(), 1)\n"
                "pathlib.Path(os.environ['SOURCE_BYTE']).write_bytes(value)\n"
                "print('source-byte=' + value.decode('ascii'), flush=True)\n"
                "time.sleep(0.5)\n",
                encoding="utf-8",
            )
            non_source_reader = root / "non_source_reader.py"
            non_source_reader.write_text(
                "import os, sys, time\n"
                "value = os.read(sys.stdin.fileno(), 1)\n"
                "if value: raise SystemExit('non-source stole terminal input')\n"
                "time.sleep(4.0)\n",
                encoding="utf-8",
            )
            python = bin_dir / "python"
            python.write_text(
                "#!/bin/sh\n"
                "case \"${1:-}\" in\n"
                "  -) printf '%s\\n' router-zid ;;\n"
                "  -c) exit 0 ;;\n"
                "  *) exit 1 ;;\n"
                "esac\n",
                encoding="utf-8",
            )
            python.chmod(0o755)
            pixi = bin_dir / "pixi"
            pixi.write_text(
                "#!/bin/sh\n"
                "[ \"${1:-}\" = run ] && [ \"${2:-}\" = python ] || exit 2\n"
                "shift 2\n"
                "exec \"$REAL_PYTHON\" \"$@\"\n",
                encoding="utf-8",
            )
            pixi.chmod(0o755)
            setsid = bin_dir / "setsid"
            setsid.write_text(
                "#!/bin/sh\n"
                "case \" $* \" in\n"
                "  *'/run_source.sh '*) exec /usr/bin/setsid \"$REAL_PYTHON\" \"$SOURCE_READER\" ;;\n"
                "  *) exec /usr/bin/setsid \"$REAL_PYTHON\" \"$NON_SOURCE_READER\" ;;\n"
                "esac\n",
                encoding="utf-8",
            )
            setsid.chmod(0o755)
            sleep = bin_dir / "sleep"
            sleep.write_text(
                "#!/bin/sh\nexec /bin/sleep 0.01\n", encoding="utf-8"
            )
            sleep.chmod(0o755)
            h5_path = root / "take.h5"
            h5_path.touch()
            env = os.environ.copy()
            env.update(
                {
                    "PATH": f"{bin_dir}:{env['PATH']}",
                    "REAL_PYTHON": sys.executable,
                    "SOURCE_READER": str(source_reader),
                    "NON_SOURCE_READER": str(non_source_reader),
                    "SOURCE_READY": str(source_ready),
                    "SOURCE_BYTE": str(source_byte),
                    "PICO_TIANJI_NODE_LIST_OVERRIDE": "",
                    "PICO_TIANJI_RUNTIME_DIR": str(runtime_dir),
                    "TIANJI_VALIDATION_HAND_MODE": "retarget",
                    "TIANJI_RUN_ID": "pty-run",
                }
            )
            master_fd, slave_fd = pty.openpty()

            def establish_controlling_tty() -> None:
                os.setsid()
                fcntl.ioctl(0, termios.TIOCSCTTY, 0)

            process = subprocess.Popen(
                [
                    str(SCRIPTS / "run_session.sh"),
                    "--profile",
                    "h5_sim",
                    "--h5",
                    str(h5_path),
                    "--headless",
                ],
                cwd=ROOT,
                stdin=slave_fd,
                stdout=slave_fd,
                stderr=slave_fd,
                env=env,
                preexec_fn=establish_controlling_tty,
                close_fds=True,
            )
            os.close(slave_fd)
            transcript = bytearray()
            try:
                deadline = time.monotonic() + 5.0
                while time.monotonic() < deadline and not source_ready.exists():
                    readable, _, _ = select.select([master_fd], [], [], 0.05)
                    if readable:
                        try:
                            transcript.extend(os.read(master_fd, 65536))
                        except OSError:
                            break
                    if process.poll() is not None:
                        break
                self.assertTrue(
                    source_ready.exists(),
                    transcript.decode("utf-8", errors="replace"),
                )
                os.write(master_fd, b"s")
                process.wait(timeout=8.0)
                while True:
                    readable, _, _ = select.select([master_fd], [], [], 0.05)
                    if not readable:
                        break
                    try:
                        transcript.extend(os.read(master_fd, 65536))
                    except OSError:
                        break
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2.0)
                os.close(master_fd)

            rendered = transcript.decode("utf-8", errors="replace")
            source_log = runtime_dir / "pty-run-source.log"
            self.assertEqual(source_byte.read_bytes(), b"s")
            self.assertIn("source-byte=s", source_log.read_text(encoding="utf-8"))
            self.assertIn("source-byte=s", rendered)
            self.assertNotIn("non-source stole terminal input", rendered)


if __name__ == "__main__":
    unittest.main()
