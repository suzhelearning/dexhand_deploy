#!/usr/bin/env python3
"""Run the XR arm route through the managed MuJoCo simulation boundary.

This is a bounded, simulation-only smoke.  A temporary in-process-compatible
``xrobotoolkit_sdk`` module supplies moving HMD and controller data; an
explicit legacy ``--arm-input xr_tracker`` run additionally supplies tracker
data;
the managed launcher then runs the real XR observation process, target bridge,
``pico_ee_dexhand_qp`` producer, coordinator and MuJoCo executor.  Manus hand
capture can be enabled with ``--with-manus``; that mode supplies a temporary
rawviz-shaped process and runs the real official Wuji2 worker.  No real device,
robot or actuator is opened.

Examples::

    pixi run python scripts/xr_mujoco_sim_smoke.py --arm-input xr_controller --disable-hands
    pixi run python scripts/xr_mujoco_sim_smoke.py --arm-input xr_controller --with-manus
"""
from __future__ import annotations

import argparse
import collections
import fcntl
import json
import math
import os
from pathlib import Path
import pty
import socket
import subprocess
import sys
import tempfile
import termios
import threading
import time
from typing import Any

import h5py
import numpy as np
import zenoh


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src/tianji_teleop"))

_FAKE_START_PRESS_TICK = 300
_FAKE_START_RELEASE_TICK = 500
_FAKE_HOME_PRESS_TICK = 700
_FAKE_HOME_RELEASE_TICK = 900


def _free_endpoint() -> str:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return f"tcp/127.0.0.1:{probe.getsockname()[1]}"


def _fake_controller_grip(tick: int) -> float:
    """Return a deterministic release -> start-press -> release sequence."""
    if isinstance(tick, bool) or not isinstance(tick, int) or tick < 0:
        raise ValueError("fake controller tick must be a non-negative integer")
    return 1.0 if _FAKE_START_PRESS_TICK <= tick < _FAKE_START_RELEASE_TICK else 0.0


def _fake_home_grip(tick: int) -> float:
    """Return a later deterministic home-press window for clean shutdown."""
    if isinstance(tick, bool) or not isinstance(tick, int) or tick < 0:
        raise ValueError("fake controller tick must be a non-negative integer")
    return 1.0 if _FAKE_HOME_PRESS_TICK <= tick < _FAKE_HOME_RELEASE_TICK else 0.0


def _fake_sdk_source(*, include_trackers: bool = True) -> str:
    """Return a small SDK-shaped module for the managed smoke only."""
    if type(include_trackers) is not bool:
        raise TypeError("include_trackers must be boolean")
    tracker_count = 4 if include_trackers else 0
    tracker_poses = (
        """    phase = _phase()
    displacement = 0.035 * math.sin(phase)
    height = 0.30 + 0.015 * math.cos(phase)
    return [
        _pose(0.42 + displacement, 0.20, height),
        _pose(0.42 + displacement, -0.20, height),
        _pose(0.18 + 0.5 * displacement, 0.31, 0.42),
        _pose(0.18 + 0.5 * displacement, -0.31, 0.42),
    ]
"""
        if include_trackers
        else "    return []\n"
    )
    tracker_serials = (
        "    return [\"190058\", \"190600\", \"190046\", \"190023\"]\n"
        if include_trackers
        else "    return []\n"
    )
    return f'''
import math

_tick = 0


def init():
    global _tick
    _tick = 0
    return True


def close():
    return None


def _phase():
    return _tick * 0.08


def _pose(x, y, z):
    return [float(x), float(y), float(z), 0.0, 0.0, 0.0, 1.0]


def get_motion_timestamp_ns():
    global _tick
    _tick += 1
    return _tick * 10_000_000


def get_headset_pose():
    return _pose(0.0, 0.0, 1.6)


def get_left_controller_pose():
    phase = _phase()
    return _pose(0.42 + 0.035 * math.sin(phase), 0.30, 0.28 + 0.015 * math.cos(phase))


def get_right_controller_pose():
    phase = _phase()
    return _pose(0.42 + 0.035 * math.sin(phase), -0.30, 0.28 + 0.015 * math.cos(phase))


def num_motion_data_available():
    return {tracker_count}


def get_motion_tracker_serial_numbers():
{tracker_serials}


def get_motion_tracker_pose():
{tracker_poses}


def get_left_trigger():
    return 0.0


def get_right_trigger():
    return 0.0


def get_left_grip():
    return 1.0 if {_FAKE_HOME_PRESS_TICK} <= _tick < {_FAKE_HOME_RELEASE_TICK} else 0.0


def get_right_grip():
    return 1.0 if {_FAKE_START_PRESS_TICK} <= _tick < {_FAKE_START_RELEASE_TICK} else 0.0


def get_left_axis():
    return [0.0, 0.0]


def get_right_axis():
    return [0.0, 0.0]
'''


def _write_fake_sdk(directory: Path, *, include_trackers: bool = True) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    module = directory / "xrobotoolkit_sdk.py"
    module.write_text(
        _fake_sdk_source(include_trackers=include_trackers), encoding="utf-8"
    )
    return module


def _fake_manus_rawviz_source() -> str:
    """Return a deterministic rawviz-compatible dual-glove stream.

    The stream intentionally uses the compact numeric node metadata consumed
    by the reference Manus parser. Its positions are generated in the raw VUH
    convention, so the production adapter's single Y reflection remains part
    of this smoke rather than being bypassed by pre-built observations.
    """
    return r'''#!/usr/bin/env python3
import math
import sys
import time


# Wrist, thumb mcp/pip/dip/tip, then the four non-thumb joints for each finger.
SEMANTICS = (
    (13, 0),
    (5, 1), (5, 2), (5, 4), (5, 5),
    (6, 2), (6, 3), (6, 4), (6, 5),
    (7, 2), (7, 3), (7, 4), (7, 5),
    (8, 2), (8, 3), (8, 4), (8, 5),
    (9, 2), (9, 3), (9, 4), (9, 5),
)


def canonical_points(sequence):
    points = [[0.0, 0.0, 0.0]]
    for finger in range(5):
        curl = 0.004 + 0.008 * (0.5 + 0.5 * math.sin(sequence * 0.11 + finger * 0.37))
        x = (finger - 2) * 0.026
        for joint in range(4):
            points.append([
                x + 0.001 * math.sin(sequence * 0.07 + joint) * joint,
                0.045 + 0.024 * joint - curl * max(0, joint - 1),
                0.002 * joint + 0.35 * curl * max(0, joint - 1),
            ])
    return points


def emit_metadata():
    for side in ("right", "left"):
        glove = side + "-glove"
        side_number = 0 if side == "right" else 1
        print(f"HAND {glove} {side} {len(SEMANTICS)}", flush=True)
        for index, (chain_type, joint_type) in enumerate(SEMANTICS):
            print(
                f"NODE {glove} {index} {100 + index} {99 + index} "
                f"{chain_type} {side_number} {joint_type}",
                flush=True,
            )


def emit_pose(side, sequence):
    glove = side + "-glove"
    values = []
    for x, y, z in canonical_points(sequence):
        # The production Manus adapter flips Y exactly once.
        values.extend((x, -y, z, 1.0, 0.0, 0.0, 0.0))
    source_timestamp_ns = 1_000_000_000 + sequence * 10_000_000
    print(
        f"POSE {glove} {sequence} {source_timestamp_ns} 0 "
        + " ".join(str(value) for value in values),
        flush=True,
    )


def main():
    sys.stdout.reconfigure(line_buffering=True)
    emit_metadata()
    sequence = 0
    while True:
        sequence += 1
        emit_pose("right", sequence)
        emit_pose("left", sequence)
        time.sleep(0.01)


if __name__ == "__main__":
    main()
'''


def _write_fake_manus(directory: Path) -> tuple[Path, Path, str]:
    """Create only the asset shape required by Manus runtime preflight."""
    root = directory / "fake_manus"
    calibration = root / "calibration"
    library = root / "ManusSDK" / "lib"
    calibration.mkdir(parents=True, exist_ok=True)
    library.mkdir(parents=True, exist_ok=True)
    user = "smoke"
    for side in ("Left", "Right"):
        (calibration / f"{user}{side}MetaglovePro.mcal").write_text(
            "temporary simulation calibration\n", encoding="utf-8"
        )
    # The fake rawviz never dlopens this file. It exists solely so the same
    # fail-closed runtime layout validation used by the real process executes.
    (library / "libManusSDK_Integrated.so").write_bytes(b"simulation-placeholder")
    rawviz = root / "rawviz"
    rawviz.write_text(_fake_manus_rawviz_source(), encoding="utf-8")
    rawviz.chmod(0o755)
    return rawviz, library, user


def _open_router(endpoint: str, log_path: Path) -> subprocess.Popen[Any]:
    router = subprocess.Popen(
        [
            str(ROOT / "vendor/zenoh-router/zenohd"),
            "-l",
            endpoint,
            "--no-multicast-scouting",
        ],
        cwd=ROOT,
        stdout=log_path.open("wb"),
        stderr=subprocess.STDOUT,
    )
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if router.poll() is not None:
            raise RuntimeError(f"zenoh router exited with status {router.returncode}")
        try:
            config = zenoh.Config()
            config.insert_json5("mode", '"client"')
            config.insert_json5("connect/endpoints", json.dumps([endpoint]))
            config.insert_json5("scouting/multicast/enabled", "false")
            session = zenoh.open(config)
            session.close()
            return router
        except Exception:
            time.sleep(0.1)
    router.terminate()
    router.wait(timeout=5.0)
    raise TimeoutError("local zenoh router did not become available")


def _zenoh_session(endpoint: str) -> zenoh.Session:
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", json.dumps([endpoint]))
    config.insert_json5("scouting/multicast/enabled", "false")
    return zenoh.open(config)


def _component_key(value: dict[str, Any]) -> tuple[str, str, str]:
    return (
        str(value.get("component_role", "")),
        str(value.get("component_id", "")),
        str(value.get("publisher_instance_id", "")),
    )


def _tail(path: Path, limit: int = 80) -> str:
    try:
        return "\n".join(path.read_text(encoding="utf-8", errors="replace").splitlines()[-limit:])
    except OSError:
        return "<launcher log unavailable>"


def _failure_report(result: dict[str, Any], stage: str, error: BaseException) -> dict[str, Any]:
    """Record a smoke failure without allowing a stale success flag."""
    result["passed"] = False
    result["stage"] = stage
    result["error"] = f"{type(error).__name__}: {error}"
    return result


def _stop_launcher(launcher: subprocess.Popen[Any], *, timeout_s: float = 30.0) -> int | None:
    """Stop and reap the managed launcher within a bounded interval."""
    if not isinstance(launcher, subprocess.Popen):
        raise TypeError("launcher must be a subprocess.Popen")
    if not math.isfinite(float(timeout_s)) or timeout_s <= 0.0:
        raise ValueError("timeout_s must be finite and positive")
    if launcher.poll() is None:
        launcher.terminate()
        try:
            launcher.wait(timeout=float(timeout_s))
        except subprocess.TimeoutExpired:
            launcher.kill()
            launcher.wait(timeout=max(5.0, float(timeout_s)))
    return launcher.returncode


def run_smoke(*, arm_input: str, frame_count: int, hands_enabled: bool = False) -> dict[str, Any]:
    if arm_input not in {"xr_tracker", "xr_controller"}:
        raise ValueError("arm_input must be xr_tracker or xr_controller")
    if isinstance(frame_count, bool) or not isinstance(frame_count, int) or frame_count < 30:
        raise ValueError("frames must be an integer >= 30")
    if type(hands_enabled) is not bool:
        raise ValueError("hands_enabled must be boolean")

    output = Path(tempfile.mkdtemp(prefix="xr-mujoco-sim-smoke-"))
    sdk_path = output / "fake_xr_sdk"
    _write_fake_sdk(sdk_path, include_trackers=arm_input == "xr_tracker")
    fake_manus = _write_fake_manus(output) if hands_enabled else None
    endpoint = _free_endpoint()
    router = None
    session = None
    subscriber = None
    launcher = None
    master = None
    slave = None
    drain_thread = None
    recording = output / "session.h5"
    counts: collections.Counter[str] = collections.Counter()
    latest_status: dict[tuple[str, str, str], dict[str, Any]] = {}
    latest: dict[str, dict[str, Any]] = {}
    samples: dict[str, collections.deque[dict[str, Any]]] = collections.defaultdict(
        lambda: collections.deque(maxlen=1000)
    )
    # ``wait_for`` evaluates predicates while holding the snapshot lock, and
    # the status helper takes the same lock.  A re-entrant lock keeps that
    # read path atomic without self-deadlocking the bounded smoke.
    lock = threading.RLock()
    stage = "setup"
    result: dict[str, Any] = {
        "kind": "xr_mujoco_full_simulation_smoke",
        "passed": False,
        "simulation_only": True,
        "robot_commands_enabled": False,
        "hardware_acceptance_complete": False,
        "arm_input": arm_input,
        "hands_enabled": hands_enabled,
        "frames_requested": frame_count,
        "artifacts": str(output),
    }

    def set_stage(value: str) -> None:
        nonlocal stage
        stage = value
        # Keep stdout machine-readable: the CLI emits the final report as one
        # JSON document, while progress remains visible to a human on stderr.
        print(f"[xr-mujoco-smoke] {value}", file=sys.stderr, flush=True)

    def receive(sample: Any) -> None:
        try:
            value = json.loads(sample.payload.to_bytes())
        except (AttributeError, TypeError, ValueError, UnicodeError):
            return
        if not isinstance(value, dict):
            return
        key = str(sample.key_expr)
        with lock:
            counts[key] += 1
            latest[key] = value
            if value.get("component_role"):
                latest_status[_component_key(value)] = value
            if key in {
                "tianji/session/state",
                "tianji/state/arm",
                "tianji/command/arm/left",
                "tianji/command/arm/right",
                "tianji/proposal/arm/left",
                "tianji/proposal/arm/right",
                "tianji/observation/hand/left",
                "tianji/observation/hand/right",
                "tianji/target/hand/left",
                "tianji/target/hand/right",
                "tianji/command/hand/left",
                "tianji/command/hand/right",
                "tianji/state/hand/left",
                "tianji/state/hand/right",
            }:
                samples[key].append(value)

    def wait_for(predicate: Any, timeout: float, label: str) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if launcher is not None and launcher.poll() is not None:
                raise RuntimeError(
                    f"managed launcher exited {launcher.returncode} while waiting for {label}; "
                    f"log={output / 'launcher.log'}\n{_tail(output / 'launcher.log')}"
                )
            with lock:
                if predicate():
                    return
            time.sleep(0.05)
        raise TimeoutError(label)

    try:
        router = _open_router(endpoint, output / "router.log")
        session = _zenoh_session(endpoint)
        subscriber = session.declare_subscriber("tianji/**", receive)
        master, slave = pty.openpty()

        def terminal() -> None:
            os.setsid()
            fcntl.ioctl(0, termios.TIOCSCTTY, 0)

        env = dict(
            os.environ,
            TIANJI_ROUTER_ENDPOINT=endpoint,
            TIANJI_TELEOP_RUNTIME_DIR=str(output / "runtime"),
            TIANJI_XR_SDK_PYTHONPATH=str(sdk_path),
        )
        command = [
            "bash",
            "scripts/run_session.sh",
            "--profile",
            "vr_manus_xr_sim",
            "--headless",
            "--arm-input",
            arm_input,
            "--operator-input",
            "controller",
            "--arm-pose-mapper",
            "xr_incremental",
            "--ik-backend",
            "pico_ee_dexhand_qp",
            "--joint-trajectory",
            "passthrough",
            "--command-step-clipping",
            "false",
            "--joint-limit-source",
            "urdf",
            "--record",
            str(recording),
        ]
        if hands_enabled:
            assert fake_manus is not None
            rawviz, library_dir, user = fake_manus
            command[command.index("--arm-input") : command.index("--arm-input")] = [
                "--manus-rawviz",
                str(rawviz),
                "--manus-user",
                user,
                "--manus-library-dir",
                str(library_dir),
            ]
        else:
            command.insert(command.index("--arm-input"), "--disable-hands")
        launcher = subprocess.Popen(
            command,
            cwd=ROOT,
            env=env,
            stdin=slave,
            stdout=slave,
            stderr=slave,
            preexec_fn=terminal,
        )
        os.close(slave)
        slave = None

        def drain() -> None:
            with (output / "launcher.log").open("wb") as log:
                while True:
                    try:
                        data = os.read(master, 65536)
                    except OSError:
                        return
                    if not data:
                        return
                    log.write(data)
                    log.flush()

        drain_thread = threading.Thread(target=drain, daemon=True)
        drain_thread.start()

        def status(role: str, component: str) -> dict[str, Any]:
            with lock:
                matches = [
                    value
                    for (entry_role, entry_component, _), value in latest_status.items()
                    if entry_role == role and entry_component == component
                ]
            return max(matches, key=lambda value: int(value.get("sequence", -1)), default={})

        set_stage("waiting for XR observation and target readiness")
        wait_for(
            lambda: (
                # The target owns the source-side lifecycle authority.  The
                # receive-only XR reporter is diagnostic only and may be
                # intentionally absent on a managed build, so the canonical
                # observation streams are the readiness barrier here.
                status("source", "hand_tracking_target").get("ready") is True
                and counts["tianji/observation/arm_input/left"] > 10
                and counts["tianji/observation/arm_input/right"] > 10
                and (
                    not hands_enabled
                    or (
                        counts["tianji/observation/hand/left"] > 2
                        and counts["tianji/observation/hand/right"] > 2
                    )
                )
            ),
            90.0,
            "XR observation and target readiness",
        )
        set_stage("XR target ready; waiting for controller start")
        time.sleep(0.5)
        with lock:
            prestart_targets = sum(
                counts[key]
                for key in ("tianji/target/arm/left", "tianji/target/arm/right")
            )
        if prestart_targets:
            raise AssertionError(f"targets published before explicit start: {prestart_targets}")

        wait_for(
            lambda: latest.get("tianji/session/state", {}).get("state") == "teleop",
            30.0,
            "synthetic controller teleop authorization",
        )
        set_stage("teleop authorized; waiting for MuJoCo state")
        wait_for(
            lambda: counts["tianji/state/arm"] >= max(15, frame_count // 4),
            30.0,
            "MuJoCo arm state",
        )
        time.sleep(max(1.0, min(4.0, frame_count / 30.0)))

        with lock:
            arm_states = list(samples["tianji/state/arm"])
            arm_commands = list(samples["tianji/command/arm/left"]) + list(
                samples["tianji/command/arm/right"]
            )
            proposals = list(samples["tianji/proposal/arm/left"]) + list(
                samples["tianji/proposal/arm/right"]
            )
            hand_topics = {
                key: value
                for key, value in counts.items()
                if "/target/hand/" in key or "/command/hand/" in key
            }
            hand_targets = list(samples["tianji/target/hand/left"]) + list(
                samples["tianji/target/hand/right"]
            )
            hand_commands = list(samples["tianji/command/hand/left"]) + list(
                samples["tianji/command/hand/right"]
            )
            hand_states = list(samples["tianji/state/hand/left"]) + list(
                samples["tianji/state/hand/right"]
            )
        if len(arm_states) < 10 or any(
            not isinstance(row.get("position_rad"), list)
            or len(row["position_rad"]) != 14
            or not np.isfinite(np.asarray(row["position_rad"], dtype=np.float64)).all()
            for row in arm_states
        ):
            raise AssertionError("MuJoCo did not publish finite 14-joint arm state")
        values = np.asarray([row["position_rad"] for row in arm_states], dtype=np.float64)
        motion = np.ptp(values, axis=0)
        if float(np.max(motion[:7])) <= 1.0e-4 or float(np.max(motion[7:])) <= 1.0e-4:
            raise AssertionError(f"synthetic XR input did not move both simulated arms: {motion.tolist()}")
        if len(arm_commands) < 10 or len(proposals) < 10:
            raise AssertionError("missing coordinator arm commands or IK proposals")
        if hand_topics:
            if not hands_enabled:
                raise AssertionError(f"--disable-hands emitted hand control topics: {hand_topics}")
        if hands_enabled:
            if len(hand_targets) < 10 or len(hand_commands) < 10 or len(hand_states) < 10:
                raise AssertionError(
                    "XR+Manus did not publish enough hand targets, commands or states: "
                    f"targets={len(hand_targets)}, commands={len(hand_commands)}, states={len(hand_states)}"
                )
            if any(
                not isinstance(row.get("keypoints_m"), list)
                or len(row["keypoints_m"]) != 21
                or not np.isfinite(np.asarray(row["keypoints_m"], dtype=np.float64)).all()
                for row in hand_targets
            ):
                raise AssertionError("hand targets contain invalid 21-point skeletons")
            for rows, label in (
                (hand_commands, "hand commands"),
                (hand_states, "hand states"),
            ):
                if any(
                    not isinstance(row.get("position_rad"), list)
                    or len(row["position_rad"]) != 20
                    or not np.isfinite(np.asarray(row["position_rad"], dtype=np.float64)).all()
                    for row in rows
                ):
                    raise AssertionError(f"{label} contain invalid 20-joint vectors")
            hand_values = np.asarray(
                [row["position_rad"] for row in hand_commands], dtype=np.float64
            )
            hand_motion = float(np.max(np.ptp(hand_values, axis=0)))
            if hand_motion <= 1.0e-4:
                raise AssertionError(
                    f"synthetic Manus input did not move official Wuji2 joints: {hand_motion}"
                )
            for side in ("left", "right"):
                producer = status("producer_hand", f"wuji_retarget_{side}")
                executor_hand = status("executor_hand", f"wuji_{side}")
                if producer.get("ready") is not True or executor_hand.get("ready") is not True:
                    raise AssertionError(
                        f"official {side} hand components not ready: "
                        f"producer={producer}, executor={executor_hand}"
                    )
                if producer.get("diagnostics", {}).get("retarget_backend") != "official_wuji_hand2":
                    raise AssertionError(f"{side} hand did not use official Wuji2 backend: {producer}")
        producer = status("producer_arm", "arm_ik_producer")
        executor = status("executor_arm", "mujoco")
        if producer.get("ready") is not True or executor.get("ready") is not True:
            raise AssertionError(f"producer/executor not ready: producer={producer}, executor={executor}")
        diagnostics = producer.get("diagnostics", {})
        if diagnostics.get("backend") != "pico_ee_dexhand_qp":
            raise AssertionError(f"unexpected IK backend diagnostics: {diagnostics}")
        if diagnostics.get("algorithm") != "pico_ee_v131_velocity_qp":
            raise AssertionError(f"unexpected IK algorithm diagnostics: {diagnostics}")

        result.update(
            passed=True,
            frames_observed=counts["tianji/observation/arm_input/left"],
            prestart_targets=prestart_targets,
            target_frames=min(
                counts["tianji/target/arm/left"], counts["tianji/target/arm/right"]
            ),
            proposal_frames=len(proposals),
            command_frames=len(arm_commands),
            arm_state_frames=len(arm_states),
            arm_joint_motion_rad=[float(value) for value in motion],
            mujoco_execution_verified=True,
            ik_backend=diagnostics.get("backend"),
            ik_algorithm=diagnostics.get("algorithm"),
            hand_control_topics=hand_topics,
            hand_target_frames=len(hand_targets),
            hand_command_frames=len(hand_commands),
            hand_state_frames=len(hand_states),
            hand_motion_rad=(hand_motion if hands_enabled else 0.0),
            mujoco_hand_overlay_verified=(
                hands_enabled
                and executor.get("diagnostics", {}).get("hand_overlay") is True
                and int(executor.get("diagnostics", {}).get("hand_commands_applied", 0)) > 10
            ),
        )
        if hands_enabled and not result["mujoco_hand_overlay_verified"]:
            raise AssertionError(
                "MuJoCo did not report applied hand-overlay commands: "
                f"{executor.get('diagnostics', {})}"
            )

        # The fake left grip is a bounded home request after the start edge.
        # It exercises the target-side controller binding without relying on
        # terminal injection, which is not portable under non-interactive CI.
        wait_for(
            lambda: latest.get("tianji/session/state", {}).get("state") in {"returning", "idle"}
            and latest.get("tianji/session/state", {}).get("intent_sequence") is not None,
            15.0,
            "synthetic controller home request",
        )
        wait_for(
            lambda: latest.get("tianji/session/state", {}).get("state") == "idle"
            and latest.get("tianji/session/state", {}).get("intent_sequence") is not None,
            30.0,
            "managed return to idle",
        )
        result["controller_home_verified"] = True
        set_stage("controller home verified; stopping managed launcher")

    except Exception as error:
        _failure_report(result, stage, error)
    finally:
        result["stage"] = stage
        # Stop the writers before touching the Zenoh session or opening the
        # HDF5 file.  The recorder keeps an exclusive HDF5 lock, and a live
        # high-rate Zenoh session can otherwise make teardown wait forever.
        if launcher is not None and launcher.poll() is None:
            _stop_launcher(launcher)
        if subscriber is not None:
            try:
                subscriber.undeclare()
            except Exception:
                pass
        if session is not None:
            try:
                session.close()
            except Exception:
                pass
        if master is not None:
            try:
                os.close(master)
            except OSError:
                pass
        if slave is not None:
            try:
                os.close(slave)
            except OSError:
                pass
        if router is not None and router.poll() is None:
            router.terminate()
            try:
                router.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                router.kill()
                router.wait()
        if recording.is_file():
            try:
                with h5py.File(recording, "r") as handle:
                    if "raw/xr_input/frame_json" not in handle:
                        raise AssertionError("recording is missing raw/xr_input")
                    result["raw_xr_frames"] = int(len(handle["raw/xr_input/frame_json"]))
                    if result["raw_xr_frames"] < 10:
                        raise AssertionError("recording contains too few XR frames")
                    if hands_enabled:
                        if "raw/manus_callbacks/callback_sequence" not in handle:
                            raise AssertionError("recording is missing raw/manus_callbacks")
                        result["raw_manus_callbacks"] = int(
                            len(handle["raw/manus_callbacks/callback_sequence"])
                        )
                        audit_kinds = [
                            value.decode("utf-8") if isinstance(value, bytes) else str(value)
                            for value in handle["meta/dual_audit/kind"][:]
                        ]
                        result["manus_rawviz_audit_rows"] = audit_kinds.count("manus_rawviz_line")
                        result["manus_callback_audit_rows"] = audit_kinds.count("manus_callback_metadata")
                        result["recorded_hand_command_frames"] = int(
                            sum(
                                len(handle[f"joint/command/hand/{side}/sequence"])
                                for side in ("left", "right")
                            )
                        )
                        if result["raw_manus_callbacks"] < 10:
                            raise AssertionError("recording contains too few Manus callbacks")
                        if result["manus_rawviz_audit_rows"] < 10:
                            raise AssertionError("recording contains too few rawviz audit lines")
                        if result["manus_callback_audit_rows"] < 10:
                            raise AssertionError("recording contains too few Manus callback audit rows")
                        if result["recorded_hand_command_frames"] < 10:
                            raise AssertionError("recording contains too few hand commands")
            except Exception as error:
                if result.get("passed"):
                    result["passed"] = False
                    result["error"] = f"{type(error).__name__}: {error}"
                else:
                    result["recording_error"] = f"{type(error).__name__}: {error}"
        result["component_counts"] = dict(counts)
        (output / "result.json").write_text(
            json.dumps(result, indent=2, allow_nan=False), encoding="utf-8"
        )
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-input", choices=("xr_tracker", "xr_controller"), default="xr_controller")
    parser.add_argument("--frames", type=int, default=100, help="synthetic frame budget (>=30)")
    parser.add_argument(
        "--disable-hands",
        action="store_true",
        help="required explicit safety flag; this smoke never starts Manus hand capture",
    )
    parser.add_argument(
        "--with-manus",
        action="store_true",
        help="run the simulation-only fake rawviz stream and official Wuji2 hand worker",
    )
    args = parser.parse_args(argv)
    if args.disable_hands == args.with_manus:
        parser.error("choose exactly one of --disable-hands or --with-manus")
    try:
        result = run_smoke(
            arm_input=args.arm_input,
            frame_count=args.frames,
            hands_enabled=args.with_manus,
        )
    except (TypeError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps(result, sort_keys=True, allow_nan=False))
    return 0 if result.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
