from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]
COMMON = ROOT / "scripts" / "common.sh"


def _pid_is_live(pid: int) -> bool:
    try:
        state = Path(f"/proc/{pid}/stat").read_text().split()[2]
    except (FileNotFoundError, ProcessLookupError):
        return False
    return state != "Z"


def _run_guard(runtime: Path, mode: str, conflicts: tuple[str, ...], *, wait: bool):
    conflict_args = " ".join(conflicts)
    command = f'''
set -eu
export TIANJI_TELEOP_RUNTIME_DIR="$2"
source "$1"
acquire_teleop_guard {mode} {conflict_args}
printf 'ready\\n'
'''
    if wait:
        command += "sleep 30\n"
    return subprocess.Popen(
        ["bash", "-c", command, "_", str(COMMON), str(runtime)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )


class TeleopGuardDomainTest(unittest.TestCase):
    def test_device_routes_reject_each_other_before_starting_children(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory)
            owner = _run_guard(runtime, "pico2_hands_sim", ("pico_vr_manus_sim",), wait=True)
            try:
                self.assertEqual(owner.stdout.readline().strip(), "ready")
                self.assertTrue((runtime / 'guards/pico2_hands_sim/owner').is_file())
                contender = subprocess.run(
                    [
                        "bash",
                        "-c",
                        'set -eu; export TIANJI_TELEOP_RUNTIME_DIR="$2"; source "$1"; acquire_teleop_guard pico_vr_manus_sim pico2_hands_sim',
                        "_",
                        str(COMMON),
                        str(runtime),
                    ],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                self.assertNotEqual(contender.returncode, 0)
                self.assertIn("已经运行", contender.stderr)
            finally:
                os.killpg(owner.pid, signal.SIGKILL)
                owner.wait(timeout=5)
                owner.stdout.close()
                owner.stderr.close()

    def test_legacy_pico2_profile_shares_the_device_route_domain(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory)
            owner = _run_guard(runtime, "pico2_hands_sim", ("hand_tracking_sim",), wait=True)
            try:
                self.assertEqual(owner.stdout.readline().strip(), "ready")
                contender = subprocess.run(
                    [
                        "bash",
                        "-c",
                        'set -eu; export TIANJI_TELEOP_RUNTIME_DIR="$2"; source "$1"; acquire_teleop_guard hand_tracking_sim pico2_hands_sim',
                        "_",
                        str(COMMON),
                        str(runtime),
                    ],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                self.assertNotEqual(contender.returncode, 0)
                self.assertIn("已经运行", contender.stderr)
            finally:
                os.killpg(owner.pid, signal.SIGKILL)
                owner.wait(timeout=5)
                owner.stdout.close()
                owner.stderr.close()

    def test_rejected_guard_cleanup_cannot_touch_conflicting_owner(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory)
            owner = subprocess.Popen(
                [
                    "bash",
                    "-c",
                    '''
set -eu
export TIANJI_TELEOP_RUNTIME_DIR="$2"
source "$1"
acquire_teleop_guard pico2_hands_sim pico_vr_manus_sim
setsid sleep 30 &
child="$!"
register_teleop_process_group "$child" conflict-child 1
printf '%s\n' "$child" > "$2/child.pid"
printf 'ready\n'
sleep 30
''',
                    "_",
                    str(COMMON),
                    str(runtime),
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                start_new_session=True,
            )
            child_pid = None
            try:
                self.assertEqual(owner.stdout.readline().strip(), "ready")
                child_pid = int((runtime / "child.pid").read_text())
                rejected = subprocess.run(
                    [
                        "bash",
                        "-c",
                        '''
set -eu
export TIANJI_TELEOP_RUNTIME_DIR="$2"
source "$1"
trap teleop_cleanup_and_release EXIT
if acquire_teleop_guard vr_manus_sim pico2_hands_sim; then
  exit 99
fi
''',
                        "_",
                        str(COMMON),
                        str(runtime),
                    ],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                self.assertNotEqual(rejected.returncode, 99)
                self.assertTrue(_pid_is_live(child_pid))
            finally:
                os.killpg(owner.pid, signal.SIGKILL)
                owner.wait(timeout=5)
                if child_pid is not None and Path(f"/proc/{child_pid}").exists():
                    try:
                        os.killpg(child_pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                owner.stdout.close()
                owner.stderr.close()

    def test_next_device_route_recovers_children_after_supervisor_sigkill(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory)
            supervisor = subprocess.Popen(
                [
                    "bash",
                    "-c",
                    '''
set -eu
export TIANJI_TELEOP_RUNTIME_DIR="$2"
source "$1"
acquire_teleop_guard pico2_hands_sim pico_vr_manus_sim
setsid sleep 30 &
child="$!"
register_teleop_process_group "$child" orphan-test 1
printf '%s\\n' "$child" > "$2/child.pid"
printf 'ready\\n'
sleep 30
''',
                    "_",
                    str(COMMON),
                    str(runtime),
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                start_new_session=True,
            )
            child_pid = None
            try:
                self.assertEqual(supervisor.stdout.readline().strip(), "ready")
                child_pid = int((runtime / "child.pid").read_text())
                supervisor.kill()
                supervisor.wait(timeout=5)
                self.assertTrue(Path(f"/proc/{child_pid}").exists())

                recovered = subprocess.run(
                    [
                        "bash",
                        "-c",
                        'set -eu; export TIANJI_TELEOP_RUNTIME_DIR="$2"; source "$1"; acquire_teleop_guard pico_vr_manus_sim pico2_hands_sim; printf recovered',
                        "_",
                        str(COMMON),
                        str(runtime),
                    ],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                self.assertEqual(recovered.returncode, 0, recovered.stderr)
                self.assertEqual(recovered.stdout, "recovered")
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline and _pid_is_live(child_pid):
                    time.sleep(0.02)
                self.assertFalse(_pid_is_live(child_pid))
            finally:
                if supervisor.poll() is None:
                    supervisor.kill()
                    supervisor.wait(timeout=5)
                try:
                    os.killpg(supervisor.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                if child_pid is not None and Path(f"/proc/{child_pid}").exists():
                    try:
                        os.killpg(child_pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                supervisor.stdout.close()
                supervisor.stderr.close()
