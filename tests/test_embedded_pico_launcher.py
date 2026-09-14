from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]
SUPERVISOR = ROOT / "scripts" / "run_embedded_pico_vr_session.sh"
SESSION = ROOT / "scripts" / "run_session.sh"


def _write_executable(path: Path, text: str) -> None:
    path.write_text(text)
    path.chmod(0o755)


def _pid_is_live(pid: int) -> bool:
    try:
        state = Path(f"/proc/{pid}/stat").read_text().split()[2]
    except (FileNotFoundError, ProcessLookupError):
        return False
    return state != "Z"


class EmbeddedPicoLauncherTest(unittest.TestCase):
    def test_lifecycle_order_and_downstream_argument_boundary(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fake_bin = root / "bin"
            fake_bin.mkdir()
            events = root / "events.log"
            adb_args = root / "adb.args"
            downstream_args = root / "downstream.args"
            nested_pid_file = root / "nested-pid"
            config_dir = root / "pico-config"
            config_dir.mkdir()
            for name in (
                "pico_left_arm_geometry.yaml",
                "pico_right_arm_geometry.yaml",
                "pico_left_palm_tcp.yaml",
                "pico_right_palm_tcp.yaml",
                "pico_left_wrist_pivot.yaml",
                "pico_right_wrist_pivot.yaml",
            ):
                (config_dir / name).write_text("fixture\n")
            record = root / "recordings" / "pico.h5"
            record.parent.mkdir()

            _write_executable(
                fake_bin / "adb",
                """#!/usr/bin/env bash
set -euo pipefail
event_log="${TIANJI_EMBEDDED_PICO_EVENT_LOG:?}"
args=("$@")
printf '%s\\n' "$*" >> "${TIANJI_EMBEDDED_PICO_ADB_ARGS:?}"
if [[ "${args[0]:-}" == -s ]]; then
  [[ "${args[1]:-}" == pico-serial ]] || exit 23
  args=("${args[@]:2}")
fi
if [[ "${args[0]:-}" == forward && "${args[1]:-}" == --remove ]]; then
  exit 0
fi
printf '%s\\n' adb >> "$event_log"
if [[ "${args[0]:-}" == get-state ]]; then
  printf '%s\\n' device
fi
""",
            )
            _write_executable(
                fake_bin / "python3",
                """#!/usr/bin/env bash
set -euo pipefail
if [[ "$*" == *vr_manus_live.py* ]]; then
  exit 0
fi
exec /usr/bin/python3 "$@"
""",
            )
            _write_executable(fake_bin / "python", (fake_bin / "python3").read_text())
            _write_executable(
                fake_bin / "pixi",
                """#!/usr/bin/env bash
set -euo pipefail
event_log="${TIANJI_EMBEDDED_PICO_EVENT_LOG:?}"
joined=" $* "
if [[ "$joined" == *embedded_pico_preflight.py* ]]; then
  mode="unknown"
  previous=""
  for argument in "$@"; do
    if [[ "$previous" == --mode ]]; then mode="$argument"; fi
    previous="$argument"
  done
  printf '%s\\n' "$mode" >> "$event_log"
  exit 0
fi
label=""
if [[ "$joined" == *start_pico_driver.sh* ]]; then label=driver; fi
if [[ "$joined" == *start_pico_m0.sh* ]]; then label=m0; fi
if [[ "$joined" == *start_tianji_mujoco_teleop.launch.py* ]]; then label=bridge; fi
if [[ -z "$label" ]]; then exit 0; fi
if [[ "$label" == m0 ]]; then
  setsid bash -c 'trap "exit 0" TERM INT; while true; do sleep 0.05; done' &
  printf '%s\\n' "$!" > "${TIANJI_EMBEDDED_PICO_NESTED_PID:?}"
fi
printf '%s\\n' "$label" >> "$event_log"
trap 'printf "stop-%s\\n" "$label" >> "$event_log"; exit 0' TERM INT
while true; do sleep 0.05; done
""",
            )
            downstream = root / "downstream.sh"
            _write_executable(
                downstream,
                """#!/usr/bin/env bash
set -euo pipefail
printf '%s\\n' downstream >> "${TIANJI_EMBEDDED_PICO_EVENT_LOG:?}"
printf '%s\\n' "$*" > "${TIANJI_EMBEDDED_PICO_DOWNSTREAM_ARGS:?}"
sleep 100 &
disk_child=$!
trap 'sleep 0.1; if ! kill -0 "$disk_child" 2>/dev/null; then printf "premature-child-stop\\n" >> "${TIANJI_EMBEDDED_PICO_EVENT_LOG}"; fi; kill "$disk_child" 2>/dev/null || true; wait "$disk_child" 2>/dev/null || true; printf "stop-downstream\\n" >> "${TIANJI_EMBEDDED_PICO_EVENT_LOG}"; exit 0' TERM INT
while true; do sleep 0.05; done
""",
            )

            install_dir = ROOT / "vendor" / "pico_tracker" / "install"
            install_file = install_dir / "local_setup.bash"
            had_install = install_file.exists()
            if not had_install:
                install_dir.mkdir(parents=True, exist_ok=True)
                install_file.write_text("# test fixture\n")
            environment = dict(
                os.environ,
                PATH=f"{fake_bin}:{os.environ.get('PATH', '')}",
                TIANJI_EMBEDDED_PICO_EVENT_LOG=str(events),
                TIANJI_EMBEDDED_PICO_ADB_ARGS=str(adb_args),
                TIANJI_EMBEDDED_PICO_DOWNSTREAM_ARGS=str(downstream_args),
                TIANJI_EMBEDDED_PICO_NESTED_PID=str(nested_pid_file),
                TIANJI_EMBEDDED_PICO_DOWNSTREAM_SCRIPT=str(downstream),
                PICO_TRACKER_CONFIG_DIR=str(config_dir),
                TIANJI_TELEOP_RUNTIME_DIR=str(root / "runtime"),
                TIANJI_ROUTER_ENDPOINT="tcp/127.0.0.1:7447",
            )
            command = [
                "bash",
                str(SUPERVISOR),
                "--profile",
                "pico_vr_manus_sim",
                "--disable-hands",
                "--tjvr-bind",
                "127.0.0.1",
                "--tjvr-port",
                "15001",
                "--adb-serial",
                "pico-serial",
                "--record",
                str(record),
                "--pico-calibration-dir",
                str(config_dir),
                "--pico-startup-timeout-s",
                "2",
            ]
            process = subprocess.Popen(command, env=environment, text=True)
            nested_process_group: int | None = None
            try:
                deadline = time.monotonic() + 10.0
                while time.monotonic() < deadline:
                    if events.exists() and "downstream\n" in events.read_text():
                        break
                    if process.poll() is not None:
                        break
                    time.sleep(0.02)
                self.assertIsNone(process.poll())
                self.assertIn("downstream\n", events.read_text())
                nested_pid = int(nested_pid_file.read_text())
                nested_process_group = os.getpgid(nested_pid)
                events_before_duplicate = events.read_text()
                duplicate = subprocess.Popen(
                    command,
                    env=environment,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                )
                try:
                    duplicate_stdout, duplicate_stderr = duplicate.communicate(
                        timeout=5
                    )
                except subprocess.TimeoutExpired:
                    duplicate.kill()
                    duplicate_stdout, duplicate_stderr = duplicate.communicate(
                        timeout=5
                    )
                    self.fail(
                        "duplicate embedded PICO session did not reject promptly: "
                        f"stdout={duplicate_stdout!r} stderr={duplicate_stderr!r}"
                    )
                self.assertEqual(duplicate.returncode, 2, duplicate_stderr)
                self.assertIn("已有内嵌 PICO 会话正在运行", duplicate_stderr)
                self.assertEqual(events.read_text(), events_before_duplicate)
                process.send_signal(signal.SIGTERM)
                return_code = process.wait(timeout=10)
                self.assertIn(return_code, (0, 130, 143))
                deadline = time.monotonic() + 5.0
                while time.monotonic() < deadline and _pid_is_live(nested_pid):
                    time.sleep(0.02)
                self.assertFalse(
                    _pid_is_live(nested_pid),
                    "nested M0 process group survived graceful supervisor shutdown",
                )
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=5)
                if nested_process_group is not None:
                    try:
                        os.killpg(nested_process_group, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                if not had_install:
                    install_file.unlink(missing_ok=True)
                    try:
                        install_dir.rmdir()
                    except OSError:
                        pass

            events_list = [line for line in events.read_text().splitlines() if line]
            collapsed = []
            for event in events_list:
                if event not in {"adb", "bridge"} or not collapsed or collapsed[-1] != event:
                    collapsed.append(event)
            self.assertEqual(
                collapsed,
                [
                    "adb",
                    "driver",
                    "raw",
                    "m0",
                    "m0",
                    "bridge",
                    "downstream",
                    "stop-downstream",
                    "stop-bridge",
                    "stop-m0",
                    "stop-driver",
                ],
            )
            downstream_command = downstream_args.read_text()
            adb_commands = [line for line in adb_args.read_text().splitlines() if line]
            self.assertTrue(adb_commands)
            self.assertTrue(
                all(command.split()[:2] == ["-s", "pico-serial"] for command in adb_commands),
                adb_commands,
            )
            self.assertIn("--profile vr_manus_sim", downstream_command)
            self.assertIn("--tjvr-port 15001", downstream_command)
            self.assertIn(f"--record {record}", downstream_command)
            self.assertNotIn("--pico-calibration-dir", downstream_command)
            self.assertNotIn("--pico-startup-timeout-s", downstream_command)
            self.assertNotIn("--adb-serial", downstream_command)

    def test_pico2_resolve_only_does_not_dispatch_embedded_runtime(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fake_bin = root / "bin"
            fake_bin.mkdir()
            (fake_bin / "python").symlink_to("/usr/bin/python3")
            events = root / "events.log"
            environment = dict(
                os.environ,
                PATH=f"{fake_bin}:{os.environ.get('PATH', '')}",
                TIANJI_EMBEDDED_PICO_EVENT_LOG=str(events),
            )
            result = subprocess.run(
                ["bash", str(SESSION), "--profile", "pico2_hands_sim", "--resolve-only"],
                env=environment,
                capture_output=True,
                text=True,
                timeout=10,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertFalse(events.exists())

    def test_interactive_child_forwards_stdin_and_streams_output(self):
        source = (ROOT / "scripts/run_embedded_pico_vr_session.sh").read_text()
        function = source[source.index('start_child() {'):source.index('\nstop_child() {')]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            received = root / "received"
            script = '''
declare -A child_process_token child_pid child_pgid child_start_ticks
runtime_env=(); embedded_process_run_token=test; log_dir="$TELEOP_RUNTIME_DIR"
acquire_teleop_guard pico_vr_manus_sim
fail() { exit 5; }
process_group_for() { printf '%s\\n' "$1"; }
register_teleop_process_group() { [[ "$1" == "${child_pid[test]}" ]]; }
start_child test --forward-stdio bash -c 'read -r key; printf "%s\\n" "$key" > "$TELEOP_INTERACTIVE_RECEIVED"; printf "child-output\\n"'
wait "${child_pid[test]}"
'''
            environment = dict(
                os.environ,
                TIANJI_TELEOP_RUNTIME_DIR=str(root / "runtime"),
                TELEOP_INTERACTIVE_RECEIVED=str(received),
            )
            process = subprocess.Popen(
                ["bash", "-c", "source \"$1\"\n" + function + script, "_", str(ROOT / "scripts/common.sh")],
                env=environment,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            try:
                assert process.stdin is not None
                process.stdin.write("c\n")
                process.stdin.close()
                process.wait(timeout=5)
                assert process.stdout is not None
                assert process.stderr is not None
                stdout = process.stdout.read()
                stderr = process.stderr.read()
                process.stdout.close()
                process.stderr.close()
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=5)
            self.assertEqual(process.returncode, 0, stderr)
            self.assertEqual(received.read_text(), "c\n")
            self.assertIn("child-output", stdout)
            self.assertIn("child-output", (root / "runtime" / "test.log").read_text())

    def test_abrupt_supervisor_exit_does_not_leave_embedded_lock_held(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fake_bin = root / "bin"
            fake_bin.mkdir()
            events = root / "events.log"
            child_pids = root / "child-pids.log"
            config_dir = root / "pico-config"
            config_dir.mkdir()
            for name in (
                "pico_left_arm_geometry.yaml",
                "pico_right_arm_geometry.yaml",
                "pico_left_palm_tcp.yaml",
                "pico_right_palm_tcp.yaml",
                "pico_left_wrist_pivot.yaml",
                "pico_right_wrist_pivot.yaml",
            ):
                (config_dir / name).write_text("fixture\n")

            _write_executable(
                fake_bin / "adb",
                """#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" == forward && "${2:-}" == --remove ]]; then
  exit 0
fi
if [[ "${1:-}" == get-state ]]; then
  printf '%s\n' device
fi
""",
            )
            _write_executable(
                fake_bin / "python3",
                """#!/usr/bin/env bash
set -euo pipefail
if [[ "$*" == *vr_manus_live.py* ]]; then
  exit 0
fi
exec /usr/bin/python3 "$@"
""",
            )
            _write_executable(fake_bin / "python", (fake_bin / "python3").read_text())
            _write_executable(
                fake_bin / "pixi",
                """#!/usr/bin/env bash
set -euo pipefail
event_log="${TIANJI_EMBEDDED_PICO_EVENT_LOG:?}"
pid_log="${TIANJI_EMBEDDED_PICO_CHILD_PIDS:?}"
joined=" $* "
if [[ "$joined" == *embedded_pico_preflight.py* ]]; then
  printf '%s\n' preflight >> "$event_log"
  exit 0
fi
label=""
if [[ "$joined" == *start_pico_driver.sh* ]]; then label=driver; fi
if [[ "$joined" == *start_pico_m0.sh* ]]; then label=m0; fi
if [[ "$joined" == *start_tianji_mujoco_teleop.launch.py* ]]; then label=bridge; fi
if [[ -z "$label" ]]; then exit 0; fi
printf '%s %s\n' "$label" "$BASHPID" >> "$pid_log"
printf '%s\n' "$label" >> "$event_log"
trap 'exit 0' TERM INT
while true; do sleep 0.05; done
""",
            )
            downstream = root / "downstream.sh"
            _write_executable(
                downstream,
                """#!/usr/bin/env bash
set -euo pipefail
printf '%s %s\n' downstream "$BASHPID" >> "${TIANJI_EMBEDDED_PICO_CHILD_PIDS:?}"
printf '%s\n' downstream >> "${TIANJI_EMBEDDED_PICO_EVENT_LOG:?}"
trap 'exit 0' TERM INT
while true; do sleep 0.05; done
""",
            )

            install_dir = ROOT / "vendor" / "pico_tracker" / "install"
            install_file = install_dir / "local_setup.bash"
            had_install = install_file.exists()
            if not had_install:
                install_dir.mkdir(parents=True, exist_ok=True)
                install_file.write_text("# test fixture\n")

            environment = dict(
                os.environ,
                PATH=f"{fake_bin}:{os.environ.get('PATH', '')}",
                TIANJI_EMBEDDED_PICO_EVENT_LOG=str(events),
                TIANJI_EMBEDDED_PICO_CHILD_PIDS=str(child_pids),
                TIANJI_EMBEDDED_PICO_DOWNSTREAM_SCRIPT=str(downstream),
                PICO_TRACKER_CONFIG_DIR=str(config_dir),
                TIANJI_TELEOP_RUNTIME_DIR=str(root / "runtime"),
                TIANJI_ROUTER_ENDPOINT="tcp/127.0.0.1:7447",
            )
            command = [
                "bash",
                str(SUPERVISOR),
                "--profile",
                "pico_vr_manus_sim",
                "--disable-hands",
                "--pico-calibration-dir",
                str(config_dir),
                "--pico-startup-timeout-s",
                "2",
            ]
            process = subprocess.Popen(command, env=environment, text=True)
            child_process_groups: set[int] = set()
            try:
                deadline = time.monotonic() + 10.0
                while time.monotonic() < deadline:
                    if events.exists() and "downstream\n" in events.read_text():
                        break
                    if process.poll() is not None:
                        break
                    time.sleep(0.02)
                self.assertIsNone(process.poll())
                self.assertIn("downstream\n", events.read_text())
                for line in child_pids.read_text().splitlines():
                    _label, pid_text = line.split()
                    pid = int(pid_text)
                    child_process_groups.add(os.getpgid(pid))

                lock_path = root / "runtime" / "embedded-pico.lock"
                self.assertTrue(lock_path.exists())
                process.kill()
                process.wait(timeout=5)

                lock_result = subprocess.run(
                    ["flock", "-n", str(lock_path), "-c", "true"],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
                self.assertEqual(
                    lock_result.returncode,
                    0,
                    f"embedded lock remained held after supervisor SIGKILL: {lock_result.stderr}",
                )
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=5)
                for process_group in child_process_groups:
                    try:
                        os.killpg(process_group, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                if not had_install:
                    install_file.unlink(missing_ok=True)
                    try:
                        install_dir.rmdir()
                    except OSError:
                        pass
