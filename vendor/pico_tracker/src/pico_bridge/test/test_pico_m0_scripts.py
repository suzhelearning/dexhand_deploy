from pathlib import Path
import os
import re
import shlex
import subprocess


PACKAGE_ROOT = Path(__file__).parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[1]
CALIBRATION_WRAPPER = PACKAGE_ROOT / "scripts" / "pico_arm_geometry_pixi.sh"
PROCESS_CLEANUP = PACKAGE_ROOT / "scripts" / "pico_process_cleanup.sh"
RUNTIME_WRAPPER = PACKAGE_ROOT / "scripts" / "start_pico_m0_pixi.sh"
USER_RUNTIME_WRAPPER = REPO_ROOT / "scripts" / "start_pico_m0.sh"
ORIENTATION_WRAPPER = REPO_ROOT / "scripts" / "calibrate_pico_palm_orientation.sh"
TREMOR_RECORDER = REPO_ROOT / "scripts" / "record_pico_tremor.sh"
TIANJI_TMUX_LAUNCHER = REPO_ROOT / "scripts" / "start_tianji_pico_teleop.sh"
TIANJI_TMUX_STOPPER = REPO_ROOT / "scripts" / "stop_tianji_pico_teleop.sh"
GITIGNORE = REPO_ROOT / ".gitignore"
CMAKE = PACKAGE_ROOT / "CMakeLists.txt"
PIXI_MANIFEST = REPO_ROOT / "pixi.toml"


def _write_executable(path: Path, source: str) -> None:
    path.write_text(source, encoding="utf-8")
    path.chmod(0o755)


def _fake_tianji_launcher_environment(tmp_path: Path) -> dict[str, str]:
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    tmux_state = tmp_path / "tmux-session"
    tmux_log = tmp_path / "tmux.log"

    _write_executable(
        fake_bin / "tmux",
        """#!/usr/bin/env bash
set -euo pipefail
printf '%q ' "$@" >> "$FAKE_TMUX_LOG"
printf '\n' >> "$FAKE_TMUX_LOG"
case "${1:-}" in
  has-session)
    [[ -f "$FAKE_TMUX_STATE" ]]
    ;;
  new-session)
    touch "$FAKE_TMUX_STATE"
    ;;
  list-windows)
    [[ -f "$FAKE_TMUX_STATE" ]]
    printf '0: driver\n1: m0\n2: bridge\n'
    ;;
  kill-session)
    rm -f "$FAKE_TMUX_STATE"
    ;;
  *)
    :
    ;;
esac
""",
    )
    _write_executable(
        fake_bin / "adb",
        """#!/usr/bin/env bash
set -euo pipefail
[[ "${1:-}" == "get-state" ]] || exit 2
printf '%s\n' "${FAKE_ADB_STATE:-device}"
""",
    )
    _write_executable(
        fake_bin / "pixi",
        """#!/usr/bin/env bash
set -euo pipefail
[[ "${1:-}" == "run" ]] || exit 2
shift
export PIXI_PROJECT_ROOT="$FAKE_REPO_ROOT"
exec "$@"
""",
    )
    _write_executable(
        fake_bin / "python3",
        """#!/usr/bin/env bash
set -euo pipefail
printf 'cleanup %q\n' "$@" >> "$FAKE_TMUX_LOG"
exit "${FAKE_CLEANUP_EXIT:-0}"
""",
    )

    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "PIXI_PROJECT_ROOT": str(REPO_ROOT),
            "FAKE_REPO_ROOT": str(REPO_ROOT),
            "FAKE_TMUX_STATE": str(tmux_state),
            "FAKE_TMUX_LOG": str(tmux_log),
        }
    )
    return environment


def test_interactive_wrapper_runs_calibrator_in_foreground_without_ros2_launch():
    source = CALIBRATION_WRAPPER.read_text(encoding="utf-8")
    assert "ros2 run pico_bridge pico_arm_geometry_calibrator" in source
    assert "ros2 launch" not in source
    assert "--start-mode \"$start_mode\"" in source


def test_geometry_wrapper_defaults_to_space_start():
    source = CALIBRATION_WRAPPER.read_text(encoding="utf-8")
    assert 'start_mode="space"' in source
    assert 'space|auto' in source


def test_interactive_wrapper_unsets_python_ros_and_conda_contamination():
    source = CALIBRATION_WRAPPER.read_text(encoding="utf-8")
    required = (
        "PYTHONPATH",
        "PYTHONHOME",
        "AMENT_PREFIX_PATH",
        "CMAKE_PREFIX_PATH",
        "COLCON_PREFIX_PATH",
        "CONDA_PREFIX",
        "CONDA_DEFAULT_ENV",
        "CONDA_PROMPT_MODIFIER",
        "CONDA_SHLVL",
        "CONDA_BUILD",
    )
    for name in required:
        assert f"-u {name}" in source
    assert "source install/local_setup.bash" in source
    assert 'sys.version_info[:2] != (3, 11)' in source
    assert '"/opt/ros/humble" in rclpy.__file__' in source


def test_interactive_wrapper_requires_explicit_side_artifacts():
    source = CALIBRATION_WRAPPER.read_text(encoding="utf-8")
    assert "--side" in source
    assert "--tcp-artifact" in source
    assert "--wrist-pivot-artifact" in source
    assert "recordings" not in source or "find recordings" not in source


def test_geometry_wrapper_bounds_tcp_publisher_process_group_cleanup():
    source = CALIBRATION_WRAPPER.read_text(encoding="utf-8")
    assert "setsid ros2 run pico_bridge pico_palm_tcp_publisher" in source
    assert "pico_stop_process_group" in source


def test_process_group_cleanup_is_reentrant_safe_and_tracks_group_liveness():
    source = PROCESS_CLEANUP.read_text(encoding="utf-8")
    wrapper = CALIBRATION_WRAPPER.read_text(encoding="utf-8")

    assert "trap - EXIT INT TERM" in wrapper
    assert 'kill -INT -- "-$leader_pid"' in source
    assert 'kill -TERM -- "-$leader_pid"' in source
    assert 'kill -KILL -- "-$leader_pid"' in source
    assert "pico_process_group_is_running" in source


def test_process_group_cleanup_returns_even_when_child_ignores_soft_signals():
    command = f'''
      set -euo pipefail
      source "{PROCESS_CLEANUP}"
      setsid bash -c 'trap "" INT TERM; while :; do sleep 30; done' &
      leader_pid=$!
      pico_stop_process_group "$leader_pid"
      echo cleanup-complete
    '''

    result = subprocess.run(
        ["bash", "-c", command],
        text=True,
        capture_output=True,
        timeout=4.0,
        check=False,
    )

    assert result.returncode == 0
    assert "cleanup-complete" in result.stdout


def test_process_group_cleanup_kills_descendant_after_leader_exits(tmp_path):
    child_file = tmp_path / "descendant.pid"
    worker = tmp_path / "process_group_worker.sh"
    worker.write_text(
        """#!/usr/bin/env bash
set -euo pipefail
child_file="$1"
trap 'exit 0' INT
bash -c 'trap "" INT TERM; while :; do sleep 30; done' >/dev/null 2>&1 &
echo "$!" > "$child_file"
while :; do sleep 30; done
""",
        encoding="utf-8",
    )
    worker.chmod(0o755)
    command = f'''
      set -euo pipefail
      source "{PROCESS_CLEANUP}"
      child_file={shlex.quote(str(child_file))}
      setsid {shlex.quote(str(worker))} "$child_file" &
      leader_pid=$!
      for _ in {{1..100}}; do
        [[ -s "$child_file" ]] && break
        sleep 0.01
      done
      descendant_pid="$(cat "$child_file")"
      pico_stop_process_group "$leader_pid"
      if /usr/bin/ps -o stat= -p "$descendant_pid" 2>/dev/null | grep -qv '^[[:space:]]*Z'; then
        echo "descendant-still-running:$descendant_pid" >&2
        exit 9
      fi
      echo cleanup-complete
    '''

    result = subprocess.run(
        ["bash", "-c", command],
        text=True,
        capture_output=True,
        timeout=5.0,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "cleanup-complete" in result.stdout


def test_process_group_cleanup_honors_extended_grace_period(tmp_path):
    marker = tmp_path / "graceful.marker"
    worker = tmp_path / "graceful_worker.py"
    worker.write_text(
        f"""import signal
import time
from pathlib import Path

marker = Path({str(marker)!r})

def handle_interrupt(_signum, _frame):
    time.sleep(1.2)
    marker.touch()
    raise SystemExit(0)

signal.signal(signal.SIGINT, handle_interrupt)
while True:
    time.sleep(30)
""",
        encoding="utf-8",
    )
    command = f'''
      set -euo pipefail
      source "{PROCESS_CLEANUP}"
      setsid python {shlex.quote(str(worker))} &
      leader_pid=$!
      sleep 0.1
      pico_stop_process_group "$leader_pid" 40 10
      [[ -f {shlex.quote(str(marker))} ]]
    '''

    result = subprocess.run(
        ["bash", "-c", command],
        text=True,
        capture_output=True,
        timeout=5.0,
        check=False,
    )

    assert result.returncode == 0, result.stderr


def test_runtime_never_selects_newest_candidate():
    source = RUNTIME_WRAPPER.read_text(encoding="utf-8")
    assert "find recordings" not in source
    assert "sort -t" not in source
    assert "--left-geometry" in source
    assert "--right-geometry" in source
    assert "--require-left-geometry" in source
    assert "--require-right-geometry" in source


def test_user_runtime_uses_only_explicit_active_artifacts_and_never_scans_recordings():
    source = USER_RUNTIME_WRAPPER.read_text(encoding="utf-8")
    assert "rglob" not in source
    assert "auto-activated" not in source
    assert "PICO_RECORDINGS_ROOT" not in source
    assert "pico_left_arm_geometry.yaml" in source
    assert "pico_right_arm_geometry.yaml" in source


def test_user_runtime_validates_each_side_and_only_requires_valid_geometry():
    source = USER_RUNTIME_WRAPPER.read_text(encoding="utf-8")

    assert "pico_calibration_artifact.py" in source
    assert 'valid_geometry_count=$((valid_geometry_count + 1))' in source
    assert "未通过严格验证；该侧不加载严格个体骨长" in source
    assert "至少需要一侧完成有效标定" in source
    assert '--require-left-geometry \\\n+  --require-right-geometry' not in source


def test_runtime_traps_and_stops_only_its_children():
    source = RUNTIME_WRAPPER.read_text(encoding="utf-8")
    assert "trap cleanup EXIT INT TERM" in source
    assert "kill 0" not in source
    assert "child_pids" in source


def test_runtime_prevents_duplicate_filter_instances_per_ros_domain():
    source = RUNTIME_WRAPPER.read_text(encoding="utf-8")

    assert "flock -n" in source
    assert "pico_m0_domain_${domain}.lock" in source
    assert "pico_palm_skeleton_filter" in source
    assert "get_node_names_and_namespaces" in source
    assert 'Path("/proc")' in source
    assert 'process_dir / "environ"' in source
    assert 'ROS_DOMAIN_ID' in source
    assert "PICO M0 is already running" in source
    assert "raise SystemExit(2)" in source


def test_runtime_defaults_to_domain_120_and_localhost_only():
    source = RUNTIME_WRAPPER.read_text(encoding="utf-8")

    assert 'domain="120"' in source
    assert 'ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"' in source
    assert 'export ROS2CLI_DISABLE_DAEMON="${ROS2CLI_DISABLE_DAEMON:-1}"' in source


def test_equivalent_domain_spellings_contend_for_the_same_lock(tmp_path):
    command = f'''
      set -euo pipefail
      source "{PROCESS_CLEANUP}"
      first="$(pico_normalize_ros_domain 120)"
      alias="$(pico_normalize_ros_domain 0120)"
      [[ "$first" == "$alias" ]]
      exec {{first_fd}}>"{tmp_path}/pico_m0_domain_${{first}}.lock"
      exec {{alias_fd}}>"{tmp_path}/pico_m0_domain_${{alias}}.lock"
      flock -n "$first_fd"
      if flock -n "$alias_fd"; then
        echo "equivalent domain alias acquired a second lock" >&2
        exit 9
      fi
    '''

    result = subprocess.run(
        ["bash", "-c", command],
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr


def test_runtime_rejects_invalid_ros_domain_before_artifact_checks():
    result = subprocess.run(
        ["bash", str(RUNTIME_WRAPPER), "--domain", "../invalid"],
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 2
    assert "--domain must be an integer between 0 and 232" in result.stderr
    assert "artifact missing" not in result.stderr


def test_readiness_helper_is_installed_and_registered_with_colcon_test():
    source = CMAKE.read_text(encoding="utf-8")

    assert "scripts/pico_m0_readiness.py" in source
    assert "ament_add_pytest_test(test_pico_m0_readiness" in source


def test_runtime_owns_and_cleans_complete_child_process_groups():
    source = RUNTIME_WRAPPER.read_text(encoding="utf-8")

    assert 'source "$script_dir/pico_process_cleanup.sh"' in source
    assert "setsid ros2 launch pico_bridge start_pico_palm_skeleton_filter.launch.py" in source
    assert 'pico_stop_process_group "$pid"' in source
    assert 'graceful_attempts=600' in source


def test_runtime_isolates_python_and_sources_only_local_overlay():
    source = RUNTIME_WRAPPER.read_text(encoding="utf-8")
    for name in (
        "PYTHONPATH",
        "PYTHONHOME",
        "AMENT_PREFIX_PATH",
        "CMAKE_PREFIX_PATH",
        "COLCON_PREFIX_PATH",
        "CONDA_PREFIX",
        "CONDA_DEFAULT_ENV",
        "CONDA_PROMPT_MODIFIER",
        "CONDA_SHLVL",
        "CONDA_BUILD",
    ):
        assert f"-u {name}" in source
    assert "source install/local_setup.bash" in source
    assert 'sys.version_info[:2] != (3, 11)' in source
    assert '"/opt/ros/humble" in rclpy.__file__' in source


def test_pixi_wrappers_do_not_hardcode_a_developer_home_directory():
    for path in (CALIBRATION_WRAPPER, RUNTIME_WRAPPER):
        source = path.read_text(encoding="utf-8")
        assert "/home/" not in source
        assert "PIXI_BIN" in source
        assert 'command -v pixi' in source


def test_local_capture_and_mujoco_logs_are_ignored_by_git():
    source = GITIGNORE.read_text(encoding="utf-8")
    assert "recordings/" in source
    assert "MUJOCO_LOG.TXT" in source


def test_runtime_has_required_topic_readiness_and_optional_children():
    source = RUNTIME_WRAPPER.read_text(encoding="utf-8")
    assert "pico_m0_readiness.py" in source
    assert "ros2 topic echo" not in source
    assert "--viewer" in source
    assert "--record" in source
    assert "--duration" in source
    assert "pico_m0_comparison_report record" in source
    assert "smpl_mujoco_visualizer" in source


def test_runtime_viewer_uses_the_explicit_ik_frame_skeleton():
    source = RUNTIME_WRAPPER.read_text(encoding="utf-8")
    viewer_block = source.split("if [[ \"$viewer\" == true ]]", 1)[1]
    viewer_block = viewer_block.split("fi", 1)[0]

    assert "--topic /pico/smpl_palm_corrected_ik" in viewer_block
    assert "--raw-topic /pico/smpl_raw" in viewer_block


def test_runtime_omits_empty_optional_geometry_launch_arguments():
    source = RUNTIME_WRAPPER.read_text(encoding="utf-8")

    assert 'if [[ -n "$left_geometry" ]]' in source
    assert 'if [[ -n "$right_geometry" ]]' in source
    assert '"left_arm_geometry_artifact:=$left_geometry"' not in source.split(
        'if [[ -n "$left_geometry" ]]', 1
    )[0]
    assert '"require_wrist_pivot_artifact:=false"' in source


def test_runtime_does_not_let_viewer_failure_terminate_filter():
    source = RUNTIME_WRAPPER.read_text(encoding="utf-8")
    assert "viewer exited; estimator remains active" in source
    assert "wait \"$filter_pid\"" in source


def test_orientation_wrapper_is_side_specific_and_preserves_other_calibrations():
    source = ORIENTATION_WRAPPER.read_text(encoding="utf-8")
    assert '<left|right>' in source
    assert '/pico/palm_orientation/${side}/calibrate' in source
    assert 'pico_${side}_palm_tcp.yaml' in source
    assert "腕部距离和上臂/前臂骨长不会改变" in source
    assert "pico_stop_process_group" in source
    assert "PIXI_PROJECT_ROOT" in source


def test_tremor_recorder_default_plan_has_three_single_30_second_stages():
    result = subprocess.run(
        ["bash", str(TREMOR_RECORDER), "--dry-run"],
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    stage_lines = [
        line for line in result.stdout.splitlines() if line.startswith("stage=")
    ]
    assert stage_lines == [
        "stage=fixed label=固定手柄 countdown_s=5 duration_s=30 repeat=1/1",
        "stage=elbow_supported label=肘部支撑 countdown_s=5 duration_s=30 repeat=1/1",
        "stage=arm_unsupported label=自然悬臂 countdown_s=5 duration_s=30 repeat=1/1",
    ]
    assert "rest" not in result.stdout.lower()


def test_tremor_recorder_rejects_non_positive_timing_before_hardware_access():
    for option in ("--duration", "--countdown"):
        result = subprocess.run(
            ["bash", str(TREMOR_RECORDER), option, "0", "--dry-run"],
            text=True,
            capture_output=True,
            check=False,
        )

        assert result.returncode == 2
        assert "must be a positive integer" in result.stderr
        assert "adb" not in result.stderr.lower()


def test_tremor_recorder_owns_startup_recording_and_cleanup_chain():
    source = TREMOR_RECORDER.read_text(encoding="utf-8")

    assert "adb forward tcp:9999 tcp:9999" in source
    assert "start_pico_bridge.launch.py" in source
    assert '"$repo_root/scripts/start_pico_m0.sh"' in source
    assert 'source "$repo_root/src/pico_bridge/scripts/pico_process_cleanup.sh"' in source
    assert "setsid ros2 bag record" in source
    assert 'pico_stop_process_group "$recorder_pid" 600 20' in source
    assert "trap cleanup EXIT" in source
    assert "trap on_signal INT TERM" in source
    assert "metadata.yaml" in source


def test_tremor_recorder_captures_raw_palm_and_corrected_streams():
    source = TREMOR_RECORDER.read_text(encoding="utf-8")

    required_topics = (
        "/pico/pose/head",
        "/pico/pose/left_hand",
        "/pico/pose/right_hand",
        "/pico/palm_left",
        "/pico/palm_right",
        "/pico/smpl_raw",
        "/pico/smpl_palm_corrected",
        "/pico/smpl_palm_corrected_ik",
        "/pico/smpl_palm_corrected/status",
        "/pico/tracking_epoch",
    )
    for topic in required_topics:
        assert topic in source


def test_tremor_recorder_reuses_existing_ros_runtime_without_owning_it():
    source = TREMOR_RECORDER.read_text(encoding="utf-8")

    assert 'node_is_running "/pico_bridge"' in source
    assert 'node_is_running "/pico_palm_skeleton_filter"' in source
    assert "复用当前 PICO bridge" in source
    assert "复用当前掌心与 corrected IK 链路" in source
    assert 'child_pids+=("$bridge_pid")' in source
    assert 'child_pids+=("$m0_pid")' in source


def test_tianji_tmux_launcher_contract_starts_only_the_pico_side():
    source = TIANJI_TMUX_LAUNCHER.read_text(encoding="utf-8")

    assert 'session_name="pico_tianji_teleop"' in source
    assert 'window_name="driver"' in source
    assert 'window_name="m0"' in source
    assert 'window_name="bridge"' in source
    assert "start_pico_driver.sh" in source
    assert "start_pico_m0.sh" in source
    assert "start_pico_m0.sh --viewer" in source
    assert "start_tianji_mujoco_teleop.launch.py" in source
    assert "destination_address:=127.0.0.1" in source
    assert "destination_port:=15000" in source
    assert "position_retargeting_mode:=robot_arm_segments" in source
    assert "robot_arm_reach_scale:=0.95" in source
    assert 'pixi_path="$(command -v pixi)"' in source
    assert "run bash -lc" in source
    assert "tianji_qp_ik_viewer" not in source
    assert "TJ_arm_control_DLS_IK" not in source


def test_tianji_tmux_launcher_gets_tmux_from_pixi_before_checking_it():
    source = TIANJI_TMUX_LAUNCHER.read_text(encoding="utf-8")
    manifest = PIXI_MANIFEST.read_text(encoding="utf-8")

    assert re.search(r"^tmux\s*=", manifest, flags=re.MULTILINE)
    assert source.index('if [[ "${PIXI_PROJECT_ROOT:-}" != "$repo_root" ]]') < source.index(
        "command -v tmux"
    )


def test_tianji_tmux_stopper_is_a_safe_executable_wrapper():
    assert TIANJI_TMUX_STOPPER.exists()
    assert os.access(TIANJI_TMUX_STOPPER, os.X_OK)

    source = TIANJI_TMUX_STOPPER.read_text(encoding="utf-8")
    assert 'exec "$script_dir/start_tianji_pico_teleop.sh" --stop' in source
    assert "pkill" not in source
    assert "adb" not in source


def test_tianji_tmux_stopper_stops_only_the_managed_session_idempotently(tmp_path):
    environment = _fake_tianji_launcher_environment(tmp_path)
    log_path = Path(environment["FAKE_TMUX_LOG"])

    started = subprocess.run(
        ["bash", str(TIANJI_TMUX_LAUNCHER), "--detach"],
        text=True,
        capture_output=True,
        env=environment,
        check=False,
    )
    assert started.returncode == 0, started.stderr

    for expected_message in (
        "Stopped Tianji PICO session: pico_tianji_teleop",
        "Tianji PICO session is already stopped: pico_tianji_teleop",
    ):
        stopped = subprocess.run(
            ["bash", str(TIANJI_TMUX_STOPPER)],
            text=True,
            capture_output=True,
            env=environment,
            check=False,
        )
        assert stopped.returncode == 0, stopped.stderr
        assert expected_message in stopped.stdout

    final_log = log_path.read_text(encoding="utf-8")
    assert final_log.count("kill-session -t pico_tianji_teleop") == 1
    assert "kill-server" not in final_log


def test_tianji_tmux_launcher_restarts_one_fixed_session_without_duplicates(tmp_path):
    environment = _fake_tianji_launcher_environment(tmp_path)
    log_path = Path(environment["FAKE_TMUX_LOG"])

    first = subprocess.run(
        ["bash", str(TIANJI_TMUX_LAUNCHER), "--detach"],
        text=True,
        capture_output=True,
        env=environment,
        check=False,
    )
    assert first.returncode == 0, first.stderr

    first_log = log_path.read_text(encoding="utf-8")
    assert first_log.count("new-session") == 1
    assert "new-session -d -s pico_tianji_teleop -n driver" in first_log
    assert "new-window -t pico_tianji_teleop -n m0" in first_log
    assert "new-window -t pico_tianji_teleop -n bridge" in first_log

    second = subprocess.run(
        ["bash", str(TIANJI_TMUX_LAUNCHER), "--detach"],
        text=True,
        capture_output=True,
        env=environment,
        check=False,
    )
    assert second.returncode == 0, second.stderr
    second_log = log_path.read_text(encoding="utf-8")
    assert second_log.count("new-session") == 2
    assert second_log.count("kill-session -t pico_tianji_teleop") == 1
    assert second_log.count("cleanup ") == 2
    assert second_log.rindex("kill-session -t pico_tianji_teleop") < second_log.rindex(
        "cleanup "
    ) < second_log.rindex("new-session")

    status = subprocess.run(
        ["bash", str(TIANJI_TMUX_LAUNCHER), "--status"],
        text=True,
        capture_output=True,
        env=environment,
        check=False,
    )
    assert status.returncode == 0, status.stderr
    assert "driver" in status.stdout
    assert "m0" in status.stdout
    assert "bridge" in status.stdout

    stopped = subprocess.run(
        ["bash", str(TIANJI_TMUX_LAUNCHER), "--stop"],
        text=True,
        capture_output=True,
        env=environment,
        check=False,
    )
    assert stopped.returncode == 0, stopped.stderr
    final_log = log_path.read_text(encoding="utf-8")
    assert "kill-session -t pico_tianji_teleop" in final_log
    assert "kill-server" not in final_log
    assert final_log.count("cleanup ") == 2


def test_tianji_tmux_launcher_preserves_existing_session_when_adb_preflight_fails(tmp_path):
    environment = _fake_tianji_launcher_environment(tmp_path)
    state_path = Path(environment["FAKE_TMUX_STATE"])
    state_path.touch()
    environment["FAKE_ADB_STATE"] = "offline"
    log_path = Path(environment["FAKE_TMUX_LOG"])

    result = subprocess.run(
        ["bash", str(TIANJI_TMUX_LAUNCHER), "--detach"],
        text=True,
        capture_output=True,
        env=environment,
        check=False,
    )

    assert result.returncode == 2
    assert state_path.exists()
    log = log_path.read_text(encoding="utf-8") if log_path.exists() else ""
    assert "kill-session" not in log
    assert "cleanup " not in log


def test_tianji_tmux_launcher_fails_closed_when_historical_cleanup_fails(tmp_path):
    environment = _fake_tianji_launcher_environment(tmp_path)
    environment["FAKE_CLEANUP_EXIT"] = "2"
    log_path = Path(environment["FAKE_TMUX_LOG"])

    result = subprocess.run(
        ["bash", str(TIANJI_TMUX_LAUNCHER), "--detach"],
        text=True,
        capture_output=True,
        env=environment,
        check=False,
    )

    assert result.returncode == 2
    log = log_path.read_text(encoding="utf-8")
    assert "cleanup " in log
    assert "new-session" not in log
