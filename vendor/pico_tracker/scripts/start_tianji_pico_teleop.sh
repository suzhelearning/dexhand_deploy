#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
session_name="pico_tianji_teleop"
mode="attach"
original_args=("$@")

usage() {
  cat <<'EOF'
Usage: ./scripts/start_tianji_pico_teleop.sh [OPTION]

Start and manage the PICO-side Tianji teleoperation pipeline in tmux.
The Tianji DLS/Spark MuJoCo viewer is not started by this script.

Options:
  --detach  Start the session without attaching to it.
  --status  Show the managed tmux session and its windows.
  --stop    Stop only the managed pico_tianji_teleop session.
  --help    Show this help text.
EOF
}

if (($# > 1)); then
  usage >&2
  exit 2
fi

if (($# == 1)); then
  case "$1" in
    --detach)
      mode="detach"
      ;;
    --status)
      mode="status"
      ;;
    --stop)
      mode="stop"
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
fi

command -v pixi >/dev/null || {
  echo "pixi not found; install Pixi before starting the Tianji PICO pipeline." >&2
  exit 2
}

if [[ "${PIXI_PROJECT_ROOT:-}" != "$repo_root" ]]; then
  exec pixi run bash "$repo_root/scripts/start_tianji_pico_teleop.sh" "${original_args[@]}"
fi

command -v tmux >/dev/null || {
  echo "tmux is missing from the Pixi environment; run: pixi install" >&2
  exit 2
}

session_exists() {
  tmux has-session -t "$session_name" 2>/dev/null
}

if [[ "$mode" == "status" ]]; then
  if ! session_exists; then
    echo "Tianji PICO session is not running: $session_name" >&2
    exit 1
  fi
  tmux list-windows -t "$session_name"
  exit 0
fi

if [[ "$mode" == "stop" ]]; then
  if session_exists; then
    tmux kill-session -t "$session_name"
    echo "Stopped Tianji PICO session: $session_name"
  else
    echo "Tianji PICO session is already stopped: $session_name"
  fi
  exit 0
fi

command -v adb >/dev/null || {
  echo "adb not found; install Android platform tools first." >&2
  exit 2
}
if [[ ! -f "$repo_root/install/local_setup.bash" ]]; then
  echo "PICO workspace is not built: missing $repo_root/install/local_setup.bash" >&2
  echo "Run: cd $repo_root && pixi run build-core" >&2
  exit 2
fi
if [[ "$(adb get-state 2>/dev/null || true)" != "device" ]]; then
  echo "No authorized PICO device detected by adb." >&2
  echo "Connect the headset over USB, accept USB debugging, then run adb devices." >&2
  exit 2
fi
command -v python3 >/dev/null || {
  echo "python3 is missing from the Pixi environment." >&2
  exit 2
}

if session_exists; then
  echo "Restarting Tianji PICO session: $session_name"
  tmux kill-session -t "$session_name"
fi
python3 "$repo_root/scripts/cleanup_tianji_pico_processes.py"

pixi_path="$(command -v pixi)"
printf -v repo_quoted '%q' "$repo_root"
printf -v pixi_quoted '%q' "$pixi_path"
ros_environment="export ROS_DOMAIN_ID=120 EXO_REQUESTED_ROS_DOMAIN_ID=120 ROS_LOCALHOST_ONLY=1 ROS2CLI_DISABLE_DAEMON=1"
driver_inner="cd $repo_quoted && $ros_environment && exec ./scripts/start_pico_driver.sh"
m0_inner="cd $repo_quoted && $ros_environment && exec ./scripts/start_pico_m0.sh --viewer"
bridge_inner="cd $repo_quoted && $ros_environment && source install/local_setup.bash && exec ros2 launch pico_bridge start_tianji_mujoco_teleop.launch.py destination_address:=127.0.0.1 destination_port:=15000 position_retargeting_mode:=robot_arm_segments robot_arm_reach_scale:=0.95"
printf -v driver_inner_quoted '%q' "$driver_inner"
printf -v m0_inner_quoted '%q' "$m0_inner"
printf -v bridge_inner_quoted '%q' "$bridge_inner"
driver_command="exec $pixi_quoted run bash -lc $driver_inner_quoted"
m0_command="exec $pixi_quoted run bash -lc $m0_inner_quoted"
bridge_command="exec $pixi_quoted run bash -lc $bridge_inner_quoted"

window_name="driver"
tmux new-session -d -s "$session_name" -n "$window_name"
tmux set-option -t "$session_name" remain-on-exit on >/dev/null
tmux send-keys -t "$session_name:$window_name" "$driver_command" C-m

window_name="m0"
tmux new-window -t "$session_name" -n "$window_name"
tmux send-keys -t "$session_name:$window_name" "$m0_command" C-m

window_name="bridge"
tmux new-window -t "$session_name" -n "$window_name"
tmux send-keys -t "$session_name:$window_name" "$bridge_command" C-m
tmux select-window -t "$session_name:driver"

echo "Started Tianji PICO tmux session: $session_name"
echo "Windows: driver, m0, bridge"
echo "Detach with Ctrl-b d; stop with ./scripts/stop_tianji_pico_teleop.sh"

if [[ "$mode" == "detach" ]]; then
  exit 0
fi
exec tmux attach-session -t "$session_name"
