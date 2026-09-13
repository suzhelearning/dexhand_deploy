#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
side=""
tcp_artifact=""
wrist_pivot_artifact=""
start_mode="space"
domain="42"
output_dir=""
countdown_s="3"
duration_scale="1.0"
verbose="0"

resolve_pixi_bin() {
  if [[ -n "${PIXI_BIN:-}" ]]; then
    if [[ ! -x "$PIXI_BIN" ]]; then
      echo "PIXI_BIN is not executable: $PIXI_BIN" >&2
      return 1
    fi
    printf '%s\n' "$PIXI_BIN"
    return 0
  fi
  if command -v pixi >/dev/null 2>&1; then
    command -v pixi
    return 0
  fi
  if [[ -x "$HOME/.pixi/bin/pixi" ]]; then
    printf '%s\n' "$HOME/.pixi/bin/pixi"
    return 0
  fi
  echo "pixi not found; install pixi or set PIXI_BIN" >&2
  return 1
}

usage() {
  echo "usage: $0 --side left|right --tcp-artifact FILE --wrist-pivot-artifact FILE [--start-mode space|auto] [--domain ID] [--output-dir DIR]" >&2
}

original_args=("$@")
while (($#)); do
  case "$1" in
    --side) side="${2:-}"; shift 2 ;;
    --tcp-artifact) tcp_artifact="${2:-}"; shift 2 ;;
    --wrist-pivot-artifact) wrist_pivot_artifact="${2:-}"; shift 2 ;;
    --start-mode) start_mode="${2:-}"; shift 2 ;;
    --domain) domain="${2:-}"; shift 2 ;;
    --output-dir) output_dir="${2:-}"; shift 2 ;;
    --countdown-s) countdown_s="${2:-}"; shift 2 ;;
    --duration-scale) duration_scale="${2:-}"; shift 2 ;;
    --verbose) verbose="1"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ "$side" != "left" && "$side" != "right" ]]; then
  echo "--side must be left or right" >&2
  exit 2
fi
if [[ "$start_mode" != "space" && "$start_mode" != "auto" ]]; then
  echo "--start-mode must be space or auto" >&2
  exit 2
fi
for artifact in "$tcp_artifact" "$wrist_pivot_artifact"; do
  if [[ -z "$artifact" || ! -r "$artifact" ]]; then
    echo "artifact missing or unreadable: $artifact" >&2
    exit 2
  fi
done

if [[ "${PICO_ARM_GEOMETRY_CLEAN_ENV:-0}" != "1" ]]; then
  pixi_bin="$(resolve_pixi_bin)"
  exec env \
    -u PYTHONPATH \
    -u PYTHONHOME \
    -u AMENT_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH \
    -u COLCON_PREFIX_PATH \
    -u CONDA_PREFIX \
    -u CONDA_DEFAULT_ENV \
    -u CONDA_PROMPT_MODIFIER \
    -u CONDA_SHLVL \
    -u CONDA_BUILD \
    PICO_ARM_GEOMETRY_CLEAN_ENV=1 \
    ROS_DOMAIN_ID="$domain" \
    EXO_REQUESTED_ROS_DOMAIN_ID="$domain" \
    "$pixi_bin" run --manifest-path "$repo_root/pixi.toml" \
    bash --noprofile --norc "$0" "${original_args[@]}"
fi

cd "$repo_root"
source "$repo_root/src/pico_bridge/scripts/pico_process_cleanup.sh"
set +u
source install/local_setup.bash
set -u
export ROS_DOMAIN_ID="$domain"
export EXO_REQUESTED_ROS_DOMAIN_ID="$domain"

python - <<'PY'
import sys
import rclpy

if sys.version_info[:2] != (3, 11):
    raise SystemExit(f"PICO requires Python 3.11, got {sys.version.split()[0]}")
if "/opt/ros/humble" in rclpy.__file__:
    raise SystemExit(f"contaminated system rclpy: {rclpy.__file__}")
if ".pixi/envs/default" not in rclpy.__file__:
    raise SystemExit(f"unexpected rclpy path: {rclpy.__file__}")
PY

publisher_pid=""
cleanup() {
  local owned_pid="$publisher_pid"
  trap - EXIT INT TERM
  publisher_pid=""
  pico_stop_process_group "$owned_pid"
}
trap cleanup EXIT INT TERM

setsid ros2 run pico_bridge pico_palm_tcp_publisher \
  --side "$side" --artifact "$tcp_artifact" &
publisher_pid=$!

calibrator_args=(
  --side "$side"
  --tcp-artifact "$tcp_artifact"
  --wrist-pivot-artifact "$wrist_pivot_artifact"
  --start-mode "$start_mode"
  --countdown-s "$countdown_s"
  --duration-scale "$duration_scale"
)
if [[ -n "$output_dir" ]]; then
  calibrator_args+=(--output-dir "$output_dir")
fi
if [[ "$verbose" == "1" ]]; then
  calibrator_args+=(--verbose)
fi

ros2 run pico_bridge pico_arm_geometry_calibrator "${calibrator_args[@]}"
