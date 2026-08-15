#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

if (($# < 1)) || [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  printf '%s\n' \
    '用法：pixi run replay-controller-only -- TRACE.jsonl [--topics-only] [--speed N]' >&2
  if (($# < 1)); then
    exit 2
  fi
  exit 0
fi

TRACE_FILE="$(realpath -m -- "$1")"
shift
WITH_RVIZ=true
REPLAY_SPEED=1.0
while (($#)); do
  case "$1" in
    --topics-only)
      WITH_RVIZ=false
      ;;
    --speed)
      shift
      if (($# == 0)); then
        printf '%s\n' '错误：--speed 缺少数值。' >&2
        exit 2
      fi
      REPLAY_SPEED="$1"
      ;;
    *)
      printf '错误：未知参数 %s\n' "$1" >&2
      exit 2
      ;;
  esac
  shift
done

if [[ ! -f "${TRACE_FILE}" ]]; then
  printf '错误：轨迹文件不存在：%s\n' "${TRACE_FILE}" >&2
  exit 1
fi
if [[ "${WITH_RVIZ}" == true && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  printf '%s\n' '错误：无图形环境，请增加 --topics-only。' >&2
  exit 1
fi

acquire_teleop_guard controller-only-replay
install_teleop_cleanup_traps
"${SCRIPT_DIR}/doctor.sh"
activate_bundle_runtime
assert_no_conflicting_teleop_nodes

with_rviz=false
if [[ "${WITH_RVIZ}" == true ]]; then
  with_rviz=true
fi

printf '%s\n' \
  '启动 preview-only controller-only replay。' \
  '该运行锁和输入身份均不能用于启动真机桥。'
setsid python "${ROS_ROOT}/bin/ros2" launch \
  pico_body_tianji controller_only_replay.launch.py \
  "trace_file:=${TRACE_FILE}" \
  "replay_speed:=${REPLAY_SPEED}" \
  "with_rviz:=${with_rviz}" &
REPLAY_PID=$!
register_teleop_process_group "${REPLAY_PID}" controller-only-replay 5
set +e
wait "${REPLAY_PID}"
task_exit=$?
set -e
exit "${task_exit}"
