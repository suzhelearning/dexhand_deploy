#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

if (($# < 1)) || [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  printf '%s\n' \
    '用法：pixi run replay-controller-only -- TRACE.jsonl [--speed N]' >&2
  if (($# < 1)); then
    exit 2
  fi
  exit 0
fi

TRACE_FILE="$(realpath -m -- "$1")"
shift
REPLAY_SPEED=1.0
while (($#)); do
  case "$1" in
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

acquire_teleop_guard controller-only-replay
install_teleop_cleanup_traps
"${SCRIPT_DIR}/doctor.sh"
activate_bundle_runtime
assert_no_conflicting_teleop_nodes

SIM_NODE="${PROJECT_PREFIX}/lib/pico_body_tianji/tianji_kinematic_sim"
PARAMETERS="${PROJECT_PREFIX}/share/pico_body_tianji/config/mode/controller_only/controller_only_ik.yaml"
URDF_PATH="${PROJECT_PREFIX}/share/pico_body_tianji/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf"
TRACE_ENTRY="${PROJECT_PREFIX}/lib/pico_body_tianji/controller_only_trace"
for required in \
  "${SIM_NODE}" \
  "${PARAMETERS}" \
  "${URDF_PATH}" \
  "${TRACE_ENTRY}"
do
  if [[ ! -f "${required}" ]]; then
    printf '错误：controller-only replay 运行文件不存在：%s\n' "${required}" >&2
    exit 1
  fi
done

printf '%s\n' \
  '启动 preview-only controller-only replay。' \
  '该运行锁和输入身份均不能用于启动真机桥。'

# C++ 节点只接受裸 key:=value 参数；yaml_params_for 直接输出该格式。
mapfile -t sim_arguments < <(
  yaml_params_for tianji_kinematic_sim "${PARAMETERS}" \
    "urdf_path:=${URDF_PATH}"
)
for index in "${!sim_arguments[@]}"; do
  sim_arguments[index]="${sim_arguments[index]#--param }"
done
setsid "${SIM_NODE}" "${sim_arguments[@]}" &
SIM_PID=$!
register_teleop_process_group "${SIM_PID}" controller-only-replay-ik-solver 5

setsid python "${TRACE_ENTRY}" replay "${TRACE_FILE}" --speed "${REPLAY_SPEED}" &
REPLAY_PID=$!
register_teleop_process_group "${REPLAY_PID}" controller-only-replay 5
set +e
wait "${REPLAY_PID}"
task_exit=$?
set -e
exit "${task_exit}"
