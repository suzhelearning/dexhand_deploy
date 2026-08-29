#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

executor_id=""
mode=""
side="right"
config_override=""
headless=false
confirm_real=false
real_capability_provider=""
required_capability="${TIANJI_REQUIRED_CAPABILITY:-simulation}"
while (($#)); do
  case "$1" in
    --executor|--profile) executor_id="${2:-}"; shift 2 ;;
    --mode) mode="${2:-}"; shift 2 ;;
    --side) side="${2:-}"; shift 2 ;;
    --config) config_override="${2:-}"; shift 2 ;;
    --confirm-real) confirm_real=true; shift ;;
    --real-capability-provider) real_capability_provider="${2:-}"; shift 2 ;;
    --help|-h)
      printf '%s\n' '用法: run_executor.sh --executor {mujoco|marvin|wuji_hand2} [--side left|right] [--headless|--confirm-real]'
      exit 0 ;;
    --) shift; break ;;
    *) break ;;
  esac
done
if [[ -z "${executor_id}" ]]; then
  printf '%s\n' '错误：必须指定 --executor。' >&2
  exit 2
fi
case "${executor_id}" in
  mujoco)
    entry="${BUNDLE_ROOT}/src/pico_body_tianji/scripts/mujoco_executor"
    default_config="executors/mujoco.yaml"
    ;;
  marvin)
    entry="${BUNDLE_ROOT}/src/pico_body_tianji/scripts/marvin_executor"
    default_config="executors/marvin.yaml"
    [[ "${confirm_real}" == true ]] || { printf '%s\n' 'Marvin executor requires --confirm-real' >&2; exit 2; }
    ;;
  wuji_hand2)
    entry="${BUNDLE_ROOT}/src/pico_body_tianji/scripts/wuji_hand2_executor"
    default_config="executors/wuji_hand2.yaml"
    ;;
  *) printf '错误：未知 executor: %s\n' "${executor_id}" >&2; exit 2 ;;
esac
if [[ ! -x "${entry}" ]]; then
  printf '错误：executor entry 不存在或不可执行: %s\n' "${entry}" >&2
  exit 1
fi
config="${config_override}"
if [[ -z "${config}" ]]; then config="$(canonical_config "${default_config}")"; fi
export TIANJI_ROUTER_ENDPOINT="${TIANJI_ROUTER_ENDPOINT:-tcp/127.0.0.1:7447}"
export TIANJI_ROUTER_ZID="${TIANJI_ROUTER_ZID:-$(require_router)}"
export TIANJI_COMPONENT_INSTANCE_ID="${TIANJI_COMPONENT_INSTANCE_ID:-$(new_instance_id)}"
export TIANJI_COORDINATOR_INSTANCE_ID="${TIANJI_COORDINATOR_INSTANCE_ID:?必须由run_session注入 TIANJI_COORDINATOR_INSTANCE_ID}"
export TIANJI_ARM_CONFIG="${TIANJI_ARM_CONFIG:-$(canonical_config robot/arm.yaml)}"
export TIANJI_HAND_CONFIG="${TIANJI_HAND_CONFIG:-$(canonical_config robot/wuji_hand2.yaml)}"
case "${executor_id}" in
  mujoco)
    args=(--coordinator-instance-id "${TIANJI_COORDINATOR_INSTANCE_ID}" --publisher-instance-id "${TIANJI_COMPONENT_INSTANCE_ID}" --config "${config}")
    [[ "${headless}" == true ]] && args+=(--headless)
    [[ -n "${TIANJI_RUN_ID:-}" ]] && args+=(--run-id "${TIANJI_RUN_ID}")
    exec python "${entry}" "${args[@]}" "$@"
    ;;
  marvin)
    args=(--confirm-real --config "${config}")
    [[ -n "${MARVIN_ROBOT_IP:-}" ]] && args+=(--robot-ip "${MARVIN_ROBOT_IP}")
    export TIANJI_REAL_CAPABILITY_PROVIDER="${real_capability_provider:-pico_body_tianji.executors.marvin.preflight:trusted_real_capability}"
    exec python "${entry}" "${args[@]}" "$@"
    ;;
  wuji_hand2)
    [[ "${mode}" == direct || "${mode}" == retarget ]] || { printf '%s\n' 'Wuji executor requires --mode direct|retarget' >&2; exit 2; }
    args=(--mode "${mode}" --side "${side}" --config "${config}")
    [[ "${required_capability}" == simulation ]] && args+=(--dry-run)
    exec python "${entry}" "${args[@]}" "$@"
    ;;
esac
