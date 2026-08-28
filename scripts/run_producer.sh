#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

producer_id=""
backend=""
config_override=""
while (($#)); do
  case "$1" in
    --producer|--profile) producer_id="${2:-}"; shift 2 ;;
    --backend) backend="${2:-}"; shift 2 ;;
    --config) config_override="${2:-}"; shift 2 ;;
    --help|-h)
      printf '%s\n' '用法: run_producer.sh --producer {ik|policy_hold} [--backend BACKEND] [--config PATH]'
      exit 0 ;;
    --) shift; break ;;
    *) break ;;
  esac
done
if [[ -z "${producer_id}" ]]; then
  printf '%s\n' '错误：必须指定 --producer。' >&2
  exit 2
fi
case "${producer_id}" in
  ik)
    entry="${BUNDLE_ROOT}/staging/ik/lib/pico_body_tianji/arm_ik_producer"
    [[ -x "${entry}" ]] || entry="${PROJECT_PREFIX}/lib/pico_body_tianji/arm_ik_producer.bin"
    default_config="producers/ik.yaml"
    ;;
  policy_hold)
    entry="${BUNDLE_ROOT}/src/pico_body_tianji/src/pico_body_tianji/scripts/policy_hold_producer"
    [[ -x "${entry}" ]] || entry="${BUNDLE_ROOT}/src/pico_body_tianji/scripts/policy_hold_producer"
    default_config="producers/policy_hold.yaml"
    ;;
  *) printf '错误：未知 producer: %s\n' "${producer_id}" >&2; exit 2 ;;
esac
if [[ ! -x "${entry}" ]]; then
  printf '错误：producer entry 不存在或不可执行: %s\n' "${entry}" >&2
  exit 1
fi
config="${config_override}"
if [[ -z "${config}" ]]; then config="$(canonical_config "${default_config}")"; fi
export TIANJI_ROUTER_ENDPOINT="${TIANJI_ROUTER_ENDPOINT:-tcp/127.0.0.1:7447}"
export TIANJI_ROUTER_ZID="${TIANJI_ROUTER_ZID:-$(require_router)}"
export TIANJI_COMPONENT_INSTANCE_ID="${TIANJI_COMPONENT_INSTANCE_ID:-$(new_instance_id)}"
export TIANJI_COORDINATOR_INSTANCE_ID="${TIANJI_COORDINATOR_INSTANCE_ID:?必须由run_session注入 TIANJI_COORDINATOR_INSTANCE_ID}"
activate_bundle_runtime
if [[ "${producer_id}" == ik ]]; then
  arm_config="$(canonical_config robot/arm.yaml)"
  urdf_path="${TIANJI_ARM_URDF:-${BUNDLE_ROOT}/src/pico_body_tianji/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf}"
  export TIANJI_ARM_CONFIG="${TIANJI_ARM_CONFIG:-${arm_config}}"
  export TIANJI_ARM_URDF="${urdf_path}"
  export TIANJI_IK_BACKEND="${TIANJI_IK_BACKEND:-${backend:-pinocchio_cpp}}"
  exec "${entry}" "$@"
fi
exec python "${entry}" "$@"
