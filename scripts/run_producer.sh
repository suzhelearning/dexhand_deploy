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
if [[ "${producer_id}" == ik ]]; then
  requested_backend="${TIANJI_VALIDATION_IK_BACKEND:-${backend:-}}"
  eval "$(
    pixi run python - "${config}" <<'PY'
import sys, yaml
value = yaml.safe_load(open(sys.argv[1], encoding="utf-8")) or {}
required = {"ik_backend", "arm_config", "rate_hz", "freshness_timeout_s", "maximum_joint_step_rad", "position_tolerance_m", "orientation_tolerance_rad", "worker_timeout_ms", "worker_restart_attempts"}
for key, item in value.items():
    name = "BACKEND" if key == "ik_backend" else key.upper()
    print(f"export TIANJI_IK_{name}={item!r}")
PY
  )"
  [[ -z "${requested_backend}" ]] || export TIANJI_IK_BACKEND="${requested_backend}"
fi
if [[ "${producer_id}" == ik ]]; then
  arm_config="$(canonical_config robot/arm.yaml)"
  urdf_path="${TIANJI_ARM_URDF:-${BUNDLE_ROOT}/src/pico_body_tianji/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf}"
  export TIANJI_ARM_CONFIG="${TIANJI_ARM_CONFIG:-${arm_config}}"
  export TIANJI_ARM_URDF="${urdf_path}"
  export TIANJI_IK_BACKEND="${TIANJI_IK_BACKEND:-${backend:-pinocchio_cpp}}"
  exec "${entry}" "$@"
fi
exec python "${entry}" "$@"
