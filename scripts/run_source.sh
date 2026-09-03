#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

source_id=""
config_override=""
while (($#)); do
  case "$1" in
    --source|--profile) source_id="${2:-}"; shift 2 ;;
    --config) config_override="${2:-}"; shift 2 ;;
    --help|-h)
      printf '%s\n' '用法: run_source.sh --source {mocap_live|h5_replay|regrind_policy} [--config PATH] [参数...]'
      exit 0 ;;
    --) shift; break ;;
    *) break ;;
  esac
done
if [[ -z "${source_id}" ]]; then
  printf '%s\n' '错误：必须指定 --source。' >&2
  exit 2
fi
case "${source_id}" in
  mocap_live) entry="${BUNDLE_ROOT}/src/tianji_teleop/scripts/mocap_live"; default_config="sources/mocap_live.yaml" ;;
  h5_replay) entry="${BUNDLE_ROOT}/src/tianji_teleop/scripts/mocap_h5_replay"; default_config="sources/h5_replay.yaml" ;;
  regrind_policy) entry="${BUNDLE_ROOT}/src/tianji_teleop/scripts/regrind_policy"; default_config="sources/regrind_policy.yaml" ;;
  target_replay|joint_replay)
    entry="${BUNDLE_ROOT}/src/tianji_teleop/scripts/${source_id}"
    default_config="replay/${source_id%_replay}.yaml" ;;
  diagnostic_mocap_calibration)
    entry="${BUNDLE_ROOT}/src/tianji_teleop/scripts/mocap_calibration"
    default_config="diagnostics/mocap_calibration.yaml" ;;
  *)
    printf '错误：未知 source: %s\n' "${source_id}" >&2; exit 2 ;;
esac
config="${config_override}"
if [[ -z "${config}" ]]; then config="$(canonical_config "${default_config}")"; fi
if [[ ! -x "${entry}" ]]; then
  printf '错误：source entry 不存在或不可执行: %s\n' "${entry}" >&2
  exit 1
fi
export TIANJI_ROUTER_ENDPOINT="${TIANJI_ROUTER_ENDPOINT:-tcp/127.0.0.1:7447}"
export TIANJI_ROUTER_ZID="${TIANJI_ROUTER_ZID:-$(require_router)}"
export TIANJI_COMPONENT_INSTANCE_ID="${TIANJI_COMPONENT_INSTANCE_ID:-$(new_instance_id)}"
export TIANJI_COORDINATOR_INSTANCE_ID="${TIANJI_COORDINATOR_INSTANCE_ID:?必须由run_session注入 TIANJI_COORDINATOR_INSTANCE_ID}"
activate_bundle_runtime
case "${source_id}" in
  target_replay|joint_replay)
    exec python "${entry}" "$@" --config "${config}" --headless ;;
  *) exec python "${entry}" --config "${config}" "$@" ;;
esac
