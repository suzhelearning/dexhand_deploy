#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

config=""
record_path=""
no_adb_forward=false
print_frames=false
duration=""
extra_args=()
while (($#)); do
  case "$1" in
    --config) config="${2:-}"; shift 2 ;;
    --record) record_path="${2:-}"; shift 2 ;;
    --no-adb-forward) no_adb_forward=true; shift ;;
    --print) print_frames=true; shift ;;
    --duration) duration="${2:-}"; shift 2 ;;
    --help|-h)
      printf '%s\n' '用法: run_observation_session.sh --config PATH [--record PATH] [--no-adb-forward] [--print] [--duration SECONDS]'
      exit 0 ;;
    --) shift; extra_args+=("$@"); break ;;
    *) extra_args+=("$1"); shift ;;
  esac
done
[[ -n "${config}" && -f "${config}" ]] || {
  printf '%s\n' '错误：观察会话必须指定存在的 observation config。' >&2
  exit 2
}
if [[ -n "${record_path}" && -e "${record_path}" ]]; then
  printf '错误：拒绝覆盖已有 recording: %s\n' "${record_path}" >&2
  exit 2
fi
if [[ -n "${record_path}" ]]; then
  mkdir -p -- "$(dirname -- "${record_path}")"
fi

export TIANJI_ROUTER_ENDPOINT="${TIANJI_ROUTER_ENDPOINT:-tcp/127.0.0.1:7447}"
export TIANJI_COMPONENT_INSTANCE_ID="${TIANJI_COMPONENT_INSTANCE_ID:-$(new_instance_id)}"
activate_bundle_runtime
entry="${BUNDLE_ROOT}/src/tianji_teleop/scripts/hand_tracking_observation"
args=(--config "${config}")
[[ -n "${record_path}" ]] && args+=(--record "${record_path}")
[[ "${no_adb_forward}" == true ]] && args+=(--no-adb-forward)
[[ "${print_frames}" == true ]] && args+=(--print)
[[ -n "${duration}" ]] && args+=(--duration "${duration}")
args+=("${extra_args[@]}")
exec python "${entry}" "${args[@]}"
