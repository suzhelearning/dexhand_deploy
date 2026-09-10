#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

# All argument/asset errors precede runtime locks, router connections or SDKs.
python "${SCRIPT_DIR}/vr_manus_live.py" --check "$@" >/dev/null
export TIANJI_REQUIRED_CAPABILITY=simulation
activate_bundle_runtime
acquire_teleop_guard vr_manus_sim
dual_terminal_state=""
dual_cleanup() {
  local result=$?
  if ! teleop_cleanup_and_release; then return 1; fi
  if [[ -n "${dual_terminal_state}" ]]; then
    stty "${dual_terminal_state}" <&0 || return 1
  fi
  return "${result}"
}
trap dual_cleanup EXIT
trap 'exit 130' INT TERM
existing_tokens="$(read_teleop_node_list)"
assert_profile_domains_free "${existing_tokens}"
export TIANJI_ROUTER_ZID="$(read_router_zid)"
export TIANJI_RUN_ID="$(new_instance_id)"
export TIANJI_DUAL_INSTANCE_ID="$(new_instance_id)"
export TIANJI_DUAL_MANAGED=1
if [[ -t 0 ]]; then
  dual_terminal_state="$(stty -g <&0)"
fi
dual_log_path="${TELEOP_RUNTIME_DIR}/${TIANJI_RUN_ID}-vr-manus-live.log"
setsid env PYTHONUNBUFFERED=1 python "${SCRIPT_DIR}/vr_manus_live.py" "$@" <&0 \
  > >(tee -- "${dual_log_path}") 2>&1 &
dual_pid=$!
if ! register_teleop_process_group "${dual_pid}" vr_manus_live 30; then
  kill -TERM -- "-${dual_pid}" 2>/dev/null || true
  wait "${dual_pid}" 2>/dev/null || true
  exit 1
fi
wait "${dual_pid}"
