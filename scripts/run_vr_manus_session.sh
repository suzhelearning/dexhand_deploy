#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"
source "${SCRIPT_DIR}/graceful_process_stop.sh"

# All argument/asset errors precede runtime locks, router connections or SDKs.
python "${SCRIPT_DIR}/vr_manus_live.py" --check "$@" >/dev/null
export TIANJI_REQUIRED_CAPABILITY=simulation
activate_bundle_runtime
guard_conflicts=()
if [[ "${TIANJI_EMBEDDED_PICO_DOWNSTREAM:-}" != 1 ]]; then
  guard_conflicts=("${TELEOP_DEVICE_ROUTE_MODES[@]}")
  acquire_teleop_guard vr_manus_sim "${guard_conflicts[@]}"
fi
dual_terminal_state=""
dual_pid=""
dual_start_ticks=""
dual_cleanup() {
  local result=$?
  trap '' INT TERM
  # Let Python close its recorder, SDK workers and viewer before any group
  # signal reaches those children. Existing group cleanup remains the fallback.
  if [[ -n "$dual_pid" ]]; then
    graceful_stop_session_owner "$dual_pid" 40 "$dual_start_ticks" || true
  fi
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
if [[ "${TIANJI_EMBEDDED_PICO_DOWNSTREAM:-}" == 1 ]]; then
  # The embedded supervisor owns the shared guard and process-group record.
  # Keep this Python owner in the downstream group so one stale parent record
  # can recover the complete nested route without a second guard/release lock.
  env PYTHONUNBUFFERED=1 python "${SCRIPT_DIR}/vr_manus_live.py" "$@" <&0 \
    > >(tee -- "${dual_log_path}") 2>&1 &
else
  setsid env PYTHONUNBUFFERED=1 python "${SCRIPT_DIR}/vr_manus_live.py" "$@" <&0 \
    > >(tee -- "${dual_log_path}") 2>&1 &
fi
dual_pid=$!
if [[ "${TIANJI_EMBEDDED_PICO_DOWNSTREAM:-}" != 1 ]] &&
   declare -F _process_start_ticks >/dev/null 2>&1; then
  dual_start_ticks="$(_process_start_ticks "$dual_pid" || true)"
  if ! register_teleop_process_group "${dual_pid}" vr_manus_live 30; then
    kill -TERM -- "-${dual_pid}" 2>/dev/null || true
    wait "${dual_pid}" 2>/dev/null || true
    exit 1
  fi
fi
wait "${dual_pid}"
