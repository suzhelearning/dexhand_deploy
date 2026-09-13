#!/usr/bin/env bash
# Notify an owned session leader before group-wide fallback cleanup. Callers
# retain responsibility for reaping and removing all remaining process groups.
_graceful_process_start_ticks() {
  local pid="$1"
  [[ "${pid}" =~ ^[0-9]+$ && -r "/proc/${pid}/stat" ]] || return 1
  awk '{print $22}' "/proc/${pid}/stat"
}

graceful_stop_session_owner() {
  local owner_pid="$1"
  local timeout_s="$2"
  local expected_ticks="${3:-}"
  local actual_ticks=""
  local state=""
  local attempt
  [[ "$owner_pid" =~ ^[0-9]+$ && "$owner_pid" -gt 1 &&
     "$timeout_s" =~ ^[0-9]+$ && "$timeout_s" -gt 0 && "$timeout_s" -le 60 ]] || return 1
  actual_ticks="$(_graceful_process_start_ticks "$owner_pid" 2>/dev/null || true)"
  [[ -n "$actual_ticks" ]] || return 0
  if [[ -n "$expected_ticks" && "$actual_ticks" != "$expected_ticks" ]]; then
    return 1
  fi
  expected_ticks="$actual_ticks"
  kill -TERM "$owner_pid" 2>/dev/null || return 0
  for ((attempt = 0; attempt < timeout_s * 20; ++attempt)); do
    state="$(ps -o stat= -p "$owner_pid" 2>/dev/null)" || return 0
    [[ -n "$state" && "$state" != *Z* ]] || return 0
    actual_ticks="$(_graceful_process_start_ticks "$owner_pid" 2>/dev/null || true)"
    [[ "$actual_ticks" == "$expected_ticks" ]] || return 1
    sleep 0.05
  done
  return 1
}
