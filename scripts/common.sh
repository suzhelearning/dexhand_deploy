#!/usr/bin/env bash
set -euo pipefail

BUNDLE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_PREFIX="${BUNDLE_ROOT}/runtime/pico_body_tianji"
ABI_LIBRARY_ROOT="${BUNDLE_ROOT}/runtime/abi/lib"
PIN_LIBRARY_ROOT="${BUNDLE_ROOT}/runtime/pin/lib"
ZENOH_LIBRARY_ROOT="${BUNDLE_ROOT}/vendor/zenoh/lib"
ZENOH_CPP_INCLUDE_ROOT="${BUNDLE_ROOT}/vendor/zenoh-cpp/include"
ZENOH_C_INCLUDE_ROOT="${BUNDLE_ROOT}/vendor/zenoh/include"
_TELEOP_RUNTIME_BASE="${XDG_RUNTIME_DIR:-/tmp}"
TELEOP_RUNTIME_DIR="${PICO_TIANJI_RUNTIME_DIR:-${_TELEOP_RUNTIME_BASE}/pico-tianji-teleop-${UID}}"
TELEOP_GUARDS_DIR="${TELEOP_RUNTIME_DIR}/guards"
TELEOP_LEGACY_GUARD_DIR="${TELEOP_RUNTIME_DIR}/guard"
TELEOP_GUARD_DIR=""
TELEOP_OWNER_FILE=""
TELEOP_CHILDREN_FILE=""
TELEOP_TAKEOVER_LOCK_FILE="${TELEOP_RUNTIME_DIR}/takeover.lock"
_TELEOP_GUARD_HELD=false
_TELEOP_TAKEOVER_FD=""

_select_guard_directory() {
  TELEOP_GUARD_DIR="$1"
  TELEOP_OWNER_FILE="${TELEOP_GUARD_DIR}/owner"
  TELEOP_CHILDREN_FILE="${TELEOP_GUARD_DIR}/children"
}

_lock_guard_administration() {
  if ! command -v flock >/dev/null 2>&1; then
    printf '%s\n' '错误：系统缺少 flock，无法安全管理遥操作运行锁。' >&2
    return 1
  fi
  exec {_TELEOP_TAKEOVER_FD}>"${TELEOP_TAKEOVER_LOCK_FILE}"
  flock -x "${_TELEOP_TAKEOVER_FD}"
}

_unlock_guard_administration() {
  if [[ -z "${_TELEOP_TAKEOVER_FD}" ]]; then
    return 0
  fi
  flock -u "${_TELEOP_TAKEOVER_FD}" 2>/dev/null || true
  exec {_TELEOP_TAKEOVER_FD}>&-
  _TELEOP_TAKEOVER_FD=""
}

_process_start_ticks() {
  local pid="$1"
  if [[ ! "${pid}" =~ ^[0-9]+$ || ! -r "/proc/${pid}/stat" ]]; then
    return 1
  fi
  awk '{print $22}' "/proc/${pid}/stat"
}

_same_process_is_alive() {
  local pid="$1"
  local expected_ticks="$2"
  local actual_ticks=""
  if ! kill -0 "${pid}" 2>/dev/null; then
    return 1
  fi
  actual_ticks="$(_process_start_ticks "${pid}")" || return 1
  [[ "${actual_ticks}" == "${expected_ticks}" ]]
}

_process_group_is_alive() {
  local process_group="$1"
  ps -eo pgid=,stat= 2>/dev/null |
    awk -v target="${process_group}" '
      $1 == target && $2 !~ /^Z/ {
        found = 1
      }
      END {
        exit !found
      }
    '
}

_stop_recorded_process_group() {
  local process_group="$1"
  local expected_ticks="$2"
  local term_timeout_s="$3"
  local label="$4"
  local actual_ticks=""
  local attempt=0
  local term_wait_steps=0

  if [[ ! "${term_timeout_s}" =~ ^[0-9]+$ ]] ||
     ((term_timeout_s < 1 || term_timeout_s > 60)); then
    term_timeout_s=5
  fi
  term_wait_steps=$((term_timeout_s * 20))

  if [[ ! "${process_group}" =~ ^[0-9]+$ ]] ||
     ((process_group <= 1)); then
    printf '错误：进程组记录非法，拒绝释放运行锁：%s (%s)\n' \
      "${process_group}" "${label}" >&2
    return 1
  fi
  if ! _process_group_is_alive "${process_group}"; then
    return 0
  fi

  if [[ -r "/proc/${process_group}/stat" ]]; then
    actual_ticks="$(_process_start_ticks "${process_group}")" || true
    if [[ -n "${actual_ticks}" &&
          "${actual_ticks}" != "${expected_ticks}" ]]; then
      printf '警告：PID 已被其他进程复用，拒绝清理：%s (%s)\n' \
        "${process_group}" "${label}" >&2
      return 1
    fi
  fi

  printf '停止受管进程组：%s (%s)\n' \
    "${process_group}" "${label}" >&2
  kill -TERM -- "-${process_group}" 2>/dev/null || true
  for ((attempt = 0; attempt < term_wait_steps; ++attempt)); do
    if ! _process_group_is_alive "${process_group}"; then
      break
    fi
    sleep 0.05
  done
  if _process_group_is_alive "${process_group}"; then
    printf '进程组未在限时内退出，执行强制清理：%s (%s)\n' \
      "${process_group}" "${label}" >&2
    kill -KILL -- "-${process_group}" 2>/dev/null || true
    for ((attempt = 0; attempt < 20; ++attempt)); do
      if ! _process_group_is_alive "${process_group}"; then
        break
      fi
      sleep 0.05
    done
  fi
  wait "${process_group}" 2>/dev/null || true
  if _process_group_is_alive "${process_group}"; then
    printf '错误：进程组清理后仍然存在：%s (%s)\n' \
      "${process_group}" "${label}" >&2
    return 1
  fi
}

cleanup_registered_process_groups() {
  local records=()
  local failed_records=()
  local index=0
  local process_group=""
  local start_ticks=""
  local term_timeout_s=""
  local label=""
  local cleanup_failed=0

  if [[ ! -f "${TELEOP_CHILDREN_FILE}" ]]; then
    return 0
  fi
  mapfile -t records < "${TELEOP_CHILDREN_FILE}"
  for ((index = ${#records[@]} - 1; index >= 0; --index)); do
    IFS=$'\t' read -r process_group start_ticks term_timeout_s label \
      <<<"${records[index]}"
    if [[ -z "${records[index]}" ]]; then
      continue
    fi
    if [[ -z "${process_group}" || -z "${start_ticks}" ]]; then
      printf '错误：受管进程记录不完整，拒绝释放运行锁：%s\n' \
        "${records[index]}" >&2
      cleanup_failed=1
      failed_records+=("${records[index]}")
      continue
    fi
    if ! _stop_recorded_process_group \
      "${process_group}" "${start_ticks}" \
      "${term_timeout_s:-5}" "${label:-unknown}"
    then
      cleanup_failed=1
      failed_records+=("${records[index]}")
    fi
  done
  : > "${TELEOP_CHILDREN_FILE}"
  for ((index = ${#failed_records[@]} - 1; index >= 0; --index)); do
    printf '%s\n' "${failed_records[index]}" >> "${TELEOP_CHILDREN_FILE}"
  done
  return "${cleanup_failed}"
}

register_teleop_process_group() {
  local process_group="$1"
  local label="$2"
  local term_timeout_s="${3:-5}"
  local start_ticks=""

  if [[ "${_TELEOP_GUARD_HELD}" != true ]]; then
    printf '%s\n' '错误：尚未取得遥操作运行锁，不能登记子进程。' >&2
    return 1
  fi
  start_ticks="$(_process_start_ticks "${process_group}")" || {
    printf '错误：无法登记进程组：%s (%s)\n' \
      "${process_group}" "${label}" >&2
    return 1
  }
  label="${label//$'\t'/ }"
  label="${label//$'\n'/ }"
  if [[ ! "${term_timeout_s}" =~ ^[0-9]+$ ]] ||
     ((term_timeout_s < 1 || term_timeout_s > 60)); then
    printf '错误：非法 TERM 等待时间：%s (%s)\n' \
      "${term_timeout_s}" "${label}" >&2
    return 1
  fi
  printf '%s\t%s\t%s\t%s\n' \
    "${process_group}" "${start_ticks}" "${term_timeout_s}" "${label}" \
    >> "${TELEOP_CHILDREN_FILE}"
}

_recover_or_reject_selected_guard() {
  local owner_pid=""
  local owner_ticks=""
  local owner_mode=""

  if [[ ! -d "${TELEOP_GUARD_DIR}" ]]; then
    return 0
  fi
  if [[ -r "${TELEOP_OWNER_FILE}" ]]; then
    IFS=$'\t' read -r owner_pid owner_ticks owner_mode \
      < "${TELEOP_OWNER_FILE}" || true
  fi
  if [[ -n "${owner_pid}" && -n "${owner_ticks}" ]] &&
     _same_process_is_alive "${owner_pid}" "${owner_ticks}"; then
    printf '%s\n' \
      "拒绝启动：同类受管任务已经运行。" \
      "  PID=${owner_pid} 模式=${owner_mode:-unknown}" \
      '请回到原终端按 Ctrl+C，等待清理完成后再启动。' >&2
    return 1
  fi

  printf '%s\n' \
    '检测到上次异常退出留下的受管进程，正在自动清理。' >&2
  if ! cleanup_registered_process_groups; then
    printf '%s\n' \
      '错误：遗留进程未能完全退出；保留记录并拒绝启动。' >&2
    return 1
  fi
  rm -f -- "${TELEOP_OWNER_FILE}" "${TELEOP_CHILDREN_FILE}"
  rmdir -- "${TELEOP_GUARD_DIR}" 2>/dev/null || true
}

acquire_teleop_guard() {
  local mode="$1"
  local current_ticks=""

  if [[ ! "${mode}" =~ ^[a-z][a-z0-9_-]*$ ]]; then
    printf '错误：非法运行锁模式：%s\n' "${mode}" >&2
    return 1
  fi
  mkdir -p -- "${TELEOP_RUNTIME_DIR}" "${TELEOP_GUARDS_DIR}"
  chmod 700 "${TELEOP_RUNTIME_DIR}" "${TELEOP_GUARDS_DIR}"
  current_ticks="$(_process_start_ticks "$$")"
  _lock_guard_administration

  # 兼容上一版单一全局守卫：活任务必须先安全退出，死任务自动回收。
  if [[ -d "${TELEOP_LEGACY_GUARD_DIR}" ]]; then
    _select_guard_directory "${TELEOP_LEGACY_GUARD_DIR}"
    if ! _recover_or_reject_selected_guard; then
      _unlock_guard_administration
      return 1
    fi
  fi

  _select_guard_directory "${TELEOP_GUARDS_DIR}/${mode}"
  if ! _recover_or_reject_selected_guard; then
    _unlock_guard_administration
    return 1
  fi

  if ! mkdir -- "${TELEOP_GUARD_DIR}"; then
    printf '%s\n' '错误：无法创建遥操作运行锁。' >&2
    _unlock_guard_administration
    return 1
  fi
  printf '%s\t%s\t%s\n' "$$" "${current_ticks}" "${mode}" \
    > "${TELEOP_OWNER_FILE}"
  : > "${TELEOP_CHILDREN_FILE}"
  _TELEOP_GUARD_HELD=true
  _unlock_guard_administration
}

release_teleop_guard() {
  if [[ "${_TELEOP_GUARD_HELD}" != true ]]; then
    return 0
  fi
  _lock_guard_administration
  rm -f -- "${TELEOP_OWNER_FILE}" "${TELEOP_CHILDREN_FILE}"
  rmdir -- "${TELEOP_GUARD_DIR}" 2>/dev/null || true
  _TELEOP_GUARD_HELD=false
  _unlock_guard_administration
}

teleop_cleanup_and_release() {
  trap - EXIT INT TERM
  if ! cleanup_registered_process_groups; then
    printf '%s\n' \
      '错误：仍有受管进程存活；保留运行锁和进程记录供下次恢复。' >&2
    return 1
  fi
  release_teleop_guard
}

_teleop_stop_on_signal() {
  teleop_cleanup_and_release || true
  exit 130
}

install_teleop_cleanup_traps() {
  trap teleop_cleanup_and_release EXIT
  trap _teleop_stop_on_signal INT TERM
}

find_conflicting_teleop_nodes() {
  local mode="${1:-all}"
  awk -v mode="${mode}" '
    function role_for(path, parts, n) {
      n = split(path, parts, "/")
      if (n < 5 || parts[1] != "tj" || parts[2] != "live") return ""
      if (parts[3] == "source" && n == 5) return "source"
      if (parts[3] == "producer" && n == 6 && (parts[4] == "arm" || parts[4] == "hand")) return "producer/" parts[4]
      if (parts[3] == "coordinator" && n == 6 && parts[4] == "arm") return "coordinator/arm"
      if (parts[3] == "executor" && n == 6 && (parts[4] == "arm" || parts[4] == "hand")) return "executor/" parts[4]
      if (parts[3] == "recorder" && n == 5) return "recorder"
      return ""
    }
    {
      role = role_for($0, fields)
      if (!role) next
      n = split($0, fields, "/")
      identity = role "/" fields[n]
      logical = role "/" fields[4]
      # One logical component may have only one live instance. Keep the
      # complete token in the diagnostic so callers cannot collapse domains.
      if (++instances[logical] > 1) {
        if (!reported[identity]++) print $0
      }
      if (mode == "real" && role == "executor/arm") {
        real_exec[identity]++
      }
    }
  '
}

assert_profile_domains_free() {
  local existing="$1"
  local token role
  while IFS= read -r token; do
    [[ -n "${token}" ]] || continue
    role="$(awk -F/ '{if ($3=="source") print "source"; else if (($3=="producer" || $3=="executor") && ($4=="arm" || $4=="hand")) print $3"/"$4; else if ($3=="coordinator" && $4=="arm") print "coordinator/arm"; else if ($3=="recorder") print "recorder"}' <<<"${token}")"
    case "${role}" in
      source|producer/arm|producer/hand|coordinator/arm|executor/arm|executor/hand)
        printf '拒绝启动：profile domain 已被 live token 占用：%s\n' "${token}" >&2
        return 1 ;;
    esac
  done <<<"${existing}"
}

read_router_zid() {
  local endpoint="${TIANJI_ROUTER_ENDPOINT:-tcp/127.0.0.1:7447}"
  TIANJI_ROUTER_ENDPOINT="${endpoint}" python - <<'PY'
import os
import zenoh

endpoint = os.environ["TIANJI_ROUTER_ENDPOINT"]
config = zenoh.Config.from_json5(
    '{"mode":"client","connect":{"endpoints":['
    + __import__("json").dumps(endpoint)
    + ']},"scouting":{"multicast":{"enabled":false}}}'
)
session = zenoh.open(config)
try:
    routers = [str(item) for item in session.info.routers_zid()]
    if len(routers) != 1 or not routers[0]:
        raise RuntimeError(f"expected exactly one router ZID, got {len(routers)}")
    print(routers[0])
finally:
    session.close()
PY
}

read_teleop_node_list() {
  local endpoint="${TIANJI_ROUTER_ENDPOINT:-tcp/127.0.0.1:7447}"
  if [[ -v PICO_TIANJI_NODE_LIST_OVERRIDE ]]; then
    printf '%s\n' "${PICO_TIANJI_NODE_LIST_OVERRIDE}"
    return 0
  fi
  TIANJI_ROUTER_ENDPOINT="${endpoint}" python - <<'PY'
import json
import os
import time
import zenoh

endpoint = os.environ["TIANJI_ROUTER_ENDPOINT"]
config = zenoh.Config.from_json5(
    '{"mode":"client","connect":{"endpoints":['
    + json.dumps(endpoint)
    + ']},"scouting":{"multicast":{"enabled":false}}}'
)
session = zenoh.open(config)
try:
    routers = [str(item) for item in session.info.routers_zid()]
    if len(routers) != 1 or not routers[0]:
        raise RuntimeError(f"expected exactly one router ZID, got {len(routers)}")
    names = set()
    for _ in range(3):
        for reply in session.liveliness().get("tj/live/**", timeout=1.0):
            if reply.ok:
                key = str(reply.result.key_expr)
                if key.startswith("tj/live/"):
                    names.add(key)
        time.sleep(0.05)
    for name in sorted(names):
        print(name)
finally:
    session.close()
PY
}

assert_no_conflicting_teleop_nodes() {
  local mode="${1:-all}"
  local node_list=""
  local conflicts=""
  if [[ "${mode}" == "real" && -v PICO_TIANJI_NODE_LIST_OVERRIDE ]]; then
    printf '%s\n' '拒绝连接真机：真机模式禁止覆盖 live token 列表。' >&2
    return 1
  fi
  if [[ "${mode}" == "real" && "${PICO_TIANJI_SKIP_ROS_CONFLICT_CHECK:-0}" == "1" ]]; then
    printf '%s\n' '拒绝连接真机：真机模式禁止跳过 live token 检查。' >&2
    return 1
  fi
  if [[ "${PICO_TIANJI_SKIP_ROS_CONFLICT_CHECK:-0}" == "1" ]]; then
    return 0
  fi
  if ! node_list="$(read_teleop_node_list)"; then
    printf '%s\n' '错误：无法检查遥操作 live token，拒绝启动。' >&2
    return 1
  fi
  conflicts="$(find_conflicting_teleop_nodes "${mode}" <<<"${node_list}")"
  if [[ -n "${conflicts}" ]]; then
    printf '%s\n' \
      '拒绝启动：同一 logical id 存在多个 live instance：' \
      "${conflicts}" >&2
    return 1
  fi
}

assert_managed_teleop_guard_alive() {
  local mode="$1"
  local guard_dir="${TELEOP_GUARDS_DIR}/${mode}"
  local owner_file="${guard_dir}/owner"
  local owner_pid=""
  local owner_ticks=""
  local owner_mode=""
  local owner_alive=false

  if [[ -d "${TELEOP_GUARDS_DIR}" ]]; then
    _lock_guard_administration
    if [[ -r "${owner_file}" ]]; then
      IFS=$'\t' read -r owner_pid owner_ticks owner_mode < "${owner_file}" || true
    fi
    if [[ "${owner_mode}" == "${mode}" && -n "${owner_pid}" && -n "${owner_ticks}" ]] &&
      _same_process_is_alive "${owner_pid}" "${owner_ticks}"; then
      owner_alive=true
    fi
    _unlock_guard_administration
  fi
  if [[ "${owner_alive}" != true ]]; then
    printf '%s\n' \
      "拒绝连接真机：未检测到新版受管 ${mode} 主机任务。" \
      "请先从同一版本运行对应 run_session profile。" >&2
    return 1
  fi
}


yaml_params_for() {
  # 用法：yaml_params_for <节点名> <yaml 路径> [urdf_path:=绝对路径 ...]
  # 输出该节点段的裸 key:=value 参数（每行一个，无 --param 前缀；
  # C++ 节点直接使用，Python 节点由调用方包装为 --param <key:=value>）。
  local node_name="$1"
  local yaml_path="$2"
  shift 2
  python - "$node_name" "$yaml_path" "$@" <<'PY' 2>/dev/null
import json
import sys

node_name, yaml_path = sys.argv[1], sys.argv[2]
extra = [arg for arg in sys.argv[3:] if ":=" in arg]

with open(yaml_path, encoding="utf-8") as fh:
    import yaml
    data = yaml.safe_load(fh) or {}
section = data.get(node_name, data)
if isinstance(section, dict) and "ros__parameters" in section:
    section = section["ros__parameters"]
for key, value in (section or {}).items():
    if isinstance(value, bool):
        encoded = "true" if value else "false"
    elif isinstance(value, (list, tuple)):
        encoded = json.dumps(list(value), separators=(",", ":"))
    else:
        encoded = str(value)
    print(f"{key}:={encoded}")
for arg in extra:
    print(arg)
PY
}

new_instance_id() {
  if command -v uuidgen >/dev/null 2>&1; then
    uuidgen
  else
    # uuid4 is generated by the launcher's Python runtime, not by a process
    # counter; every component receives a unique authority identity.
    pixi run python -c 'import uuid; print(uuid.uuid4())'
  fi
}

router_unavailable_message() {
  printf '%s\n' \
    'Zenoh router unavailable; set TIANJI_ROUTER_ENDPOINT and run /home/current/syz/mocap/acquisition: pixi run start-router' >&2
}

require_router() {
  local zid
  if ! zid="$(read_router_zid 2>/dev/null)"; then
    router_unavailable_message
    return 1
  fi
  [[ -n "${zid}" ]] || {
    router_unavailable_message
    return 1
  }
  printf '%s\n' "${zid}"
}

canonical_config() {
  local relative="$1"
  local candidate="${BUNDLE_ROOT}/src/pico_body_tianji/config/${relative}"
  if [[ ! -f "${candidate}" && -n "${PICO_BODY_TIANJI_BUNDLE_ROOT:-}" ]]; then
    candidate="${PICO_BODY_TIANJI_BUNDLE_ROOT}/runtime/pico_body_tianji/share/pico_body_tianji/config/${relative}"
  fi
  if [[ ! -f "${candidate}" ]]; then
    printf '错误：缺少 canonical config: %s\n' "${relative}" >&2
    return 1
  fi
  printf '%s\n' "${candidate}"
}

activate_bundle_runtime() {
  if [[ ! -d "${BUNDLE_ROOT}/vendor/python" ||
        ! -d "${ZENOH_LIBRARY_ROOT}" ]]; then
    printf '%s\n' \
      '错误：缺少随包运行环境（vendor/python、vendor/zenoh）。' >&2
    return 1
  fi
  if ! python -c 'import zenoh' 2>/dev/null; then
    printf '%s\n' \
      '错误：当前 Python 环境缺少 zenoh（请用 pixi run 执行）。' >&2
    return 1
  fi

  export PYTHONDONTWRITEBYTECODE=1
  export PICO_BODY_TIANJI_BUNDLE_ROOT="${BUNDLE_ROOT}"
  export PATH="${PROJECT_PREFIX}/lib/pico_body_tianji:${PATH}"
  export PYTHONPATH="${BUNDLE_ROOT}/src/pico_body_tianji:${BUNDLE_ROOT}/vendor/python:${PROJECT_PREFIX}/lib/python3.10/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
  conda_library_path=""
  if [[ -n "${CONDA_PREFIX:-}" ]]; then
    conda_library_path="${CONDA_PREFIX}/lib:"
  fi
  export LD_LIBRARY_PATH="${conda_library_path}${BUNDLE_ROOT}/vendor/lib:${ZENOH_LIBRARY_ROOT}:${PROJECT_PREFIX}/lib:${PIN_LIBRARY_ROOT}:${ABI_LIBRARY_ROOT}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
}
