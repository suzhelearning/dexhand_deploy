#!/usr/bin/env bash
set -euo pipefail

BUNDLE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_ROOT="${BUNDLE_ROOT}/runtime/ros/humble"
PROJECT_PREFIX="${BUNDLE_ROOT}/runtime/pico_body_tianji"
ABI_LIBRARY_ROOT="${BUNDLE_ROOT}/runtime/abi/lib"
PIN_LIBRARY_ROOT="${BUNDLE_ROOT}/runtime/pin/lib"
GUI_LIBRARY_ROOT="${BUNDLE_ROOT}/runtime/gui/lib"
QT_PLUGIN_ROOT="${BUNDLE_ROOT}/runtime/gui/qt/plugins"
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
    {
      host_node = ($0 ~ /^\/(pico_controller_input|pico_controller_only_input|tianji_kinematic_sim|pico_body_sim\/marvin_robot_state_publisher)$/)
      output_node = ($0 ~ /^\/(marvin_hardware_bridge|tianji_world_output_node|tianji_arm_node)$/)
      if ((mode != "real" && host_node) || output_node) {
        if (!seen[$0]++) {
          print $0
        }
      }
    }
  '
}

read_teleop_node_list() {
  if [[ "${PICO_TIANJI_SKIP_ROS_CONFLICT_CHECK:-0}" == "1" ]]; then
    return 0
  fi
  if [[ -v PICO_TIANJI_NODE_LIST_OVERRIDE ]]; then
    printf '%s\n' "${PICO_TIANJI_NODE_LIST_OVERRIDE}"
    return 0
  fi
  timeout 4 python "${ROS_ROOT}/bin/ros2" node list \
    --no-daemon 2>/dev/null
}

assert_no_conflicting_teleop_nodes() {
  local mode="${1:-all}"
  local node_list=""
  local conflicts=""
  if [[ "${PICO_TIANJI_SKIP_ROS_CONFLICT_CHECK:-0}" == "1" ]]; then
    return 0
  fi
  if ! node_list="$(read_teleop_node_list)"; then
    printf '%s\n' \
      '错误：无法检查 ROS 图中的旧控制节点，拒绝启动。' >&2
    return 1
  fi
  conflicts="$(find_conflicting_teleop_nodes "${mode}" <<<"${node_list}")"
  if [[ -n "${conflicts}" ]]; then
    printf '%s\n' \
      '拒绝启动：检测到不受本次任务管理的旧控制节点：' \
      "${conflicts}" \
      '请先关闭旧终端、Docker 预览及其他控制程序。' >&2
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
      IFS=$'\t' read -r owner_pid owner_ticks owner_mode \
        < "${owner_file}" || true
    fi
    if [[ "${owner_mode}" == "${mode}" &&
          -n "${owner_pid}" &&
          -n "${owner_ticks}" ]] &&
       _same_process_is_alive "${owner_pid}" "${owner_ticks}"
    then
      owner_alive=true
    fi
    _unlock_guard_administration
  fi

  if [[ "${owner_alive}" != true ]]; then
    local host_command="pixi run sim"
    if [[ "${mode}" == "controller-only-simulation" ]]; then
      host_command="pixi run sim_controller_only"
    elif [[ "${mode}" == "mocap-replay" ]]; then
      host_command="pixi run sim_mocap -- TAKE.h5"
    fi
    printf '%s\n' \
      "拒绝连接真机：未检测到新版受管 ${mode} 主机任务。" \
      "请从同一新版解压包先运行 ${host_command}。" >&2
    return 1
  fi
}

assert_single_simulation_host_chain() {
  local node_list=""
  local pico_count=0
  local ik_count=0
  if [[ -v PICO_TIANJI_NODE_LIST_OVERRIDE ]]; then
    printf '%s\n' \
      '拒绝连接真机：真机模式禁止覆盖 ROS 节点列表。' >&2
    return 1
  fi
  if [[ "${PICO_TIANJI_SKIP_ROS_CONFLICT_CHECK:-0}" == "1" ]]; then
    printf '%s\n' \
      '拒绝连接真机：真机模式禁止跳过主机 ROS 链路检查。' >&2
    return 1
  fi
  assert_managed_teleop_guard_alive simulation
  if ! node_list="$(read_teleop_node_list)"; then
    printf '%s\n' \
      '错误：无法检查仿真主机链路，拒绝连接真机。' >&2
    return 1
  fi
  pico_count="$(
    awk '$0 == "/pico_controller_input" {count++} END {print count + 0}' \
      <<<"${node_list}"
  )"
  ik_count="$(
    awk '$0 == "/tianji_kinematic_sim" {count++} END {print count + 0}' \
      <<<"${node_list}"
  )"
  if ((pico_count != 1 || ik_count != 1)); then
    printf '%s\n' \
      '拒绝连接真机：主机侧必须恰好运行一套 PICO + IK。' \
      "  当前计数：PICO=${pico_count} IK=${ik_count}" \
      '请先运行 pixi run sim，并关闭其他旧仿真/遥操作任务。' >&2
    return 1
  fi
}

assert_single_controller_only_simulation_host_chain() {
  local node_list=""
  local controller_only_count=0
  local mocap_replay_count=0
  local smpl_count=0
  local ik_count=0
  if [[ -v PICO_TIANJI_NODE_LIST_OVERRIDE ]]; then
    printf '%s\n' \
      '拒绝连接真机：真机模式禁止覆盖 ROS 节点列表。' >&2
    return 1
  fi
  if [[ "${PICO_TIANJI_SKIP_ROS_CONFLICT_CHECK:-0}" == "1" ]]; then
    printf '%s\n' \
      '拒绝连接真机：真机模式禁止跳过主机 ROS 链路检查。' >&2
    return 1
  fi
  if ! node_list="$(read_teleop_node_list)"; then
    printf '%s\n' \
      '错误：无法检查纯手柄仿真主机链路，拒绝连接真机。' >&2
    return 1
  fi
  mocap_replay_count="$(
    awk '$0 == "/mocap_h5_replay" {count++} END {print count + 0}' \
      <<<"${node_list}"
  )"
  controller_only_count="$(
    awk '$0 == "/pico_controller_only_input" {count++} END {print count + 0}' \
      <<<"${node_list}"
  )"
  smpl_count="$(
    awk '$0 == "/pico_controller_input" {count++} END {print count + 0}' \
      <<<"${node_list}"
  )"
  ik_count="$(
    awk '$0 == "/tianji_kinematic_sim" {count++} END {print count + 0}' \
      <<<"${node_list}"
  )"
  if ((mocap_replay_count == 1)); then
    # mocap HDF5 确定性轨迹回放主机（真机 50mm 位移验收）：
    # 输入身份为 /mocap_h5_replay，运行锁为 mocap-replay。
    assert_managed_teleop_guard_alive mocap-replay
    if ((controller_only_count != 0 || smpl_count != 0 || ik_count != 1)); then
      printf '%s\n' \
        '拒绝连接真机：mocap 回放主机必须恰好运行一套回放 + IK。' \
        "  当前计数：回放=${mocap_replay_count} 纯手柄=${controller_only_count} SMPL=${smpl_count} IK=${ik_count}" \
        '请先运行 pixi run sim_mocap -- TAKE.h5，并关闭其他仿真任务。' >&2
      return 1
    fi
    return 0
  fi
  assert_managed_teleop_guard_alive controller-only-simulation
  if ((controller_only_count != 1 || smpl_count != 0 || ik_count != 1)); then
    printf '%s\n' \
      '拒绝连接真机：主机侧必须恰好运行一套纯手柄 + IK。' \
      "  当前计数：纯手柄=${controller_only_count} SMPL=${smpl_count} IK=${ik_count}" \
      '请先运行 pixi run sim_controller_only，并关闭其他仿真任务。' >&2
    return 1
  fi
}

activate_bundle_runtime() {
  if [[ ! -f \
    "${ROS_ROOT}/local/lib/python3.10/dist-packages/rclpy/__init__.py" ]]
  then
    printf '错误：缺少随包 ROS 2 运行时：%s\n' "${ROS_ROOT}" >&2
    return 1
  fi

  ros_library_path=""
  if [[ -d "${ROS_ROOT}/lib/x86_64-linux-gnu" ]]; then
    ros_library_path="${ROS_ROOT}/lib/x86_64-linux-gnu"
  fi
  while IFS= read -r -d '' library_dir; do
    ros_library_path+="${ros_library_path:+:}${library_dir}"
  done < <(find "${ROS_ROOT}" -type d -name lib -print0)

  export AMENT_PREFIX_PATH="${PROJECT_PREFIX}:${ROS_ROOT}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
  export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
  export PYTHONDONTWRITEBYTECODE=1
  export PATH="${PROJECT_PREFIX}/lib/pico_body_tianji:${ROS_ROOT}/bin:${PATH}"
  export PYTHONPATH="${BUNDLE_ROOT}/src/pico_body_tianji:${BUNDLE_ROOT}/vendor/python:${ROS_ROOT}/local/lib/python3.10/dist-packages:${ROS_ROOT}/lib/python3.10/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
  conda_library_path=""
  if [[ -n "${CONDA_PREFIX:-}" ]]; then
    conda_library_path="${CONDA_PREFIX}/lib:"
  fi
  export LD_LIBRARY_PATH="${conda_library_path}${BUNDLE_ROOT}/vendor/lib:${PROJECT_PREFIX}/lib:${GUI_LIBRARY_ROOT}:${ros_library_path}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  export QT_PLUGIN_PATH="${QT_PLUGIN_ROOT}${QT_PLUGIN_PATH:+:${QT_PLUGIN_PATH}}"
  export QT_QPA_PLATFORM_PLUGIN_PATH="${QT_PLUGIN_ROOT}/platforms"
  export QT_X11_NO_MITSHM=1
}
