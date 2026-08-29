#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

profile=""
record_path=""
input_path=""
confirm_real=false
extra_args=()
while (($#)); do
  case "$1" in
    --profile) profile="${2:-}"; shift 2 ;;
    --record) record_path="${2:-}"; shift 2 ;;
    --h5|--input) input_path="${2:-}"; shift 2 ;;
    --confirm-real) confirm_real=true; shift ;;
    --help|-h)
      printf '%s\n' '用法: run_session.sh --profile PROFILE [--record PATH] [--confirm-real] [--h5 PATH]'
      exit 0 ;;
    --) shift; extra_args+=("$@"); break ;;
    *)
      if [[ -z "${input_path}" && "$1" != -* ]]; then input_path="$1"; else extra_args+=("$1"); fi
      shift ;;
  esac
done
if [[ -z "${profile}" ]]; then
  printf '%s\n' '错误：必须指定 --profile。' >&2
  exit 2
fi
case "${profile}" in
  pico_sim|pico_real|mocap_live_sim|mocap_live_real|h5_sim|h5_real|target_replay_sim|joint_replay_sim|diagnostic_mocap_calibration_sim) ;;
  *) printf '错误：未知 session profile: %s\n' "${profile}" >&2; exit 2 ;;
esac
if [[ "${profile}" == target_replay_sim || "${profile}" == joint_replay_sim ]]; then
  if [[ -n "${record_path}" ]]; then
    printf '%s\n' 'replay profile cannot be recorded' >&2
    exit 2
  fi
fi
if [[ "${profile}" == diagnostic_mocap_calibration_sim && -n "${record_path}" ]]; then
  printf '%s\n' 'diagnostic profile cannot be recorded: no session raw schema' >&2
  exit 2
fi
profile_config="$(canonical_config "sessions/${profile}.yaml")"
profile_value() {
  local key="$1"
  pixi run python - "${profile_config}" "${key}" <<'PY'
import sys
import yaml
value = yaml.safe_load(open(sys.argv[1], encoding="utf-8")) or {}
result = value.get(sys.argv[2])
if result is None:
    print("")
elif isinstance(result, (list, tuple)):
    print(",".join(str(item) for item in result))
elif isinstance(result, bool):
    print("true" if result else "false")
else:
    print(str(result))
PY
}
source_config="$(profile_value source_config)"
arm_producer_config="$(profile_value arm_producer_config)"
arm_executor_config="$(profile_value arm_executor_config)"
coordinator_config="$(profile_value coordinator_config)"
required_capability="$(profile_value required_capability)"
active_sides="$(profile_value active_sides)"
inactive_sides="$(profile_value inactive_sides)"
hand_mode="$(profile_value hand_mode)"
[[ -n "${source_config}" && -n "${arm_executor_config}" && -n "${coordinator_config}" ]] || {
  printf '%s\n' '错误：session profile 缺少 source/executor/coordinator config。' >&2
  exit 2
}
if [[ "${required_capability}" == real && "${confirm_real}" != true ]]; then
  printf '%s\n' '错误：real profile 必须显式提供 --confirm-real。' >&2
  exit 2
fi
if [[ -n "${record_path}" ]]; then
  if [[ -e "${record_path}" ]]; then
    printf '错误：拒绝覆盖已有 recording: %s\n' "${record_path}" >&2
    exit 2
  fi
  mkdir -p -- "$(dirname -- "${record_path}")"
fi
source_name="$(basename -- "${source_config}" .yaml)"
case "${source_name}" in
  pico_controller|mocap_live|h5_replay|target|joint) ;;
  mocap_calibration) ;;
  *) printf '错误：source config 不在 canonical source/replay/diagnostic 树: %s\n' "${source_config}" >&2; exit 2 ;;
esac
case "${source_name}" in
  target) source_id=target_replay ;;
  joint) source_id=joint_replay ;;
  *) source_id="${source_name}" ;;
esac
if [[ "${source_id}" == h5_replay ]]; then
  [[ -n "${input_path}" && -f "${input_path}" ]] || {
    printf '%s\n' '错误：H5 profile 需要 --h5 PATH 或位置参数。' >&2
    exit 2
  }
  if [[ "${hand_mode}" == auto ]]; then
    if pixi run python - "${input_path}" <<'PY'
import sys
import h5py
with h5py.File(sys.argv[1], "r") as handle:
    found = any("wuji2_joints" in str(key) for key in handle.keys())
    def visit(name, obj):
        nonlocal_found[0] |= "wuji2_joints" in name
    nonlocal_found = [found]
    handle.visititems(visit)
    raise SystemExit(0 if nonlocal_found[0] else 1)
PY
    then hand_mode=direct
    else hand_mode=retarget
    fi
  fi
fi
if [[ "${hand_mode}" == auto ]]; then hand_mode=retarget; fi
run_id="$(new_instance_id)"
coordinator_id="$(new_instance_id)"
source_instance="$(new_instance_id)"
arm_producer_instance=""
[[ -n "${arm_producer_config}" && "${arm_producer_config}" != null ]] && arm_producer_instance="$(new_instance_id)"
arm_executor_instance="$(new_instance_id)"
if [[ "${source_id}" == target_replay || "${source_id}" == joint_replay ]]; then
  [[ -n "${input_path}" && -f "${input_path}" ]] || {
    printf '%s\n' '错误：replay profile 需要 session HDF5 位置参数或 --input PATH。' >&2
    exit 2
  }
fi
hand_producer_instance=""
hand_executor_instance=""
if [[ "${hand_mode}" != disabled ]]; then
  hand_producer_instance="$(new_instance_id)"
  hand_executor_instance="$(new_instance_id)"
fi
export TIANJI_RUN_ID="${run_id}"
export TIANJI_ROUTER_ENDPOINT="${TIANJI_ROUTER_ENDPOINT:-tcp/127.0.0.1:7447}"
if [[ "${required_capability}" == real && ( "${source_id}" == h5_replay || "${source_id}" == mocap_live ) ]]; then
  export TIANJI_REAL_SPEED="0.25"
  export TIANJI_REAL_YAW_DEG="0"
fi
if ! router_zid="$(require_router)"; then
  exit 1
fi
export TIANJI_ROUTER_ZID="${router_zid}"
export TIANJI_ACTIVE_SIDES="${active_sides}"
export TIANJI_ACTIVE_HAND_SIDES="${active_sides}"
export TIANJI_INACTIVE_HAND_SIDES="${inactive_sides}"
export TIANJI_ARM_PRODUCER_LOGICAL_ID="arm_ik_producer"
export TIANJI_ARM_PRODUCER_INSTANCE_ID="${arm_producer_instance}"
hand_producer_id="wuji_retarget"
[[ "${source_id}" == h5_replay && "${hand_mode}" == direct ]] && hand_producer_id="h5_direct"
[[ "${source_id}" == joint_replay ]] && hand_producer_id="joint_replay"
export TIANJI_HAND_PRODUCER_ID="${hand_producer_id}"
if [[ "${source_id}" == h5_replay && "${hand_mode}" == direct ]]; then
  hand_producer_instance="${source_instance}"
else
  hand_producer_instance="${hand_producer_instance}"
fi
export TIANJI_HAND_PRODUCER_INSTANCE_ID="${hand_producer_instance}"
export TIANJI_SOURCE_INSTANCE_ID="${source_instance}"
activate_bundle_runtime
if ! existing_tokens="$(read_teleop_node_list)"; then
  printf '%s\n' '错误：无法完成启动前 live domain preflight。' >&2
  exit 1
fi
if [[ -n "${existing_tokens}" ]]; then
  assert_profile_domains_free "${existing_tokens}"
fi
mode="simulation"
[[ "${required_capability}" == real ]] && mode=real
acquire_teleop_guard "${profile}"
install_teleop_cleanup_traps
launch() {
  local label="$1"; shift
  setsid env "$@" >"${TELEOP_RUNTIME_DIR}/${run_id}-${label}.log" 2>&1 &
  local pid=$!
  register_teleop_process_group "${pid}" "${label}" 5
  for _ in {1..20}; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      printf '错误：组件 %s 在启动阶段退出。\n' "${label}" >&2
      return 1
    fi
    sleep 0.1
  done
}
base_env=("TIANJI_ROUTER_ENDPOINT=${TIANJI_ROUTER_ENDPOINT}" "TIANJI_ROUTER_ZID=${TIANJI_ROUTER_ZID}" "TIANJI_COORDINATOR_INSTANCE_ID=${coordinator_id}" "TIANJI_RUN_ID=${run_id}" "TIANJI_REQUIRED_CAPABILITY=${required_capability}" "TIANJI_HAND_MODE=${hand_mode}" "TIANJI_HAND_PRODUCER_ID=${hand_producer_id}" "TIANJI_HAND_PRODUCER_INSTANCE_ID=${hand_producer_instance}" "TIANJI_ARM_PRODUCER_INSTANCE_ID=${arm_producer_instance}")
if [[ -n "${record_path}" ]]; then
  launch recorder "${base_env[@]}" TIANJI_COMPONENT_INSTANCE_ID="$(new_instance_id)" TIANJI_RECORD_PATH="${record_path}" TIANJI_RECORD_SOURCE_TYPE="${source_id}" python -m pico_body_tianji.recording.session_recorder
fi
launch coordinator "${base_env[@]}" TIANJI_COORDINATOR_INSTANCE_ID="${coordinator_id}" TIANJI_COORDINATOR_CONFIG="$(canonical_config "${coordinator_config}")" python "${BUNDLE_ROOT}/src/pico_body_tianji/scripts/arm_command_coordinator"
source_args=("${base_env[@]}" TIANJI_COMPONENT_INSTANCE_ID="${source_instance}" TIANJI_SOURCE_INSTANCE_ID="${source_instance}" TIANJI_PRODUCER_INSTANCE_ID="${hand_producer_instance}" bash "${SCRIPT_DIR}/run_source.sh" --source "${source_id}" --config "$(canonical_config "${source_config}")")
if [[ "${source_id}" == h5_replay ]]; then
  source_args+=(-- "${input_path}")
  if [[ "${required_capability}" == real ]]; then source_args+=(--speed 0.25 --yaw-deg 0); fi
elif [[ "${source_id}" == mocap_live && "${required_capability}" == real ]]; then
  source_args+=(--param speed:=0.25 --param yaw_deg:=0)
elif [[ "${source_id}" == target_replay || "${source_id}" == joint_replay ]]; then
  source_args+=(-- "${input_path}" --active-hand-sides "${active_sides}" --inactive-hand-sides "${inactive_sides}")
fi
source_args+=("${extra_args[@]}")
launch_arm_executor() {
  if [[ "${arm_executor_config}" == executors/mujoco.yaml ]]; then
    launch arm_executor "${base_env[@]}" TIANJI_COMPONENT_INSTANCE_ID="${arm_executor_instance}" bash "${SCRIPT_DIR}/run_executor.sh" --executor mujoco --config "$(canonical_config "${arm_executor_config}")"
  else
    launch arm_executor "${base_env[@]}" TIANJI_COMPONENT_INSTANCE_ID="${arm_executor_instance}" bash "${SCRIPT_DIR}/run_executor.sh" --executor marvin --config "$(canonical_config "${arm_executor_config}")" --confirm-real
  fi
}
launch_hand_executor() {
  [[ "${hand_mode}" != disabled ]] || return 0
  launch hand_executor "${base_env[@]}" TIANJI_COMPONENT_INSTANCE_ID="${hand_executor_instance}" bash "${SCRIPT_DIR}/run_executor.sh" --executor wuji_hand2 --mode "${hand_mode}" --side right --config "$(canonical_config executors/wuji_hand2.yaml)"
}
launch_arm_producer() {
  [[ -n "${arm_producer_config}" && "${arm_producer_config}" != null ]] || return 0
  producer_name="$(basename -- "${arm_producer_config}" .yaml)"
  [[ "${producer_name}" == ik ]] && producer_name=ik
  launch arm_producer "${base_env[@]}" TIANJI_COMPONENT_INSTANCE_ID="${arm_producer_instance}" bash "${SCRIPT_DIR}/run_producer.sh" --producer "${producer_name}" --config "$(canonical_config "${arm_producer_config}")"
}
if [[ "${required_capability}" == real ]]; then
  launch_arm_producer
  launch source "${source_args[@]}"
  launch_arm_executor
  launch_hand_executor
else
  launch_arm_executor
  launch_hand_executor
  launch_arm_producer
  launch source "${source_args[@]}"
fi
printf '%s\n' "session ${profile} started; router_zid=${router_zid}; run_id=${run_id}"
wait
