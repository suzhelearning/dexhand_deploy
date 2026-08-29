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
active_hand_sides="$(profile_value active_hand_sides)"
hand_executor="$(profile_value hand_executor)"
hand_overlay="$(profile_value hand_overlay)"
if [[ "${TIANJI_VALIDATION_PRODUCER:-}" == policy_hold ]]; then
  arm_producer_config="producers/policy_hold.yaml"
fi
[[ -n "${active_hand_sides}" ]] || active_hand_sides="${active_sides}"
[[ -n "${hand_executor}" ]] || hand_executor=none
[[ -n "${hand_overlay}" ]] || hand_overlay=none
[[ -n "${source_config}" && -n "${arm_executor_config}" && -n "${coordinator_config}" ]] || {
  printf '%s\n' '错误：session profile 缺少 source/executor/coordinator config。' >&2
  exit 2
}
if [[ "${hand_mode}" == disabled ]]; then
  hand_executor=none
  active_hand_sides=""
elif [[ "${hand_executor}" != wuji_hand2 && "${hand_executor}" != mujoco ]]; then
  printf '错误：hand-enabled profile 必须选择唯一 hand_executor: %s\n' "${hand_executor}" >&2
  exit 2
fi
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
run_id="${TIANJI_RUN_ID:-$(new_instance_id)}"
coordinator_id="${TIANJI_COORDINATOR_INSTANCE_ID:-$(new_instance_id)}"
source_instance="${TIANJI_SOURCE_INSTANCE_ID:-$(new_instance_id)}"
arm_producer_instance=""
arm_producer_id="arm_ik_producer"
if [[ "${source_id}" == joint_replay ]]; then
  arm_producer_instance="${TIANJI_ARM_PRODUCER_INSTANCE_ID:-$(new_instance_id)}"
  arm_producer_id="joint_replay"
elif [[ -n "${arm_producer_config}" && "${arm_producer_config}" != null ]]; then
  arm_producer_instance="${TIANJI_ARM_PRODUCER_INSTANCE_ID:-$(new_instance_id)}"
fi
arm_executor_instance="${TIANJI_ARM_EXECUTOR_INSTANCE_ID:-$(new_instance_id)}"
if [[ "${source_id}" == target_replay || "${source_id}" == joint_replay ]]; then
  [[ -n "${input_path}" && -f "${input_path}" ]] || {
    printf '%s\n' '错误：replay profile 需要 session HDF5 位置参数或 --input PATH。' >&2
    exit 2
  }
fi
declare -a hand_side_array=()
declare -a hand_producer_id_array=()
declare -a hand_producer_instance_array=()
declare -a hand_executor_instance_array=()
lookup_instance() {
  local mapping="$1"
  local wanted_side="$2"
  local pair=""
  local value=""
  IFS=',' read -r -a mapping_pairs <<< "${mapping}"
  for pair in "${mapping_pairs[@]}"; do
    if [[ "${pair%%=*}" == "${wanted_side}" ]]; then
      value="${pair#*=}"
      [[ -n "${value}" ]] && printf '%s\n' "${value}"
      return 0
    fi
  done
  return 1
}
if [[ -n "${active_hand_sides}" ]]; then
  IFS=',' read -r -a hand_side_array <<< "${active_hand_sides}"
  for hand_side in "${hand_side_array[@]}"; do
    [[ "${hand_side}" == left || "${hand_side}" == right ]] || {
      printf '错误：active_hand_sides 包含非法 side: %s\n' "${hand_side}" >&2
      exit 2
    }
    mapped_hand_executor=""
    mapped_hand_executor="$(lookup_instance "${TIANJI_HAND_EXECUTOR_INSTANCES:-}" "${hand_side}" || true)"
    hand_executor_instance_array+=("${mapped_hand_executor:-$(new_instance_id)}")
    if [[ "${source_id}" == h5_replay && "${hand_mode}" == direct ]]; then
      hand_producer_id_array+=("h5_direct")
      hand_producer_instance_array+=("${source_instance}")
    elif [[ "${source_id}" == joint_replay ]]; then
      hand_producer_id_array+=("joint_replay")
      hand_producer_instance_array+=("${arm_producer_instance}")
    elif [[ "${hand_executor}" == wuji_hand2 ]]; then
      hand_producer_id_array+=("wuji_retarget_${hand_side}")
      mapped_hand_producer=""
      mapped_hand_producer="$(lookup_instance "${TIANJI_HAND_PRODUCER_INSTANCES:-}" "${hand_side}" || true)"
      hand_producer_instance_array+=("${mapped_hand_producer:-$(new_instance_id)}")
    else
      hand_producer_id_array+=("disabled")
      hand_producer_instance_array+=("disabled")
    fi
  done
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
export TIANJI_ACTIVE_HAND_SIDES="${active_hand_sides}"
export TIANJI_INACTIVE_HAND_SIDES="${inactive_sides}"
export TIANJI_ARM_PRODUCER_LOGICAL_ID="${arm_producer_id}"
export TIANJI_ARM_PRODUCER_INSTANCE_ID="${arm_producer_instance}"
hand_producer_id="disabled"
hand_producer_instance="disabled"
hand_input_instance="disabled"
if ((${#hand_side_array[@]} > 0)); then
  hand_producer_id="${hand_producer_id_array[0]}"
  hand_producer_instance="${hand_producer_instance_array[0]}"
  if [[ "${hand_mode}" == retarget ]]; then
    hand_input_instance="${source_instance}"
  else
    hand_input_instance="${hand_producer_instance}"
  fi
fi
export TIANJI_HAND_PRODUCER_ID="${hand_producer_id}"
export TIANJI_HAND_PRODUCER_INSTANCE_ID="${hand_producer_instance}"
export TIANJI_HAND_INPUT_INSTANCE_ID="${hand_input_instance}"
export TIANJI_SOURCE_INSTANCE_ID="${source_instance}"
hand_authority_rows=""
for hand_index in "${!hand_side_array[@]}"; do
  hand_authority_rows+="${hand_side_array[hand_index]}|${hand_producer_id_array[hand_index]}|${hand_producer_instance_array[hand_index]}|${hand_executor_instance_array[hand_index]},"
done
authorities_json="$(
  pixi run python - \
    "${source_id}" "${source_instance}" "${arm_producer_id}" "${arm_producer_instance}" \
    "${arm_executor_config##*/}" "${arm_executor_instance}" "${coordinator_id}" \
    "${router_zid}" "${hand_authority_rows}" <<'PY'
import json
import sys

source, source_instance, arm_producer, arm_producer_instance, executor_config, executor_instance, coordinator, router, rows = sys.argv[1:]
executor = "marvin" if executor_config == "marvin.yaml" else "mujoco"
disabled = {"logical_id": "disabled", "publisher_instance_id": "disabled", "router_zid": router, "enabled": False}
hand_producers = {"left": dict(disabled), "right": dict(disabled)}
hand_executors = {"left": dict(disabled), "right": dict(disabled)}
for row in rows.split(","):
    if not row:
        continue
    side, producer, producer_instance, executor_instance = row.split("|")
    hand_producers[side] = {"logical_id": producer, "publisher_instance_id": producer_instance, "router_zid": router}
    hand_executors[side] = {"logical_id": f"wuji_{side}", "publisher_instance_id": executor_instance, "router_zid": router}
print(json.dumps({
    "source": {"logical_id": source, "publisher_instance_id": source_instance, "router_zid": router},
    "producer_arm": {"logical_id": arm_producer, "publisher_instance_id": arm_producer_instance or "disabled", "router_zid": router, "enabled": bool(arm_producer_instance)},
    "producer_hand": hand_producers,
    "coordinator_arm": {"logical_id": "arm", "publisher_instance_id": coordinator, "router_zid": router},
    "executor_arm": {"logical_id": executor, "publisher_instance_id": executor_instance, "router_zid": router},
    "executor_hand": hand_executors,
}, separators=(",", ":")))
PY
)"
[[ -n "${authorities_json}" ]] || { printf '%s\n' '错误：无法构造完整 authority mapping。' >&2; exit 1; }
export TIANJI_AUTHORITIES="${authorities_json}"
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
recorder_instance=""
[[ -n "${record_path}" ]] && recorder_instance="${TIANJI_RECORDER_INSTANCE_ID:-$(new_instance_id)}"
base_env=(
  "TIANJI_ROUTER_ENDPOINT=${TIANJI_ROUTER_ENDPOINT}"
  "TIANJI_ROUTER_ZID=${TIANJI_ROUTER_ZID}"
  "TIANJI_COORDINATOR_INSTANCE_ID=${coordinator_id}"
  "TIANJI_SAFETY_SUPERVISOR_INSTANCE_ID=${TIANJI_SAFETY_SUPERVISOR_INSTANCE_ID:-}"
  "TIANJI_RUN_ID=${run_id}"
  "TIANJI_REQUIRED_CAPABILITY=${required_capability}"
  "TIANJI_HAND_MODE=${hand_mode}"
  "TIANJI_HAND_PRODUCER_ID=${hand_producer_id}"
  "TIANJI_HAND_PRODUCER_INSTANCE_ID=${hand_producer_instance}"
  "TIANJI_HAND_INPUT_INSTANCE_ID=${hand_input_instance}"
  "TIANJI_ARM_PRODUCER_INSTANCE_ID=${arm_producer_instance}"
  "TIANJI_AUTHORITIES=${TIANJI_AUTHORITIES}"
)
if [[ -n "${record_path}" ]]; then
  launch recorder "${base_env[@]}" TIANJI_COMPONENT_INSTANCE_ID="${recorder_instance}" TIANJI_RECORD_PATH="${record_path}" TIANJI_RECORD_SOURCE_TYPE="${source_id}" TIANJI_RECORDING_CONFIG="$(canonical_config recording/session.yaml)" python -m pico_body_tianji.recording.session_recorder
fi
launch coordinator "${base_env[@]}" TIANJI_COORDINATOR_INSTANCE_ID="${coordinator_id}" TIANJI_COORDINATOR_CONFIG="$(canonical_config "${coordinator_config}")" python "${BUNDLE_ROOT}/src/pico_body_tianji/scripts/arm_command_coordinator"
source_args=("${base_env[@]}" TIANJI_COMPONENT_INSTANCE_ID="${source_instance}" TIANJI_SOURCE_INSTANCE_ID="${source_instance}" TIANJI_PRODUCER_INSTANCE_ID="${arm_producer_instance:-${hand_producer_instance}}" bash "${SCRIPT_DIR}/run_source.sh" --source "${source_id}" --config "$(canonical_config "${source_config}")")
if [[ "${source_id}" == h5_replay ]]; then
  source_args+=(-- "${input_path}")
  if [[ "${required_capability}" == real ]]; then source_args+=(--speed 0.25 --yaw-deg 0); fi
elif [[ "${source_id}" == mocap_live && "${required_capability}" == real ]]; then
  source_args+=(--param speed:=0.25 --param yaw_deg:=0)
elif [[ "${source_id}" == target_replay || "${source_id}" == joint_replay ]]; then
  source_args+=(-- "${input_path}" --active-hand-sides "${active_hand_sides}" --inactive-hand-sides "${inactive_sides}")
fi
source_args+=("${extra_args[@]}")
launch_arm_executor() {
  local hand_args=()
  if [[ "${hand_overlay}" == mujoco && -n "${active_hand_sides}" ]]; then
    hand_args+=(--hand-sides "${active_hand_sides}" --hand-overlay)
  else
    # Wuji is the sole hand executor authority for hand-enabled sim/replay.
    hand_args+=(--hand-sides "")
  fi
  if [[ "${arm_executor_config}" == executors/mujoco.yaml ]]; then
    launch arm_executor "${base_env[@]}" TIANJI_COMPONENT_INSTANCE_ID="${arm_executor_instance}" bash "${SCRIPT_DIR}/run_executor.sh" --executor mujoco --config "$(canonical_config "${arm_executor_config}")" "${hand_args[@]}"
  else
    launch arm_executor "${base_env[@]}" TIANJI_COMPONENT_INSTANCE_ID="${arm_executor_instance}" bash "${SCRIPT_DIR}/run_executor.sh" --executor marvin --config "$(canonical_config "${arm_executor_config}")" --confirm-real
  fi
}
launch_hand_executor() {
  [[ "${hand_mode}" != disabled && "${hand_executor}" == wuji_hand2 ]] || return 0
  for hand_index in "${!hand_side_array[@]}"; do
    launch "hand_executor_${hand_side_array[hand_index]}" "${base_env[@]}" \
      TIANJI_COMPONENT_INSTANCE_ID="${hand_executor_instance_array[hand_index]}" \
      TIANJI_HAND_PRODUCER_ID="${hand_producer_id_array[hand_index]}" \
      TIANJI_HAND_PRODUCER_INSTANCE_ID="${hand_producer_instance_array[hand_index]}" \
      TIANJI_HAND_INPUT_INSTANCE_ID="${hand_input_instance}" \
      bash "${SCRIPT_DIR}/run_executor.sh" --executor wuji_hand2 --mode "${hand_mode}" \
      --side "${hand_side_array[hand_index]}" --config "$(canonical_config executors/wuji_hand2.yaml)"
  done
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
