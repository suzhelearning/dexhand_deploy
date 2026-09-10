#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

profile=""
resolve_only=false
record_path=""
input_path=""
confirm_real=false
display_mode=""
playback_speed=""
observation_config_override=""
disable_hands=false
pico_overlay=false
xr_overlay=false
ik_target_overlay=false
ik_backend_override=""
joint_trajectory_override=""
command_clipping_override=""
target_processor_override=""
pose_mapper_override=""
joint_limit_source=""
operator_input_override=""
arm_input_override=""
xr_manus_rawviz_override=""
xr_manus_user_override=""
xr_manus_library_dir_override=""
xr_manus_right_glove_override=""
xr_manus_left_glove_override=""
xr_sdk_pythonpath_override=""
extra_args=()
dual_runtime_args=()
while (($#)); do
  case "$1" in
    --profile) profile="${2:-}"; shift 2 ;;
    --resolve-only) resolve_only=true; shift ;;
    --disable-hands) disable_hands=true; shift ;;
    --spark-overlay) dual_runtime_args+=("$1"); shift ;;
    --manus-rawviz|--manus-user|--manus-library-dir|--right-glove|--left-glove|--tjvr-bind|--tjvr-port|--duration-s)
      dual_runtime_args+=("$1" "${2:?missing dual-input runtime value}"); shift 2 ;;
    --xr-sdk-pythonpath)
      xr_sdk_pythonpath_override="${2:?missing XR SDK Python path}"; shift 2 ;;
    --pico-overlay) pico_overlay=true; shift ;;
    --xr-overlay) xr_overlay=true; shift ;;
    --ik-target-overlay) ik_target_overlay=true; shift ;;
    --ik-backend) ik_backend_override="${2:?missing IK backend}"; shift 2 ;;
    --joint-trajectory) joint_trajectory_override="${2:?missing trajectory processor}"; shift 2 ;;
    --command-step-clipping) command_clipping_override="${2:?missing clipping mode}"; shift 2 ;;
    --arm-target-processor) target_processor_override="${2:?missing target processor}"; shift 2 ;;
    --arm-pose-mapper) pose_mapper_override="${2:?missing pose mapper}"; shift 2 ;;
    --joint-limit-source) joint_limit_source="${2:?missing joint limit source}"; shift 2 ;;
    --operator-input) operator_input_override="${2:?missing operator input}"; shift 2 ;;
    --arm-input) arm_input_override="${2:?missing arm input}"; shift 2 ;;
    --record) record_path="${2:-}"; shift 2 ;;
    --h5|--input) input_path="${2:-}"; shift 2 ;;
    --speed) playback_speed="${2:-}"; shift 2 ;;
    --observation-config) observation_config_override="${2:-}"; shift 2 ;;
    --confirm-real) confirm_real=true; shift ;;
    --viewer)
      [[ "${display_mode}" != headless ]] || {
        printf '%s\n' '错误：--viewer 与 --headless 互斥。' >&2
        exit 2
      }
      display_mode=viewer
      shift
      ;;
    --headless)
      [[ "${display_mode}" != viewer ]] || {
        printf '%s\n' '错误：--viewer 与 --headless 互斥。' >&2
        exit 2
      }
      display_mode=headless
      shift
      ;;
    --help|-h)
      printf '%s\n' \
        '用法: run_session.sh --profile PROFILE [--record PATH] [--observation-config PATH] [--disable-hands] [--confirm-real] [--h5 PATH] [--speed RATE] [--viewer|--headless]' \
        '显示模式：h5_sim 默认打开 MuJoCo viewer；追加 --headless 可显式启用无窗口模式。' \
        '新双输入配置只读检查：--profile {pico2_hands_sim|vr_manus_sim|vr_manus_xr_sim} --resolve-only。' \
        '仅新PICO入口可选：--operator-input gesture（armed下双手先释放再张开0.8秒请求启动，默认keyboard）。' \
        'VR+Manus仿真：--profile vr_manus_sim --manus-rawviz PATH --manus-user USER；仅双臂使用 --disable-hands。' \
        'XR+Manus仿真：--profile vr_manus_xr_sim --manus-rawviz PATH --manus-user USER [--arm-input {xr_tracker|xr_controller}]。' \
        'XR原生SDK路径：可选 --xr-sdk-pythonpath PATH（仅注入 XR 采集进程，也可用 TIANJI_XR_SDK_PYTHONPATH）。' \
        'XR原始可视化：--profile vr_manus_xr_sim --viewer --xr-overlay（仅显示头显、控制器和Tracker）。' \
        'VR诊断/录制：--spark-overlay --record NEW.h5（父目录须存在）；h 回Home，r 联合重置，再用新输入和 s 启动。' \
        'hand_tracking 仿真可选：--ik-backend NAME --joint-trajectory {passthrough|ruckig} --command-step-clipping {true|false} --arm-target-processor {passthrough|conditioned}' \
        'PICO 仿真位姿映射：--arm-pose-mapper {relative_home|head_direct|head_palm_direct}（默认 relative_home）' \
        'PICO 原始头显/手腕/26点骨架显示：--profile hand_tracking_sim --viewer --pico-overlay（兼容 --disable-hands）' \
        'Dexhand QP 仿真限位来源：--joint-limit-source {yaml|urdf}（默认 yaml，保留硬限位检查）' \
        'hand_tracking 仿真显示实际 IK 期望 TCP：--ik-target-overlay（可与 --pico-overlay 同用）' \
        'regrind_real 可用 --viewer 打开只读 frame0 对齐窗口；其他 profile 保持 executor config 默认。'
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
hand_tracking_simulation=false
if [[ -n "${operator_input_override}" && "${profile}" != pico2_hands_sim &&
      "${profile}" != vr_manus_xr_sim ]]; then
  printf '%s\n' '错误：--operator-input 仅支持 pico2_hands_sim 或 vr_manus_xr_sim。' >&2
  exit 2
fi
if [[ -n "${arm_input_override}" && "${profile}" != vr_manus_xr_sim ]]; then
  printf '%s\n' '错误：--arm-input 仅支持 vr_manus_xr_sim。' >&2
  exit 2
fi
if [[ -n "${xr_sdk_pythonpath_override}" && "${profile}" != vr_manus_xr_sim ]]; then
  printf '%s\n' '错误：--xr-sdk-pythonpath 仅支持 vr_manus_xr_sim。' >&2
  exit 2
fi
pico_simulation=false
xr_manus_simulation=false
[[ "${profile}" != hand_tracking_sim && "${profile}" != hand_tracking_sim_manus && "${profile}" != pico2_hands_sim ]] || hand_tracking_simulation=true
[[ "${profile}" != hand_tracking_sim && "${profile}" != pico2_hands_sim ]] || pico_simulation=true
[[ "${profile}" != vr_manus_xr_sim ]] || xr_manus_simulation=true
if [[ "${profile}" == pico2_hands_sim || "${profile}" == vr_manus_sim ||
      "${profile}" == vr_manus_xr_sim ]]; then
  if [[ "${resolve_only}" == true ]] && { [[ -n "${record_path}${input_path}${display_mode}${playback_speed}${observation_config_override}" ||
        "${confirm_real}" == true || "${pico_overlay}" == true || "${ik_target_overlay}" == true ||
        "${xr_overlay}" == true ||
        ${#extra_args[@]} -gt 0 || ${#dual_runtime_args[@]} -gt 0 ||
        -n "${xr_sdk_pythonpath_override}" ]]; }; then
    printf '%s\n' '错误：--resolve-only 不接受采集、录制、显示或未识别的运行参数。' >&2
    exit 2
  fi
  resolve_args=(--profile "${profile}")
  [[ "${disable_hands}" != true ]] || resolve_args+=(--disable-hands)
  [[ -z "${ik_backend_override}" ]] || resolve_args+=(--ik-backend "${ik_backend_override}")
  [[ -z "${pose_mapper_override}" ]] || resolve_args+=(--arm-pose-mapper "${pose_mapper_override}")
  [[ -z "${target_processor_override}" ]] || resolve_args+=(--arm-target-processor "${target_processor_override}")
  [[ -z "${joint_trajectory_override}" ]] || resolve_args+=(--joint-trajectory "${joint_trajectory_override}")
  [[ -z "${command_clipping_override}" ]] || resolve_args+=(--command-step-clipping "${command_clipping_override}")
  [[ -z "${joint_limit_source}" ]] || resolve_args+=(--joint-limit-source "${joint_limit_source}")
  [[ -z "${operator_input_override}" ]] || resolve_args+=(--operator-input "${operator_input_override}")
  [[ -z "${arm_input_override}" ]] || resolve_args+=(--arm-input "${arm_input_override}")
  if [[ "${resolve_only}" == true ]]; then
    exec python "${SCRIPT_DIR}/resolve_dual_session.py" "${resolve_args[@]}"
  fi
  if [[ "${profile}" == pico2_hands_sim ]]; then
    if [[ "${confirm_real}" == true || -n "${input_path}${playback_speed}" ||
          ${#extra_args[@]} -gt 0 || ${#dual_runtime_args[@]} -gt 0 ]]; then
      printf '%s\n' '错误：pico2_hands_sim 为 simulation-only，不接受真机、VR/Manus、回放或未知参数。' >&2
      exit 2
    fi
    pico_resolved="$(python "${SCRIPT_DIR}/resolve_dual_session.py" "${resolve_args[@]}")"
    pico_settings="$(python - "${pico_resolved}" "${BUNDLE_ROOT}/src/tianji_teleop" <<'PY'
import json, sys
sys.path.insert(0, sys.argv[2])
from tianji_teleop.hand_tracking.session_config import validate_pico_runtime
value = json.loads(sys.argv[1])['config']
validate_pico_runtime(value)
for key in ('ik_backend', 'arm_pose_mapper', 'arm_target_processor', 'joint_trajectory',
            'command_step_clipping', 'joint_limit_source', 'operator_input'):
    item = value[key]
    print(str(item).lower() if isinstance(item, bool) else item)
PY
)"
    mapfile -t pico_settings_array <<< "${pico_settings}"
    ik_backend_override="${pico_settings_array[0]}"
    pose_mapper_override="${pico_settings_array[1]}"
    target_processor_override="${pico_settings_array[2]}"
    joint_trajectory_override="${pico_settings_array[3]}"
    command_clipping_override="${pico_settings_array[4]}"
    joint_limit_source="${pico_settings_array[5]}"
    operator_input_override="${pico_settings_array[6]}"
    if [[ "${disable_hands}" != true && ! -x "${BUNDLE_ROOT}/tools/wuji_hand_native/.pixi/envs/default/bin/python" ]]; then
      printf '%s\n' '错误：缺少官方 Hand2 独立运行环境。' >&2; exit 2
    fi
  elif [[ "${profile}" == vr_manus_sim ]]; then
  if [[ -n "${input_path}${playback_speed}${observation_config_override}" ||
        "${confirm_real}" == true || "${pico_overlay}" == true || "${ik_target_overlay}" == true ||
        "${xr_overlay}" == true ||
        ${#extra_args[@]} -gt 0 ]]; then
    printf '%s\n' '错误：vr_manus_sim live 当前不接受回放、旧PICO overlay、真机或未识别参数。' >&2
    exit 2
  fi
  [[ -z "${display_mode}" ]] || resolve_args+=("--${display_mode}")
  [[ -z "${record_path}" ]] || resolve_args+=(--record "${record_path}")
    exec bash "${SCRIPT_DIR}/run_vr_manus_session.sh" "${resolve_args[@]}" "${dual_runtime_args[@]}"
  fi
fi
xr_resolved=""
if [[ "${profile}" == vr_manus_xr_sim ]]; then
  xr_resolved="$(python "${SCRIPT_DIR}/resolve_dual_session.py" "${resolve_args[@]}")"
  xr_settings="$(python - "${xr_resolved}" <<'PY'
import json
import sys
value = json.loads(sys.argv[1])['config']
print(value['arm_input'])
print(value['operator_input'])
PY
)"
  mapfile -t xr_settings_array <<< "${xr_settings}"
  arm_input_override="${xr_settings_array[0]}"
  operator_input_override="${xr_settings_array[1]}"
fi
if ((${#dual_runtime_args[@]})); then
  if [[ "${profile}" != vr_manus_xr_sim ]]; then
    printf '%s\n' '错误：新增采集参数仅用于 vr_manus_sim 或 vr_manus_xr_sim。' >&2
    exit 2
  fi
  for ((dual_index = 0; dual_index < ${#dual_runtime_args[@]}; dual_index += 1)); do
    case "${dual_runtime_args[dual_index]}" in
      --manus-rawviz)
        xr_manus_rawviz_override="${dual_runtime_args[dual_index + 1]}"; dual_index=$((dual_index + 1)) ;;
      --manus-user)
        xr_manus_user_override="${dual_runtime_args[dual_index + 1]}"; dual_index=$((dual_index + 1)) ;;
      --manus-library-dir)
        xr_manus_library_dir_override="${dual_runtime_args[dual_index + 1]}"; dual_index=$((dual_index + 1)) ;;
      --right-glove)
        xr_manus_right_glove_override="${dual_runtime_args[dual_index + 1]}"; dual_index=$((dual_index + 1)) ;;
      --left-glove)
        xr_manus_left_glove_override="${dual_runtime_args[dual_index + 1]}"; dual_index=$((dual_index + 1)) ;;
      *) printf '错误：vr_manus_xr_sim 不支持运行参数: %s\n' "${dual_runtime_args[dual_index]}" >&2; exit 2 ;;
    esac
  done
fi
if [[ "${resolve_only}" == true ]]; then
  printf '%s\n' '错误：--resolve-only 仅用于新的双输入配置；旧 profile 启动行为保持不变。' >&2
  exit 2
fi
case "${profile}" in
  mocap_live_sim|mocap_live_real|h5_sim|h5_real|regrind_real|target_replay_sim|joint_replay_sim|wuji_direct_real|diagnostic_mocap_calibration_sim|hand_tracking_observation|hand_tracking_observation_manus|hand_tracking_sim|hand_tracking_sim_manus|pico2_hands_sim|vr_manus_xr_sim) ;;
  *) printf '错误：未知 session profile: %s\n' "${profile}" >&2; exit 2 ;;
esac
if [[ -n "${pose_mapper_override}" ]]; then
  if [[ "${xr_manus_simulation}" == true ]]; then
    [[ "${pose_mapper_override}" == xr_incremental ]] || {
      printf '%s\n' '错误：vr_manus_xr_sim 的 --arm-pose-mapper 必须为 xr_incremental。' >&2; exit 2;
    }
  else
    if [[ "${pico_simulation}" != true ]]; then
      printf '%s\n' '错误：--arm-pose-mapper 仅支持 PICO hand_tracking_sim。' >&2; exit 2
    fi
    case "${pose_mapper_override}" in
      relative_home|head_direct|head_palm_direct) ;;
      *) printf '%s\n' '错误：--arm-pose-mapper 必须为 relative_home、head_direct 或 head_palm_direct。' >&2; exit 2 ;;
    esac
  fi
fi
# No inherited override may silently change another profile's mapping.
unset TIANJI_ARM_POSE_MAPPER
unset TIANJI_JOINT_TRAJECTORY_PROCESSOR TIANJI_COMMAND_STEP_CLIPPING TIANJI_ARM_TARGET_PROCESSOR
[[ -z "${pose_mapper_override}" ]] || export TIANJI_ARM_POSE_MAPPER="${pose_mapper_override}"
if [[ -n "${joint_limit_source}" ]]; then
  case "${joint_limit_source}" in
    yaml|urdf) ;;
    *) printf '%s\n' '错误：--joint-limit-source 必须为 yaml 或 urdf。' >&2; exit 2 ;;
  esac
  if [[ "${hand_tracking_simulation}" != true && "${xr_manus_simulation}" != true ]]; then
    printf '%s\n' '错误：--joint-limit-source 仅支持 hand_tracking 或 XR/Manus 仿真。' >&2; exit 2
  fi
  if [[ "${joint_limit_source}" == urdf && "${ik_backend_override}" != pico_ee_dexhand_qp ]]; then
    printf '%s\n' '错误：--joint-limit-source urdf 需要 --ik-backend pico_ee_dexhand_qp。' >&2; exit 2
  fi
fi
if [[ "${pico_overlay}" == true && "${pico_simulation}" != true ]]; then
  printf '%s\n' '错误：--pico-overlay 仅支持 hand_tracking_sim。' >&2
  exit 2
fi
if [[ "${xr_overlay}" == true && "${profile}" != vr_manus_xr_sim ]]; then
  printf '%s\n' '错误：--xr-overlay 仅支持 vr_manus_xr_sim。' >&2
  exit 2
fi
if [[ "${ik_target_overlay}" == true && "${hand_tracking_simulation}" != true ]]; then
  printf '%s\n' '错误：--ik-target-overlay 仅支持 hand_tracking 仿真。' >&2
  exit 2
fi
if [[ "${hand_tracking_simulation}" == true || "${xr_manus_simulation}" == true ]]; then
  [[ "${confirm_real}" != true ]] || {
    printf '%s\n' '错误：hand_tracking_sim 是 simulation-only profile，不接受 --confirm-real。' >&2
    exit 2
  }
fi
if [[ "${profile}" == hand_tracking_observation || "${profile}" == hand_tracking_observation_manus ]]; then
  if [[ -n "${input_path}" || -n "${playback_speed}" || "${confirm_real}" == true || "${display_mode}" == viewer || "${display_mode}" == headless ]]; then
    printf '%s\n' '错误：hand_tracking_observation 只支持接收/发布/记录，不接受机器人或 viewer 参数。' >&2
    exit 2
  fi
  if [[ -n "${observation_config_override}" ]]; then
    observation_config="${observation_config_override}"
  elif [[ "${profile}" == hand_tracking_observation_manus ]]; then
    observation_config="${SCRIPT_DIR}/../src/tianji_teleop/config/sources/hand_tracking_observation_manus.yaml"
  else
    observation_config="${SCRIPT_DIR}/../src/tianji_teleop/config/sources/hand_tracking_observation.yaml"
  fi
  [[ -f "${observation_config}" ]] || {
    printf '错误：缺少 observation config: %s\n' "${observation_config}" >&2
    exit 1
  }
  if [[ "${profile}" == hand_tracking_observation_manus ]]; then
    export TIANJI_REQUIRED_OBSERVATION_PROFILE=manus
  else
    export TIANJI_REQUIRED_OBSERVATION_PROFILE=pico
  fi
  observation_args=(--config "${observation_config}")
  [[ -n "${record_path}" ]] && observation_args+=(--record "${record_path}")
  ((${#extra_args[@]} == 0)) || observation_args+=(-- "${extra_args[@]}")
  exec bash "${SCRIPT_DIR}/run_observation_session.sh" "${observation_args[@]}"
fi
if [[ "${profile}" == target_replay_sim || "${profile}" == joint_replay_sim || "${profile}" == wuji_direct_real ]]; then
  if [[ -n "${record_path}" ]]; then
    printf '%s\n' 'replay profile cannot be recorded' >&2
    exit 2
  fi
fi
if [[ "${profile}" == diagnostic_mocap_calibration_sim && -n "${record_path}" ]]; then
  printf '%s\n' 'diagnostic profile cannot be recorded: no session raw schema' >&2
  exit 2
fi
if [[ "${profile}" == regrind_real && -n "${record_path}" ]]; then
  printf '%s\n' 'regrind_real recording is not implemented yet' >&2
  exit 2
fi
profile_config="$(canonical_config "sessions/${profile}.yaml")"
[[ "${profile}" != pico2_hands_sim ]] || profile_config="$(canonical_config sessions/pico2_hands_runtime.yaml)"
[[ "${profile}" != vr_manus_xr_sim ]] || profile_config="$(canonical_config sessions/vr_manus_xr_runtime.yaml)"
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
arm_command_path="$(profile_value arm_command_path)"
inactive_sides="$(profile_value inactive_sides)"
hand_mode="$(profile_value hand_mode)"
active_hand_sides="$(profile_value active_hand_sides)"
hand_executor="$(profile_value hand_executor)"
hand_executor_config="$(profile_value hand_executor_config)"
hand_overlay="$(profile_value hand_overlay)"
observation_config="$(profile_value observation_config)"
[[ -n "${arm_command_path}" ]] || arm_command_path=coordinator
if [[ "${arm_command_path}" != coordinator && "${arm_command_path}" != direct ]]; then
  printf '错误：非法 arm_command_path: %s\n' "${arm_command_path}" >&2
  exit 2
fi
if [[ "${TIANJI_VALIDATION_PRODUCER:-}" == policy_hold && "${TIANJI_VALIDATION_CASE_ID:-}" == policy_hold_sim ]]; then
  arm_producer_config="producers/policy_hold.yaml"
fi
forced_hand_mode="${TIANJI_VALIDATION_HAND_MODE:-}"
if [[ -n "${ik_backend_override}${joint_trajectory_override}${command_clipping_override}${target_processor_override}" ]]; then
  [[ "${hand_tracking_simulation}" == true || "${xr_manus_simulation}" == true ]] || {
    printf '%s\n' '错误：IK/处理链覆盖参数只支持 hand_tracking 或 XR/Manus 仿真。' >&2; exit 2;
  }
  case "${ik_backend_override}" in ""|pinocchio_qp|pinocchio_cpp|tianji_official|pico_ee_dexhand_qp) ;; *) exit 2 ;; esac
  case "${joint_trajectory_override}" in ""|passthrough|ruckig) ;; *) exit 2 ;; esac
  case "${command_clipping_override}" in ""|true|false) ;; *) exit 2 ;; esac
  case "${target_processor_override}" in ""|passthrough|conditioned) ;; *) exit 2 ;; esac
fi
if [[ "${ik_backend_override}" == pico_ee_dexhand_qp ]]; then
  arm_producer_config=producers/ik_dexhand_qp.yaml
  coordinator_config=coordinator/arm_v131.yaml
  joint_trajectory_override="${joint_trajectory_override:-passthrough}"
  command_clipping_override="${command_clipping_override:-false}"
  target_processor_override="${target_processor_override:-passthrough}"
fi
[[ -z "${joint_trajectory_override}" ]] || export TIANJI_JOINT_TRAJECTORY_PROCESSOR="${joint_trajectory_override}"
[[ -z "${command_clipping_override}" ]] || export TIANJI_COMMAND_STEP_CLIPPING="${command_clipping_override}"
[[ -z "${target_processor_override}" ]] || export TIANJI_ARM_TARGET_PROCESSOR="${target_processor_override}"
if [[ "${disable_hands}" == true ]]; then
  [[ "${hand_tracking_simulation}" == true || "${xr_manus_simulation}" == true ]] || {
    printf '%s\n' '错误：--disable-hands 仅支持 hand_tracking 或 XR/Manus 仿真。' >&2; exit 2;
  }
  forced_hand_mode=disabled
fi
if [[ -n "${forced_hand_mode}" && "${forced_hand_mode}" != disabled && "${forced_hand_mode}" != direct && "${forced_hand_mode}" != retarget ]]; then
  printf '错误：非法 validation hand mode: %s\n' "${forced_hand_mode}" >&2
  exit 2
fi
if [[ -n "${forced_hand_mode}" ]]; then hand_mode="${forced_hand_mode}"; fi
[[ -n "${active_hand_sides}" ]] || active_hand_sides="${active_sides}"
[[ -n "${hand_executor}" ]] || hand_executor=none
[[ -n "${hand_executor_config}" ]] || hand_executor_config=executors/wuji_hand2.yaml
[[ -n "${hand_overlay}" ]] || hand_overlay=none
[[ -n "${source_config}" && -n "${arm_executor_config}" && -n "${coordinator_config}" ]] || {
  printf '%s\n' '错误：session profile 缺少 source/executor/coordinator config。' >&2
  exit 2
}
if [[ "${hand_tracking_simulation}" == true || "${xr_manus_simulation}" == true ]]; then
  if [[ -n "${observation_config_override}" ]]; then
    observation_config="${observation_config_override}"
    [[ -f "${observation_config}" ]] || { printf '错误：缺少 observation config: %s\n' "${observation_config}" >&2; exit 2; }
  else
    observation_config="$(canonical_config "${observation_config}")"
  fi
  [[ -n "${observation_config}" ]] || {
    printf '%s\n' '错误：hand_tracking_sim profile 缺少 observation_config。' >&2
    exit 2
  }
  if [[ "${hand_tracking_simulation}" == true ]]; then
    [[ "${required_capability}" == simulation && ( "${hand_mode}" == disabled ||
       ( "${hand_mode}" == retarget && "${hand_executor}" == wuji_hand2 ) ||
       ( "${profile}" == pico2_hands_sim && "${hand_mode}" == direct && "${hand_executor}" == mujoco ) ) ]] || {
      printf '%s\n' '错误：hand_tracking_sim 只允许 simulation + (disabled 或 retarget + wuji_hand2)。' >&2
      exit 2
    }
  else
    [[ "${required_capability}" == simulation && ( "${hand_mode}" == disabled ||
       ( "${hand_mode}" == retarget && "${hand_executor}" == wuji_hand2 ) ) ]] || {
      printf '%s\n' '错误：vr_manus_xr_sim 只允许 simulation + (disabled 或 retarget + wuji_hand2)。' >&2
      exit 2
    }
  fi
fi
if [[ "${xr_manus_simulation}" == true && "${hand_mode}" != disabled ]]; then
  retarget_python="${TIANJI_WUJI_RETARGET_PYTHON:-${BUNDLE_ROOT}/tools/wuji_hand_native/.pixi/envs/default/bin/python}"
  retarget_worker="${TIANJI_WUJI_RETARGET_WORKER:-${BUNDLE_ROOT}/scripts/wuji_hand_worker.py}"
  [[ -x "${retarget_python}" && -f "${retarget_worker}" ]] || {
    printf '%s\n' '错误：XR+Manus 仿真需要官方 Wuji2 retarget worker；请先准备 tools/wuji_hand_native 环境。' >&2
    exit 1
  }
  PYTHONPATH="${BUNDLE_ROOT}/src/tianji_teleop${PYTHONPATH:+:${PYTHONPATH}}" pixi run python - \
    "${observation_config}" "${xr_manus_rawviz_override}" "${xr_manus_user_override}" \
    "${xr_manus_library_dir_override}" "${xr_manus_right_glove_override}" \
    "${xr_manus_left_glove_override}" <<'PY'
import sys
from tianji_teleop.hand_tracking.xr_manus_observation import _load_config, validate_manus_runtime

config = _load_config(sys.argv[1])
manus = dict(config.get('manus') or {})
for field, value in zip(('rawviz', 'user', 'library_dir', 'right_glove', 'left_glove'), sys.argv[2:]):
    if value:
        manus[field] = value
validate_manus_runtime(manus)
PY
fi
if [[ "${xr_manus_simulation}" == true ]]; then
  xr_sdk_pythonpath="${xr_sdk_pythonpath_override:-${TIANJI_XR_SDK_PYTHONPATH:-}}"
  if ! TIANJI_XR_SDK_PYTHONPATH="${xr_sdk_pythonpath}" \
       PYTHONPATH="${BUNDLE_ROOT}/src/tianji_teleop${PYTHONPATH:+:${PYTHONPATH}}" \
       pixi run python "${SCRIPT_DIR}/check_xr_sdk.py"; then
    printf '%s\n' \
      '错误：XRoboToolkit SDK 预检失败；请安装兼容 Pybind 并通过 --xr-sdk-pythonpath 或 TIANJI_XR_SDK_PYTHONPATH 指定。' \
      >&2
    exit 1
  fi
fi
if [[ -z "${display_mode}" ]]; then
  if [[ "${profile}" == h5_sim ]]; then
    display_mode=viewer
  else
    display_mode=config
  fi
fi
regrind_alignment_viewer=false
if [[ "${profile}" == regrind_real && "${display_mode}" == viewer ]]; then
  regrind_alignment_viewer=true
elif [[ "${display_mode}" != config && "${arm_executor_config}" != executors/mujoco.yaml ]]; then
  printf '%s\n' '错误：--viewer/--headless 仅适用于 MuJoCo executor。' >&2
  exit 2
fi
if [[ "${display_mode}" == viewer && "${required_capability}" != simulation && "${regrind_alignment_viewer}" != true ]]; then
  printf '%s\n' '错误：--viewer 只允许 simulation + MuJoCo executor。' >&2
  exit 2
fi
arm_display_args=()
if [[ "${display_mode}" == viewer && "${regrind_alignment_viewer}" != true ]]; then
  arm_display_args+=(--viewer)
elif [[ "${display_mode}" == headless ]]; then
  arm_display_args+=(--headless)
fi
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
if [[ "${required_capability}" == real ]]; then
  if [[ "${profile}" == regrind_real ]]; then
    if [[ -n "${playback_speed}" ]]; then
      if ! pixi run python - "${playback_speed}" <<'PY'
import math
import sys
try:
    speed = float(sys.argv[1])
except (TypeError, ValueError):
    raise SystemExit(1)
raise SystemExit(0 if math.isfinite(speed) and speed == 1.0 else 1)
PY
      then
        printf '%s\n' '错误：regrind_real 的策略 capability 要求 --speed 1.0；物理动态限制由 profile config 控制。' >&2
        exit 2
      fi
    fi
    playback_speed=1.0
  fi
  export TIANJI_REAL_SPEED="${playback_speed:-${TIANJI_REAL_SPEED:-0.25}}"
  export TIANJI_REAL_YAW_DEG="${TIANJI_REAL_YAW_DEG:-0}"
  if [[ -z "${TIANJI_REAL_PREFLIGHT_FD:-}" &&
        -z "${TIANJI_REAL_PREFLIGHT_SCANNER_FD:-}" &&
        -z "${TIANJI_CONFIRMED_REAL_PREFLIGHT_FD:-}" ]]; then
    relaunch=(bash "${SCRIPT_DIR}/run_session.sh" --profile "${profile}" --confirm-real)
    [[ -n "${record_path}" ]] && relaunch+=(--record "${record_path}")
    [[ -n "${input_path}" ]] && relaunch+=(--input "${input_path}")
    relaunch+=(--speed "${TIANJI_REAL_SPEED}")
    [[ "${display_mode}" == viewer ]] && relaunch+=(--viewer)
    [[ "${display_mode}" == headless ]] && relaunch+=(--headless)
    ((${#extra_args[@]} == 0)) || relaunch+=(-- "${extra_args[@]}")
    exec pixi run python "${SCRIPT_DIR}/run_confirmed_real_session.py" \
      --profile "${profile}" --speed "${TIANJI_REAL_SPEED}" \
      --yaw-deg "${TIANJI_REAL_YAW_DEG}" -- "${relaunch[@]}"
  fi
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
  mocap_live|h5_replay|regrind_policy|hand_tracking_target|hand_tracking_target_manus|hand_tracking_target_xr_manus|target|joint|joint_real) ;;
  mocap_calibration) ;;
  *) printf '错误：source config 不在 canonical source/replay/diagnostic 树: %s\n' "${source_config}" >&2; exit 2 ;;
esac
case "${source_name}" in
  hand_tracking_target_manus|hand_tracking_target_xr_manus) source_id=hand_tracking_target ;;
  target) source_id=target_replay ;;
  joint|joint_real) source_id=joint_replay ;;
  mocap_calibration) source_id=diagnostic_mocap_calibration ;;
  *) source_id="${source_name}" ;;
esac
if [[ "${source_id}" == h5_replay ]]; then
  [[ -n "${input_path}" && -f "${input_path}" ]] || {
    printf '%s\n' '错误：H5 profile 需要 --h5 PATH 或位置参数。' >&2
    exit 2
  }
  if [[ "${hand_mode}" == auto ]]; then
    if PYTHONPATH="${BUNDLE_ROOT}/src/tianji_teleop:${BUNDLE_ROOT}/vendor/python:${PYTHONPATH:-}" pixi run python - "${input_path}" "${active_hand_sides}" <<'PY'
import sys
import numpy as np
from tianji_teleop.sources.mocap.h5 import load_mocap_h5
recording = load_mocap_h5(sys.argv[1])
sides = tuple(side for side in sys.argv[2].split(",") if side)
if not sides:
    raise SystemExit("auto hand mode requires at least one active hand side")
for side in sides:
    if side not in {"left", "right"}:
        raise SystemExit(f"invalid active hand side: {side}")
    if side != "right":
        raise SystemExit("h5_replay canonical hand publisher supports only active right side")
    hand = recording.hands[side]
    joints = hand.wuji2_joints
    valid = hand.valid
    if joints is None or joints.shape != (recording.frame_count, 20):
        raise SystemExit(f"active hand side {side} has no canonical wuji2_joints dataset")
    if not bool(valid.any()):
        raise SystemExit(f"active hand side {side} has no valid frames")
    if not bool(np.isfinite(joints[valid]).all()):
        raise SystemExit(f"active hand side {side} has nonfinite direct joint frames")
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
observation_instance=""
observation_profile=""
if [[ "${hand_tracking_simulation}" == true || "${xr_manus_simulation}" == true ]]; then
  observation_instance="${TIANJI_OBSERVATION_INSTANCE_ID:-$(new_instance_id)}"
  if [[ "${profile}" == hand_tracking_sim_manus || "${xr_manus_simulation}" == true ]]; then
    observation_profile=manus
  else
    observation_profile=pico
  fi
fi
arm_producer_instance=""
arm_producer_id="arm_ik_producer"
if [[ "${arm_producer_config}" == producers/policy_hold.yaml ]]; then
  arm_producer_id="policy_hold"
fi
if [[ "${source_id}" == joint_replay ]]; then
  arm_producer_instance="${TIANJI_ARM_PRODUCER_INSTANCE_ID:-$(new_instance_id)}"
  arm_producer_id="joint_replay"
elif [[ -n "${arm_producer_config}" && "${arm_producer_config}" != null ]]; then
  arm_producer_instance="${TIANJI_ARM_PRODUCER_INSTANCE_ID:-$(new_instance_id)}"
fi
if [[ "${arm_command_path}" == direct ]]; then
  [[ "${arm_producer_id}" == arm_ik_producer && -n "${arm_producer_instance}" ]] || {
    printf '%s\n' '错误：direct arm path 仅允许绑定已配置的 IK producer。' >&2
    exit 2
  }
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
pico_hand_instance=""
[[ "${profile}" != pico2_hands_sim ]] || pico_hand_instance="$(new_instance_id)"
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
    if [[ "${profile}" == pico2_hands_sim ]]; then
      hand_executor_instance_array+=("${arm_executor_instance}")
    else
      hand_executor_instance_array+=("${mapped_hand_executor:-$(new_instance_id)}")
    fi
    if [[ "${profile}" == pico2_hands_sim ]]; then
      hand_producer_id_array+=(official_wuji_hand2)
      hand_producer_instance_array+=("${pico_hand_instance}")
    elif [[ "${source_id}" == h5_replay && "${hand_mode}" == direct ]]; then
      hand_producer_id_array+=("h5_direct")
      hand_producer_instance_array+=("${source_instance}")
    elif [[ "${source_id}" == regrind_policy && "${hand_mode}" == direct ]]; then
      hand_producer_id_array+=("regrind_policy")
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

source, source_instance, arm_producer, arm_producer_instance, arm_executor_config, arm_executor_instance, coordinator, router, rows = sys.argv[1:]
arm_executor_logical_id = "mujoco" if arm_executor_config == "mujoco.yaml" else "marvin"
disabled = {"logical_id": "disabled", "publisher_instance_id": "disabled", "router_zid": router, "enabled": False}
hand_producers = {"left": dict(disabled), "right": dict(disabled)}
hand_executors = {"left": dict(disabled), "right": dict(disabled)}
for row in rows.split(","):
    if not row:
        continue
    side, producer, producer_instance, hand_executor_instance = row.split("|")
    hand_producers[side] = {"logical_id": producer, "publisher_instance_id": producer_instance, "router_zid": router}
    hand_executors[side] = {"logical_id": f"wuji_{side}", "publisher_instance_id": hand_executor_instance, "router_zid": router}
print(json.dumps({
    "source": {"logical_id": source, "publisher_instance_id": source_instance, "router_zid": router},
    "producer_arm": {"logical_id": arm_producer, "publisher_instance_id": arm_producer_instance or "disabled", "router_zid": router, "enabled": bool(arm_producer_instance)},
    "producer_hand": hand_producers,
    "coordinator_arm": {"logical_id": "arm", "publisher_instance_id": coordinator, "router_zid": router},
    "executor_arm": {"logical_id": arm_executor_logical_id, "publisher_instance_id": arm_executor_instance, "router_zid": router},
    "executor_hand": hand_executors,
}, separators=(",", ":")))
PY
)"
[[ -n "${authorities_json}" ]] || { printf '%s\n' '错误：无法构造完整 authority mapping。' >&2; exit 1; }
export TIANJI_AUTHORITIES="${authorities_json}"
export TIANJI_REQUIRED_CAPABILITY="${required_capability}"
activate_bundle_runtime
mode="simulation"
[[ "${required_capability}" == real ]] && mode=real
acquire_teleop_guard "${profile}"
if ! existing_tokens="$(read_teleop_node_list)"; then
  release_teleop_guard
  printf '%s\n' '错误：无法完成启动前 live domain preflight。' >&2
  exit 1
fi
if [[ -n "${existing_tokens}" ]] &&
   ! assert_profile_domains_free "${existing_tokens}"; then
  release_teleop_guard
  exit 1
fi
source_terminal_state=""
restore_source_terminal() {
  [[ -n "${source_terminal_state}" ]] || return 0
  local saved_state="${source_terminal_state}"
  source_terminal_state=""
  if ! stty "${saved_state}" </dev/tty 2>/dev/null; then
    printf '%s\n' '错误：受管 source 退出后无法恢复启动终端状态。' >&2
    return 1
  fi
}
source_process_group_remains() {
  local process_group=""
  local start_ticks=""
  local term_timeout_s=""
  local label=""
  [[ -r "${TELEOP_CHILDREN_FILE}" ]] || return 1
  while IFS=$'\t' read -r process_group start_ticks term_timeout_s label; do
    [[ "${label}" == source ]] && return 0
  done < "${TELEOP_CHILDREN_FILE}"
  return 1
}
run_session_cleanup_and_release() {
  local cleanup_status=0
  trap - EXIT INT TERM
  teleop_cleanup_and_release || cleanup_status=$?
  if [[ -n "${source_terminal_state}" ]] &&
     source_process_group_remains; then
    printf '%s\n' \
      '错误：受管 source 进程组仍存活；保留 guard/children 记录且拒绝恢复终端状态。' \
      >&2
    cleanup_status=1
  else
    restore_source_terminal || cleanup_status=1
  fi
  return "${cleanup_status}"
}
run_session_stop_on_signal() {
  run_session_cleanup_and_release || true
  exit 130
}
trap run_session_cleanup_and_release EXIT
trap run_session_stop_on_signal INT TERM
launch() {
  local label="$1"; shift
  local log_path="${TELEOP_RUNTIME_DIR}/${run_id}-${label}.log"
  local term_timeout_s=5
  if [[ "${label}" == arm_executor &&
        "${required_capability}" == real &&
        "${arm_executor_config}" == executors/marvin_impedance.yaml ]]; then
    # Allow the real arm's bounded home trajectory to finish before release.
    term_timeout_s=60
  fi
  local source_tty_fd=""
  if [[ "${label}" == source && -t 0 ]] &&
     { exec {source_tty_fd}<>/dev/tty; } 2>/dev/null; then
    if ! source_terminal_state="$(stty -g <&"${source_tty_fd}" 2>/dev/null)" ||
       [[ -z "${source_terminal_state}" ]]; then
      exec {source_tty_fd}>&-
      printf '%s\n' '错误：无法保存交互式 source 的启动终端状态。' >&2
      return 1
    fi
    setsid env PYTHONUNBUFFERED=1 "$@" <&"${source_tty_fd}" \
      > >(tee -- "${log_path}" >&"${source_tty_fd}") 2>&1 &
    local pid=$!
    exec {source_tty_fd}>&-
  else
    setsid env "$@" </dev/null >"${log_path}" 2>&1 &
    local pid=$!
  fi
  if ! register_teleop_process_group "${pid}" "${label}" "${term_timeout_s}"; then
    kill -TERM -- "-${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
    return 1
  fi
  for _ in {1..20}; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      printf '错误：组件 %s 在启动阶段退出。\n' "${label}" >&2
      return 1
    fi
    sleep 0.1
  done
  child_pids+=("${pid}")
  child_labels+=("${label}")
}
recorder_instance=""
[[ -n "${record_path}" ]] && recorder_instance="${TIANJI_RECORDER_INSTANCE_ID:-$(new_instance_id)}"
if [[ -n "${joint_limit_source}" ]]; then
  export TIANJI_JOINT_LIMIT_SOURCE="${joint_limit_source}"
  if [[ "${joint_limit_source}" == urdf ]]; then
    export TIANJI_ARM_URDF="${TIANJI_ARM_URDF:-${BUNDLE_ROOT}/src/tianji_teleop/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf}"
    effective_arm_config="${TELEOP_RUNTIME_DIR}/${run_id}-arm-limits.yaml"
    PYTHONPATH="${BUNDLE_ROOT}/src/tianji_teleop${PYTHONPATH:+:${PYTHONPATH}}" pixi run python -m tianji_teleop.joint_limit_source \
      --arm-config "${TIANJI_ARM_CONFIG:-$(canonical_config robot/arm.yaml)}" \
      --urdf "${TIANJI_ARM_URDF}" --source urdf --output "${effective_arm_config}"
    export TIANJI_ARM_CONFIG="${effective_arm_config}"
    printf 'joint_limit_source=urdf; arm_config=%s; urdf=%s\n' "${TIANJI_ARM_CONFIG}" "${TIANJI_ARM_URDF}"
  fi
fi
base_env=(
  "TIANJI_COORDINATOR_INSTANCE_ID=${coordinator_id}"
  "TIANJI_ROUTER_ENDPOINT=${TIANJI_ROUTER_ENDPOINT}"
  "TIANJI_ROUTER_ZID=${TIANJI_ROUTER_ZID}"
  "TIANJI_RUN_ID=${run_id}"
  "TIANJI_REQUIRED_CAPABILITY=${required_capability}"
  "TIANJI_HAND_MODE=${hand_mode}"
  "TIANJI_VALIDATION_CASE_ID=${TIANJI_VALIDATION_CASE_ID:-}"
  "TIANJI_VALIDATION_HAND_MODE=${TIANJI_VALIDATION_HAND_MODE:-}"
  "TIANJI_VALIDATION_PRODUCER=${TIANJI_VALIDATION_PRODUCER:-}"
  "TIANJI_VALIDATION_IK_BACKEND=${TIANJI_VALIDATION_IK_BACKEND:-}"
  "TIANJI_HAND_PRODUCER_ID=${hand_producer_id}"
  "TIANJI_HAND_PRODUCER_INSTANCE_ID=${hand_producer_instance}"
  "TIANJI_HAND_INPUT_INSTANCE_ID=${hand_input_instance}"
  "TIANJI_SOURCE_LOGICAL_ID=${source_id}"
  "TIANJI_SOURCE_INSTANCE_ID=${source_instance}"
  "TIANJI_OBSERVATION_PUBLISHER_INSTANCE_ID=${observation_instance}"
  "TIANJI_REQUIRED_OBSERVATION_PROFILE=${observation_profile}"
  "TIANJI_ARM_COMMAND_PATH=${arm_command_path}"
  "TIANJI_ARM_PRODUCER_LOGICAL_ID=${arm_producer_id}"
  "TIANJI_ARM_PRODUCER_INSTANCE_ID=${arm_producer_instance}"
  "TIANJI_AUTHORITIES=${TIANJI_AUTHORITIES}"
  "TIANJI_VALIDATION_SUPERVISOR_INSTANCE_ID=${TIANJI_VALIDATION_SUPERVISOR_INSTANCE_ID:-}"
  "TIANJI_SAFETY_SUPERVISOR_INSTANCE_ID=${TIANJI_SAFETY_SUPERVISOR_INSTANCE_ID:-${TIANJI_VALIDATION_SUPERVISOR_INSTANCE_ID:-}}"
  "TIANJI_REAL_PREFLIGHT_FD=${TIANJI_REAL_PREFLIGHT_FD:-}"
  "TIANJI_REAL_PREFLIGHT_SCANNER_FD=${TIANJI_REAL_PREFLIGHT_SCANNER_FD:-}"
)
# XR/Manus retarget executors publish a passive target-to-command audit used
# only by the dual-input recording checker.  PICO2 keeps the legacy hand path
# and does not receive this extra transport stream.
if [[ "${profile}" == vr_manus_xr_sim ]]; then
  base_env+=("TIANJI_HAND_OUTPUT_AUDIT=1")
fi
if [[ "${profile}" == pico2_hands_sim ]]; then
  base_env+=("TIANJI_RESOLVED_DUAL_SESSION=${pico_resolved}" "TIANJI_PICO_OPERATOR_INPUT=${operator_input_override}")
elif [[ "${profile}" == vr_manus_xr_sim ]]; then
  base_env+=(
    "TIANJI_RESOLVED_DUAL_SESSION=${xr_resolved}"
    "TIANJI_XR_OPERATOR_INPUT=${operator_input_override}"
    "TIANJI_XR_ARM_INPUT=${arm_input_override}"
    "TIANJI_MANUS_RIGHT_GLOVE=${xr_manus_right_glove_override}"
    "TIANJI_MANUS_LEFT_GLOVE=${xr_manus_left_glove_override}"
  )
fi
if [[ "${required_capability}" == real ]]; then
  # Real admission is process-issued and fail-closed.  Speed/yaw are fixed by
  # the profile; deadman and preflight remain false unless an authorized
  # launcher has supplied a typed result.
  base_env+=(
    "TIANJI_REAL_SPEED=${TIANJI_REAL_SPEED:-1}"
    "TIANJI_REAL_YAW_DEG=${TIANJI_REAL_YAW_DEG:-nan}"
    "TIANJI_REAL_DEADMAN_AVAILABLE=${TIANJI_REAL_DEADMAN_AVAILABLE:-0}"
  )
fi
if [[ -n "${record_path}" ]]; then
  record_source_type="${source_id}"
  record_input_profile=""
  recording_config_path="$(canonical_config recording/session.yaml)"
  if [[ "${hand_tracking_simulation}" == true || "${xr_manus_simulation}" == true ]]; then
    # A hand-tracking simulation has two source processes: the observation
    # receiver and the target bridge.  The recorder is passive and owns the
    # only HDF5 writer, while the session profile identifies the extended
    # source type and input profile.
    record_source_type="${profile}"
    record_input_profile="${observation_profile}"
    recording_config_path="$(canonical_config recording/session_hand_tracking.yaml)"
    if [[ "${profile}" == pico2_hands_sim || "${profile}" == vr_manus_xr_sim ]]; then
      recording_config_path="$(canonical_config recording/dual_input.yaml)"
    fi
  fi
  launch recorder "${base_env[@]}" \
    TIANJI_COMPONENT_INSTANCE_ID="${recorder_instance}" \
    TIANJI_RECORD_PATH="${record_path}" \
    TIANJI_RECORD_SOURCE_TYPE="${record_source_type}" \
    TIANJI_RECORD_INPUT_PROFILE="${record_input_profile}" \
    TIANJI_RECORDING_CONFIG="${recording_config_path}" \
    python -m tianji_teleop.recording.session_recorder
fi
launch coordinator "${base_env[@]}" TIANJI_COORDINATOR_INSTANCE_ID="${coordinator_id}" TIANJI_COORDINATOR_CONFIG="$(canonical_config "${coordinator_config}")" python "${BUNDLE_ROOT}/src/tianji_teleop/scripts/arm_command_coordinator"
if [[ "${regrind_alignment_viewer}" == true ]]; then
  viewer_entry="${BUNDLE_ROOT}/scripts/regrind_live_infer.py"
  [[ -f "${viewer_entry}" ]] || {
    printf '错误：regrind_real viewer 入口不存在：%s\n' "${viewer_entry}" >&2
    exit 1
  }
  launch regrind_alignment_viewer "${base_env[@]}" python "${viewer_entry}" "${extra_args[@]}" --viewer
fi
if [[ "${profile}" == h5_real && "${hand_overlay}" == mujoco ]]; then
  overlay_entry="${BUNDLE_ROOT}/src/tianji_teleop/scripts/h5_wrist_diagnostic"
  [[ -x "${overlay_entry}" ]] || {
    printf '错误：h5_real 要求 passive Frame0 overlay，但入口不存在：%s\n' "${overlay_entry}" >&2
    exit 1
  }
  launch h5_wrist_overlay "${base_env[@]}" python "${overlay_entry}" "${input_path}" --viewer
fi
source_hand_mode="${hand_mode}"
# Official hands own raw PICO processing independently. The legacy target
# bridge owns arms only, preserving its existing visual-loss hold semantics.
[[ "${profile}" != pico2_hands_sim ]] || source_hand_mode=disabled
source_args=("${base_env[@]}" TIANJI_HAND_MODE="${source_hand_mode}" TIANJI_COMPONENT_INSTANCE_ID="${source_instance}" TIANJI_SOURCE_INSTANCE_ID="${source_instance}" TIANJI_PRODUCER_INSTANCE_ID="${arm_producer_instance:-${hand_producer_instance}}" bash "${SCRIPT_DIR}/run_source.sh" --source "${source_id}" --config "$(canonical_config "${source_config}")")
if [[ "${source_id}" == h5_replay ]]; then
  source_args+=(-- "${input_path}")
  if [[ "${required_capability}" == real ]]; then
    source_args+=(--speed "${TIANJI_REAL_SPEED}" --yaw-deg "${TIANJI_REAL_YAW_DEG}")
  elif [[ -n "${playback_speed}" ]]; then
    source_args+=(--speed "${playback_speed}")
  fi
elif [[ "${source_id}" == mocap_live && "${required_capability}" == real ]]; then
  source_args+=(--param "speed:=${TIANJI_REAL_SPEED}" --param "yaw_deg:=${TIANJI_REAL_YAW_DEG}")
elif [[ "${source_id}" == target_replay || "${source_id}" == joint_replay ]]; then
  source_args+=(-- "${input_path}" --active-hand-sides "${active_hand_sides}" --inactive-hand-sides "${inactive_sides}")
fi
source_args+=("${extra_args[@]}")
observation_args=()
if [[ "${hand_tracking_simulation}" == true ]]; then
  observation_args=(
    "${base_env[@]}"
    "TIANJI_COMPONENT_INSTANCE_ID=${observation_instance}"
    "TIANJI_SOURCE_INSTANCE_ID=${observation_instance}"
    bash "${SCRIPT_DIR}/run_source.sh"
    --source hand_tracking_observation
    --config "${observation_config}"
    --suppress-status
  )
  [[ "${profile}" != pico2_hands_sim ]] || observation_args+=(--receiver-instance-id "${observation_instance}" --gesture-observations)
elif [[ "${xr_manus_simulation}" == true ]]; then
  observation_args=(
    "${base_env[@]}"
    "TIANJI_XR_SDK_PYTHONPATH=${xr_sdk_pythonpath_override:-${TIANJI_XR_SDK_PYTHONPATH:-}}"
    "TIANJI_COMPONENT_INSTANCE_ID=${observation_instance}"
    "TIANJI_SOURCE_INSTANCE_ID=${observation_instance}"
    bash "${SCRIPT_DIR}/run_source.sh"
    --source xr_manus_observation
    --config "${observation_config}"
    --arm-input "${arm_input_override}"
  )
  observation_args+=("${dual_runtime_args[@]}")
  [[ "${hand_mode}" != disabled ]] || observation_args+=(--no-manus)
fi
launch_arm_executor() {
  local hand_args=()
  if [[ "${profile}" == pico2_hands_sim && -n "${active_hand_sides}" ]]; then
    hand_args+=(--hand-sides "${active_hand_sides}" --official-hand-producer-instance "${hand_producer_instance}")
  elif [[ "${hand_overlay}" == mujoco && -n "${active_hand_sides}" ]]; then
    hand_args+=(--hand-sides "${active_hand_sides}" --hand-overlay)
  else
    # Wuji is the sole hand executor authority for hand-enabled sim/replay.
    hand_args+=(--hand-sides "")
  fi
  [[ "${pico_overlay}" != true ]] || hand_args+=(--pico-overlay)
  [[ "${xr_overlay}" != true ]] || hand_args+=(--xr-overlay)
  [[ "${ik_target_overlay}" != true ]] || hand_args+=(--ik-target-overlay)
  if [[ "${arm_executor_config}" == executors/mujoco.yaml ]]; then
    launch arm_executor "${base_env[@]}" TIANJI_COMPONENT_INSTANCE_ID="${arm_executor_instance}" bash "${SCRIPT_DIR}/run_executor.sh" --executor mujoco --config "$(canonical_config "${arm_executor_config}")" "${arm_display_args[@]}" "${hand_args[@]}"
  else
    launch arm_executor "${base_env[@]}" TIANJI_COMPONENT_INSTANCE_ID="${arm_executor_instance}" bash "${SCRIPT_DIR}/run_executor.sh" --executor marvin --config "$(canonical_config "${arm_executor_config}")" --confirm-real
  fi
}
launch_hand_executor() {
  if [[ "${profile}" == pico2_hands_sim && "${hand_mode}" != disabled ]]; then
    launch pico_official_hands "${base_env[@]}" TIANJI_PICO_HAND_MANAGED=1 \
      python "${SCRIPT_DIR}/pico_hand_live.py"
    return 0
  fi
  [[ "${hand_mode}" != disabled && "${hand_executor}" == wuji_hand2 ]] || return 0
  for hand_index in "${!hand_side_array[@]}"; do
    launch "hand_executor_${hand_side_array[hand_index]}" "${base_env[@]}" \
      TIANJI_COMPONENT_INSTANCE_ID="${hand_executor_instance_array[hand_index]}" \
      TIANJI_HAND_PRODUCER_ID="${hand_producer_id_array[hand_index]}" \
      TIANJI_HAND_PRODUCER_INSTANCE_ID="${hand_producer_instance_array[hand_index]}" \
      TIANJI_HAND_INPUT_INSTANCE_ID="${hand_input_instance}" \
      bash "${SCRIPT_DIR}/run_executor.sh" --executor wuji_hand2 --mode "${hand_mode}" \
      --side "${hand_side_array[hand_index]}" --config "$(canonical_config "${hand_executor_config}")"
  done
}
launch_arm_producer() {
  [[ -n "${arm_producer_config}" && "${arm_producer_config}" != null ]] || return 0
  producer_name="$(basename -- "${arm_producer_config}" .yaml)"
  [[ "${producer_name}" == ik_regrind || "${producer_name}" == ik_dexhand_qp ]] && producer_name=ik
  launch arm_producer "${base_env[@]}" TIANJI_COMPONENT_INSTANCE_ID="${arm_producer_instance}" bash "${SCRIPT_DIR}/run_producer.sh" --producer "${producer_name}" --backend "${ik_backend_override}" --config "$(canonical_config "${arm_producer_config}")"
}
launch_hand_tracking_observation() {
  [[ -n "${observation_config}" ]] || return 0
  if [[ "${xr_manus_simulation}" == true ]]; then
    launch xr_manus_observation "${observation_args[@]}"
  else
    launch hand_tracking_observation "${observation_args[@]}"
  fi
}
if [[ "${required_capability}" == real ]]; then
  launch_arm_producer
  launch source "${source_args[@]}"
  launch_hand_executor
  launch_arm_executor
else
  launch_arm_executor
  launch_hand_executor
  launch_arm_producer
  launch_hand_tracking_observation
  launch source "${source_args[@]}"
fi
printf '%s\n' "session ${profile} started; router_zid=${router_zid}; run_id=${run_id}"
printf '%s\n' "session_startup_complete run_id=${run_id}; profile=${profile}; router_zid=${router_zid}"
while true; do
  for index in "${!child_pids[@]}"; do
    pid="${child_pids[index]}"
    if ! kill -0 "${pid}" 2>/dev/null; then
      child_status=0
      wait "${pid}" || child_status=$?
      session_shutdown_requested=true
      if ((child_status == 0)); then
        printf '组件 %s 已受控退出，开始反序清理。\n' "${child_labels[index]}" >&2
        exit 0
      fi
      printf '错误：组件 %s 在运行期间异常退出 (status=%s)，开始反序清理。\n' "${child_labels[index]}" "${child_status}" >&2
      exit "${child_status}"
    fi
  done
  sleep 0.1
done
