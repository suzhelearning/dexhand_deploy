#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

if (($#)); then
  printf '%s\n' \
    '错误：此内部启动器不接受 IK 参数。' \
    '请在 config/mode/controller_only/controller_only_ik.yaml 中修改 ik_backend。' >&2
  exit 2
fi

acquire_teleop_guard controller-only-ik
install_teleop_cleanup_traps

"${SCRIPT_DIR}/doctor.sh"
activate_bundle_runtime
assert_no_conflicting_teleop_nodes

IK_NODE="${PROJECT_PREFIX}/lib/pico_body_tianji/tianji_kinematic_sim"
PARAMETERS="${PROJECT_PREFIX}/share/pico_body_tianji/config/mode/controller_only/controller_only_ik.yaml"
URDF_PATH="${PROJECT_PREFIX}/share/pico_body_tianji/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf"
IK_BACKEND="$(
  awk '$1 == "ik_backend:" {print $2; exit}' "${PARAMETERS}"
)"
case "${IK_BACKEND}" in
  pinocchio_cpp|pinocchio_qp|tianji_official) ;;
  *)
    printf '错误：%s 中的 ik_backend=%s 无效\n' \
      "${PARAMETERS}" "${IK_BACKEND:-<missing>}" >&2
    exit 2
    ;;
esac
INPUT_NODE="${PROJECT_PREFIX}/lib/pico_body_tianji/pico_controller_only_input"
for required in \
  "${IK_NODE}" \
  "${PARAMETERS}" \
  "${URDF_PATH}" \
  "${INPUT_NODE}"
do
  if [[ ! -f "${required}" ]]; then
    printf '错误：纯手柄 IK 运行文件不存在：%s\n' "${required}" >&2
    exit 1
  fi
done

printf '%s\n' \
  '启动 PICO 纯手柄 → 可配置 IK 链路。' \
  "IK 后端：${IK_BACKEND}" \
  '不读取 Body/Motion Tracker，不启动 RViz/MuJoCo，不连接 Marvin。' \
  'IK 输出：/pico_body_sim/{left,right}_arm/joint_commands'

# C++ 节点只接受裸 key:=value 参数；yaml_params_for 直接输出该格式。
mapfile -t ik_arguments < <(
  yaml_params_for tianji_kinematic_sim "${PARAMETERS}" \
    "urdf_path:=${URDF_PATH}"
)
for index in "${!ik_arguments[@]}"; do
  ik_arguments[index]="${ik_arguments[index]#--param }"
done
setsid "${IK_NODE}" "${ik_arguments[@]}" &
ik_pid=$!
register_teleop_process_group "${ik_pid}" controller-only-ik-solver 5

setsid python "${INPUT_NODE}" --config "${PARAMETERS}" &
input_pid=$!
register_teleop_process_group \
  "${input_pid}" controller-only-pico-input 5

set +e
wait -n "${ik_pid}" "${input_pid}"
task_exit=$?
set -e
exit "${task_exit}"
