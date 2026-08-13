#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

acquire_teleop_guard controller-only-ik
install_teleop_cleanup_traps

"${SCRIPT_DIR}/doctor.sh"
activate_bundle_runtime
assert_no_conflicting_teleop_nodes

IK_NODE="${PROJECT_PREFIX}/lib/pico_body_tianji/tianji_kinematic_sim"
PARAMETERS="${BUNDLE_ROOT}/src/pico_body_tianji/config/controller_only_ik.yaml"
INPUT_NODE="${BUNDLE_ROOT}/src/pico_body_tianji/scripts/pico_controller_only_input"
for required in "${IK_NODE}" "${PARAMETERS}" "${INPUT_NODE}"; do
  if [[ ! -f "${required}" ]]; then
    printf '错误：纯手柄 IK 运行文件不存在：%s\n' "${required}" >&2
    exit 1
  fi
done

printf '%s\n' \
  '启动 PICO 纯手柄 → Pinocchio IK 链路。' \
  '不读取 Body/Motion Tracker，不启动 RViz/MuJoCo，不连接 Marvin。' \
  'IK 输出：/pico_body_sim/{left,right}_arm/joint_commands'

setsid "${IK_NODE}" --ros-args --params-file "${PARAMETERS}" &
ik_pid=$!
register_teleop_process_group "${ik_pid}" controller-only-ik-solver 5

setsid python "${INPUT_NODE}" \
  --ros-args --params-file "${PARAMETERS}" &
input_pid=$!
register_teleop_process_group \
  "${input_pid}" controller-only-pico-input 5

set +e
wait -n "${ik_pid}" "${input_pid}"
task_exit=$?
set -e
exit "${task_exit}"
