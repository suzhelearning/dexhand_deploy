#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

WITH_MUJOCO=true
MUJOCO_PID=""
SIM_PID=""
INPUT_PID=""

usage() {
  cat <<'EOF'
用法：
  pixi run sim -- [模式]
  bash scripts/run_sim.sh [模式]

模式：
  --both         同时启动仿真链路与 MuJoCo 预览（默认）
  --mujoco-only  只启动仿真链路与 MuJoCo 预览
  --topics-only  只启动 PICO、IK 和仿真关节话题（无界面）
  -h, --help

本脚本只启动纯运动学仿真，不加载 Marvin SDK，不连接实体机械臂。
EOF
}

while (($#)); do
  case "$1" in
    --both|--mujoco-only)
      WITH_MUJOCO=true
      ;;
    --topics-only)
      WITH_MUJOCO=false
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf '错误：未知参数 %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [[
  "${WITH_MUJOCO}" == true &&
  -z "${DISPLAY:-}" &&
  -z "${WAYLAND_DISPLAY:-}"
]]; then
  printf '%s\n' \
    '错误：未检测到 DISPLAY/WAYLAND_DISPLAY，无法启动 MuJoCo 预览。' \
    '可改用 pixi run sim -- --topics-only 验证无界面仿真链路。' >&2
  exit 1
fi

acquire_teleop_guard simulation
install_teleop_cleanup_traps

"${SCRIPT_DIR}/doctor.sh"
activate_bundle_runtime
assert_no_conflicting_teleop_nodes

SIM_NODE="${PROJECT_PREFIX}/lib/pico_body_tianji/tianji_kinematic_sim"
INPUT_NODE="${PROJECT_PREFIX}/lib/pico_body_tianji/pico_controller_input"
PREVIEW_CONFIG="${PROJECT_PREFIX}/share/pico_body_tianji/config/mode/full_body/preview.yaml"
URDF_PATH="${PROJECT_PREFIX}/share/pico_body_tianji/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf"
MUJOCO_VIEWER="${BUNDLE_ROOT}/src/pico_body_tianji/scripts/mujoco_joint_viewer.py"

for required in \
  "${SIM_NODE}" \
  "${INPUT_NODE}" \
  "${PREVIEW_CONFIG}" \
  "${URDF_PATH}" \
  "${MUJOCO_VIEWER}"
do
  if [[ ! -f "${required}" ]]; then
    printf '错误：仿真运行文件不存在：%s\n' "${required}" >&2
    exit 1
  fi
done

if [[ "${WITH_MUJOCO}" == true ]]; then
  setsid python "${MUJOCO_VIEWER}" &
  MUJOCO_PID=$!
  register_teleop_process_group "${MUJOCO_PID}" mujoco-viewer 5
fi

printf '%s\n' \
  '启动纯运动学仿真：PICO + SMPL → 可配置 IK → JointState' \
  "  MuJoCo=${WITH_MUJOCO}" \
  '该任务不会连接 Marvin 控制器。'

# C++ 节点只接受裸 key:=value 参数；yaml_params_for 直接输出该格式。
mapfile -t sim_arguments < <(
  yaml_params_for tianji_kinematic_sim "${PREVIEW_CONFIG}" \
    "urdf_path:=${URDF_PATH}"
)
for index in "${!sim_arguments[@]}"; do
  sim_arguments[index]="${sim_arguments[index]#--param }"
done
setsid "${SIM_NODE}" "${sim_arguments[@]}" &
SIM_PID=$!
register_teleop_process_group "${SIM_PID}" sim-ik-solver 5

setsid python "${INPUT_NODE}" --config "${PREVIEW_CONFIG}" &
INPUT_PID=$!
register_teleop_process_group "${INPUT_PID}" sim-pico-input 5

if [[ "${WITH_MUJOCO}" == true ]]; then
  set +e
  wait -n "${SIM_PID}" "${INPUT_PID}" "${MUJOCO_PID}"
  task_exit=$?
  set -e
else
  set +e
  wait -n "${SIM_PID}" "${INPUT_PID}"
  task_exit=$?
  set -e
fi

exit "${task_exit}"
