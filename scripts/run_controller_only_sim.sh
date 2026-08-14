#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

WITH_RVIZ=true
WITH_MUJOCO=true
FORWARDED_ARGS=()
MUJOCO_PID=""
LAUNCH_PID=""

usage() {
  cat <<'EOF'
用法：
  pixi run sim_controller_only -- [模式]
  bash scripts/run_controller_only_sim.sh [模式] [-- ROS launch 参数...]

模式：
  --both         同时启动 RViz 与 MuJoCo（默认）
  --rviz-only    只启动 RViz
  --mujoco-only  只启动 MuJoCo
  --topics-only  只启动纯手柄、IK 和仿真关节话题
  -h, --help

本脚本只启动纯运动学仿真，
不加载 Marvin SDK，不连接实体机械臂。
EOF
}

while (($#)); do
  case "$1" in
    --both)
      WITH_RVIZ=true
      WITH_MUJOCO=true
      ;;
    --rviz-only)
      WITH_RVIZ=true
      WITH_MUJOCO=false
      ;;
    --mujoco-only)
      WITH_RVIZ=false
      WITH_MUJOCO=true
      ;;
    --topics-only)
      WITH_RVIZ=false
      WITH_MUJOCO=false
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      FORWARDED_ARGS+=("$@")
      break
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
  ("${WITH_RVIZ}" == true || "${WITH_MUJOCO}" == true) &&
  -z "${DISPLAY:-}" &&
  -z "${WAYLAND_DISPLAY:-}"
]]; then
  printf '%s\n' \
    '错误：未检测到 DISPLAY/WAYLAND_DISPLAY，无法启动 GUI。' \
    '可使用 --topics-only 验证无界面仿真链路。' >&2
  exit 1
fi

acquire_teleop_guard controller-only-simulation
install_teleop_cleanup_traps

"${SCRIPT_DIR}/doctor.sh"
activate_bundle_runtime
assert_no_conflicting_teleop_nodes

ROS2_BIN="${ROS_ROOT}/bin/ros2"
CONTROLLER_ONLY_LAUNCH="${PROJECT_PREFIX}/share/pico_body_tianji/launch/controller_only_sim.launch.py"
MUJOCO_VIEWER="${BUNDLE_ROOT}/src/pico_body_tianji/scripts/mujoco_joint_viewer.py"

for required in "${ROS2_BIN}" "${CONTROLLER_ONLY_LAUNCH}" "${MUJOCO_VIEWER}"; do
  if [[ ! -f "${required}" ]]; then
    printf '错误：纯手柄仿真运行文件不存在：%s\n' "${required}" >&2
    exit 1
  fi
done

if [[ "${WITH_MUJOCO}" == true ]]; then
  setsid python "${MUJOCO_VIEWER}" &
  MUJOCO_PID=$!
  register_teleop_process_group "${MUJOCO_PID}" mujoco-viewer 5
fi

printf '%s\n' \
  '启动纯运动学仿真：PICO 纯手柄 → 可配置 IK → JointState' \
  "  RViz=${WITH_RVIZ}  MuJoCo=${WITH_MUJOCO}" \
  '该任务不会连接 Marvin 控制器。'

with_rviz="false"
if [[ "${WITH_RVIZ}" == true ]]; then
  with_rviz="true"
fi

setsid python "${ROS2_BIN}" launch pico_body_tianji controller_only_sim.launch.py \
  "with_rviz:=${with_rviz}" "${FORWARDED_ARGS[@]}" &
LAUNCH_PID=$!
register_teleop_process_group \
  "${LAUNCH_PID}" ros-controller-only-preview 5

if [[ "${WITH_MUJOCO}" == true ]]; then
  set +e
  wait -n "${LAUNCH_PID}" "${MUJOCO_PID}"
  task_exit=$?
  set -e
else
  set +e
  wait "${LAUNCH_PID}"
  task_exit=$?
  set -e
fi

exit "${task_exit}"
