#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

WITH_RVIZ=true
WITH_MUJOCO=true
REPLAY_SPEED=1.0
YAW_DEG=0.0
REFERENCE_FRAME=-1
HOLD_ARM=0.0
H5_FILE=""
MUJOCO_PID=""
LAUNCH_PID=""

usage() {
  cat <<'EOF'
用法：
  pixi run sim_mocap -- H5.h5 [模式] [--speed N] [--yaw-deg N]
                      [--reference-frame N] [--hold-arm N]
  bash scripts/run_mocap_sim.sh H5.h5 [模式] [--speed N] [--yaw-deg N]
                                     [--reference-frame N] [--hold-arm N]

模式：
  --both          同时启动 RViz 与 MuJoCo（默认）
  --rviz-only     只启动 RViz
  --mujoco-only   只启动 MuJoCo
  --topics-only   只启动回放、IK 和仿真关节话题
  -h, --help

回放参数：
  --speed N             回放倍速（默认 1.0）
  --yaw-deg N           绕竖直轴旋转整条轨迹的朝向标定（度，默认 0）
  --reference-frame N   参考帧下标（等效按 A 时刻，默认第一个有效帧）
  --hold-arm N          回放前保持 idle 的秒数（默认 0；真机验收时
                        等真机桥进入 armed_idle）

说明：
  把 mocap-acquisition HDF5（v4.0）里录制的手腕位姿作为轨迹跟踪
  仿真输入，经与在线 PICO 相同的映射送入可配置 IK；只启动纯运动学
  仿真，不加载 Marvin SDK，不连接实体机械臂。配合 --hold-arm 时
  可作为真机桥（real_controller_only）的主机输入，用于确定性轨迹
  真机验收（见 docs/mocap_real_acceptance.md）；不得与其他输入
  身份同时运行。
EOF
}

if (($# < 1)) || [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

H5_FILE="$(realpath -m -- "$1")"
shift
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
    --speed)
      shift
      if (($# == 0)); then
        printf '%s\n' '错误：--speed 缺少数值。' >&2
        exit 2
      fi
      REPLAY_SPEED="$1"
      ;;
    --yaw-deg)
      shift
      if (($# == 0)); then
        printf '%s\n' '错误：--yaw-deg 缺少数值。' >&2
        exit 2
      fi
      YAW_DEG="$1"
      ;;
    --reference-frame)
      shift
      if (($# == 0)); then
        printf '%s\n' '错误：--reference-frame 缺少数值。' >&2
        exit 2
      fi
      REFERENCE_FRAME="$1"
      ;;
    --hold-arm)
      shift
      if (($# == 0)); then
        printf '%s\n' '错误：--hold-arm 缺少数值。' >&2
        exit 2
      fi
      HOLD_ARM="$1"
      ;;
    *)
      printf '错误：未知参数 %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [[ ! -f "${H5_FILE}" ]]; then
  printf '错误：mocap HDF5 文件不存在：%s\n' "${H5_FILE}" >&2
  exit 1
fi
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

acquire_teleop_guard mocap-replay
install_teleop_cleanup_traps

"${SCRIPT_DIR}/doctor.sh"
activate_bundle_runtime
assert_no_conflicting_teleop_nodes

ROS2_BIN="${ROS_ROOT}/bin/ros2"
MOCAP_LAUNCH="${PROJECT_PREFIX}/share/pico_body_tianji/launch/mocap_replay.launch.py"
MUJOCO_VIEWER="${BUNDLE_ROOT}/src/pico_body_tianji/scripts/mujoco_joint_viewer.py"

for required in "${ROS2_BIN}" "${MOCAP_LAUNCH}" "${MUJOCO_VIEWER}"; do
  if [[ ! -f "${required}" ]]; then
    printf '错误：mocap 回放运行文件不存在：%s\n' "${required}" >&2
    exit 1
  fi
done

if [[ "${WITH_MUJOCO}" == true ]]; then
  setsid python "${MUJOCO_VIEWER}" &
  MUJOCO_PID=$!
  register_teleop_process_group "${MUJOCO_PID}" mujoco-viewer 5
fi

printf '%s\n' \
  '启动 mocap 轨迹跟踪仿真：HDF5 手腕位姿 → 纯手柄映射 → 可配置 IK' \
  "  H5=${H5_FILE}  speed=${REPLAY_SPEED}  yaw_deg=${YAW_DEG}" \
  "  reference_frame=${REFERENCE_FRAME}  RViz=${WITH_RVIZ}  MuJoCo=${WITH_MUJOCO}" \
  '该任务不会连接 Marvin 控制器。'

with_rviz="false"
if [[ "${WITH_RVIZ}" == true ]]; then
  with_rviz="true"
fi

setsid python "${ROS2_BIN}" launch \
  pico_body_tianji mocap_replay.launch.py \
  "h5_file:=${H5_FILE}" \
  "replay_speed:=${REPLAY_SPEED}" \
  "yaw_deg:=${YAW_DEG}" \
  "reference_frame:=${REFERENCE_FRAME}" \
  "hold_arm:=${HOLD_ARM}" \
  "with_rviz:=${with_rviz}" &
LAUNCH_PID=$!
register_teleop_process_group \
  "${LAUNCH_PID}" ros-mocap-replay 5

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
