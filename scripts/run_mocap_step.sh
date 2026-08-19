#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

WITH_RVIZ=true
WITH_MUJOCO=true
STEP_MM=10.0
MUJOCO_PID=""
LAUNCH_PID=""

usage() {
  cat <<'EOF'
用法：
  pixi run sim_mocap_step -- [模式] [--step-mm N]
  bash scripts/run_mocap_step.sh [模式] [--step-mm N]

模式：
  --both          同时启动 RViz 与 MuJoCo（默认）
  --rviz-only     只启动 RViz
  --mujoco-only   只启动 MuJoCo
  --topics-only   只启动步进、IK 和仿真关节话题
  -h, --help

步进参数：
  --step-mm N     每次按键位移毫米（默认 10）

按键（动捕/Motive 系，y-up；raw 模式方向键）：
  上 ↑ = +z    下 ↓ = -z    左 ← = +x    右 → = -x
  '1' = +y     '0' = -y
  's' 开始步进（armed 时）/ 结束并回 Home（步进中）

说明：
  键盘在动捕坐标系里给机器人末端目标增量（双臂同步），经与在线
  PICO 相同的映射链路（1:1 目标整形）送入可配置 IK；只启动纯运动学
  仿真，不加载 Marvin SDK，不连接实体机械臂。可作为真机桥主机输入
  做确定性位移验收（见 docs/mocap_real_acceptance.md）。
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
    --step-mm)
      shift
      if (($# == 0)); then
        printf '%s\n' '错误：--step-mm 缺少数值。' >&2
        exit 2
      fi
      STEP_MM="$1"
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
STEP_LAUNCH="${PROJECT_PREFIX}/share/pico_body_tianji/launch/mocap_step.launch.py"
MUJOCO_VIEWER="${BUNDLE_ROOT}/src/pico_body_tianji/scripts/mujoco_joint_viewer.py"

for required in "${ROS2_BIN}" "${STEP_LAUNCH}" "${MUJOCO_VIEWER}"; do
  if [[ ! -f "${required}" ]]; then
    printf '错误：mocap 步进运行文件不存在：%s\n' "${required}" >&2
    exit 1
  fi
done

if [[ "${WITH_MUJOCO}" == true ]]; then
  setsid python "${MUJOCO_VIEWER}" &
  MUJOCO_PID=$!
  register_teleop_process_group "${MUJOCO_PID}" mujoco-viewer 5
fi

printf '%s\n' \
  '启动 mocap 键盘步进：按键 → 动捕系目标增量 → 可配置 IK' \
  "  step_mm=${STEP_MM}  RViz=${WITH_RVIZ}  MuJoCo=${WITH_MUJOCO}" \
  '该任务不会连接 Marvin 控制器。'

with_rviz="false"
if [[ "${WITH_RVIZ}" == true ]]; then
  with_rviz="true"
fi

# 键盘转发：脚本作为终端前台进程读取按键（含方向键转义序列，逐字节），
# 经 FIFO 转发到步进节点 stdin（launch 以 setsid 启动，子进程无法
# 直接读终端）；exec 9 常开写端保证读端永不 EOF。
KEY_FIFO=""
if [[ -t 0 ]]; then
  KEY_FIFO="$(mktemp -u /tmp/mocap-key-XXXXXX)"
  mkfifo "${KEY_FIFO}"
  printf '%s\n' \
    '键盘控制：上/下/左/右 = 动捕 ±z/∓z/±x/∓x，1/0 = ±y，' \
    "每次 ${STEP_MM} mm；s 开始，步进中再按 s 结束回 Home。" \
    "（经 ${KEY_FIFO} 转发按键）"
fi

if [[ -n "${KEY_FIFO}" ]]; then
  setsid python "${ROS2_BIN}" launch \
    pico_body_tianji mocap_step.launch.py \
    "step_mm:=${STEP_MM}" \
    "with_rviz:=${with_rviz}" \
    < "${KEY_FIFO}" &
  LAUNCH_PID=$!
  exec 9>"${KEY_FIFO}"
else
  setsid python "${ROS2_BIN}" launch \
    pico_body_tianji mocap_step.launch.py \
    "step_mm:=${STEP_MM}" \
    "with_rviz:=${with_rviz}" &
  LAUNCH_PID=$!
fi
register_teleop_process_group \
  "${LAUNCH_PID}" ros-mocap-step 5

forward_keypresses() {
  while kill -0 "${LAUNCH_PID}" 2>/dev/null ||
        { [[ -n "${MUJOCO_PID}" ]] && kill -0 "${MUJOCO_PID}" 2>/dev/null; }
  do
    # 逐字节转发（方向键为 \x1b[A 等 3 字节序列，由节点端解析）。
    if read -rsn1 -t 0.05 key; then
      printf '%s' "${key}" >&9 2>/dev/null || true
    fi
  done
}

if [[ -n "${KEY_FIFO}" ]]; then
  set +e
  forward_keypresses
  task_exit=0
  if [[ "${WITH_MUJOCO}" == true ]]; then
    wait "${LAUNCH_PID}" 2>/dev/null
    task_exit=$?
    wait "${MUJOCO_PID}" 2>/dev/null || true
  else
    wait "${LAUNCH_PID}" 2>/dev/null
    task_exit=$?
  fi
  set -e
  rm -f -- "${KEY_FIFO}"
else
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
fi

exit "${task_exit}"
