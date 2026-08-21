#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# 直接 bash 运行（不经 pixi）时系统 python 缺少 zenoh；自动经
# pixi run 重新执行本脚本（doctor / activate_bundle_runtime 均需
# zenoh 环境）。pixi run 下 PATH 已含 zenoh 环境，不会递归。
if ! python -c 'import zenoh' 2>/dev/null; then
  if command -v pixi >/dev/null 2>&1; then
    exec pixi run bash "${BASH_SOURCE[0]}" "$@"
  fi
fi

# mocap 键盘步进仿真（Zenoh 通讯版，无 ROS）。
#
# 结构：IK 与 MuJoCo 后台（setsid + 受管进程组），步进节点**前台**
# 运行——stdin 直连终端（termios raw 模式读键，与 pty 实测一致），
# 不需要 launch/FIFO 转发。目标经 Zenoh 发布
# （/pico_body/{left,right}_arm_target_pose），动捕系 10mm/键
# （--step-mm 可调），s 启停。
#
# 用法：
#   pixi run sim_mocap_step            # MuJoCo 预览 + 键盘步进
#   pixi run sim_mocap_step -- --topics-only   # 无界面
#   bash scripts/run_mocap_step.sh --step-mm 5 --side both

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

WITH_MUJOCO=true
STEP_MM=10.0
SIDE=right
MUJOCO_PID=""
SIM_PID=""
STEP_NODE_PID=""

usage() {
  cat <<'EOF'
用法：
  pixi run sim_mocap_step [-- 模式]
  bash scripts/run_mocap_step.sh [模式]

模式：
  --mujoco-only  同时启动 IK、MuJoCo 预览与键盘步进（默认）
  --topics-only  只启动 IK 与键盘步进（无界面）
  --step-mm N    每次按键位移毫米（默认 10）
  --side SIDE    控制侧：right（仅右臂，默认）/ both（双臂同步）
  -h, --help

本脚本只启动纯运动学仿真，不加载 Marvin SDK，不连接实体机械臂。
EOF
}

while (($#)); do
  case "$1" in
    --mujoco-only)
      WITH_MUJOCO=true
      ;;
    --topics-only)
      WITH_MUJOCO=false
      ;;
    --step-mm)
      if (($# < 2)); then
        printf '%s\n' '错误：--step-mm 缺少数值。' >&2
        usage >&2
        exit 2
      fi
      STEP_MM="$2"
      shift
      ;;
    --side)
      if (($# < 2)); then
        printf '%s\n' '错误：--side 缺少数值。' >&2
        usage >&2
        exit 2
      fi
      case "$2" in
        right|both) SIDE="$2" ;;
        *)
          printf '错误：--side 必须是 right 或 both，实际 %s\n' "$2" >&2
          usage >&2
          exit 2
          ;;
      esac
      shift
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
    '可使用 --topics-only 验证无界面仿真链路。' >&2
  exit 1
fi

acquire_teleop_guard mocap-replay
install_teleop_cleanup_traps

"${SCRIPT_DIR}/doctor.sh"
activate_bundle_runtime
assert_no_conflicting_teleop_nodes

SIM_NODE="${PROJECT_PREFIX}/lib/pico_body_tianji/tianji_kinematic_sim"
STEP_NODE="${PROJECT_PREFIX}/lib/pico_body_tianji/mocap_keyboard_step"
PARAMETERS="${PROJECT_PREFIX}/share/pico_body_tianji/config/mode/controller_only/controller_only_ik.yaml"
URDF_PATH="${PROJECT_PREFIX}/share/pico_body_tianji/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf"
MUJOCO_VIEWER="${BUNDLE_ROOT}/src/pico_body_tianji/scripts/mujoco_joint_viewer.py"

for required in \
  "${SIM_NODE}" \
  "${STEP_NODE}" \
  "${PARAMETERS}" \
  "${URDF_PATH}" \
  "${MUJOCO_VIEWER}"
do
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

# C++ 节点只接受裸 key:=value 参数；yaml_params_for 直接输出该格式。
mapfile -t sim_arguments < <(
  yaml_params_for tianji_kinematic_sim "${PARAMETERS}" \
    "urdf_path:=${URDF_PATH}"
)
for index in "${!sim_arguments[@]}"; do
  sim_arguments[index]="${sim_arguments[index]#--param }"
done
setsid "${SIM_NODE}" "${sim_arguments[@]}" &
SIM_PID=$!
register_teleop_process_group "${SIM_PID}" sim-ik-solver 5

printf '%s\n' \
  '启动 mocap 键盘步进（Zenoh）：按键 → 动捕系目标增量 → 可配置 IK' \
  "  step_mm=${STEP_MM}  side=${SIDE}  MuJoCo=${WITH_MUJOCO}" \
  '该任务不会连接 Marvin 控制器。'

# 步进节点前台运行：stdin 直连终端（raw 模式读键）。
"${STEP_NODE}" \
  --config "${PARAMETERS}" \
  --param "step_mm:=${STEP_MM}" \
  --param "side:=${SIDE}" \
  --param "rate:=60"
STEP_NODE_EXIT=$?

exit "${STEP_NODE_EXIT}"
