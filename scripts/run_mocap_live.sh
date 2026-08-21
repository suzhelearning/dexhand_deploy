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

# Motive 刚体定零 + 键盘步进/正面圆轨迹仿真（Zenoh，无 ROS）。
#
# 's' 冻结 right_arm 当前位姿为零点；方向键/1/0 手动累计。零位
# 按 'c' 只装载圆轨迹；物理 Enter 按住期间才推进，松开即暂停，
# 再按从暂停点继续。轨迹上移 200mm 后画 r=100mm 正面圆。's' 回
# Home，'q' 安全退出。节点前台运行，并通过 X11 检测 Enter 松开。
#
# 前置：Windows Motive + natnet-zenoh publisher 已发布
# mocap/hands/frame；本机 zenohd Router（tcp/0.0.0.0:7447）常驻
# （acquisition 项目 start_zenohd.sh / systemd 服务）。
#
# 用法：
#   pixi run sim_mocap_live
#   pixi run sim_mocap_live -- --topics-only
#   pixi run sim_mocap_live -- --right-rigid-id right_arm --step-mm 10
#   pixi run sim_mocap_live -- --circle-speed-mm-s 30

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

WITH_MUJOCO=true
LEFT_RIGID_ID="left_wrist"
RIGHT_RIGID_ID="right_arm"
CONNECT_ENDPOINT=""
SIDE=right
STEP_MM=10.0
CIRCLE_SPEED_MM_S=""
MUJOCO_PID=""
SIM_PID=""

usage() {
  cat <<'EOF'
用法：
  pixi run sim_mocap_live [-- 模式]
  bash scripts/run_mocap_live.sh [模式]

模式：
  --mujoco-only         同时启动 IK、MuJoCo 预览与动捕实时驱动（默认）
  --topics-only         只启动 IK 与动捕实时驱动（无界面）
  --left-rigid-id SPEC   左臂 Motive 刚体：数字 id 或刚体名（默认 left_wrist）
  --right-rigid-id SPEC  右臂 Motive 刚体：数字 id 或刚体名（默认 right_arm）
  --connect-endpoint EP zenohd Router 端点（默认空=本机 scouting；
                         仅当 scouting 不可达时才需显式连 router）
  --side SIDE            控制侧：right（仅右臂，默认）/ both（双臂同步）
  --step-mm MM           每次按键位置步长（默认 10mm）
  --circle-speed-mm-s N  正面圆轨迹峰值速度 mm/s（默认取配置 50）
  键盘 c / Enter         c 装载右臂正面圆；Enter 按住推进、松开暂停
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
    --left-rigid-id)
      if (($# < 2)); then
        printf '%s\n' '错误：--left-rigid-id 缺少数值。' >&2
        usage >&2
        exit 2
      fi
      LEFT_RIGID_ID="$2"
      shift
      ;;
    --right-rigid-id)
      if (($# < 2)); then
        printf '%s\n' '错误：--right-rigid-id 缺少数值。' >&2
        usage >&2
        exit 2
      fi
      RIGHT_RIGID_ID="$2"
      shift
      ;;
    --connect-endpoint)
      if (($# < 2)); then
        printf '%s\n' '错误：--connect-endpoint 缺少值。' >&2
        usage >&2
        exit 2
      fi
      CONNECT_ENDPOINT="$2"
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
    --step-mm)
      if (($# < 2)); then
        printf '%s\n' '错误：--step-mm 缺少数值。' >&2
        usage >&2
        exit 2
      fi
      STEP_MM="$2"
      shift
      ;;
    --circle-speed-mm-s)
      if (($# < 2)); then
        printf '%s\n' '错误：--circle-speed-mm-s 缺少数值。' >&2
        usage >&2
        exit 2
      fi
      CIRCLE_SPEED_MM_S="$2"
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
LIVE_NODE="${PROJECT_PREFIX}/lib/pico_body_tianji/mocap_live"
PARAMETERS="${PROJECT_PREFIX}/share/pico_body_tianji/config/mode/controller_only/controller_only_ik.yaml"
URDF_PATH="${PROJECT_PREFIX}/share/pico_body_tianji/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf"
MUJOCO_VIEWER="${BUNDLE_ROOT}/src/pico_body_tianji/scripts/mujoco_joint_viewer.py"

for required in \
  "${SIM_NODE}" \
  "${LIVE_NODE}" \
  "${PARAMETERS}" \
  "${URDF_PATH}" \
  "${MUJOCO_VIEWER}"
do
  if [[ ! -f "${required}" ]]; then
    printf '错误：mocap 动捕实时驱动运行文件不存在：%s\n' "${required}" >&2
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
  '启动 Motive 定零 + 键盘步进/Enter 保压正面圆轨迹：虚拟目标 → IK' \
  "  刚体=${LEFT_RIGID_ID}/${RIGHT_RIGID_ID}  side=${SIDE}  step_mm=${STEP_MM}" \
  "  圆峰值速度=${CIRCLE_SPEED_MM_S:-<配置默认>}mm/s" \
  "  Router=${CONNECT_ENDPOINT:-<scouting>}  MuJoCo=${WITH_MUJOCO}" \
  '  s 定零；方向键/1/0 手动步进；零位按 c 装载 r=100mm 正面圆；' \
  '  持续按住 Enter 才推进，松开即暂停，再按继续；s 回 Home；q 安全退出。' \
  '该任务不会连接 Marvin 控制器。'

# 动捕实时节点前台运行：stdin 直连终端（raw 模式读键）。
live_arguments=(
  --config "${PARAMETERS}"
  --param "rate:=60"
  --left-rigid-id "${LEFT_RIGID_ID}"
  --right-rigid-id "${RIGHT_RIGID_ID}"
  --side "${SIDE}"
  --step-mm "${STEP_MM}"
  --connect-endpoint "${CONNECT_ENDPOINT}"
)
if [[ -n "${CIRCLE_SPEED_MM_S}" ]]; then
  live_arguments+=(
    --param "circle_maximum_speed_mm_s:=${CIRCLE_SPEED_MM_S}"
  )
fi
"${LIVE_NODE}" "${live_arguments[@]}"
LIVE_EXIT=$?

exit "${LIVE_EXIT}"
