#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# 直接 bash 运行时自动进入 pixi default 环境（h5py / zenoh）。
if ! python -c 'import h5py, zenoh' 2>/dev/null; then
  if command -v pixi >/dev/null 2>&1; then
    exec pixi run bash "${BASH_SOURCE[0]}" "$@"
  fi
fi

# 任意 mocap-acquisition v4.0 HDF5：以实时 Motive tianji_wrist Home 为
# IK 增量起点，移动到 hands/right Manus 手腕的绝对动捕位姿，再经
# controller-only 相对末端映射送入天机右臂 IK。节点前台读取 s/r/q；
# X11 物理 Enter 作为回放的持续保压门控。
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

WITH_MUJOCO=true
WITH_WUJI2=false
VALIDATE_ONLY=false
H5_PATH=""
SPEED=1.0
YAW_DEG=0.0
RIGHT_RIGID_ID=tianji_wrist
CONNECT_ENDPOINT=""
SHOW_FRAME_ZERO_SKELETON=false
WITH_HAND_RETARGET=false
WITH_HAND_COMMAND_VIEW=false
MUJOCO_PID=""
SIM_PID=""
HAND_BRIDGE_PID=""

usage() {
  cat <<'EOF'
用法：
  pixi run sim_mocap_h5 -- TAKE.h5 [选项]
  bash scripts/run_mocap_h5_replay.sh TAKE.h5 [选项]

轨迹文件：
  TAKE.h5                 任意 mocap-acquisition v4.0 HDF5；读取
                          hands/right/keypoints_world、wrist_position 与
                          wrist_quaternion_xyzw

模式：
  --mujoco-only           启动 IK、MuJoCo 和 H5 回放（默认）
  --topics-only           只启动 IK 与 H5 回放
  --wuji2                MuJoCo 使用 tianji+wuji2 双手组合 URDF（tianji_wuji2.urdf）
  --speed N               按住 Enter 时的源轨迹倍速（默认 1.0）
  --yaw-deg N             轨迹绕 Motive 竖直轴(+Z)的朝向修正（默认 0）
  --right-rigid-id ID     天机右末端刚体 id/名称（默认 tianji_wrist）
  --connect-endpoint EP   可选 Zenoh Router 端点（默认 scouting）
  --frame0-skeleton      MuJoCo 显示 H5 frame0 的 21 点/20 骨段目标骨架；
                          自动启用 --wuji2，按 s 前预览、按 s 后冻结
  --complete-wuji2-replay 完整仿真回放：wuji2、frame0 骨架、
                          dry retarget 桥及同窗手指动画
  --complete-wuji2-real-preview
                          完整真机预览：wuji2、frame0 骨架、同窗手指动画；
                          不启动 dry 桥，命令来自外部 wuji_hand2_real
  --validate-only         只检查并汇总 H5，不启动 IK、不运动
  -h, --help

键盘流程：
  s                 读取刚体并推导 r_mount 与 r_wrist Home
  按住 Enter        将机器人 r_wrist 移动到 H5 绝对 frame0；松开保持
  r                 稳定到达 frame0 后装载后续轨迹
  按住 Enter        从 frame0 推进后续 wrist 轨迹；松开保持
  s                 任意活动阶段立即取消并回 Home
  q                 回 Home 后退出（已经在 Home 时直接退出）

H5 路径不是固定值；每次运行可选择不同 TAKE.h5。运行前必须启动
Motive natnet-zenoh 并确保 tianji_wrist marker 有效。marker 用于定位
wuji2 r_mount；H5 Manus wrist 直接对齐厂商 URDF 的 r_wrist。
只控制右臂，左臂保持 Home；本脚本不连接 Marvin 实体机械臂。
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
    --wuji2)
      WITH_WUJI2=true
      ;;
    --speed)
      if (($# < 2)); then
        printf '%s\n' '错误：--speed 缺少数值。' >&2
        exit 2
      fi
      SPEED="$2"
      shift
      ;;
    --yaw-deg)
      if (($# < 2)); then
        printf '%s\n' '错误：--yaw-deg 缺少数值。' >&2
        exit 2
      fi
      YAW_DEG="$2"
      shift
      ;;
    --right-rigid-id)
      if (($# < 2)); then
        printf '%s\n' '错误：--right-rigid-id 缺少数值。' >&2
        exit 2
      fi
      RIGHT_RIGID_ID="$2"
      shift
      ;;
    --connect-endpoint)
      if (($# < 2)); then
        printf '%s\n' '错误：--connect-endpoint 缺少值。' >&2
        exit 2
      fi
      CONNECT_ENDPOINT="$2"
      shift
      ;;
    --frame0-skeleton)
      SHOW_FRAME_ZERO_SKELETON=true
      WITH_WUJI2=true
      ;;
    --complete-wuji2-replay)
      WITH_MUJOCO=true
      WITH_WUJI2=true
      SHOW_FRAME_ZERO_SKELETON=true
      WITH_HAND_RETARGET=true
      WITH_HAND_COMMAND_VIEW=true
      ;;
    --complete-wuji2-real-preview)
      WITH_MUJOCO=true
      WITH_WUJI2=true
      SHOW_FRAME_ZERO_SKELETON=true
      WITH_HAND_RETARGET=false
      WITH_HAND_COMMAND_VIEW=true
      ;;
    --validate-only)
      VALIDATE_ONLY=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --*)
      printf '错误：未知参数 %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
    *)
      if [[ -n "${H5_PATH}" ]]; then
        printf '错误：只能提供一个 H5，重复值：%s\n' "$1" >&2
        exit 2
      fi
      H5_PATH="$1"
      ;;
  esac
  shift
done
if [[
  "${SHOW_FRAME_ZERO_SKELETON}" == true &&
  "${WITH_MUJOCO}" != true
]]; then
  printf '%s\n' '错误：--frame0-skeleton 需要 MuJoCo，不能与 --topics-only 同用。' >&2
  exit 2
fi


if [[ -z "${H5_PATH}" ]]; then
  printf '%s\n' '错误：必须提供要回放的 TAKE.h5 路径。' >&2
  usage >&2
  exit 2
fi
if [[ ! -f "${H5_PATH}" ]]; then
  printf '错误：H5 文件不存在：%s\n' "${H5_PATH}" >&2
  exit 2
fi

activate_bundle_runtime

node_arguments=(
  "${H5_PATH}"
  --config "${PROJECT_PREFIX}/share/pico_body_tianji/config/mode/controller_only/controller_only_ik.yaml"
  --speed "${SPEED}"
  --yaw-deg "${YAW_DEG}"
  --right-rigid-id "${RIGHT_RIGID_ID}"
  --rate 60
  --connect-endpoint "${CONNECT_ENDPOINT}"
)

if [[ "${WITH_HAND_COMMAND_VIEW}" == true && "${WITH_MUJOCO}" != true ]]; then
  printf '%s\n' '错误：完整 wuji2 回放模式需要 MuJoCo，不能与 --topics-only 组合。' >&2
  exit 2
fi

if [[ "${VALIDATE_ONLY}" == true ]]; then
  exec python -m \
    pico_body_tianji.controller_only.mocap_h5_replay_node \
    "${node_arguments[@]}" --validate-only
fi

if [[ -z "${DISPLAY:-}" ]]; then
  printf '%s\n' \
    '错误：未设置 DISPLAY，无法可靠读取 Enter 按下/松开状态。' \
    '请在桌面终端运行；只检查文件可使用 --validate-only。' >&2
  exit 1
fi
if [[ "${WITH_MUJOCO}" == true && -z "${WAYLAND_DISPLAY:-}" && -z "${DISPLAY:-}" ]]; then
  printf '%s\n' '错误：无显示环境，无法启动 MuJoCo。' >&2
  exit 1
fi

acquire_teleop_guard mocap-replay
install_teleop_cleanup_traps

"${SCRIPT_DIR}/doctor.sh"
assert_no_conflicting_teleop_nodes

SIM_NODE="${PROJECT_PREFIX}/lib/pico_body_tianji/tianji_kinematic_sim"
H5_NODE="${PROJECT_PREFIX}/lib/pico_body_tianji/mocap_h5_replay"
PARAMETERS="${PROJECT_PREFIX}/share/pico_body_tianji/config/mode/controller_only/controller_only_ik.yaml"
IK_URDF_PATH="${PROJECT_PREFIX}/share/pico_body_tianji/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf"
VIEWER_URDF_PATH="${PROJECT_PREFIX}/share/pico_body_tianji/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4_mujoco.urdf"
if [[ "${WITH_WUJI2}" == true ]]; then
  VIEWER_URDF_PATH="${PROJECT_PREFIX}/share/pico_body_tianji/assets/tianji_wuji2/tianji_wuji2.urdf"
fi
MUJOCO_VIEWER="${BUNDLE_ROOT}/src/pico_body_tianji/scripts/mujoco_joint_viewer.py"
HAND_BRIDGE="${PROJECT_PREFIX}/lib/pico_body_tianji/wuji_hand2_bridge"

for required in \
  "${SIM_NODE}" \
  "${H5_NODE}" \
  "${PARAMETERS}" \
  "${IK_URDF_PATH}" \
  "${VIEWER_URDF_PATH}" \
  "${MUJOCO_VIEWER}"
do
  if [[ ! -f "${required}" ]]; then
    printf '错误：H5 回放运行文件不存在：%s；请重新 build-ik/deploy-ik。\n' \
      "${required}" >&2
    exit 1
  fi
done

if [[ "${WITH_HAND_RETARGET}" == true ]]; then
  if [[ ! -x "${HAND_BRIDGE}" ]]; then
    printf '错误：完整回放缺少 wuji_hand2_bridge：%s；请重新 build/deploy。\n'       "${HAND_BRIDGE}" >&2
    exit 1
  fi
  setsid "${HAND_BRIDGE}" --dry-run --rate 100 --side right &
  HAND_BRIDGE_PID=$!
  register_teleop_process_group     "${HAND_BRIDGE_PID}" wuji-hand2-dry-retarget 10
fi

if [[ "${WITH_MUJOCO}" == true ]]; then
  viewer_arguments=(--urdf "${VIEWER_URDF_PATH}")
  if [[ "${SHOW_FRAME_ZERO_SKELETON}" == true ]]; then
    viewer_arguments+=(
      --frame0-skeleton-topic /pico_body_sim/frame0_hand_skeleton
    )
  fi
  if [[ "${WITH_HAND_COMMAND_VIEW}" == true ]]; then
    viewer_arguments+=(
      --hand-commands-topic /pico_body_sim/right_hand/joint_commands
    )
  fi
  setsid python "${MUJOCO_VIEWER}" "${viewer_arguments[@]}" &
  MUJOCO_PID=$!
  register_teleop_process_group "${MUJOCO_PID}" mujoco-viewer 5
fi

mapfile -t sim_arguments < <(
  yaml_params_for tianji_kinematic_sim "${PARAMETERS}" \
    "urdf_path:=${IK_URDF_PATH}"
)
for index in "${!sim_arguments[@]}"; do
  sim_arguments[index]="${sim_arguments[index]#--param }"
done
setsid "${SIM_NODE}" "${sim_arguments[@]}" &
SIM_PID=$!
register_teleop_process_group "${SIM_PID}" sim-ik-solver 5

printf '%s\n' \
  '启动可选 H5 Manus 右手腕轨迹回放：right wrist -> 右臂目标 -> IK' \
  "  H5=${H5_PATH}" \
  "  speed=${SPEED}  yaw_deg=${YAW_DEG}" \
  "  Router=${CONNECT_ENDPOINT:-<scouting>}  MuJoCo=${WITH_MUJOCO}" \
  "  Wuji2=${WITH_WUJI2}  Frame0Skeleton=${SHOW_FRAME_ZERO_SKELETON}  HandRetarget=${WITH_HAND_RETARGET}  HandCommandView=${WITH_HAND_COMMAND_VIEW}" \
  '  s 读取 marker -> Enter 保压接近绝对 frame0 -> r -> Enter 回放。' \
  '  任意活动阶段 s 回 Home；q 回 Home 后退出；左臂保持 Home。' \
  '该任务不会连接 Marvin 控制器。'

# 节点前台运行，stdin 直连 raw 键盘；Enter 按下/松开由 X11 查询。
"${H5_NODE}" "${node_arguments[@]}"
NODE_EXIT=$?
exit "${NODE_EXIT}"
