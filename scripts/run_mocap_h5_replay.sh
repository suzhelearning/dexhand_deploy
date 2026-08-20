#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# 直接 bash 运行时自动进入 pixi default 环境（h5py / zenoh）。
if ! python -c 'import h5py, zenoh' 2>/dev/null; then
  if command -v pixi >/dev/null 2>&1; then
    exec pixi run bash "${BASH_SOURCE[0]}" "$@"
  fi
fi

# 任意 mocap-acquisition v4.0 HDF5：以实时 Motive right_arm Home 为
# IK 增量起点，移动到 hands/right Manus 手腕的绝对动捕位姿，再经
# controller-only 相对末端映射送入天机右臂 IK。节点前台读取 s/r/q；
# X11 物理 Enter 作为回放的持续保压门控。
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

WITH_MUJOCO=true
VALIDATE_ONLY=false
H5_PATH=""
SPEED=1.0
YAW_DEG=0.0
RIGHT_RIGID_ID=right_arm
CONNECT_ENDPOINT=""
MUJOCO_PID=""
SIM_PID=""

usage() {
  cat <<'EOF'
用法：
  pixi run sim_mocap_h5 -- TAKE.h5 [选项]
  bash scripts/run_mocap_h5_replay.sh TAKE.h5 [选项]

轨迹文件：
  TAKE.h5                 任意 mocap-acquisition v4.0 HDF5；读取
                          hands/right/wrist_position 与
                          hands/right/wrist_quaternion_xyzw

模式：
  --mujoco-only           启动 IK、MuJoCo 和 H5 回放（默认）
  --topics-only           只启动 IK 与 H5 回放
  --speed N               按住 Enter 时的源轨迹倍速（默认 1.0）
  --yaw-deg N             轨迹绕 Motive +Y 的朝向修正（默认 0）
  --right-rigid-id ID     天机右末端刚体 id/名称（默认 right_arm）
  --connect-endpoint EP   可选 Zenoh Router 端点（默认 scouting）
  --validate-only         只检查并汇总 H5，不启动 IK、不运动
  -h, --help

键盘流程：
  s                 在 Home 捕获当前 right_arm 作为 IK 增量起点
  按住 Enter        从实测 Home 移动到 H5 绝对第 0 帧
  r                 到达 0 帧并出现提示后装载正式回放
  按住 Enter        推进正式轨迹；松开保持，再按继续
  s                 任意活动阶段立即取消并回 Home
  q                 回 Home 后退出（已经在 Home 时直接退出）

H5 路径不是固定值；每次运行可选择不同 TAKE.h5。运行前必须启动
Motive natnet-zenoh 发布器并确保 right_arm 有效。只控制右臂，左臂
保持 Home；本脚本本身不连接 Marvin 实体机械臂。
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
URDF_PATH="${PROJECT_PREFIX}/share/pico_body_tianji/assets/marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf"
MUJOCO_VIEWER="${BUNDLE_ROOT}/src/pico_body_tianji/scripts/mujoco_joint_viewer.py"

for required in \
  "${SIM_NODE}" \
  "${H5_NODE}" \
  "${PARAMETERS}" \
  "${URDF_PATH}" \
  "${MUJOCO_VIEWER}"
do
  if [[ ! -f "${required}" ]]; then
    printf '错误：H5 回放运行文件不存在：%s；请重新 build-ik/deploy-ik。\n' \
      "${required}" >&2
    exit 1
  fi
done

if [[ "${WITH_MUJOCO}" == true ]]; then
  setsid python "${MUJOCO_VIEWER}" &
  MUJOCO_PID=$!
  register_teleop_process_group "${MUJOCO_PID}" mujoco-viewer 5
fi

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
  '启动可选 H5 Manus 右手腕轨迹回放：right wrist -> 右臂目标 -> IK' \
  "  H5=${H5_PATH}" \
  "  speed=${SPEED}  yaw_deg=${YAW_DEG}" \
  "  Router=${CONNECT_ENDPOINT:-<scouting>}  MuJoCo=${WITH_MUJOCO}" \
  '  s -> Enter 保压到 0 帧 -> 松开 Enter -> r -> Enter 保压回放。' \
  '  任意活动阶段 s 回 Home；q 回 Home 后退出；左臂保持 Home。' \
  '该任务不会连接 Marvin 控制器。'

# 节点前台运行，stdin 直连 raw 键盘；Enter 按下/松开由 X11 查询。
"${H5_NODE}" "${node_arguments[@]}"
NODE_EXIT=$?
exit "${NODE_EXIT}"
