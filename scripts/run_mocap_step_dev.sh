#!/usr/bin/env bash
set -euo pipefail

# 源码直跑版键盘步进仿真：不需要 build-ik/deploy-ik，也不需要
# ros2 launch（launch 子进程的 stdin 不直连终端，raw 键盘不可用）。
#
# 结构：MuJoCo/IK 后台（setsid + 受管进程组），步进节点**前台**跑，
# stdin 直连终端（termios raw 模式读键，与 pty 实测一致）。
# 仅支持 --topics-only / --mujoco-only；RViz 模式请用正式入口
# pixi run sim_mocap_step（需先 build-ik + deploy-ik）。

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

# 源码直跑不经 pixi run：把 pixi default 环境补进 PATH（python 等）。
PIXI_DEFAULT_BIN="${BUNDLE_ROOT}/.pixi/envs/default/bin"
if [[ -d "${PIXI_DEFAULT_BIN}" ]]; then
  export PATH="${PIXI_DEFAULT_BIN}:${PATH}"
fi

WITH_MUJOCO=true
STEP_MM=10.0
IK_PID=""
MUJOCO_PID=""

usage() {
  cat <<'EOF'
用法（源码直跑，无需构建部署）：
  bash scripts/run_mocap_step_dev.sh [--topics-only] [--mujoco-only]
                                     [--step-mm N]

默认 --mujoco-only。按 s 开始步进，上/下/左/右/1/0 移动（动捕系，
每次 STEP_MM mm），再按 s 结束回 Home。RViz 模式请用正式入口。
EOF
}

while (($#)); do
  case "$1" in
    --mujoco-only) WITH_MUJOCO=true ;;
    --topics-only) WITH_MUJOCO=false ;;
    --step-mm)
      shift
      if (($# == 0)); then printf '%s\n' '错误：--step-mm 缺少数值。' >&2; exit 2; fi
      STEP_MM="$1"
      ;;
    -h|--help) usage; exit 0 ;;
    *) printf '错误：未知参数 %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

if [[ "${WITH_MUJOCO}" == true && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  printf '%s\n' '错误：无图形环境，请使用 --topics-only。' >&2
  exit 1
fi

# 与正式入口同一运行锁：真机桥可识别该主机身份（mocap-replay），
# 且不能同时跑两套主机。
acquire_teleop_guard mocap-replay
install_teleop_cleanup_traps

# 源码直跑快捷方式不跑 doctor.sh：本机（2026-08-19）vendor 已被更新
# 为较新版本，与仓库 VENDOR_SHA256SUMS（7-31 release 清单）不匹配，
# doctor 的完整性校验会失败；activate_bundle_runtime 已检查关键
# ROS 运行时。正式入口（pixi run sim_mocap_step 等）仍走 doctor。
activate_bundle_runtime

SRC_YAML="${BUNDLE_ROOT}/src/pico_body_tianji/config/mode/controller_only/controller_only_ik.yaml"
IK_BIN="${PROJECT_PREFIX}/lib/pico_body_tianji/tianji_kinematic_sim"
MUJOCO_VIEWER="${BUNDLE_ROOT}/src/pico_body_tianji/scripts/mujoco_joint_viewer.py"
for required in "${SRC_YAML}" "${IK_BIN}" "${MUJOCO_VIEWER}"; do
  if [[ ! -f "${required}" ]]; then
    printf '错误：运行文件不存在：%s\n' "${required}" >&2
    exit 1
  fi
done

if [[ "${WITH_MUJOCO}" == true ]]; then
  setsid python "${MUJOCO_VIEWER}" &
  MUJOCO_PID=$!
  register_teleop_process_group "${MUJOCO_PID}" mujoco-viewer 5
fi

setsid "${IK_BIN}" --ros-args --params-file "${SRC_YAML}" &
IK_PID=$!
register_teleop_process_group "${IK_PID}" ik-sim 5
sleep 2

printf '%s\n' \
  '启动源码直跑键盘步进仿真（无需 build-ik/deploy-ik）' \
  "  step_mm=${STEP_MM}  MuJoCo=${WITH_MUJOCO}" \
  '按 s 开始；上/下/左/右/1/0 = 动捕 ±z/∓z/±x/∓x/±y；再按 s 结束。'

# 节点前台运行：stdin 直连终端（raw 模式读键）。
# 注意：不能经 ros2 launch——其子进程 stdin 不直连终端。
python -m pico_body_tianji.controller_only.mocap_keyboard_step_node \
  --step-mm "${STEP_MM}" --ros-args --params-file "${SRC_YAML}"
NODE_EXIT=$?

exit "${NODE_EXIT}"
